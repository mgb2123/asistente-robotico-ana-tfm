"""
gestor_emergencia — protocolo de llamada de emergencia (Twilio) para Ana.

Encapsula toda la lógica de emergencia que antes vivía dispersa en
`asistente_node`, para mantener el nodo principal legible. Corre EN PROCESO
dentro de `asistente_node` (necesita su TTS y su logger), pero como clase
aislada y testeable.

Al activarse (`activar(tipo)`):
  1. Cooldown de 60 s con mutex: ignora reactivaciones del mismo evento.
  2. Pausa el pipeline de conversación del nodo (dispatch + TTS normal).
     El STT y el LLM de coherencia siguen activos.
  3. Anuncia por voz que va a llamar.
  4. Exporta el contexto (perfil + historial) a /tmp/emergencia_contexto.json
     para que el bridge pueda usarlo en el LLM de emergencia.
  5. Llama por Twilio con hasta 3 intentos (1 + 2 reintentos, 2 s entre fallos).
  6. Reanuda el pipeline al finalizar.

Mensaje de la llamada (TwiML):
  - SIEMPRE incluye <Say> con la identificación de Ana + dirección + aviso médico.
  - Si TWILIO_BRIDGE_URL está configurada, añade <Connect><Stream track="both_tracks">
    para abrir el puente de audio bidireccional con bridge_twilio_emergencia.py.
    El bridge corre en la RPi (expuesto vía ngrok) y gestiona el protocolo de
    4 fases (ver bridge_twilio_emergencia.py):
      · arecord → μ-law → Twilio (operador oye al humano)
      · Twilio → μ-law → aplay (humano oye al operador)
      · Audio directo primero; si el residente deja de comunicarse (≥6 s sin
        habla coherente), Piper transmite al operador la info del residente y,
        si hay historial, un resumen del LLM; luego vuelve a audio directo.
  - Si no hay TWILIO_BRIDGE_URL → <Hangup/> tras el Say (fallback robusto).

Las credenciales Twilio y la URL del puente se inyectan SOLO por variables de
entorno; nunca se guardan en el código.
"""

import json
import os
import threading
import time
from xml.sax.saxutils import escape, quoteattr

from std_msgs.msg import String

COOLDOWN_SEG = 60.0          # ventana en la que se ignoran reactivaciones
REINTENTOS = 3               # intentos totales de la llamada (1 + 2 reintentos)
ESPERA_REINTENTO_SEG = 2.0   # espera entre intentos fallidos

BRIDGE_COHERENT_PORT = 9999  # UDP: asistente_node notifica habla coherente al bridge

PERFIL_PATH = os.path.join(
    os.path.expanduser('~'), 'asistente_turtlebot4-main', 'perfil_residente.json')
CONTEXTO_EMERGENCIA_PATH = '/tmp/emergencia_contexto.json'
BRIDGE_URL_PATH = '/tmp/bridge_url.txt'  # escrito por bridge_twilio_emergencia.py


class GestorEmergencia:
    """Gestiona el protocolo de emergencia con Twilio (cooldown + reintentos)."""

    def __init__(self, node):
        """`node` es el AsistenteNode (usa su logger, TTS y métricas)."""
        self._node = node
        self._log = node.get_logger()
        self._pub_status = node.create_publisher(String, '/emergency/status', 10)

        self._lock = threading.Lock()
        self._activa = False
        self._ultimo_disparo = 0.0

    # ------------------------------------------------------------------
    def activar(self, tipo='accidente'):
        """Dispara el protocolo si no hay cooldown ni otra llamada en curso."""
        with self._lock:
            ahora = time.time()
            if self._activa:
                self._log.warn('Emergencia ya en curso, ignoro reactivación.')
                return
            restante = COOLDOWN_SEG - (ahora - self._ultimo_disparo)
            if restante > 0:
                self._log.warn(f'Emergencia en cooldown ({restante:.0f}s), ignoro.')
                self._status(f'COOLDOWN restante={restante:.0f}s')
                return
            self._activa = True
            self._ultimo_disparo = ahora

        threading.Thread(target=self._ejecutar, args=(tipo,), daemon=True).start()

    def notificar_habla(self):
        """El nodo asistente llama a este método cuando detecta habla coherente.

        Reenvía la señal 'COHERENT' al bridge por UDP para que resetee su timer
        de relevo (RESIDENTE_SILENCIO_SEG) en la Fase 1 de audio directo.
        """
        try:
            sock = getattr(self._node, '_bridge_udp_sock', None)
            if sock is not None:
                sock.sendto(b'COHERENT', ('127.0.0.1', BRIDGE_COHERENT_PORT))
        except Exception:
            pass

    # ------------------------------------------------------------------
    def _status(self, texto):
        msg = String()
        msg.data = texto
        self._pub_status.publish(msg)
        self._log.info(f'/emergency/status: {texto}')

    def _hablar(self, texto, tipo, intensidad):
        try:
            self._node._hablar(texto, tipo, intensidad, priority='drop_old')
        except Exception as e:
            self._log.error(f'No pude hablar en emergencia: {e}')

    def _cargar_perfil(self):
        try:
            with open(PERFIL_PATH, 'r', encoding='utf-8') as f:
                return json.load(f)
        except (OSError, ValueError) as e:
            self._log.warn(f'perfil_residente.json no disponible ({e}); '
                           'uso datos vacíos.')
            return {}

    def _exportar_contexto_emergencia(self, tipo):
        """Escribe el contexto de la emergencia en un fichero temporal.

        El bridge_twilio_emergencia.py lo lee para que el LLM de emergencia
        tenga toda la información (perfil del residente + historial).
        """
        perfil = self._cargar_perfil()
        historial = getattr(self._node, '_historial', [])[-6:]

        # Ruta del modelo Piper para que el bridge pueda sintetizar
        piper_model = getattr(self._node, 'PIPER_MODEL', '')
        if not piper_model:
            piper_model = os.path.join(
                os.path.expanduser('~'),
                'asistente_turtlebot4-main', 'models', 'piper',
                'es_MX-ald-medium.onnx')

        # Tarjeta ALSA de reproducción para que el bridge use el mismo altavoz
        alsa_dac = getattr(self._node, 'ALSA_CARD_DAC', 'Headset')

        ctx = {
            'tipo': tipo,
            'timestamp': time.time(),
            'perfil': perfil,
            'historial': historial,
            'piper_model': piper_model,
            'openrouter_api_key': os.environ.get('OPENROUTER_API_KEY', ''),
            'alsa_dac': alsa_dac,
        }
        try:
            with open(CONTEXTO_EMERGENCIA_PATH, 'w', encoding='utf-8') as f:
                json.dump(ctx, f, ensure_ascii=False, indent=2)
            self._log.info(f'Contexto emergencia exportado a {CONTEXTO_EMERGENCIA_PATH}')
        except OSError as e:
            self._log.error(f'No pude exportar contexto de emergencia: {e}')

    def _construir_twiml(self):
        """Construye el TwiML.

        Con bridge: <Say> mínimo de identificación + <Connect><Stream>. La
        información detallada del residente la transmite el bridge en la Fase B
        (Piper), así que aquí no se repite.
        Sin bridge (fallback): <Say> con el mensaje completo + <Pause>.
        """
        perfil = self._cargar_perfil()
        nombre = perfil.get('nombre', '')
        direccion = perfil.get('direccion', '')

        # Leer URL del bridge: primero del archivo que escribe bridge_twilio_emergencia.py,
        # luego del env var (compatibilidad manual), y si nada → fallback Say+Pause.
        bridge = ''
        try:
            with open(BRIDGE_URL_PATH, 'r') as f:
                bridge = f.read().strip()
            self._log.info(f'[EMG] URL bridge leída de {BRIDGE_URL_PATH}: {bridge}')
        except OSError as e:
            self._log.info(f'[EMG] {BRIDGE_URL_PATH} no disponible ({e}), probando env var')
        if not bridge:
            bridge = os.environ.get('TWILIO_BRIDGE_URL', '').strip()
            if bridge:
                self._log.info(f'[EMG] URL bridge desde env var: {bridge}')

        if bridge:
            self._log.info('[EMG] TwiML con <Connect><Stream> (audio bidireccional)')
            # NO usar track="both_tracks" aquí: con <Connect><Stream> el único valor
            # válido de track es "inbound_track" (el por defecto). Poner "both_tracks"
            # (que sólo vale para <Start><Stream>) hace que Twilio rechace el verbo
            # <Connect> con el error 31941 y NUNCA abra el WebSocket → la llamada cae
            # al <Pause> en silencio. <Connect><Stream> ya es bidireccional de por sí:
            # Twilio envía el audio del operador al WS y reproduce lo que el WS le manda.
            # <Say> mínimo: se reproduce MIENTRAS el WebSocket se abre (~0.3 s) y sirve
            # de fallback audible si el bridge WebSocket falla. El anuncio completo de alta
            # calidad lo transmite el bridge en la Fase 0 (daniela-high, length_scale=1.3).
            # <Pause> tras </Connect>: si el bridge se desconecta, Twilio cae aquí y
            # mantiene la llamada abierta hasta que el operador cuelgue.
            return (f'<Response>'
                    f'<Say voice="Polly.Conchita" language="es-ES">'
                    f'Emergencia médica.</Say>'
                    f'<Connect>'
                    f'<Stream url={quoteattr(bridge)}/>'
                    f'</Connect>'
                    f'<Pause length="3600"/>'
                    f'</Response>')

        # <Pause> mantiene la línea abierta para que el operador pueda hablar.
        # El polling detectará el 'completed' cuando el operador cuelgue.
        self._log.info('[EMG] TwiML con <Pause> (fallback sin bridge)')
        intro = 'Soy Ana, la asistente robótica'
        if nombre:
            intro += f' de {nombre}'
        intro += '. Puede haber una emergencia'
        if direccion:
            intro += f' en {direccion}'
        intro += '. Por favor, acuda lo antes posible.'
        return (f'<Response>'
                f'<Say voice="Polly.Conchita" language="es-ES">{escape(intro)}</Say>'
                f'<Pause length="3600"/>'
                f'</Response>')

    # ------------------------------------------------------------------
    _ESTADOS_FINALES_TWILIO = frozenset(
        {'completed', 'failed', 'busy', 'no-answer', 'canceled'})
    _POLL_INTERVAL_SEG = 3.0
    _POLL_TIMEOUT_SEG = 600.0   # 10 minutos máximo (era 1 hora)
    _POLL_LOG_INTERVAL = 10     # Mostrar estado cada N polls (~30 s)

    def _esperar_fin_llamada(self, client, call_sid):
        """Hace polling del estado de la llamada hasta que termine.

        Mantiene el modo emergencia activo (y el pipeline pausado) durante
        toda la duración de la llamada. Retorna cuando el operador cuelga o
        se supera el timeout de seguridad.
        """
        inicio = time.time()
        n_poll = 0
        while time.time() - inicio < self._POLL_TIMEOUT_SEG:
            time.sleep(self._POLL_INTERVAL_SEG)
            n_poll += 1
            try:
                estado = client.calls(call_sid).fetch().status
                transcurrido = time.time() - inicio
                # Log de debug en cada poll, info cada ~30 s para no saturar
                if n_poll % self._POLL_LOG_INTERVAL == 0:
                    self._log.info(
                        f'[EMG poll #{n_poll}] estado={estado} '
                        f'transcurrido={transcurrido:.0f}s')
                else:
                    self._log.debug(
                        f'[EMG poll #{n_poll}] estado={estado} '
                        f'transcurrido={transcurrido:.0f}s')
                if estado in self._ESTADOS_FINALES_TWILIO:
                    self._log.info(
                        f'Llamada emergencia finalizada: {estado} '
                        f'(duración {transcurrido:.0f}s, {n_poll} polls)')
                    return
            except Exception as e:
                self._log.warn(f'[EMG poll #{n_poll}] error al consultar estado: {e}')
        self._log.warn(
            f'Timeout de {self._POLL_TIMEOUT_SEG/60:.0f} min esperando fin '
            'de la llamada de emergencia. Reanudando pipeline.')

    # ------------------------------------------------------------------
    def _ejecutar(self, tipo):
        try:
            self._log.info(f'[EMG] _ejecutar iniciado tipo={tipo}')
            self._status(f'EMERGENCY_TRIGGERED tipo={tipo}')

            # 1. Pausar el pipeline de conversación normal.
            #    El STT y el LLM de coherencia siguen activos en el nodo.
            self._node.pausar_para_emergencia()
            self._log.info('[EMG] pipeline de conversación pausado')

            # 2. Exportar contexto para el bridge.
            self._exportar_contexto_emergencia(tipo)
            self._log.info('[EMG] contexto exportado')

            # 3. Anunciar (TTS de emergencia, bypasa el bloqueo normal).
            self._hablar('Llamando a emergencias.', 'urgencia', 0.9)
            self._log.info('[EMG] TTS "Llamando a emergencias" encolado')

            # 4. Verificar credenciales.
            sid = os.environ.get('TWILIO_SID', '')
            token = os.environ.get('TWILIO_TOKEN', '')
            origen = os.environ.get('TWILIO_FROM', '')
            destino = os.environ.get('TWILIO_TO', '')
            self._log.info(
                f'[EMG] credenciales: SID={bool(sid)} TOKEN={bool(token)} '
                f'FROM={bool(origen)} TO={bool(destino)}')
            if not all([sid, token, origen, destino]):
                self._log.error('Twilio creds no configurados '
                                '(env vars TWILIO_SID/TOKEN/FROM/TO).')
                self._status('FAILED motivo=sin_credenciales')
                self._node._logger_sesion.evento(
                    'error', donde='twilio', msg='sin credenciales')
                self._hablar('No tengo configurada la llamada de emergencia.',
                             'preocupacion', 0.7)
                return

            twiml = self._construir_twiml()
            self._log.info(f'[EMG] TwiML construido: {twiml[:120]}...')
            try:
                from twilio.rest import Client
                client = Client(sid, token)
                self._log.info('[EMG] cliente Twilio creado')
            except Exception as e:
                self._log.error(f'No pude crear el cliente Twilio: {e}')
                self._status('FAILED motivo=cliente')
                self._node._logger_sesion.evento('error', donde='twilio', msg=str(e))
                self._hablar('No he podido realizar la llamada de emergencia.',
                             'preocupacion', 0.7)
                return

            # 5. Realizar la llamada con reintentos.
            call = None
            ultimo_error = None
            for intento in range(1, REINTENTOS + 1):
                self._log.info(f'[EMG] intentando llamada Twilio (intento {intento}/{REINTENTOS})')
                try:
                    call = client.calls.create(
                        to=destino, from_=origen, twiml=twiml)
                    self._log.info(
                        f'Llamada emergencia OK (intento {intento}): {call.sid}')
                    self._status(f'TWILIO_CALL_SID={call.sid}')
                    self._node._logger_sesion.evento('emergencia', sid=str(call.sid))
                    break  # llamada iniciada; salir del loop de reintentos
                except Exception as e:
                    ultimo_error = e
                    self._log.error(f'Twilio intento {intento} falló: {e}')
                    if intento < REINTENTOS:
                        time.sleep(ESPERA_REINTENTO_SEG)

            if call is None:
                self._status('FAILED motivo=reintentos_agotados')
                self._node._logger_sesion.evento(
                    'error', donde='twilio', msg=str(ultimo_error))
                self._hablar('No he podido realizar la llamada de emergencia.',
                             'preocupacion', 0.7)
                return

            # 6. Esperar a que el operador cuelgue.
            #    La emergencia permanece activa (modo pausado) durante toda la llamada.
            self._esperar_fin_llamada(client, call.sid)
        finally:
            with self._lock:
                self._activa = False
            # Reanudar el pipeline normal (se ejecuta siempre: éxito, fallo o excepción).
            self._node.reanudar_tras_emergencia()
