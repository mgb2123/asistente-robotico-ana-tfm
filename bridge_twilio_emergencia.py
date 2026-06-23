"""
bridge_twilio_emergencia.py — Puente de audio bidireccional Twilio ↔ RPi.

Cómo funciona:
  · Servidor WebSocket asyncio que implementa Twilio Media Streams.
  · La RPi lo lanza manualmente (o al arrancar el sistema) y se expone
    públicamente con ngrok: ngrok http <PORT>
  · TWILIO_BRIDGE_URL=wss://<ngrok-id>.ngrok.io/twilio/bridge

Flujo durante una llamada de emergencia (4 fases):
  · Audio bidireccional siempre activo: mic RPi → UDP 9998 → μ-law → Twilio
    (operador oye al humano); μ-law de Twilio → aplay (humano oye al operador).
  · Fase 1: audio directo. La persona habla con el operador. Se vigila si la
    persona DEJA de comunicarse: si pasan ≥6 s sin habla coherente del residente
    (silencio o habla ininteligible sostenida) → Ana toma el relevo (Fase 2).
    La señal de "habla coherente" llega de asistente_node por UDP 9999 ('COHERENT').
    Si el operador cuelga (el WebSocket cierra) → fin del protocolo.
  · Fase 2: Piper transmite al operador la info del residente, en orden
    (1º nombre+dirección, 2º condiciones médicas, 3º alergias/peculiaridades)
    desde perfil_residente.json.
  · Fase 3 (si hay historial previo): el LLM de emergencia genera un resumen del
    contexto y Piper lo transmite al operador. Si no hay historial, se omite.
  · Fase 4: segunda ronda de audio directo. La llamada SOLO la cierra el operador;
    el relevo de Piper no vuelve a dispararse.

Dependencias (aparte de stdlib):
  pip install websockets

Uso:
  python3 bridge_twilio_emergencia.py [--port PORT]
  # PORT por defecto: 8080
"""

import argparse
import asyncio
import audioop
import base64
import hashlib
import json
import logging
import os
import socket
import subprocess
import sys
import time
import threading

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [bridge] %(levelname)s %(message)s',
)
log = logging.getLogger('bridge')

BRIDGE_URL_PATH = '/tmp/bridge_url.txt'  # gestor_emergencia.py lee la URL de aquí

# ──────────────────────────────────────────────────────────────────────────────
# Constantes de audio
# ──────────────────────────────────────────────────────────────────────────────
MIC_RATE = 16000          # RPi arecord → asistente_node envía a 16kHz S16_LE
TWILIO_RATE = 8000        # Twilio Media Streams: μ-law 8kHz mono
TWILIO_CHUNK_MS = 20      # Twilio envía chunks de 20 ms

BRIDGE_AUDIO_PORT = 9998     # UDP: recibimos PCM de asistente_node
BRIDGE_COHERENT_PORT = 9999  # UDP: recibimos señal "COHERENT"

# ── Tiempos del protocolo de fases (1→4) ─────────────────────────────────────
# Fase 1: ≥ este tiempo sin habla coherente del residente (silencio o habla
# ininteligible sostenida) → relevo de Piper para hablar con el 112.
RESIDENTE_SILENCIO_SEG = 6.0
CONTEXTO_PATH = '/tmp/emergencia_contexto.json'

# ── Audio fijo pre-sintetizado (Fase 2) ──────────────────────────────────────
# El mensaje del residente (nombre+dirección, condiciones, alergias) sale de
# perfil_residente.json, así que es FIJO entre llamadas. Lo pre-sintetizamos UNA
# vez con un modelo de ALTA CALIDAD (la latencia offline no importa) y lo cacheamos
# en μ-law 8kHz (formato Twilio). En la llamada solo se streamean los bytes →
# latencia ~0 y mejor voz. La cache se invalida sola si cambia el texto o el modelo.
_HOME = os.path.expanduser('~')
PERFIL_PATH = os.path.join(_HOME, 'asistente_turtlebot4-main', 'perfil_residente.json')
PIPER_MODEL_EMERGENCIA = os.environ.get('PIPER_MODEL_EMERGENCIA', '').strip() or os.path.join(
    _HOME, 'asistente_turtlebot4-main', 'models', 'piper', 'es_AR-daniela-high.onnx')
CACHE_DIR = os.path.join(_HOME, 'asistente_turtlebot4-main', 'cache_emergencia')
# Velocidad de síntesis para los audios fijos de emergencia (>1 = más lento/pausado).
# 1.3 da un tono calmado y claro sin alargar en exceso el mensaje al operador del 112.
LENGTH_SCALE_EMERGENCIA = float(os.environ.get('PIPER_LENGTH_SCALE_EMERGENCIA', '1.3'))

LLM_MODEL = 'meta-llama/llama-3.2-3b-instruct'
OPENROUTER_BASE_URL = 'https://openrouter.ai/api/v1'

# System prompt del LLM de emergencia: se lee de contexto_emergencia.txt; si no
# existe, se usa este texto como fallback robusto.
CONTEXTO_EMERGENCIA_TXT = os.path.join(
    os.path.expanduser('~'), 'asistente_turtlebot4-main', 'contexto_emergencia.txt')

_SYSTEM_PROMPT_EMERGENCIA_FALLBACK = (
    'Eres un asistente médico hablando directamente con un operador del 112. '
    'La persona junto al robot no puede comunicarse con claridad. '
    'Informa brevemente en español formal: nombre, edad, dirección, condiciones '
    'médicas conocidas, y qué ocurrió según la conversación anterior. '
    'Máximo 60 palabras. Sin saludos ni despedidas. Solo texto plano, sin etiquetas.'
)


def _cargar_system_prompt() -> str:
    """Lee el system prompt del LLM de emergencia desde contexto_emergencia.txt."""
    try:
        with open(CONTEXTO_EMERGENCIA_TXT, 'r', encoding='utf-8') as f:
            txt = f.read().strip()
        if txt:
            return txt
    except OSError as e:
        log.warning(f'No pude leer {CONTEXTO_EMERGENCIA_TXT} ({e}); uso prompt por defecto.')
    return _SYSTEM_PROMPT_EMERGENCIA_FALLBACK

# ──────────────────────────────────────────────────────────────────────────────
# Utilidades de audio
# ──────────────────────────────────────────────────────────────────────────────

def pcm16k_a_ulaw8k(data: bytes) -> bytes:
    """Convierte PCM S16_LE 16kHz mono a μ-law 8kHz mono (formato Twilio)."""
    data_8k, _ = audioop.ratecv(data, 2, 1, MIC_RATE, TWILIO_RATE, None)
    return audioop.lin2ulaw(data_8k, 2)


def ulaw8k_a_pcm16k(data: bytes) -> bytes:
    """Convierte μ-law 8kHz (de Twilio) a PCM S16_LE 16kHz para aplay."""
    pcm_8k = audioop.ulaw2lin(data, 2)
    pcm_16k, _ = audioop.ratecv(pcm_8k, 2, 1, TWILIO_RATE, MIC_RATE, None)
    return pcm_16k


# ──────────────────────────────────────────────────────────────────────────────
# Carga del contexto de emergencia
# ──────────────────────────────────────────────────────────────────────────────

def cargar_contexto() -> dict:
    try:
        with open(CONTEXTO_PATH, 'r', encoding='utf-8') as f:
            return json.load(f)
    except (OSError, json.JSONDecodeError) as e:
        log.warning(f'No pude leer el contexto de emergencia: {e}')
        return {}


def construir_prompt_llm(ctx: dict) -> str:
    perfil = ctx.get('perfil', {})
    historial = ctx.get('historial', [])

    partes = []
    if perfil:
        partes.append('Datos del residente:')
        for k, v in perfil.items():
            partes.append(f'  {k}: {v}')

    if historial:
        partes.append('\nÚltimos intercambios de la conversación:')
        for msg in historial:
            rol = 'Usuario' if msg.get('role') == 'user' else 'Ana'
            partes.append(f'  {rol}: {msg.get("content", "")[:200]}')

    return '\n'.join(partes) if partes else 'Sin datos disponibles.'


def construir_info_residente(ctx: dict) -> str:
    """Texto que Piper transmite al operador en la Fase 2.

    Orden: 1) nombre (+ edad) y dirección, 2) condiciones médicas,
    3) alergias y peculiaridades. Cada dato se omite si no está en el perfil.
    """
    perfil = ctx.get('perfil', {})
    if not perfil:
        return ''
    nombre = perfil.get('nombre', '')
    edad = perfil.get('edad', '')
    direccion = perfil.get('direccion', '')
    condiciones = perfil.get('condiciones', '')
    alergias = perfil.get('alergias', '') or perfil.get('peculiaridades', '')

    partes = []
    if nombre:
        ident = f'Le informo sobre {nombre}'
        if edad:
            ident += f', de {edad} años'
        partes.append(ident + '.')
    if direccion:
        partes.append(f'Se encuentra en {direccion}.')
    if condiciones:
        partes.append(f'Condiciones médicas conocidas: {condiciones}.')
    if alergias:
        partes.append(f'Alergias y peculiaridades: {alergias}.')
    return ' '.join(partes)


def construir_intro(ctx: dict) -> str:
    """Frase de presentación de Ana al operador del 112 (Fase 0).

    Se pre-sintetiza en alta calidad y se envía al inicio de la llamada,
    sustituyendo el <Say> de Twilio (que solo queda como fallback mínimo).
    """
    perfil = ctx.get('perfil', {})
    nombre = perfil.get('nombre', '')
    texto = 'Soy Ana, la asistente robótica'
    if nombre:
        texto += f' de {nombre}'
    texto += '. Hay una emergencia. Me pongo en contacto para informarle.'
    return texto


# ──────────────────────────────────────────────────────────────────────────────
# Síntesis TTS con Piper para enviar al operador
# ──────────────────────────────────────────────────────────────────────────────

def sintetizar_piper(texto: str, piper_model: str, timeout: float = 30,
                     length_scale: float = 1.0) -> bytes | None:
    """Sintetiza texto con Piper y devuelve PCM S16_LE a la frecuencia del modelo.

    `timeout`: 30 s en ruta en vivo; usar 300 s para pre-síntesis offline (daniela-high
    es lento en aarch64). `length_scale` > 1.0 alarga los fonemas → voz más pausada.
    """
    if not piper_model or not os.path.exists(piper_model):
        log.error(f'Modelo Piper no encontrado: {piper_model}')
        return None
    try:
        piper_bin = subprocess.run(
            ['which', 'piper'], capture_output=True, text=True
        ).stdout.strip() or 'piper'
        cmd = [piper_bin, '--model', piper_model, '--output_raw']
        if length_scale != 1.0:
            cmd += ['--length_scale', str(length_scale)]
        proc = subprocess.run(
            cmd,
            input=texto.encode('utf-8'),
            capture_output=True,
            timeout=timeout,
        )
        if proc.returncode != 0:
            log.error(f'Piper error: {proc.stderr.decode()[:200]}')
            return None
        return proc.stdout
    except Exception as e:
        log.error(f'Síntesis Piper fallida: {e}')
        return None


def pcm_piper_a_ulaw8k(pcm_data: bytes, piper_rate: int = 22050) -> bytes:
    """Convierte PCM de Piper (22050 Hz S16_LE) a μ-law 8kHz para Twilio."""
    pcm_8k, _ = audioop.ratecv(pcm_data, 2, 1, piper_rate, TWILIO_RATE, None)
    return audioop.lin2ulaw(pcm_8k, 2)


# ──────────────────────────────────────────────────────────────────────────────
# Cache del audio fijo del residente (Fase 2): pre-síntesis en alta calidad
# ──────────────────────────────────────────────────────────────────────────────

def _leer_sample_rate(model_path: str) -> int:
    """Lee audio.sample_rate de <model>.json (default 22050 si no se puede)."""
    try:
        with open(model_path + '.json', 'r', encoding='utf-8') as f:
            return int(json.load(f).get('audio', {}).get('sample_rate', 22050))
    except Exception:
        return 22050


def _modelo_emergencia(ctx: dict) -> str:
    """Modelo de alta calidad para la Fase 2; fallback al del contexto (medium)."""
    if os.path.exists(PIPER_MODEL_EMERGENCIA):
        return PIPER_MODEL_EMERGENCIA
    return ctx.get('piper_model', '')


def _obtener_audio_cacheado(ctx: dict, *, texto: str, prefijo: str,
                            etiqueta: str, key_extra: str = '',
                            regenerar: bool = False) -> bytes | None:
    """Devuelve el μ-law 8kHz de `texto`, cacheado en disco (escritura atómica).

    Patrón común de la intro (Fase 0) y la info del residente (Fase 2): cache hit
    → lectura instantánea; cache miss → síntesis con el modelo de alta calidad,
    conversión a μ-law y guardado, purgando versiones antiguas con el mismo
    `prefijo`. Devuelve None si no hay `texto`, no hay modelo o falla la síntesis
    (las fases caen entonces al fallback en vivo).

    `prefijo` nombra los ficheros de cache (info_residente_ / intro_); `key_extra`
    se intercala en la clave sha1 para que dos textos distintos con el mismo modelo
    no colisionen; `etiqueta` solo da forma a los mensajes de log.
    """
    if not texto:
        return None
    model = _modelo_emergencia(ctx)
    if not model:
        return None
    key = hashlib.sha1(
        f'{os.path.basename(model)}|{key_extra}{texto}'.encode('utf-8')
    ).hexdigest()[:16]
    cache_file = os.path.join(CACHE_DIR, f'{prefijo}{key}.ulaw')

    if not regenerar and os.path.exists(cache_file):
        try:
            with open(cache_file, 'rb') as f:
                return f.read()
        except OSError as e:
            log.warning(f'No pude leer cache {cache_file} ({e}); re-sintetizo.')

    # Timeout generoso: la pre-síntesis es offline y los modelos high son lentos en RPi.
    log.info(f'Pre-sintetizando {etiqueta} con {os.path.basename(model)} '
             f'(alta calidad, length_scale={LENGTH_SCALE_EMERGENCIA})...')
    pcm = sintetizar_piper(texto, model, timeout=300, length_scale=LENGTH_SCALE_EMERGENCIA)
    if pcm is None:
        return None
    ulaw = pcm_piper_a_ulaw8k(pcm, _leer_sample_rate(model))

    try:
        os.makedirs(CACHE_DIR, exist_ok=True)
        tmp = cache_file + '.tmp'
        with open(tmp, 'wb') as f:
            f.write(ulaw)
        os.replace(tmp, cache_file)
        # Purgar audios antiguos de otra versión del perfil/modelo (mismo prefijo).
        for nombre in os.listdir(CACHE_DIR):
            if nombre.startswith(prefijo) and nombre != os.path.basename(cache_file):
                try:
                    os.remove(os.path.join(CACHE_DIR, nombre))
                except OSError:
                    pass
        log.info(f'Audio de «{etiqueta}» cacheado en {cache_file} ({len(ulaw)} bytes μ-law).')
    except OSError as e:
        log.warning(f'No pude escribir la cache de audio ({e}); uso el audio en memoria.')
    return ulaw


def obtener_audio_info_residente(ctx: dict, regenerar: bool = False) -> bytes | None:
    """Mensaje fijo del residente (Fase 2), cacheado en alta calidad."""
    return _obtener_audio_cacheado(
        ctx, texto=construir_info_residente(ctx),
        prefijo='info_residente_', etiqueta='info del residente',
        regenerar=regenerar)


def obtener_audio_intro(ctx: dict, regenerar: bool = False) -> bytes | None:
    """Intro de presentación de Ana (Fase 0), cacheada en alta calidad.

    `key_extra='intro|'` mantiene la clave de cache distinta de la info del
    residente aunque compartan modelo.
    """
    return _obtener_audio_cacheado(
        ctx, texto=construir_intro(ctx),
        prefijo='intro_', etiqueta='intro', key_extra='intro|',
        regenerar=regenerar)


def prewarm_cache_emergencia():
    """Pre-sintetiza todos los audios fijos de emergencia al arrancar.

    Sintetiza la intro (Fase 0) y la info del residente (Fase 2) con daniela-high
    a velocidad pausada. En la llamada real solo se leen los bytes cacheados (~0 ms).
    Si el perfil no existe o la síntesis falla, la llamada usa fallback en vivo.
    """
    try:
        with open(PERFIL_PATH, 'r', encoding='utf-8') as f:
            perfil = json.load(f)
    except (OSError, ValueError) as e:
        log.info(f'Pre-warm: perfil_residente.json no disponible ({e}); omito.')
        return
    ctx = {'perfil': perfil}
    n_ok = 0
    if obtener_audio_intro(ctx) is not None:
        n_ok += 1
        log.info('Pre-warm: intro cacheada.')
    if obtener_audio_info_residente(ctx) is not None:
        n_ok += 1
        log.info('Pre-warm: info del residente cacheada.')
    log.info(f'Pre-warm del audio de emergencia completado ({n_ok}/2 audios).')


# ──────────────────────────────────────────────────────────────────────────────
# LLM de emergencia
# ──────────────────────────────────────────────────────────────────────────────

def generar_resumen_llm(ctx: dict) -> str | None:
    # La Fase 3 solo tiene sentido si hay historial previo de la conversación.
    if not ctx.get('historial'):
        log.info('Sin historial de conversación; se omite el resumen LLM (Fase 3).')
        return None

    api_key = ctx.get('openrouter_api_key') or os.environ.get('OPENROUTER_API_KEY', '')
    if not api_key:
        log.error('OPENROUTER_API_KEY no disponible para LLM de emergencia.')
        return None

    prompt_usuario = construir_prompt_llm(ctx)
    try:
        from openai import OpenAI
        client = OpenAI(base_url=OPENROUTER_BASE_URL, api_key=api_key)
        resp = client.chat.completions.create(
            model=LLM_MODEL,
            messages=[
                {'role': 'system', 'content': _cargar_system_prompt()},
                {'role': 'user', 'content': prompt_usuario},
            ],
            max_tokens=120,
            stream=False,
            timeout=10.0,
        )
        return resp.choices[0].message.content.strip()
    except Exception as e:
        log.error(f'LLM de emergencia falló: {e}')
        return None


# ──────────────────────────────────────────────────────────────────────────────
# Servidor WebSocket (Twilio Media Streams)
# ──────────────────────────────────────────────────────────────────────────────

class SesionBridge:
    """Gestiona una sesión de Twilio Media Streams."""

    def __init__(self, websocket):
        self.ws = websocket
        self.stream_sid = None
        self.ctx = cargar_contexto()
        self._activa = True
        self._llm_hablando = False
        self._aplay_proc = None
        self._n_pkts_operador = 0
        self._n_pkts_mic = 0
        # Estado del resampler de audioop. Mantenerlo entre paquetes evita los
        # clicks/artefactos que aparecen al reiniciarlo en cada chunk (lo que
        # degradaba la inteligibilidad en ambos sentidos).
        self._rcv_state = None  # μ-law 8k → PCM 16k (operador → altavoz)
        self._snd_state = None  # PCM 16k → μ-law 8k (mic → operador)
        # Marks pendientes: nombre_mark -> asyncio.Event. Twilio reproduce el audio
        # saliente desde su propio buffer; el eco del 'mark' nos dice cuándo TERMINA
        # de reproducirlo, no de recibirlo. Lo usa _enviar_ulaw para esperar al fin
        # de la reproducción real (ver allí).
        self._marks_pendientes = {}

        # ── Protocolo de fases (1→4) ───────────────────────────────────────
        self._fase = '1_audio_directo'      # fase actual del protocolo de llamada
        self._t_ultimo_audio_op = time.time()  # último paquete de audio del operador
        # Fase 1: vigilamos si el residente deja de comunicarse. asistente_node
        # envía 'COHERENT' por UDP 9999 cada vez que detecta habla coherente del
        # residente; al recibirlo reseteamos este timestamp. Si pasan
        # RESIDENTE_SILENCIO_SEG sin coherencia → Ana toma el relevo (Fase 2).
        self._t_ultima_coherencia = time.time()

    async def manejar(self):
        tasks = [
            asyncio.create_task(self._recibir_twilio()),
            asyncio.create_task(self._enviar_mic()),
            asyncio.create_task(self._secuenciador_fases()),
            asyncio.create_task(self._escuchar_coherencia()),
        ]
        try:
            # Esperar a que CUALQUIER tarea termine (normalmente _recibir_twilio
            # cuando Twilio cierra el WS). Las demás tienen loops con while self._activa
            # y saldrán en cuanto _recibir_twilio ponga _activa = False en su finally.
            done, _ = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
            for t in done:
                if not t.cancelled() and t.exception():
                    log.error(f'Tarea bridge falló: {t.exception()}')
        finally:
            self._activa = False
            for t in tasks:
                t.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)
            self._cerrar_aplay()

    # ── Recibir audio de Twilio → aplay ──────────────────────────────────────

    async def _recibir_twilio(self):
        try:
            async for mensaje in self.ws:
                if not self._activa:
                    break
                try:
                    datos = json.loads(mensaje)
                except Exception:
                    continue

                evento = datos.get('event')
                if evento == 'connected':
                    log.info(f'Twilio conectado: protocolo={datos.get("protocol")} '
                             f'version={datos.get("version")}')

                elif evento == 'start':
                    start_data = datos.get('start', {})
                    self.stream_sid = start_data.get('streamSid')
                    tracks = start_data.get('tracks', [])
                    media_fmt = start_data.get('mediaFormat', {})
                    log.info(f'Stream iniciado: sid={self.stream_sid} '
                             f'tracks={tracks} formato={media_fmt}')
                    self._iniciar_aplay()

                elif evento == 'media':
                    if self._llm_hablando:
                        continue  # Ignorar audio entrante mientras el LLM habla
                    payload = datos.get('media', {}).get('payload', '')
                    if payload:
                        ulaw = base64.b64decode(payload)
                        pcm_8k = audioop.ulaw2lin(ulaw, 2)
                        pcm, self._rcv_state = audioop.ratecv(
                            pcm_8k, 2, 1, TWILIO_RATE, MIC_RATE, self._rcv_state)
                        self._escribir_aplay(pcm)
                        self._t_ultimo_audio_op = time.time()
                        self._n_pkts_operador += 1
                        if self._n_pkts_operador % 50 == 0:
                            log.info(f'[rx] {self._n_pkts_operador} paquetes del operador reproducidos')

                elif evento == 'mark':
                    # Eco de un mark que enviamos al final de un clip de Ana: Twilio
                    # lo devuelve cuando ha TERMINADO de reproducir todo lo previo.
                    nombre = datos.get('mark', {}).get('name', '')
                    ev = self._marks_pendientes.get(nombre)
                    if ev is not None:
                        ev.set()
                        log.info(f'[mark] reproducción completada: {nombre}')

                elif evento == 'stop':
                    log.info('Twilio: evento stop recibido.')
                    self._activa = False
                    break
        except Exception as e:
            log.error(f'_recibir_twilio: {e}')
        finally:
            log.info('_recibir_twilio: WebSocket terminado, señalizando parada')
            self._activa = False

    def _iniciar_aplay(self):
        card = self.ctx.get('alsa_dac', '')
        dev_arg = ['-D', f'plughw:CARD={card},DEV=0'] if card else []
        log.info(f'Iniciando aplay card={card or "(default)"}')
        try:
            self._aplay_proc = subprocess.Popen(
                ['aplay', '-f', 'S16_LE', '-r', str(MIC_RATE), '-c', '1',
                 *dev_arg, '-'],
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
            )
        except Exception as e:
            log.error(f'No pude iniciar aplay: {e}')
            return

        # Vigilar aplay: si muere (típicamente "Device or resource busy" cuando
        # otro proceso tiene el altavoz hw abierto), loguear el error en vez de
        # tragárnoslo en silencio mientras seguimos contando paquetes recibidos.
        def _vigilar(p):
            try:
                err = p.stderr.read()
                rc = p.wait()
                if rc != 0:
                    detalle = err.decode(errors='replace').strip() if err else ''
                    log.error(f'aplay murió (rc={rc}): {detalle} '
                              '— el operador NO se oye por el altavoz.')
            except Exception:
                pass
        threading.Thread(target=_vigilar, args=(self._aplay_proc,),
                         daemon=True).start()

    def _escribir_aplay(self, pcm: bytes):
        if self._aplay_proc and self._aplay_proc.stdin:
            try:
                self._aplay_proc.stdin.write(pcm)
            except Exception:
                pass

    def _cerrar_aplay(self):
        if self._aplay_proc:
            try:
                self._aplay_proc.stdin.close()
                self._aplay_proc.wait(timeout=2)
            except Exception:
                self._aplay_proc.kill()

    # ── Enviar audio del mic → Twilio ────────────────────────────────────────

    async def _enviar_mic(self):
        """Recibe chunks PCM del socket UDP 9998 y los envía a Twilio como μ-law.

        Usa loop.sock_recv (event-driven, sin polling ni thread pool): despierta en
        cuanto llega un datagrama, sin añadir latencia artificial.
        """
        loop = asyncio.get_event_loop()
        udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_sock.setblocking(False)
        try:
            udp_sock.bind(('127.0.0.1', BRIDGE_AUDIO_PORT))
        except OSError as e:
            log.error(f'No pude abrir UDP {BRIDGE_AUDIO_PORT}: {e}')
            return

        log.info(f'Escuchando audio del mic en UDP {BRIDGE_AUDIO_PORT}')
        try:
            while self._activa:
                try:
                    data = await asyncio.wait_for(
                        loop.sock_recv(udp_sock, 8192), timeout=0.5)
                except asyncio.TimeoutError:
                    continue  # sin datos → comprobar _activa y volver
                if self._llm_hablando or not self.stream_sid:
                    continue  # descartar mientras Ana habla al operador
                data_8k, self._snd_state = audioop.ratecv(
                    data, 2, 1, MIC_RATE, TWILIO_RATE, self._snd_state)
                ulaw = audioop.lin2ulaw(data_8k, 2)
                msg = json.dumps({
                    'event': 'media',
                    'streamSid': self.stream_sid,
                    'media': {
                        'payload': base64.b64encode(ulaw).decode('ascii'),
                    },
                })
                await self.ws.send(msg)
                self._n_pkts_mic += 1
                if self._n_pkts_mic % 50 == 0:
                    log.info(f'[tx] {self._n_pkts_mic} paquetes mic enviados a Twilio')
        finally:
            udp_sock.close()

    # ── Secuenciador de fases del protocolo de llamada (1→4) ─────────────────

    async def _secuenciador_fases(self):
        """Orquesta las 4 fases de la llamada de emergencia.

        1: audio directo; si el residente deja de comunicarse (≥6 s sin habla
           coherente) → relevo de Piper. Si el operador cuelga → fin.
        2: Piper transmite la info estructurada del residente al operador.
        3: si hay historial, resumen LLM por Piper.
        4: segunda ronda de audio directo (solo el operador cierra la llamada;
           el relevo de Piper no vuelve a dispararse).
        """
        # Esperar al evento 'start' de Twilio (stream_sid disponible).
        while self._activa and not self.stream_sid:
            await asyncio.sleep(0.1)
        if not self._activa:
            return

        # ── Fase 0: intro de alta calidad desde cache ──
        self._fase = '0_intro'
        await self._fase_intro()
        if not self._activa:
            return

        # ── Fase 1: audio directo, vigilando el silencio del residente ──
        self._fase = '1_audio_directo'
        log.info(f'[Fase 1] audio directo; vigilando si el residente deja de '
                 f'comunicarse (≥{RESIDENTE_SILENCIO_SEG:.0f}s sin habla coherente)...')
        self._t_ultima_coherencia = time.time()  # reset al entrar en la fase
        while self._activa:
            await asyncio.sleep(0.5)
            silencio = time.time() - self._t_ultima_coherencia
            if silencio >= RESIDENTE_SILENCIO_SEG:
                log.info(f'[Fase 1] {silencio:.0f}s sin habla coherente del '
                         'residente; Ana toma el relevo con el 112.')
                break
        if not self._activa:
            return  # el operador colgó → fin del protocolo

        # ── Fase 2: Piper transmite la info estructurada del residente ──
        self._fase = '2_info_residente'
        await self._fase_info_residente()
        if not self._activa:
            return

        # ── Fase 3 (condicional): resumen de contexto con el LLM ──
        self._fase = '3_contexto_llm'
        await self._fase_contexto_llm()
        if not self._activa:
            return

        # ── Fase 4: audio directo hasta que el operador cuelgue ──
        self._fase = '4_audio_directo'
        await self._ronda_audio_directo('4')

    async def _ronda_audio_directo(self, fase: str):
        """Ronda de audio directo bidireccional (_recibir_twilio + _enviar_mic).

        Solo termina cuando el operador cuelga (la llamada NUNCA la cierra el
        sistema); `_recibir_twilio` baja `_activa` al cerrarse el WebSocket.
        """
        log.info(f'[Fase {fase}] audio directo bidireccional.')
        self._t_ultimo_audio_op = time.time()
        while self._activa:
            await asyncio.sleep(1.0)

    async def _fase_intro(self):
        """Fase 0: presentación de Ana al operador, pre-sintetizada en alta calidad.

        Sustituye en contenido al <Say> de Twilio (que queda solo como fallback
        mínimo "Emergencia médica." mientras el WebSocket se abre). En cache hit
        la latencia es ~0; en cache miss (primer arranque) tarda ~45 s y no se
        ejecuta esta fase (la cache se rellena en background al arrancar el bridge).
        """
        loop = asyncio.get_event_loop()
        ulaw = await loop.run_in_executor(None, obtener_audio_intro, self.ctx)
        if ulaw is not None:
            log.info('[Fase 0] transmitiendo intro (audio cacheado, alta calidad).')
            await self._enviar_ulaw(ulaw)
        else:
            log.info('[Fase 0] sin cache de intro; el <Say> de Twilio sirvió de anuncio.')

    async def _fase_info_residente(self):
        """Fase 2: transmite al operador la info fija del residente (audio cacheado).

        El texto es fijo (perfil) → se sirve desde la cache pre-sintetizada en alta
        calidad (latencia ~0). Si no hay cache/síntesis, cae a síntesis en vivo.
        """
        texto = construir_info_residente(self.ctx)
        if not texto:
            log.info('[Fase 2] sin datos de perfil; nada que transmitir.')
            return
        loop = asyncio.get_event_loop()
        ulaw = await loop.run_in_executor(
            None, obtener_audio_info_residente, self.ctx)
        if ulaw is not None:
            log.info('[Fase 2] transmitiendo info del residente (audio cacheado, '
                     'alta calidad).')
            await self._enviar_ulaw(ulaw)
        else:
            log.info('[Fase 2] sin audio cacheado; sintetizo en vivo (fallback).')
            await self._enviar_texto_piper(texto)

    async def _fase_contexto_llm(self):
        """Fase 3: resumen de contexto con el LLM (solo si hay historial)."""
        if not self.ctx.get('historial'):
            log.info('[Fase 3] sin historial de conversación; se omite.')
            return
        log.info('[Fase 3] generando resumen de contexto con el LLM...')
        loop = asyncio.get_event_loop()
        texto = await loop.run_in_executor(None, generar_resumen_llm, self.ctx)
        if not texto:
            log.info('[Fase 3] el LLM no devolvió resumen; se omite.')
            return
        log.info(f'[Fase 3] resumen: {texto[:100]}...')
        await self._enviar_texto_piper(texto)

    async def _enviar_texto_piper(self, texto: str):
        """Sintetiza `texto` con Piper EN VIVO y lo envía al operador (Fase 3 / fallback)."""
        loop = asyncio.get_event_loop()
        piper_model = self.ctx.get('piper_model', '')
        pcm = await loop.run_in_executor(
            None, lambda: sintetizar_piper(
                texto, piper_model, length_scale=LENGTH_SCALE_EMERGENCIA))
        if pcm is None:
            log.error('Síntesis Piper fallida; no se envía audio al operador.')
            return
        ulaw = pcm_piper_a_ulaw8k(pcm, _leer_sample_rate(piper_model))
        await self._enviar_ulaw(ulaw)

    async def _enviar_ulaw(self, ulaw: bytes):
        """Envía μ-law 8kHz al operador por Twilio y espera a que TERMINE de sonar.

        Entrega TODO el clip en ráfaga (sin pacing en tiempo real): Twilio bufferiza
        el audio saliente y lo reproduce a 8 kHz constante, así que entregarlo de
        golpe elimina los cortes/robotización que provocaba el `asyncio.sleep` por
        chunk (entregaba más lento que tiempo real → underrun del buffer de Twilio).

        Tras el clip enviamos un `mark` y esperamos su eco: Twilio lo devuelve cuando
        acaba de REPRODUCIR todo lo bufferizado, no cuando lo recibe. Eso mantiene
        `_llm_hablando=True` durante toda la reproducción real, evitando que
        `_enviar_mic` reanude el micro y se mezcle con la voz de Ana aún sonando.
        """
        if not self.stream_sid:
            log.warning('No hay stream_sid todavía; envío de audio postergado.')
            return
        self._llm_hablando = True
        nombre_mark = f'ana_{int(time.time() * 1000)}'
        ev = asyncio.Event()
        self._marks_pendientes[nombre_mark] = ev
        try:
            chunk_size = int(TWILIO_RATE * TWILIO_CHUNK_MS / 1000)
            # Ráfaga: todos los chunks seguidos (await ws.send ya cede el event loop).
            for i in range(0, len(ulaw), chunk_size):
                if not self._activa or not self.stream_sid:
                    break
                chunk = ulaw[i:i + chunk_size]
                await self.ws.send(json.dumps({
                    'event': 'media',
                    'streamSid': self.stream_sid,
                    'media': {
                        'payload': base64.b64encode(chunk).decode('ascii'),
                    },
                }))

            if not self._activa or not self.stream_sid:
                return  # operador colgó a mitad; nada que esperar

            await self.ws.send(json.dumps({
                'event': 'mark',
                'streamSid': self.stream_sid,
                'mark': {'name': nombre_mark},
            }))
            log.info('Audio enviado al operador en ráfaga; esperando fin de reproducción.')

            # Esperar el eco del mark (fin de reproducción). Timeout = duración del
            # clip + margen, por si el operador cuelga y el mark nunca llega.
            dur = len(ulaw) / TWILIO_RATE  # μ-law 8 kHz: 1 byte = 1 muestra
            try:
                await asyncio.wait_for(ev.wait(), timeout=dur + 5.0)
            except asyncio.TimeoutError:
                log.warning('No llegó el eco del mark de Twilio; continúo igualmente.')
        except Exception as e:
            log.error(f'_enviar_ulaw: {e}')
        finally:
            self._marks_pendientes.pop(nombre_mark, None)
            self._llm_hablando = False

    # ── Drenar señales de coherencia desde asistente_node ────────────────────

    async def _escuchar_coherencia(self):
        """Recibe el UDP 'COHERENT' que envía asistente_node.

        asistente_node mantiene el STT del residente activo durante la emergencia
        y emite 'COHERENT' cada vez que detecta habla coherente. Al recibirlo
        reseteamos `_t_ultima_coherencia`, que es lo que la Fase 1 usa para decidir
        si el residente ha dejado de comunicarse (≥RESIDENTE_SILENCIO_SEG sin señal).
        """
        loop = asyncio.get_event_loop()
        udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_sock.setblocking(False)
        try:
            udp_sock.bind(('127.0.0.1', BRIDGE_COHERENT_PORT))
        except OSError as e:
            log.error(f'No pude abrir UDP {BRIDGE_COHERENT_PORT}: {e}')
            return

        log.info(f'Escuchando coherencia en UDP {BRIDGE_COHERENT_PORT}')
        try:
            while self._activa:
                try:
                    await loop.run_in_executor(
                        None, lambda: udp_sock.recv(64))
                    self._t_ultima_coherencia = time.time()
                    log.info('[coherencia] habla coherente del residente; '
                             'reseteo el timer de relevo.')
                except BlockingIOError:
                    await asyncio.sleep(0.05)
                except Exception:
                    await asyncio.sleep(0.05)
        finally:
            udp_sock.close()


# ──────────────────────────────────────────────────────────────────────────────
# Servidor principal
# ──────────────────────────────────────────────────────────────────────────────

async def handler(websocket):
    """Punto de entrada para cada conexión WebSocket de Twilio."""
    peer = websocket.remote_address
    log.info(f'Nueva conexión de Twilio desde {peer}')
    sesion = SesionBridge(websocket)
    try:
        await sesion.manejar()
    except Exception as e:
        log.error(f'Sesión terminada con error: {e}')
    finally:
        log.info(f'Conexión cerrada: {peer}')


_NGROK_CANDIDATOS = [
    # Instalación del usuario (ubuntu)
    os.path.expanduser('~/.local/bin/ngrok'),
    # Instalación root (cuando install.sh se ejecuta con sudo)
    '/root/.local/bin/ngrok',
    # Sistema
    '/usr/local/bin/ngrok',
    '/usr/bin/ngrok',
    # PATH del proceso actual
    'ngrok',
]


def _buscar_ngrok() -> str | None:
    """Devuelve la ruta al binario ngrok, o None si no está instalado."""
    for candidato in _NGROK_CANDIDATOS:
        try:
            r = subprocess.run([candidato, 'version'],
                               capture_output=True, timeout=3)
            if r.returncode == 0:
                return candidato
        except Exception:
            continue
    return None


def _iniciar_ngrok(port: int) -> str | None:
    """Abre un túnel ngrok en el puerto dado y devuelve la URL wss://.

    Intenta primero el módulo pyngrok (si está disponible para este usuario),
    y si no, llama al binario ngrok directamente analizando su salida JSON.
    Requiere haber ejecutado: ngrok config add-authtoken <tu-token>
    """
    # — Intento 1: módulo pyngrok —
    try:
        from pyngrok import ngrok, conf
        token = os.environ.get('NGROK_AUTHTOKEN', '')
        if token:
            conf.get_default().auth_token = token
        tunnel = ngrok.connect(port, 'http')
        public_url = tunnel.public_url
        ws_url = public_url.replace('https://', 'wss://').replace('http://', 'ws://')
        ws_url = ws_url.rstrip('/') + '/twilio/bridge'
        log.info(f'ngrok (pyngrok) activo: {ws_url}')
        return ws_url
    except ImportError:
        pass  # pyngrok no disponible para este usuario; intentar binario
    except Exception as e:
        log.warning(f'pyngrok falló: {e}')

    # — Intento 2: binario ngrok directamente —
    ngrok_bin = _buscar_ngrok()
    if ngrok_bin is None:
        log.warning(
            'ngrok no encontrado. Para el bridge automático:\n'
            '  1) Crea cuenta en https://ngrok.com (gratis)\n'
            '  2) ngrok config add-authtoken <token>\n'
            '  O pon TWILIO_BRIDGE_URL manualmente en secrets.env')
        return None

    token = os.environ.get('NGROK_AUTHTOKEN', '')
    if token:
        try:
            subprocess.run([ngrok_bin, 'config', 'add-authtoken', token],
                           capture_output=True, timeout=5)
        except Exception:
            pass

    try:
        # Arrancar ngrok en background y leer la URL de su API local (JSON log)
        proc = subprocess.Popen(
            [ngrok_bin, 'http', str(port), '--log=stdout', '--log-format=json'],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
        )
        deadline = time.time() + 10
        while time.time() < deadline:
            line = proc.stdout.readline()
            if not line:
                if proc.poll() is not None:
                    break  # proceso terminó (error fatal)
                time.sleep(0.05)
                continue
            try:
                obj = json.loads(line.decode())
                # URL pública encontrada
                url = obj.get('url', '')
                if url.startswith('http'):
                    ws_url = url.replace('https://', 'wss://').replace('http://', 'ws://')
                    ws_url = ws_url.rstrip('/') + '/twilio/bridge'
                    log.info(f'ngrok (binario) activo: {ws_url}')
                    # Drenar stdout en background: sin esto el buffer del OS (~64 KB)
                    # se llena, ngrok se bloquea escribiendo logs y el túnel deja de
                    # responder → Twilio no puede conectar al WebSocket.
                    threading.Thread(target=proc.stdout.read, daemon=True).start()
                    return ws_url
                # Error de autenticación → salir inmediatamente sin esperar los 10 s
                err = obj.get('err', '')
                lvl = obj.get('lvl', '')
                if lvl in ('eror', 'crit') and ('authentication failed' in err
                                                  or 'ERR_NGROK_4018' in err):
                    log.warning(
                        'ngrok requiere authtoken. Para activarlo:\n'
                        '  1) Crea cuenta en https://ngrok.com (gratis)\n'
                        '  2) ngrok config add-authtoken <token>   (una vez)')
                    proc.terminate()
                    return None
            except Exception:
                pass
        proc.terminate()
        log.warning('ngrok no publicó URL en 10 s. '
                    '¿Falta el authtoken? Ejecuta: ngrok config add-authtoken <token>')
        return None
    except Exception as e:
        log.warning(f'No pude arrancar ngrok: {e}')
        return None


def _encontrar_puerto_libre(base: int, intentos: int = 10) -> int:
    """Devuelve el primer puerto >= base que esté libre en TCP."""
    for p in range(base, base + intentos):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                s.bind(('0.0.0.0', p))
                return p
            except OSError:
                log.debug(f'Puerto {p} ocupado, probando siguiente...')
    raise OSError(f'No hay puerto libre en rango {base}–{base + intentos - 1}')


async def main(port: int):
    try:
        import websockets
    except ImportError:
        log.error('Falta la librería websockets. Instala con: pip install websockets')
        sys.exit(1)

    # Buscar puerto libre antes de lanzar ngrok (evita race condition)
    try:
        actual_port = _encontrar_puerto_libre(port)
    except OSError as e:
        log.error(str(e))
        sys.exit(1)
    if actual_port != port:
        log.warning(f'Puerto {port} ocupado; usando {actual_port}')

    log.info(f'Bridge Twilio emergencia iniciado en ws://0.0.0.0:{actual_port}/twilio/bridge')

    # Intentar abrir túnel ngrok automáticamente
    ws_url = os.environ.get('TWILIO_BRIDGE_URL', '').strip()
    if not ws_url:
        ws_url = _iniciar_ngrok(actual_port) or ''

    if ws_url:
        # Escribir URL para que gestor_emergencia.py la lea al hacer la llamada
        try:
            with open(BRIDGE_URL_PATH, 'w') as f:
                f.write(ws_url)
            log.info(f'URL del bridge escrita en {BRIDGE_URL_PATH}: {ws_url}')
        except OSError as e:
            log.error(f'No pude escribir {BRIDGE_URL_PATH}: {e}')
    else:
        log.warning(
            'Sin URL pública ngrok. El bridge funciona solo en red local.\n'
            '  Para activar ngrok:\n'
            '    1) Crea cuenta gratuita en https://ngrok.com\n'
            '    2) ngrok config add-authtoken <token>   (una sola vez)\n'
            '    3) Reinicia el launch — se configura solo.')
        # Limpiar archivo anterior para que gestor_emergencia use el fallback Say+Pause
        try:
            os.remove(BRIDGE_URL_PATH)
        except OSError:
            pass

    # Pre-sintetizar todos los audios fijos de emergencia (intro + info residente)
    # en alta calidad y en background, para que la llamada arranque con latencia ~0.
    threading.Thread(target=prewarm_cache_emergencia, daemon=True).start()

    async with websockets.serve(handler, '0.0.0.0', actual_port):
        await asyncio.Future()  # correr indefinidamente


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Bridge Twilio ↔ RPi para emergencias')
    parser.add_argument('--port', type=int, default=8765,
                        help='Puerto WebSocket base (defecto: 8765). '
                             'Si está ocupado prueba el siguiente.')
    args = parser.parse_args()
    asyncio.run(main(args.port))
