"""
coordinador_tarea — flujo de recuperación colaborativa de objetos (FETCH).

Implementa el caso de uso "tráeme un café": Ana navega al sitio del objeto,
lo busca con YOLO, pide a un auxiliar humano que lo coloque en su bandeja,
espera confirmación por voz, vuelve con el usuario y lo entrega.

Corre EN PROCESO dentro de `asistente_node` porque necesita su TTS y, sobre
todo, su STT (la confirmación del auxiliar se escucha con el mismo Vosk del
diálogo), pero se aísla como clase para mantener el nodo principal legible.

Orquestación (todo por /voice_command + topics de resultado, sin re-cablear voz):
  registrar_origen → nodo_navegacion guarda la pose actual.
  navegar:<wp>     → ... → /navegacion_resultado 'llegada:<wp>'.
  buscar:<clase>   → object_detector → /deteccion_resultado 'encontrado:<clase>'.
  _hablar(frase) + node.escuchar_confirmacion() → "sí" del auxiliar.
  volver_a_origen  → ... → /navegacion_resultado 'llegada:origen'.
  _hablar(frase_entrega).

El catálogo de objetos (clase YOLO, waypoint, frases) se lee de objetos.yaml,
así añadir objetos no toca código. Un mutex de tarea única impide solapamientos.
Estados publicados en /task/status: NAVIGATING, SEARCHING, WAITING_CONFIRM,
RETURNING, COMPLETED, FAILED.
"""

import os
import threading
import time
import unicodedata

import yaml

from std_msgs.msg import String

OBJETOS_PATH = os.path.join(
    os.path.expanduser('~'), 'asistente_turtlebot4-main', 'objetos.yaml')

NAV_TIMEOUT = 120.0        # s máx esperando que termine una navegación
ORIGEN_TIMEOUT = 5.0       # s esperando el ack de registrar_origen
BUSQUEDA_TIMEOUT = 40.0    # s esperando el resultado de buscar (detector corta a 30 s)
CONFIRM_TIMEOUT = 12.0     # s de cada espera del "sí" del auxiliar (= cooldown de re-pregunta)
# Ana repite "¿ya tienes X en la bandeja?" cada CONFIRM_TIMEOUT s hasta oír "sí",
# re-preguntando también si dice "no" o calla. Tope de seguridad para no quedar
# atascada indefinidamente (~10 intentos x 12 s ≈ 2 min); el barge-in la cancela.
CONFIRM_MAX_INTENTOS = 10


def _normalizar(s):
    """minúsculas + sin acentos, para emparejar el objeto pedido con las claves."""
    s = unicodedata.normalize('NFKD', s.strip().lower())
    return ''.join(c for c in s if not unicodedata.combining(c))


class CoordinadorTarea:
    """Máquina de estados del flujo FETCH. Una tarea a la vez."""

    def __init__(self, node):
        """`node` es el AsistenteNode (usa logger, TTS, STT, pub_comando)."""
        self._node = node
        self._log = node.get_logger()
        self._pub_status = node.create_publisher(String, '/task/status', 10)

        # Resultados machine-readable de nav y percepción.
        self._nav_resultado = None
        self._nav_event = threading.Event()
        self._det_resultado = None
        self._det_event = threading.Event()
        node.create_subscription(
            String, '/navegacion_resultado', self._cb_nav, 10)
        node.create_subscription(
            String, '/deteccion_resultado', self._cb_det, 10)

        self._lock = threading.Lock()
        self._activa = False
        self._cancelar = threading.Event()

    # ------------------------------------------------------------------
    def activa(self):
        """Devuelve True si hay una tarea FETCH en curso."""
        return self._activa

    def cancelar(self):
        """Aborta la tarea en curso (p.ej. barge-in)."""
        if self._activa:
            self._cancelar.set()
            self._pub_cmd('parar')

    def iniciar(self, objeto):
        """Arranca el flujo FETCH para `objeto` si no hay otra tarea en curso."""
        with self._lock:
            if self._activa:
                self._hablar('Ahora mismo estoy ocupada con otra cosa, '
                             'dame un momento.', 'calma', 0.5)
                return
            self._activa = True
        self._cancelar.clear()
        threading.Thread(
            target=self._ejecutar, args=(objeto,), daemon=True).start()

    # ------------------------------------------------------------------
    def _cb_nav(self, msg):
        self._nav_resultado = msg.data.strip()
        self._nav_event.set()

    def _cb_det(self, msg):
        self._det_resultado = msg.data.strip()
        self._det_event.set()

    def _status(self, estado):
        msg = String()
        msg.data = estado
        self._pub_status.publish(msg)
        self._log.info(f'/task/status: {estado}')

    def _hablar(self, texto, tipo='calma', intensidad=0.5):
        try:
            self._node._hablar(texto, tipo, intensidad, priority='drop_old')
        except Exception as e:
            self._log.error(f'No pude hablar en tarea: {e}')

    def _pub_cmd(self, comando):
        msg = String()
        msg.data = comando
        self._node.pub_comando.publish(msg)

    def _esperar_nav(self, timeout):
        """Espera el próximo resultado de navegación (o cancelación/timeout)."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self._cancelar.is_set():
                return None
            if self._nav_event.wait(0.5):
                return self._nav_resultado
        return None

    def _esperar_det(self, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self._cancelar.is_set():
                return None
            if self._det_event.wait(0.5):
                return self._det_resultado
        return None

    def _cargar_objetos(self):
        try:
            with open(OBJETOS_PATH, 'r', encoding='utf-8') as f:
                return yaml.safe_load(f) or {}
        except (OSError, yaml.YAMLError) as e:
            self._log.error(f'No pude leer objetos.yaml: {e}')
            return {}

    # ------------------------------------------------------------------
    def _ejecutar(self, objeto):
        entregado = False
        confirm_intentos = 0
        motivo_fetch = ''
        try:
            objeto = _normalizar(objeto)
            catalogo = self._cargar_objetos()
            cfg = catalogo.get(objeto)
            if not cfg:
                self._log.warn(f'Objeto desconocido en objetos.yaml: "{objeto}"')
                self._hablar(f'Lo siento, no sé dónde encontrar {objeto}.',
                             'preocupacion', 0.5)
                self._status(f'FAILED objeto={objeto} motivo=desconocido')
                motivo_fetch = 'desconocido'
                return

            clase = str(cfg.get('clase_yolo', '')).strip().lower()
            waypoint = _normalizar(str(cfg.get('waypoint', '')))
            frase_pedir = cfg.get('frase_pedir',
                                  '¿Puedes ponerlo en mi bandeja, por favor?')
            frase_entrega = cfg.get('frase_entrega', 'Aquí tienes.')

            # 1) Registrar pose de origen (de dónde sale, para volver).
            self._nav_event.clear()
            self._pub_cmd('registrar_origen')
            res = self._esperar_nav(ORIGEN_TIMEOUT)
            if res != 'origen_ok':
                self._log.warn(
                    f'registrar_origen devolvió "{res}"; abortando tarea.')
                self._hablar(
                    'No sé dónde estás ahora mismo, no puedo traerte el objeto.',
                    'preocupacion', 0.5)
                self._status(f'FAILED objeto={objeto} motivo=sin_origen')
                motivo_fetch = 'sin_origen'
                return
            if self._cancelar.is_set():
                return self._abortar(objeto)

            # 2) Navegar al sitio del objeto.
            self._status(f'NAVIGATING destino={waypoint}')
            self._nav_event.clear()
            self._pub_cmd(f'navegar:{waypoint}')
            res = self._esperar_nav(NAV_TIMEOUT)
            if self._cancelar.is_set():
                return self._abortar(objeto)
            if res is None or not res.startswith('llegada:'):
                self._hablar(f'No he podido llegar a por {objeto}.',
                             'preocupacion', 0.6)
                self._status(f'FAILED objeto={objeto} motivo=navegacion')
                motivo_fetch = 'navegacion'
                return

            # 3) Búsqueda activa del objeto.
            self._status(f'SEARCHING clase={clase}')
            self._det_event.clear()
            self._pub_cmd(f'buscar:{clase}')
            res = self._esperar_det(BUSQUEDA_TIMEOUT)
            if self._cancelar.is_set():
                return self._abortar(objeto)
            encontrado = bool(res and res.startswith('encontrado:'))

            if encontrado:
                # 4) Pedir al auxiliar y re-preguntar cada CONFIRM_TIMEOUT s hasta
                # oír "sí" (también si responde "no" o calla), con tope de seguridad.
                self._status('WAITING_CONFIRM')
                while (not self._cancelar.is_set()
                       and confirm_intentos < CONFIRM_MAX_INTENTOS):
                    self._hablar(frase_pedir, 'calma', 0.5)
                    confirm_intentos += 1
                    # escuchar_confirmacion bloquea CONFIRM_TIMEOUT s => es el cooldown.
                    if self._node.escuchar_confirmacion(CONFIRM_TIMEOUT):
                        entregado = True
                        break
                    # 'no' o silencio: se vuelve a preguntar en la siguiente vuelta.
                if not entregado and not self._cancelar.is_set():
                    motivo_fetch = 'sin_confirmacion'
                    self._hablar('Lo dejo por ahora, vuelvo sin ello.', 'calma', 0.5)
            else:
                motivo_fetch = 'no_encontrado'
                self._hablar(f'No he conseguido encontrar {objeto}, '
                             'vuelvo contigo.', 'preocupacion', 0.6)

            # 5) Volver a la pose de origen (con el usuario).
            self._status('RETURNING')
            self._nav_event.clear()
            self._pub_cmd('volver_a_origen')
            res = self._esperar_nav(NAV_TIMEOUT)
            if self._cancelar.is_set():
                return self._abortar(objeto)

            # 6) Entrega / cierre.
            if entregado:
                self._hablar(frase_entrega, 'alegria', 0.5)
            self._status(f'COMPLETED objeto={objeto} entregado={entregado}')
        except Exception as e:
            self._log.error(f'Error en flujo FETCH: {e}')
            self._status(f'FAILED objeto={objeto} motivo=excepcion')
            motivo_fetch = 'excepcion'
        finally:
            if self._cancelar.is_set() and not entregado:
                motivo_fetch = motivo_fetch or 'cancelada'
            try:
                self._node._logger_sesion.evento(
                    'tarea_fetch', objeto=objeto, entregado=entregado,
                    intentos=confirm_intentos, motivo=motivo_fetch)
            except Exception:
                pass
            with self._lock:
                self._activa = False

    def _abortar(self, objeto):
        self._log.info('Tarea FETCH cancelada.')
        self._status(f'FAILED objeto={objeto} motivo=cancelada')
