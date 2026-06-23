"""
asistente_node — Nodo fusionado: voz, diálogo, movimiento y emergencias.

Fusiona en un único proceso Python lo que antes eran 4 nodos separados
(voz, diálogo, movimiento y emergencias). Reduce contención de RAM/CPU en
RPi 4B y elimina 3 hops DDS para los comandos rápidos.

Pipeline interno:
    arecord (subprocess) ──► Vosk streaming ──► parser
                                                 │
                                                 ▼
                                       _dispatch(comando)
                                       ├── movimiento (Twist, dock/undock)
                                       ├── emergencias (Twilio)
                                       └── diálogo (Piper TTS + LLM)

Topics que se mantienen como ROS (interfaz con nodos externos):
  pub  /voice_command     → object_detector_node, nodo_navegacion_node
  pub  /voice_text        → debug
  pub  /tts_activo (Bool) → object_detector_node (silenciar visión TTS)
  pub  /cmd_vel (Twist)   → base TurtleBot4
  sub  /detected_objects  ← object_detector_node (para hablarlos)
  sub  /navegacion_estado ← nodo_navegacion_node (para hablarlo)
  sub  /dock_status       ← Create 3
  action clients /dock, /undock

Subsistemas extraídos (en proceso, ver coordinador_tarea.py / gestor_emergencia.py):
  CoordinadorTarea (FETCH): pub /task/status; sub /navegacion_resultado,
                            /deteccion_resultado.
  GestorEmergencia:         pub /emergency/status (+ Twilio).

ADVERTENCIA: las credenciales Twilio se leen de variables de entorno
(TWILIO_SID, TWILIO_TOKEN, TWILIO_FROM, TWILIO_TO). Rotar si llegaron
a un repo público (console.twilio.com).
"""

import atexit
import audioop
import json
import os
import queue
import re
import signal
import subprocess
import socket
import threading
import time
import unicodedata
from datetime import datetime

from .coordinador_tarea import CoordinadorTarea
from .gestor_emergencia import GestorEmergencia
from .metricas_logger import SesionLogger
from .monitor_sistema import MonitorSistema

import numpy as np
if not np.__version__.startswith('1.'):
    raise RuntimeError(
        f'NumPy {np.__version__} detectado. cv_bridge de ROS Jazzy requiere '
        f'NumPy 1.x. Ejecuta:\n'
        f'  pip3 uninstall -y numpy && '
        f'pip3 install --user --break-system-packages --force-reinstall '
        f'numpy==1.26.4\n'
        f'o relanza install.sh.')

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Empty, String, UInt8MultiArray
from std_srvs.srv import Trigger

from irobot_create_msgs.action import Dock, Undock
from irobot_create_msgs.msg import DockStatus

# =====================================================================
# Constantes
# =====================================================================

# --- Audio ---
TASA = 16000
CHUNK_SEG = 0.1
CHUNK_BYTES = int(TASA * CHUNK_SEG) * 2
CHUNK_SMALL = int(TASA * 0.02) * 2  # 640 bytes = 20ms — sub-chunk para bridge en emergencia
MAX_GRABACION_SEG = 5
UMBRAL_RMS_NORMAL = 90
UMBRAL_RMS_BARGE_IN_MULT = 2.0  # umbral durante TTS = NORMAL × esto (filtra eco)
SILENCIO_FIN = 7  # 7 × 0.1 s = 700 ms

# --- Bridge de emergencia (UDP IPC con bridge_twilio_emergencia.py) ---
# La señal "COHERENT" (puerto 9999) la envía gestor_emergencia, no este archivo.
BRIDGE_AUDIO_PORT = 9998     # asistente_node → bridge: chunks PCM S16_LE 16kHz

# --- Vosk ---
VOSK_MODEL_PATH = os.path.join(
    os.path.expanduser('~'),
    'asistente_turtlebot4-main/models/vosk-model-small-es-0.42')

# --- Parser ---
WAKE_WORD = 'ana'
TIMEOUT_CONVERSACION = 25.0
POST_TTS_MUTE_SEC = 1.8

# --- Emergencia (CASO A: petición explícita confirmada en código) ---
# Ventana en la que un "sí" cuenta como confirmación de la llamada pedida. Pasado
# este tiempo el flag caduca, para que un "sí" tardío (dirigido a otra cosa) no
# dispare una llamada de emergencia por error.
PENDIENTE_EMERGENCIA_TIMEOUT = 30.0

# --- Red de seguridad CASO B: intención verbal de llamar a emergencias ---
# Si el LLM DICE que está llamando a emergencias pero se olvida del token
# <action>emergency:auto</action> (el 3B es poco fiable con etiquetas), detectamos
# la intención en el texto hablado y disparamos igual. EMERGENCIA_NEG es la guarda
# contra negaciones ("no voy a llamar a emergencias").
EMERGENCIA_INTENT = re.compile(
    r'(realizando|haciendo|voy a (hacer|realizar)|estoy (haciendo|realizando))'
    r'.{0,30}llamada de emergencia'
    r'|llamando a (emergencias|urgencias)'
    r'|(aviso|avisando|voy a avisar) a (emergencias|urgencias)', re.I)
EMERGENCIA_NEG = re.compile(
    r'\bno\b.{0,25}(voy a |puedo |llamo|llamar|hacer la llamada|avisar)', re.I)

MAPA_COMANDOS = [
    # Emergencia (CASO A): petición explícita del usuario. Frases multi-palabra
    # para no dispararse con menciones casuales de "emergencia". El texto ya llega
    # en minúsculas y sin acentos cuando se compara, así que "medico" sin tilde.
    ('llama a emergencias', 'llamar_emergencias'),
    ('llamar a emergencias', 'llamar_emergencias'),
    ('llama emergencias', 'llamar_emergencias'),
    ('avisa a emergencias', 'llamar_emergencias'),
    ('llama a urgencias', 'llamar_emergencias'),
    ('llama una ambulancia', 'llamar_emergencias'),
    ('necesito una ambulancia', 'llamar_emergencias'),
    ('necesito ayuda urgente', 'llamar_emergencias'),
    ('llama al medico', 'llamar_emergencias'),
    ('activa la emergencia', 'llamar_emergencias'),
    ('volver a base', 'volver_a_base'),
    ('vuelve a base', 'volver_a_base'),
    ('no sabes donde estas', 'relocalizar'),
    ('relocalizate', 'relocalizar'),
    ('ubicate', 'relocalizar'),
    ('que estas viendo', 'ver'),
    ('a tu alrededor', 'ver'),
    ('que ves', 'ver'),
    ('que hay', 'ver'),
    ('que es lo que ves', 'ver'),
    ('mira ahi', 'ver'),
    ('adelante', 'adelante'),
    ('desacoplar', 'desacoplar'),
    ('acoplar', 'acoplar'),
    ('repetir', 'repetir'),
    ('sacudir', 'sacudir'),
    ('atras', 'atras'),
    ('izquierda', 'izquierda'),
    ('derecha', 'derecha'),
    ('girar', 'girar'),
    ('parar', 'parar'),
    ('mira', 'ver'),
]

# --- OpenRouter ---
OPENROUTER_BASE_URL = 'https://openrouter.ai/api/v1'
#LLM_MODEL = 'meta-llama/llama-3.2-3b-instruct'
LLM_MODEL = 'google/gemini-2.5-flash-lite'
MAX_TOKENS = 100
TEMPERATURE = 0.5            # bajado de 0.7 para reducir divagación/repetición del 3B
FREQUENCY_PENALTY = 0.6     # penaliza repetir los mismos tokens (anti bucle)
PRESENCE_PENALTY = 0.3      # empuja a introducir vocabulario nuevo
HISTORIAL_MAX_PARES = 3
HISTORIAL_TIMEOUT = 30

# --- TTS flush (optimizado para conexión lenta) ---
FLUSH_PUNCT = '.!?'
FLUSH_MIN_CHARS = 60    # esperar al menos N chars antes de flush por puntuación
FLUSH_HARD_CAP_CHARS = 400  # red de seguridad: forzar flush aunque no haya punto

CONTEXT_PATH = os.path.join(
    os.path.expanduser('~'), 'asistente_turtlebot4-main', 'contexto_LLM.txt')

# Directorio de logs de sesión (un fichero JSONL + resumen por ejecución).
# `log/` ya lo usa colcon, así que usamos uno propio.
LOGS_SESIONES_DIR = os.path.join(
    os.path.expanduser('~'), 'asistente_turtlebot4-main', 'logs_sesiones')

# --- Piper TTS ---
PIPER_MODEL = os.path.join(
    os.path.expanduser('~'),
    'asistente_turtlebot4-main/models/piper/es_MX-ald-medium.onnx')
    #'asistente_turtlebot4-main/models/piper/es_AR-daniela-high.onnx')
EMOCION_DEFAULT_TIPO = 'calma'
EMOCION_DEFAULT_INTENSIDAD = 0.5
PAUSA_COMA_MS  = 120   # silencio tras coma / punto y coma / dos puntos
PAUSA_PUNTO_MS = 200   # silencio al final de cada frase sintetizada

MSG_SIN_WIFI = (
    'Lo siento creo que me estoy resfriando, '
    'no consigo conectarme al WIFI')
WIFI_CHECK_INTERVAL = 3.0   # segundos entre comprobaciones de WiFi

# Watchdog TTS remoto: si Ana está "hablando" pero no llega audio del servidor
# remoto en este tiempo, se asume que el servidor cayó y se fuerza recuperación
# (evita que _frases_pendientes se quede atascado y Ana enmudezca).
REMOTE_TTS_TIMEOUT = 8.0

# Dedup de /tts_audio: si el seq del servidor retrocede MÁS que esto, se asume
# reinicio del servidor (rebase) en vez de copia duplicada. Un duplicado de la
# malla Zenoh siempre llega como mucho una frase por detrás del último seq, así
# que el umbral separa "copia" de "servidor reiniciado" sin depender de cuántas
# frases lleve la sesión.
AUDIO_DEDUP_BACKJUMP = 5000

# --- Guards semánticos de acciones LLM ---
# Subconjuntos de keywords que indican intención real de movimiento / pickup.
# Si el texto del usuario del turno NO contiene ninguna, la acción se descarta.
_INTENT_GOTO_KW = frozenset({
    'ir a', 've a', 'llevame', 'lleva', 'navega', 'muevete', 'acercate',
    'voy', 'salon', 'cocina', 'habitacion', 'cama', 'mesa',
    'vuelve', 'regresa', 'ven aqui',
})
_INTENT_PICKUP_KW = frozenset({
    'trae', 'traeme', 'busca', 'recoge', 'coge', 'toma', 'dame',
    'quiero', 'necesito', 'cafe', 'agua', 'libro', 'mando', 'movil',
})

# Guard emergencia CASO B: sólo se ejecuta si hay keyword de riesgo físico.
# Una ruptura sentimental o estrés académico NO lo cumple.
EMERGENCY_FISICA_KW = re.compile(
    r'dolor|duele|ca[íi]da|ca[íi]do|sangre|herida|inconsciente|'
    r'respira|pecho|mareo|desmay|accidente|golpe|fractura|quemadura|'
    r'asfixia|no (me )?puedo mover|socorro|ayuda urgente',
    re.IGNORECASE
)

# Tags de emoción que el 3B emite como wrapper <calma>…</calma> en vez del
# formato canónico <emocion tipo="calma"…/>. Se limpian en _parsear_respuesta.
_EMOTION_TYPES = (
    'calma', 'alegria', 'empatia', 'preocupacion',
    'sorpresa', 'entusiasmo', 'urgencia', 'confusion',
)
_EMOTION_WRAPPER_RE = re.compile(
    r'</?(' + '|'.join(_EMOTION_TYPES) + r')\b[^>]*>',
    re.IGNORECASE,
)

# --- TTS remoto (sintesis en PC) ---
TTS_SAMPLE_RATE = 22050  # rate de es_MX-ald-medium y daniela-high
TTS_QOS = QoSProfile(
    depth=50,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

# --- Recuperación audio ---
USB_HUB_PORT = '1-1.3'
ALSA_CARD_MIC = 'SF558'         # micrófono real (sólo captura)
ALSA_CARD_DAC = 'Headset'       # altavoz (sólo reproducción)
ALSA_CARD_FALLBACK = 'Headphones'
ESPERA_REENUMERACION = 5

# --- Movimiento ---
DURACION_MOVIMIENTO = 2.0

QOS_SENSOR = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

# QoS latcheado para /emergency/active: los nodos que pausamos (object_detector,
# navegacion) reciben el estado actual aunque se suscriban tarde, y el False final
# (reanudar) se entrega de forma fiable aunque el suscriptor parpadee.
QOS_LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

# Servicios para pausar recursos externos durante la emergencia (best-effort).
SRV_CAMARA_STOP = '/oakd/stop_camera'
SRV_CAMARA_START = '/oakd/start_camera'
SRV_NAV2_LIFECYCLE = (
    '/lifecycle_manager_navigation/manage_nodes',
    '/lifecycle_manager_localization/manage_nodes',
)

RESPUESTAS_RAPIDAS = {
    'adelante': 'Adelante.',
    'atras': 'Atras.',
    'izquierda': 'Izquierda.',
    'derecha': 'Derecha.',
    'parar': 'Parado.',
    'girar': 'Giro.',
    'acoplar': 'Acoplando.',
    'desacoplar': 'Desacoplando.',
    'repetir': 'Repito.',
    'sacudir': 'Sacudida.',
    'wake': 'Dime.',
    'ver': 'Mirando.',
}

EMOCION_RAPIDA = {
    'adelante':   ('entusiasmo', 0.5),
    'atras':      ('calma',      0.4),
    'izquierda':  ('calma',      0.4),
    'derecha':    ('calma',      0.4),
    'parar':      ('calma',      0.3),
    'girar':      ('entusiasmo', 0.4),
    'acoplar':    ('calma',      0.5),
    'desacoplar': ('calma',      0.4),
    'repetir':    ('calma',      0.3),
    'sacudir':    ('entusiasmo', 0.7),
    'wake':       ('alegria',    0.5),
    'ver':        ('sorpresa',   0.4),
}

# --- Emergencias --- toda la lógica Twilio (credenciales, cooldown, reintentos,
# TwiML dinámico, waypoint, /emergency/status) vive en gestor_emergencia.py.

# Sentinel: marca fin de frase en la cola de audio
FRASE_END = object()


# =====================================================================
# Nodo fusionado

class AsistenteNode(Node):
    """Nodo fusionado: STT, TTS, movimiento y emergencias."""

    def __init__(self):
        """Inicializa subsistemas de audio, TTS, LLM y ROS."""
        super().__init__('asistente_node')
        self._mute_stt_until = 0.0

        # ---- Publishers / Subscribers (sólo interfaz externa) ----
        self.pub_comando = self.create_publisher(String, '/voice_command', 10)
        self.pub_texto = self.create_publisher(String, '/voice_text', 10)
        self.pub_tts_activo = self.create_publisher(Bool, '/tts_activo', 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel_unstamped', 10)
        # Señal de "modo emergencia activo": object_detector libera YOLO y
        # navegacion cancela/ignora mientras dure la llamada (ver _pausar/_reanudar).
        self.pub_emergencia_activa = self.create_publisher(
            Bool, '/emergency/active', QOS_LATCHED)
        # Clientes de servicio para pausar recursos externos durante la emergencia.
        # Se crean aquí (antes del spin) por seguridad de hilos; las llamadas son
        # best-effort con wait_for_service, así que da igual que el servicio no
        # exista todavía (cámara/Nav2 arrancan de forma asíncrona).
        self._cli_cam_stop = self.create_client(Trigger, SRV_CAMARA_STOP)
        self._cli_cam_start = self.create_client(Trigger, SRV_CAMARA_START)
        # Nav2 es opcional: guardamos el import por si nav2_msgs no está instalado.
        try:
            from nav2_msgs.srv import ManageLifecycleNodes
            self._ManageLifecycleNodes = ManageLifecycleNodes
            self._cli_nav2 = [
                (s, self.create_client(ManageLifecycleNodes, s))
                for s in SRV_NAV2_LIFECYCLE]
        except Exception:
            self._ManageLifecycleNodes = None
            self._cli_nav2 = []

        self.create_subscription(
            String, '/detected_objects', self._cb_objetos, 10)
        self.create_subscription(
            String, '/navegacion_estado', self._cb_navegacion, 10)
        # Métricas de validación de navegación (Tabla 2.17): nodo_navegacion las
        # publica como JSON; aquí se vuelcan al log de sesión.
        self.create_subscription(
            String, '/navegacion_metricas', self._cb_navegacion_metrica, 10)
        self.create_subscription(DockStatus, '/dock_status',
                                 self._cb_dock_status, QOS_SENSOR)

        # ---- Action clients dock / undock ----
        self.cliente_acoplar = ActionClient(self, Dock, '/dock')
        self.cliente_desacoplar = ActionClient(self, Undock, '/undock')
        self.esta_acoplado = None
        self.acoplando = False
        self.desacoplando = False
        # ---- Estado movimiento ----
        self.twist_actual = Twist()
        self.en_movimiento = False
        self.timer_parar = None
        self.timer_sacudida = None
        self.secuencia_sacudida = []
        self.indice_sacudida = 0
        self.create_timer(0.1, self._publicar_movimiento)

        # ---- Estado voz / conversación ----
        self._tts_event = threading.Event()
        self._en_conversacion = False
        self._ultimo_input = 0.0
        self.create_timer(2.0, self._check_timeout)

        # ---- Estado diálogo ----
        self._hablando = False
        self._historial = []
        self._ultimo_intercambio = 0.0
        try:
            with open(CONTEXT_PATH, 'r', encoding='utf-8') as f:
                self._system_prompt = f.read()
        except FileNotFoundError:
            self.get_logger().error(f'No se encontró {CONTEXT_PATH}')
            self._system_prompt = ''

        # Cliente OpenRouter
        try:
            from openai import OpenAI
            self.client = OpenAI(
                base_url=OPENROUTER_BASE_URL,
                api_key=os.environ.get('OPENROUTER_API_KEY', ''),
            )
        except ImportError:
            self.get_logger().error('openai no instalado. pip install openai')
            self.client = None

        # ---- Confirmación por voz (la usa el coordinador de tareas FETCH) ----
        self._esperando_confirmacion = threading.Event()
        self._confirmacion_event = threading.Event()
        self._resultado_confirmacion = False

        # ---- Emergencia pendiente de confirmación (CASO A, en código) ----
        # Se arma cuando el usuario pide explícitamente la llamada; el siguiente
        # "sí" detectado en _procesar_texto la dispara, sin pasar por el LLM.
        self._emergencia_pendiente = threading.Event()
        self._t_emergencia_pendiente = 0.0

        # ---- Modo emergencia: bloquea dispatch normal pero mantiene STT+LLM ----
        # El GestorEmergencia lo activa al iniciar la llamada y lo baja al terminar.
        self._modo_emergencia = threading.Event()
        # Durante la llamada de emergencia el bridge (otro proceso) necesita el
        # altavoz para reproducir al operador. El dispositivo ALSA hw es EXCLUSIVO:
        # si nuestro aplay persistente lo mantiene abierto, el aplay del bridge
        # falla con "Device or resource busy" y no suena nada. Con este evento
        # cedemos el altavoz: _abrir_aplay deja de reabrirlo y _hilo_reproduccion
        # lo cierra para liberar el hw mientras dura la llamada.
        self._ceder_altavoz = threading.Event()
        # Socket UDP para enviar audio PCM y señales de coherencia al bridge.
        self._bridge_udp_sock = None

        # ---- Texto del turno actual (guard semántico de acciones LLM) ----
        # Capturado en _llm_responder antes de llamar a _ejecutar_accion.
        self._texto_usuario_turno = ''

        # ---- TTS pipeline: dos hilos (síntesis + reproducción) ----
        self._voice = None
        self._tts_queue = queue.Queue()
        self._audio_queue = queue.Queue()
        # Dedup del stream /tts_audio: el servidor sella un contador monotónico
        # de sesión en layout.data_offset. Una malla Zenoh redundante puede
        # entregar cada mensaje varias veces (mismo seq); aceptamos sólo seq
        # crecientes y descartamos las copias, así Ana reproduce cada frase 1×.
        # NO se resetea en barge-in: el contador del servidor es global y
        # monotónico, y resetear reabriría copias rezagadas del turno anterior.
        self._last_audio_seq = 0
        self._tts_listo = threading.Event()
        self._cancelar_llm = threading.Event()
        # Generación de turno: cada turno LLM captura su id al arrancar; si
        # _gen_llm avanza (nuevo turno o barge-in), el hilo viejo queda muerto
        # PERMANENTEMENTE, sin que el clear con timer de _cancelar_llm lo resucite.
        self._gen_llm = 0
        self._lock_gen = threading.Lock()
        self._frases_pendientes = 0
        self._lock_frases = threading.Lock()
        self._aplay_eta = 0.0
        self._aplay_proc = None
        self._usando_fallback = False
        self._contador_fallback = 0
        self._t_ultimo_audio_tts = 0.0  # watchdog TTS remoto

        # ---- TTS remoto (sintesis en PC) ----
        self._pub_tts_request = self.create_publisher(
            String, '/tts_request', TTS_QOS)
        self._pub_tts_cancel = self.create_publisher(
            Empty, '/tts_cancel', TTS_QOS)
        self.create_subscription(
            UInt8MultiArray, '/tts_audio', self._cb_tts_audio, TTS_QOS)

        # ---- Estado offline / WiFi ----
        self._wifi_disponible = True      # False cuando el monitor detecta caída
        self._tts_remoto_ok = True        # False tras primera frase sin servidor remoto
        self._anunciado_sin_wifi = False  # True después del primer aviso por turno de corte

        threading.Thread(target=self._hilo_sintesis, daemon=True).start()
        threading.Thread(target=self._hilo_reproduccion, daemon=True).start()
        threading.Thread(target=self._monitor_wifi, daemon=True).start()

        # ---- Speech queue: worker único con backpressure ----
        self._speech_queue = queue.Queue(maxsize=6)
        threading.Thread(target=self._speech_worker, daemon=True).start()


        # ---- Métricas por turno ----
        self._t_stt_final = 0.0
        self._t_llm_primer_token = 0.0
        self._t_tts_primer_byte = 0.0
        self._t_tts_ultimo_byte = 0.0
        self._metrics_n_chars = 0
        self._metrics_speech_q_max = 0
        self._metrics_tts_q_max = 0
        self._metrics_audio_q_max = 0

        # ---- Logger de sesión (JSONL + resumen por ejecución) ----
        self._logger_sesion = SesionLogger(LOGS_SESIONES_DIR, llm_model=LLM_MODEL)
        # Marca de fin del turno anterior, para medir el gap entre acciones.
        self._t_ultimo_turno_fin = 0.0

        # ---- Monitor de carga del sistema (CPU/RAM/temp/load de la RPi 4B) ----
        # Muestrea cada 5 s y lo registra como eventos 'sistema' para detectar
        # saturación durante la sesión. Sin psutil (lee /proc y /sys).
        self._monitor_sistema = MonitorSistema(self._registrar_sistema, periodo=5.0)
        self._monitor_sistema.start()

        # ---- Subsistemas extraídos (corren en proceso, reusan TTS/STT) ----
        self._gestor_emergencia = GestorEmergencia(self)
        self._coordinador = CoordinadorTarea(self)

        # ---- STT: Vosk en hilo (carga lazy) ----
        self._recognizer = None
        self._vosk_listo = threading.Event()
        threading.Thread(target=self._cargar_vosk, daemon=True).start()

        # ---- Detectar mic y arrancar captura ----
        self.dispositivo_mic = self._detectar_dispositivo()
        self._arecord_proc = None
        threading.Thread(target=self._bucle_escucha, daemon=True).start()

        self.get_logger().info('asistente_node listo (fusionado).')

    def destroy_node(self):
        """Termina subprocesos arecord/aplay antes de destruir el nodo."""
        try:
            self._monitor_sistema.stop()
        except Exception:
            pass
        try:
            self._logger_sesion.cerrar()
        except Exception:
            pass
        for proc in (self._arecord_proc, self._aplay_proc):
            if proc is not None:
                try:
                    proc.terminate()
                    proc.wait(timeout=1)
                except Exception:
                    try:
                        proc.kill()
                    except Exception:
                        pass
        super().destroy_node()

    # ==================================================================
    # Sincronización TTS / mute
    # ==================================================================

    def _set_tts_activo(self, activo):
        """Mute interno + publicación a /tts_activo (para object_detector)."""
        if activo:
            self._tts_event.set()
        else:
            self._tts_event.clear()
        msg = Bool()
        msg.data = activo
        self.pub_tts_activo.publish(msg)

    # ==================================================================
    # Carga Vosk
    # ==================================================================

    def _cargar_vosk(self):
        """Carga el modelo Vosk en un hilo daemon."""
        try:
            from vosk import Model, KaldiRecognizer, SetLogLevel
            SetLogLevel(-1)
            model = Model(VOSK_MODEL_PATH)
            self._recognizer = KaldiRecognizer(model, TASA)
            self._recognizer.SetWords(False)
            self._vosk_listo.set()
            self.get_logger().info('Vosk cargado.')
        except Exception as e:
            self.get_logger().error(f'Vosk: {e}')

    # ==================================================================
    # Detección de micrófono
    # ==================================================================

    def _detectar_dispositivo(self):
        """Busca el dispositivo de captura ALSA correcto por nombre de tarjeta."""
        try:
            salida = subprocess.run(
                ['arecord', '-l'], capture_output=True, text=True
            ).stdout
        except FileNotFoundError:
            return 'default'

        # Extraer nombres cortos de tarjeta (más estables que los números)
        nombres = re.findall(r'card \d+:\s+(\w+)', salida)
        self.get_logger().info(f'Tarjetas de captura encontradas: {nombres}')

        # Priorizar el micro configurado (SF558) y EXCLUIR el altavoz (DAC):
        # el G430 expone endpoint de captura, así que su prueba de 1 s pasa pero
        # graba silencio. Sin esto, _detectar_dispositivo elegía el altavoz como
        # micro y Vosk nunca oía nada. 'default' como último recurso.
        preferidos = [n for n in nombres if n == ALSA_CARD_MIC]
        otros = [n for n in nombres if n not in (ALSA_CARD_MIC, ALSA_CARD_DAC)]
        candidatos = [f'plughw:CARD={n},DEV=0' for n in preferidos + otros]
        candidatos.append('default')

        for disp in candidatos:
            r = subprocess.run(
                ['arecord', '-D', disp, '-f', 'S16_LE', '-r', '16000',
                 '-c', '1', '-t', 'wav', '-d', '1', '/tmp/_probe.wav'],
                capture_output=True
            )
            if r.returncode == 0:
                self.get_logger().info(f'Mic seleccionado: {disp}')
                return disp
        return 'default'

    # ==================================================================
    # Timeout de conversación
    # ==================================================================

    def _check_timeout(self):
        """Timer callback: watchdog TTS remoto + cierre por inactividad."""
        # Watchdog: si Ana lleva hablando sin que llegue audio (servidor remoto
        # caído tras suscribirse), nunca llegaría FRASE_END y _hablando quedaría
        # atascado. Forzar recuperación reutilizando _interrumpir_tts.
        ahora = time.time()
        with self._lock_frases:
            atascado = (self._hablando and self._frases_pendientes > 0
                        and self._t_ultimo_audio_tts > 0.0
                        and ahora - self._t_ultimo_audio_tts > REMOTE_TTS_TIMEOUT)
        if atascado:
            self.get_logger().warn(
                f'Watchdog TTS: sin audio en {REMOTE_TTS_TIMEOUT:.0f}s, '
                'forzando recuperación.')
            self._interrumpir_tts()
            # Las colas suelen estar vacías (servidor caído), así que el drenado
            # de _interrumpir_tts no bajaría el contador: forzarlo a 0.
            with self._lock_frases:
                self._frases_pendientes = 0
            return

        if not self._en_conversacion or self._tts_event.is_set():
            return
        if ahora - self._ultimo_input >= TIMEOUT_CONVERSACION:
            self._en_conversacion = False
            self.get_logger().info('Conversacion expirada, esperando "ana".')
            self._logger_sesion.evento('conversacion', estado='expirada')

    # ==================================================================
    # Parser
    # ==================================================================

    @staticmethod
    def _quitar_acentos(texto):
        """Elimina diacríticos para comparación robusta."""
        nfkd = unicodedata.normalize('NFKD', texto)
        return ''.join(c for c in nfkd if unicodedata.category(c) != 'Mn')

    def _buscar_comando(self, texto):
        """Devuelve el primer comando que aparece en el texto, o None."""
        for frase, cmd in MAPA_COMANDOS:
            if frase in texto:
                return cmd
        return None

    def _procesar_texto(self, texto_raw):
        """TRIPLE BARRERA ANTI-ECO: mute STT + timestamp + _hablando flag."""
        # En modo emergencia: no dispatch normal. Solo evaluar coherencia para el bridge.
        if self._modo_emergencia.is_set():
            self._check_coherencia_emergencia(texto_raw.strip())
            return

        if self._hablando:
            self.get_logger().debug(
                f'STT IGNORADO: Ana hablando ({repr(texto_raw[:30])})')
            return

        ahora = time.time()
        if ahora < self._mute_stt_until:
            margen = self._mute_stt_until - ahora
            self.get_logger().debug(
                f'STT IGNORADO: mute post-TTS ({margen:.2f}s)')
            return

        texto = self._quitar_acentos(texto_raw.lower().strip())
        texto_limpio = texto.strip('.,;:¿?¡! ')

        # Confirmación de emergencia EN CÓDIGO (CASO A): si está armada por una
        # petición explícita del usuario, el sí/no se decide aquí sin pasar por el
        # LLM. Así la llamada no depende de que el 3B emita el token de acción.
        # VA ANTES del bloque _esperando_confirmacion: la emergencia es prioritaria
        # sobre cualquier otra tarea en curso (FETCH, etc.) para que el "sí" no sea
        # robado por el coordinador de tareas.
        if self._emergencia_pendiente.is_set():
            self.get_logger().info(f'[EMG] texto pendiente confirmacion: "{texto_limpio}"')
            if time.time() - self._t_emergencia_pendiente > PENDIENTE_EMERGENCIA_TIMEOUT:
                # Caducada: el usuario no respondió a tiempo. Limpiamos el flag y
                # dejamos que este texto se procese como entrada normal.
                self._emergencia_pendiente.clear()
                self.get_logger().info('Confirmacion de emergencia caducada.')
            elif self._es_afirmativo(texto_limpio):
                self._emergencia_pendiente.clear()
                self.get_logger().info(
                    f'Emergencia CONFIRMADA por voz (código): "{texto_limpio}".')
                self._gestor_emergencia.activar('peticion')  # CASO A → va al salón
                return
            elif self._es_negativo(texto_limpio):
                self._emergencia_pendiente.clear()
                self.get_logger().info(
                    f'Emergencia CANCELADA por el usuario: "{texto_limpio}".')
                self._hablar('Vale, no llamo a nadie.', 'calma', 0.5)
                return
            else:
                self.get_logger().warn(
                    f'[EMG] confirmacion no reconocida: "{texto_limpio}". '
                    'es_afirmativo=False, es_negativo=False. Repitiendo pregunta.')
                self._hablar('Necesito un sí o un no. ¿Llamo a emergencias?',
                             'urgencia', 0.8)
                return

        # Captura de confirmación por voz para el coordinador FETCH: si está
        # esperando un sí/no del auxiliar, interceptamos aquí y NO pasamos por el
        # dispatcher (no es un comando de Ana, es la respuesta a su pregunta).
        # Va DESPUÉS de _emergencia_pendiente para que la emergencia tenga prioridad.
        if self._esperando_confirmacion.is_set():
            self._resultado_confirmacion = self._es_afirmativo(texto_limpio)
            self.get_logger().info(
                f'Confirmacion auxiliar: "{texto_limpio}" -> '
                f'{self._resultado_confirmacion}')
            self._confirmacion_event.set()
            return

        msg_txt = String()
        msg_txt.data = texto_raw.lower()
        self.pub_texto.publish(msg_txt)

        self._cancelar_llm.clear()
        if WAKE_WORD in texto:
            if not self._en_conversacion:
                self._en_conversacion = True
                self.get_logger().info('Modo conversacion: ON')
                self._logger_sesion.evento('conversacion', estado='on')
            self._ultimo_input = time.time()

            idx = texto.index(WAKE_WORD) + len(WAKE_WORD)
            resto = texto[idx:].strip().strip('.,;:¿?¡! ')
            
            if not resto:
                self._dispatch('wake', texto_raw)
            else:
                comando = self._buscar_comando(resto)
                if comando:
                    self._dispatch(comando, texto_raw)
                else:
                    self._dispatch(f'pregunta:{texto}', texto_raw)
            return

        if self._en_conversacion:
            self._ultimo_input = time.time()
            comando = self._buscar_comando(texto_limpio)
            if comando:
                self._dispatch(comando, texto_raw)
            else:
                self._dispatch(f'pregunta:{texto}', texto_raw)

    # ==================================================================
    # Captura audio + Vosk streaming
    # ==================================================================

    def _bucle_escucha(self):
        """Bucle de captura: reinicia arecord y redetecta mic+altavoz si muere."""
        self._vosk_listo.wait()
        self._mute_microfono(False)  # garantiza switch ALSA activo al arrancar
        self.get_logger().info(
            f'Escucha iniciada (mic={self.dispositivo_mic}).')

        while rclpy.ok():
            proc = None
            try:
                proc = subprocess.Popen([
                    'arecord', '-D', self.dispositivo_mic,
                    '-f', 'S16_LE', '-r', str(TASA), '-c', '1', '-t', 'raw',
                ], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
                self._arecord_proc = proc
                self._stream_vosk(proc)
            except Exception as e:
                self.get_logger().error(f'arecord: {e}')
            finally:
                if proc is not None:
                    try:
                        proc.terminate()
                        proc.wait()
                    except Exception:
                        pass
                self._arecord_proc = None

            if not rclpy.ok():
                break
            self.get_logger().warn(
                'arecord terminó; redetectando mic en 2s...')
            time.sleep(2)                   # margen para reenumeración USB
            self.dispositivo_mic = self._detectar_dispositivo()

    def _stream_vosk(self, proc):
        """Lee chunks de arecord y los procesa con Vosk."""
        grabando = False
        n_silencio = 0
        max_chunks = int(MAX_GRABACION_SEG / CHUNK_SEG)
        n_chunks = 0
        chunks_desde_parcial = 0

        while rclpy.ok():
            # En emergencia: leer en sub-chunks de 20ms y reenviar al bridge cada uno
            # inmediatamente (latencia ~20ms en vez de ~100ms). Se acumula un chunk
            # completo de 100ms para Vosk, que no ve diferencia.
            if self._modo_emergencia.is_set():
                data = b''
                while len(data) < CHUNK_BYTES and rclpy.ok():
                    piece = proc.stdout.read(CHUNK_SMALL)
                    if not piece or len(piece) < CHUNK_SMALL:
                        data = None
                        break
                    if self._bridge_udp_sock is not None:
                        try:
                            self._bridge_udp_sock.sendto(
                                piece, ('127.0.0.1', BRIDGE_AUDIO_PORT))
                        except Exception:
                            pass
                    data += piece
            else:
                data = proc.stdout.read(CHUNK_BYTES)

            if not data or len(data) < CHUNK_BYTES:
                break

            rms = audioop.rms(data, 2)

            if self._tts_event.is_set():
                # Durante el TTS sólo escuchamos barge-in: si el RMS supera con
                # holgura el umbral normal (filtra el eco del propio altavoz) y
                # Vosk reconoce "ana" en el parcial → interrumpir. Si no, se
                # ignora el audio (no alimentamos Vosk con la voz de Ana).
                if grabando:
                    grabando = False
                    self._recognizer.Reset()
                umbral_barge = UMBRAL_RMS_NORMAL * UMBRAL_RMS_BARGE_IN_MULT
                if rms >= umbral_barge:
                    self._recognizer.AcceptWaveform(data)
                    chunks_desde_parcial += 1
                    if chunks_desde_parcial >= 3:
                        chunks_desde_parcial = 0
                        parcial = self._check_parcial()
                        if parcial and WAKE_WORD in parcial:
                            self._interrumpir_tts()
                            self._recognizer.Reset()
                continue

            chunks_desde_parcial = 0

            if not grabando:
                if rms >= UMBRAL_RMS_NORMAL:
                    grabando = True
                    n_silencio = 0
                    n_chunks = 1
                    self._recognizer.Reset()
                    if self._recognizer.AcceptWaveform(data):
                        self._finalizar_reconocimiento()
                        grabando = False
                        continue
            else:
                es_final = self._recognizer.AcceptWaveform(data)
                n_chunks += 1

                if es_final:
                    self._finalizar_reconocimiento()
                    grabando = False
                    continue

                if rms < UMBRAL_RMS_NORMAL:
                    n_silencio += 1
                    if n_silencio >= SILENCIO_FIN:
                        self._finalizar_reconocimiento()
                        grabando = False
                else:
                    n_silencio = 0

                if n_chunks >= max_chunks:
                    self._finalizar_reconocimiento()
                    grabando = False

    def _check_parcial(self):
        """Devuelve el texto parcial actual de Vosk."""
        try:
            parcial_json = self._recognizer.PartialResult()
            return json.loads(parcial_json).get('partial', '')
        except Exception:
            return ''

    def _finalizar_reconocimiento(self):
        """Cierra el turno Vosk y enruta el texto reconocido."""
        try:
            result_json = self._recognizer.FinalResult()
            texto = json.loads(result_json).get('text', '').strip()
        except Exception as e:
            self.get_logger().error(f'Vosk result: {e}')
            self._recognizer.Reset()
            return
        self._recognizer.Reset()
        if not texto:
            return

        if self._hablando:
            self.get_logger().info(
                f'STT DESCARTADO (_hablando=True): "{texto}"')
            return

        self._t_stt_final = time.time()
        self.get_logger().info(f'STT: "{texto}"')
        self._procesar_texto(texto)

    # ==================================================================
    # Confirmación por voz (usada por el coordinador de tareas FETCH)
    # ==================================================================

    def escuchar_confirmacion(self, timeout=12.0):
        """Bloquea hasta captar un sí/no del usuario por STT o agotar timeout.

        Reutiliza el mismo pipeline Vosk del diálogo (ver el hook en
        `_procesar_texto`). Como el micro está silenciado durante el TTS, la
        respuesta del auxiliar llega de forma natural tras la pregunta de Ana.
        Devuelve True si fue afirmativa, False si fue negativa o no llegó.
        """
        self._resultado_confirmacion = False
        self._confirmacion_event.clear()
        self._esperando_confirmacion.set()
        try:
            if not self._confirmacion_event.wait(timeout):
                self.get_logger().info('Confirmacion: timeout sin respuesta.')
                return False
            return self._resultado_confirmacion
        finally:
            self._esperando_confirmacion.clear()

    @staticmethod
    def _es_afirmativo(texto):
        """Heurística sí/no sobre texto ya en minúsculas y sin acentos."""
        palabras = set(texto.replace('.', ' ').replace(',', ' ').split())
        afirmativas = {
            'si', 'claro', 'vale', 'venga', 'hecho', 'listo', 'perfecto',
            'exacto', 'adelante', 'toma', 'dale', 'ok', 'okay', 'correcto',
            'puesto', 'eso', 'sip', 'afirmativo',
            # Frases naturales de confirmación de emergencia
            'confirma', 'confirmado', 'confirmo', 'hazlo', 'procede', 'anda',
        }
        if palabras & afirmativas:
            return True
        for frase in ('eso es', 'ya esta', 'de acuerdo', 'por supuesto',
                      'aqui esta', 'ahi esta', 'ya lo tienes', 'hecho esta'):
            if frase in texto:
                return True
        return False

    @staticmethod
    def _es_negativo(texto):
        """Heurística de negación sobre texto ya en minúsculas y sin acentos."""
        palabras = set(texto.replace('.', ' ').replace(',', ' ').split())
        negativas = {
            'no', 'nada', 'cancela', 'cancelar', 'para', 'deja', 'dejalo',
            'olvidalo', 'olvida', 'negativo', 'nunca', 'tampoco',
        }
        if palabras & negativas:
            return True
        for frase in ('mejor no', 'no llames', 'no hace falta',
                      'no es necesario', 'da igual', 'dejalo estar'):
            if frase in texto:
                return True
        return False

    # ==================================================================
    # Dispatcher central
    # ==================================================================

    def _dispatch(self, comando, texto_raw=''):
        """Punto único: enruta el comando al subsistema correcto."""
        self.get_logger().info(f'"{texto_raw}" -> {comando}')

        # Tiempo desde el fin del turno anterior (tiempo entre acción y acción).
        gap_prev = (round(time.time() - self._t_ultimo_turno_fin, 2)
                    if self._t_ultimo_turno_fin > 0 else None)
        self._logger_sesion.evento(
            'comando', texto=texto_raw, comando=comando, gap_prev=gap_prev)

        msg = String()
        msg.data = comando
        self.pub_comando.publish(msg)

        if comando.startswith('pregunta:'):
            if self._hablando:
                return
            pregunta = comando[len('pregunta:'):].strip()
            if not pregunta:
                self._hablar('Dime.')
                return
            if not self._wifi_disponible:
                if not self._anunciado_sin_wifi:
                    self._anunciado_sin_wifi = True
                    self._hablar(
                        MSG_SIN_WIFI
                        + '. Sin conexion no puedo responder preguntas.')
                else:
                    self._hablar('Sin conexion, no puedo responder preguntas.')
                return
            with self._lock_gen:
                self._gen_llm += 1
                mi_gen = self._gen_llm
            threading.Thread(
                target=self._llm_responder, args=(pregunta, mi_gen),
                daemon=True).start()
            return

        if (comando == 'volver_a_base' or comando == 'relocalizar'
                or comando.startswith('navegar:')):
            # Los gestiona nodo_navegacion_node (ya publicados a /voice_command arriba).
            self._logger_sesion.evento('navegacion', comando=comando)
            return

        if comando == 'llamar_emergencias':
            # CASO A: el usuario la pide. No llamamos aún; armamos la confirmación
            # y el siguiente "sí" (detectado en _procesar_texto, sin LLM) dispara
            # activar('peticion'). Determinista: no depende del token del 3B.
            self._emergencia_pendiente.set()
            self._t_emergencia_pendiente = time.time()
            self._hablar('Voy a llamar a emergencias ahora mismo, ¿lo confirmas?',
                         'urgencia', 0.9)
            return

        self._aplicar_movimiento(comando)

        if comando in RESPUESTAS_RAPIDAS:
            emocion_tipo, emocion_intensidad = EMOCION_RAPIDA.get(
                comando, (EMOCION_DEFAULT_TIPO, EMOCION_DEFAULT_INTENSIDAD))
            self._hablar(RESPUESTAS_RAPIDAS[comando], emocion_tipo, emocion_intensidad)

    # ==================================================================
    # Subsistema MOVIMIENTO
    # ==================================================================

    def _cb_dock_status(self, msg):
        """Actualiza el estado de acoplamiento."""
        anterior = self.esta_acoplado
        self.esta_acoplado = msg.is_docked
        if anterior is None:
            estado = 'acoplado' if msg.is_docked else 'desacoplado'
            self.get_logger().info(f'Dock: {estado}')

    def _aplicar_movimiento(self, comando):
        """Traduce el comando a un Twist o acción de dock."""
        _CMDS_MOVIMIENTO = {'adelante', 'atras', 'izquierda', 'derecha', 'girar', 'sacudir'}
        if self.esta_acoplado and comando in _CMDS_MOVIMIENTO:
            self._hablar("Para moverme primero necesito desacoplarme.")
            return

        self.twist_actual = Twist()
        self.en_movimiento = False

        if comando == 'adelante':
            self.twist_actual.linear.x = 0.1
            self._iniciar_movimiento_temporal()
        elif comando == 'atras':
            self.twist_actual.linear.x = -0.1
            self._iniciar_movimiento_temporal()
        elif comando == 'izquierda':
            self.twist_actual.angular.z = 0.33
            self._iniciar_movimiento_temporal()
        elif comando == 'derecha':
            self.twist_actual.angular.z = -0.33
            self._iniciar_movimiento_temporal()
        elif comando == 'girar':
            self.twist_actual.angular.z = 1.0
            self._iniciar_movimiento_temporal()
        elif comando == 'sacudir':
            self._iniciar_sacudida()
        elif comando == 'parar':
            if self.timer_parar is not None:
                self.timer_parar.cancel()
                self.timer_parar = None
            self._detener_robot()
        elif comando == 'acoplar':
            self._enviar_objetivo_acoplar()
        elif comando == 'desacoplar':
            self._enviar_objetivo_desacoplar()
        elif comando in ('wake', 'ver', 'repetir'):
            pass
        else:
            self.get_logger().debug(f'Movimiento no aplica: {comando}')

    def _iniciar_movimiento_temporal(self):
        """Arranca movimiento con temporizador de parada automática."""
        if self.timer_parar is not None:
            self.timer_parar.cancel()
        self.en_movimiento = True
        self.get_logger().info(
            f'Movimiento iniciado: linear.x={self.twist_actual.linear.x:.2f} '
            f'angular.z={self.twist_actual.angular.z:.2f}'
        )
        self.timer_parar = self.create_timer(
            DURACION_MOVIMIENTO, self._parar_por_timer)

    def _parar_por_timer(self):
        """Callback del timer: detiene el robot."""
        if self.timer_parar is not None:
            self.timer_parar.cancel()
            self.timer_parar = None
        self._detener_robot()

    def _publicar_movimiento(self):
        """Timer callback a 10 Hz: publica el Twist activo."""
        if self.en_movimiento:
            self.pub_cmd_vel.publish(self.twist_actual)

    def _detener_robot(self):
        """Para el robot publicando un Twist vacío."""
        self.en_movimiento = False
        self.pub_cmd_vel.publish(Twist())
        self.get_logger().info('Robot detenido.')

    def _iniciar_sacudida(self):
        """Inicia la secuencia de sacudida lateral."""
        self.get_logger().info('Iniciando sacudida.')
        if self.timer_sacudida is not None:
            try:
                self.timer_sacudida.cancel()
            except Exception:
                pass
            self.timer_sacudida = None
        self.secuencia_sacudida = [-1.0, 1.0, -1.0, 1.0]
        self.indice_sacudida = 0
        self.timer_sacudida = self.create_timer(0.5, self._paso_sacudida)

    def _paso_sacudida(self):
        """Publica el siguiente paso de la secuencia de sacudida."""
        if self.indice_sacudida < len(self.secuencia_sacudida):
            twist = Twist()
            twist.angular.z = self.secuencia_sacudida[self.indice_sacudida]
            self.pub_cmd_vel.publish(twist)
            self.indice_sacudida += 1
        else:
            self.timer_sacudida.cancel()
            self.timer_sacudida = None
            self._detener_robot()

    def _enviar_objetivo_acoplar(self):
        """Envía goal de acoplamiento al action server /dock."""
        self._enviar_dock_goal(
            cliente=self.cliente_acoplar, goal=Dock.Goal(),
            flag='acoplando', acoplado_final=True,
            nombre='Acople', servidor='dock', ya='Ya acoplado.')

    def _enviar_objetivo_desacoplar(self):
        """Envía goal de desacoplamiento al action server /undock."""
        self._enviar_dock_goal(
            cliente=self.cliente_desacoplar, goal=Undock.Goal(),
            flag='desacoplando', acoplado_final=False,
            nombre='Desacople', servidor='undock', ya='Ya desacoplado.')

    def _enviar_dock_goal(self, *, cliente, goal, flag, acoplado_final,
                          nombre, servidor, ya):
        """Patrón común dock/undock: guardas + send_goal_async → get_result_async.

        `flag` es el nombre del atributo booleano "en curso" (acoplando /
        desacoplando); `acoplado_final` el valor de `esta_acoplado` al completar
        (True acopla, False desacopla). `nombre`/`servidor`/`ya` solo dan forma a
        los mensajes de log. La secuencia es idéntica a la de los antiguos cuatro
        handlers separados.
        """
        if self.esta_acoplado is acoplado_final:
            self.get_logger().warn(ya)
            return
        if getattr(self, flag):
            return
        if not cliente.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(f'Servidor {servidor} no disponible.')
            return
        setattr(self, flag, True)

        def _on_goal(futuro):
            try:
                handler = futuro.result()
            except Exception as e:
                self.get_logger().error(f'{nombre} envio: {e}')
                setattr(self, flag, False)
                return
            if not handler.accepted:
                self.get_logger().warn(f'{nombre} rechazado.')
                setattr(self, flag, False)
                return
            handler.get_result_async().add_done_callback(_on_result)

        def _on_result(futuro):
            try:
                futuro.result()
                self.esta_acoplado = acoplado_final
                self.get_logger().info(f'{nombre} completado.')
            except Exception as e:
                self.get_logger().error(f'{nombre} fallo: {e}')
            finally:
                setattr(self, flag, False)

        cliente.send_goal_async(goal).add_done_callback(_on_goal)

    # ==================================================================
    # Subsistema EMERGENCIAS (Twilio)
    # ==================================================================

    # ==================================================================
    # Subsistema DIÁLOGO: callbacks de eventos externos
    # ==================================================================

    def _cb_navegacion(self, msg):
        """Vocaliza el estado de navegación recibido."""
        texto = msg.data.strip()
        if texto:
            self._hablar(texto, priority='drop_old')

    def _cb_navegacion_metrica(self, msg):
        """Vuelca al log de sesión la métrica de validación de una navegación."""
        try:
            d = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        self._logger_sesion.evento(
            'navegacion_metrica',
            destino=d.get('destino'),
            exito=bool(d.get('exito')),
            motivo=d.get('motivo', ''),
            t_nav=d.get('t_nav'),
            err_pos=d.get('err_pos'),
            err_yaw=d.get('err_yaw_deg'),
            cov_max=d.get('cov_max'))

    def _registrar_sistema(self, cpu_pct, mem_pct, temp_c, load1):
        """Callback del MonitorSistema: registra una muestra de carga del sistema."""
        self._logger_sesion.evento(
            'sistema', cpu_pct=cpu_pct, mem_pct=mem_pct,
            temp_c=temp_c, load1=load1)

    def _cb_objetos(self, msg):
        """Vocaliza los objetos detectados recibidos."""
        texto = msg.data.strip()
        if texto:
            self._hablar(texto, priority='drop_old')

    # ==================================================================
    # Subsistema TTS — Piper persistente en memoria
    # ==================================================================

    def _hilo_sintesis(self):
        """Hilo A: router texto → TTS remoto (PC) o Piper local (fallback)."""
        self._asegurar_audio()
        self._sr = TTS_SAMPLE_RATE
        self._tts_listo.set()

        while True:
            item = self._tts_queue.get()
            if item is None:
                continue
            texto, emocion_tipo, emocion_intensidad = item

            if self._cancelar_llm.is_set():
                self._audio_queue.put(FRASE_END)
                continue

            texto_limpio = self._limpiar_texto_tts(texto)
            if not texto_limpio:
                self._audio_queue.put(FRASE_END)
                continue

            hay_remoto = self._pub_tts_request.get_subscription_count() > 0

            if hay_remoto:
                self._tts_remoto_ok = True
                try:
                    payload = json.dumps({
                        'texto': texto_limpio,
                        'tipo': emocion_tipo,
                        'intensidad': emocion_intensidad,
                    }, ensure_ascii=False)
                    self._pub_tts_request.publish(String(data=payload))
                except Exception as e:
                    self.get_logger().warn(f'TTS publish error: {e}')
                    self._audio_queue.put(FRASE_END)
            else:
                # Primera frase sin servidor → anunciar "resfriado" antes del texto
                if self._tts_remoto_ok and not self._anunciado_sin_wifi:
                    self._tts_remoto_ok = False
                    self._anunciado_sin_wifi = True
                    self.get_logger().warn('TTS remoto no disponible, usando Piper local.')
                    self._logger_sesion.evento('tts_fallback', motivo='remoto_no_disponible')
                    try:
                        self._asegurar_voice_local()
                        with self._lock_frases:
                            self._frases_pendientes += 1  # contabilizar FRASE_END extra
                        self._sintetizar(MSG_SIN_WIFI)
                    except Exception as e:
                        self.get_logger().warn(f'Resfriado TTS error: {e}')
                        with self._lock_frases:
                            self._frases_pendientes -= 1
                else:
                    self._tts_remoto_ok = False

                try:
                    self._asegurar_voice_local()
                    self._sintetizar(texto)
                except Exception as e:
                    self.get_logger().warn(f'TTS local error: {e}')
                    self._logger_sesion.evento('error', donde='tts', msg=str(e))
                    self._audio_queue.put(FRASE_END)

    def _asegurar_voice_local(self):
        """Carga lazy de PiperVoice cuando no hay servidor TTS remoto."""
        if self._voice is not None:
            return
        from piper import PiperVoice
        self.get_logger().warn(
            'No hay servidor TTS remoto. Cargando Piper local (~200 MB).')
        self._voice = PiperVoice.load(PIPER_MODEL)
        list(self._voice.synthesize('Hola.'))

    @staticmethod
    def _tiene_wifi():
        """True si wlan0 está asociada al AP."""
        try:
            with open('/sys/class/net/wlan0/operstate') as f:
                return f.read().strip() == 'up'
        except OSError:
            return False

    def _monitor_wifi(self):
        """Hilo: actualiza _wifi_disponible cada WIFI_CHECK_INTERVAL s."""
        while True:
            disponible = self._tiene_wifi()
            if disponible != self._wifi_disponible:
                self._wifi_disponible = disponible
                if disponible:
                    self._anunciado_sin_wifi = False
                    self._tts_remoto_ok = True
                    self.get_logger().info('WiFi recuperado.')
                    self._logger_sesion.evento('wifi', estado='recuperado')
                else:
                    self.get_logger().warn('WiFi perdido.')
                    self._logger_sesion.evento('wifi', estado='perdido')
            time.sleep(WIFI_CHECK_INTERVAL)

    def _cb_tts_audio(self, msg):
        """Recibe chunks PCM del servidor TTS remoto; data vacío = fin frase."""
        # Salvaguarda anti-duplicado: el servidor sella un contador monotónico
        # en layout.data_offset. Las copias que crea una malla Zenoh redundante
        # reutilizan un seq ya visto -> se descartan; sólo pasan seq crecientes.
        seq = msg.layout.data_offset
        if seq:  # 0 = servidor sin sello (compat hacia atrás) -> no filtrar
            retro = self._last_audio_seq - seq
            if 0 <= retro <= AUDIO_DEDUP_BACKJUMP:
                return  # copia duplicada -> descartar
            # seq mayor (frase nueva) o salto atrás grande (servidor reiniciado)
            self._last_audio_seq = seq
        self._t_ultimo_audio_tts = time.time()
        if msg.data:
            self._audio_queue.put(bytes(msg.data))
        else:
            self._audio_queue.put(FRASE_END)

    def _sintetizar(self, texto):
        """Sintetiza con Piper y mete chunks en _audio_queue."""
        if self._voice is None:
            raise RuntimeError('PiperVoice no inicializado.')

        from piper import SynthesisConfig

        texto = self._limpiar_texto_tts(texto)
        if not texto:
            self._audio_queue.put(FRASE_END)
            return

        syn = SynthesisConfig(
            length_scale=0.92,
            noise_scale=0.36,
            noise_w_scale=1.3,
        )

        # Insertar silencios en comas/puntos y coma/dos puntos
        segmentos = re.split(r'([,;:])', texto)
        partes = []
        buf = ''
        for seg in segmentos:
            if seg in (',', ';', ':'):
                if buf.strip():
                    partes.append((buf.strip(), PAUSA_COMA_MS))
                buf = ''
            else:
                buf += seg
        if buf.strip():
            ms_fin = PAUSA_PUNTO_MS if texto[-1] in '.!?' else 0
            partes.append((buf.strip(), ms_fin))

        for parte, pausa_ms in partes:
            if not parte:
                continue
            for chunk in self._voice.synthesize(parte, syn_config=syn):
                if self._cancelar_llm.is_set():
                    break
                data = chunk.audio_int16_bytes
                self._audio_queue.put(data)
                qs = self._audio_queue.qsize()
                if qs > self._metrics_audio_q_max:
                    self._metrics_audio_q_max = qs
            if self._cancelar_llm.is_set():
                break
            if pausa_ms > 0:
                n = int(self._sr * pausa_ms / 1000)
                self._audio_queue.put(bytes(n * 2))

        self._audio_queue.put(FRASE_END)

    def _hilo_reproduccion(self):
        """Hilo B: bytes de _audio_queue → aplay."""
        self._tts_listo.wait()
        # Garantizar que el 'default' de ALSA apunte al altavoz (DAC) y no a un
        # ~/.asoundrc obsoleto de un fallback previo (jack 3.5mm 'Headphones'),
        # que dejaría el TTS saliendo por una salida sin altavoz conectado.
        if self._card_disponible(ALSA_CARD_DAC):
            self._set_asoundrc_card(ALSA_CARD_DAC)
            self._usando_fallback = False
        self._abrir_aplay()

        while True:
            try:
                item = self._audio_queue.get(timeout=0.3)
            except queue.Empty:
                # Si estamos cediendo el altavoz al bridge y ya no queda audio
                # pendiente (p.ej. tras reproducir "Llamando a emergencias"),
                # cerramos aplay para liberar el dispositivo hw exclusivo.
                if self._ceder_altavoz.is_set() and self._aplay_proc is not None:
                    self._abrir_aplay()  # con cede activo: cierra y NO reabre
                continue
            if item is None:
                continue

            if item is FRASE_END:
                # Si venimos de barge-in, _interrumpir_tts ya ajustó
                # _frases_pendientes; un FRASE_END tardío de la frase
                # abortada llegaría aquí y dejaría el contador en negativo.
                if self._cancelar_llm.is_set():
                    continue
                # Esperar a que aplay drene el buffer
                espera = max(0.0, self._aplay_eta - time.time())
                if espera > 0:
                    time.sleep(espera)
                time.sleep(0.05)
                with self._lock_frases:
                    self._frases_pendientes -= 1
                    ultima = (self._frases_pendientes == 0)
                if ultima:
                    self._mute_stt_until = time.time() + POST_TTS_MUTE_SEC
                    t = threading.Timer(POST_TTS_MUTE_SEC, self._fin_tts_real)
                    t.start()
                continue

            if self._cancelar_llm.is_set():
                continue

            card_actual = (ALSA_CARD_FALLBACK if self._usando_fallback
                           else ALSA_CARD_DAC)
            if not self._card_disponible(card_actual):
                self.get_logger().warn(
                    f'Tarjeta {card_actual} perdida, recuperando...')
                self._asegurar_audio()
                self._abrir_aplay()
            elif (self._aplay_proc is None
                  or self._aplay_proc.poll() is not None):
                self._abrir_aplay()

            duracion_chunk = len(item) / (self._sr * 2)
            ahora = time.time()
            if self._aplay_eta < ahora:
                self._aplay_eta = ahora + duracion_chunk
            else:
                self._aplay_eta += duracion_chunk

            try:
                if self._t_tts_primer_byte == 0.0:
                    self._t_tts_primer_byte = time.time()
                self._aplay_proc.stdin.write(item)
                self._aplay_proc.stdin.flush()
                self._t_tts_ultimo_byte = time.time()
                self._t_ultimo_audio_tts = self._t_tts_ultimo_byte
            except Exception as e:
                self.get_logger().warn(f'TTS aplay write error: {e}')
                self._asegurar_audio()
                self._abrir_aplay()

            if self._usando_fallback:
                self._contador_fallback += 1
                if self._contador_fallback >= 100:
                    self._contador_fallback = 0
                    if self._card_disponible(ALSA_CARD_DAC):
                        self._set_asoundrc_card(ALSA_CARD_DAC)
                        self._usando_fallback = False
                        self._abrir_aplay()

    def _abrir_aplay(self):
        """Abre (o reabre) el proceso aplay."""
        if self._aplay_proc is not None:
            try:
                if (self._aplay_proc.stdin
                        and not self._aplay_proc.stdin.closed):
                    self._aplay_proc.stdin.close()
            except Exception:
                pass
            try:
                self._aplay_proc.terminate()
            except Exception:
                pass
            self._aplay_proc = None
        # Emergencia: cedemos el altavoz al bridge (ALSA hw es exclusivo). No
        # reabrimos aplay aquí; el bridge es el dueño del altavoz durante la llamada.
        if self._ceder_altavoz.is_set():
            return
        self._aplay_proc = subprocess.Popen(
            ['aplay', '-q', '-r', str(self._sr),
             '-f', 'S16_LE', '-t', 'raw', '-c', '1',
             '-'],
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        self._aplay_eta = 0.0

    def _interrumpir_tts(self):
        """Barge-in: corta aplay, vacía colas y cancela LLM en curso."""
        self._logger_sesion.evento('barge_in')
        # Invalidar el turno LLM en curso de forma permanente: aunque el timer
        # de 0.5 s limpie _cancelar_llm, el hilo viejo verá su mi_gen desfasado
        # y morirá sin volver a publicar (evita audio duplicado/entrelazado).
        with self._lock_gen:
            self._gen_llm += 1
        self._cancelar_llm.set()
        # Si hay un flujo FETCH en curso, abortarlo también (corta navegación).
        if self._coordinador.activa():
            self._coordinador.cancelar()
        try:
            self._pub_tts_cancel.publish(Empty())
        except Exception:
            pass
        while True:
            try:
                item = self._tts_queue.get_nowait()
                if item is not None:
                    with self._lock_frases:
                        fp = max(0, self._frases_pendientes - 1)
                        self._frases_pendientes = fp
            except queue.Empty:
                break
        while True:
            try:
                item = self._audio_queue.get_nowait()
                if item is FRASE_END:
                    with self._lock_frases:
                        fp = max(0, self._frases_pendientes - 1)
                        self._frases_pendientes = fp
            except queue.Empty:
                break
        while True:
            try:
                self._speech_queue.get_nowait()
            except queue.Empty:
                break
        self._abrir_aplay()
        self._aplay_eta = 0.0
        self._t_tts_primer_byte = 0.0
        self._t_tts_ultimo_byte = 0.0
        self._hablando = False
        self._set_tts_activo(False)
        self._mute_microfono(False)
        self.get_logger().info('TTS interrumpido.')

        t = threading.Timer(0.5, self._cancelar_llm.clear)
        t.daemon = True
        t.start()

    # ---- Modo emergencia ----

    def pausar_para_emergencia(self):
        """Bloquea el dispatch de conversación y abre el socket UDP para el bridge.

        STT y LLM siguen activos: _procesar_texto redirige a _check_coherencia_emergencia.
        """
        self._modo_emergencia.set()
        self._interrumpir_tts()
        # Ceder el altavoz al bridge: tras reproducir "Llamando a emergencias",
        # _hilo_reproduccion cerrará aplay y _abrir_aplay no lo reabrirá, dejando
        # el dispositivo libre para que el bridge reproduzca al operador.
        self._ceder_altavoz.set()
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setblocking(False)
        self._bridge_udp_sock = sock
        # Señalar a los demás nodos que esta conversación es lo único que importa:
        # liberan RAM/CPU (YOLO, navegación) hasta que se reanude.
        self._set_emergencia_activa(True)
        # Quiescer recursos externos (cámara OAK-D, stack Nav2) en hilo aparte para
        # no retrasar la llamada Twilio. Best-effort: si no están, no pasa nada.
        threading.Thread(
            target=self._quiesce_recursos_externos, daemon=True).start()
        self.get_logger().info('Modo emergencia: pipeline de conversación pausado.')

    def reanudar_tras_emergencia(self):
        """Reactiva el pipeline de conversación normal tras finalizar la llamada."""
        self._modo_emergencia.clear()
        # Recuperar el altavoz: el bridge ya lo ha liberado al cerrar la llamada.
        self._ceder_altavoz.clear()
        if self._bridge_udp_sock is not None:
            try:
                self._bridge_udp_sock.close()
            except Exception:
                pass
            self._bridge_udp_sock = None
        # Reactivar el resto del sistema (garantizado: corre en el finally de
        # GestorEmergencia._ejecutar, que está triple-protegido).
        self._set_emergencia_activa(False)
        threading.Thread(
            target=self._reactivar_recursos_externos, daemon=True).start()
        self._hablar('La llamada ha finalizado. ¿Cómo estás?', 'calma', 0.6)
        self.get_logger().info('Modo emergencia: pipeline de conversación reanudado.')

    def _set_emergencia_activa(self, activa):
        """Publica el estado del modo emergencia en /emergency/active (latcheado)."""
        msg = Bool()
        msg.data = bool(activa)
        self.pub_emergencia_activa.publish(msg)

    # ---- Pausa/reanudación de recursos externos (cámara + Nav2) ----

    def _quiesce_recursos_externos(self):
        """Pausa cámara OAK-D y stack Nav2 para liberar RAM/CPU. Best-effort."""
        self._llamar_trigger(self._cli_cam_stop, SRV_CAMARA_STOP, 'parar cámara')
        self._manage_nav2(pausar=True)

    def _reactivar_recursos_externos(self):
        """Reactiva cámara OAK-D y stack Nav2 tras la emergencia. Best-effort."""
        self._llamar_trigger(self._cli_cam_start, SRV_CAMARA_START, 'arrancar cámara')
        self._manage_nav2(pausar=False)

    def _llamar_trigger(self, cli, servicio, descripcion):
        """Llama a un cliente std_srvs/Trigger ya creado, sin bloquear (best-effort)."""
        try:
            if not cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn(
                    f'[EMG] servicio {servicio} no disponible; omito {descripcion}.')
                return
            fut = cli.call_async(Trigger.Request())
            fut.add_done_callback(
                lambda f, s=servicio: self.get_logger().info(
                    f'[EMG] {s} -> {self._resultado_servicio(f)}'))
        except Exception as e:
            self.get_logger().warn(f'[EMG] error llamando {servicio}: {e}')

    def _manage_nav2(self, pausar):
        """PAUSE/RESUME del lifecycle_manager de Nav2 (best-effort, si existe)."""
        if self._ManageLifecycleNodes is None:
            self.get_logger().debug('[EMG] nav2_msgs no disponible; omito Nav2.')
            return
        comando = (self._ManageLifecycleNodes.Request.PAUSE if pausar
                   else self._ManageLifecycleNodes.Request.RESUME)
        accion = 'PAUSE' if pausar else 'RESUME'
        for servicio, cli in self._cli_nav2:
            try:
                if not cli.wait_for_service(timeout_sec=2.0):
                    self.get_logger().debug(
                        f'[EMG] {servicio} no disponible (¿nav:=false?); omito {accion}.')
                    continue
                req = self._ManageLifecycleNodes.Request()
                req.command = comando
                fut = cli.call_async(req)
                fut.add_done_callback(
                    lambda f, s=servicio, a=accion: self.get_logger().info(
                        f'[EMG] {s} {a} -> {self._resultado_servicio(f)}'))
            except Exception as e:
                self.get_logger().warn(f'[EMG] error en {servicio}: {e}')

    @staticmethod
    def _resultado_servicio(fut):
        try:
            return 'ok' if fut.result().success else 'fallo'
        except Exception as e:
            return f'error({e})'

    def _check_coherencia_emergencia(self, texto):
        """Marca habla coherente del residente durante la emergencia.

        El STT ya es la prueba de inteligibilidad: si Vosk transcribe alguna palabra
        real (≥3 caracteres), el residente está comunicándose, así que notificamos al
        bridge (vía GestorEmergencia) para que resetee su timer de relevo. Heurística
        inline y barata (sin LLM): corre en el propio hilo del STT sin bloquearlo y sin
        consumir CPU/red extra, algo crítico en la RPi 4B durante la llamada.
        """
        if not self._gestor_emergencia._activa:
            return
        if not any(len(p) >= 3 for p in texto.strip().split()):
            self.get_logger().debug(
                f'[EMG coherencia] descartado (sin palabra ≥3 chars): "{texto}"')
            return
        self.get_logger().info(
            f'Emergencia: habla coherente detectada ("{texto[:40]}"), resetear timer bridge.')
        self._gestor_emergencia.notificar_habla()

    # ---- Recuperación tarjeta audio ----

    @staticmethod
    def _card_disponible(nombre):
        """Devuelve True si la tarjeta ALSA aparece en /proc/asound/cards."""
        try:
            with open('/proc/asound/cards', 'r') as f:
                return nombre in f.read()
        except OSError:
            return False

    def _asegurar_audio(self):
        """Garantiza que haya una tarjeta de audio operativa."""
        if self._card_disponible(ALSA_CARD_DAC):
            if self._usando_fallback:
                self._set_asoundrc_card(ALSA_CARD_DAC)
                self._usando_fallback = False
                self.get_logger().info('Audio: DAC USB.')
            return
        self.get_logger().warn('DAC no disponible. Reseteando hub USB...')
        if self._reset_hub_usb() and self._card_disponible(ALSA_CARD_DAC):
            self._set_asoundrc_card(ALSA_CARD_DAC)
            self._usando_fallback = False
            self.get_logger().info('Audio: DAC USB recuperado tras reset.')
            return
        self.get_logger().warn('DAC no recuperable. Fallback jack 3.5mm.')
        self._set_asoundrc_card(ALSA_CARD_FALLBACK)
        self._usando_fallback = True
        self._contador_fallback = 0

    def _reset_hub_usb(self):
        """Desconecta y reconecta el hub USB; devuelve True si el DAC vuelve."""
        unbind = '/sys/bus/usb/drivers/usb/unbind'
        bind = '/sys/bus/usb/drivers/usb/bind'
        try:
            subprocess.run(['sudo', 'tee', unbind],
                           input=USB_HUB_PORT.encode(),
                           timeout=5, capture_output=True)
            time.sleep(2)
            subprocess.run(['sudo', 'tee', bind],
                           input=USB_HUB_PORT.encode(),
                           timeout=5, capture_output=True)
        except Exception as e:
            self.get_logger().error(f'USB reset: {e}')
            return False
        for _ in range(ESPERA_REENUMERACION):
            time.sleep(1)
            if self._card_disponible(ALSA_CARD_DAC):
                time.sleep(1)
                return True
        return False

    @staticmethod
    def _set_asoundrc_card(card_name):
        """Escribe ~/.asoundrc apuntando a la tarjeta indicada."""
        contenido = f"""pcm.!default {{
    type asym
    playback.pcm "plughw:CARD={card_name},DEV=0"
    capture.pcm "plughw:CARD={ALSA_CARD_MIC},DEV=0"
}}

ctl.!default {{
    type hw
    card {card_name}
}}
"""
        with open(os.path.expanduser('~/.asoundrc'), 'w') as f:
            f.write(contenido)

    def _mute_microfono(self, mute):
        """Mute/unmute del capture ALSA; ignora controles inexistentes."""
        accion = 'nocap' if mute else 'cap'
        for control in ('Capture', 'Mic', 'Front Mic'):
            try:
                r = subprocess.run(
                    ['amixer', '-c', ALSA_CARD_MIC, 'set', control, accion],
                    capture_output=True, timeout=1
                )
                if r.returncode == 0:
                    return
            except Exception:
                continue
        if mute:
            subprocess.run(['pkill', '-STOP', 'arecord'], capture_output=True)
        else:
            subprocess.run(['pkill', '-CONT', 'arecord'], capture_output=True)

    # ==================================================================
    # Speech queue: worker único (sustituye _hablar_async + _hablar_sync)
    # ==================================================================

    def _hablar(self, texto,
                emocion_tipo=EMOCION_DEFAULT_TIPO,
                emocion_intensidad=EMOCION_DEFAULT_INTENSIDAD,
                priority='normal'):
        """Encola texto para síntesis.

        priority='drop_old': descarta el mas antiguo si la cola esta llena.
        priority='normal': bloquea hasta haber hueco (backpressure LLM).
        """
        if priority == 'drop_old':
            if self._speech_queue.full():
                try:
                    self._speech_queue.get_nowait()
                except queue.Empty:
                    pass
            try:
                self._speech_queue.put_nowait((texto, emocion_tipo, emocion_intensidad))
            except queue.Full:
                self.get_logger().warn(
                    'speech_queue llena (drop_old), frase descartada.')
        else:
            try:
                self._speech_queue.put(
                    (texto, emocion_tipo, emocion_intensidad), timeout=10.0)
            except queue.Full:
                self.get_logger().warn(
                    'speech_queue bloqueada >10s, frase descartada.')

        qs = self._speech_queue.qsize()
        if qs > self._metrics_speech_q_max:
            self._metrics_speech_q_max = qs

    def _speech_worker(self):
        """Worker único: traslada frases de _speech_queue a _tts_queue."""
        while True:
            item = self._speech_queue.get()
            if item is None:
                continue
            texto, emocion_tipo, emocion_intensidad = item
            self._tts_listo.wait()
            with self._lock_frases:
                self._frases_pendientes += 1
                if not self._hablando:
                    self._hablando = True
                    self._t_ultimo_audio_tts = time.time()
                    self._set_tts_activo(True)
                    self._mute_microfono(True)
            self._tts_queue.put((texto, emocion_tipo, emocion_intensidad))
            qs = self._tts_queue.qsize()
            if qs > self._metrics_tts_q_max:
                self._metrics_tts_q_max = qs

    def _fin_tts_real(self):
        """Cierre definitivo del TTS tras la ventana de gracia post-audio."""
        with self._lock_frases:
            if self._frases_pendientes > 0:
                return
            self._hablando = False
            self._set_tts_activo(False)
            self._mute_microfono(False)
            if self._en_conversacion:
                self._ultimo_input = time.time()

        t_stt = self._t_stt_final
        t_llm = self._t_llm_primer_token
        t_ap1 = self._t_tts_primer_byte
        t_ape = self._t_tts_ultimo_byte
        if t_stt > 0 and t_llm > 0 and t_ap1 > 0 and t_ape > 0:
            self.get_logger().info(
                f'METRICS turn n_chars={self._metrics_n_chars} '
                f'stt_to_llm1={t_llm - t_stt:.2f}s '
                f'llm1_to_aplay1={t_ap1 - t_llm:.2f}s '
                f'aplay1_to_end={t_ape - t_ap1:.2f}s '
                f'cola_speech_max={self._metrics_speech_q_max} '
                f'cola_tts_max={self._metrics_tts_q_max} '
                f'cola_audio_max={self._metrics_audio_q_max}'
            )
            gap_prev = (round(t_stt - self._t_ultimo_turno_fin, 2)
                        if self._t_ultimo_turno_fin > 0 else None)
            self._logger_sesion.evento(
                'turno',
                n_chars=self._metrics_n_chars,
                stt_to_llm1=round(t_llm - t_stt, 3),
                llm1_to_aplay1=round(t_ap1 - t_llm, 3),
                aplay1_to_end=round(t_ape - t_ap1, 3),
                gap_prev=gap_prev,
                cola_speech_max=self._metrics_speech_q_max,
                cola_tts_max=self._metrics_tts_q_max,
                cola_audio_max=self._metrics_audio_q_max,
            )
            # El gap del siguiente turno se mide desde que acaba este.
            self._t_ultimo_turno_fin = time.time()

        # Reset métricas para el siguiente turno
        self._t_stt_final = 0.0
        self._t_llm_primer_token = 0.0
        self._t_tts_primer_byte = 0.0
        self._t_tts_ultimo_byte = 0.0
        self._metrics_n_chars = 0
        self._metrics_speech_q_max = 0
        self._metrics_tts_q_max = 0
        self._metrics_audio_q_max = 0

        self.get_logger().info('TTS realmente terminado, micro reabierto.')

    # ==================================================================
    # LLM — OpenRouter streaming con flush por puntuación / tiempo
    # ==================================================================

    def _llm_responder(self, pregunta, mi_gen=None):
        """Llama a OpenRouter con streaming y flushea al TTS por frases.

        `mi_gen` es el id de generación capturado al lanzar el turno. Si
        _gen_llm avanza (nuevo turno o barge-in) este hilo se aborta sin
        publicar nada más, así dos turnos nunca alimentan el TTS a la vez.
        """
        if self.client is None:
            self._hablar('El módulo de lenguaje no está disponible.')
            return

        # Guard semántico: capturar el texto del usuario para _ejecutar_accion.
        self._texto_usuario_turno = pregunta

        if time.time() - self._ultimo_intercambio > HISTORIAL_TIMEOUT:
            self._historial.clear()

        hora = datetime.now().strftime('%H:%M')
        prefijo = (
            f'[hora: {hora} | emocion: desconocido | usuario: desconocido]'
        )
        msg_usuario = f'{prefijo} {pregunta}'

        self._historial.append({'role': 'user', 'content': msg_usuario})

        if len(self._historial) > HISTORIAL_MAX_PARES * 2:
            self._historial = self._historial[-HISTORIAL_MAX_PARES * 2:]

        # Inyectar estado operacional para que el LLM sepa qué está pasando.
        estado_partes = []
        if self._coordinador.activa():
            estado_partes.append('tarea_pickup:en_curso')
        with self._gestor_emergencia._lock:
            if self._gestor_emergencia._activa:
                estado_partes.append('emergencia:activa')
        system_content = self._system_prompt
        if estado_partes:
            system_content += (
                '\n\n[ESTADO OPERACIONAL: ' + ' | '.join(estado_partes) + ']'
                '\nNo emitas nuevos goto ni pickup mientras haya tareas en curso.'
            )

        messages = (
            [{'role': 'system', 'content': system_content}]
            + self._historial
        )

        raw_full = ''
        buffer = ''
        self._t_llm_primer_token = 0.0
        self._metrics_n_chars = 0

        emocion_tipo = EMOCION_DEFAULT_TIPO
        emocion_intensidad = EMOCION_DEFAULT_INTENSIDAD
        emocion_extraida = False

        try:
            stream = self.client.chat.completions.create(
                model=LLM_MODEL,
                messages=messages,
                max_tokens=MAX_TOKENS,
                temperature=TEMPERATURE,
                frequency_penalty=FREQUENCY_PENALTY,
                presence_penalty=PRESENCE_PENALTY,
                stream=True,
            )

            for chunk in stream:
                # Aborta si llegó barge-in (_cancelar_llm) o si otro turno tomó
                # el relevo (mi_gen desfasado). La comprobación del gen va ANTES
                # de procesar/publicar el chunk, así un hilo superado nunca
                # vuelve a alimentar el TTS.
                if self._cancelar_llm.is_set() or (
                        mi_gen is not None and mi_gen != self._gen_llm):
                    break

                delta = chunk.choices[0].delta.content or ''
                if not delta:
                    continue

                if self._t_llm_primer_token == 0.0:
                    self._t_llm_primer_token = time.time()

                raw_full += delta
                buffer += delta

                # Extraer emoción del tag al inicio de la respuesta (lazy, una vez)
                if not emocion_extraida:
                    resultado = self._extraer_emocion(raw_full)
                    if resultado is not None:
                        emocion_tipo, emocion_intensidad = resultado
                        emocion_extraida = True

                # Con conexión lenta: sólo flusha en puntuación fuerte
                # (y con mínimo de chars para evitar fragmentos de 3 palabras)
                # o al alcanzar el hard cap.
                tiene_punt = any(p in delta for p in FLUSH_PUNCT)
                debe_flush = (
                    (tiene_punt and len(buffer) >= FLUSH_MIN_CHARS)
                    or len(buffer) >= FLUSH_HARD_CAP_CHARS
                )
                if debe_flush and buffer.strip():
                    tts_texto, _ = self._parsear_respuesta(buffer)
                    tts_limpio = self._limpiar_texto_tts(tts_texto)
                    if tts_limpio:
                        self.get_logger().info(
                            f'[LLM->TTS gen={mi_gen}] {tts_limpio!r}')
                        self._hablar(tts_limpio, emocion_tipo, emocion_intensidad)
                        self._metrics_n_chars += len(tts_limpio)
                    buffer = ''

        except Exception as e:
            self.get_logger().error(f'OpenRouter: {e}')
            self._logger_sesion.evento('error', donde='openrouter', msg=str(e))
            self._hablar('Lo siento no puedo responder ahora')
            return

        # Si este turno fue superado por otro (barge-in / nuevo turno), salir sin
        # tocar el estado del turno nuevo ni publicar el flush residual.
        if mi_gen is not None and mi_gen != self._gen_llm:
            return
        if self._cancelar_llm.is_set():
            self._hablando = False
            self._set_tts_activo(False)
            return

        # Flush residual
        if buffer.strip():
            tts_texto, _ = self._parsear_respuesta(buffer)
            tts_limpio = self._limpiar_texto_tts(tts_texto)
            if tts_limpio:
                self.get_logger().info(
                    f'[LLM->TTS gen={mi_gen} residual] {tts_limpio!r}')
                self._hablar(tts_limpio, emocion_tipo, emocion_intensidad)
                self._metrics_n_chars += len(tts_limpio)

        # Parsear acciones del texto completo acumulado
        texto_hablado, acciones = self._parsear_respuesta(raw_full)

        # Red de seguridad (CASO B): si Ana DICE que está llamando a emergencias
        # pero el 3B se olvidó del token <action>emergency:auto</action>, lo
        # disparamos por intención verbal. Guarda de negación para no actuar ante
        # "no voy a llamar a emergencias". Solo si no hay ya un token de emergencia.
        if not any(t.strip().lower().startswith('emergency') for t in acciones):
            if (EMERGENCIA_INTENT.search(texto_hablado)
                    and not EMERGENCIA_NEG.search(texto_hablado)):
                self.get_logger().warning(
                    'Emergencia por INTENCIÓN VERBAL (el LLM no emitió el token).')
                self._logger_sesion.evento('emergencia', via='red_verbal')
                acciones = list(acciones) + ['emergency:auto']

        self._historial.append({'role': 'assistant', 'content': raw_full})
        self._ultimo_intercambio = time.time()

        llm_ft = (round(self._t_llm_primer_token - self._t_stt_final, 3)
                  if self._t_llm_primer_token > 0 and self._t_stt_final > 0
                  else None)
        # Texto real de la respuesta (sin tags) y emoción aplicada: lo necesita
        # el análisis de coherencia afectiva de la Sesión 1 del TFM. Se trunca
        # para que las líneas del JSONL no crezcan sin límite. El logger es
        # defensivo: si algo falla aquí, no debe tumbar el turno.
        try:
            resp_limpia = self._limpiar_texto_tts(
                self._parsear_respuesta(raw_full)[0])
        except Exception:
            resp_limpia = ''
        self._logger_sesion.evento(
            'llm', q_chars=len(pregunta), r_chars=len(raw_full),
            llm_first_token=llm_ft, acciones=list(acciones),
            pregunta=pregunta[:300], respuesta=resp_limpia[:500],
            emocion=emocion_tipo, intensidad=emocion_intensidad)

        self.get_logger().info(
            f'LLM[Q]: {pregunta}\n'
            f'LLM[R]: {raw_full}\n'
            f'LLM[A]: acciones={acciones}'
        )

        for token in acciones:
            self._ejecutar_accion(token)

    @staticmethod
    def _parsear_respuesta(raw):
        """Extrae acciones y texto limpio (sin tags <action> ni <emocion>) de la respuesta LLM."""
        acciones = re.findall(r'<action>(.*?)</action>', raw, re.DOTALL)
        texto = re.sub(r'<action>.*?</action>', '', raw, flags=re.DOTALL)
        # Formato canónico: <emocion tipo="X" intensidad="Y"/>
        texto = re.sub(r'</?emocion\b[^>]*>', '', texto, flags=re.IGNORECASE)
        # Formato alternativo que emite el 3B: <calma intensidad="0.5">…</calma>
        texto = _EMOTION_WRAPPER_RE.sub('', texto)
        return texto.strip(), acciones

    @staticmethod
    def _extraer_emocion(raw):
        """Extrae tipo e intensidad del tag de emoción si existe en raw.

        Soporta dos formatos:
          1. Canónico:    <emocion tipo="calma" intensidad="0.5"/>
          2. Alternativo: <calma intensidad="0.5"> (formato real del 3B)
        """
        # Formato canónico
        m = re.search(r'<emocion\b[^>]*>', raw, re.IGNORECASE)
        if m:
            tag = m.group(0)
            mt = re.search(r'tipo\s*=\s*"?([a-zA-Z]+)"?', tag)
            mi = re.search(r'intensidad\s*=\s*"?([0-9.]+)"?', tag)
            if mt and mi:
                try:
                    return mt.group(1).lower(), float(mi.group(1))
                except ValueError:
                    pass

        # Formato alternativo <calma intensidad="0.5"> o <calma intensidad=0.5>
        m2 = re.search(
            r'<(' + '|'.join(_EMOTION_TYPES) + r')\b[^>]*'
            r'intensidad\s*=\s*"?([0-9.]+)"?',
            raw, re.IGNORECASE)
        if m2:
            try:
                return m2.group(1).lower(), float(m2.group(2))
            except ValueError:
                pass

        # Fallback: nombre del tag de emoción sin atributo intensidad
        m3 = re.search(
            r'<(' + '|'.join(_EMOTION_TYPES) + r')\b',
            raw, re.IGNORECASE)
        if m3:
            return m3.group(1).lower(), EMOCION_DEFAULT_INTENSIDAD

        return None

    @staticmethod
    def _tiene_intent(texto_usuario, keywords):
        """True si alguna keyword aparece en el texto normalizado del usuario."""
        t = texto_usuario.lower()
        return any(kw in t for kw in keywords)

    def _ejecutar_accion(self, token):
        """Despacha una acción extraída de la respuesta del LLM.

        Guards semánticos:
          - goto/pickup sólo se ejecutan si el texto del turno contiene
            una keyword de intención real. Si no, es relleno del 3B → descarte.
          - emergency:auto sólo si hay keyword de riesgo físico en el texto.
        """
        tipo, _, param = token.partition(':')
        tipo = tipo.strip()
        param = param.strip()
        texto = self._texto_usuario_turno

        if tipo == 'goto':
            if not self._tiene_intent(texto, _INTENT_GOTO_KW):
                self.get_logger().warn(
                    f'Accion LLM goto:{param} descartada '
                    f'(sin intención de movimiento en el turno: "{texto[:60]}")')
                return
            msg = String()
            msg.data = f'navegar:{param}'
            self.pub_comando.publish(msg)
            self.get_logger().info(f'Accion LLM: navegar:{param}')

        elif tipo == 'pickup':
            if not self._tiene_intent(texto, _INTENT_PICKUP_KW):
                self.get_logger().warn(
                    f'Accion LLM pickup:{param} descartada '
                    f'(sin intención de recogida en el turno: "{texto[:60]}")')
                return
            self.get_logger().info(f'Accion LLM: pickup:{param}')
            self._coordinador.iniciar(param)

        elif tipo == 'emergency':
            if param == 'auto' and not EMERGENCY_FISICA_KW.search(texto):
                self.get_logger().warn(
                    f'Accion LLM emergency:auto bloqueada '
                    f'(sin keywords de riesgo físico en el turno: "{texto[:60]}")')
                return
            self.get_logger().info('Accion LLM: emergencia')
            self._gestor_emergencia.activar(param or 'accidente')

        else:
            self.get_logger().warn(f'Accion LLM desconocida: {token}')

    @staticmethod
    def _limpiar_texto_tts(texto):
        """Reduce pausas artificiales de Piper eliminando puntuación molesta."""
        # Red de seguridad: ningún tag (ni fragmento sin cerrar por el flush de
        # streaming) debe llegar nunca a Piper, sea cual sea la malformación.
        texto = re.sub(r'<[^>]*>', ' ', texto)   # tag residual completo
        texto = re.sub(r'<[^>]*$', ' ', texto)   # fragmento de tag sin cerrar
        # Fragmentos SIN brackets que dejan los tags partidos en el límite del
        # buffer del flush (p.ej. 'intensidad="0.5"/>') o un 3B que malforma el
        # tag. Sólo se borra lo que tiene forma de tag/atributo, nunca palabras
        # españolas sueltas: attr="valor" / attr=valor, restos de <emocion>/<action>
        # y tokens de acción tipo 'goto:'/'pickup:'/'emergency:'.
        texto = re.sub(r'[\wáéíóúñ]+\s*=\s*"?[\w.\-]+"?', ' ', texto, flags=re.I)
        texto = re.sub(r'\b(emocion|action)\b', ' ', texto, flags=re.I)
        texto = re.sub(r'\b(goto|pickup|emergency)\s*:\s*\w*', ' ', texto, flags=re.I)
        texto = re.sub(r'[<>/]+', ' ', texto)   # brackets/barras huérfanos
        texto = re.sub(r'[,:;]+', ' ', texto)
        texto = re.sub(r'[¿?¡!\.]+', ' ', texto)
        texto = re.sub(r'\s+', ' ', texto)
        return texto.strip()

def main(args=None):
    """Entry point del nodo asistente."""
    rclpy.init(args=args)
    nodo = AsistenteNode()

    # Garantizar que el log de sesión se cierra pase lo que pase: Ctrl-C,
    # SIGTERM (ros2 launch), o excepción. `cerrar()` es idempotente.
    atexit.register(nodo._logger_sesion.cerrar)

    def _on_sigterm(signum, frame):
        nodo._logger_sesion.cerrar()
        raise KeyboardInterrupt

    try:
        signal.signal(signal.SIGTERM, _on_sigterm)
    except (ValueError, OSError):
        # signal.signal sólo funciona en el hilo principal.
        pass

    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo._logger_sesion.cerrar()
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
