"""
nodo_navegacion_node — navegación por voz con Nav2.

Escucha /voice_command para comandos 'navegar:<destino>', 'volver_a_base',
'parar', 'relocalizar', 'registrar_origen' y 'volver_a_origen'. Publica estado
hablable en /navegacion_estado y resultado machine-readable
('llegada:<destino>', 'fallo:<destino>', 'cancelada') en /navegacion_resultado.
Gestiona desacople automático antes de navegar y acople al volver a base.
Requiere Nav2 activo (localization + navigation).

Waypoints: se leen en runtime de ~/mapeos/nav2/waypoints.yaml (mismo formato que
save_waypoints.py / go_to_waypoint.py). Añadir un waypoint con save_waypoints.py
NO requiere reiniciar el nodo: se relee si cambia el mtime del archivo.

Localización dinámica: el nodo es la única fuente de la pose inicial. Persiste la
última pose de AMCL en ~/mapeos/nav2/last_pose.yaml y la restaura al arrancar, así
no hay que indicar la posición en RViz cada vez. Si no hay pose guardada, siembra
desde el dock (si está acoplado) o desde una pose central por defecto. El comando
'relocalizar' dispara relocalización global + giro en sitio como recuperación.
"""

import atexit
import json
import math
import os
import signal
import threading
import time
import unicodedata

import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from std_msgs.msg import String, Bool
from std_srvs.srv import Empty
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from irobot_create_msgs.action import Dock, Undock
from irobot_create_msgs.msg import DockStatus

from .metricas_logger import SesionLogger
from .monitor_sistema import MonitorSistema

# QoS compatible con Create 3 (publica BEST_EFFORT / VOLATILE)
QOS_SENSOR = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

# QoS latcheado para /emergency/active (debe coincidir con el publisher de
# asistente_node): TRANSIENT_LOCAL para recibir el estado actual al suscribirse.
QOS_LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

WAYPOINTS_FILE = '/home/ubuntu/mapeos/nav2/waypoints.yaml'
LAST_POSE_FILE = '/home/ubuntu/mapeos/nav2/last_pose.yaml'

# Nombre del waypoint sobre el dock (capturado una vez con save_waypoints.py).
# Lo usa volver_a_base y el fallback de pose inicial cuando el robot arranca acoplado.
WAYPOINT_BASE = 'base'

# Fallback de pose inicial cuando no hay last_pose.yaml ni dock conocido.
# (zona central de la casa, en frame map; misma que tenía localization.yaml)
DEFAULT_INITIAL_POSE = (-2.14, -7.90, 0.20)

UNDOCK_TIMEOUT = 15.0
DOCK_TIMEOUT = 30.0

POSE_SAVE_PERIOD = 5.0       # cada cuánto se vuelca la pose de AMCL a disco
RELOC_SPIN_SPEED = 0.5       # rad/s del giro de relocalización
RELOC_SPIN_TIME = 13.0       # ~360° + margen a 0.5 rad/s

# Validación de navegación autónoma (Tabla 2.17 del TFM: RF-02 / RNF-02 / Métrica 1).
# Éxito = alcanzar la pose objetivo con error de posición < ERR_POS_MAX y de
# orientación < ERR_YAW_MAX_DEG en menos de NAV_HARD_TIMEOUT segundos.
NAV_HARD_TIMEOUT = 120.0     # s; superarlo cuenta como fallo (motivo=timeout)
ERR_POS_MAX = 0.30           # m; tolerancia de posición para considerar éxito
ERR_YAW_MAX_DEG = 15.0       # grados; tolerancia de orientación para éxito
# Traza de covarianza XY de AMCL por encima de la cual se considera "pérdida de
# localización" durante el trayecto (best-effort, motivo=amcl).
AMCL_COV_MAX = 0.5           # m²

# Directorio de logs de sesión (mismo que usa el stack completo vía asistente_node),
# para que los resúmenes de validación de Nav2 aislado convivan con los demás.
LOG_DIR = os.path.join(
    os.path.expanduser('~'), 'asistente_turtlebot4-main', 'logs_sesiones')
# Pausa tras cada pierna de una orden de validación para que AMCL asiente la pose
# antes de medir el error / arrancar la siguiente pierna.
SETTLE_SEG = 2.0


def _normalizar(s):
    """minúsculas + sin acentos, para emparejar destinos hablados con las claves."""
    s = unicodedata.normalize('NFKD', s.strip().lower())
    return ''.join(c for c in s if not unicodedata.combining(c))


def _yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def make_pose(navigator, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = navigator.get_clock().now().to_msg()
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.orientation.z = math.sin(yaw / 2)
    p.pose.orientation.w = math.cos(yaw / 2)
    return p


class NodoNavegacion(Node):
    def __init__(self):
        super().__init__('nodo_navegacion_node')

        self.declare_parameter('modo_terminal', False)
        self._modo_terminal = self.get_parameter(
            'modo_terminal').get_parameter_value().bool_value

        # Waypoint de arranque: si está definido y existe, la pose inicial se
        # fuerza ahí en cada lanzamiento (cada ensayo de validación parte de la
        # misma pose conocida). Pasar start_waypoint:='' recupera la persistencia.
        self.declare_parameter('start_waypoint', 'salon')
        self._start_waypoint = _normalizar(self.get_parameter(
            'start_waypoint').get_parameter_value().string_value)

        # Modo validación: el nodo corre AISLADO (Nav2 sin el stack de voz) y se
        # auto-loguea (métricas de navegación + carga de CPU/RAM) en su propio
        # SesionLogger, ya que aquí no existe asistente_node. En modo normal
        # (default False) sólo publica a /navegacion_metricas y es asistente_node
        # quien loguea, así no hay doble conteo.
        self.declare_parameter('validacion', False)
        self._validacion = self.get_parameter(
            'validacion').get_parameter_value().bool_value

        # Rutas parametrizables: permiten mover el nodo al PC sin tocar el código.
        # Los defaults apuntan a ~/mapeos/nav2/ (ruta histórica de la RPi).
        self.declare_parameter('waypoints_file', WAYPOINTS_FILE)
        self.declare_parameter('last_pose_file', LAST_POSE_FILE)
        self.declare_parameter('log_dir', LOG_DIR)
        self._waypoints_file = self.get_parameter(
            'waypoints_file').get_parameter_value().string_value
        self._last_pose_file = self.get_parameter(
            'last_pose_file').get_parameter_value().string_value
        self._log_dir = self.get_parameter(
            'log_dir').get_parameter_value().string_value

        self._logger_sesion = None
        self._monitor = None
        self._cierre_lock = threading.Lock()
        self._cerrado = False
        if self._validacion:
            self._logger_sesion = SesionLogger(self._log_dir, llm_model='')
            self._monitor = MonitorSistema(self._registrar_sistema, periodo=5.0)
            self._monitor.start()
            self.get_logger().info(
                '[VALIDACION] logger + monitor de sistema activos.')

        # Señaliza el desenlace de cada navegación (éxito/fallo/timeout) para que
        # una orden de validación pueda encadenar la ida y la vuelta.
        self._leg_evento = threading.Event()

        self._tts_activo = False
        self._emergencia = False
        self._navegando = False
        self._destino_actual = ''
        self._ubicacion = None
        self._navigator = None
        self._nav2_listo = False
        self._volviendo_a_base = False
        # Pose de origen para el flujo FETCH: se captura con 'registrar_origen'
        # (pose actual de AMCL) y se recupera con 'volver_a_origen' (sin acoplar).
        self._pose_origen = None

        # Waypoints (clave normalizada -> (x, y, yaw)). Se releen si cambia el mtime.
        self._waypoints = {}
        self._waypoints_mtime = 0.0
        self._cargar_waypoints()

        # Pose actual reportada por AMCL (para persistirla en disco).
        self._ultima_pose = None
        self._ultima_cov = 0.0          # traza de covarianza XY de AMCL (m²)
        self._pose_lock = threading.Lock()

        # Métricas de validación de la navegación en curso.
        self._t_nav_inicio = None       # monotonic() al enviar goToPose
        self._cov_max_nav = 0.0         # máx covarianza XY vista durante el trayecto

        # Dock status
        self._esta_acoplado = None
        self.create_subscription(
            DockStatus, '/dock_status', self._cb_dock_status, QOS_SENSOR)

        # ActionClients propios para dock/undock (independiente de movement_controller)
        self._cliente_dock = ActionClient(self, Dock, '/dock')
        self._cliente_undock = ActionClient(self, Undock, '/undock')
        self._undock_evento = threading.Event()
        self._dock_evento = threading.Event()

        # Localización dinámica: leer pose de AMCL y poder relocalizar
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._cb_amcl_pose, 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self._cliente_reloc = self.create_client(
            Empty, '/reinitialize_global_localization')

        self.create_subscription(Bool, '/tts_activo', self._cb_tts, 10)
        self.create_subscription(
            Bool, '/emergency/active', self._cb_emergencia, QOS_LATCHED)
        self.create_subscription(String, '/voice_command', self._cb_comando, 10)
        self.pub_estado = self.create_publisher(String, '/navegacion_estado', 10)
        # Estado machine-readable para coordinadores (flujo FETCH / emergencia):
        # 'llegada:<destino>', 'fallo:<destino>', 'cancelada'. El texto humano
        # sigue yendo a /navegacion_estado para que Ana lo hable.
        self.pub_resultado = self.create_publisher(
            String, '/navegacion_resultado', 10)
        # Métricas de validación por navegación (las consume asistente_node y las
        # vuelca al log de sesión para la Tabla 2.17 del TFM). JSON con destino,
        # exito, motivo, t_nav, err_pos, err_yaw_deg.
        self.pub_metricas = self.create_publisher(
            String, '/navegacion_metricas', 10)

        # Timer para monitorear progreso de navegación (1 Hz)
        self._timer_feedback = self.create_timer(1.0, self._check_navegacion)
        # Timer para persistir la última pose de AMCL en disco
        self._timer_pose = self.create_timer(POSE_SAVE_PERIOD, self._guardar_pose)

        # Inicializar Nav2 en hilo separado (waitUntilNav2Active bloquea)
        threading.Thread(target=self._init_nav2, daemon=True).start()

        if self._modo_terminal:
            threading.Thread(target=self._bucle_terminal, daemon=True).start()

        self.get_logger().info(
            f'Nodo navegacion iniciado. Destinos: {sorted(self._waypoints.keys())}'
            + (' [MODO TERMINAL]' if self._modo_terminal else ''))

    # ------------------------------------------------------------------
    # Waypoints (lectura en runtime de ~/mapeos/nav2/waypoints.yaml)
    # ------------------------------------------------------------------

    def _cargar_waypoints(self, forzar=False):
        """Lee waypoints.yaml si cambió el mtime. Claves normalizadas."""
        try:
            mtime = os.path.getmtime(self._waypoints_file)
        except OSError:
            if not self._waypoints:
                self.get_logger().error(
                    f'No existe {self._waypoints_file}; sin waypoints.')
            return
        if not forzar and mtime == self._waypoints_mtime:
            return
        try:
            with open(self._waypoints_file) as f:
                data = yaml.safe_load(f) or {}
            wps = data.get('waypoints') or {}
        except Exception as e:
            self.get_logger().error(f'Error leyendo {self._waypoints_file}: {e}')
            return
        nuevos = {}
        for nombre, wp in wps.items():
            try:
                nuevos[_normalizar(nombre)] = (
                    float(wp['x']), float(wp['y']), float(wp.get('yaw', 0.0)))
            except (KeyError, TypeError, ValueError):
                self.get_logger().warn(f'Waypoint "{nombre}" mal formado, ignorado.')
        self._waypoints = nuevos
        self._waypoints_mtime = mtime
        self.get_logger().info(
            f'Waypoints cargados: {sorted(self._waypoints.keys())}')

    # ------------------------------------------------------------------
    # Localización dinámica: persistir / restaurar pose de AMCL
    # ------------------------------------------------------------------

    def _cb_amcl_pose(self, msg):
        p = msg.pose.pose
        yaw = _yaw_from_quaternion(p.orientation)
        # Traza de covarianza XY (índices 0 y 7 de la matriz 6x6 fila-mayor).
        cov = msg.pose.covariance
        cov_xy = float(cov[0] + cov[7]) if len(cov) >= 8 else 0.0
        with self._pose_lock:
            self._ultima_pose = (p.position.x, p.position.y, yaw)
            self._ultima_cov = cov_xy

    def _guardar_pose(self):
        """Vuelca la última pose de AMCL a disco (throttled por el timer)."""
        with self._pose_lock:
            pose = self._ultima_pose
        if pose is None:
            return
        x, y, yaw = pose
        try:
            with open(self._last_pose_file, 'w') as f:
                yaml.dump(
                    {'x': round(x, 4), 'y': round(y, 4), 'yaw': round(yaw, 4),
                     'stamp': time.time()},
                    f, default_flow_style=False)
        except Exception as e:
            self.get_logger().warn(f'No pude guardar last_pose: {e}')

    def _leer_pose_guardada(self):
        try:
            with open(self._last_pose_file) as f:
                d = yaml.safe_load(f) or {}
            return (float(d['x']), float(d['y']), float(d['yaw']))
        except (OSError, KeyError, TypeError, ValueError):
            return None

    def _seleccionar_pose_inicial(self):
        """Pose inicial.

        Prioridad: waypoint de arranque forzado (start_waypoint, default 'salon',
        para que cada ensayo de validación parta de la misma pose) -> última
        guardada -> dock si acoplado -> central por defecto. Con start_waypoint:=''
        se omite el forzado y se recupera la persistencia entre sesiones.
        """
        if self._start_waypoint and self._start_waypoint in self._waypoints:
            pose = self._waypoints[self._start_waypoint]
            self.get_logger().info(
                f"Pose inicial forzada a waypoint '{self._start_waypoint}': {pose}")
            return pose
        if self._start_waypoint:
            self.get_logger().warn(
                f"start_waypoint '{self._start_waypoint}' no existe en waypoints.yaml; "
                'uso la cadena de persistencia.')
        pose = self._leer_pose_guardada()
        if pose is not None:
            self.get_logger().info(f'Pose inicial desde last_pose.yaml: {pose}')
            return pose
        if self._esta_acoplado is True and WAYPOINT_BASE in self._waypoints:
            pose = self._waypoints[WAYPOINT_BASE]
            self.get_logger().info(f'Pose inicial desde dock (waypoint base): {pose}')
            return pose
        self.get_logger().warn(
            f'Sin pose guardada ni dock; uso pose por defecto {DEFAULT_INITIAL_POSE}.')
        return DEFAULT_INITIAL_POSE

    def _bucle_terminal(self):
        """Lee comandos de stdin y los inyecta como si vinieran de /voice_command."""
        print('\n--- Modo terminal activo ---')
        print(f'Destinos: {sorted(self._waypoints.keys())}')
        if self._validacion:
            print('[VALIDACION] escribir un destino = ida y vuelta (salon<->destino), '
                  'ambas piernas registradas en el log.')
            print('Comandos: <destino> | valida:<destino> | volver_a_base | parar | '
                  'relocalizar | salir\n')
        else:
            print('Comandos: navegar:<destino> | volver_a_base | parar | relocalizar | salir\n')
        while True:
            try:
                cmd = input('nav> ').strip().lower()
            except (EOFError, KeyboardInterrupt):
                break
            if not cmd:
                continue
            if cmd == 'salir':
                break
            # Atajo: si escriben solo el nombre del destino, añadir prefijo. En
            # modo validación es una orden de ida y vuelta; en normal, navegar.
            if _normalizar(cmd) in self._waypoints:
                cmd = (f'valida:{cmd}' if self._validacion else f'navegar:{cmd}')
            msg = String()
            msg.data = cmd
            self._cb_comando(msg)

    # ------------------------------------------------------------------
    # Modo validación: registro de carga del sistema + orden de ida y vuelta
    # ------------------------------------------------------------------

    def _registrar_sistema(self, cpu_pct, mem_pct, temp_c, load1):
        """Callback de MonitorSistema -> evento 'sistema' en el log de sesión."""
        if self._logger_sesion is None:
            return
        self._logger_sesion.evento('sistema', cpu_pct=cpu_pct, mem_pct=mem_pct,
                            temp_c=temp_c, load1=load1)

    def _ejecutar_orden_validacion(self, destino):
        """Una orden de validación = ida (salon->destino) y vuelta (destino->salon).

        Cada pierna es una navegación real que pasa por _emitir_metrica_nav, de
        modo que ambas quedan registradas como ensayos de la Tabla 2.17.
        """
        for leg in (destino, self._start_waypoint):
            if not leg:
                continue
            self._leg_evento.clear()
            self._ejecutar_navegacion(leg)   # async: goToPose; el timer ve el fin
            if not self._leg_evento.wait(NAV_HARD_TIMEOUT + 15):
                self.get_logger().warn(
                    f'[VALIDACION] pierna a "{leg}" sin desenlace; abandono la orden.')
                return
            time.sleep(SETTLE_SEG)            # deja asentar AMCL antes de la siguiente
        self.get_logger().info(
            f'[VALIDACION] orden de ida y vuelta a "{destino}" completada.')

    def _init_nav2(self):
        self.get_logger().info('[DEBUG] Inicializando BasicNavigator...')
        self._navigator = BasicNavigator()
        # Dar un instante a que llegue /dock_status antes de elegir la pose.
        time.sleep(1.0)
        x, y, yaw = self._seleccionar_pose_inicial()
        self.get_logger().info(f'[DEBUG] setInitialPose: x={x}, y={y}, yaw={yaw}')
        self._navigator.setInitialPose(make_pose(self._navigator, x, y, yaw))
        self.get_logger().info('[DEBUG] Esperando Nav2 activo (waitUntilNav2Active)...')
        self._navigator.waitUntilNav2Active()
        self._nav2_listo = True
        self.get_logger().info('[DEBUG] Nav2 activo y listo.')

    def _cb_tts(self, msg):
        self._tts_activo = msg.data

    def _cb_emergencia(self, msg):
        """Durante la emergencia: cancela navegación, para el robot e ignora comandos."""
        if msg.data and not self._emergencia:
            self._emergencia = True
            if self._navegando and self._navigator is not None:
                try:
                    self._navigator.cancelTask()
                except Exception as e:
                    self.get_logger().warn(f'No pude cancelar navegación: {e}')
                self._navegando = False
                self._volviendo_a_base = False
            self.pub_cmd_vel.publish(Twist())  # parada inmediata
            self.get_logger().info('Emergencia activa: navegación pausada (robot detenido).')
        elif not msg.data and self._emergencia:
            self._emergencia = False
            self.get_logger().info('Emergencia finalizada: navegación reactivada.')

    def _cb_dock_status(self, msg):
        anterior = self._esta_acoplado
        self._esta_acoplado = msg.is_docked
        if anterior is None:
            self.get_logger().info(
                f'Dock: {"acoplado" if msg.is_docked else "desacoplado"}')
        # Señalar eventos de dock/undock
        if not msg.is_docked:
            self._undock_evento.set()
        if msg.is_docked:
            self._dock_evento.set()

    def _publicar_estado(self, texto):
        msg = String()
        msg.data = texto
        self.pub_estado.publish(msg)
        self.get_logger().info(f'Estado: {texto}')

    def _publicar_resultado(self, codigo):
        """Resultado machine-readable para coordinadores (no se habla)."""
        msg = String()
        msg.data = codigo
        self.pub_resultado.publish(msg)
        self.get_logger().info(f'Resultado: {codigo}')

    def _cb_comando(self, msg):
        dato = msg.data.strip().lower()
        self.get_logger().debug(f'[DEBUG] Comando recibido: "{dato}" | tts_activo={self._tts_activo} | nav2_listo={self._nav2_listo} | navegando={self._navegando} | acoplado={self._esta_acoplado}')

        # Durante la emergencia ignoramos toda orden de navegación.
        if self._emergencia and not self._modo_terminal:
            self.get_logger().info(f'[DEBUG] Ignorando "{dato}" (emergencia activa)')
            return

        # NOTA: no gatear por tts_activo. /voice_command sólo lo publica
        # asistente_node con comandos deliberados (navegar:/parar/…), nunca voz
        # ambiente; las acciones del LLM (p.ej. 'navegar:habitacion') se publican
        # justo cuando el TTS de la confirmación hablada ('Voy para allá') está
        # sonando, así que descartarlas por TTS activo rompía la navegación.

        # Cancelar navegación con 'parar'
        if dato == 'parar' and self._navegando:
            self.get_logger().info('[DEBUG] Cancelando navegacion...')
            self._navigator.cancelTask()
            self._navegando = False
            self._volviendo_a_base = False
            self._publicar_estado('Navegacion cancelada.')
            self._publicar_resultado('cancelada')
            return

        # Relocalización global (recuperación, sustituye al 2D Pose Estimate de RViz)
        if dato == 'relocalizar':
            threading.Thread(target=self._relocalizar, daemon=True).start()
            return

        # Registrar la pose actual como origen (flujo FETCH).
        if dato == 'registrar_origen':
            with self._pose_lock:
                self._pose_origen = self._ultima_pose
            if self._pose_origen is None:
                self.get_logger().warn(
                    '[DEBUG] registrar_origen sin pose de AMCL todavía.')
                self._publicar_resultado('fallo:origen')
            else:
                self.get_logger().info(
                    f'[DEBUG] Origen registrado: {self._pose_origen}')
                self._publicar_resultado('origen_ok')
            return

        # Volver a la pose de origen registrada (sin acoplar).
        if dato == 'volver_a_origen':
            threading.Thread(
                target=self._ejecutar_volver_a_origen, daemon=True).start()
            return

        # Volver a base
        if dato == 'volver_a_base':
            self.get_logger().info('[DEBUG] Iniciando volver_a_base en hilo...')
            threading.Thread(
                target=self._ejecutar_volver_a_base, daemon=True).start()
            return

        # Orden de validación: 'valida:destino' = ida y vuelta (ambas piernas
        # registradas). Sólo tiene sentido en modo validación.
        if dato.startswith('valida:'):
            destino = dato[len('valida:'):]
            threading.Thread(
                target=self._ejecutar_orden_validacion, args=(destino,),
                daemon=True).start()
            return

        # Comando de navegación: 'navegar:destino'
        if not dato.startswith('navegar:'):
            return

        destino = dato[len('navegar:'):]
        self.get_logger().info(f'[DEBUG] Iniciando navegacion a "{destino}" en hilo...')
        threading.Thread(
            target=self._ejecutar_navegacion, args=(destino,),
            daemon=True).start()

    # ------------------------------------------------------------------
    # Relocalización global + giro en sitio
    # ------------------------------------------------------------------

    def _relocalizar(self):
        self._publicar_estado('Voy a mirar a mi alrededor para ubicarme.')
        if self._cliente_reloc.wait_for_service(timeout_sec=2.0):
            self._cliente_reloc.call_async(Empty.Request())
            self.get_logger().info('[DEBUG] reinitialize_global_localization solicitado.')
        else:
            self.get_logger().warn('[DEBUG] Servicio de relocalizacion no disponible.')

        # Giro lento en sitio para que AMCL reparta y converja las partículas.
        twist = Twist()
        twist.angular.z = RELOC_SPIN_SPEED
        t0 = time.time()
        while time.time() - t0 < RELOC_SPIN_TIME:
            self.pub_cmd_vel.publish(twist)
            time.sleep(0.1)
        self.pub_cmd_vel.publish(Twist())  # parar
        self._publicar_estado('Ya creo saber dónde estoy.')

    # ------------------------------------------------------------------
    # Navegación con auto-undock
    # ------------------------------------------------------------------

    def _ejecutar_navegacion(self, destino):
        self._cargar_waypoints()  # recoge waypoints nuevos sin reiniciar
        destino = _normalizar(destino)
        self.get_logger().info(f'[DEBUG] _ejecutar_navegacion("{destino}") | nav2_listo={self._nav2_listo} | navegando={self._navegando} | acoplado={self._esta_acoplado} | ubicacion={self._ubicacion}')

        if not self._nav2_listo:
            self.get_logger().warn('[DEBUG] Nav2 NO listo, abortando.')
            self._publicar_estado('Navegacion no lista.')
            self._publicar_resultado(f'fallo:{destino}')
            self._leg_evento.set()
            return

        if destino not in self._waypoints:
            self.get_logger().warn(f'[DEBUG] Destino "{destino}" no esta en {sorted(self._waypoints.keys())}')
            self._publicar_estado(f'No conozco el sitio {destino}.')
            self._publicar_resultado(f'fallo:{destino}')
            self._leg_evento.set()
            return

        if destino == self._ubicacion and not self._navegando:
            self.get_logger().info(f'[DEBUG] Ya en destino "{destino}", no navego.')
            self._publicar_estado(f'Ya estoy en {destino}.')
            self._publicar_resultado(f'llegada:{destino}')
            self._leg_evento.set()
            return

        if self._navegando:
            self.get_logger().info('[DEBUG] Cancelando navegacion anterior...')
            self._navigator.cancelTask()

        # Auto-undock si está acoplado
        if self._esta_acoplado is True:
            self.get_logger().info('[DEBUG] Robot acoplado, desacoplando primero...')
            self._publicar_estado('Desacoplando para navegar.')
            if not self._desacoplar_sync():
                self._publicar_estado('No pude desacoplar.')
                self._publicar_resultado(f'fallo:{destino}')
                self._leg_evento.set()
                return
            self.get_logger().info('[DEBUG] Desacople OK, procediendo a navegar.')

        x, y, yaw = self._waypoints[destino]
        self._destino_actual = destino
        self._navegando = True
        self._t_nav_inicio = time.monotonic()
        self._cov_max_nav = 0.0
        pose = make_pose(self._navigator, x, y, yaw)
        self.get_logger().info(f'[DEBUG] goToPose -> x={x}, y={y}, yaw={yaw}')
        self._publicar_estado(f'Navegando a {destino}.')
        self._navigator.goToPose(pose)
        self.get_logger().info('[DEBUG] goToPose enviado, esperando feedback en timer...')

    # ------------------------------------------------------------------
    # Volver a base: navegar al waypoint base + acoplar
    # ------------------------------------------------------------------

    def _ejecutar_volver_a_base(self):
        if not self._nav2_listo:
            self._publicar_estado('Navegacion no lista.')
            return

        if self._esta_acoplado is True:
            self._publicar_estado('Ya estoy en la base.')
            return

        self._cargar_waypoints()
        self._volviendo_a_base = True
        self._publicar_estado('Volviendo a base.')

        # Navegar al waypoint base (si está definido). Si no, intentamos acoplar
        # directamente confiando en que el robot esté ya cerca del dock.
        if WAYPOINT_BASE in self._waypoints:
            x, y, yaw = self._waypoints[WAYPOINT_BASE]
            self._destino_actual = WAYPOINT_BASE
            self._navegando = True
            self._navigator.goToPose(make_pose(self._navigator, x, y, yaw))

            # Esperar a que llegue (polling en vez de callback para simplificar)
            while not self._navigator.isTaskComplete():
                rclpy.spin_once(self, timeout_sec=0.5)

            result = self._navigator.getResult()
            self._navegando = False

            if result != TaskResult.SUCCEEDED:
                self._volviendo_a_base = False
                if result == TaskResult.CANCELED:
                    self._publicar_estado('Navegacion cancelada.')
                else:
                    self._publicar_estado('No pude llegar a la base.')
                return

            self._ubicacion = WAYPOINT_BASE
        else:
            self.get_logger().warn(
                f'[DEBUG] No hay waypoint "{WAYPOINT_BASE}"; intento acoplar en sitio.')

        self._publicar_estado('Acoplando.')
        if self._acoplar_sync():
            self._publicar_estado('Acoplado en base.')
        else:
            self._publicar_estado('No pude acoplar.')
        self._volviendo_a_base = False

    # ------------------------------------------------------------------
    # Volver a la pose de origen (flujo FETCH): goToPose sin acoplar
    # ------------------------------------------------------------------

    def _ejecutar_volver_a_origen(self):
        if not self._nav2_listo:
            self._publicar_estado('Navegacion no lista.')
            self._publicar_resultado('fallo:origen')
            return

        if self._pose_origen is None:
            self.get_logger().warn('[DEBUG] volver_a_origen sin origen registrado.')
            self._publicar_estado('No tengo un punto de origen guardado.')
            self._publicar_resultado('fallo:origen')
            return

        # Auto-undock si está acoplado (poco probable a media tarea, por seguridad).
        if self._esta_acoplado is True:
            self._publicar_estado('Desacoplando para volver.')
            if not self._desacoplar_sync():
                self._publicar_estado('No pude desacoplar.')
                self._publicar_resultado('fallo:origen')
                return

        if self._navegando:
            self._navigator.cancelTask()

        x, y, yaw = self._pose_origen
        self._destino_actual = 'origen'
        self._navegando = True
        self._t_nav_inicio = time.monotonic()
        self._cov_max_nav = 0.0
        self._publicar_estado('Volviendo contigo.')
        self._navigator.goToPose(make_pose(self._navigator, x, y, yaw))
        # La finalización la detecta _check_navegacion (timer 1 Hz).

    # ------------------------------------------------------------------
    # Dock / Undock sincronos
    # ------------------------------------------------------------------

    def _desacoplar_sync(self):
        """Desacopla y espera hasta completar o timeout."""
        self.get_logger().info('[DEBUG] _desacoplar_sync: esperando servidor undock...')
        if not self._cliente_undock.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('[DEBUG] Servidor undock NO disponible (timeout 2s).')
            return False

        self.get_logger().info('[DEBUG] Servidor undock disponible, enviando goal...')
        self._undock_evento.clear()
        future = self._cliente_undock.send_goal_async(Undock.Goal())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'[DEBUG] Undock send_goal error: {e}')
            return False

        if not goal_handle.accepted:
            self.get_logger().warn('[DEBUG] Undock goal RECHAZADO.')
            return False

        self.get_logger().info('[DEBUG] Undock goal aceptado, esperando resultado...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=UNDOCK_TIMEOUT)

        try:
            result_future.result()
            self.get_logger().info('[DEBUG] Desacople completado OK.')
            self._esta_acoplado = False
            return True
        except Exception as e:
            self.get_logger().error(f'[DEBUG] Undock resultado fallo: {e}')
            return False

    def _acoplar_sync(self):
        """Acopla y espera hasta completar o timeout."""
        if not self._cliente_dock.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Servidor dock no disponible.')
            return False

        self._dock_evento.clear()
        future = self._cliente_dock.send_goal_async(Dock.Goal())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'Dock envio: {e}')
            return False

        if not goal_handle.accepted:
            self.get_logger().warn('Dock rechazado.')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=DOCK_TIMEOUT)

        try:
            result_future.result()
            self.get_logger().info('Acople completado.')
            self._esta_acoplado = True
            return True
        except Exception as e:
            self.get_logger().error(f'Dock fallo: {e}')
            return False

    # ------------------------------------------------------------------
    # Monitor de navegación (1 Hz)
    # ------------------------------------------------------------------

    def _check_navegacion(self):
        if not self._navegando or self._navigator is None:
            return

        # volver_a_base maneja su propio polling
        if self._volviendo_a_base:
            return

        # Acumular la peor covarianza de AMCL vista en el trayecto (pérdida loc.).
        with self._pose_lock:
            self._cov_max_nav = max(self._cov_max_nav, self._ultima_cov)

        if not self._navigator.isTaskComplete():
            # Watchdog de 120 s: superar el límite cuenta como fallo (timeout).
            if (self._t_nav_inicio is not None
                    and time.monotonic() - self._t_nav_inicio > NAV_HARD_TIMEOUT):
                destino = self._destino_actual
                self.get_logger().warn(
                    f'[DEBUG] Timeout {NAV_HARD_TIMEOUT:.0f}s navegando a {destino}; cancelo.')
                try:
                    self._navigator.cancelTask()
                except Exception as e:
                    self.get_logger().warn(f'No pude cancelar por timeout: {e}')
                self._navegando = False
                self._emitir_metrica_nav(destino, None, motivo_override='timeout')
                self._publicar_estado('He tardado demasiado, no he podido llegar.')
                self._publicar_resultado(f'fallo:{destino}')
                self._leg_evento.set()
                return
            feedback = self._navigator.getFeedback()
            if feedback:
                dist = feedback.distance_remaining
                self.get_logger().info(
                    f'[DEBUG] Feedback: {self._destino_actual} dist={dist:.2f}m')
            else:
                self.get_logger().info('[DEBUG] isTaskComplete=False pero feedback=None')
            return

        # Navegación completada
        self._navegando = False
        result = self._navigator.getResult()
        destino = self._destino_actual
        self.get_logger().info(f'[DEBUG] Navegacion terminada, result={result}')
        self._emitir_metrica_nav(destino, result)
        if result == TaskResult.SUCCEEDED:
            if destino != 'origen':
                self._ubicacion = destino
                self._publicar_estado(f'He llegado a {destino}.')
            else:
                self._publicar_estado('Ya estoy de vuelta.')
            self._publicar_resultado(f'llegada:{destino}')
        elif result == TaskResult.CANCELED:
            self._publicar_estado('Navegacion cancelada.')
            self._publicar_resultado('cancelada')
        else:
            self._publicar_estado('No puedo llegar.')
            self._publicar_resultado(f'fallo:{destino}')
        self._leg_evento.set()

    def _emitir_metrica_nav(self, destino, result, motivo_override=None):
        """Calcula y publica la métrica de validación de una navegación.

        Éxito = Nav2 SUCCEEDED y error de posición < ERR_POS_MAX y de orientación
        < ERR_YAW_MAX_DEG y tiempo < NAV_HARD_TIMEOUT. Se omite para la vuelta al
        origen del flujo FETCH y para destinos sin waypoint conocido (no hay pose
        objetivo contra la que medir el error). `result` None => fallo forzado
        (p.ej. timeout) con `motivo_override`.
        """
        if destino == 'origen' or destino not in self._waypoints:
            return
        t_nav = (time.monotonic() - self._t_nav_inicio
                 if self._t_nav_inicio is not None else -1.0)

        with self._pose_lock:
            pose_fin = self._ultima_pose
            cov_max = max(self._cov_max_nav, self._ultima_cov)
        x_obj, y_obj, yaw_obj = self._waypoints[destino]
        if pose_fin is not None:
            err_pos = math.hypot(pose_fin[0] - x_obj, pose_fin[1] - y_obj)
            d_yaw = pose_fin[2] - yaw_obj
            err_yaw_deg = abs(math.degrees(
                math.atan2(math.sin(d_yaw), math.cos(d_yaw))))
        else:
            err_pos = float('nan')
            err_yaw_deg = float('nan')

        if motivo_override:
            exito = False
            motivo = motivo_override
        elif result == TaskResult.CANCELED:
            exito = False
            motivo = 'cancelada'
        elif result != TaskResult.SUCCEEDED:
            # Nav2 abortó: costmap en impasse o (best-effort) colisión no evitada.
            exito = False
            motivo = 'abortado'
        else:
            dentro_tol = (pose_fin is not None
                          and err_pos < ERR_POS_MAX and err_yaw_deg < ERR_YAW_MAX_DEG)
            a_tiempo = 0 <= t_nav < NAV_HARD_TIMEOUT
            if cov_max > AMCL_COV_MAX:
                exito = False
                motivo = 'amcl'
            elif not a_tiempo:
                exito = False
                motivo = 'timeout'
            elif not dentro_tol:
                exito = False
                motivo = 'fuera_tolerancia'
            else:
                exito = True
                motivo = ''

        datos = {
            'destino': destino,
            'exito': exito,
            'motivo': motivo,
            't_nav': round(t_nav, 2),
            'err_pos': (round(err_pos, 3) if math.isfinite(err_pos) else None),
            'err_yaw_deg': (round(err_yaw_deg, 1)
                            if math.isfinite(err_yaw_deg) else None),
            'cov_max': round(cov_max, 4),
        }
        self.get_logger().info(f'[METRICA NAV] {datos}')
        msg = String()
        msg.data = json.dumps(datos)
        self.pub_metricas.publish(msg)
        # En modo validación (Nav2 aislado) no hay asistente_node que recoja el
        # topic: registramos la métrica directamente en nuestro propio log.
        if self._logger_sesion is not None:
            self._logger_sesion.evento('navegacion_metrica', **datos)

    def _cerrar_logging(self):
        """Para el monitor y cierra el logger. Idempotente (Ctrl-C/SIGTERM/destroy)."""
        with self._cierre_lock:
            if self._cerrado:
                return
            self._cerrado = True
        if self._monitor is not None:
            self._monitor.stop()
        if self._logger_sesion is not None:
            self._logger_sesion.cerrar()

    def destroy_node(self):
        # Persistir la última pose conocida antes de cerrar.
        self._guardar_pose()
        self._cerrar_logging()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    nodo = NodoNavegacion()

    # Garantizar el cierre del log de validación pase lo que pase: Ctrl-C,
    # SIGTERM (ros2 launch) o excepción. _cerrar_logging es idempotente.
    atexit.register(nodo._cerrar_logging)

    def _on_sigterm(signum, frame):
        nodo._cerrar_logging()
        raise KeyboardInterrupt

    try:
        signal.signal(signal.SIGTERM, _on_sigterm)
    except (ValueError, OSError):
        # signal.signal sólo funciona en el hilo principal.
        pass

    # IMPORTANTE: usar un executor EXPLÍCITO (no el global de rclpy) para que el
    # hilo _init_nav2 pueda llamar rclpy.spin_until_future_complete (que usa el
    # executor global) sin colisionar con nuestro spin. Si usáramos rclpy.spin()
    # ambos competirían por el mismo executor global → "Executor is already spinning".
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(nodo)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        nodo._cerrar_logging()
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
