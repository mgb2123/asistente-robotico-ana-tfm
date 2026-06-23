"""
object_detector_node — YOLO solo cuando se pide por voz ("ana mira", "ana que ves").

En reposo: NO suscrito a la cámara. 0% CPU.
Al recibir 'ver' en /voice_command: se suscribe a /oakd/rgb/preview/image_raw,
captura 1 frame, se desuscribe y ejecuta YOLO. Publica lo que ve en
/detected_objects.

Al recibir 'buscar:<clase>' en /voice_command (flujo FETCH del coordinador de
tareas): mantiene la cámara abierta y corre YOLO en bucle hasta que aparece la
clase COCO pedida (en inglés, p.ej. 'cup') con confianza >= UMBRAL_BUSQUEDA o
hasta agotar DETECTION_TIMEOUT. Publica un resultado machine-readable en
/deteccion_resultado: 'encontrado:<clase>' o 'no_encontrado:<clase>'.
"""

import gc
import os
import threading
import time

YOLO_MODEL = os.path.expanduser('~/asistente_turtlebot4-main/models/best.pt')

import numpy as _np_check
if not _np_check.__version__.startswith('1.'):
    raise RuntimeError(
        f'NumPy {_np_check.__version__} detectado. cv_bridge de ROS Jazzy '
        f'requiere NumPy 1.x. Ejecuta:\n'
        f'  pip3 uninstall -y numpy && '
        f'pip3 install --user --break-system-packages --force-reinstall '
        f'numpy==1.26.4\n'
        f'o relanza install.sh.')

import rclpy
from rclpy.node import Node
from rclpy.qos import (qos_profile_sensor_data, QoSProfile,
                       ReliabilityPolicy, DurabilityPolicy)
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
import cv2
# ultralytics se importa lazy dentro de _ejecutar_deteccion la primera vez
# que llega el comando 'ver'. Ahorra ~2 s de arranque y ~200 MB de RAM si
# nunca se usa visión.

TIMEOUT_FRAME = 1.0       # s esperando el primer frame tras suscribirse
UMBRAL_DETECCION = 0.5    # confianza mínima YOLO (comando 'ver')
UMBRAL_BUSQUEDA = 0.5     # confianza mínima para dar por encontrada la clase buscada
DETECTION_TIMEOUT = 30.0  # s de búsqueda activa antes de rendirse (flujo FETCH)
BUSQUEDA_PERIODO = 0.3    # s entre inferencias YOLO sucesivas durante la búsqueda

# QoS latcheado para /emergency/active (debe coincidir con el publisher de
# asistente_node): TRANSIENT_LOCAL para recibir el estado actual al suscribirse.
QOS_LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


YOLO_ES = {
    # Clases custom del fine-tuning
    'botella_termica': 'botella térmica',
    'agenda_favorita': 'agenda favorita',
    'pesa': 'pesa',
    'Antonio': 'Antonio',
    'Maria': 'María',
    # Clases COCO estándar
    'person': 'persona', 'bicycle': 'bicicleta', 'car': 'coche', 'motorcycle': 'moto',
    'airplane': 'avión', 'bus': 'autobús', 'train': 'tren', 'truck': 'camión',
    'boat': 'barco', 'traffic light': 'semáforo', 'fire hydrant': 'hidrante',
    'stop sign': 'señal de stop', 'parking meter': 'parquímetro', 'bench': 'banco',
    'bird': 'pájaro', 'cat': 'gato', 'dog': 'perro', 'horse': 'caballo',
    'sheep': 'oveja', 'cow': 'vaca', 'elephant': 'elefante', 'bear': 'oso',
    'zebra': 'cebra', 'giraffe': 'jirafa', 'backpack': 'mochila', 'umbrella': 'paraguas',
    'handbag': 'bolso', 'tie': 'corbata', 'suitcase': 'maleta', 'frisbee': 'frisbee',
    'skis': 'esquís', 'snowboard': 'snowboard', 'sports ball': 'pelota',
    'kite': 'cometa', 'baseball bat': 'bate', 'baseball glove': 'guante',
    'skateboard': 'monopatín', 'surfboard': 'tabla de surf', 'tennis racket': 'raqueta',
    'bottle': 'botella', 'wine glass': 'copa', 'cup': 'taza', 'fork': 'tenedor',
    'knife': 'cuchillo', 'spoon': 'cuchara', 'bowl': 'cuenco', 'banana': 'plátano',
    'apple': 'manzana', 'sandwich': 'sándwich', 'orange': 'naranja', 'broccoli': 'brócoli',
    'carrot': 'zanahoria', 'hot dog': 'perrito caliente', 'pizza': 'pizza',
    'donut': 'donut', 'cake': 'pastel', 'chair': 'silla', 'couch': 'sofá',
    'potted plant': 'planta', 'bed': 'cama', 'dining table': 'mesa',
    'toilet': 'inodoro', 'tv': 'televisor', 'laptop': 'portátil', 'mouse': 'ratón',
    'remote': 'mando', 'keyboard': 'teclado', 'cell phone': 'móvil',
    'microwave': 'microondas', 'oven': 'horno', 'toaster': 'tostadora',
    'sink': 'fregadero', 'refrigerator': 'nevera', 'book': 'libro',
    'clock': 'reloj', 'vase': 'jarrón', 'scissors': 'tijeras',
    'teddy bear': 'peluche', 'hair drier': 'secador', 'toothbrush': 'cepillo de dientes',
}


class NodoDetectorObjetos(Node):
    def __init__(self):
        super().__init__('object_detector_node')

        self.puente_cv = CvBridge()
        self.modelo = None  # lazy: se carga en _ejecutar_deteccion

        self._ultimo_frame = None
        self._inferencia_en_curso = False
        self._lock = threading.Lock()
        self._sub_imagen = None  # se crea on-demand

        self._tts_activo = False
        # Modo emergencia: liberamos YOLO (~200 MB) e ignoramos comandos para
        # dejar RAM/CPU a la llamada de emergencia.
        self._emergencia = False
        self.create_subscription(Bool, '/tts_activo', self._cb_tts, 10)
        self.create_subscription(
            Bool, '/emergency/active', self._cb_emergencia, QOS_LATCHED)
        self.create_subscription(
            String, '/voice_command', self.cb_comando, 10)

        self.pub_imagen  = self.create_publisher(Image,  '/yolo_image_raw', qos_profile_sensor_data)
        self.pub_objetos = self.create_publisher(String, '/detected_objects', 10)
        # Resultado machine-readable de la búsqueda activa (flujo FETCH).
        self.pub_busqueda = self.create_publisher(String, '/deteccion_resultado', 10)

        self.get_logger().info(
            f'YOLO_MODEL: {YOLO_MODEL} (existe={os.path.exists(YOLO_MODEL)})'
        )

        # Pre-warm YOLO en background para que el primer 'ver' no espere.
        def _prewarm():
            try:
                self._asegurar_modelo()
            except Exception as e:
                self.get_logger().error(f'Pre-calentamiento YOLO fallido: {e}')

        threading.Thread(target=_prewarm, daemon=True).start()

        self.get_logger().info('Detector iniciado (pre-cargando YOLO en background).')

    def cb_imagen(self, msg):
        try:
            frame = self.puente_cv.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self._lock:
                self._ultimo_frame = frame
        except Exception as e:
            self.get_logger().error(f'Error imagen: {e}')

    def _cb_tts(self, msg):
        self._tts_activo = msg.data

    def _cb_emergencia(self, msg):
        """Pausa/reanuda el detector según el modo emergencia de asistente_node."""
        if msg.data and not self._emergencia:
            self._emergencia = True
            # Liberar YOLO de la RAM (solo si no hay inferencia en curso).
            if not self._inferencia_en_curso and self.modelo is not None:
                self.modelo = None
                gc.collect()
                self.get_logger().info('YOLO liberado por emergencia (RAM liberada).')
            else:
                self.get_logger().info(
                    'Emergencia activa: detector en pausa (ignoro comandos).')
        elif not msg.data and self._emergencia:
            self._emergencia = False
            self.get_logger().info('Emergencia finalizada: re-precargando YOLO.')
            threading.Thread(target=self._asegurar_modelo, daemon=True).start()

    def cb_comando(self, msg):
        if self._emergencia:
            return
        dato = msg.data.strip()

        # Búsqueda activa de una clase concreta (flujo FETCH). El coordinador
        # orquesta el momento, así que NO se filtra por _tts_activo.
        if dato.startswith('buscar:'):
            if self._inferencia_en_curso:
                self.get_logger().warn('buscar: inferencia ya en curso, ignoro.')
                return
            clase = dato[len('buscar:'):].strip().lower()
            if not clase:
                return
            threading.Thread(
                target=self._buscar_objeto, args=(clase,), daemon=True).start()
            return

        # 'ver': descripción puntual de lo que ve (no durante el TTS).
        if dato != 'ver':
            return
        if self._tts_activo:
            return
        if self._inferencia_en_curso:
            return
        threading.Thread(target=self._capturar_y_detectar, daemon=True).start()

    def _asegurar_modelo(self):
        """Carga YOLO la primera vez (lazy)."""
        if self.modelo is None:
            try:
                from ultralytics import YOLO
                self.modelo = YOLO(YOLO_MODEL)
                self.get_logger().info('YOLO cargado (lazy).')
            except Exception as e:
                self.modelo = None
                self.get_logger().error(f'Error cargando YOLO desde {YOLO_MODEL}: {e}')

    def _buscar_objeto(self, clase):
        """Mantiene la cámara abierta y corre YOLO hasta hallar `clase` o timeout.

        `clase` es la clase COCO en inglés (p.ej. 'cup', 'bottle', 'cell phone').
        Publica 'encontrado:<clase>' o 'no_encontrado:<clase>' en /deteccion_resultado.
        """
        self._inferencia_en_curso = True
        encontrado = False
        try:
            self._asegurar_modelo()
            with self._lock:
                self._ultimo_frame = None
            self._sub_imagen = self.create_subscription(
                Image, '/oakd/rgb/preview/image_raw',
                self.cb_imagen, qos_profile_sensor_data)

            self.get_logger().info(
                f'Búsqueda activa de "{clase}" (timeout {DETECTION_TIMEOUT:.0f}s).')
            t0 = time.time()
            while time.time() - t0 < DETECTION_TIMEOUT:
                with self._lock:
                    frame = self._ultimo_frame
                if frame is None:
                    time.sleep(0.05)
                    continue
                try:
                    resultados = self.modelo.predict(
                        source=frame, conf=UMBRAL_BUSQUEDA, verbose=False)[0]
                    for caja in resultados.boxes:
                        nombre = self.modelo.names[int(caja.cls[0])].lower()
                        if nombre == clase and float(caja.conf[0]) >= UMBRAL_BUSQUEDA:
                            encontrado = True
                            break
                except Exception as e:
                    self.get_logger().error(f'Error YOLO en búsqueda: {e}')
                if encontrado:
                    break
                time.sleep(BUSQUEDA_PERIODO)
        finally:
            if self._sub_imagen is not None:
                self.destroy_subscription(self._sub_imagen)
                self._sub_imagen = None
            self._inferencia_en_curso = False

        msg = String()
        msg.data = f'{"encontrado" if encontrado else "no_encontrado"}:{clase}'
        self.pub_busqueda.publish(msg)
        self.get_logger().info(msg.data)

    def _capturar_y_detectar(self):
        """Suscribe a la cámara, espera 1 frame, se desuscribe y ejecuta YOLO."""
        self._inferencia_en_curso = True
        try:
            with self._lock:
                self._ultimo_frame = None
            self._sub_imagen = self.create_subscription(
                Image, '/oakd/rgb/preview/image_raw',
                self.cb_imagen, qos_profile_sensor_data)

            t0 = time.time()
            frame = None
            while time.time() - t0 < TIMEOUT_FRAME:
                with self._lock:
                    frame = self._ultimo_frame
                if frame is not None:
                    break
                time.sleep(0.05)

            if self._sub_imagen is not None:
                self.destroy_subscription(self._sub_imagen)
                self._sub_imagen = None

            if frame is None:
                self.get_logger().warn(
                    f'No llegó frame de cámara en {TIMEOUT_FRAME}s — '
                    '¿está corriendo el driver OAK-D?'
                )
                msg = String()
                msg.data = 'No estoy viendo nada ahora mismo.'
                self.pub_objetos.publish(msg)
                return

            self._ejecutar_deteccion(frame)
        finally:
            self._inferencia_en_curso = False

    def _ejecutar_deteccion(self, frame):
        try:
            self._asegurar_modelo()
            resultados = self.modelo.predict(
                source=frame, conf=UMBRAL_DETECCION, verbose=False)[0]

            etiquetas = []
            frame_anotado = frame.copy()

            for caja in resultados.boxes:
                clase    = int(caja.cls[0])
                etiqueta_en = self.modelo.names[clase]
                etiqueta = YOLO_ES.get(etiqueta_en, etiqueta_en)
                conf     = float(caja.conf[0])
                etiquetas.append(etiqueta)

                x1, y1, x2, y2 = map(int, caja.xyxy[0])
                cv2.rectangle(frame_anotado, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame_anotado, f'{etiqueta} {conf:.2f}',
                            (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 255, 0), 2)

            texto = (f'Veo: {", ".join(set(etiquetas))}'
                     if etiquetas else 'No veo nada.')

            try:
                self.pub_imagen.publish(
                    self.puente_cv.cv2_to_imgmsg(frame_anotado, encoding='bgr8'))
            except Exception as e:
                self.get_logger().error(f'Error publicando imagen: {e}')

            msg = String()
            msg.data = texto
            self.pub_objetos.publish(msg)
            self.get_logger().info(texto)
        except Exception as e:
            self.get_logger().error(f'Error detección: {e}')


def main(args=None):
    rclpy.init(args=args)
    nodo = NodoDetectorObjetos()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
