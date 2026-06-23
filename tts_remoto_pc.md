# Servidor TTS remoto — pasos en el PC (Ubuntu 24.04)

Este servidor sintetiza voz con Piper en el PC y publica el PCM a la RPi4
por ROS 2. La RPi4 ya está preparada: si detecta que este servidor está
suscrito a `/tts_request`, le pasa el texto y reproduce el audio que llega
por `/tts_audio`. Si el servidor no está corriendo, la RPi4 cae a Piper
local automáticamente.

Asume que ROS 2 ya está instalado, las variables (`ROS_DOMAIN_ID`,
`RMW_IMPLEMENTATION`) están alineadas entre PC y RPi4, y los topics
`/tts_request` `/tts_audio` `/tts_cancel` se ven desde ambos lados.

---

## 1. Instalar Piper y onnxruntime

```bash
pip install --user --break-system-packages piper-tts onnxruntime
```

(En el PC sí se puede usar NumPy 2.x sin problemas; sólo la RPi4 está
limitada a 1.x por `cv_bridge`.)

## 2. Descargar el modelo

```bash
mkdir -p ~/tts_server/models
cd ~/tts_server/models

# Voz masculina, ligera (la que usa la RPi4 hoy)
wget https://huggingface.co/rhasspy/piper-voices/resolve/main/es/es_MX/ald/medium/es_MX-ald-medium.onnx
wget https://huggingface.co/rhasspy/piper-voices/resolve/main/es/es_MX/ald/medium/es_MX-ald-medium.onnx.json

# (Opcional) Voz femenina más natural, mucho más pesada de sintetizar.
# Ahora que el TTS corre en el PC, esta voz vuelve a ser viable.
# wget https://huggingface.co/rhasspy/piper-voices/resolve/main/es/es_AR/daniela/high/es_AR-daniela-high.onnx
# wget https://huggingface.co/rhasspy/piper-voices/resolve/main/es/es_AR/daniela/high/es_AR-daniela-high.onnx.json
```

Si quieres usar `daniela-high`, edita `PIPER_MODEL` en el script del paso 3.

## 3. Guardar el script del servidor

Crea `~/tts_server/tts_server_node.py` con este contenido:

```python
#!/usr/bin/env python3
"""tts_server_node — Sintesis Piper remota para asistente_node de la RPi4.

Suscribe /tts_request (String), sintetiza con PiperVoice y publica chunks
PCM (S16_LE mono 22050 Hz) a /tts_audio (UInt8MultiArray). data vacia =
fin de frase. Cancelable via /tts_cancel (Empty).
"""
import os
import queue
import re
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, String, UInt8MultiArray

PIPER_MODEL = os.path.expanduser(
    '~/tts_server/models/es_MX-ald-medium.onnx')
TTS_SAMPLE_RATE = 22050
PAUSA_COMA_MS = 120
PAUSA_PUNTO_MS = 200
MAX_CHUNK_BYTES = 32768  # cap conservador para DDS sobre WiFi

QOS = QoSProfile(
    depth=50,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


class TtsServer(Node):
    def __init__(self):
        super().__init__('tts_server_node')
        from piper import PiperVoice, SynthesisConfig
        self._SynthesisConfig = SynthesisConfig
        self.get_logger().info(f'Cargando {PIPER_MODEL}...')
        self._voice = PiperVoice.load(PIPER_MODEL)
        list(self._voice.synthesize('Hola.'))  # pre-warm
        self.get_logger().info('Piper listo.')

        self._pub_audio = self.create_publisher(
            UInt8MultiArray, '/tts_audio', QOS)
        self.create_subscription(
            String, '/tts_request', self._cb_request, QOS)
        self.create_subscription(
            Empty, '/tts_cancel', self._cb_cancel, QOS)

        self._req_queue = queue.Queue()
        self._cancel = threading.Event()
        threading.Thread(target=self._worker, daemon=True).start()

    def _cb_request(self, msg):
        self._req_queue.put(msg.data)

    def _cb_cancel(self, _msg):
        # Marca cancelacion (rompe la sintesis en curso) y vacia pendientes.
        # Publica un end marker por cada item descartado para que la RPi4
        # mantenga su contador de frases pendientes consistente. El end marker
        # de la frase en curso lo emite el worker al salir de _sintetizar.
        self._cancel.set()
        descartados = 0
        while True:
            try:
                self._req_queue.get_nowait()
                descartados += 1
            except queue.Empty:
                break
        for _ in range(descartados):
            self._publicar_fin()
        self.get_logger().info(
            f'Cancel: {descartados} pendientes purgados.')

    def _publicar_chunk(self, data: bytes):
        msg = UInt8MultiArray()
        msg.data = list(data)
        self._pub_audio.publish(msg)

    def _publicar_fin(self):
        msg = UInt8MultiArray()
        msg.data = []
        self._pub_audio.publish(msg)

    def _worker(self):
        while rclpy.ok():
            texto = self._req_queue.get()
            self._cancel.clear()
            try:
                self._sintetizar(texto)
            except Exception as e:
                self.get_logger().warn(f'Sintesis error: {e}')
            self._publicar_fin()

    def _sintetizar(self, texto: str):
        syn = self._SynthesisConfig(
            length_scale=0.92, noise_scale=0.36, noise_w_scale=1.3)
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
            ms_fin = PAUSA_PUNTO_MS if texto[-1:] in '.!?' else 0
            partes.append((buf.strip(), ms_fin))

        for parte, pausa_ms in partes:
            if self._cancel.is_set():
                return
            for chunk in self._voice.synthesize(parte, syn_config=syn):
                if self._cancel.is_set():
                    return
                data = chunk.audio_int16_bytes
                for i in range(0, len(data), MAX_CHUNK_BYTES):
                    if self._cancel.is_set():
                        return
                    self._publicar_chunk(data[i:i + MAX_CHUNK_BYTES])
            if pausa_ms > 0:
                n_bytes = int(TTS_SAMPLE_RATE * pausa_ms / 1000) * 2
                silencio = bytes(n_bytes)
                for i in range(0, len(silencio), MAX_CHUNK_BYTES):
                    if self._cancel.is_set():
                        return
                    self._publicar_chunk(silencio[i:i + MAX_CHUNK_BYTES])


def main():
    rclpy.init()
    node = TtsServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 4. Ejecutar

```bash
python3 ~/tts_server/tts_server_node.py
```

Deja la terminal abierta mientras uses el robot. `Ctrl+C` para parar.

## 5. Verificación rápida

Con `tts_server_node` corriendo en el PC y `asistente_node` corriendo en la RPi4:

```bash
# En cualquiera de las dos maquinas
ros2 topic info /tts_request    # 1 publisher (RPi4) + 1 subscriber (PC)
ros2 topic info /tts_audio      # 1 publisher (PC)   + 1 subscriber (RPi4)
```

Dile `"ana"` al robot. En el log de la RPi4 **NO** debe aparecer
`Cargando Piper local`; eso confirma que está usando el servidor remoto.

Si el log de la RPi4 dice `Cargando Piper local`, es que la RPi4 no
ve al servidor: revisa que `ROS_DOMAIN_ID` y `RMW_IMPLEMENTATION`
coinciden y que el PC ve los topics con `ros2 topic list`.

## Cambiar a la voz femenina (daniela-high)

1. Descarga `es_AR-daniela-high.onnx` y su `.json` (descomenta los `wget` del paso 2).
2. Cambia en el script:
   ```python
   PIPER_MODEL = os.path.expanduser(
       '~/tts_server/models/es_AR-daniela-high.onnx')
   ```
3. Relanza `tts_server_node.py`. Ningún cambio en la RPi4 (mismo rate 22050).
