"""
voice_controlled_turtlebot.launch.py — Launch a bordo (RPi 4B).

Lanza ÚNICAMENTE los nodos que corren en la RPi:
  - bridge_twilio_emergencia (audio bidireccional vía Twilio Media Streams + ngrok)
  - asistente_node  (voz, STT, LLM, TTS, movimiento directo, emergencias)
  - object_detector_node  (visión YOLO, lazy-load)

El subsistema de navegación (tf_relay, localization/AMCL, Nav2, nodo_navegacion_node)
corre ahora en el PC remoto con navegacion_pc.launch.py.
"""

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    # Bridge de audio bidireccional para emergencias (Twilio Media Streams).
    # Abre un túnel ngrok automáticamente y escribe la URL en /tmp/bridge_url.txt
    # para que gestor_emergencia.py la use al realizar la llamada.
    # Requiere: pip install pyngrok websockets  y  ngrok config add-authtoken <token>
    bridge_script = os.path.join(
        os.path.expanduser('~'), 'asistente_turtlebot4-main',
        'bridge_twilio_emergencia.py')
    bridge_emergencia = ExecuteProcess(
        cmd=['python3', bridge_script, '--port', '8765'],
        name='bridge_twilio_emergencia',
        output='screen',
    )

    # asistente_node fusiona: voice_pipeline + movement + nodo_dialogo + nodo_emergencias.
    # object_detector queda separado por su ciclo de vida distinto (lazy YOLO).
    nodos_voz = [
        Node(package='voice_controlled_turtlebot', executable='asistente_node',
             name='asistente_node', output='screen'),
        Node(package='voice_controlled_turtlebot', executable='object_detector_node',
             name='object_detector_node', output='screen'),
    ]

    return LaunchDescription([
        bridge_emergencia,
        *nodos_voz,
    ])
