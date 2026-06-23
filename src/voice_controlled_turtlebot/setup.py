from setuptools import find_packages, setup

nombre_paquete = 'voice_controlled_turtlebot'

setup(
    name=nombre_paquete,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + nombre_paquete]),
        ('share/' + nombre_paquete, ['package.xml']),
        ('share/voice_controlled_turtlebot/launch', [
            'launch/voice_controlled_turtlebot.launch.py',
            'launch/validacion_nav.launch.py',
            'launch/navegacion_pc.launch.py',
        ]),
        ('share/voice_controlled_turtlebot/maps', [
            'map_name.yaml',
            'map_name.pgm',
        ]),
    ],
    install_requires=['setuptools', 'numpy<2'],
    zip_safe=True,
    maintainer='m',
    maintainer_email='todo@udc.es',
    description='Asistente robótico autónomo con interacción afectiva — TFM UDC',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'asistente_node = voice_controlled_turtlebot.asistente_node:main',
            'object_detector_node = voice_controlled_turtlebot.object_detector_node:main',
            'nodo_navegacion_node = voice_controlled_turtlebot.nodo_navegacion_node:main',
            'tf_relay_node = voice_controlled_turtlebot.tf_relay_node:main',
        ],
    },
)
