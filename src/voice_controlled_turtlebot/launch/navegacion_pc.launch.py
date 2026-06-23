"""
navegacion_pc.launch.py — Subsistema de navegación para ejecutar en el PC remoto.

Levanta en el PC: tf_relay_node + localization (AMCL) + Nav2 + nodo_navegacion_node.
El stack de voz (asistente_node, object_detector_node, bridge_twilio_emergencia)
sigue corriendo en la RPi con voice_controlled_turtlebot.launch.py.

Requisitos en el PC:
  - ROS 2 Jazzy instalado y sourced
  - Paquetes: turtlebot4_navigation, nav2_simple_commander, irobot_create_msgs
  - Este paquete compilado: colcon build --packages-select voice_controlled_turtlebot
  - Directorio ~/mapeos/ sincronizado desde la RPi (mapa + params + waypoints)
  - ROS_DOMAIN_ID y RMW_IMPLEMENTATION idénticos a los de la RPi (heredados del shell)

Uso:
  ./lanzar_nav_pc.sh                          # defaults de ~/mapeos/
  ./lanzar_nav_pc.sh map:=/ruta/mapa.yaml     # sobreescribir args
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    mapeos = os.path.join(os.path.expanduser('~'), 'mapeos')
    mapeos_nav = os.path.join(mapeos, 'nav2')

    # ── Args del mapa y parámetros Nav2 (mismos nombres/defaults que en el launch RPi) ──
    map_arg = DeclareLaunchArgument(
        'map', default_value=os.path.join(mapeos, 'maps', 'casa_2d.yaml'),
        description='Mapa (.yaml) para map_server / AMCL')
    loc_params_arg = DeclareLaunchArgument(
        'loc_params', default_value=os.path.join(mapeos_nav, 'localization.yaml'),
        description='Params de localización (AMCL)')
    nav_params_arg = DeclareLaunchArgument(
        'nav_params', default_value=os.path.join(mapeos_nav, 'nav2_params.yaml'),
        description='Params de Nav2 (planners, controllers, docking)')

    # ── Args de rutas de archivos del nodo de navegación ──
    waypoints_file_arg = DeclareLaunchArgument(
        'waypoints_file', default_value=os.path.join(mapeos_nav, 'waypoints.yaml'),
        description='Archivo YAML con waypoints nombrados')
    last_pose_file_arg = DeclareLaunchArgument(
        'last_pose_file', default_value=os.path.join(mapeos_nav, 'last_pose.yaml'),
        description='Archivo YAML donde se persiste la última pose de AMCL')
    log_dir_arg = DeclareLaunchArgument(
        'log_dir',
        default_value=os.path.join(
            os.path.expanduser('~'), 'asistente_turtlebot4-main', 'logs_sesiones'),
        description='Directorio de logs de sesión (métricas de navegación Tabla 2.17)')

    mapa = LaunchConfiguration('map')
    loc_params = LaunchConfiguration('loc_params')
    nav_params = LaunchConfiguration('nav_params')

    tb4_nav_share = get_package_share_directory('turtlebot4_navigation')

    # TF relay: re-publica odom->base_link de BEST_EFFORT a RELIABLE para Nav2.
    # Necesario porque el Create 3 publica TF con QoS BEST_EFFORT y Nav2 subscribe
    # RELIABLE; sin este nodo AMCL y los costmaps no reciben el árbol TF.
    tf_relay = Node(
        package='voice_controlled_turtlebot',
        executable='tf_relay_node',
        name='tf_relay_node',
        output='screen',
    )

    localizacion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb4_nav_share, 'launch', 'localization.launch.py')
        ),
        launch_arguments={'map': mapa, 'params': loc_params}.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb4_nav_share, 'launch', 'nav2.launch.py')
        ),
        launch_arguments={'params_file': nav_params}.items(),
    )

    nodo_nav = Node(
        package='voice_controlled_turtlebot',
        executable='nodo_navegacion_node',
        name='nodo_navegacion_node',
        output='screen',
        parameters=[{
            'waypoints_file': LaunchConfiguration('waypoints_file'),
            'last_pose_file': LaunchConfiguration('last_pose_file'),
            'log_dir':        LaunchConfiguration('log_dir'),
        }],
    )

    return LaunchDescription([
        map_arg,
        loc_params_arg,
        nav_params_arg,
        waypoints_file_arg,
        last_pose_file_arg,
        log_dir_arg,
        tf_relay,
        localizacion,
        nav2,
        nodo_nav,
    ])
