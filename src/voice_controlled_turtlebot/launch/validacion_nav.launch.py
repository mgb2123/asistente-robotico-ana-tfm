"""
validacion_nav.launch.py — Nav2 AISLADO para la validación de navegación del TFM.

Levanta SÓLO la maquinaria de navegación (tf_relay + localización AMCL + Nav2),
sin un solo nodo de voz (ni asistente_node, ni object_detector, ni el bridge de
emergencias). El objetivo es medir la navegación (Tabla 2.17) sin la contaminación
de carga del stack completo, que satura la RPi 4B (CPU 100 %, load 28-49).

El nodo_navegacion_node NO se incluye aquí a propósito: se ejecuta aparte en una
terminal con stdin limpio para el modo interactivo de validación, p.ej.

  ros2 run voice_controlled_turtlebot nodo_navegacion_node --ros-args \
      -p validacion:=true -p modo_terminal:=true -p start_waypoint:=salon

En ese modo, escribir un destino en el prompt 'nav>' dispara una ida y vuelta
(salon -> destino -> salon) y registra ambas piernas en logs_sesiones/.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Mismos mapa y params afinados que usa el lanzamiento completo (viven en ~/mapeos).
    mapeos = os.path.join(os.path.expanduser('~'), 'mapeos')
    map_arg = DeclareLaunchArgument(
        'map', default_value=os.path.join(mapeos, 'maps', 'casa_2d.yaml'),
        description='Mapa (.yaml) para map_server / AMCL')
    loc_params_arg = DeclareLaunchArgument(
        'loc_params', default_value=os.path.join(mapeos, 'nav2', 'localization.yaml'),
        description='Params de localizacion (AMCL)')
    nav_params_arg = DeclareLaunchArgument(
        'nav_params', default_value=os.path.join(mapeos, 'nav2', 'nav2_params.yaml'),
        description='Params de Nav2 (planners, controllers, docking)')

    mapa = LaunchConfiguration('map')
    loc_params = LaunchConfiguration('loc_params')
    nav_params = LaunchConfiguration('nav_params')

    tb4_nav_share = get_package_share_directory('turtlebot4_navigation')

    # TF relay: re-publica odom->base_link de BEST_EFFORT a RELIABLE para Nav2.
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

    return LaunchDescription([
        map_arg,
        loc_params_arg,
        nav_params_arg,
        tf_relay,
        localizacion,
        nav2,
    ])
