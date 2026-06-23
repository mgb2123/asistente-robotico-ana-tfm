#!/usr/bin/env bash
# Lanza el subsistema de navegación en el PC remoto.
# No carga secrets.env: la navegación no usa OpenRouter ni Twilio.
# ROS_DOMAIN_ID y RMW_IMPLEMENTATION se heredan del shell (no se tocan aquí).
#
# Uso:
#   ./lanzar_nav_pc.sh                           # defaults de ~/mapeos/
#   ./lanzar_nav_pc.sh map:=/ruta/mapa.yaml      # sobreescribir args
#   ./lanzar_nav_pc.sh log_dir:=/mnt/nfs/logs    # apuntar logs a carpeta compartida
set -e
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 1) Entorno ROS (no toca ROS_DOMAIN_ID: se hereda del shell actual)
source /opt/ros/jazzy/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"

# 2) Lanzar (reenvía todos los args: map:=..., log_dir:=..., etc.)
exec ros2 launch voice_controlled_turtlebot navegacion_pc.launch.py "$@"
