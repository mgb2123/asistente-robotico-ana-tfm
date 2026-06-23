#!/usr/bin/env bash
# Carga secretos + entorno ROS y lanza el asistente.
#   ./lanzar.sh             -> solo voz/visión
#   ./lanzar.sh nav:=true   -> voz + Nav2
set -e
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 1) Secretos
if [ ! -f "$DIR/secrets.env" ]; then
    echo "ERROR: falta $DIR/secrets.env"
    echo "  Crea uno:  cp secrets.env.example secrets.env  &&  nano secrets.env"
    exit 1
fi
source "$DIR/secrets.env"
if [ -z "$OPENROUTER_API_KEY" ]; then          # if-form (no '&&'): set -e no mata aquí
    echo "AVISO: OPENROUTER_API_KEY vacío; el LLM fallará (el resto funciona)."
fi

# 2) Entorno ROS (no toca ROS_DOMAIN_ID: se hereda del shell actual)
source /opt/ros/jazzy/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"

# 3) Lanzar (reenvía args: nav:=true, etc.)
exec ros2 launch voice_controlled_turtlebot voice_controlled_turtlebot.launch.py "$@"
