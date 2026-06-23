#!/usr/bin/env bash
# install.sh — bootstrap del asistente Ana en TurtleBot4 (Raspberry Pi 4B, ROS2 Jazzy).
#
# Deja el sistema en estado "listo para lanzar":
#   - Modelos Vosk (STT) y Piper (TTS femenino daniela es_AR) en ~/asistente_turtlebot4/models/
#   - Binario piper en ~/.local/bin/piper
#   - Dependencias Python (vosk, openai, ultralytics, twilio, numpy)
#   - Dependencias ROS2 (cv_bridge, nav2_simple_commander) vía apt
#   - Symlink ~/asistente_turtlebot4 → este repo (para que las rutas hardcoded funcionen)
#
# Idempotente: cada paso comprueba si ya está hecho.

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HOME_LINK="$HOME/asistente_turtlebot4-main"
MODELS_DIR="$HOME_LINK/models"
PIPER_DIR="$MODELS_DIR/piper"
VOSK_NAME="vosk-model-small-es-0.42"
VOSK_DIR="$MODELS_DIR/$VOSK_NAME"
PIPER_VOICE="es_AR-daniela-high"
PIPER_HF_PATH="es/es_AR/daniela/high"
PIPER_ONNX="$PIPER_DIR/${PIPER_VOICE}.onnx"
PIPER_JSON="$PIPER_DIR/${PIPER_VOICE}.onnx.json"
PIPER_BIN_DIR="$HOME/.local/share/piper"
PIPER_BIN="$HOME/.local/bin/piper"

# Detectar arquitectura para descargar la build correcta de piper
ARCH="$(uname -m)"
case "$ARCH" in
    aarch64) PIPER_TGZ="piper_linux_aarch64.tar.gz" ;;
    armv7l)  PIPER_TGZ="piper_linux_armv7l.tar.gz" ;;
    x86_64)  PIPER_TGZ="piper_linux_x86_64.tar.gz" ;;
    *) echo "Arquitectura no soportada: $ARCH"; exit 1 ;;
esac
PIPER_TAG="2023.11.14-2"
PIPER_URL="https://github.com/rhasspy/piper/releases/download/${PIPER_TAG}/${PIPER_TGZ}"

step() { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }
ok()   { printf '\033[1;32m    OK\033[0m %s\n' "$*"; }
skip() { printf '\033[1;33m    --\033[0m %s (ya existe)\n' "$*"; }

# ---------------------------------------------------------------- 1. SYMLINK
step "Symlink ~/asistente_turtlebot4-main → $REPO_DIR"
if [ "$(readlink -f "$HOME_LINK" 2>/dev/null)" = "$REPO_DIR" ]; then
    skip "el repo ya está en $HOME_LINK (no hace falta symlink)"
elif [ -L "$HOME_LINK" ]; then
    ln -sfn "$REPO_DIR" "$HOME_LINK"
    ok "symlink actualizado"
elif [ -e "$HOME_LINK" ]; then
    echo "ERROR: $HOME_LINK existe y NO es un symlink. Mueve o borra ese directorio antes de continuar."
    exit 1
else
    ln -s "$REPO_DIR" "$HOME_LINK"
    ok "symlink creado"
fi

# -------------------------------------------------------------- 2. APT DEPS
step "Paquetes APT (alsa-utils, sox, opencv, cv_bridge, nav2_simple_commander, curl, unzip)"
APT_PKGS=(
    alsa-utils
    sox
    python3-pip
    python3-opencv
    ros-jazzy-cv-bridge
    ros-jazzy-nav2-simple-commander
    curl
    unzip
)
MISSING=()
for p in "${APT_PKGS[@]}"; do
    dpkg -s "$p" >/dev/null 2>&1 || MISSING+=("$p")
done
if [ ${#MISSING[@]} -eq 0 ]; then
    skip "todos los paquetes apt presentes"
elif sudo -n true 2>/dev/null; then
    sudo apt-get update
    sudo apt-get install -y "${MISSING[@]}"
    ok "instalados: ${MISSING[*]}"
else
    cat <<EOF >&2

\033[1;33m    !!\033[0m sudo requiere contraseña; no puedo instalar paquetes apt aquí.
    Ejecuta MANUALMENTE en otra terminal:

        sudo apt-get update && sudo apt-get install -y ${MISSING[*]}

    Y luego vuelve a lanzar este script para continuar el resto.
EOF
    APT_PENDING=1
fi

# -------------------------------------------------------------- 3. PIP DEPS
step "Paquetes Python (numpy LOCKED 1.26.4 + vosk, openai, ultralytics, twilio, piper-tts)"
if ! command -v pip3 >/dev/null 2>&1; then
    echo "    !! pip3 no disponible; instala python3-pip primero (paso APT)."
    PIP_PENDING=1
else
    PIP_FLAGS="--user"
    # Pi 4B con Python >=3.11 (PEP 668) requiere --break-system-packages
    if python3 -c 'import sys; sys.exit(0 if sys.version_info >= (3,11) else 1)'; then
        PIP_FLAGS="$PIP_FLAGS --break-system-packages"
    fi

    # NumPy DEBE quedarse en 1.26.4: cv_bridge de ROS Jazzy está compilado
    # contra NumPy 1.x y revienta con 2.x. El procedimiento en 3 pasos
    # evita que un --upgrade posterior (p.ej. de ultralytics) lo suba.

    # PASO 1 — desinstalar cualquier numpy previo (puede ser 2.x heredado)
    pip3 uninstall -y numpy 2>/dev/null || true
    pip3 uninstall -y --break-system-packages numpy 2>/dev/null || true

    # PASO 2 — instalar numpy 1.26.4 PRIMERO con --force-reinstall
    pip3 install $PIP_FLAGS --force-reinstall 'numpy==1.26.4'

    # PASO 3 — resto de paquetes con --constraint que IMPIDE subir numpy
    CONSTRAINTS="$(mktemp)"
    echo 'numpy==1.26.4' > "$CONSTRAINTS"
    pip3 install $PIP_FLAGS --upgrade --constraint "$CONSTRAINTS" \
        vosk openai ultralytics twilio piper-tts pyngrok websockets
    rm -f "$CONSTRAINTS"

    # PASO 4 — verificación final
    INSTALLED_NP="$(python3 -c 'import numpy; print(numpy.__version__)')"
    if [[ "$INSTALLED_NP" != 1.* ]]; then
        echo "ERROR: numpy quedó en $INSTALLED_NP, se esperaba 1.x"
        exit 1
    fi
    ok "numpy bloqueado en $INSTALLED_NP + resto de pip3 install completado"

    # PASO 5 — si se lanzó con sudo, el --user anterior instaló paquetes en /root/.local
    # y el usuario real no los encontraría. Reinstalamos solo pyngrok+websockets para él.
    if [ "$EUID" -eq 0 ] && [ -n "${SUDO_USER:-}" ]; then
        REAL_PIP_FLAGS="--user"
        sudo -u "$SUDO_USER" python3 -c 'import sys; sys.exit(0 if sys.version_info >= (3,11) else 1)' \
            && REAL_PIP_FLAGS="$REAL_PIP_FLAGS --break-system-packages" || true
        sudo -u "$SUDO_USER" pip3 install $REAL_PIP_FLAGS pyngrok websockets 2>/dev/null || true
        ok "pyngrok y websockets instalados también para $SUDO_USER"
    fi
fi

# -------------------------------------------------------------- 4. PIPER BIN
step "Binario piper ($PIPER_BIN)"
if command -v piper >/dev/null 2>&1; then
    skip "piper ya en PATH ($(command -v piper))"
else
    mkdir -p "$PIPER_BIN_DIR" "$HOME/.local/bin"
    TMP_TGZ="$(mktemp --suffix=.tar.gz)"
    echo "Descargando $PIPER_URL"
    curl -L --fail -o "$TMP_TGZ" "$PIPER_URL"
    tar -xzf "$TMP_TGZ" -C "$PIPER_BIN_DIR" --strip-components=1
    rm -f "$TMP_TGZ"
    ln -sf "$PIPER_BIN_DIR/piper" "$PIPER_BIN"
    ok "piper instalado en $PIPER_BIN"
fi

# ------------------------------------------------------------ 5. VOSK MODEL
step "Modelo Vosk $VOSK_NAME en $MODELS_DIR"
mkdir -p "$MODELS_DIR"
if [ -d "$VOSK_DIR" ]; then
    skip "$VOSK_DIR"
else
    TMP_ZIP="$(mktemp --suffix=.zip)"
    curl -L --fail -o "$TMP_ZIP" "https://alphacephei.com/vosk/models/${VOSK_NAME}.zip"
    unzip -q "$TMP_ZIP" -d "$MODELS_DIR"
    rm -f "$TMP_ZIP"
    ok "descargado y extraído"
fi

# ----------------------------------------------------------- 6. PIPER VOICE
step "Voz Piper $PIPER_VOICE (femenina, es_AR, high quality)"
mkdir -p "$PIPER_DIR"
HF_BASE="https://huggingface.co/rhasspy/piper-voices/resolve/main/${PIPER_HF_PATH}"
if [ -f "$PIPER_ONNX" ] && [ -f "$PIPER_JSON" ]; then
    skip "voz ya descargada en $PIPER_DIR"
else
    curl -L --fail -o "$PIPER_ONNX" "$HF_BASE/${PIPER_VOICE}.onnx"
    curl -L --fail -o "$PIPER_JSON" "$HF_BASE/${PIPER_VOICE}.onnx.json"
    ok "voz descargada"
fi

# ------------------------------------------------------------- 7. YOLO PESO
step "Peso YOLO (yolov8n.pt)"
YOLO_SRC="$HOME_LINK/yolov8n.pt"
YOLO_DST="$HOME/yolov8n.pt"
if [ ! -f "$YOLO_SRC" ]; then
    echo "ERROR: $YOLO_SRC no existe; ¿está el repo completo?"
    exit 1
fi
if [ -L "$YOLO_DST" ] || [ ! -e "$YOLO_DST" ]; then
    ln -sf "$YOLO_SRC" "$YOLO_DST"
    ok "symlink $YOLO_DST → $YOLO_SRC"
else
    skip "$YOLO_DST ya existe (no es symlink, lo dejo)"
fi

# ---------------------------------------------------- 8. PRE-WARM PIPER ONNX
step "Pre-warm Piper (cachea ONNX en disco)"
if [ -x "$PIPER_BIN" ] || command -v piper >/dev/null 2>&1; then
    PIPER_CMD="$(command -v piper || echo "$PIPER_BIN")"
    echo "lista" | "$PIPER_CMD" --model "$PIPER_ONNX" --output_raw > /dev/null 2>&1 || true
    ok "pre-warm completado"
else
    echo "Aviso: piper no ejecutable; salta pre-warm."
fi

# ---------------------------------------------------------- 9. MEMORIA FILE
step "Archivo de memoria (si no existe)"
MEM_FILE="$HOME_LINK/memoria.json"
if [ ! -f "$MEM_FILE" ]; then
    echo '{}' > "$MEM_FILE"
    ok "memoria.json inicializada"
else
    skip "memoria.json ya existe"
fi

# ------------------------------------------------------------ MENSAJE FINAL
cat <<'EOF'

============================================================
  Instalación COMPLETADA
============================================================

Antes de lanzar el asistente, exporta esta variable de entorno
(puedes añadirla a ~/.bashrc):

  export OPENROUTER_API_KEY="sk-or-..."

  (Para las llamadas de emergencia, exporta también las credenciales Twilio:
   TWILIO_SID, TWILIO_TOKEN, TWILIO_FROM, TWILIO_TO)

Después:

  ln -sf ~/asistente_turtlebot4/src/voice_controlled_turtlebot ~/ros2_ws/src/
  cd ~/ros2_ws
  colcon build --symlink-install --packages-select voice_controlled_turtlebot
  source install/setup.bash
  ros2 launch voice_controlled_turtlebot voice_controlled_turtlebot.launch.py

Para activar Nav2:
  ros2 launch voice_controlled_turtlebot voice_controlled_turtlebot.launch.py nav:=true

============================================================
EOF
