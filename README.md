# Asistente robótico de voz "Ana"

Asistente conversacional de voz para asistencia a personas mayores, desarrollado como
Trabajo Fin de Máster. Funciona sobre un **TurtleBot 4** (Raspberry Pi 4B) con **ROS 2
Jazzy** y navegación autónoma con **Nav2**.

El robot escucha la palabra de activación "ana", transcribe las órdenes en español,
responde por voz con modulación emocional y es capaz de desplazarse de forma autónoma
por el entorno, reconocer objetos con la cámara y avisar a un familiar en caso de
emergencia.

## Funcionalidades

- **Reconocimiento de voz (STT)** offline con Vosk (modelo `vosk-model-small-es-0.42`).
- **Diálogo** mediante un modelo de lenguaje servido por OpenRouter
  (`google/gemini-2.5-flash-lite`), con respuestas cortas y de baja latencia.
- **Síntesis de voz (TTS)** con Piper. Por defecto la síntesis se delega en un servidor
  remoto en el PC (que aplica la modulación emocional); si no hay servidor disponible, la
  Raspberry recurre a Piper local como respaldo.
- **Navegación autónoma** con Nav2: navegación a destinos con nombre (entrada, salón,
  cocina), acoplamiento/desacoplamiento de la base y vuelta a la base.
- **Visión** con YOLO para responder a "¿qué ves?" (carga perezosa para ahorrar memoria).
- **Emergencias**: ante una situación de peligro descrita por el usuario, el sistema
  realiza una llamada con Twilio a un familiar o cuidador, con audio bidireccional.

## Arquitectura de despliegue

El sistema se reparte entre dos máquinas conectadas a la misma red ROS 2 (mismo
`ROS_DOMAIN_ID`):

| Componente | Máquina |
|---|---|
| `asistente_node` (STT + LLM + TTS + emergencias) | Raspberry Pi 4B |
| `bridge_twilio_emergencia` (puente de audio de la llamada) | Raspberry Pi 4B |
| `object_detector_node` (visión YOLO) | Raspberry Pi 4B |
| Servidor TTS remoto | PC |
| Pila Nav2 completa (AMCL, planners, controllers) | PC |
| `nodo_navegacion_node` (waypoints con `BasicNavigator`) | PC |
| `tf_relay_node` (puente de QoS para TF) | PC |

El código fuente del paquete ROS 2 (`voice_controlled_turtlebot`) es común a ambas
máquinas; cada una arranca solo los nodos que le corresponden.

## Requisitos

- Ubuntu 24.04
- ROS 2 Jazzy Jalisco
- Python 3
- TurtleBot 4 (Raspberry Pi 4B) y un PC en la misma red para la navegación y el TTS remoto

## Instalación

```bash
git clone <url-del-repositorio>
cd asistente_ros-separacion-nav-pc
./install.sh
```

`install.sh` instala dependencias, descarga los modelos (Vosk, Piper) y prepara el
enlace del paquete en el workspace de colcon.

A continuación hay que crear los ficheros de configuración a partir de las plantillas:

```bash
cp secrets.env.example secrets.env            # claves de OpenRouter y Twilio
cp perfil_residente.json.example perfil_residente.json   # datos del residente
```

Edita `secrets.env` y rellena al menos `OPENROUTER_API_KEY` (obligatoria). Las
credenciales de Twilio son opcionales: sin ellas el sistema funciona igual, pero no
realiza las llamadas de emergencia. Ninguno de estos dos ficheros se versiona (están en
`.gitignore`).

## Uso

En la Raspberry Pi 4B:

```bash
./lanzar.sh             # voz + visión
./lanzar.sh nav:=true   # voz + visión + Nav2
```

En el PC (navegación y TTS remoto):

```bash
./lanzar_nav_pc.sh                      # navegación con los mapas de ~/mapeos/
./lanzar_nav_pc.sh map:=/ruta/mapa.yaml # usar otro mapa
```

## Estructura

```
.
├── src/voice_controlled_turtlebot/   Paquete ROS 2 (nodos, launch, mapas, tests)
├── bridge_twilio_emergencia.py       Puente de audio de las llamadas de emergencia
├── contexto_LLM.txt                  Contexto del sistema para el modelo de lenguaje
├── contexto_emergencia.txt           Protocolo de emergencia
├── objetos.yaml                      Objetos reconocibles por la visión
├── install.sh                        Script de instalación
├── lanzar.sh / lanzar_nav_pc.sh      Scripts de arranque (RPi / PC)
├── sesiones_de_ejemplo/              Sesiones de ejemplo del asistente (formato JSONL + resumen)
├── herramientas/                     Utilidades (p. ej. diagnóstico de Twilio)
├── PARAMETROS.md                     Parámetros configurables del nodo principal
└── tts_remoto_pc.md                  Guía del servidor TTS remoto
```

## Documentación adicional

- [`PARAMETROS.md`](PARAMETROS.md): parámetros de configuración del asistente.
- [`tts_remoto_pc.md`](tts_remoto_pc.md): montaje del servidor TTS remoto en el PC.
- [`sesiones_de_ejemplo/README.md`](sesiones_de_ejemplo/README.md): formato y contenido de las sesiones de ejemplo.
