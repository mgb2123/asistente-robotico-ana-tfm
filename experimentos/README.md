# Experimentos — Resultados finales del TFM

Herramientas de **captura y análisis** para ejecutar el `plan_experimentos_tfm`
(3 sesiones, ~75 min, cobertura del 100 % de los requisitos). Yo no conduzco el
robot: estos scripts convierten los logs de cada sesión en las tablas y figuras
que van a la memoria.

| Script | Sesión | Cubre |
|--------|--------|-------|
| `analizar_sesion.py` | 1 — Conversación afectiva | RF-04/05/06, RNF-01, Métricas 3/4/5 |
| `wer.py` | 1 — calidad STT | RF-04 |
| `extraer_caso_uso.py` | 2 — Casos de uso | RF-01/02/03/07/08, RNF-02, Métrica 1 |
| `evaluar_yolo.py` | 3 — YOLO 3 condiciones | RF-03, RNF-03, Métrica 2 |

Todos los scripts son stdlib salvo `evaluar_yolo.py` (usa `ultralytics`/`cv2`).
Los resultados se vuelcan en `experimentos/resultados/` (en `.gitignore`).

---

## Antes de empezar: capturar el texto de la conversación

El análisis de **coherencia afectiva** (Sesión 1) necesita el texto real de
pregunta/respuesta y la emoción. Se añadió al evento `llm` del logger. Para que
surta efecto hay que **recompilar** el paquete antes de grabar la Sesión 1:

```bash
cd ~/ros2_ws && colcon build --symlink-install \
  --packages-select voice_controlled_turtlebot
```

Cada ejecución del asistente escribe `logs_sesiones/sesion_<fecha>_<pid>.jsonl`.

---

## Sesión 1 — Conversación afectiva

1. Lanza el asistente y mantén una conversación de 10-15 turnos con 3 registros
   emocionales (neutral / preocupado / urgente), como indica el plan.
2. Analiza la sesión (por defecto, la más reciente de `logs_sesiones/`):

```bash
python3 experimentos/analizar_sesion.py \
  --salida experimentos/resultados/sesion1
```

Genera:
- `sesion1.md` — tabla de latencias por turno, resumen **P50/P90**, comprobación
  de la **Métrica 4** (latencia de respuesta < 3 s: nº y % de turnos que cumplen)
  y 2-3 **ejemplos de intercambio** por registro emocional para valorar la
  coherencia afectiva.
- `sesion1_turnos.csv` — todos los turnos (para Excel/figuras).
- `sesion1_transcripciones.csv` — para el WER (columna `referencia` vacía).

3. **WER (RF-04):** rellena la columna `referencia` del CSV con lo que dijiste de
   verdad en cada turno y ejecuta:

```bash
python3 experimentos/wer.py experimentos/resultados/sesion1_transcripciones.csv
```

> La **latencia de respuesta** = `STT→1er token LLM` + `LLM→1er audio`, es decir
> el tiempo desde que dejas de hablar hasta que Ana empieza a sonar. Es la que se
> compara con el umbral de 3 s de la Métrica 4 / RNF-01.

---

## Sesión 2 — Demo integrada de los dos casos de uso

La Sesión 2 es una **demo en vivo**; el éxito/fallo por etapa se anota a mano.

1. Lanza con navegación: `ros2 launch ... voice_controlled_turtlebot.launch.py nav:=true`.
2. Ejecuta los intentos (búsqueda colaborativa ×3, emergencia ×2-3) y anota en
   la plantilla `plantillas/sesion2_casos_uso.csv` (cópiala a `resultados/`).
3. Corrobora y data los eventos automáticos (navegación lanzada, emergencia
   Twilio con su SID, fallback, errores) extrayéndolos del log:

```bash
python3 experimentos/extraer_caso_uso.py \
  --salida experimentos/resultados/sesion2.md
```

Combina la línea de tiempo automática con tu plantilla manual para la tabla
`caso / intento / resultado / tiempo total` de la memoria. Acuérdate de las
**capturas/vídeo** del log de ROS2 que pide el plan.

---

## Sesión 3 — YOLO en 3 condiciones

Análisis **offline** de imágenes. Organiza las fotos así (el nombre de la
carpeta de clase es la etiqueta esperada; admite español o inglés):

```
experimentos/dataset_validacion/
  A_ideal/        botella/ *.jpg   silla/ *.jpg   ...
  B_angulo/       botella/ *.jpg   ...
  C_iluminacion/  botella/ *.jpg   ...
```

```bash
python3 experimentos/evaluar_yolo.py \
  --datos experimentos/dataset_validacion \
  --salida experimentos/resultados/sesion3
```

Genera la tabla `clase × condición → acierto/fallo`, un CSV de detalle y las
imágenes anotadas con prefijo `OK_/FALLO_`. Una imagen es acierto si el modelo
detecta (conf ≥ 0.5, igual que el robot) al menos una caja de la clase esperada.

- `--model best.pt` para usar el **modelo entrenado de 6 clases** del TFM (cópialo
  del PC; en la RPi solo está el `yolov8n.pt` genérico de 80 clases COCO).
- `--conf 0.5` por defecto (mismo umbral que `object_detector_node`).

> **Importante:** ejecuta la Sesión 3 en el **PC de entrenamiento**, no en la
> RPi. El `torch` instalado en la Pi (`2.12.0+cu130`, build CUDA) crashea con
> `Illegal instruction` (SIGILL) al hacer inferencia en el Cortex-A72. Es el
> mismo motivo por el que conviene revisar si "ana, ¿qué ves?" funciona hoy en
> el robot. En el PC (con un `torch` de CPU/GPU correcto) el script corre sin
> tocar nada.

---

## Cobertura de requisitos

| Sesión | Requisitos |
|--------|------------|
| 1 — Conversación afectiva | RF-04, RF-05, RF-06, RNF-01, Métricas 3, 4, 5 |
| 2 — Demo casos de uso | RF-01, RF-02, RF-03*, RF-07, RF-08, RNF-02, Métrica 1 |
| 3 — YOLO 3 condiciones | RF-03, RNF-03, Métrica 2 |

\* RF-03 en integración real (complementa a las métricas offline del dataset).
