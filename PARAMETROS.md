# Parámetros ajustables del pipeline — Ana (TurtleBot4)

Todos los valores están en `asistente_node.py`, sección de constantes (líneas ~70–145).
Después de cambiar cualquier constante **relanza el nodo**; no es necesario recompilar.

---

## Audio de entrada (micrófono / STT)

| Parámetro | Valor actual | Subir | Bajar |
|---|---|---|---|
| `UMBRAL_RMS_NORMAL` | `90` | Ignora sonidos más débiles — menos falsos positivos por ruido ambiente, pero puede perder voz baja | Recoge voz más débil — más sensible al ruido de fondo |
| `UMBRAL_RMS_BARGE_IN_MULT` | `2.0` | Hace más difícil interrumpir a Ana mientras habla — reduce eco; puede ignorar "ana" dicho cerca del altavoz | Permite interrumpir más fácilmente — más riesgo de que el eco del altavoz active el barge-in |
| `SILENCIO_FIN` | `7` (× 100 ms = 700 ms) | Ana espera más silencio antes de cerrar la frase — captura frases más pausadas, pero añade latencia | Corta antes la escucha — útil si las frases son cortas y claras; puede truncar frases largas |
| `MAX_GRABACION_SEG` | `5` s | Permite frases más largas | Recorta frases largas al límite; útil para forzar comandos cortos |
| `CHUNK_SEG` | `0.1` s | Procesa bloques más grandes — menos carga CPU, más latencia mínima de detección | Bloques más pequeños — reactividad más alta, más overhead |

---

## Conversación y diálogo

| Parámetro | Valor actual | Subir | Bajar |
|---|---|---|---|
| `TIMEOUT_CONVERSACION` | `25.0` s | Ana permanece "atenta" más tiempo tras hablar — conveniente pero consume recursos si el usuario se distrae | Se cierra antes la sesión; el usuario tiene que decir "Ana" más pronto |
| `POST_TTS_MUTE_SEC` | `1.8` s | El micrófono permanece silenciado más tiempo tras acabar Ana — menos eco, pero el usuario tarda más en poder hablar | Abre el micro antes — puede coger el fin del eco del altavoz y confundirlo con voz |
| `EARLY_UNMUTE_BEFORE_END_SEC` | `0.5` s | Se abre el micro bastante antes de que Ana acabe — mayor riesgo de eco; útil si el usuario quiere interrumpir | Se abre el micro justo al acabar — más seguro frente al eco, pero corta ligeramente la capacidad de barge-in |
| `HISTORIAL_MAX_PARES` | `3` pares | Más contexto de conversación al LLM — respuestas más coherentes, más tokens de entrada y coste/latencia | Menos contexto — el LLM "olvida" antes; reduce tokens y latencia |
| `HISTORIAL_TIMEOUT` | `30` s | El historial se mantiene más tiempo entre turnos antes de borrarse | El historial se limpia antes si el usuario tarda en responder |

---

## LLM (OpenRouter)

| Parámetro | Valor actual | Subir | Bajar |
|---|---|---|---|
| `MAX_TOKENS` | `100` | Respuestas más largas — útil para preguntas complejas; más latencia hasta el primer audio | Respuestas más cortas y rápidas; puede truncar respuestas que necesitan más espacio |
| `TEMPERATURE` | `0.7` | Más creatividad y variedad — respuestas más espontáneas pero a veces menos precisas | Más determinismo — respuestas más predecibles y directas |
| `FLUSH_MIN_CHARS` | `60` chars | Acumula más texto antes de enviarlo a Piper — frases más completas y naturales; mayor latencia al primer audio | Envía texto a Piper antes — menor latencia al primer audio; puede crear pausas extrañas si corta en medio de una idea |
| `FLUSH_HARD_CAP_CHARS` | `400` chars | Permite acumular bloques muy grandes antes del flush forzado — para frases LLM muy largas | Fuerza el flush antes aunque no haya punto — útil para frases sin puntuación |

---

## TTS — velocidad y prosodia (Piper)

Estos parámetros están dentro de `_sintetizar()`, en el bloque `SynthesisConfig(...)`.

| Parámetro | Valor actual | Subir | Bajar |
|---|---|---|---|
| `length_scale` | `1.25` | Habla más lenta — más pausada y comprensible, pero tardará más en terminar | Habla más rápida; por debajo de `0.9` puede sonar apresurada |
| `noise_scale` | `0.32` | Más variación en pronunciación de fonemas — puede sonar más natural o menos estable | Pronunciación más uniforme y "limpia" |
| `noise_w_scale` | `1.3` | Más variación de entonación y ritmo — voz más expresiva y menos robótica | Entonación más plana y monótona |

---

## TTS — silencios entre segmentos

| Parámetro | Valor actual | Subir | Bajar |
|---|---|---|---|
| `PAUSA_COMA_MS` | `120` ms | Pausa más larga tras coma/punto y coma/dos puntos — más natural pero más lenta | Pausa más breve; puede sonar más apresurada en listas largas |
| `PAUSA_PUNTO_MS` | `200` ms | Pausa más larga al final de frase — separa bien las frases, da sensación de calma | Pausa más corta al final; útil si la respuesta tiene muchas frases |

---

## Emociones de voz (arousal / valence)

`AROUSAL_DEFAULT = 0.6` y `VALENCE_DEFAULT = 0.8` son los valores base que usa el LLM
para las respuestas de conversación. Los comandos rápidos tienen sus propios pares en `EMOCION_RAPIDA`.

| Parámetro | Efecto |
|---|---|
| **arousal** más alto | Voz más intensa/activa (enérgica) |
| **arousal** más bajo | Voz más calmada y suave |
| **valence** más alto | Tono más positivo/alegre |
| **valence** más bajo | Tono más neutro o serio |

> Nota: el efecto real de arousal/valence depende del modelo Piper. Con `es_MX-ald-medium` el impacto es moderado; con `es_AR-daniela-high` suele ser más perceptible.

---

## Movimiento

| Parámetro | Valor actual | Efecto |
|---|---|---|
| `DURACION_MOVIMIENTO` | `2.0` s | Tiempo que dura cada comando de movimiento rápido (adelante, atrás, girar…). Subir = más desplazamiento por comando |
