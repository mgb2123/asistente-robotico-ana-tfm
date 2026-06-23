# Sesiones de ejemplo

Muestra de 5 sesiones reales registradas por el asistente, incluidas como referencia del
formato de log que analizan los scripts de `experimentos/`. El resto de sesiones se generan
en ejecución y no se versionan (`logs_sesiones/` está en `.gitignore`).

Cada sesión tiene dos ficheros:

- `sesion_<fecha>_<pid>.jsonl`: un evento JSON por línea (inicio de sesión, comandos,
  conversaciones, métricas de latencia por turno, barge-ins, navegación, emergencias…).
- `sesion_<fecha>_<pid>_resumen.txt`: resumen legible de la sesión (duración, número de
  turnos, estadísticas de latencia, eventos y comandos más usados).

Las sesiones cubren distintos tipos de interacción: visión, conversación y apoyo
emocional, navegación y protocolo de emergencia. Los identificadores de llamada de Twilio
que aparecían en los eventos de emergencia se han anonimizado.

Para analizar una sesión:

```bash
python3 ../analizar_sesion.py sesion_2026-06-20_15-49-47_35419.jsonl
```
