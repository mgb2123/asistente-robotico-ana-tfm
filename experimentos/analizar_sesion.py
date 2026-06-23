#!/usr/bin/env python3
"""
analizar_sesion.py — Sesión 1 del plan de experimentos (conversación afectiva).

Lee uno o varios ficheros `sesion_*.jsonl` que produce `metricas_logger` y
genera las tablas que pide la memoria del TFM:

  * Tabla de latencias por turno (t_ASR+LLM, t_LLM->audio, latencia de
    respuesta percibida, duración del habla de Ana, emoción).
  * Resumen de latencias con P50 / P90 / media / min / max.
  * Comparación con Métrica 4 / RNF-01 (latencia de respuesta < 3 s):
    nº y % de turnos que cumplen.
  * 2-3 ejemplos de intercambio (uno por registro emocional cuando se puede)
    para el análisis de coherencia afectiva.
  * Recuento de eventos relevantes: barge-ins, caídas de WiFi, fallback de TTS,
    errores, emergencias.
  * CSV por turno y CSV de transcripciones (para estimar el WER con `wer.py`).

Cubre: RF-04 (STT), RF-05 (LLM), RF-06 (TTS), RNF-01, Métricas 3, 4 y 5.

Uso:
    # Analiza la sesión más reciente de logs_sesiones/
    python3 experimentos/analizar_sesion.py

    # Analiza un fichero concreto
    python3 experimentos/analizar_sesion.py logs_sesiones/sesion_2026-06-07_17-59-45_3653.jsonl

    # Vuelca el informe y los CSV a experimentos/resultados/
    python3 experimentos/analizar_sesion.py --salida experimentos/resultados/sesion1
"""

import argparse
import csv
import glob
import json
import os
import sys

# Métrica 4 / RNF-01: la latencia de respuesta percibida debe quedar por
# debajo de este umbral (segundos).
UMBRAL_LATENCIA = 3.0

DIR_LOGS_POR_DEFECTO = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'logs_sesiones')


# ----------------------------------------------------------------------
# Estadística
# ----------------------------------------------------------------------
def percentil(valores, p):
    """Percentil p (0-100) por interpolación lineal. `valores` no vacío."""
    if not valores:
        return None
    orden = sorted(valores)
    if len(orden) == 1:
        return orden[0]
    k = (len(orden) - 1) * (p / 100.0)
    f = int(k)
    c = min(f + 1, len(orden) - 1)
    if f == c:
        return orden[f]
    return orden[f] + (orden[c] - orden[f]) * (k - f)


def stats(valores):
    """Devuelve dict con n, media, min, max, p50, p90 o None si está vacío."""
    if not valores:
        return None
    return {
        'n': len(valores),
        'media': sum(valores) / len(valores),
        'min': min(valores),
        'max': max(valores),
        'p50': percentil(valores, 50),
        'p90': percentil(valores, 90),
    }


# ----------------------------------------------------------------------
# Carga y emparejado de eventos
# ----------------------------------------------------------------------
def cargar_eventos(path):
    eventos = []
    with open(path, encoding='utf-8') as f:
        for n, linea in enumerate(f, 1):
            linea = linea.strip()
            if not linea:
                continue
            try:
                eventos.append(json.loads(linea))
            except json.JSONDecodeError:
                print(f'  aviso: línea {n} ilegible en {path}', file=sys.stderr)
    return eventos


def construir_turnos(eventos):
    """Empareja cada `turno` con el último `comando` y `llm` que lo preceden.

    El pipeline procesa un turno cada vez, así que el último comando/llm desde
    el `turno` anterior pertenece al turno que se cierra ahora.
    """
    turnos = []
    pend_comando = None
    pend_llm = None
    for ev in eventos:
        t = ev.get('ev')
        if t == 'comando':
            pend_comando = ev
        elif t == 'llm':
            pend_llm = ev
        elif t == 'turno':
            stt = ev.get('stt_to_llm1')
            ll1 = ev.get('llm1_to_aplay1')
            latencia = None
            if isinstance(stt, (int, float)) and isinstance(ll1, (int, float)):
                latencia = round(stt + ll1, 3)
            turnos.append({
                'ts': ev.get('ts', ''),
                'texto_usuario': (pend_comando or {}).get('texto', ''),
                'comando': (pend_comando or {}).get('comando', ''),
                'pregunta': (pend_llm or {}).get('pregunta'),
                'respuesta': (pend_llm or {}).get('respuesta'),
                'emocion': (pend_llm or {}).get('emocion'),
                'intensidad': (pend_llm or {}).get('intensidad'),
                'acciones': (pend_llm or {}).get('acciones', []),
                'stt_to_llm1': stt,
                'llm1_to_aplay1': ll1,
                'aplay1_to_end': ev.get('aplay1_to_end'),
                'latencia_respuesta': latencia,
                'n_chars': ev.get('n_chars'),
                'gap_prev': ev.get('gap_prev'),
            })
            pend_comando = None
            pend_llm = None
    return turnos


def contar_eventos(eventos):
    c = {'barge_in': 0, 'wifi_perdido': 0, 'wifi_recuperado': 0,
         'tts_fallback': 0, 'error': 0, 'emergencia': 0}
    meta = {}
    for ev in eventos:
        t = ev.get('ev')
        if t == 'sesion_inicio':
            meta.update({k: ev.get(k) for k in
                         ('fecha', 'pid', 'llm_model')})
        elif t == 'sesion_fin':
            meta['duracion_s'] = ev.get('duracion_s')
            meta['n_turnos'] = ev.get('n_turnos')
        elif t == 'barge_in':
            c['barge_in'] += 1
        elif t == 'wifi':
            if ev.get('estado') == 'perdido':
                c['wifi_perdido'] += 1
            elif ev.get('estado') == 'recuperado':
                c['wifi_recuperado'] += 1
        elif t == 'tts_fallback':
            c['tts_fallback'] += 1
        elif t == 'error':
            c['error'] += 1
        elif t == 'emergencia':
            c['emergencia'] += 1
    return c, meta


def elegir_ejemplos(turnos, maximo=3):
    """Elige hasta `maximo` turnos con respuesta del LLM, priorizando
    registros emocionales distintos para ilustrar la coherencia afectiva."""
    con_resp = [t for t in turnos if t.get('respuesta')]
    elegidos = []
    vistos = set()
    for t in con_resp:
        emo = t.get('emocion') or 'sin_emocion'
        if emo not in vistos:
            vistos.add(emo)
            elegidos.append(t)
        if len(elegidos) >= maximo:
            break
    # Si hay pocas emociones distintas, completar con los primeros turnos.
    for t in con_resp:
        if len(elegidos) >= maximo:
            break
        if t not in elegidos:
            elegidos.append(t)
    return elegidos


# ----------------------------------------------------------------------
# Formato
# ----------------------------------------------------------------------
def f(x, dec=2):
    return '—' if x is None else f'{x:.{dec}f}'


def informe_markdown(nombre, meta, turnos, conteos):
    L = []
    L.append(f'# Sesión 1 — Conversación afectiva  \n`{nombre}`')
    L.append('')
    L.append(f"- **Modelo LLM:** {meta.get('llm_model', '—')}")
    L.append(f"- **Fecha:** {meta.get('fecha', '—')}")
    dur = meta.get('duracion_s')
    L.append(f"- **Duración:** {f(dur, 1)} s")
    L.append(f"- **Turnos analizados:** {len(turnos)}")
    L.append('')

    # --- Tabla por turno ---
    L.append('## Latencias por turno')
    L.append('')
    L.append('| # | Hora | Usuario (transcrito) | Emoción | STT→LLM (s) | '
             'LLM→audio (s) | **Latencia resp. (s)** | Habla Ana (s) |')
    L.append('|---|------|----------------------|---------|-------------|'
             '---------------|------------------------|---------------|')
    for i, t in enumerate(turnos, 1):
        cumple = ''
        lr = t['latencia_respuesta']
        if lr is not None:
            cumple = ' OK' if lr < UMBRAL_LATENCIA else ' ALTA'
        usuario = (t['texto_usuario'] or '').replace('|', '/')[:40]
        L.append(
            f"| {i} | {t['ts']} | {usuario} | {t.get('emocion') or '—'} | "
            f"{f(t['stt_to_llm1'])} | {f(t['llm1_to_aplay1'])} | "
            f"{f(lr)}{cumple} | {f(t['aplay1_to_end'])} |")
    L.append('')

    # --- Resumen de latencias ---
    def col(clave):
        return [t[clave] for t in turnos
                if isinstance(t.get(clave), (int, float))]

    L.append('## Resumen de latencias (s)')
    L.append('')
    L.append('| Métrica | n | media | P50 | P90 | min | max |')
    L.append('|---------|---|-------|-----|-----|-----|-----|')
    filas = [
        ('STT → 1er token LLM', 'stt_to_llm1'),
        ('LLM → 1er audio', 'llm1_to_aplay1'),
        ('**Latencia de respuesta**', 'latencia_respuesta'),
        ('Duración habla de Ana', 'aplay1_to_end'),
    ]
    for etiqueta, clave in filas:
        st = stats(col(clave))
        if st is None:
            L.append(f'| {etiqueta} | 0 | — | — | — | — | — |')
        else:
            L.append(f"| {etiqueta} | {st['n']} | {f(st['media'])} | "
                     f"{f(st['p50'])} | {f(st['p90'])} | {f(st['min'])} | "
                     f"{f(st['max'])} |")
    L.append('')

    # --- Métrica 4 / RNF-01 ---
    lat = col('latencia_respuesta')
    if lat:
        cumplen = sum(1 for x in lat if x < UMBRAL_LATENCIA)
        pct = 100.0 * cumplen / len(lat)
        L.append(f'## Métrica 4 / RNF-01 — latencia de respuesta < {UMBRAL_LATENCIA:.0f} s')
        L.append('')
        L.append(f'- Turnos que cumplen: **{cumplen}/{len(lat)} '
                 f'({pct:.0f} %)**')
        L.append(f"- P50 = {f(percentil(lat, 50))} s · "
                 f"P90 = {f(percentil(lat, 90))} s")
        L.append('')

    # --- Ejemplos afectivos ---
    ejemplos = elegir_ejemplos(turnos)
    if ejemplos:
        L.append('## Ejemplos de intercambio (coherencia afectiva — Métrica 5)')
        L.append('')
        for t in ejemplos:
            L.append(f"**Usuario:** {t['texto_usuario'] or '—'}  ")
            L.append(f"**Ana ({t.get('emocion') or '—'}, "
                     f"int. {t.get('intensidad')}):** {t['respuesta']}  ")
            if t.get('acciones'):
                L.append(f"_Acciones:_ {', '.join(t['acciones'])}  ")
            L.append(f"_Latencia de respuesta:_ {f(t['latencia_respuesta'])} s")
            L.append('')
            L.append('> Valoración afectiva (rellenar): ')
            L.append('')
    else:
        L.append('## Ejemplos de intercambio')
        L.append('')
        L.append('_No hay texto de respuesta en el log. Relanza el asistente '
                 'con el `asistente_node.py` actualizado para capturarlo._')
        L.append('')

    # --- Eventos ---
    L.append('## Eventos de la sesión')
    L.append('')
    L.append(f"- Barge-ins (interrupciones): {conteos['barge_in']}")
    L.append(f"- Caídas de WiFi: {conteos['wifi_perdido']} "
             f"(recuperaciones: {conteos['wifi_recuperado']})")
    L.append(f"- Fallback de TTS (Piper local): {conteos['tts_fallback']}")
    L.append(f"- Errores: {conteos['error']}")
    L.append(f"- Emergencias: {conteos['emergencia']}")
    L.append('')
    return '\n'.join(L)


def escribir_csvs(base, turnos):
    """Escribe `<base>_turnos.csv` y `<base>_transcripciones.csv`."""
    with open(base + '_turnos.csv', 'w', newline='', encoding='utf-8') as fh:
        w = csv.writer(fh)
        w.writerow(['turno', 'hora', 'texto_usuario', 'emocion', 'intensidad',
                    'stt_to_llm1', 'llm1_to_aplay1', 'latencia_respuesta',
                    'aplay1_to_end', 'n_chars', 'acciones', 'respuesta'])
        for i, t in enumerate(turnos, 1):
            w.writerow([i, t['ts'], t['texto_usuario'], t.get('emocion'),
                        t.get('intensidad'), t['stt_to_llm1'],
                        t['llm1_to_aplay1'], t['latencia_respuesta'],
                        t['aplay1_to_end'], t['n_chars'],
                        '|'.join(t.get('acciones') or []),
                        t.get('respuesta') or ''])
    # CSV para estimar WER: el usuario rellena la columna 'referencia' con lo
    # que dijo de verdad y luego ejecuta wer.py sobre este fichero.
    with open(base + '_transcripciones.csv', 'w', newline='',
              encoding='utf-8') as fh:
        w = csv.writer(fh)
        w.writerow(['turno', 'referencia', 'hipotesis'])
        for i, t in enumerate(turnos, 1):
            if t['texto_usuario']:
                w.writerow([i, '', t['texto_usuario']])


# ----------------------------------------------------------------------
def resolver_ficheros(args):
    if args.ficheros:
        return args.ficheros
    patron = os.path.join(DIR_LOGS_POR_DEFECTO, 'sesion_*.jsonl')
    candidatos = sorted(glob.glob(patron), key=os.path.getmtime)
    if not candidatos:
        print(f'No hay ficheros de sesión en {DIR_LOGS_POR_DEFECTO}',
              file=sys.stderr)
        sys.exit(1)
    return [candidatos[-1]]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('ficheros', nargs='*',
                    help='ficheros sesion_*.jsonl (por defecto, el más reciente)')
    ap.add_argument('--salida', metavar='BASE',
                    help='ruta base para volcar informe .md y CSVs')
    args = ap.parse_args()

    for path in resolver_ficheros(args):
        if not os.path.exists(path):
            print(f'No existe: {path}', file=sys.stderr)
            continue
        eventos = cargar_eventos(path)
        turnos = construir_turnos(eventos)
        conteos, meta = contar_eventos(eventos)
        nombre = os.path.basename(path)
        informe = informe_markdown(nombre, meta, turnos, conteos)

        if args.salida:
            os.makedirs(os.path.dirname(os.path.abspath(args.salida)) or '.',
                        exist_ok=True)
            base = args.salida
            with open(base + '.md', 'w', encoding='utf-8') as fh:
                fh.write(informe)
            escribir_csvs(base, turnos)
            print(f'Escrito: {base}.md, {base}_turnos.csv, '
                  f'{base}_transcripciones.csv')
        else:
            print(informe)


if __name__ == '__main__':
    main()
