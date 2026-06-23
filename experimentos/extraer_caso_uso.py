#!/usr/bin/env python3
"""
extraer_caso_uso.py — Sesión 2 del plan (demo integrada de los dos casos de uso).

La Sesión 2 es una demo en vivo cuyos resultados (éxito/fallo por etapa, tiempo
total) son fundamentalmente una **observación manual**. Este script no inventa
esos datos: extrae del log JSONL la línea de tiempo automática de los eventos
que SÍ se registran (comandos de navegación, llamadas de emergencia, fallback,
errores) para corroborar y datar lo que anotes a mano en la plantilla
`plantillas/sesion2_casos_uso.csv`.

Cubre (apoyo a): RF-01, RF-02, RF-03, RF-07, RF-08, RNF-02, Métrica 1.

Uso:
    python3 experimentos/extraer_caso_uso.py [sesion_*.jsonl]
    python3 experimentos/extraer_caso_uso.py --salida experimentos/resultados/sesion2.md
"""

import argparse
import glob
import json
import os
import sys

DIR_LOGS = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'logs_sesiones')

# Eventos relevantes para los dos casos de uso de la Sesión 2.
EV_INTERES = {
    'comando': lambda e: e.get('comando', '').startswith(('navegar:', 'ver'))
    or e.get('comando') in ('volver_a_base', 'relocalizar', 'parar'),
    'navegacion': lambda e: True,
    'emergencia': lambda e: True,
    'tts_fallback': lambda e: True,
    'error': lambda e: True,
}

DESCR = {
    'comando': lambda e: f"comando: {e.get('comando')}  (\"{e.get('texto', '')}\")",
    'navegacion': lambda e: f"navegación lanzada: {e.get('comando')}",
    'emergencia': lambda e: f"EMERGENCIA — llamada Twilio sid={e.get('sid')}",
    'tts_fallback': lambda e: f"fallback TTS local ({e.get('motivo', '')})",
    'error': lambda e: f"ERROR en {e.get('donde')}: {e.get('msg', '')}",
}


def cargar(path):
    out = []
    with open(path, encoding='utf-8') as f:
        for linea in f:
            linea = linea.strip()
            if linea:
                try:
                    out.append(json.loads(linea))
                except json.JSONDecodeError:
                    pass
    return out


def linea_de_tiempo(eventos):
    filas = []
    t_anterior = None
    for e in eventos:
        ev = e.get('ev')
        filtro = EV_INTERES.get(ev)
        if filtro is None or not filtro(e):
            continue
        t = e.get('t', 0.0)
        dt = '' if t_anterior is None else f'+{t - t_anterior:.1f}s'
        filas.append((e.get('ts', ''), f'{t:.1f}', dt, DESCR[ev](e)))
        t_anterior = t
    return filas


def informe(nombre, filas):
    L = [f'# Sesión 2 — Línea de tiempo automática  \n`{nombre}`', '']
    if not filas:
        L.append('_No hay eventos de navegación/emergencia en este log._')
        L.append('')
        L.append('Lanza la demo (`nav:=true`) y repite. Recuerda anotar a mano '
                 'éxito/fallo por etapa en `plantillas/sesion2_casos_uso.csv`.')
        return '\n'.join(L)
    L.append('| Hora | t (s) | Δ | Evento |')
    L.append('|------|-------|---|--------|')
    for ts, t, dt, descr in filas:
        L.append(f'| {ts} | {t} | {dt} | {descr.replace("|", "/")} |')
    L.append('')
    L.append('> Combina esta línea de tiempo con tu plantilla manual '
             '`sesion2_casos_uso.csv` (etapas: orden → navegación → '
             'detección/llamada → retorno).')
    return '\n'.join(L)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('ficheros', nargs='*')
    ap.add_argument('--salida', metavar='MD')
    args = ap.parse_args()

    ficheros = args.ficheros
    if not ficheros:
        cand = sorted(glob.glob(os.path.join(DIR_LOGS, 'sesion_*.jsonl')),
                      key=os.path.getmtime)
        if not cand:
            print('No hay logs de sesión.', file=sys.stderr)
            sys.exit(1)
        ficheros = [cand[-1]]

    for path in ficheros:
        eventos = cargar(path)
        texto = informe(os.path.basename(path), linea_de_tiempo(eventos))
        if args.salida:
            os.makedirs(os.path.dirname(os.path.abspath(args.salida)) or '.',
                        exist_ok=True)
            with open(args.salida, 'w', encoding='utf-8') as fh:
                fh.write(texto)
            print(f'Escrito: {args.salida}')
        else:
            print(texto)
            print()


if __name__ == '__main__':
    main()
