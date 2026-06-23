#!/usr/bin/env python3
"""
wer.py — Word Error Rate (Sesión 1, calidad del STT — RF-04).

Calcula el WER comparando lo que dijiste de verdad (referencia) con lo que
transcribió Vosk (hipótesis). El WER es la distancia de edición a nivel de
palabra (sustituciones + inserciones + borrados) dividida por el nº de
palabras de la referencia.

Flujo recomendado:
  1) `analizar_sesion.py --salida ...` genera `..._transcripciones.csv` con
     columnas `turno, referencia, hipotesis` y la `referencia` vacía.
  2) Rellena la columna `referencia` con lo que dijiste realmente en cada turno.
  3) `python3 experimentos/wer.py ..._transcripciones.csv`

También admite dos frases sueltas:
  python3 experimentos/wer.py --ref "hola que tal" --hip "ola ke tal"
"""

import argparse
import csv
import re
import sys
import unicodedata


def normalizar(texto):
    """Minúsculas, sin tildes ni puntuación, espacios colapsados -> palabras."""
    texto = texto.lower().strip()
    texto = unicodedata.normalize('NFKD', texto)
    texto = ''.join(c for c in texto if not unicodedata.combining(c))
    texto = re.sub(r'[^\w\s]', ' ', texto)
    return texto.split()


def distancia_palabras(ref, hip):
    """Levenshtein a nivel de palabra. Devuelve (sust, ins, borr, distancia)."""
    n, m = len(ref), len(hip)
    # dp[i][j] = coste de transformar ref[:i] en hip[:j]
    dp = [[0] * (m + 1) for _ in range(n + 1)]
    for i in range(n + 1):
        dp[i][0] = i
    for j in range(m + 1):
        dp[0][j] = j
    for i in range(1, n + 1):
        for j in range(1, m + 1):
            if ref[i - 1] == hip[j - 1]:
                dp[i][j] = dp[i - 1][j - 1]
            else:
                dp[i][j] = 1 + min(dp[i - 1][j],      # borrado
                                   dp[i][j - 1],      # inserción
                                   dp[i - 1][j - 1])  # sustitución
    return dp[n][m]


def wer(referencia, hipotesis):
    ref = normalizar(referencia)
    hip = normalizar(hipotesis)
    if not ref:
        return None, 0, 0
    d = distancia_palabras(ref, hip)
    return d / len(ref), d, len(ref)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('csv', nargs='?',
                    help='CSV con columnas referencia,hipotesis')
    ap.add_argument('--ref', help='frase de referencia (modo suelto)')
    ap.add_argument('--hip', help='frase hipótesis (modo suelto)')
    args = ap.parse_args()

    if args.ref is not None and args.hip is not None:
        w, d, n = wer(args.ref, args.hip)
        print(f'WER = {w*100:.1f} %  ({d} errores / {n} palabras)')
        return

    if not args.csv:
        ap.error('indica un CSV o --ref/--hip')

    total_err = 0
    total_pal = 0
    filas = 0
    sin_ref = 0
    print(f"{'turno':>5}  {'WER':>6}  {'err/pal':>9}  referencia | hipótesis")
    print('-' * 70)
    with open(args.csv, encoding='utf-8') as fh:
        for fila in csv.DictReader(fh):
            ref = (fila.get('referencia') or '').strip()
            hip = (fila.get('hipotesis') or '').strip()
            turno = fila.get('turno', '?')
            if not ref:
                sin_ref += 1
                continue
            w, d, n = wer(ref, hip)
            total_err += d
            total_pal += n
            filas += 1
            print(f'{turno:>5}  {w*100:5.1f}%  {d:>3}/{n:<4}  '
                  f'{ref[:30]} | {hip[:30]}')

    print('-' * 70)
    if total_pal:
        print(f'WER GLOBAL = {100*total_err/total_pal:.1f} %  '
              f'({total_err} errores / {total_pal} palabras, {filas} frases)')
    else:
        print('No hay filas con columna "referencia" rellena.', file=sys.stderr)
    if sin_ref:
        print(f'({sin_ref} filas sin referencia, omitidas)')


if __name__ == '__main__':
    main()
