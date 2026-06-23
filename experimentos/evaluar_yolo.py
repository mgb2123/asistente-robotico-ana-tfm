#!/usr/bin/env python3
"""
evaluar_yolo.py — Sesión 3 del plan (YOLO en 3 condiciones).

Evalúa el detector sobre un conjunto de imágenes organizadas por condición y
clase, y produce la tabla `clase × condición → acierto/fallo` que pide la
memoria, además de guardar las imágenes anotadas para ilustrar aciertos/fallos.

Cubre: RF-03, RNF-03, Métrica 2 (detección fiable).

Estructura esperada del dataset (el nombre de la carpeta de clase es la
etiqueta esperada; admite nombre en español o inglés):

    dataset_validacion/
      A_ideal/
        botella/  img1.jpg  img2.jpg ...
        silla/    ...
      B_angulo/
        botella/  ...
      C_iluminacion/
        botella/  ...

Una imagen es ACIERTO si el modelo detecta (conf >= umbral) al menos una caja
de la clase esperada.

Uso:
    # Con el modelo que usa el robot (yolov8n.pt genérico COCO)
    python3 experimentos/evaluar_yolo.py --datos experimentos/dataset_validacion

    # Con el modelo entrenado de 6 clases del TFM (si lo copias del PC)
    python3 experimentos/evaluar_yolo.py --datos ... --model ruta/al/best.pt
"""

import argparse
import csv
import os
import sys
import unicodedata

# Traducción EN->ES de las clases COCO (espejo de object_detector_node.YOLO_ES),
# para que las carpetas de clase puedan nombrarse en español.
YOLO_ES = {
    'person': 'persona', 'bicycle': 'bicicleta', 'car': 'coche', 'bottle': 'botella',
    'wine glass': 'copa', 'cup': 'taza', 'fork': 'tenedor', 'knife': 'cuchillo',
    'spoon': 'cuchara', 'bowl': 'cuenco', 'chair': 'silla', 'couch': 'sofa',
    'potted plant': 'planta', 'bed': 'cama', 'dining table': 'mesa',
    'tv': 'televisor', 'laptop': 'portatil', 'mouse': 'raton', 'remote': 'mando',
    'keyboard': 'teclado', 'cell phone': 'movil', 'book': 'libro', 'clock': 'reloj',
    'vase': 'jarron', 'scissors': 'tijeras', 'teddy bear': 'peluche',
    'backpack': 'mochila', 'handbag': 'bolso', 'suitcase': 'maleta',
    'cat': 'gato', 'dog': 'perro', 'banana': 'platano', 'apple': 'manzana',
    'orange': 'naranja', 'pizza': 'pizza', 'cake': 'pastel',
}

EXT_IMG = ('.jpg', '.jpeg', '.png', '.bmp', '.webp')


def normalizar(s):
    s = unicodedata.normalize('NFKD', s.lower().strip())
    return ''.join(c for c in s if not unicodedata.combining(c))


def nombres_detectados(label_en):
    """Conjunto de nombres normalizados que valen para una detección EN."""
    nombres = {normalizar(label_en)}
    es = YOLO_ES.get(label_en)
    if es:
        nombres.add(normalizar(es))
    return nombres


def listar_imagenes(carpeta):
    return sorted(f for f in os.listdir(carpeta)
                  if f.lower().endswith(EXT_IMG))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--datos', default='experimentos/dataset_validacion',
                    help='carpeta raíz con <condicion>/<clase>/*.jpg')
    ap.add_argument('--model', default='yolov8n.pt',
                    help='ruta al modelo YOLO (.pt). Por defecto el del robot')
    ap.add_argument('--conf', type=float, default=0.5,
                    help='umbral de confianza (el robot usa 0.5)')
    ap.add_argument('--salida', default='experimentos/resultados/sesion3',
                    help='ruta base de salida (.md, _detalle.csv, anotadas/)')
    args = ap.parse_args()

    if not os.path.isdir(args.datos):
        print(f'No existe la carpeta de datos: {args.datos}', file=sys.stderr)
        print('Crea la estructura <condicion>/<clase>/*.jpg y vuelve a ejecutar.',
              file=sys.stderr)
        sys.exit(1)

    try:
        from ultralytics import YOLO
        import cv2
    except ImportError as e:
        print(f'Falta una dependencia: {e}', file=sys.stderr)
        sys.exit(1)

    print(f'Cargando modelo {args.model} ...')
    modelo = YOLO(args.model)

    condiciones = sorted(d for d in os.listdir(args.datos)
                         if os.path.isdir(os.path.join(args.datos, d)))
    if not condiciones:
        print('No hay subcarpetas de condición en', args.datos, file=sys.stderr)
        sys.exit(1)

    dir_anotadas = args.salida + '_anotadas'
    os.makedirs(dir_anotadas, exist_ok=True)

    # resultados[clase][condicion] = [aciertos, total]
    resultados = {}
    detalle = []  # filas para el CSV

    for cond in condiciones:
        ruta_cond = os.path.join(args.datos, cond)
        clases = sorted(d for d in os.listdir(ruta_cond)
                        if os.path.isdir(os.path.join(ruta_cond, d)))
        for clase in clases:
            ruta_clase = os.path.join(ruta_cond, clase)
            esperada = normalizar(clase)
            imagenes = listar_imagenes(ruta_clase)
            if not imagenes:
                continue
            resultados.setdefault(clase, {}).setdefault(cond, [0, 0])
            for img in imagenes:
                ruta_img = os.path.join(ruta_clase, img)
                res = modelo.predict(source=ruta_img, conf=args.conf,
                                     verbose=False)[0]
                detectadas = set()
                conf_clase = 0.0
                frame = res.orig_img.copy()
                for caja in res.boxes:
                    cid = int(caja.cls[0])
                    label_en = modelo.names[cid]
                    conf = float(caja.conf[0])
                    nombres = nombres_detectados(label_en)
                    detectadas |= nombres
                    es_la_esperada = esperada in nombres
                    if es_la_esperada:
                        conf_clase = max(conf_clase, conf)
                    x1, y1, x2, y2 = map(int, caja.xyxy[0])
                    color = (0, 255, 0) if es_la_esperada else (0, 165, 255)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(frame, f'{YOLO_ES.get(label_en, label_en)} '
                                f'{conf:.2f}', (x1, max(15, y1 - 8)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                acierto = esperada in detectadas
                resultados[clase][cond][1] += 1
                if acierto:
                    resultados[clase][cond][0] += 1

                # Guardar anotada con prefijo OK_/FALLO_ para inspección visual.
                pref = 'OK' if acierto else 'FALLO'
                destino = os.path.join(
                    dir_anotadas, f'{pref}_{cond}_{clase}_{img}')
                cv2.imwrite(destino, frame)

                detalle.append({
                    'condicion': cond, 'clase_esperada': clase, 'imagen': img,
                    'acierto': int(acierto), 'conf_clase': round(conf_clase, 3),
                    'detectado': ';'.join(sorted(detectadas)),
                })
                estado = 'OK   ' if acierto else 'FALLO'
                print(f'  [{estado}] {cond}/{clase}/{img} '
                      f'(conf={conf_clase:.2f})')

    # --- Tabla Markdown clase × condición ---
    clases_orden = sorted(resultados.keys())
    L = ['# Sesión 3 — YOLO en 3 condiciones', '',
         f'- **Modelo:** `{args.model}`  ·  **Umbral conf:** {args.conf}',
         f'- **Condiciones:** {", ".join(condiciones)}', '',
         '## Aciertos por clase y condición', '']
    cab = '| Clase | ' + ' | '.join(condiciones) + ' | Total |'
    sep = '|-------|' + '|'.join(['------'] * (len(condiciones) + 1)) + '|'
    L.append(cab)
    L.append(sep)
    tot_cond = {c: [0, 0] for c in condiciones}
    for clase in clases_orden:
        celdas = []
        tac, ttot = 0, 0
        for c in condiciones:
            ac, tt = resultados[clase].get(c, [0, 0])
            tac += ac
            ttot += tt
            tot_cond[c][0] += ac
            tot_cond[c][1] += tt
            celdas.append(f'{ac}/{tt}' if tt else '—')
        total = f'{tac}/{ttot} ({100*tac/ttot:.0f}%)' if ttot else '—'
        L.append(f'| {clase} | ' + ' | '.join(celdas) + f' | {total} |')
    # Fila de totales por condición
    celdas_tot = []
    g_ac, g_tot = 0, 0
    for c in condiciones:
        ac, tt = tot_cond[c]
        g_ac += ac
        g_tot += tt
        celdas_tot.append(f'**{ac}/{tt}**' if tt else '—')
    g = f'**{g_ac}/{g_tot} ({100*g_ac/g_tot:.0f}%)**' if g_tot else '—'
    L.append('| **Total** | ' + ' | '.join(celdas_tot) + f' | {g} |')
    L.append('')
    L.append(f'Imágenes anotadas (OK_/FALLO_) en `{dir_anotadas}/`.')
    L.append('')

    os.makedirs(os.path.dirname(os.path.abspath(args.salida)) or '.',
                exist_ok=True)
    with open(args.salida + '.md', 'w', encoding='utf-8') as fh:
        fh.write('\n'.join(L))
    with open(args.salida + '_detalle.csv', 'w', newline='',
              encoding='utf-8') as fh:
        w = csv.DictWriter(fh, fieldnames=['condicion', 'clase_esperada',
                           'imagen', 'acierto', 'conf_clase', 'detectado'])
        w.writeheader()
        w.writerows(detalle)

    print()
    print('\n'.join(L))
    print(f'\nEscrito: {args.salida}.md, {args.salida}_detalle.csv, '
          f'{dir_anotadas}/')


if __name__ == '__main__':
    main()
