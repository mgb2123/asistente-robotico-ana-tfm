"""
metricas_logger — Logger de sesión para el asistente de voz.

Persiste, en un fichero por sesión, las métricas de latencia y los eventos
relevantes del nodo (`asistente_node`). El objetivo es poder analizar a
posteriori los tiempos de interacción (gráficas/tablas del TFM) sin depender
del log de consola, que se pierde al cerrar el proceso.

Formato:
  - `<base>.jsonl`        un evento JSON por línea (append + flush por línea, así
                          los datos sobreviven aunque el proceso muera de golpe).
  - `<base>_resumen.txt`  resumen legible generado al cerrar la sesión (totales,
                          medias, min/max/p95 de cada latencia, recuentos).

El nombre base es `sesion_YYYY-MM-DD_HH-MM-SS_<pid>`, de modo que cada
ejecución produce un fichero distinto (el PID evita colisiones si se relanza
dentro del mismo segundo).

Diseño defensivo: todos los métodos son thread-safe (los eventos llegan desde
varios hilos daemon: STT, LLM, TTS, monitor WiFi) y nunca lanzan excepción
hacia el llamante — un fallo de logging jamás debe tumbar el nodo. `cerrar()`
es idempotente para poder invocarse desde `destroy_node`, `atexit` y el
handler de `SIGTERM` sin duplicar el resumen.
"""

import json
import os
import threading
import time
from collections import Counter
from datetime import datetime


class SesionLogger:
    """Escribe un JSONL de eventos por sesión + un resumen al cerrar."""

    def __init__(self, log_dir, llm_model=''):
        self._lock = threading.Lock()
        self._cerrado = False
        self._fh = None

        # Acumuladores para el resumen.
        self._n_turnos = 0
        self._n_chars_total = 0
        self._latencias = {
            'stt_to_llm1': [],
            'llm1_to_aplay1': [],
            'aplay1_to_end': [],
            'gap_prev': [],
            'llm_first_token': [],
        }
        self._n_barge_in = 0
        self._n_wifi_perdido = 0
        self._n_errores = 0
        self._n_emergencias = 0
        self._comandos = Counter()

        # Validación de navegación (Tabla 2.17 del TFM): por destino se cuenta
        # cuántos intentos hubo, cuántos fueron éxito y los tiempos de trayecto.
        # Se agrega por el nombre real del waypoint navegado (sin fijar nombres).
        self._nav_por_destino = {}        # destino -> {'intentos', 'exitos', 'tiempos'[]}
        self._nav_motivos = Counter()     # motivo de fallo -> nº de veces
        # Carga del sistema (CPU/RAM/temp/load) muestreada por monitor_sistema.
        self._sys = {'cpu': [], 'mem': [], 'temp': [], 'load': []}
        # Resumen de tareas FETCH (caso "ve a por X").
        self._fetch = []                  # lista de dicts {objeto, entregado, intentos, motivo}

        self._t0 = time.monotonic()
        self._wall0 = datetime.now()

        try:
            os.makedirs(log_dir, exist_ok=True)
            base = 'sesion_{}_{}'.format(
                self._wall0.strftime('%Y-%m-%d_%H-%M-%S'), os.getpid())
            self._path_jsonl = os.path.join(log_dir, base + '.jsonl')
            self._path_resumen = os.path.join(log_dir, base + '_resumen.txt')
            self._fh = open(self._path_jsonl, 'a', encoding='utf-8')
        except Exception as e:
            # Si no se puede abrir el fichero, el logger queda inerte pero el
            # nodo sigue funcionando.
            print('[metricas_logger] no se pudo abrir el log: {}'.format(e))
            self._fh = None
            self._path_jsonl = None
            self._path_resumen = None
            return

        self.evento('sesion_inicio',
                    fecha=self._wall0.strftime('%Y-%m-%d %H:%M:%S'),
                    pid=os.getpid(),
                    llm_model=llm_model)

    # ------------------------------------------------------------------
    def evento(self, ev, **campos):
        """Escribe una línea de evento y actualiza los acumuladores."""
        with self._lock:
            if self._cerrado or self._fh is None:
                return
            try:
                registro = {
                    't': round(time.monotonic() - self._t0, 3),
                    'ts': datetime.now().strftime('%H:%M:%S'),
                    'ev': ev,
                }
                registro.update(campos)
                self._fh.write(
                    json.dumps(registro, ensure_ascii=False) + '\n')
                self._fh.flush()
                self._acumular(ev, campos)
            except Exception as e:
                print('[metricas_logger] error escribiendo evento: {}'.format(e))

    def _acumular(self, ev, campos):
        """Actualiza los contadores que alimentan el resumen final."""
        if ev == 'turno':
            self._n_turnos += 1
            self._n_chars_total += campos.get('n_chars', 0) or 0
            for clave, lista in self._latencias.items():
                val = campos.get(clave)
                if isinstance(val, (int, float)) and val >= 0:
                    lista.append(val)
        elif ev == 'llm':
            val = campos.get('llm_first_token')
            if isinstance(val, (int, float)) and val >= 0:
                self._latencias['llm_first_token'].append(val)
        elif ev == 'comando':
            comando = campos.get('comando', '')
            if comando:
                self._comandos[comando] += 1
        elif ev == 'barge_in':
            self._n_barge_in += 1
        elif ev == 'wifi' and campos.get('estado') == 'perdido':
            self._n_wifi_perdido += 1
        elif ev == 'error':
            self._n_errores += 1
        elif ev == 'emergencia':
            self._n_emergencias += 1
        elif ev == 'navegacion_metrica':
            destino = campos.get('destino') or '(desconocido)'
            d = self._nav_por_destino.setdefault(
                destino, {'intentos': 0, 'exitos': 0, 'tiempos': []})
            d['intentos'] += 1
            if campos.get('exito'):
                d['exitos'] += 1
            t_nav = campos.get('t_nav')
            if isinstance(t_nav, (int, float)) and t_nav >= 0:
                d['tiempos'].append(t_nav)
            if not campos.get('exito'):
                self._nav_motivos[campos.get('motivo') or 'desconocido'] += 1
        elif ev == 'sistema':
            for clave, campo in (('cpu', 'cpu_pct'), ('mem', 'mem_pct'),
                                 ('temp', 'temp_c'), ('load', 'load1')):
                val = campos.get(campo)
                if isinstance(val, (int, float)):
                    self._sys[clave].append(val)
        elif ev == 'tarea_fetch':
            self._fetch.append({
                'objeto': campos.get('objeto', ''),
                'entregado': bool(campos.get('entregado')),
                'intentos': campos.get('intentos', 0),
                'motivo': campos.get('motivo', ''),
            })

    # ------------------------------------------------------------------
    def cerrar(self):
        """Escribe el evento de fin + el resumen .txt. Idempotente."""
        with self._lock:
            if self._cerrado or self._fh is None:
                self._cerrado = True
                return
            self._cerrado = True
            duracion = time.monotonic() - self._t0
            try:
                fin = {
                    't': round(duracion, 3),
                    'ts': datetime.now().strftime('%H:%M:%S'),
                    'ev': 'sesion_fin',
                    'duracion_s': round(duracion, 1),
                    'n_turnos': self._n_turnos,
                }
                self._fh.write(json.dumps(fin, ensure_ascii=False) + '\n')
                self._fh.flush()
            except Exception:
                pass
            try:
                self._fh.close()
            except Exception:
                pass
            try:
                self._escribir_resumen(duracion)
            except Exception as e:
                print('[metricas_logger] error escribiendo resumen: {}'.format(e))

    def _escribir_resumen(self, duracion):
        """Genera el `<base>_resumen.txt` legible."""
        if self._path_resumen is None:
            return

        def stats(lista):
            if not lista:
                return None
            ordenada = sorted(lista)
            n = len(ordenada)
            media = sum(ordenada) / n
            p95 = ordenada[min(n - 1, int(round(0.95 * (n - 1))))]
            return (n, media, ordenada[0], ordenada[-1], p95)

        lineas = []
        lineas.append('=' * 56)
        lineas.append('RESUMEN DE SESION — asistente de voz "Ana"')
        lineas.append('=' * 56)
        lineas.append('Inicio:    {}'.format(
            self._wall0.strftime('%Y-%m-%d %H:%M:%S')))
        lineas.append('Fin:       {}'.format(
            datetime.now().strftime('%Y-%m-%d %H:%M:%S')))
        m, s = divmod(int(duracion), 60)
        lineas.append('Duracion:  {}m {:02d}s ({:.1f}s)'.format(m, s, duracion))
        lineas.append('Turnos:    {}'.format(self._n_turnos))
        lineas.append('Chars TTS: {}'.format(self._n_chars_total))
        lineas.append('')
        lineas.append('LATENCIAS (segundos)   n     media    min     max     p95')
        lineas.append('-' * 56)
        etiquetas = {
            'stt_to_llm1': 'STT -> 1er token LLM',
            'llm_first_token': 'LLM 1er token (abs)',
            'llm1_to_aplay1': 'LLM -> 1er audio',
            'aplay1_to_end': 'Audio inicio -> fin',
            'gap_prev': 'Gap entre acciones',
        }
        for clave, etiqueta in etiquetas.items():
            st = stats(self._latencias.get(clave, []))
            if st is None:
                lineas.append('{:22s} (sin datos)'.format(etiqueta))
            else:
                n, media, mn, mx, p95 = st
                lineas.append(
                    '{:22s} {:3d}   {:6.2f}  {:6.2f}  {:6.2f}  {:6.2f}'.format(
                        etiqueta, n, media, mn, mx, p95))
        lineas.append('')
        lineas.append('EVENTOS')
        lineas.append('-' * 56)
        lineas.append('Barge-in (interrupciones): {}'.format(self._n_barge_in))
        lineas.append('Caidas de WiFi:            {}'.format(self._n_wifi_perdido))
        lineas.append('Errores:                   {}'.format(self._n_errores))
        lineas.append('Emergencias:               {}'.format(self._n_emergencias))
        lineas.append('')

        # --- Validación de navegación (Tabla 2.17) ---
        lineas.append('VALIDACION NAVEGACION (Tabla 2.17: RF-02 / RNF-02 / Metrica 1)')
        lineas.append('Exito = err_pos < 0.30 m  y  err_yaw < 15 deg  y  t < 120 s')
        lineas.append('-' * 56)
        lineas.append('{:14s} {:>8s} {:>7s} {:>9s} {:>8s} {:>8s}'.format(
            'waypoint', 'intentos', 'exitos', 'tasa(%)', 't_med(s)', 't_max(s)'))
        if self._nav_por_destino:
            tot_int = tot_ex = 0
            tiempos_glob = []
            for destino in sorted(self._nav_por_destino):
                d = self._nav_por_destino[destino]
                intentos = d['intentos']
                exitos = d['exitos']
                tiempos = d['tiempos']
                tot_int += intentos
                tot_ex += exitos
                tiempos_glob.extend(tiempos)
                tasa = (100.0 * exitos / intentos) if intentos else 0.0
                t_med = (sum(tiempos) / len(tiempos)) if tiempos else 0.0
                t_max = max(tiempos) if tiempos else 0.0
                lineas.append('{:14s} {:8d} {:7d} {:9.1f} {:8.1f} {:8.1f}'.format(
                    destino[:14], intentos, exitos, tasa, t_med, t_max))
            tasa_g = (100.0 * tot_ex / tot_int) if tot_int else 0.0
            t_med_g = (sum(tiempos_glob) / len(tiempos_glob)) if tiempos_glob else 0.0
            t_max_g = max(tiempos_glob) if tiempos_glob else 0.0
            lineas.append('{:14s} {:8d} {:7d} {:9.1f} {:8.1f} {:8.1f}'.format(
                'Global', tot_int, tot_ex, tasa_g, t_med_g, t_max_g))
            if self._nav_motivos:
                lineas.append('Motivos de fallo: ' + ', '.join(
                    '{}={}'.format(m, n)
                    for m, n in self._nav_motivos.most_common()))
        else:
            lineas.append('  (sin navegaciones registradas)')
        lineas.append('')

        # --- Tareas FETCH (caso "ve a por X") ---
        if self._fetch:
            lineas.append('TAREAS FETCH (ve a por un objeto)')
            lineas.append('-' * 56)
            for t in self._fetch:
                lineas.append(
                    '  objeto={:10s} entregado={!s:5s} intentos={} {}'.format(
                        str(t['objeto'])[:10], t['entregado'], t['intentos'],
                        ('motivo=' + t['motivo']) if t['motivo'] else ''))
            lineas.append('')

        # --- Carga del sistema (delata saturacion de la RPi 4B) ---
        lineas.append('SISTEMA (CPU/RAM/TEMP)   n     media    min     max')
        lineas.append('-' * 56)
        etiquetas_sys = {
            'cpu': ('CPU global (%)', ''),
            'mem': ('RAM usada (%)', ''),
            'temp': ('Temp SoC (C)', ''),
            'load': ('Load 1 min', ''),
        }
        hay_sys = any(self._sys[c] for c in self._sys)
        if hay_sys:
            for clave, (etiqueta, _) in etiquetas_sys.items():
                st = stats(self._sys.get(clave, []))
                if st is None:
                    lineas.append('{:22s} (sin datos)'.format(etiqueta))
                else:
                    n, media, mn, mx, _p95 = st
                    lineas.append(
                        '{:22s} {:3d}   {:6.1f}  {:6.1f}  {:6.1f}'.format(
                            etiqueta, n, media, mn, mx))
        else:
            lineas.append('  (sin muestras de sistema)')
        lineas.append('')

        lineas.append('COMANDOS MAS USADOS')
        lineas.append('-' * 56)
        if self._comandos:
            for comando, cuenta in self._comandos.most_common(15):
                lineas.append('  {:5d}  {}'.format(cuenta, comando))
        else:
            lineas.append('  (ninguno)')
        lineas.append('')

        with open(self._path_resumen, 'w', encoding='utf-8') as f:
            f.write('\n'.join(lineas))
