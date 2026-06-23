"""
monitor_sistema — muestreo periódico de carga del sistema (RPi 4B).

Hilo daemon que cada `periodo` segundos lee el uso global de CPU, la RAM, el
load average y la temperatura del SoC, y los entrega por callback (típicamente
a `metricas_logger.SesionLogger`) para poder ver a posteriori si la Raspberry
"no podía más" en algún momento de la sesión (saturación de CPU/temperatura).

Sin dependencias externas (NO usa psutil): lee directamente de `/proc` y
`/sys`, así que no toca el lock de NumPy 1.26.4 del entorno ni añade nada a
`setup.py`. Diseño defensivo: cualquier lectura que falle se omite (queda como
None en ese campo) y jamás lanza excepción hacia el hilo del nodo.

Fuentes:
  - %CPU global: delta de jiffies entre dos lecturas de /proc/stat (línea 'cpu').
  - %RAM usada:  (MemTotal - MemAvailable) / MemTotal de /proc/meminfo.
  - load 1 min:  primer campo de /proc/loadavg.
  - temp SoC:    /sys/class/thermal/thermal_zone0/temp (miligrados -> °C).
"""

import threading
import time


def _leer_cpu_jiffies():
    """Devuelve (total, idle) de la línea 'cpu' de /proc/stat, o None si falla."""
    try:
        with open('/proc/stat', 'r') as f:
            primera = f.readline()
        campos = primera.split()
        if not campos or campos[0] != 'cpu':
            return None
        valores = [int(v) for v in campos[1:]]
        # user nice system idle iowait irq softirq steal guest guest_nice
        idle = valores[3] + (valores[4] if len(valores) > 4 else 0)  # idle + iowait
        total = sum(valores)
        return total, idle
    except Exception:
        return None


def _leer_mem_pct():
    """% de RAM usada según MemTotal/MemAvailable de /proc/meminfo, o None."""
    try:
        total = disponible = None
        with open('/proc/meminfo', 'r') as f:
            for linea in f:
                if linea.startswith('MemTotal:'):
                    total = float(linea.split()[1])
                elif linea.startswith('MemAvailable:'):
                    disponible = float(linea.split()[1])
                if total is not None and disponible is not None:
                    break
        if not total or disponible is None:
            return None
        return round(100.0 * (total - disponible) / total, 1)
    except Exception:
        return None


def _leer_load1():
    """Load average de 1 minuto, o None."""
    try:
        with open('/proc/loadavg', 'r') as f:
            return round(float(f.readline().split()[0]), 2)
    except Exception:
        return None


def _leer_temp_c():
    """Temperatura del SoC en °C (de miligrados), o None."""
    try:
        with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
            return round(int(f.readline().strip()) / 1000.0, 1)
    except Exception:
        return None


class MonitorSistema:
    """Muestrea CPU/RAM/temp/load cada `periodo` s y los pasa por callback."""

    def __init__(self, callback, periodo=5.0):
        """`callback(cpu_pct, mem_pct, temp_c, load1)` — cualquiera puede ser None."""
        self._callback = callback
        self._periodo = max(1.0, float(periodo))
        self._stop = threading.Event()
        self._hilo = None
        self._prev_cpu = None  # (total, idle) de la lectura anterior

    def start(self):
        if self._hilo is not None:
            return
        self._prev_cpu = _leer_cpu_jiffies()
        self._hilo = threading.Thread(target=self._bucle, daemon=True)
        self._hilo.start()

    def stop(self):
        self._stop.set()

    def _cpu_pct(self):
        """%CPU global entre la lectura previa y la actual (None si no hay base)."""
        actual = _leer_cpu_jiffies()
        if actual is None or self._prev_cpu is None:
            self._prev_cpu = actual
            return None
        total0, idle0 = self._prev_cpu
        total1, idle1 = actual
        self._prev_cpu = actual
        d_total = total1 - total0
        d_idle = idle1 - idle0
        if d_total <= 0:
            return None
        return round(100.0 * (d_total - d_idle) / d_total, 1)

    def _bucle(self):
        while not self._stop.wait(self._periodo):
            try:
                cpu = self._cpu_pct()
                mem = _leer_mem_pct()
                temp = _leer_temp_c()
                load1 = _leer_load1()
                self._callback(cpu, mem, temp, load1)
            except Exception:
                # Nunca tumbar el nodo por un fallo de muestreo.
                pass
