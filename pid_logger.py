"""
gimbal_pid_plotter.py
---------------------
High-performance real-time scrolling plotter for the ESP32-C3 gimbal.
Uses pyqtgraph (OpenGL-accelerated) — handles 60 fps easily.

Usage:
    python gimbal_pid_plotter.py              # COM9, 115200 baud
    python gimbal_pid_plotter.py COM3
    python gimbal_pid_plotter.py COM9 500000

Requirements:
    pip install pyserial pyqtgraph PyQt6
    (or PyQt5:  pip install pyserial pyqtgraph PyQt5)
"""

import sys
import threading
import collections
import datetime
import csv
import serial
import numpy as np

import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore

# ─── Config ───────────────────────────────────────────────────────────────────

PORT     = sys.argv[1] if len(sys.argv) > 1 else "COM9"
BAUD     = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
WINDOW   = 300   # samples visible (~12 s at 25 Hz)
LOG_FILE = f"gimbal_log_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

# ─── Column order (matches firmware TUNE line) ────────────────────────────────

HEADERS = [
    "time_ms",
    "roll_raw",        "roll_filt",      "roll_cmd_dps",  "roll_err",
    "pitch_raw",       "pitch_filt",     "pitch_cmd_dps", "pitch_err",
    "roll_mot_angle",  "roll_mot_speed",
    "pitch_mot_angle", "pitch_mot_speed",
]

# ─── Ring buffers (one per channel) ──────────────────────────────────────────

bufs     = {h: collections.deque([0.0] * WINDOW, maxlen=WINDOW) for h in HEADERS}
lock     = threading.Lock()
running  = True
new_data = threading.Event()

# ─── Serial reader (background thread) ───────────────────────────────────────

def serial_reader():
    global running

    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        print(f"[OK] {PORT} @ {BAUD} baud")
    except serial.SerialException as e:
        print(f"[ERROR] {e}")
        running = False
        return

    with open(LOG_FILE, "w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(HEADERS)

        while running:
            try:
                raw  = ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()

                if not line.startswith("TUNE,"):
                    continue

                parts = line.split(",")
                if len(parts) != 14:
                    continue

                try:
                    values = [float(p) for p in parts[1:]]
                except ValueError:
                    continue

                row = dict(zip(HEADERS, values))
                writer.writerow([row.get(h, "") for h in HEADERS])
                csv_file.flush()

                with lock:
                    for h, v in row.items():
                        bufs[h].append(v)
                new_data.set()

            except serial.SerialException:
                print("[WARN] Serial disconnected")
                break

    ser.close()


# ─── UI ───────────────────────────────────────────────────────────────────────

pg.setConfigOptions(antialias=True, useOpenGL=True)

app = QtWidgets.QApplication(sys.argv)

win = pg.GraphicsLayoutWidget(title=f"Gimbal PID Monitor — {PORT}")
win.resize(1400, 860)
win.setBackground("#0d0d1a")

x = np.arange(WINDOW, dtype=np.float32)


def make_plot(row, col, title, ylabel, yrange, legend_items):
    """Add one scrolling panel and return its list of curves."""
    p = win.addPlot(row=row, col=col)
    p.setTitle(f"<span style='color:#ccccee;font-size:9pt'>{title}</span>")
    p.setLabel("left", f"<span style='color:#8888aa'>{ylabel}</span>")
    p.setXRange(0, WINDOW, padding=0)
    p.setYRange(*yrange, padding=0.05)
    p.showGrid(x=False, y=True, alpha=0.2)
    p.getAxis("left").setTextPen("#8888aa")
    p.getAxis("bottom").setTextPen("#8888aa")
    p.setMenuEnabled(False)
    p.hideButtons()
    p.addItem(pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen("#333355", width=1)))

    p.addLegend(offset=(1, 1),
                labelTextColor="#aaaacc",
                brush=pg.mkBrush("#0d0d1acc"),
                pen=pg.mkPen("#222244"))

    curves = []
    for label, color, width, style in legend_items:
        pen = pg.mkPen(
            color=color, width=width,
            style=QtCore.Qt.PenStyle.DashLine if style == "dash"
                  else QtCore.Qt.PenStyle.SolidLine
        )
        curves.append(p.plot(x, np.zeros(WINDOW), pen=pen, name=label))

    return curves


# Row 0 — angles
roll_angle_curves  = make_plot(0, 0, "ROLL — angle (deg)",  "deg", (-45,  45), [
    ("raw",      "#4fc3f788", 1.0, "solid"),
    ("filtered", "#00e5ff",   1.8, "solid"),
])
pitch_angle_curves = make_plot(0, 1, "PITCH — angle (deg)", "deg", (-45,  45), [
    ("raw",      "#ce93d888", 1.0, "solid"),
    ("filtered", "#ea80fc",   1.8, "solid"),
])

# Row 1 — speeds
roll_speed_curves  = make_plot(1, 0, "ROLL — speed (dps)",  "dps", (-110, 110), [
    ("cmd dps",   "#ffb74d", 1.4, "dash"),
    ("motor dps", "#ff8a65", 1.8, "solid"),
])
pitch_speed_curves = make_plot(1, 1, "PITCH — speed (dps)", "dps", (-110, 110), [
    ("cmd dps",   "#80cbc4", 1.4, "dash"),
    ("motor dps", "#4db6ac", 1.8, "solid"),
])

# Row 2 — errors
roll_err_curves  = make_plot(2, 0, "ROLL — error (deg)",  "deg", (-25, 25), [
    ("error", "#ef5350", 1.8, "solid"),
])
pitch_err_curves = make_plot(2, 1, "PITCH — error (deg)", "deg", (-25, 25), [
    ("error", "#f06292", 1.8, "solid"),
])

# Status bar below the plot grid
status_label = QtWidgets.QLabel(f"  Connecting to {PORT}…   |   log: {LOG_FILE}")
status_label.setStyleSheet(
    "color:#6666aa; background:#0d0d1a; font-size:11px; padding:3px 8px;"
)

root = QtWidgets.QWidget()
vbox = QtWidgets.QVBoxLayout(root)
vbox.setContentsMargins(0, 0, 0, 0)
vbox.setSpacing(0)
vbox.addWidget(win)
vbox.addWidget(status_label)
root.setWindowTitle(f"Gimbal PID Monitor — {PORT}")
root.resize(1400, 890)
root.show()


# ─── Qt timer drives all UI updates (no threading conflict with Qt) ───────────

def update():
    if not new_data.is_set():
        return
    new_data.clear()

    with lock:
        r_raw  = np.array(bufs["roll_raw"],        dtype=np.float32)
        r_filt = np.array(bufs["roll_filt"],        dtype=np.float32)
        r_cmd  = np.array(bufs["roll_cmd_dps"],     dtype=np.float32)
        r_mspd = np.array(bufs["roll_mot_speed"],   dtype=np.float32)
        r_err  = np.array(bufs["roll_err"],         dtype=np.float32)

        p_raw  = np.array(bufs["pitch_raw"],        dtype=np.float32)
        p_filt = np.array(bufs["pitch_filt"],       dtype=np.float32)
        p_cmd  = np.array(bufs["pitch_cmd_dps"],    dtype=np.float32)
        p_mspd = np.array(bufs["pitch_mot_speed"],  dtype=np.float32)
        p_err  = np.array(bufs["pitch_err"],        dtype=np.float32)

    roll_angle_curves[0].setData(x, r_raw)
    roll_angle_curves[1].setData(x, r_filt)

    roll_speed_curves[0].setData(x, r_cmd)
    roll_speed_curves[1].setData(x, r_mspd)

    roll_err_curves[0].setData(x, r_err)

    pitch_angle_curves[0].setData(x, p_raw)
    pitch_angle_curves[1].setData(x, p_filt)

    pitch_speed_curves[0].setData(x, p_cmd)
    pitch_speed_curves[1].setData(x, p_mspd)

    pitch_err_curves[0].setData(x, p_err)

    status_label.setText(
        f"  {PORT} @ {BAUD}  |  "
        f"roll  err {r_err[-1]:+.2f}°  cmd {r_cmd[-1]:+.1f} dps  motor {r_mspd[-1]:+.1f} dps  |  "
        f"pitch err {p_err[-1]:+.2f}°  cmd {p_cmd[-1]:+.1f} dps  motor {p_mspd[-1]:+.1f} dps  |  "
        f"log: {LOG_FILE}"
    )


timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(16)   # 60 fps UI cap; serial thread fills the buffer independently


# ─── Entry point ──────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print(f"[INFO] Port:   {PORT}")
    print(f"[INFO] Baud:   {BAUD}")
    print(f"[INFO] Log:    {LOG_FILE}")
    print(f"[INFO] Window: {WINDOW} samples (~{WINDOW*40//1000} s at 25 Hz)")
    print("[INFO] Close the window to stop\n")

    reader = threading.Thread(target=serial_reader, daemon=True)
    reader.start()

    code = app.exec()
    running = False
    print(f"\n[INFO] Saved: {LOG_FILE}")
    sys.exit(code)