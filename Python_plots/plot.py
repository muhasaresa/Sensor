#!/usr/bin/env python3
"""
LivePlot_multiplots_Sync.py – 2025-06-12 rev-D
• Rescale button now handles bipolar COER data:
  axis becomes symmetric about 0 pF and fits current extrema.
• No canvas dependency; single Python file.
"""

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button
import serial, os, time
from collections import deque

# ────────── serial config ──────────
SERIAL_PORT = "COM7"
BAUD_RATE   = 115200

# ────────── ECG mapping / smoothing ──────────
RAW_ECG_MIN = -65000.0
RAW_ECG_MAX =  65000.0
TARGET_ECG_MIN = -2500.0
TARGET_ECG_MAX =  2500.0
ECG_SMOOTH_WINDOW = 3

# ────────── sensors & colours ──────────
sensors  = ["Coer Sensor", "P", "E"]
colors   = ["red", "blue", "black", "green", "pink"]
DATA_RETENTION_SECONDS = 30

class LivePlotter:
    def __init__(self):
        # open serial
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05)
            self.ser.reset_input_buffer()
        except serial.SerialException as e:
            raise SystemExit(e)

        # csv
        self.csv_files = {"C": "coer_sensor_data.csv",
                          "E": "ecg_data.csv",
                          "P": "ppg_data.csv"}
        self.csv = {}
        self.recording = False

        # plot scaffold
        self.fig, self.ax = plt.subplots(len(sensors), sharex=True, figsize=(15, 8))
        if len(sensors) == 1:
            self.ax = [self.ax]
        self.lines = []
        for i, name in enumerate(sensors):
            ln, = self.ax[i].plot([], [], lw=1.0, color=colors[i % len(colors)])
            self.lines.append(ln)
            self.ax[i].set_title(name)
            if name == "E":
                pad = (TARGET_ECG_MAX - TARGET_ECG_MIN) * 0.1
                self.ax[i].set_ylim(TARGET_ECG_MIN - pad, TARGET_ECG_MAX + pad)
            elif name == "Coer Sensor":
                self.ax[i].set_ylim(-25, 25)       # initial symmetric window
            elif name == "P":
                self.ax[i].set_ylim(-1.5, 1.5)
        self.ax[-1].set_xlabel("Time (s)")
        self.ax[-1].set_xlim(0, 10)

        # buttons
        self.b_rec = Button(plt.axes([0.70, 0.005, 0.13, 0.035]), "Record")
        self.b_rec.on_clicked(self.toggle_record)
        self.b_res = Button(plt.axes([0.85, 0.005, 0.13, 0.035]), "Rescale Y-Axes")
        self.b_res.on_clicked(self.rescale)

        # buffers
        self.buff = b""
        self.streams = [deque() for _ in sensors]
        self.ecg_ma  = deque(maxlen=ECG_SMOOTH_WINDOW)
        self.t0_ms = -1
        self.t_latest_ms = 0

        # animation
        self.ani = animation.FuncAnimation(self.fig, self.animate,
                                           interval=20, blit=True)
        self.fig.canvas.mpl_connect("close_event", self.close)

    # ────────── helpers ──────────
    def toggle_record(self, _):
        if not self.recording:
            os.makedirs("data", exist_ok=True)
            for k, fn in self.csv_files.items():
                self.csv[k] = open(os.path.join("data", fn), "a", newline="")
            self.recording = True
            self.b_rec.label.set_text("Stop Record")
        else:
            for f in self.csv.values():
                f.close()
            self.csv.clear()
            self.recording = False
            self.b_rec.label.set_text("Record")

    def rescale(self, _):
        """Fit each Y-axis to current 30-s window; COER axis symmetric."""
        if self.t0_ms < 0:
            return
        cur = (self.t_latest_ms - self.t0_ms) / 1000.0
        self.ax[-1].set_xlim(max(0, cur - 10), max(10, cur + 0.1))
        for i, name in enumerate(sensors):
            if not self.streams[i]:
                continue
            ys = [v for _, v in self.streams[i]]
            if name == "Coer Sensor":
                max_abs = max(abs(v) for v in ys)
                pad = max_abs * 0.1 or 1.0
                self.ax[i].set_ylim(-(max_abs + pad), max_abs + pad)
            else:
                y_min, y_max = min(ys), max(ys)
                if y_min == y_max:
                    pad = abs(y_max) * 0.1 or 1.0
                    y_min -= pad; y_max += pad
                else:
                    pad = (y_max - y_min) * 0.1
                    y_min -= pad; y_max += pad
                self.ax[i].set_ylim(y_min, y_max)
        self.fig.canvas.draw_idle()

    def close(self, _):
        if self.ser.is_open:
            self.ser.close()
        for f in self.csv.values():
            f.close()

    # ────────── main loop ──────────
    def animate(self, _):
        # read serial
        if self.ser.in_waiting:
            self.buff += self.ser.read(self.ser.in_waiting)

        # process lines
        while b"\n" in self.buff:
            line, self.buff = self.buff.split(b"\n", 1)
            if not line:
                continue
            try:
                s = line.decode("ascii", "replace").strip()
                if not s:
                    continue
                t_idx = s.rfind("T")
                if t_idx == -1 or t_idx + 9 > len(s):
                    continue
                ts_ms = int(s[t_idx + 1:t_idx + 9])
                if self.t0_ms == -1:
                    self.t0_ms = ts_ms
                self.t_latest_ms = max(self.t_latest_ms, ts_ms)
                rel_ms = ts_ms - self.t0_ms
                ts_sec = rel_ms / 1000.0

                # COER
                c_idx = s.find("C")
                if c_idx != -1:
                    end = t_idx if t_idx > c_idx else len(s)
                    fF = int(s[c_idx + 1:end])
                    pF = fF / 1000.0                       # plot units
                    self.streams[0].append((rel_ms, pF))
                    if self.recording:
                        self.csv["C"].write(f"{ts_sec:+.6E},{fF * 1e-15:+.6E}\n")

                # ECG
                e_idx = s.find("E")
                if e_idx != -1:
                    end = t_idx if t_idx > e_idx else len(s)
                    raw = int(s[e_idx + 1:end])
                    raw = max(RAW_ECG_MIN, min(raw, RAW_ECG_MAX))
                    mapped = ((raw - RAW_ECG_MIN) * (TARGET_ECG_MAX - TARGET_ECG_MIN) /
                              (RAW_ECG_MAX - RAW_ECG_MIN) + TARGET_ECG_MIN)
                    self.ecg_ma.append(mapped)
                    if len(self.ecg_ma) == ECG_SMOOTH_WINDOW:
                        mapped = sum(self.ecg_ma) / ECG_SMOOTH_WINDOW
                    idx = sensors.index("E")
                    self.streams[idx].append((rel_ms, mapped))
                    if self.recording:
                        self.csv["E"].write(f"{ts_sec:+.6E},{mapped:+.6E}\n")
            except Exception as exc:
                print("Parse error:", exc)

        # trim & draw
        win_ms = DATA_RETENTION_SECONDS * 1000
        for i, stream in enumerate(self.streams):
            while stream and stream[0][0] < self.t_latest_ms - self.t0_ms - win_ms:
                stream.popleft()
            if stream:
                xs = [t / 1000.0 for t, _ in stream]
                ys = [v for _, v in stream]
                self.lines[i].set_data(xs, ys)
            else:
                self.lines[i].set_data([], [])
        return self.lines

# ────────── run ──────────
if __name__ == "__main__":
    os.makedirs("data", exist_ok=True)
    plotter = LivePlotter()
    try:
        plt.tight_layout(rect=[0, 0.05, 1, 1])
        plt.show()
    finally:
        plotter.close(None)
