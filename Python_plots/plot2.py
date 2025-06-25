#!/usr/bin/env python3
"""
LivePlot_multiplots_Sync.py – 2025-06-12 rev-F (COER Debug)
• Rescale button now handles bipolar COER data:
  axis becomes symmetric about 0 pF and fits current extrema.
• No canvas dependency; single Python file.
• Removed PPG sensor.
• Added debugging for COER and improved integer parsing.
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
sensors  = ["Coer Sensor", "E"] # Order should match plot order and stream index
colors   = ["red", "black", "blue", "green", "pink"]
DATA_RETENTION_SECONDS = 30
DISPLAY_WINDOW_SECONDS = 10.0

# Helper function to check if a string represents an integer
def is_int_string(s_val):
    s_val = s_val.strip()
    if not s_val:
        return False
    if s_val[0] in ('-', '+'):
        return s_val[1:].isdigit()
    return s_val.isdigit()

class LivePlotter:
    def __init__(self):
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05)
            self.ser.reset_input_buffer()
        except serial.SerialException as e:
            raise SystemExit(e)

        self.csv_files = {"C": "coer_sensor_data.csv",
                          "E": "ecg_data.csv"}
        self.csv = {}
        self.recording = False

        self.fig, self.ax = plt.subplots(len(sensors), sharex=True, figsize=(15, 8))
        if len(sensors) == 1:
            self.ax = [self.ax]
        self.lines = []
        self.sensor_indices = {name: i for i, name in enumerate(sensors)}

        for i, name in enumerate(sensors):
            ln, = self.ax[i].plot([], [], lw=1.0, color=colors[i % len(colors)])
            self.lines.append(ln)
            self.ax[i].set_title(name)
            if name == "E":
                pad = (TARGET_ECG_MAX - TARGET_ECG_MIN) * 0.1
                self.ax[i].set_ylim(TARGET_ECG_MIN - pad, TARGET_ECG_MAX + pad)
            elif name == "Coer Sensor":
                self.ax[i].set_ylim(-25, 25)
        self.ax[-1].set_xlabel("Time (s)")
        self.ax[-1].set_xlim(0, DISPLAY_WINDOW_SECONDS)

        self.b_rec = Button(plt.axes([0.70, 0.005, 0.13, 0.035]), "Record")
        self.b_rec.on_clicked(self.toggle_record)
        self.b_res = Button(plt.axes([0.85, 0.005, 0.13, 0.035]), "Rescale Y-Axes")
        self.b_res.on_clicked(self.rescale)

        self.buff = b""
        self.streams = [deque() for _ in sensors]
        self.ecg_ma  = deque(maxlen=ECG_SMOOTH_WINDOW)
        self.t0_ms = -1
        self.t_latest_ms = 0

        self.ani = animation.FuncAnimation(self.fig, self.animate,
                                           interval=20,
                                           blit=False,
                                           cache_frame_data=False)
        self.fig.canvas.mpl_connect("close_event", self.close)

    def toggle_record(self, _):
        if not self.recording:
            os.makedirs("data", exist_ok=True)
            for k, fn in self.csv_files.items():
                try:
                    if k == "C" and "Coer Sensor" in self.sensor_indices:
                        self.csv[k] = open(os.path.join("data", fn), "a", newline="")
                    elif k == "E" and "E" in self.sensor_indices:
                        self.csv[k] = open(os.path.join("data", fn), "a", newline="")
                except Exception as e:
                    print(f"Error opening CSV for {k}: {e}")
            self.recording = True
            self.b_rec.label.set_text("Stop Record")
        else:
            for f in self.csv.values():
                f.close()
            self.csv.clear()
            self.recording = False
            self.b_rec.label.set_text("Record")

    def rescale(self, _):
        if self.t0_ms < 0: return
        current_time_sec = (self.t_latest_ms - self.t0_ms) / 1000.0
        margin_sec = 0.5
        x_max_view = current_time_sec + margin_sec
        x_min_view = x_max_view - DISPLAY_WINDOW_SECONDS
        if current_time_sec < (DISPLAY_WINDOW_SECONDS - margin_sec):
            self.ax[-1].set_xlim(0, DISPLAY_WINDOW_SECONDS)
        else:
            self.ax[-1].set_xlim(x_min_view, x_max_view)

        for i, name in enumerate(sensors):
            stream_idx = self.sensor_indices.get(name)
            if stream_idx is None or not self.streams[stream_idx]: continue
            ys = [v for _, v in self.streams[stream_idx]]
            if not ys: continue
            if name == "Coer Sensor":
                max_abs = max(abs(v) for v in ys) if ys else 0
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
                if name == "E":
                    ecg_pad = (TARGET_ECG_MAX - TARGET_ECG_MIN) * 0.1
                    default_ecg_min, default_ecg_max = TARGET_ECG_MIN - ecg_pad, TARGET_ECG_MAX + ecg_pad
                    y_min = min(y_min, default_ecg_min)
                    y_max = max(y_max, default_ecg_max)
                self.ax[i].set_ylim(y_min, y_max)
        self.fig.canvas.draw_idle()

    def close(self, _):
        if self.ser.is_open: self.ser.close()
        for f in self.csv.values(): f.close()

    def animate(self, _):
        if self.ser.in_waiting:
            self.buff += self.ser.read(self.ser.in_waiting)

        while b"\n" in self.buff:
            line, self.buff = self.buff.split(b"\n", 1)
            if not line: continue
            try:
                s = line.decode("ascii", "replace").strip()
                print(f"DEBUG: Raw line received: '{s}'") # <<< PRINT RAW LINE

                if not s: continue

                t_idx = s.rfind("T")
                if t_idx == -1:
                    print(f"DEBUG: No 'T' (timestamp delimiter) in line: '{s}'")
                    continue

                ts_str = s[t_idx + 1:]
                if not ts_str or not is_int_string(ts_str): # Use is_int_string for timestamp too
                    print(f"DEBUG: Invalid timestamp string '{ts_str}' in line: '{s}'")
                    continue
                ts_ms = int(ts_str)

                if self.t0_ms == -1: self.t0_ms = ts_ms
                self.t_latest_ms = max(self.t_latest_ms, ts_ms)
                rel_ms = ts_ms - self.t0_ms
                ts_sec = rel_ms / 1000.0

                # Coer Sensor
                if "Coer Sensor" in self.sensor_indices:
                    c_idx = s.find("C")
                    if c_idx != -1 and c_idx < t_idx:
                        val_str_coer = s[c_idx + 1:t_idx]
                        print(f"DEBUG COER: Found 'C'. Extracted val_str_coer: '{val_str_coer}' from full line: '{s}'")
                        if is_int_string(val_str_coer):
                            fF = int(val_str_coer)
                            pF = fF / 1000.0
                            self.streams[self.sensor_indices["Coer Sensor"]].append((rel_ms, pF))
                            if self.recording and "C" in self.csv:
                                self.csv["C"].write(f"{ts_sec:+.6E},{fF * 1e-15:+.6E}\n")
                            print(f"DEBUG COER: Successfully parsed value: {pF} pF")
                        else:
                            print(f"DEBUG COER: val_str_coer '{val_str_coer}' is NOT a valid integer string.")
                    # Optional: Add an else here if you want to know if 'C' was found but not before 'T'
                    # elif c_idx != -1:
                    #     print(f"DEBUG COER: Found 'C' but not before 'T' or 'T' was missing. c_idx={c_idx}, t_idx={t_idx}. Line: '{s}'")


                # ECG ("E")
                if "E" in self.sensor_indices:
                    e_idx = s.find("E")
                    if e_idx != -1 and e_idx < t_idx:
                        val_str_ecg = s[e_idx + 1:t_idx]
                        # print(f"DEBUG ECG: Found 'E'. Extracted val_str_ecg: '{val_str_ecg}'") # Optional ECG debug
                        if is_int_string(val_str_ecg):
                            raw = int(val_str_ecg)
                            raw = max(RAW_ECG_MIN, min(raw, RAW_ECG_MAX))
                            mapped = ((raw - RAW_ECG_MIN) * (TARGET_ECG_MAX - TARGET_ECG_MIN) /
                                      (RAW_ECG_MAX - RAW_ECG_MIN) + TARGET_ECG_MIN)
                            self.ecg_ma.append(mapped)
                            if len(self.ecg_ma) == ECG_SMOOTH_WINDOW:
                                mapped = sum(self.ecg_ma) / ECG_SMOOTH_WINDOW
                            self.streams[self.sensor_indices["E"]].append((rel_ms, mapped))
                            if self.recording and "E" in self.csv:
                                self.csv["E"].write(f"{ts_sec:+.6E},{mapped:+.6E}\n")
                        # else:
                            # print(f"DEBUG ECG: val_str_ecg '{val_str_ecg}' is NOT a valid integer string.")


            except Exception as exc:
                print(f"Parse error: {exc} on line: '{s}'")

        win_ms = DATA_RETENTION_SECONDS * 1000
        for i, stream_idx in enumerate(self.sensor_indices.values()):
            stream = self.streams[stream_idx]
            while stream and stream[0][0] < self.t_latest_ms - self.t0_ms - win_ms:
                stream.popleft()
            if stream:
                xs = [(t / 1000.0) for t, _ in stream]
                ys = [v for _, v in stream]
                self.lines[i].set_data(xs, ys)
            else:
                self.lines[i].set_data([], [])

        if self.t0_ms >= 0:
            current_latest_data_time_sec = (self.t_latest_ms - self.t0_ms) / 1000.0
            margin_sec = 0.5
            x_max_new = current_latest_data_time_sec + margin_sec
            x_min_new = x_max_new - DISPLAY_WINDOW_SECONDS
            if current_latest_data_time_sec < (DISPLAY_WINDOW_SECONDS - margin_sec):
                self.ax[-1].set_xlim(0, DISPLAY_WINDOW_SECONDS)
            else:
                self.ax[-1].set_xlim(x_min_new, x_max_new)
        return self.lines

if __name__ == "__main__":
    os.makedirs("data", exist_ok=True)
    plotter = LivePlotter()
    try:
        plt.tight_layout(rect=[0, 0.06, 1, 0.96])
        plt.show()
    finally:
        plotter.close(None)