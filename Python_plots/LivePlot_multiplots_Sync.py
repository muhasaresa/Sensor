import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button
import serial
import numpy as np
from datetime import datetime
import pickle
import time
import os
from collections import deque

# Serial port configuration
SERIAL_PORT = 'COM7'
BAUD_RATE = 115200

# --- Configuration for ECG Data Mapping (in Python) ---
RAW_ECG_PYTHON_MIN = -3000.0
RAW_ECG_PYTHON_MAX = 3000.0
TARGET_DISPLAY_ECG_MIN = 0.0
TARGET_DISPLAY_ECG_MAX = 4000.0

# --- Configuration for Display Smoothing (ECG only) ---
ECG_DISPLAY_SMOOTHING_WINDOW = 5

sensors1 = ['Coer Sensor']
sensors2 = ['P', 'E']
sensors = sensors1 + sensors2
colors = ['red', 'blue', 'black', 'green', 'pink']

DATA_RETENTION_SECONDS = 30  # How many seconds of data to keep in memory for each stream


class Animation:
    def __init__(self):
        self.ser = None
        self.ani = None
        self.f = None
        self.plot_start_time_ms = -1  # Initialize when first data with timestamp arrives
        self.latest_data_time_ms = 0  # Tracks the latest timestamp from any sensor

        try:
            print(f"Attempting to open serial port {SERIAL_PORT} at {BAUD_RATE} baud...")
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05)
            self.ser.flushInput()
            self.ser.flushOutput()
            print(f"Serial port {SERIAL_PORT} opened successfully.")
        except serial.SerialException as e:
            print(f"CRITICAL Error opening serial port {SERIAL_PORT}: {e}")
            if self.ser and self.ser.is_open: self.ser.close()
            return

        time.sleep(0.1)

        self.esp32_ecg_send_interval_ms = 5  # ECG sends data every 5ms (200 Hz)
        # The X-axis will be based on actual timestamps, not a fixed sampling rate assumption
        print(f"ECG data expected at ~{1000 / self.esp32_ecg_send_interval_ms:.1f} Hz.")

        self.record = False
        self.buttonText = 'Record'
        self.numplots = len(sensors)
        self.x_display_len_seconds = 10  # How many seconds the plot window initially shows

        # Data storage: list of deques, one for each sensor.
        # Each inner deque will store (relative_timestamp_ms, value) tuples.
        self.sensor_data_streams = [deque() for _ in range(self.numplots)]

        # Store last known values (used if a sensor updates less frequently than plot rate)
        # For timestamp-based plotting, this is less about filling gaps and more for initial state.
        self.last_known_sensor_values = [0.0] * self.numplots

        if 'E' in sensors:
            self.ecg_mapped_for_smoothing_buffer = deque(maxlen=ECG_DISPLAY_SMOOTHING_WINDOW)
            ecg_baseline = (TARGET_DISPLAY_ECG_MAX - TARGET_DISPLAY_ECG_MIN) / 2 + TARGET_DISPLAY_ECG_MIN
            for _ in range(ECG_DISPLAY_SMOOTHING_WINDOW):
                self.ecg_mapped_for_smoothing_buffer.append(ecg_baseline)
            # Initialize last known for ECG if needed, though timestamp plotting reduces its role
            if sensors.index('E') < len(self.last_known_sensor_values):
                self.last_known_sensor_values[sensors.index('E')] = ecg_baseline

        if 'Coer Sensor' in sensors:
            coer_idx = sensors.index('Coer Sensor')
            if coer_idx < len(self.last_known_sensor_values):
                self.last_known_sensor_values[coer_idx] = 10.0  # Approx mid pF

        # --- Figure and Axes Setup ---
        self.fig, self.ax = plt.subplots(self.numplots, sharex=True, figsize=(15, 8))  # figsize (width, height)
        if self.numplots == 1: self.ax = [self.ax]

        self.lines = []
        for i in range(self.numplots):
            # Initial plot with empty data, X-axis will be set dynamically
            line_obj, = self.ax[i].plot([], [], lw=1.0)
            line_obj.set_color(colors[i % len(colors)])
            self.lines.append(line_obj)
            self.ax[i].set_title(sensors[i])

            # Set initial Y-limits
            if sensors[i] == 'E':
                self.ax[i].set_ylim(TARGET_DISPLAY_ECG_MIN - (TARGET_DISPLAY_ECG_MAX * 0.1),
                                    TARGET_DISPLAY_ECG_MAX + (TARGET_DISPLAY_ECG_MAX * 0.1))
            elif sensors[i] == 'Coer Sensor':
                self.ax[i].set_ylim(0, 25)
            elif sensors[i] == 'P':
                self.ax[i].set_ylim(-1.5, 1.5)  # Example for PPG
            else:
                self.ax[i].set_ylim(-1, 1)

        self.ax[self.numplots - 1].set_xlabel(f'Time (seconds from start)')
        # Initial X-axis limits (will be updated in animate)
        self.ax[self.numplots - 1].set_xlim(0, self.x_display_len_seconds)

        # --- Button Setup ---
        self.button_ax = plt.axes([0.7, 0.005, 0.13, 0.035])
        self.button_record = Button(self.button_ax, self.buttonText)
        self.button_record.on_clicked(self.record_click)

        self.button_rs_ax = plt.axes([0.85, 0.005, 0.13, 0.035])
        self.button_rescale = Button(self.button_rs_ax, 'Rescale Y-Axes')
        self.button_rescale.on_clicked(self.rescale_click)

        self.serial_buffer = b''

        animation_interval = 20  # ms, target for GUI update (50 FPS). Adjust if needed.
        print(f"Animation FuncAnimation interval set to: {animation_interval} ms")
        self.ani = animation.FuncAnimation(self.fig, self.animate, interval=animation_interval, blit=True,
                                           cache_frame_data=False)
        self.fig.canvas.mpl_connect('close_event', self.on_close)

        if self.ser: self.ser.flushInput()
        print("Animation initialized.")

    def on_close(self, event):
        print("Close event triggered.")
        if hasattr(self.ani, 'event_source') and self.ani.event_source:
            self.ani.event_source.stop()
            print("Animation stopped.")
        if self.ser and self.ser.is_open:
            print("Closing serial port.")
            self.ser.close()
        if self.f and not self.f.closed:
            print('Closing data file.')
            self.f.flush();
            os.fsync(self.f.fileno());
            self.f.close()
            print('File closed!')

    def record_click(self, event):
        if not self.record:
            if not os.path.exists("data"): os.makedirs("data")
            now = datetime.now()
            fname_sensors = "_".join(s.replace(" ", "") for s in sensors)
            pickle_filename = os.path.join("data", now.strftime('%Y%m%d_%H%M%S_') + fname_sensors + '.pkl')
            try:
                self.f = open(pickle_filename, 'ab')
                config_info = {
                    'sensors': sensors, 'start_timestamp_iso': now.isoformat(),
                    'x_axis_source_sampling_rate_ms': self.esp32_ecg_send_interval_ms,  # What drives the x-axis updates
                    'ecg_raw_map_min': RAW_ECG_PYTHON_MIN, 'ecg_raw_map_max': RAW_ECG_PYTHON_MAX,
                    'ecg_display_map_min': TARGET_DISPLAY_ECG_MIN, 'ecg_display_map_max': TARGET_DISPLAY_ECG_MAX,
                    'ecg_display_smoothing_window': ECG_DISPLAY_SMOOTHING_WINDOW if 'E' in sensors else 0
                }
                pickle.dump(config_info, self.f)
                self.record = True
                self.buttonText = 'Stop Record'
                self.button_record.label.set_text(self.buttonText)
                print(f'Recording to {pickle_filename}')
            except Exception as e:
                print(f"Error opening file for recording: {e}")
        else:
            self.record = False
            self.buttonText = 'Record'
            self.button_record.label.set_text(self.buttonText)
            if self.f and not self.f.closed:
                print('Recording stopped.');
                self.f.flush();
                os.fsync(self.f.fileno());
                self.f.close();
                self.f = None
            else:
                print("Stop record: No file open.")
        if hasattr(plt.gcf(), 'canvas') and plt.gcf().canvas: plt.gcf().canvas.draw_idle()

    def rescale_click(self, event):
        print("Rescaling Y-axes...")
        if self.plot_start_time_ms == -1:  # No data yet to determine plot window
            print("No data received yet to rescale.")
            return

        current_max_relative_time_ms = self.latest_data_time_ms - self.plot_start_time_ms
        display_window_start_time_ms = max(0, current_max_relative_time_ms - self.x_display_len_seconds * 1000)

        for i in range(self.numplots):
            if not self.sensor_data_streams[i]: continue

            y_values_in_window = [val for ts_rel, val in self.sensor_data_streams[i]
                                  if ts_rel >= display_window_start_time_ms and ts_rel <= current_max_relative_time_ms]

            if len(y_values_in_window) > 1 and len(set(y_values_in_window)) > 1:
                min_val = min(y_values_in_window)
                max_val = max(y_values_in_window)
                padding = (max_val - min_val) * 0.10
                if padding == 0: padding = abs(min_val * 0.1) if min_val != 0 else 1.0
                self.ax[i].set_ylim(min_val - padding, max_val + padding)
            elif len(y_values_in_window) > 0:
                self.ax[i].set_ylim(y_values_in_window[0] - 1, y_values_in_window[0] + 1)
        if hasattr(plt.gcf(), 'canvas') and plt.gcf().canvas: self.fig.canvas.draw_idle()

    def animate(self, frame_num):
        if not (self.ser and self.ser.is_open): return self.lines

        if self.ser.in_waiting > 0:
            try:
                data_bytes = self.ser.read(self.ser.in_waiting)
                self.serial_buffer += data_bytes
            except Exception as e:
                print(f"Error reading serial: {e}")
                return self.lines

        new_data_was_added_to_streams = False
        while b'\n' in self.serial_buffer:
            line_bytes, self.serial_buffer = self.serial_buffer.split(b'\n', 1)
            if not line_bytes: continue

            new_data_was_added_to_streams = True
            if self.record and self.f and not self.f.closed: pickle.dump(line_bytes, self.f)

            try:
                line_str = line_bytes.decode('ascii', errors='replace').strip()
                if not line_str: continue

                t_idx_global = line_str.rfind('T')
                timestamp_ms_from_packet = -1
                if t_idx_global != -1 and t_idx_global + 1 + 8 <= len(line_str):
                    ts_str = line_str[t_idx_global + 1: t_idx_global + 1 + 8]
                    if ts_str.isdigit():
                        timestamp_ms_from_packet = int(ts_str)
                        if self.plot_start_time_ms == -1:
                            self.plot_start_time_ms = timestamp_ms_from_packet
                            print(f"Plot time origin set: {self.plot_start_time_ms} ms")
                        self.latest_data_time_ms = max(self.latest_data_time_ms, timestamp_ms_from_packet)

                if timestamp_ms_from_packet == -1: continue

                relative_ts_ms = timestamp_ms_from_packet - self.plot_start_time_ms if self.plot_start_time_ms != -1 else 0

                # Coer Sensor Parsing
                c_idx = line_str.find('C')
                if c_idx != -1 and 'Coer Sensor' in sensors:
                    val_end_c = t_idx_global if t_idx_global > c_idx else len(line_str)
                    val_str_c = line_str[c_idx + 1: val_end_c]
                    if val_str_c:
                        try:
                            capacitance_val = float(val_str_c) / 1000.0
                            self.sensor_data_streams[sensors.index('Coer Sensor')].append(
                                (relative_ts_ms, capacitance_val))
                        except ValueError:
                            pass

                # ECG Parsing
                e_idx = line_str.find('E')
                if e_idx != -1 and 'E' in sensors:
                    val_end_e = t_idx_global if t_idx_global > e_idx else len(line_str)
                    ecg_val_str = line_str[e_idx + 1: val_end_e]
                    if ecg_val_str:
                        try:
                            raw_ecg_value = int(ecg_val_str.strip())
                            clamped_raw_ecg = max(RAW_ECG_PYTHON_MIN, min(raw_ecg_value, RAW_ECG_PYTHON_MAX))
                            if (RAW_ECG_PYTHON_MAX == RAW_ECG_PYTHON_MIN):
                                mapped_val = TARGET_DISPLAY_ECG_MIN
                            else:
                                mapped_val = (((clamped_raw_ecg - RAW_ECG_PYTHON_MIN) *
                                               (TARGET_DISPLAY_ECG_MAX - TARGET_DISPLAY_ECG_MIN)) /
                                              (RAW_ECG_PYTHON_MAX - RAW_ECG_PYTHON_MIN)
                                              ) + TARGET_DISPLAY_ECG_MIN

                            self.ecg_mapped_for_smoothing_buffer.append(mapped_val)
                            smoothed_val = mapped_val
                            if len(self.ecg_mapped_for_smoothing_buffer) == ECG_DISPLAY_SMOOTHING_WINDOW:
                                smoothed_val = sum(self.ecg_mapped_for_smoothing_buffer) / ECG_DISPLAY_SMOOTHING_WINDOW

                            self.sensor_data_streams[sensors.index('E')].append((relative_ts_ms, smoothed_val))
                        except ValueError:
                            pass

                # PPG ('P') Parsing Placeholder
                p_idx = line_str.find('P')
                if p_idx != -1 and 'P' in sensors:
                    val_end_p = t_idx_global if t_idx_global > p_idx else len(line_str)
                    ppg_val_str = line_str[p_idx + 1: val_end_p]
                    if ppg_val_str:
                        try:
                            ppg_value = float(ppg_val_str.strip())
                            self.sensor_data_streams[sensors.index('P')].append((relative_ts_ms, ppg_value))
                        except ValueError:
                            pass

            except Exception as e:
                print(f"Error processing line: {line_bytes[:50]}... Error: {e}")

        # Update X-axis limits dynamically
        if self.plot_start_time_ms != -1:
            current_max_relative_time_sec = (self.latest_data_time_ms - self.plot_start_time_ms) / 1000.0
            # Set xlim to show the last x_display_len_seconds of data
            self.ax[self.numplots - 1].set_xlim(
                max(0, current_max_relative_time_sec - self.x_display_len_seconds),
                current_max_relative_time_sec + 0.1  # Small buffer to the right
            )

        for i, sensor_name in enumerate(sensors):
            # Prune old data outside the retention window
            min_retain_relative_time_ms = (self.latest_data_time_ms - self.plot_start_time_ms) - (
                        DATA_RETENTION_SECONDS * 1000)
            while self.sensor_data_streams[i] and self.sensor_data_streams[i][0][0] < min_retain_relative_time_ms:
                self.sensor_data_streams[i].popleft()

            if self.sensor_data_streams[i]:
                plot_xs = [(ts_rel_ms / 1000.0) for ts_rel_ms, val in self.sensor_data_streams[i]]
                plot_ys = [val for ts_rel_ms, val in self.sensor_data_streams[i]]
                self.lines[i].set_data(plot_xs, plot_ys)
            else:  # If no data in the stream (e.g. after pruning and no new data)
                self.lines[i].set_data([], [])

        return self.lines


if __name__ == '__main__':
    if not os.path.exists("data"):
        try:
            os.makedirs("data"); print("Created 'data' directory.")
        except OSError as e:
            print(f"Error creating 'data' dir: {e}.")

    print("Starting plotter with timestamp-based X-axis...")
    plot_animation = Animation()

    if hasattr(plot_animation, 'ser') and plot_animation.ser is not None and plot_animation.ser.is_open:
        try:
            plt.tight_layout(rect=[0, 0.05, 1, 1])  # Adjust rect to ensure buttons are visible
            plt.show()
        except Exception as e:
            print(f"Error during plot.show(): {e}")
        finally:
            print("Plot window exited.")
            if hasattr(plot_animation, 'ani') and plot_animation.ani and hasattr(plot_animation.ani, 'event_source'):
                plot_animation.ani.event_source.stop()
            if hasattr(plot_animation, 'ser') and plot_animation.ser and plot_animation.ser.is_open:
                plot_animation.ser.close()
            if hasattr(plot_animation, 'f') and plot_animation.f and not plot_animation.f.closed:
                plot_animation.f.close()
        print("Script finished.")
    else:
        print("Plotter not started due to serial init failure.")