#!/usr/bin/env python3
"""
ap_sine_receiver_20k.py  –  Connect to ESP32 soft-AP at 192.168.4.1:5000,
                            receive 20 k samples (1 s) and plot them.
"""

import socket, struct, numpy as np, matplotlib.pyplot as plt

ESP_IP, ESP_PORT = '192.168.4.1', 5000
FS_HZ, SECS = 20_000, 1
BYTES_F = 4
N = FS_HZ * SECS

with socket.create_connection((ESP_IP, ESP_PORT), timeout=10) as s:
    raw = bytearray()
    while len(raw) < N * BYTES_F:
        raw.extend(s.recv(8192))

floats = struct.unpack('<' + 'f' * N, raw[:N * BYTES_F])
t = np.arange(N) / FS_HZ

plt.plot(t, floats)
plt.xlabel('Time (s)')
plt.ylabel('Amplitude')
plt.title('1 kHz sine streamed over Wi-Fi (20 kHz sampling)')
plt.xlim(0, 0.005)        # five ms → five cycles
plt.grid(True)
plt.show()
