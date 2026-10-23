#!/usr/bin/env python3
"""
measure_all_rates_live.py
Directly tests and logs 1 Hz diagnostics for all 6 sample rates on Node 21.
"""

import sys
import time
import serial

PORT = "COM121"
BAUD = 115200

RATES = [48000, 44100, 32000, 24000, 16000, 8000]

def main():
    s = serial.Serial(PORT, BAUD, timeout=0.5)
    time.sleep(0.5)
    s.reset_input_buffer()

    print("\n[STARTING BROADCAST]...")
    s.write(b"\r\nstart\r\n")
    s.flush()
    time.sleep(1.0)

    summary = []

    for sr in RATES:
        cmd = f"sr {sr}\r\n".encode("utf-8")
        s.write(cmd)
        s.flush()
        print(f"\n---> Set Sample Rate to {sr} Hz...")

        # Wait 1s for rate to settle, then capture lines for 3s
        time.sleep(1.0)
        start_t = time.time()
        diag_lines = []
        while time.time() - start_t < 3.0:
            line = s.readline().decode("utf-8", errors="replace").strip()
            if line:
                if line.startswith("|") and ("BROAD" in line or "LC3" in line or "IDLE" in line):
                    print(line)
                    diag_lines.append(line)
                elif "MAIN" in line or "ESP_LC3" in line:
                    print(line)

        if diag_lines:
            summary.append((sr, diag_lines[-1]))

    s.close()

    print("\n\n" + "="*110)
    print("                      ESP32-C6 SOURCE LIVE STREAMING METRICS SUMMARY (120 OCTETS / FRAME)")
    print("="*110)
    for sr, line in summary:
        print(f"[{sr:5d} Hz] {line}")
    print("="*110 + "\n")

if __name__ == "__main__":
    main()
