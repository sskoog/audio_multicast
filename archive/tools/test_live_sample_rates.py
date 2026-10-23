#!/usr/bin/env python3
"""
test_live_sample_rates.py
Connects to Node 21 (ESP32-C6 SOURCE) on COM121,
activates BROADCASTING, and tests every sample rate (48k, 44.1k, 32k, 24k, 16k, 8k)
while capturing real-time CODEC ms duration and transmission metrics.
"""

import sys
import time
import serial

PORT = "COM121"
BAUD = 115200

RATES_TO_TEST = [48000, 44100, 32000, 24000, 16000, 8000]

def main():
    try:
        s = serial.Serial(PORT, BAUD, timeout=1.0)
    except Exception as e:
        print(f"Error opening {PORT}: {e}")
        return 1

    time.sleep(0.5)
    s.reset_input_buffer()

    print("\n[TEST] Putting Node 21 into BROADCASTING mode...")
    s.write(b"\r\nstart\r\n")
    s.flush()
    time.sleep(1.0)

    results = []

    for sr in RATES_TO_TEST:
        print(f"\n========================================================")
        print(f"  Switching SOURCE to Sample Rate: {sr} Hz ({sr/1000.0:.1f} kHz)")
        print(f"========================================================")
        cmd = f"sr {sr}\r\n".encode("utf-8")
        s.write(cmd)
        s.flush()

        start_t = time.time()
        table_lines = []
        while time.time() - start_t < 4.0:
            line = s.readline().decode("utf-8", errors="replace").strip()
            if line:
                print(line)
                if line.startswith("|") and ("BROAD" in line or "LC3" in line):
                    table_lines.append(line)

        if table_lines:
            results.append((sr, table_lines[-1]))

    s.close()

    print("\n\n+===================================================================================================================================+")
    print("|                                 ESP32-C6 LIVE SOURCE STREAMING PERFORMANCE BY SAMPLE RATE                                         |")
    print("+===================================================================================================================================+")
    print("| Target SR  | Last Captured 1 Hz Diagnostic Line                                                                                   |")
    print("+------------+----------------------------------------------------------------------------------------------------------------------+")
    for sr, line in results:
        print(f"| {sr:5d} Hz   | {line}")
    print("+===================================================================================================================================+\n")
    return 0

if __name__ == "__main__":
    exit(main())
