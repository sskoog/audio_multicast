#!/usr/bin/env python3
"""
run_benchmark.py
Connects to Node 21 on COM121, triggers the LC3 hardware benchmark suite,
and captures the full statistical results table.
"""

import time
import sys
import serial

PORT = "COM121"
BAUD = 115200

def main():
    print(f"Connecting to {PORT} at {BAUD} baud...")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1.0)
    except Exception as e:
        print(f"Error opening {PORT}: {e}")
        return 1

    time.sleep(1.5)
    ser.reset_input_buffer()

    print("Sending 'bench' command to Node 21...")
    ser.write(b"bench\r\n")
    ser.flush()

    start_time = time.time()
    table_lines = []
    capturing_table = False
    table_border_count = 0

    while time.time() - start_time < 100.0:
        line = ser.readline().decode("utf-8", errors="ignore")
        if line:
            print(line, end="", flush=True)
            if "ESP32-C6 HARDWARE LC3 ENCODER BENCHMARK RESULTS" in line:
                capturing_table = True
            if capturing_table:
                table_lines.append(line)
                if "+======================================================================================================================+" in line:
                    table_border_count += 1
                    if table_border_count >= 3:
                        print("\n[Benchmark Complete]")
                        break

    ser.close()

    if table_lines:
        with open("benchmark_results.txt", "w", encoding="utf-8") as f:
            f.writelines(table_lines)
        print("\nSaved benchmark table to benchmark_results.txt")
    return 0

if __name__ == "__main__":
    exit(main())
