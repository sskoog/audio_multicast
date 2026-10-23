#!/usr/bin/env python3
"""
flash_s3_node.py
Automates the flash procedure for Seeed Studio XIAO ESP32-S3:
1. Detects device on COM16 / COM3.
2. If in CDC mode (2886:0059), triggers 1200 baud touch to enter ROM bootloader.
3. Detects the bootloader port (e.g. COM3 / 303A:1001).
4. Flashes bootloader, partitions, app binary, and audio clips storage partition.
"""

import sys
import time
import subprocess
import serial
import serial.tools.list_ports

ESPTOOL_PATH = r"C:\Users\stefa\.espressif\python_env\idf6.1_py3.13_env\Scripts\esptool.exe"
BOOTLOADER = r"apps\lc3_benchmark\build_s3\bootloader\bootloader.bin"
PARTITIONS = r"apps\lc3_benchmark\build_s3\partition_table\partition-table.bin"
APP_BIN    = r"apps\lc3_benchmark\build_s3\esp32_lc3_benchmark.bin"
AUDIO_DATA = r"data\benchmark_clips_4mb.bin"

def find_s3_port():
    for p in serial.tools.list_ports.comports():
        # Check for Seeed XIAO (2886:0059) or Espressif S3 (303A:1001 with S3 serial)
        if "2886:0059" in p.hwid or "E0:72:A1" in p.hwid or "E072A1" in p.hwid:
            return p.device, ("2886:0059" in p.hwid)
        if p.device in ["COM16", "COM3"]:
            return p.device, False
    return None, False

def trigger_bootloader(port):
    print(f"Triggering 1200 baud touch on {port}...")
    try:
        # Try direct opening
        s = serial.Serial(port, 1200)
        s.dtr = False
        s.rts = True
        time.sleep(0.1)
        s.close()
    except Exception as e:
        print(f"Direct open failed ({e}), trying device interface path...")
        try:
            path = r"\\?\USB#VID_2886&PID_0059&MI_00#8&64E961D&0&0000#{86e0d1e0-8089-11d0-9ce4-08003e301f73}"
            s = serial.Serial(path, 1200)
            s.dtr = False
            s.rts = True
            time.sleep(0.1)
            s.close()
        except Exception as e2:
            print(f"Device interface open failed: {e2}")

def wait_for_bootloader_port(timeout=10.0):
    start = time.time()
    while time.time() - start < timeout:
        for p in serial.tools.list_ports.comports():
            if "303A:1001" in p.hwid and ("E0:72:A1" in p.hwid or "E072A1" in p.hwid):
                print(f"Found ESP32-S3 ROM bootloader port: {p.device}")
                return p.device
            if p.device == "COM3":
                print(f"Found COM3!")
                return "COM3"
        time.sleep(0.5)
    return None

def main():
    port, is_cdc = find_s3_port()
    print(f"Detected S3 on: {port} (is_cdc={is_cdc})")

    if is_cdc or port == "COM16":
        trigger_bootloader(port)
        time.sleep(1.5)
        flash_port = wait_for_bootloader_port()
        if not flash_port:
            flash_port = "COM3"
    else:
        flash_port = port if port else "COM3"

    print(f"\nFlashing ESP32-S3 on port {flash_port}...")
    cmd = [
        ESPTOOL_PATH,
        "--chip", "esp32s3",
        "-p", flash_port,
        "-b", "460800",
        "--before", "default_reset",
        "--after", "hard_reset",
        "write_flash",
        "--flash_mode", "dio",
        "--flash_size", "4MB",
        "--flash_freq", "80m",
        "0x0", BOOTLOADER,
        "0x8000", PARTITIONS,
        "0x10000", APP_BIN,
        "0x190000", AUDIO_DATA
    ]
    print("Executing:", " ".join(cmd))
    res = subprocess.run(cmd)
    return res.returncode

if __name__ == "__main__":
    exit(main())
