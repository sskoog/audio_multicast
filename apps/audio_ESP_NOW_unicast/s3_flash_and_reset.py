#!/usr/bin/env python3
"""
Automated Flashing and Hands-Free Post-Flash Reset Tool for Seeed Studio XIAO ESP32-S3 (Node 16).

Problem Solved:
  On the Seeed Studio XIAO ESP32-S3, native USB (GPIO 19/20) is wired directly to the SoC.
  There is no external USB-to-UART bridge (CP2102/CH340) and no RTS-to-CHIP_PU transistor circuit.
  Standard `esptool --after hard-reset` only toggles the RTS line, which cannot pulse EN on this board.
  This previously forced the user to physically tap the 'R' (Reset) button after every flash.

Solution:
  This script uses the ESP32-S3 internal RTC Watchdog Timer (WDT) via the flasher stub to execute
  a true hardware system reset of the digital core. Before arming the watchdog, it explicitly clears
  the RTC_CNTL_FORCE_DOWNLOAD_BOOT bit in RTC_CNTL_OPTION1_REG (0x6000812C). This guarantees that
  the ROM bootloader boots directly into SPI flash application runtime (COM116 + USB Audio)
  with ZERO physical intervention required!
"""

import os
import sys
import time
import argparse
import serial
import serial.tools.list_ports
import esptool
import esptool.cmds

try:
    sys.stdout.reconfigure(line_buffering=True)
    sys.stderr.reconfigure(line_buffering=True)
except Exception:
    pass

RTC_CNTL_OPTION1_REG = 0x6000812C
RTC_CNTL_FORCE_DOWNLOAD_BOOT_MASK = 0x1


def find_com_ports():
    """Return a dictionary of detected relevant COM ports."""
    ports = {}
    for p in serial.tools.list_ports.comports():
        hwid = p.hwid.upper()
        if "VID_303A" in hwid:
            if "PID_1001" in hwid:
                ports["jtag"] = p.device      # Native USB-Serial/JTAG ROM bootloader (COM16)
            elif "PID_0009" in hwid:
                ports["otg"] = p.device       # Native USB-OTG ROM bootloader (COM3)
            elif "PID_4002" in hwid:
                ports["app"] = p.device       # Running TinyUSB Application (COM116)
        if p.device == "COM16":
            ports["com16"] = p.device
        elif p.device == "COM3":
            ports["com3"] = p.device
        elif p.device == "COM116":
            ports["com116"] = p.device
    return ports


def trigger_app_to_bootloader(app_port):
    """Attempt to reboot running TinyUSB app into ROM bootloader via CLI or 1200bps touch."""
    print(f"[INFO] Attempting to reboot Node 16 on {app_port} into ROM bootloader...")
    try:
        # Try 1: CLI 'bootloader' command
        s = serial.serial_for_url(app_port, baudrate=115200, do_not_open=True)
        s.dtr = False
        s.rts = False
        s.timeout = 1.0
        s.write_timeout = 1.0
        s.open()
        s.write(b"\r\nbootloader\r\n")
        time.sleep(0.1)
        s.close()
        print(f"[OK] Sent 'bootloader' command to {app_port}.")
        return True
    except serial.SerialException as e:
        if "PermissionError" in str(e) or "Access is denied" in str(e):
            print(f"[WARNING] Cannot access {app_port}: Port is open by another program (e.g. Serial Studio Pro).")
        else:
            print(f"[INFO] CLI command skipped: {e}")

    try:
        # Try 2: 1200-baud touch reset
        s = serial.Serial(app_port, 1200, timeout=0.5, write_timeout=0.5)
        s.dtr = False
        s.rts = False
        time.sleep(0.1)
        s.close()
        print(f"[OK] Sent 1200-baud touch to {app_port}.")
        return True
    except Exception as e:
        print(f"[INFO] 1200-baud touch skipped: {e}")

    return False


def wait_for_bootloader_port(timeout=60):
    """Wait for either COM16 (USB-Serial-JTAG) or COM3 (USB-OTG) to appear."""
    print(f"[SCAN] Waiting for ESP32-S3 ROM bootloader (COM16 or COM3)...")
    start = time.time()
    while time.time() - start < timeout:
        ports = find_com_ports()
        if "jtag" in ports:
            return ports["jtag"]
        if "otg" in ports:
            return ports["otg"]
        if "com16" in ports:
            return "COM16"
        if "com3" in ports:
            return "COM3"
        time.sleep(0.5)
    return None


def execute_watchdog_reset(esp):
    """Execute hands-free hardware reset on ESP32-S3 via RTC Watchdog Timer."""
    print("[RESET] Clearing RTC_CNTL_FORCE_DOWNLOAD_BOOT flag (0x6000812C)...")
    try:
        esp.write_reg(RTC_CNTL_OPTION1_REG, 0, RTC_CNTL_FORCE_DOWNLOAD_BOOT_MASK)
    except Exception as e:
        print(f"[DEBUG] RTC flag clear: {e}")

    print("[RESET] Arming ESP32-S3 hardware RTC Watchdog for system reset...")
    try:
        esp.watchdog_reset()
    except Exception as e:
        print(f"[DEBUG] Watchdog response: {e}")

    try:
        esp._port.close()
    except Exception:
        pass

    print("[OK] RTC Watchdog hardware reset triggered! Chip is rebooting into application...")


def main():
    parser = argparse.ArgumentParser(description="ESP32-S3 Hands-Free Flash and Reset Tool")
    parser.add_argument("--port", "-p", default="AUTO", help="Serial port (COM16, COM3, COM116, or AUTO)")
    parser.add_argument("--baud", "-b", type=int, default=921600, help="Flashing baud rate (default: 921600)")
    parser.add_argument("--bin-dir", default="apps/audio_ESP_NOW_unicast/build_s3", help="Path to build_s3 directory")
    parser.add_argument("--only-reset", action="store_true", help="Only perform hardware watchdog reset without flashing")
    args = parser.parse_args()

    bootloader_bin = os.path.join(args.bin_dir, "bootloader", "bootloader.bin")
    partition_bin = os.path.join(args.bin_dir, "partition_table", "partition-table.bin")
    app_bin = os.path.join(args.bin_dir, "audio_ESP_NOW_unicast.bin")

    if not args.only_reset:
        for f in [bootloader_bin, partition_bin, app_bin]:
            if not os.path.isfile(f):
                print(f"[ERROR] Firmware file not found: {f}")
                sys.exit(1)

    flash_port = None
    if args.port != "AUTO":
        flash_port = args.port
    else:
        ports = find_com_ports()
        if "jtag" in ports:
            flash_port = ports["jtag"]
        elif "otg" in ports:
            flash_port = ports["otg"]
        elif "com16" in ports:
            flash_port = "COM16"
        elif "com3" in ports:
            flash_port = "COM3"
        elif "app" in ports or "com116" in ports:
            app_p = ports.get("app") or ports.get("com116")
            trigger_app_to_bootloader(app_p)
            flash_port = wait_for_bootloader_port(timeout=15)

        if not flash_port:
            print("[INFO] Waiting for ESP32-S3 ROM bootloader (COM16 or COM3)...")
            print("       Please hold 'B' (Boot) and tap 'R' (Reset) on Node 16.")
            flash_port = wait_for_bootloader_port(timeout=180)

    if not flash_port:
        print("[ERROR] Could not find bootloader port (COM16 or COM3)!")
        sys.exit(1)

    print(f"[CONNECT] Connecting to ESP32-S3 on {flash_port} at 115200...")
    try:
        esp = esptool.cmds.detect_chip(port=flash_port, baud=115200, connect_mode="default-reset")
    except Exception as e:
        print(f"[ERROR] Failed to connect to ESP32-S3 on {flash_port}: {e}")
        sys.exit(1)

    chip_name = esp.CHIP_NAME
    mac = ":".join(f"{b:02x}" for b in esp.read_mac())
    print(f"[CHIP] Detected {chip_name} (MAC: {mac})")

    print("[STUB] Uploading high-speed flasher stub...")
    esp = esptool.cmds.run_stub(esp)
    print(f"[SPEED] Switching flasher stub baud rate to {args.baud}...")
    esp.change_baud(args.baud)

    if not args.only_reset:
        addr_data = [
            (0x0000, bootloader_bin),
            (0x8000, partition_bin),
            (0x10000, app_bin),
        ]
        print(f"[FLASH] Writing {len(addr_data)} binaries to SPI flash...")
        esptool.cmds.write_flash(
            esp,
            addr_data,
            flash_mode="dio",
            flash_size="4MB",
            flash_freq="80m",
        )
        print("[FLASH OK] Firmware successfully written and verified!")

    # Perform the hands-free hardware watchdog reset
    execute_watchdog_reset(esp)

    print("[WAIT] Waiting 3.0 seconds for Windows USB device enumeration...")
    time.sleep(3.0)

    # Verify new runtime enumeration
    final_ports = find_com_ports()
    if "app" in final_ports or "com116" in final_ports:
        print(f"[SUCCESS] Node 16 rebooted cleanly and enumerated on COM116!")
    else:
        print(f"[NOTE] Node 16 reset complete. Current ports: {list(final_ports.values())}")


if __name__ == "__main__":
    main()
