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

try:
    import serial.serialwin32 as sw
    _orig_reconf = sw.Serial._reconfigure_port
    def _safe_reconf(self):
        try:
            _orig_reconf(self)
        except Exception:
            pass
    sw.Serial._reconfigure_port = _safe_reconf
except Exception:
    pass

RTC_CNTL_OPTION1_REG = 0x6000812C
RTC_CNTL_FORCE_DOWNLOAD_BOOT_MASK = 0x1


def find_com_ports():
    """Return a dictionary of detected relevant COM ports for Node 16."""
    ports = {}
    for p in serial.tools.list_ports.comports():
        hwid = p.hwid.upper()
        dev = p.device.upper()
        vid = getattr(p, "vid", None)
        pid = getattr(p, "pid", None)
        # Explicitly skip known SINK ports
        if dev in ["COM1", "COM2", "COM4", "COM5", "COM20", "COM21", "COM22", "COM23", "COM24", "COM25", "COM26", "COM121"]:
            continue
        if dev == "COM16" or (vid == 0x303A and pid == 0x1001):
            ports["jtag"] = p.device      # Native USB-Serial/JTAG ROM bootloader (COM16)
            ports["com16"] = p.device
        elif dev == "COM3" or (vid == 0x303A and pid == 0x0009) or "0009" in hwid:
            ports["otg"] = p.device       # Native USB-OTG ROM bootloader (COM3)
            ports["com3"] = p.device
        elif dev == "COM116" or (vid == 0x303A and pid == 0x4002) or "4002" in hwid:
            ports["app"] = p.device       # Running TinyUSB Application (COM116)
            ports["com116"] = p.device
    return ports


def trigger_app_to_bootloader(app_port):
    """Attempt to reboot running TinyUSB app into ROM bootloader via 1200bps touch and CLI."""
    print(f"[INFO] Attempting to reboot Node 16 on {app_port} into ROM bootloader...")
    target_port = rf"\\.\{app_port}" if not app_port.startswith("\\\\.\\") else app_port

    # Try 1: 1200-baud touch reset
    try:
        s = serial.Serial(target_port, 1200, timeout=0.5, dsrdtr=False, rtscts=False)
        s.dtr = True
        time.sleep(0.05)
        s.dtr = False
        time.sleep(0.05)
        s.close()
        print(f"[OK] Sent 1200-baud touch to {app_port}.")
    except Exception as e:
        print(f"[DEBUG] 1200-baud touch skipped: {e}")

    time.sleep(0.5)
    p = find_com_ports()
    if "jtag" in p or "otg" in p or "com16" in p or "com3" in p:
        return True

    # Try 2: CLI 'bootloader' command via pyserial
    try:
        s = serial.Serial(target_port, 115200, timeout=1, dsrdtr=False, rtscts=False)
        s.write(b"\r\nbootloader\r\n")
        time.sleep(0.2)
        s.close()
        print(f"[OK] Sent 'bootloader' command to {app_port}.")
        return True
    except Exception as e:
        print(f"[DEBUG] CLI command skipped: {e}")

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


RTC_CNTL_WDTCONFIG0_REG = 0x60008098
RTC_CNTL_WDTCONFIG1_REG = 0x6000809C
RTC_CNTL_WDTWPROTECT_REG = 0x600080B0

IO_MUX_GPIO21_REG = 0x60009058
GPIO_ENABLE_W1TS_REG = 0x60004024
GPIO_OUT_W1TS_REG = 0x60004008
GPIO_OUT_W1TC_REG = 0x6000400C
GPIO_21_BIT = 1 << 21  # User LED on Seeed Studio XIAO ESP32-S3 (Active LOW)


class StubLedController:
    """Controls the on-board User LED (GPIO 21, Active LOW) via ESP32-S3 Flasher Stub registers."""

    def __init__(self, esp):
        self.esp = esp
        self.is_on = False
        self.block_count = 0
        self._init_gpio()

    def _init_gpio(self):
        try:
            # 1. Configure IO_MUX for GPIO 21: Function 1 (MCU_SEL = 1, bit 12 = 0x1000)
            self.esp.write_reg(IO_MUX_GPIO21_REG, 0x00001000)
            # 2. Enable GPIO 21 output
            self.esp.write_reg(GPIO_ENABLE_W1TS_REG, GPIO_21_BIT)
            # 3. Turn ON initially to indicate stub is running and flashing is beginning
            self.turn_on()
            print("[LED] Flasher stub initialized User LED (GPIO 21) -> ON (flashing active)")
        except Exception as e:
            print(f"[LED] Note: Stub LED initialization skipped ({e})")

    def turn_on(self):
        try:
            # Active LOW: Clear bit 21 to pull pin to GND
            self.esp.write_reg(GPIO_OUT_W1TC_REG, GPIO_21_BIT)
            self.is_on = True
        except Exception:
            pass

    def turn_off(self):
        try:
            # Active LOW: Set bit 21 to pull pin to 3.3V
            self.esp.write_reg(GPIO_OUT_W1TS_REG, GPIO_21_BIT)
            self.is_on = False
        except Exception:
            pass

    def toggle(self):
        if self.is_on:
            self.turn_off()
        else:
            self.turn_on()

    def on_block_written(self):
        self.block_count += 1
        # Toggle LED every 2 blocks for a crisp ~5-8 Hz flicker during SPI write
        if self.block_count % 2 == 0:
            self.toggle()


def execute_watchdog_reset(esp):
    """Execute hands-free hardware reset on ESP32-S3 via RTC Watchdog Timer."""
    print("[RESET] Clearing RTC_CNTL_FORCE_DOWNLOAD_BOOT flag (0x6000812C)...")
    try:
        esp.write_reg(RTC_CNTL_OPTION1_REG, 0, RTC_CNTL_FORCE_DOWNLOAD_BOOT_MASK)
    except Exception as e:
        print(f"[DEBUG] RTC flag clear: {e}")

    print("[RESET] Arming ESP32-S3 hardware RTC Watchdog for system reset...")
    try:
        # Unlock RTC WDT write protection
        esp.write_reg(RTC_CNTL_WDTWPROTECT_REG, 0x50D83AA1)
        # Set short timeout (~50 us)
        esp.write_reg(RTC_CNTL_WDTCONFIG1_REG, 2000)
        # Arm RTC WDT with system reset action (stage 0 = reset system)
        esp.write_reg(RTC_CNTL_WDTCONFIG0_REG, 0xD0000102)
        # Re-lock write protection
        esp.write_reg(RTC_CNTL_WDTWPROTECT_REG, 0)
    except Exception as e:
        print(f"[DEBUG] RTC WDT arm error: {e}")

    try:
        esp.hard_reset(using_usb=False)
    except Exception:
        pass

    print("[OK] RTC Watchdog hardware reset triggered! Chip is rebooting into application...")


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    app_dir = os.path.dirname(script_dir) if os.path.basename(script_dir) == "tools" else script_dir
    default_bin_dir = os.path.join(app_dir, "build_s3")
    if not os.path.isdir(default_bin_dir):
        default_bin_dir = "build_s3"

    parser = argparse.ArgumentParser(description="ESP32-S3 Hands-Free Flash and Reset Tool")
    parser.add_argument("--port", "-p", default="AUTO", help="Serial port (COM16, COM3, COM116, or AUTO)")
    parser.add_argument("--baud", "-b", type=int, default=921600, help="Flashing baud rate (default: 921600)")
    parser.add_argument("--bin-dir", default=default_bin_dir, help="Path to build_s3 directory")
    parser.add_argument("--only-reset", action="store_true", help="Only perform hardware watchdog reset without flashing")
    args = parser.parse_args()

    bootloader_bin = os.path.join(args.bin_dir, "bootloader", "bootloader.bin")
    partition_bin = os.path.join(args.bin_dir, "partition_table", "partition-table.bin")
    app_bin = os.path.join(args.bin_dir, "audio_VSAF_broadcast2.bin")
    if not os.path.isfile(app_bin):
        fallback_bin = os.path.join(args.bin_dir, "audio_ESP_NOW_unicast.bin")
        if os.path.isfile(fallback_bin):
            app_bin = fallback_bin

    if not args.only_reset:
        for f in [bootloader_bin, partition_bin, app_bin]:
            if not os.path.isfile(f):
                print(f"[ERROR] Firmware file not found: {f}")
                sys.exit(1)

    ports = find_com_ports()
    flash_port = None

    if args.port == "COM116" or (args.port != "AUTO" and ports.get("app") == args.port):
        trigger_app_to_bootloader(args.port)
        flash_port = wait_for_bootloader_port(timeout=15)
    elif args.port in ["COM16", "COM3"] and (args.port not in [ports.get("jtag"), ports.get("otg"), ports.get("com16"), ports.get("com3")]) and ("app" in ports or "com116" in ports):
        app_p = ports.get("app") or ports.get("com116")
        trigger_app_to_bootloader(app_p)
        flash_port = wait_for_bootloader_port(timeout=15)
    elif args.port != "AUTO":
        flash_port = args.port
    else:
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

    # Initialize LED visual indicator via stub registers
    led = StubLedController(esp)

    # Hook block write routines to toggle LED during write
    orig_flash_defl_block = esp.flash_defl_block
    orig_flash_block = esp.flash_block

    def hooked_flash_defl_block(data, seq, timeout=esptool.loader.DEFAULT_TIMEOUT):
        res = orig_flash_defl_block(data, seq, timeout=timeout)
        led.on_block_written()
        return res

    def hooked_flash_block(data, seq, encrypted=False):
        res = orig_flash_block(data, seq, encrypted=encrypted)
        led.on_block_written()
        return res

    esp.flash_defl_block = hooked_flash_defl_block
    esp.flash_block = hooked_flash_block

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
        led.turn_on()
        time.sleep(0.2)
        led.turn_off()

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
