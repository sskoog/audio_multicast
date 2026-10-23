---
name: esp32-usb-serial-debug
description: >-
  Procedures, scripts, and commands for flashing (uploading firmware) and debugging ESP32/ESP32-C6/ESP32-S3 devices via USB Virtual Serial Port (USB Serial JTAG / UART) on Windows PowerShell using ESP-IDF v6.1. Includes port locking mitigation, lingering process termination, programmatic USB CDC resets, interactive test simulation, and timeout management.
---

# ESP32 USB Virtual Serial Flashing & Debugging Skill

This skill provides comprehensive operating procedures, automated PowerShell scripts, and reference commands for compiling, flashing, resetting, and monitoring ESP32 / ESP32-C6 / ESP32-S3 boards over USB Virtual Serial (Native USB Serial/JTAG or UART bridge) in Windows PowerShell.

---

## Hardware & COM Port Reference

Node IDs, MACs, COM ports and channel roles: see the Hardware Registry in `AGENTS.md` (single source of truth, not duplicated here). COM port number = node ID; nodes with a second port use node ID + 100 (e.g. Node 16: COM16 ROM bootloader, COM116 app console).

---

## 1. Environment Setup (ESP-IDF v6.1)

The app build/flash scripts activate ESP-IDF themselves. For manual `idf.py` / `esptool` use, dot-source the shared helper first (the default version is set in that file):

```powershell
. C:\Git_audio_multicast\tools\idf_env.ps1
# then e.g.: Reset-StaleIdfBuildDir <build_dir>; idf.py -B <build_dir> ... build
```

---

## 2. Serial Port Management & Pre-Flash Check

Before attempting to flash, ensure no background serial monitors or active Python scripts are locking the target COM port handle (`PermissionError(13, 'Access is denied.')`).

### A. List Active COM Ports with Descriptions
```powershell
python -c "import serial.tools.list_ports; print('\n'.join([p.device + ' - ' + p.description for p in serial.tools.list_ports.comports()]))"
```

### B. Terminate Lingering Serial Tasks & Locked Handles
On Windows, pySerial handles can get locked in lingering background processes. Clear them before flashing:

```powershell
Get-CimInstance Win32_Process -Filter "CommandLine LIKE '%device monitor%' OR CommandLine LIKE '%idf_monitor%' OR CommandLine LIKE '%test_audio_matrix%' OR CommandLine LIKE '%pc_audio_streamer%'" | ForEach-Object { Stop-Process -Id $_.ProcessId -Force }
```

### C. Handle USB Re-Enumeration Delay
When the onboard USB Serial JTAG / USB CDC controller resets during flashing, Windows unbinds and re-binds the virtual COM port.
- Allow **3 seconds** after reset before attempting to reopen serial connections.
- Always use `--before default_reset`, `--after hard_reset`, and `--connect-attempts 10` in `esptool.py`.

---

## 3. Firmware Flashing Procedure

### Method A: Automated Build and Flash (audio_VSAF_broadcast2)
```powershell
# Flash SOURCE (Node 16 / S3) - hands-free reset, see node16-flash-and-reset skill
powershell -ExecutionPolicy Bypass -File apps\audio_VSAF_broadcast2\tools\build_and_flash.ps1 -Role SOURCE

# Flash an S3 PCM5102A SINK (Node 4 Left / Node 5 Right)
powershell -ExecutionPolicy Bypass -File apps\audio_VSAF_broadcast2\tools\build_and_flash.ps1 -Role SINK -Port COM4 -NodeId 4

# Flash a C6 SINK (Node 23 / 24 / 20 / 25 / 26)
powershell -ExecutionPolicy Bypass -File apps\audio_VSAF_broadcast2\tools\build_and_flash.ps1 -Role SINK -Port COM23 -NodeId 23

# Re-flash existing binaries without rebuilding
powershell -ExecutionPolicy Bypass -File apps\audio_VSAF_broadcast2\tools\build_and_flash.ps1 -Role SINK -Port COM24 -NodeId 24 -OnlyFlash
```
For several nodes at once, use the `parallel-compile-and-flash` skill.

### Method B: Direct esptool (C6 SINK, after a build into build_c6)
```powershell
python -m esptool --chip esp32c6 -p COM23 -b 921600 --connect-attempts 10 --before default_reset --after hard_reset write_flash --flash_mode dio --flash_size 8MB --flash_freq 80m 0x0 apps\audio_VSAF_broadcast2\build_c6\bootloader\bootloader.bin 0x8000 apps\audio_VSAF_broadcast2\build_c6\partition_table\partition-table.bin 0x10000 apps\audio_VSAF_broadcast2\build_c6\audio_VSAF_broadcast2.bin
```
S3 SINKs (Node 4/5) use `build_s3` and `--chip esp32s3 --flash_size 4MB`. Node 16 must use `s3_flash_and_reset.py` (no physical reset line).

---

## 4. Programmatic Node Reset (USB CDC / Serial JTAG)

Native USB CDC ports on ESP32-C6 / ESP32-S3 disconnect and re-enumerate upon reset. To reset safely:
1. Toggle DTR / RTS lines.
2. Close the port handle immediately.
3. Wait 3 seconds for Windows to re-enumerate the COM port.

---

## 5. Reading Serial Logs & Telemetry

Open the port at 115200 baud with a safe timeout and ensure the port is closed cleanly in a `try...finally` block.

---

## 6. Helper Scripts (`scripts/` in this skill folder)

Run from the repo root, e.g. `powershell -ExecutionPolicy Bypass -File .claude\skills\esp32-usb-serial-debug\scripts\read_serial.ps1 -Port COM23 -DurationSeconds 5`.

| Script | Parameters | Purpose |
| :--- | :--- | :--- |
| `list_esp_ports.ps1` | none | List COM ports with description and HWID |
| `read_serial.ps1` | `-Port -Baud -DurationSeconds -TimeoutMs -MaxRetries` | Capture serial output for a bounded time |
| `send_serial_cmd.ps1` | `-Command -Port -Baud -ListenSeconds` | Send one console command (`help`, `stop`, `start`, `bootloader`, ...) and capture the reply |
| `reset_node.ps1` | `-Port -ReopenDelayMs` | DTR/RTS reset. Has no effect on Node 16; use `s3_flash_and_reset.py --only-reset` there |
| `flash_node.ps1` | - | Legacy ESP-IDF 5.2 flasher for `node2node` / `android2node` only. Do not use for VSAF2 |
