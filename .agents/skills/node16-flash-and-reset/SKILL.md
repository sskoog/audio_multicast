---
name: node16-flash-and-reset
description: >-
  Procedure and automated tooling for hands-free flashing and autonomous hardware reset of Node 16 (Seeed Studio XIAO ESP32-S3 SOURCE) using the ESP32-S3 internal RTC Watchdog Timer (WDT) and RTC strapping registers, eliminating manual reset button presses after firmware upload.
---

# Node 16 (Seeed Studio XIAO ESP32-S3) Hands-Free Flash & Reset Skill

This skill provides the architectural explanation, register-level mechanics, automated scripts, and step-by-step operational workflows for flashing and autonomously resetting **Node 16 (Seeed Studio XIAO ESP32-S3 SOURCE)** without manual button intervention.

---

## 1. Problem Statement: Why Standard `esptool` Resets Fail

On traditional ESP32 development kits (e.g., NodeMCU, DevKitC), an external USB-to-UART bridge (CP2102, CH340, FTDI) is wired to the chip via a dual-transistor auto-reset circuit. Toggling `DTR` and `RTS` pulls the physical `EN` (`CHIP_PU`) pin low to reset the SoC.

On the **Seeed Studio XIAO ESP32-S3**:
1. **Direct Native USB**: The USB-C data lines connect directly to the ESP32-S3 silicon pins (GPIO 19 `D-` and GPIO 20 `D+`).
2. **No Hardware Reset Circuitry**: There is no external USB-to-UART bridge and no transistor circuit connected to `EN`.
3. **Virtual RTS Limitation**: When `esptool.py` executes `--after hard-reset` over native USB-Serial-JTAG (`COM16`), it toggles a virtual RTS register inside the USB peripheral. Because this register is not physically wired to the `EN` pin, the hardware power state is untouched, leaving the chip **stuck in the ROM download bootloader indefinitely**.
4. **Strapping Register Persistence**: When entering download mode via software (e.g., the CLI `bootloader` command or a 1200-baud touch reset), the firmware sets `RTC_CNTL_FORCE_DOWNLOAD_BOOT` in `RTC_CNTL_OPTION1_REG` (`0x6000812C`). Because RTC domain registers survive software resets, any reset that does not explicitly clear this bit causes the ROM to immediately re-enter download mode.

---

## 2. Hardware Registry & USB Topology Reference

| State / Function | Operating Mode | Windows Device Friendly Name | Hardware Instance ID (VID/PID) | Assigned Port |
| :--- | :--- | :--- | :--- | :--- |
| **Download / Flash** | Native USB-Serial/JTAG ROM Bootloader | `USB Serial Device (COM16)` | `USB\VID_303A&PID_1001` | **COM16** |
| **Download / Flash** | Native USB-OTG ROM Bootloader | `USB Serial Device (COM3)` | `USB\VID_303A&PID_0009` | **COM3** |
| **Application Runtime** | TinyUSB CDC ACM Console | `USB Serial Device (COM116)` | `USB\VID_303A&PID_4002&MI_00` | **COM116** |
| **Application Runtime** | TinyUSB UAC1 Stereo Speaker | `Node16 audio (USB Speaker)` | `USB\VID_303A&PID_4002&MI_02` | N/A (Audio Endpoint) |

---

## 3. Autonomous Post-Flash Reset Architecture

Instead of relying on nonexistent physical RTS-to-EN hardware, the flashing sequence utilizes the **ESP32-S3 Internal RTC Watchdog Timer (WDT)** via the running flasher stub:

1. **Upload Stub Flasher**: Establish communication with the ROM bootloader and upload the high-speed flasher stub (921,600 baud).
2. **Write & Verify Binaries**: Flash and verify `bootloader.bin` (at `0x0`), `partition-table.bin` (at `0x8000`), and `audio_ESP_NOW_unicast.bin` (at `0x10000`).
3. **Clear Boot Strap Flag**: Write `0` to Bit 0 of `RTC_CNTL_OPTION1_REG` (`0x6000812C`) to clear the `RTC_CNTL_FORCE_DOWNLOAD_BOOT` flag, ensuring the chip will not reboot back into download mode.
4. **Arm RTC Watchdog Timer**: Configure the RTC WDT registers directly via the stub:
   - Unlock: Write `0x50D83AA1` to `RTC_CNTL_WDTWPROTECT_REG` (`0x600080B0`).
   - Timeout: Write 2000 cycles (~50 us) to `RTC_CNTL_WDTCONFIG1_REG` (`0x6000809C`).
   - Arm: Write `0xD0000102` to `RTC_CNTL_WDTCONFIG0_REG` (`0x60008098`).
   - Lock: Write `0` to `RTC_CNTL_WDTWPROTECT_REG` (`0x600080B0`).
5. **Execute Hardware Reset**: The RTC Watchdog expires after ~50 us, resetting the CPU, digital core, memory buses, and USB PHY. The ROM reads strapping pins (GPIO 0 HIGH, download boot flag 0) and boots directly into the SPI flash application.
6. **Windows Enumeration**: Windows detects the rebooted device after ~3 seconds and registers:
   - Audio endpoint: `Node16 audio (USB Speaker)`
   - Serial CLI/telemetry: `USB Serial Device (COM116)`

---

## 4. Scripting Tools in Repository

### A. Python Flashing Engine (`apps/audio_ESP_NOW_unicast/tools/s3_flash_and_reset.py`)
Direct Python utility executing chip detection, stub upload, high-speed flash programming, and the atomic RTC watchdog reset sequence.

Key reset implementation:
```python
def execute_watchdog_reset(esp):
    # Clear force download boot bit so ROM does not re-enter download mode
    RTC_CNTL_OPTION1_REG = 0x6000812C
    RTC_CNTL_FORCE_DOWNLOAD_BOOT_MASK = 0x1
    try:
        esp.write_reg(RTC_CNTL_OPTION1_REG, 0, RTC_CNTL_FORCE_DOWNLOAD_BOOT_MASK)
    except Exception as e:
        pass

    # Arm RTC Watchdog for immediate digital core reset
    esp.hard_reset(using_usb=False)
```

### B. PowerShell Automation Wrapper (`apps/audio_ESP_NOW_unicast/tools/flash_s3.ps1`)
Handles lingering monitor process termination, activates the ESP-IDF v6.0.2 environment, and invokes `s3_flash_and_reset.py`:

```powershell
param(
    [string]$Port = "AUTO",
    [int]$Baud = 921600
)

$ErrorActionPreference = "Stop"

# Terminate lingering monitors holding COM handles
Get-CimInstance Win32_Process -Filter "CommandLine LIKE '%device monitor%' OR CommandLine LIKE '%idf_monitor%' OR CommandLine LIKE '%test_audio_matrix%' OR CommandLine LIKE '%pc_audio_streamer%'" | ForEach-Object { Stop-Process -Id $_.ProcessId -Force } -ErrorAction SilentlyContinue

# Environment Activation
$env:IDF_TOOLS_PATH = "C:\Users\stefa\.espressif"
. "C:\Users\stefa\OneDrive\Documents\ESP\v6.0.2\esp-idf\export.ps1"

$toolsDir = Split-Path -Parent $MyInvocation.MyCommand.Path
python -u "$toolsDir\s3_flash_and_reset.py" --port $Port --baud $Baud
```

---

## 5. Step-by-Step Flashing Procedure

### Step 1: Pre-Flash Clean Up
Close any active serial monitors (e.g., Serial Studio Pro, PuTTY, `idf.py monitor`) connected to `COM116` or `COM16` to prevent Windows `PermissionError: [WinError 5] Access is denied`.

### Step 2: Initiate Flashing Tool
Run the script from the repository root in PowerShell:
```powershell
powershell -ExecutionPolicy Bypass -File apps\audio_ESP_NOW_unicast\tools\flash_s3.ps1
```
Or use the unified builder/flasher:
```powershell
powershell -ExecutionPolicy Bypass -File apps\audio_ESP_NOW_unicast\tools\build_and_flash.ps1 -Role SOURCE
```

### Step 3: Enter Bootloader Mode (If in Application Mode)
If Node 16 is currently running on `COM116`, enter download mode by **holding the `B` (Boot) button and tapping the `R` (Reset) button** on the XIAO ESP32-S3.

The tool automatically detects `COM16`, switches to 921,600 baud, and flashes the binaries:
```text
[CONNECT] Connecting to ESP32-S3 on COM16 at 115200...
[CHIP] Detected ESP32-S3 (MAC: e0:72:a1:d8:4c:d0)
[STUB] Uploading high-speed flasher stub...
[SPEED] Switching flasher stub baud rate to 921600...
[FLASH] Writing 3 binaries to SPI flash...
Wrote 21056 bytes at 0x00000000... (100.0%)
Wrote 3072 bytes at 0x00008000... (100.0%)
Wrote 1091872 bytes at 0x00010000... (100.0%)
[FLASH OK] Firmware successfully written and verified!
```

### Step 4: Autonomous Hardware Reset (Hands-Free)
Immediately after verification, the script triggers the watchdog reset. **Do NOT press any buttons.**
```text
[RESET] Clearing RTC_CNTL_FORCE_DOWNLOAD_BOOT flag (0x6000812C)...
[RESET] Arming ESP32-S3 hardware RTC Watchdog for system reset...
Hard resetting with a watchdog...
[OK] RTC Watchdog hardware reset triggered! Chip is rebooting into application...
[WAIT] Waiting 3.0 seconds for Windows USB device enumeration...
[SUCCESS] Node 16 rebooted cleanly and enumerated on COM116!
```

### Step 5: Verify Device Enumeration in Windows
Run in PowerShell:
```powershell
Get-PnpDevice | Where-Object { $_.InstanceId -like '*PID_4002*' } | Select-Object Status, Problem, FriendlyName, InstanceId | Format-Table -AutoSize
```
Expected Output:
```text
Status Problem FriendlyName               InstanceId
------ ------- ------------               ----------
OK     CM_PROB_NONE USB Serial Device (COM116) USB\VID_303A&PID_4002&MI_00\...
OK     CM_PROB_NONE USB Speaker                USB\VID_303A&PID_4002&MI_02\...
OK     CM_PROB_NONE USB Composite Device       USB\VID_303A&PID_4002\...
```

---

## 6. Standalone Reset Command (Emergency / Debugging)

If Node 16 is stuck in the ROM bootloader on `COM16` or `COM3` and you want to boot into flash without re-flashing or touching the hardware buttons:

```powershell
python apps\audio_ESP_NOW_unicast\tools\s3_flash_and_reset.py --only-reset
```
This connects to the existing bootloader stub, clears the strapping register, and triggers the hardware watchdog reboot.
