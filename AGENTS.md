# Audio Multicast System (ESP-NOW & LC3)

## Overview
This repository contains firmware applications and host utilities for low-latency digital audio multicast and multi-unicast systems using 802.11 Wi-Fi (ESP-NOW Layer 1 & 2) as the primary wireless transport protocol.
Audio compression utilizes the LC3 encoder and decoder via Google's `liblc3` library (hardware FPU on ESP32-S3) or ESP-IDF's fixed-point LC3 codec (for RISC-V ESP32-C6).
Legacy / exploratory experiments in Bluetooth Low Energy Audio (Auracast) are also archived within the repository.

---

## Hardware Registry & Node Topology

| Node ID | Board / Hardware | SoC Target | Flash / RAM | Factory MAC Address | Default COM Port(s) | Default Role / Function |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Node 1**  | Seeed Studio XIAO ESP32-S3 Plus + Wio-SX1262 B2B | ESP32-S3 (Xtensa LX7 + FPU) | 8 MB / 8 MB PSRAM | TBD | **COM1** | Audio SINK (PCM5102A DAC + TPA3118 Amp, GPIO 3 Mute) |
| **Node 2**  | Seeed Studio XIAO ESP32-S3 Plus + Wio-SX1262 B2B | ESP32-S3 (Xtensa LX7 + FPU) | 8 MB / 8 MB PSRAM | TBD | **COM2** | Audio SINK (PCM5102A DAC + TPA3118 Amp, GPIO 3 Mute) |
| **Node 3**  | Seeed Studio XIAO ESP32-S3 Plus + Wio-SX1262 B2B | ESP32-S3 (Xtensa LX7 + FPU) | 8 MB / 8 MB PSRAM | TBD | **COM3** | Audio SINK (PCM5102A DAC + TPA3118 Amp, GPIO 3 Mute) |
| **Node 4**  | Seeed Studio XIAO ESP32-S3 Plus + Wio-SX1262 B2B | ESP32-S3 (Xtensa LX7 + FPU) | 8 MB / 8 MB PSRAM | `E8:3D:C1:FB:E8:3C` | **COM4** | **Audio SINK (Ch 0: Left)** (PCM5102A DAC + TPA3118 Amp) |
| **Node 5**  | Seeed Studio XIAO ESP32-S3 Plus + Wio-SX1262 B2B | ESP32-S3 (Xtensa LX7 + FPU) | 8 MB / 8 MB PSRAM | `E8:3D:C1:FC:8B:50` | **COM5** | **Audio SINK (Ch 1: Right)** (PCM5102A DAC + TPA3118 Amp) |
| **Node 16** | Seeed Studio XIAO ESP32-S3 | ESP32-S3 (Xtensa LX7 + FPU) | 4 MB / 512 KB | `E0:72:A1:D8:4C:D0` | **COM16** (Boot) / **COM116** (App) | **Audio SOURCE** (Stereo/Mono LC3 Encoder + Broadcaster) |
| **Node 20** | Waveshare ESP32-C6-LCD-1.47 | ESP32-C6 (RISC-V) | 8 MB / 512 KB | `AC:EB:E6:23:DC:24` | **COM20** | Audio SINK / Subwoofer (Ch 5) (ST7789 LCD Console + WS2812B RGB) |
| **Node 21** | ESP32-C6-WROOM-1 DevKit | ESP32-C6 (RISC-V) | 8 MB / 512 KB | `98:A3:16:9D:57:EC` | **COM21** (Flash) & **COM121** (Bumble/App) | Audio SINK (Ch 2: Center) / USB Host Bridge |
| **Node 23** | Waveshare ESP32-C6-Zero | ESP32-C6 (RISC-V) | 8 MB / 512 KB | `B0:A6:04:99:38:44` | **COM23** | **Audio SINK Center (Ch 2)** (MAX98357A I2S DAC + WS2812B) |
| **Node 24** | Waveshare ESP32-C6-Zero | ESP32-C6 (RISC-V) | 8 MB / 512 KB | `B0:A6:04:99:18:E4` | **COM24** | **Audio SINK Subwoofer (Ch 5)** (MAX98357A I2S DAC + WS2812B) |
| **Node 25** | Heemol ESP32-C6 Mini | ESP32-C6 (RISC-V) | 8 MB / 512 KB | `E8:3D:C1:FB:DC:C4` | **COM25** (or COM10) | Audio SINK (Ch 3: Surround Left) / Test Node |
| **Node 26** | Heemol ESP32-C6 Mini | ESP32-C6 (RISC-V) | 8 MB / 512 KB | `98:A3:16:AC:13:38` | **COM26** (or COM22) | Audio SINK (Ch 4: Surround Right) / Test Node |

### Pinout Reference
- **Node 1, 2, 3, 4, 5 (SINK PCM5102A + TPA3118 Amp on XIAO S3 Plus)**:
  - BCK: D5 / GPIO 6
  - LCK (WS): D3 / GPIO 4
  - DIN (DOUT): D4 / GPIO 5
  - Power Amp Mute / SD: D2 / GPIO 3 (Active LOW Mute, High-Z Unmute)
  - User Status LED: GPIO 21 (Active LOW discrete LED)
  - BOOT Button: GPIO 0
  - Wio-SX1262 LoRa B2B Pins: GPIO 38..42, 7..9 (Reserved)
- **Node 23 & Node 24 (SINK DACs)**:
  - BCLK: GPIO 2
  - LRCLK (WS): GPIO 3
  - DIN (DOUT): GPIO 1
  - WS2812B Status LED: GPIO 8
  - BOOT Button: GPIO 9
- **Node 16 (SOURCE)**:
  - User Status LED: GPIO 21 (Active LOW discrete LED)
  - BOOT Button: GPIO 0

---

## Audio data payload

Both Bluetooth 5.3+ Low Energy Audio and ESP-NOW (802.11) are used as layer-2 connectionless, unreliable datagram protocols for audio streaming in this repository, depending on what app you choose to use. The LC3 audio encoder is used regardless of layer-2 solution.

### Bluetooth 5.3+ Low Energy Audio (to be implemented)
To be defined.

### 802.11 (ESP-NOW) Packet Structure (VSAF 3.0)
ESP-NOW audio packets use the VSAF 3.0 container format (strictly 248 bytes, 32-bit word aligned) with dual-frame (t0 + t-1) redundancy and round-robin SINK replies:
- **8-Byte Word-Aligned Header**:
  - `type_id` (2B): `0x1337` (Audio Broadcast), `0x1338` (Control), `0x1339` (SINK Telemetry Reply)
  - `packet_flags` (1B): Bit 0: Frame duration (0=7.5ms, 1=10ms), Bits 1-3: Sample rate, Bits 4-6: Target channel/receiver ID, Bit 7: Request for ACK/reply flag
  - `seq` (1B): Monotonically incrementing 8-bit sequence number (0-255)
  - `t_tx1_us` (4B): SOURCE microsecond master presentation timestamp (`esp_timer_get_time()`)
- **Payload (240 Bytes)**:
  - Primary Frame (Frame t0, 120 bytes, 32-bit word-aligned at offset 8)
  - Redundant Frame (Frame t-1, 120 bytes, 32-bit word-aligned at offset 128)

### Supported Audio Configurations
- **Sample Rates**: [8, 16, 24, 32, 48] kHz
- **Frame Cadence**: [7.5, 10] ms (10.0 ms Default)
- **Bitrates / Frame Sizes**:
  - 10.0 ms @ 120 octets = 96 kbps per channel
  - 7.5 ms @ 120 octets = 128 kbps per channel
- **Channel Modes**:
  - **Mono Mode**: Single LC3 encode; packet duplicated into two VSAF packets for Ch 0 and Ch 1.
  - **Stereo Mode**: Two distinct LC3 encodes; sent as independent VSAF packets for Ch 0 and Ch 1.

### Critical Timing Requirement
Audio broadcasting MUST use absolute microsecond hardware timer pacing (`esp_timer_get_time()`) instead of relative delays (`vTaskDelayUntil` / `vTaskDelay`) to eliminate clock drift and frame creep.

---

## Standard Build & Flash Commands (ESP-IDF v6.0.2)

Builds use ESP-IDF v6.0.2 at `C:\Users\stefa\OneDrive\Documents\ESP\v6.0.2\esp-idf`:

```powershell
# Environment Activation (PowerShell)
if (Test-Path "C:\Users\stefa\OneDrive\Documents\ESP\.esptools") {
    $env:IDF_TOOLS_PATH="C:\Users\stefa\OneDrive\Documents\ESP\.esptools"
} else {
    $env:IDF_TOOLS_PATH="C:\Users\stefa\.espressif"
}

if (Test-Path "$env:IDF_TOOLS_PATH\python_env\idf6.0_py3.13_env") {
    $env:IDF_PYTHON_ENV_PATH="$env:IDF_TOOLS_PATH\python_env\idf6.0_py3.13_env"
} else {
    $env:IDF_PYTHON_ENV_PATH="$env:IDF_TOOLS_PATH\python_env\idf6.0_py3.11_env"
}
$env:PATH="$env:IDF_PYTHON_ENV_PATH\Scripts;" + $env:PATH

if (Test-Path "C:\Users\stefa\OneDrive\Documents\ESP\v6.0.2\esp-idf\export.ps1") {
    . "C:\Users\stefa\OneDrive\Documents\ESP\v6.0.2\esp-idf\export.ps1"
} else {
    . "C:\Users\stefa\OneDrive\Documents\ESP\v6.0.2\export.ps1"
}

# Automated Build and Flash for audio_ESP_NOW_broadcast:
# Flash SOURCE (ESP32-S3 on COM16)
powershell -ExecutionPolicy Bypass -File apps\audio_ESP_NOW_broadcast\build_and_flash.ps1 -Role SOURCE -Port COM16

# Flash SINK Left (ESP32-C6 on COM23)
powershell -ExecutionPolicy Bypass -File apps\audio_ESP_NOW_broadcast\build_and_flash.ps1 -Role SINK -Port COM23 -NodeId 23

# Flash SINK Right (ESP32-C6 on COM24)
powershell -ExecutionPolicy Bypass -File apps\audio_ESP_NOW_broadcast\build_and_flash.ps1 -Role SINK -Port COM24 -NodeId 24
```
