# BLE Audio & ESP-NOW Audio Broadcast System

## Overview
This repository contains several apps for low-latency digital audio broadcasting systems using either Bluetooth Low Energy Audio, or ESP-NOW (802.11 layer 2) as the transport protocol.
Audio compression is in all cases utilizing the LC3 encoder and decoder, available from Google's liblc3 open library, or ESP-IDF's fixed-point implementation of LC3 encoder/decoder (for ESP32 SoCs without FPU support).

---

## Hardware Registry & Node Topology

| Node ID | Board / Hardware | SoC Target | Flash / RAM | Default COM Port(s) | Default Role / Function |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Node 16** | Seeed Studio XIAO ESP32-S3 | ESP32-S3 (Xtensa LX7 + FPU) | 4 MB / 512 KB | **COM16** | **Audio SOURCE** (Stereo/Mono LC3 Encoder + Broadcaster) |
| **Node 20** | Waveshare ESP32-C6-LCD-1.47 | ESP32-C6 (RISC-V) | 8 MB / 512 KB | **COM20** | Audio SINK (ST7789 LCD Console + WS2812B RGB) |
| **Node 21** | ESP32-C6-WROOM-1 DevKit | ESP32-C6 (RISC-V) | 8 MB / 512 KB | **COM21** (Flash) & **COM121** (Bumble) | Audio SOURCE / USB Host Bridge |
| **Node 23** | Waveshare ESP32-C6-Zero | ESP32-C6 (RISC-V) | 8 MB / 512 KB | **COM23** | **Audio SINK Left (Ch 0)** (MAX98357A I2S DAC + WS2812B) |
| **Node 24** | Waveshare ESP32-C6-Zero | ESP32-C6 (RISC-V) | 8 MB / 512 KB | **COM24** | **Audio SINK Right (Ch 1)** (MAX98357A I2S DAC + WS2812B) |
| **Node 25** | Heemol ESP32-C6 Mini | ESP32-C6 (RISC-V) | 8 MB / 512 KB | **COM25** | Audio SINK / Test Node |
| **Node 26** | Heemol ESP32-C6 Mini | ESP32-C6 (RISC-V) | 8 MB / 512 KB | **COM26** | Audio SINK / Test Node |

### Pinout Reference
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

### 802.11 (ESP-NOW) Packet Structure 
ESP-NOW audio packets use the VSAF container format, condaining maximum 250 bytes. A minimal 8-byte header is used for meta-data.  
- **8-Byte Header**:
  - `magic` (2B): `0x1337`
  - `seq` (2B): Monotonically incrementing 16-bit sequence number
  - `cfg` (1B): Bitfield (Bit 0: Redundancy present, Bit 1: Stereo mode, Bits 2-3: Channel index, Bits 4-7: Reserved)
  - `pts_us` (3B): Presentation timestamp in microseconds modulo 2^24
- **Payload**:
  - Primary Frame (Frame N, e.g., 120 bytes)
  - Redundant Frame (Frame N-1, e.g., 120 bytes)

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
