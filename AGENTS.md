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

### Software Post-Gain Calibration (PCM5102A Nodes)
Per-node software attenuation compensates for hardware gain and driver impedance differences across identically-specced PCM5102A + TPA3118 amplifier boards. Values are applied at runtime via MAC-address lookup in `main.cpp`.

| Node ID | Channel | Software Post-Gain | Notes |
| :--- | :--- | :--- | :--- |
| Node 4 | Ch 0 (Left) | -20.0 dB | Baseline reference |
| Node 5 | Ch 1 (Right) | -9.0 dB | +11 dB vs baseline (quieter hardware) |
| Node 1, 2, 3 | TBD | -20.0 dB (default) | Not yet calibrated |

---

## Audio data payload

Both Bluetooth 5.3+ Low Energy Audio and ESP-NOW (802.11) are used as layer-2 connectionless, unreliable datagram protocols for audio streaming in this repository, depending on what app you choose to use. The LC3 audio encoder is used regardless of layer-2 solution.

### Bluetooth 5.3+ Low Energy Audio (to be implemented)
To be defined.

### 802.11 (ESP-NOW) Packet Structure (VSAF 3.0, `apps/audio_VSAF`)
Source of truth: `apps/audio_VSAF/main/vsaf_protocol.hpp`. Audio packets are strictly 248 bytes, 32-bit word aligned, little-endian, with multi-tier Unequal Error Protection (UEP) redundancy and round-robin SINK replies. Full spec: `apps/audio_VSAF/README_audio_VSAF.md`.
- **Packet types (`type_id`)**: `0x1337` Satellite audio (Ch 0-4), `0x1338` Subwoofer audio (Ch 5), `0x1350` Control (from SOURCE), `0x1360` SINK telemetry reply (16 bytes).
- **8-Byte Word-Aligned Header** (both audio types):
  - `type_id` (2B)
  - `packet_flags` (1B): Bit 0: Frame duration (0=7.5 ms deprecated, 1=10 ms), Bits 1-3: Sample rate code (0=8k, 1=16k, 2=24k, 3=32k, 4=48k), Bits 4-6: Receiver channel ID (0-5, 7=broadcast), Bit 7: REQ_ACK (this SINK must reply with telemetry)
  - `seq` (1B): Monotonically incrementing 8-bit sequence number (0-255)
  - `t_tx1_us` (4B): SOURCE microsecond master presentation timestamp (`esp_timer_get_time()`)
- **Satellite payload (`0x1337`, 240 Bytes, 3-tier)**: t0 120 B (HQ, offset 8) + t-1 60 B (redundancy, offset 128) + t-2 60 B (redundancy, offset 188).
- **Subwoofer payload (`0x1338`, 240 Bytes, 4-tier)**: t0 / t-1 / t-2 / t-3, 60 B each @ 8 kHz (offsets 8 / 68 / 128 / 188).

### Supported Audio Configurations
- **Satellites (Ch 0-4)**: 48 kHz default (`CONFIG_ESPNOW_SAMPLE_RATE_HZ`; the SOURCE console `sr <8k|16k|24k|32k|48k>` switches it at runtime), 10 ms frames; HQ 120 octets = 96 kbps, redundancy 60 octets = 48 kbps. LR4 high-pass at 100 Hz.
- **Subwoofer (Ch 5)**: 8 kHz (`CONFIG_ESPNOW_SUB_SAMPLE_RATE_HZ`), 60 octets = 48 kbps, LR4 low-pass at 100 Hz with polyphase decimation.
- **Archived apps** (`archive/audio_ESP_NOW_broadcast` / `_unicast`) use the older dual-frame (t0 + t-1, 120 B each) layout with runtime-selectable 8-48 kHz, 7.5/10 ms and mono/stereo modes.

### Critical Timing Requirement
Audio broadcasting MUST use absolute microsecond hardware timer pacing (`esp_timer_get_time()`) instead of relative delays (`vTaskDelayUntil` / `vTaskDelay`) to eliminate clock drift and frame creep.

---

## Standard Build & Flash Commands (ESP-IDF v6.1)

Builds use ESP-IDF v6.1 at `C:\Users\stefa\OneDrive\Documents\ESP\v6.1\esp-idf` with tools in `C:\Users\stefa\.espressif` (GCC 15.2.0, Python env `idf6.1_py3.13_env`).
All build/flash scripts activate the environment through one shared helper, `tools\idf_env.ps1`. To change the IDF version for the whole repo, edit `$DefaultIdfVersion` there. For a single run, pass `-IdfVersion 6.0.2` to a build script or set `$env:AUDIO_MC_IDF_VERSION`. Build folders configured by a different IDF version are cleaned with `idf.py fullclean` automatically.

```powershell
# Environment Activation (PowerShell, manual idf.py / esptool use)
. C:\Git_audio_multicast\tools\idf_env.ps1

# Automated Build and Flash for audio_VSAF (main track):
# Flash SOURCE (Node 16, ESP32-S3, hands-free reset)
powershell -ExecutionPolicy Bypass -File apps\audio_VSAF\tools\build_and_flash.ps1 -Role SOURCE

# Flash SINK Left (Node 4, ESP32-S3 PCM5102A on COM4)
powershell -ExecutionPolicy Bypass -File apps\audio_VSAF\tools\build_and_flash.ps1 -Role SINK -Port COM4 -NodeId 4

# Flash SINK Center (Node 23, ESP32-C6 on COM23)
powershell -ExecutionPolicy Bypass -File apps\audio_VSAF\tools\build_and_flash.ps1 -Role SINK -Port COM23 -NodeId 23

# Build both targets in parallel and flash all detected nodes
powershell -ExecutionPolicy Bypass -File apps\audio_VSAF\tools\parallel_build_and_flash.ps1
```
