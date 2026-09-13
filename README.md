# Audio Multicast: Low-Latency Multi-Speaker Wireless Audio

High-fidelity, ultra-low-latency wireless audio multicast and broadcast streaming over 802.11 Wi-Fi (ESP-NOW Layer 1 & 2) using the **Low Complexity Communication Codec (LC3)** across Espressif ESP32-S3 and ESP32-C6 microcontrollers.

---

## Architectural Evolution

This repository originated as an exploration into Bluetooth Low Energy (BLE 5.3+) Audio broadcasting and Auracast. However, due to hardware and firmware limitations in current ESP32 silicon (specifically the lack of hardware Isochronous Streams / Broadcast Isochronous Streams (BIG/BIS) controller support in ESP32-C6 and ESP32-S3), BLE Audio broadcasting proved impractical for multi-channel, production-grade low-latency hifi audio.

The project pivoted to **802.11 Layer 1 & 2 (ESP-NOW and raw Wi-Fi Action Frames)** as the primary transport protocol. Utilizing connectionless Wi-Fi datagrams paired with microsecond hardware timer pacing, the system achieves sub-10 ms wireless latency, microsecond multi-speaker time synchronization, and CD-quality transparent audio streaming.

---

## Repository Layout

```
.
├── apps/
│   ├── audio_ESP_NOW_broadcast/   # 1-to-Many broadcast over Wi-Fi Action Frames (VSAF)
│   ├── audio_ESP_NOW_unicast/     # 1-to-Many multi-unicast with ACK / retransmission
│   ├── win11audio/                # Windows 11 WASAPI audio capture & C++ DSP pipeline
│   ├── lc3_benchmark/             # Standalone cross-SoC LC3 performance benchmark
│   ├── usb_ble_bumble/            # [Legacy/Research] ESP32-C6 HCI + Google Bumble Auracast
│   ├── node2node/                 # [Legacy/Research] Early BLE broadcast prototype
│   └── android2node/              # [Legacy/Research] Early BLE sink prototype
├── components/                    # Shared ESP-IDF components (liblc3, drivers, etc.)
├── docs/                          # Technical specs, LC3 benchmarks, and hardware guides
├── images/                        # Hardware pinouts, board schematics, and photos
└── tools/                         # Cross-platform benchmark runners, plotters, and migration tools
```

---

## Primary Applications

### 1. [Audio ESP-NOW Broadcast (`apps/audio_ESP_NOW_broadcast`)](apps/audio_ESP_NOW_broadcast/README_audioESP-NOW.md)
* **Transport**: Connectionless 802.11 Action Frames using the Very Low Latency Synchronized Audio Frame (VSAF) container.
* **Topology**: 1 Audio SOURCE (e.g., Node 16 ESP32-S3) broadcasting to unlimited SINK nodes (Node 23 Left, Node 24 Right, Node 20 LCD).
* **Key Features**:
  * Microsecond presentation timestamps (modulo 2^24 us) for synchronized multi-speaker playback.
  * In-band redundancy (Dual-frame $N-1$ interleaving) providing zero-latency recovery from single-packet drops.
  * Packet Loss Concealment (PLC) via Google `liblc3`.
  * Real-time Windows audio streamer (`pc_audio_streamer.py`) supporting WASAPI loopback, MP3 playback, and 6-channel audio.

### 2. [Audio ESP-NOW Unicast (`apps/audio_ESP_NOW_unicast`)](apps/audio_ESP_NOW_unicast/)
* **Transport**: ESP-NOW peer-to-peer unicast with hardware MAC-layer ACKs and automatic retry pacing.
* **Topology**: 1 SOURCE streaming dedicated channel streams to individual SINK endpoints with independent sequence tracking.
* **Key Features**:
  * Guaranteed packet delivery for challenging RF environments.
  * Dedicated `tools/` folder containing automated multi-target build/flash tooling (`build_and_flash.ps1`) and PC streamer (`pc_unicast_streamer.py`).
  * Live runtime serial telemetry reporting RSSI, buffer depth, CPU load, and playback statistics.

### 3. [Windows 11 DSP & Audio Ingest (`apps/win11audio`)](apps/win11audio/README_win11audio.md)
* High-performance Windows 11 audio ingestion and processing engine.
* Native C++ DSP extension module (`dsp_engine.cpp`) with Python pybind11 bindings for Hilbert transform spatial matrixing, biquad IIR filtering, and low-latency audio capture.

---

## Hardware Registry & Node Topology

| Node ID | Board / Hardware | SoC Target | Flash / RAM | Default COM Port(s) | Default Role / Function |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Node 16** | Seeed Studio XIAO ESP32-S3 | ESP32-S3 (Xtensa LX7 + FPU) | 4 MB / 512 KB | **COM16** (Flash) & **COM116** (Telemetry) | **Audio SOURCE** (Stereo/Mono LC3 Encoder + Broadcaster) |
| **Node 20** | Waveshare ESP32-C6-LCD-1.47 | ESP32-C6 (RISC-V) | 8 MB / 512 KB | **COM20** | Audio SINK (ST7789 LCD Console + WS2812B RGB) |
| **Node 21** | ESP32-C6-WROOM-1 DevKit | ESP32-C6 (RISC-V) | 8 MB / 512 KB | **COM21** (Flash) & **COM121** (Bumble) | Audio SOURCE / USB Host Bridge |
| **Node 23** | Waveshare ESP32-C6-Zero | ESP32-C6 (RISC-V) | 8 MB / 512 KB | **COM23** | **Audio SINK Left (Ch 0)** (MAX98357A I2S DAC + WS2812B) |
| **Node 24** | Waveshare ESP32-C6-Zero | ESP32-C6 (RISC-V) | 8 MB / 512 KB | **COM24** | **Audio SINK Right (Ch 1)** (MAX98357A I2S DAC + WS2812B) |
| **Node 25** | Heemol ESP32-C6 Mini | ESP32-C6 (RISC-V) | 8 MB / 512 KB | **COM25** | Audio SINK / Test Node |
| **Node 26** | Heemol ESP32-C6 Mini | ESP32-C6 (RISC-V) | 8 MB / 512 KB | **COM26** | Audio SINK / Test Node |

### Pinout Reference
* **Node 23 & Node 24 (SINK DACs - MAX98357A)**:
  * BCLK: GPIO 2
  * LRCLK (WS): GPIO 3
  * DIN (DOUT): GPIO 1
  * WS2812B Status RGB LED: GPIO 8
  * BOOT Button: GPIO 9
* **Node 16 (SOURCE - XIAO ESP32-S3)**:
  * Status LED: GPIO 21 (Active LOW discrete LED)
  * BOOT Button: GPIO 0

---

## LC3 Audio Codec Profiles

The LC3 codec provides transparent audio compression across sample rates from 8 kHz to 48 kHz. In this system, encoding cadence is standard 7.5 ms (133.3 Hz) or 10.0 ms (100 Hz):

| Tier | Sampling Rate | Cadence | Frame Payload | Bitrate per Channel | Acoustic Bandwidth | Target Profile |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **VHQ** | 48 kHz | 10.0 ms | 160 bytes | 128 kbps | 20 Hz – 20+ kHz | Reference transparency; dense symphonic & transient fidelity |
| **HQ** | 48 kHz | 7.5 / 10.0 ms | 120 bytes | 128 / 96 kbps | 20 Hz – 20+ kHz | Near-transparent fidelity; matches 320 kbps MP3 |
| **MQ** | 32 kHz | 10.0 ms | 80 bytes | 64 kbps | 20 Hz – 16 kHz | High-efficiency music; comparable to 160–192 kbps MP3 |
| **LQ** | 24 kHz | 10.0 ms | 40 bytes | 32 kbps | 20 Hz – 12 kHz | Highly constrained RF streaming; preserves voice and core mix |

Full cross-SoC encoder and decoder benchmarks comparing Xtensa LX7 (ESP32-S3), RISC-V (ESP32-C6), and Xtensa LX6 (ESP32) are documented in [docs/lc3_encoder_cross_soc_benchmark.md](docs/lc3_encoder_cross_soc_benchmark.md).

---

## Build and Flash Setup

Builds use **ESP-IDF v6.0.2** with isolated target build folders (`build_s3/`, `build_c6/`):

```powershell
# Environment Activation (PowerShell)
$env:IDF_TOOLS_PATH = "C:\Users\stefa\.espressif"
. "C:\Users\stefa\OneDrive\Documents\ESP\v6.0.2\esp-idf\export.ps1"

# Flash Audio ESP-NOW Broadcast System:
# SOURCE (ESP32-S3 on COM16)
powershell -ExecutionPolicy Bypass -File apps\audio_ESP_NOW_broadcast\build_and_flash.ps1 -Role SOURCE -Port COM16

# SINK Left (ESP32-C6 on COM23)
powershell -ExecutionPolicy Bypass -File apps\audio_ESP_NOW_broadcast\build_and_flash.ps1 -Role SINK -Port COM23 -NodeId 23

# SINK Right (ESP32-C6 on COM24)
powershell -ExecutionPolicy Bypass -File apps\audio_ESP_NOW_broadcast\build_and_flash.ps1 -Role SINK -Port COM24 -NodeId 24
```

---

## License

This project is licensed under the **GNU Affero General Public License Version 3 (AGPL-3.0)**. See [LICENSE](LICENSE) for details.
