# Audio Multicast: Low-Latency Multi-Speaker Wireless Audio

High-fidelity, ultra-low-latency wireless audio multicast and broadcast streaming over 802.11 Wi-Fi (ESP-NOW Layer 1 & 2) using the **Low Complexity Communication Codec (LC3)** across Espressif ESP32-S3 and ESP32-C6 microcontrollers.

---

## Architectural Evolution

This repository originated as an exploration into Bluetooth Low Energy (BLE 5.3+) Audio broadcasting and Auracast. However, due to hardware and firmware limitations in current ESP32 silicon (specifically the lack of hardware Isochronous Streams / Broadcast Isochronous Streams (BIG/BIS) controller support in ESP32-C6 and ESP32-S3), BLE Audio broadcasting proved impractical for multi-channel, production-grade low-latency hifi audio.

The project pivoted to **802.11 Layer 1 & 2 (ESP-NOW and raw Wi-Fi Action Frames)** as the primary transport protocol. Utilizing connectionless Wi-Fi datagrams paired with microsecond hardware timer pacing, the system achieves sub-10 ms wireless latency, microsecond multi-speaker time synchronization, and CD-quality transparent audio streaming.

---

## Project Goals

Targets (latency, inter-speaker sync, reliability, scale), priorities and non-goals are in **[GOALS.md](GOALS.md)**.

---

## Repository Layout

```
.
├── apps/
│   ├── audio_VSAF/     # MAIN TRACK: VSAF 3.0 multi-channel broadcast (ESP32-S3 SOURCE, S3/C6 SINKs)
│   ├── win11audio/                # Windows 11 WASAPI audio capture & C++ DSP pipeline
│   └── lc3_benchmark/             # Standalone cross-SoC LC3 performance benchmark
├── archive/                       # Retired apps (old ESP-NOW broadcast/unicast, BLE/Auracast experiments), unmaintained
├── components/                    # Shared ESP-IDF components (liblc3, esp-dsp, drivers, etc.)
├── docs/                          # Technical specs, LC3 benchmarks, and hardware guides
├── images/                        # Hardware pinouts, board schematics, and photos
└── tools/                         # idf_env.ps1 (ESP-IDF activation), pc_lc3/ (PC LC3 encoder), benchmark runners, plotters
```

---

## Primary Applications

### 1. [Audio VSAF Broadcast 2 (`apps/audio_VSAF`)](apps/audio_VSAF/README_audio_VSAF.md)
* **Transport**: Connectionless 802.11 broadcast (ESP-NOW) using the VSAF 3.0 container (248-byte packets).
* **Topology**: 1 SOURCE (Node 16, ESP32-S3, UAC1 USB speaker input) broadcasting up to 6 channels (5 satellites + subwoofer) to S3 (PCM5102A) and C6 (MAX98357A) SINK nodes.
* **Key Features**:
  * Multi-tier unequal error protection: each packet carries t0 plus 2 (satellites) or 3 (subwoofer) older frames for burst-loss recovery, then LC3 PLC.
  * Hardware-timer paced SOURCE, Wi-Fi TX-done ISR slot timing, microsecond presentation timestamps and dual-way PTP-style time sync.
  * Round-robin SINK telemetry (one reply per 10 ms window) and 1 s fixed-width heartbeat telemetry on every node.

### 2. [Windows 11 DSP & Audio Ingest (`apps/win11audio`)](apps/win11audio/README_win11audio.md)
* High-performance Windows 11 audio ingestion and processing engine.
* Native C++ DSP extension module (`dsp_engine.cpp`) with Python pybind11 bindings for Hilbert transform spatial matrixing, biquad IIR filtering, and low-latency audio capture.

### 3. [LC3 Benchmark (`apps/lc3_benchmark`)](apps/lc3_benchmark/)
* On-device LC3 encode/decode timing across ESP32-S3, ESP32-C6 and ESP32-WROOM-32.

Retired apps are described in [archive/README.md](archive/README.md).

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

Builds use **ESP-IDF v6.1** with isolated target build folders (`build_s3/`, `build_c6/`). All scripts activate ESP-IDF via the shared helper `tools\idf_env.ps1`, where the repo-wide version is set:

```powershell
# Environment Activation (PowerShell, only needed for manual idf.py use)
. .\tools\idf_env.ps1

# Main track: audio_VSAF
powershell -ExecutionPolicy Bypass -File apps\audio_VSAF\tools\build_and_flash.ps1 -Role SOURCE                      # Node 16
powershell -ExecutionPolicy Bypass -File apps\audio_VSAF\tools\build_and_flash.ps1 -Role SINK -Port COM4 -NodeId 4   # Left
powershell -ExecutionPolicy Bypass -File apps\audio_VSAF\tools\parallel_build_and_flash.ps1                         # whole cluster
```

---

## License

This project is licensed under the **GNU Affero General Public License Version 3 (AGPL-3.0)**. See [LICENSE](LICENSE) for details.
