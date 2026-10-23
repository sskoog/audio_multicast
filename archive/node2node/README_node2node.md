# Node2Node: BLE 5.3 Auracast Audio Broadcast & Distributed Volume Control

![License: AGPL-3.0](https://img.shields.io/badge/License-AGPL--3.0-blue.svg)
![Target: ESP32-C6](https://img.shields.io/badge/Platform-ESP32--C6-orange.svg)
![ESP-IDF: v6.0.2 / v6.0](https://img.shields.io/badge/ESP--IDF-v6.0.2%20%7C%20v6.0-red.svg)
![Bluetooth: BLE 5.3 LE Audio](https://img.shields.io/badge/Bluetooth-5.3%20LE%20Audio-blue.svg)
![Codec: Espressif Fixed-Point LC3](https://img.shields.io/badge/Codec-Espressif%20LC3%20(Fixp)-green.svg)

Production-grade, low-latency multi-node Bluetooth Low Energy (BLE 5.3) Audio Broadcasting mesh and SINK receiver network built on **ESP-IDF v6.0.2 / v6.0** for the **Espressif ESP32-C6** (Single-Core 32-bit RISC-V @ 160 MHz).

---

## 1. System Architecture Overview

The `node2node` ecosystem implements an **Auracast Broadcast Audio** topology combined with standard Bluetooth LE Audio GATT profiles for distributed telemetry and volume control:

```
                                  +------------------------------------------------------+
                                  |                     AUDIO SOURCE                     |
                                  |               Node21 (ESP32-C6 DevKit)               |
                                  +------------------------------------------------------+
                                            |                                |
                Extended Advertising Train  |                                |  GATT Volume Control Service
                   (PAwR / PBA / BASE)      |                                |   (VCS 0x1844 Absolute Vol)
                   48 kHz / 64 kbps LC3     |                                |
                                            v                                v
    +-----------------------------------------------+   +-----------------------------------------------+
    |             AUDIO SINK 1 (Node23)             |   |             AUDIO SINK 2 (Node24)             |
    |       Waveshare ESP32-C6-Zero + MAX98357A     |   |       Waveshare ESP32-C6-Zero + MAX98357A     |
    |        (48 kHz Mono 16-bit, 12 dB Gain)       |   |        (48 kHz Mono 16-bit, 12 dB Gain)       |
    +-----------------------------------------------+   +-----------------------------------------------+
```

### Core Architectural Features:
* **Audio Pipeline**: Real-time **48.0 kHz 16-bit Mono** stream encoded and decoded with Espressif's native **Fixed-Point LC3** (`esp_audio_codec`) at 64 kbps (80 octets per 10 ms frame).
* **Zero Float Emulation Overhead**: Pure 32-bit integer arithmetic (Q15/Q31) tailored for the single-core ESP32-C6 RISC-V processor without hardware FPU, delivering **100.0 Hz continuous playback** at low CPU load.
* **Continuous I2S DMA Engine**: Dual-slot Philips standard stereo output with 8 DMA descriptors x 480 frames depth (80 ms total buffer margin), completely eliminating buffer underruns and harmonic distortion.
* **Universal Auto-Lock Discovery**: Passive scanner (`passive = 1`) locking onto any Auracast broadcast train on both **1M and LE Coded PHYs** (`0x1851` BASE / `0x1852` PBA).
* **Multi-Node Volume Control**: Full compliance with the Bluetooth Volume Control Service (VCS `0x1844`) and Broadcast Audio Scan Service (BASS `0x184F`).
* **Visual Status Feedback**: Onboard WS2812 RGB LED (`GPIO 8`) providing instant feedback for Scanning (Soft Green pulse), Streaming (Fast Cyan/Teal pulse), and Sync Loss (Red strobe).

---

## 2. Hardware Topology & Node Mapping

| Node Identifier | Serial Port | Hardware Board & Modules | Active Role | Description |
| :--- | :--- | :--- | :--- | :--- |
| **`Node20`** | `COM20` | Waveshare ESP32-C6-LCD (1.47" ST7789) | **SINK** (Receiver) | Visual LCD SINK with real-time waveform display & volume gauges. |
| **`Node21`** | `COM21` / `COM121` | ESP32-C6-WROOM-1 DevKit (COM21: Flash & Telemetry, COM121: Bumble & Audio Ingest) | **SOURCE** (Broadcaster / Bumble) | Hardware Tone Generator, LFO volume oscillator, and Python Bumble HCI broadcaster. |
| **`Node23`** | `COM23` | Waveshare ESP32-C6-Zero + MAX98357A DAC | **SINK** (Receiver) | Dedicated audio playback node with speaker & 12 dB hardware gain. |
| **`Node24`** | `COM24` | Waveshare ESP32-C6-Zero + MAX98357A DAC | **SINK** (Receiver) | Multi-speaker secondary audio playback node. |

---

## 3. Hardware Pinout & Wiring

### Waveshare ESP32-C6-Zero to MAX98357A I2S DAC Module

On the **Waveshare ESP32-C6-Zero**, all required power, control, and audio pins are arranged **consecutively on the Left 9-Pin Header**, enabling direct, clean 1-to-1 jumper wiring:

```
+------------------------------------+              +------------------------------------+
|      Waveshare ESP32-C6-Zero       |              |      MAX98357A I2S DAC Module      |
|       (Left 9-Pin Header)          |              |          (7-Pin Breakout)          |
|                                    |              |                                    |
|   Pin 1 [ 5V / VBUS ] -------------|------------> | [ VIN ]       (5V Power Supply)    |
|   Pin 2 [ GND ]       -------------|------------> | [ GND ]       (Common Ground)      |
|   Pin 3 [ 3V3 ]       -------------|------------> | [ SD_MODE ]   (High = Left Channel)|
|   Pin 4 [ GP0 ]       -------------|------------> | [ GAIN ]      (High = 12 dB Gain)  |
|   Pin 5 [ GP1 ]       -------------|------------> | [ BCLK ]      (Bit Clock)          |
|   Pin 6 [ GP2 ]       -------------|------------> | [ LRC / WS ]  (Word Select Clock)  |
|   Pin 7 [ GP3 ]       -------------|------------> | [ DIN ]       (Serial Audio Data)  |
|                                    |              |                                    |
|   Pin 8 [ GP8 ] (Onboard WS2812)   |              |   [ SPK+ / SPK- ] -> 4-8 Ohm Spkr  |
+------------------------------------+              +------------------------------------+
```

### Detailed Signal Pin Mapping:

| Signal | ESP32-C6-Zero Pin | MAX98357A Pin | Purpose / Operating Mode |
| :--- | :--- | :--- | :--- |
| **Power (5V)** | **Left Header Pin 1 (`5V`)** | `VIN` | 5V rail for maximum Class-D speaker output power (up to 3.2W into 4Ω). |
| **Ground** | **Left Header Pin 2 (`GND`)** | `GND` | Common ground reference. |
| **Channel Select** | **Left Header Pin 3 (`3V3`)** | `SD_MODE` | Tied to 3.3V to cleanly latch the internal comparator to **Left Channel** playback. |
| **Hardware Gain** | **Left Header Pin 4 (`GP0`)** | `GAIN` | Driven **HIGH (3.3V)** on boot for **12 dB maximum gain** (Floating = 9 dB, LOW = 6 dB). |
| **I2S Bit Clock** | **Left Header Pin 5 (`GP1`)** | `BCLK` | Continuous bit clock ($F_s 	imes 2 	imes 16 = 1.536	ext{ MHz}$ @ 48 kHz). |
| **I2S Word Select** | **Left Header Pin 6 (`GP2`)** | `LRC / WS` | Frame sync clock running at **48.0 kHz** (Low = Left, High = Right). |
| **I2S Serial Data** | **Left Header Pin 7 (`GP3`)** | `DIN` | 16-bit interleaved stereo PCM audio data from DMA transmitter. |
| **Status LED** | **Onboard (`GPIO 8`)** | N/A | Onboard WS2812B RGB status indicator with GRB color order compensation. |

---

## 4. Audio Engine: Espressif Fixed-Point LC3 Codec

The project utilizes Espressif's official **`esp_audio_codec`** component (`espressif/esp_audio_codec`):

```
+---------------------------------------------------------------------------------+
|                              AUDIO PROCESSING PIPELINE                          |
|                                                                                 |
|  [ 48 kHz SINK Audio Loop ]                                                     |
|            |                                                                    |
|            v                                                                    |
|  [ esp_lc3_dec_decode() ] --------> Pure 32-bit Integer DSP (Q15/Q31)           |
|            |                        No Software Float Emulation                 |
|            v                                                                    |
|  [ VCS Volume Modulation ] -------> SINK Absolute Digital Gain Scaling (0-255)  |
|            |                                                                    |
|            v                                                                    |
|  [ Dual-Slot Interleaving ] ------> Duplicate Mono PCM into Left + Right Slots  |
|            |                                                                    |
|            v                                                                    |
|  [ i2s_channel_write() ] ---------> 8 Descriptors x 480 Frame DMA Ping-Pong    |
|            |                        Paced synchronously at 100.00 Hz            |
|            v                                                                    |
|  [ MAX98357A Class-D DAC ] -------> 4-8 Ohm Speaker (12 dB Crisp Audio)         |
+---------------------------------------------------------------------------------+
```

### Codec Operating Specifications:
* **Sampling Frequency ($F_s$)**: `48000 Hz`
* **Frame Duration**: `10000 µs (10 ms / 100 dms)`
* **Compressed Frame Budget**: `80 octets / frame`
* **Bitrate**: `64.0 kbps`
* **Samples per Frame**: `480 samples`
* **Packet Loss Concealment (PLC)**: Enabled natively in `esp_lc3_dec_cfg_t`

---

## 5. Build, Flash, and Monitor Instructions

### Environment Setup (ESP-IDF v6.0.2 in PowerShell)
```powershell
. "C:\Users\stefa\OneDrive\Documents\ESP\v6.0.2\esp-idf\export.ps1"
```

### 1. Build and Flash Audio SINK (Node23 on COM23)
```powershell
Set-Location apps\node2node
idf.py build
python -m esptool --chip esp32c6 -p COM23 -b 460800 --before default_reset --after hard_reset write_flash 0x0 build\bootloader\bootloader.bin 0x8000 build\partition_table\partition-table.bin 0x10000 build\esp32c6_ble_audio_broadcast.bin
```

### 2. Build and Flash Audio SOURCE (Node21 on COM21)
1. Set `#define CONFIG_ACTIVE_NODE_ROLE NODE_ROLE_SOURCE` in `main/config.h`.
2. Compile and flash:
```powershell
Set-Location apps\node2node
idf.py build
python -m esptool --chip esp32c6 -p COM21 -b 460800 --before default_reset --after hard_reset write_flash 0x0 build\bootloader\bootloader.bin 0x8000 build\partition_table\partition-table.bin 0x10000 build\esp32c6_ble_audio_broadcast.bin
```

### 3. Running the Python Bumble Broadcaster (Node21 on COM121)
```powershell
python apps\usb_ble_bumble\bumble_broadcaster.py --port COM121 --sample-rate 48000
```

---

## 6. Live Diagnostic Telemetry Trace

```text
[NODE23] I (593) I2S_AUDIO: I2S DAC Driver Initialized: Fs=48000 Hz, Mono 16-bit, BCLK=1, WS=2, DOUT=3
[NODE23] I (738) ESP_LC3: Espressif Fixed-Point LC3 Decoder Initialized: 48000 Hz, 1-ch, 10000 us duration, 80 octets/frame
[NODE23] I (889) BLE_AUDIO: Bluetooth State Transition: [SCANNING] ---> [STREAMING]
[NODE23] I (1579) : ===== [ESP32-C6-23] heartbeat #1 | Uptime: 1 s =====
[NODE23] I (1581) BT: Role: SINK (Receiver) | State: STREAMING | Pkts: 73 | RSSI: -43 dBm | BIS ID: 1
[NODE23] I (1591) AUDIO: Codec: LC3 fixp @ 64 kbps | Mono 16-bit 48.0 kHz | VCS Vol: 30% (77/255 UNMUTED)
[NODE23] I (2580) : ===== [ESP32-C6-23] heartbeat #2 | Uptime: 2 s =====
[NODE23] I (2583) BT: Role: SINK (Receiver) | State: STREAMING | Pkts: 173 | RSSI: -39 dBm | BIS ID: 1
[NODE23] I (3574) : ===== [ESP32-C6-23] heartbeat #3 | Uptime: 3 s =====
[NODE23] I (3582) BT: Role: SINK (Receiver) | State: STREAMING | Pkts: 273 | RSSI: -43 dBm | BIS ID: 1
[NODE23] I (4574) : ===== [ESP32-C6-23] heartbeat #4 | Uptime: 4 s =====
[NODE23] I (4577) BT: Role: SINK (Receiver) | State: STREAMING | Pkts: 373 | RSSI: -36 dBm | BIS ID: 1
[NODE23] I (5575) : ===== [ESP32-C6-23] heartbeat #5 | Uptime: 5 s =====
[NODE23] I (5578) BT: Role: SINK (Receiver) | State: STREAMING | Pkts: 473 | RSSI: -42 dBm | BIS ID: 1
[NODE23] I (6577) : ===== [ESP32-C6-23] heartbeat #6 | Uptime: 6 s =====
[NODE23] I (6579) BT: Role: SINK (Receiver) | State: STREAMING | Pkts: 573 | RSSI: -41 dBm | BIS ID: 1
[NODE23] I (7579) : ===== [ESP32-C6-23] heartbeat #7 | Uptime: 7 s =====
[NODE23] I (7588) BT: Role: SINK (Receiver) | State: STREAMING | Pkts: 674 | RSSI: -44 dBm | BIS ID: 1
```

---

## 7. Troubleshooting & Hardware Verification

| Symptom | Probable Cause | Corrective Action |
| :--- | :--- | :--- |
| **Complete Silence** | `SD_MODE` floating near threshold | Tie `SD_MODE` directly to **`3.3V` (Left Header Pin 3)**. |
| **Low / Quiet Volume** | Amplifier gain set to 6 dB | `GP0` automatically sets **12 dB gain**; verify wire is firmly connected. |
| **Digital Distortion / Buzzing** | FreeRTOS task delay choking DMA | Do not call `vTaskDelay` during streaming; let `i2s_channel_write()` pace continuous DMA buffers. |
| **Task Watchdog Panic (`IDLE CPU 0`)** | Task priority or missing yield | Set `ble_audio_task` priority to `3` and ensure `vTaskDelay(1)` is executed when yielding. |
| **RGB LED Blinks Red during Idle** | GRB vs RGB byte order swap | `status_led.cpp` handles byte order compensation automatically. |

---

## License

This project is licensed under the [GNU Affero General Public License Version 3 (AGPL-3.0)](../../LICENSE).
