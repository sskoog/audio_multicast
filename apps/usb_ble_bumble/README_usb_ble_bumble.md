# USB BLE Bumble: ESP32-C6 Bluetooth 5.3 HCI Controller & Auracast Broadcaster

## 0. Intro
An ESP32-C6 firmware and Python application to configure a Bluetooth Low-Energy (BLE) 5.3 Audio Broadcaster (Auracast transmitter) on Windows 11 PCs.

### Requirements
* **ESP32-C6** ("ESP32-C6-WROOM-1 DevKit")
* [Google Bumble](https://github.com/google/bumble)
* [ESP-IDF](https://docs.espressif.com/projects/esp-idf/)
* Python 3.10+
* [VB-Audio Virtual Cable](https://vb-audio.com/Cable/) (optional, for streaming live Windows PC audio)

## 1. System Overview & Architecture

`usb_ble_bumble` provides a complete, high-performance Bluetooth Low Energy (BLE) 5.3 Audio / Auracast broadcasting stack for Windows 11 PCs.

Because native Windows 11 Bluetooth drivers currently lack Host-level APIs for creating Broadcast Isochronous Streams (BIS) and Broadcast Isochronous Groups (BIG), this system offloads the Bluetooth 5.3 Controller Link Layer to an external **ESP32-C6 RISC-V SoC** connected via USB-UART, driven by **Google Bumble** and **Google `liblc3`** in Python.

```
+---------------------------------------------------------------------------------------------------------+
|                                           WINDOWS 11 PC (HOST)                                          |
|                                                                                                         |
|  +---------------------------------------------------------------------------------------------------+  |
|  | [Audio Sources]                                                                                   |  |
|  |  * LFO Synthesizer: 220Hz-880Hz @ 0.2Hz LFO, 25% Amplitude, Continuous Phase Integration           |  |
|  |  * Live Windows Audio Capture: VB-Audio Virtual Cable / WASAPI / DirectSound (48kHz 16-bit PCM)  |  |
|  +---------------------------------------------------------------------------------------------------+  |
|                                                     │ (480 PCM Samples / 10ms Frame)                    |
|                                                     ▼                                                   |
|  +---------------------------------------------------------------------------------------------------+  |
|  | [Google liblc3 Codec Engine] (liblc3.dll + lc3_encoder.py)                                        |  |
|  |  * 48kHz, 10.0ms Framing, LC3 Compression (960 Bytes PCM -> 120 Bytes Stereo / 60 Bytes Mono)    |  |
|  +---------------------------------------------------------------------------------------------------+  |
|                                                     │ (LC3 Compressed Bitstream)                        |
|                                                     ▼                                                   |
|  +---------------------------------------------------------------------------------------------------+  |
|  | [bumble_broadcaster.py] (Google Bumble Host Stack)                                                |  |
|  |  * Extended Advertising (1M/2M PHY, PBA UUID 0x1852)                                              |  |
|  |  * Periodic Advertising Sync (BASE UUID 0x1851, LC3 Codec Config, Spatial Channel Map)            |  |
|  |  * Isochronous Stream Scheduler: HCI_IsoDataPacket framing (10.0ms microsecond pacing)           |  |
|  +---------------------------------------------------------------------------------------------------+  |
|                                                     │                                                   |
|                                 H4 Transport (CMD 0x01, ACL 0x02, EVT 0x04, ISO 0x05)                   |
|                                                     ▼                                                   |
|                                     PySerial / serial_asyncio (COM121 @ 115200 baud)                    |
+-----------------------------------------------------│---------------------------------------------------+
                                                      │ USB Virtual Serial Port (CH343 / USB-UART)
+-----------------------------------------------------▼---------------------------------------------------+
|                                            ESP32-C6 RISC-V SOC                                          |
|                                                                                                         |
|  +---------------------------------------------------------------------------------------------------+  |
|  | usb_ble_bumble Firmware (Pure H4 Bridge)                                                          |  |
|  |  * UART0 RX Task -> HCI Packet Demux -> ble_hci_trans_hs_cmd_tx() / hs_acl_tx() / hs_iso_tx()      |  |
|  |  * NimBLE LL Callback -> ble_hci_trans_cfg_hs() -> UART0 Direct Buffered TX                        |  |
|  |  * WS2812 Status LED Controller (GPIO 8): 0.5Hz Green (Idle), 3.0Hz Blue (Transmitting)          |  |
|  |  * Console & Bootloader ASCII logs disabled for 100% Binary Stream Integrity                      |  |
|  +---------------------------------------------------------------------------------------------------+  |
|                                                     │                                                   |
|  +---------------------------------------------------------------------------------------------------+  |
|  | ESP32-C6 Hardware Bluetooth 5.3 Link Layer                                                        |  |
|  |  * Extended Advertising Engine (5 simultaneous advertising sets)                                  |  |
|  |  * Periodic Advertising Train Generator                                                           |  |
|  |  * Isochronous Broadcaster: BIG / BIS Controller Engine (LE 2M PHY)                               |  |
|  +---------------------------------------------------------------------------------------------------+  |
|                                                     │                                                   |
|                                                2.4 GHz RF                                               |
|                                                     ▼                                                   |
|                                 Auracast Receivers / BLE Audio Earbuds / Speakers                       |
+---------------------------------------------------------------------------------------------------------+
```

---

## 2. Operational Modes

### Mode 1: Single BIS Stereo Broadcaster
* **Configuration**: 1 Broadcast Isochronous Group (BIG Handle 0) containing **1 BIS stream** (BIS Handle 1).
* **Audio Location**: Left (`0x00000001`) + Right (`0x00000002`) Front Channels (`0x00000003`).
* **Frame Size**: 120 octets per 10ms frame @ 48 kHz LC3 (60 bytes Left + 60 bytes Right).
* **Usage**: Standard stereo Auracast broadcast for consumer headphones, earbuds, and stereo soundbars.

### Mode 2: Multi-Channel BIG (2 to 6 Discrete Mono BIS Streams)
* **Configuration**: 1 BIG containing **2 to 6 independent BIS streams** (e.g. BIS Handles 1, 2, 3, 4, 5, 6).
* **Audio Location**: Discrete per-channel spatial positions:
  * **BIS 1**: Front Left (`0x00000001`)
  * **BIS 2**: Front Right (`0x00000002`)
  * **BIS 3**: Front Center (`0x00000004`)
  * **BIS 4**: Low Frequency Effects / Subwoofer (`0x00000008`)
  * **BIS 5**: Surround Left (`0x00000010`)
  * **BIS 6**: Surround Right (`0x00000020`)
* **Frame Size**: 60 octets per stream per 10ms frame @ 48 kHz LC3.
* **Usage**: Multi-channel surround sound (5.1 / 6.0), multi-room synchronized audio, or simultaneous multi-language interpretation.

---

## 3. Real-Time Audio Sources & Codec Engine

### 1. Google `liblc3` Audio Codec ([`lc3_encoder.py`](lc3_encoder.py))
* **Standard Implementation**: Google's official C implementation of the Bluetooth SIG Low Complexity Communication Codec (LC3), compiled natively for Windows (`liblc3.dll`).
* **Format**: 48,000 Hz, 10.0 ms frame duration (480 PCM samples per frame).
* **Performance**: Sub-millisecond ctypes encoding, producing 60-byte mono frames (48 kbps) or 120-byte stereo SDUs (96 kbps).

### 2. Synthesizer Test Source (`--source synth`)
* **Carrier & Modulation**: Modulated sine wave with **0.2 Hz LFO** (5.0s sweep period) continuously sweeping carrier frequency between **220.0 Hz** (A3) and **880.0 Hz** (A5).
* **Glitch-Free Phase Accumulation**: Continuous angular phase integration preventing clicks or pops across 10ms buffer boundaries.
* **Amplitude**: Default 25% peak amplitude (`8191 / 32767`).

### 3. Live Audio Capture / Virtual Cable (`--source device`)
* **Capture Engine**: Uses `sounddevice` (WASAPI / DirectSound) to capture live 48 kHz PCM audio from **VB-Audio Virtual Cable** (`"CABLE Output"`) or any connected microphone/soundcard.
* **Asynchronous Queue**: Non-blocking 10ms frame slicing feeding the LC3 encoder without audio stutter.

---

## 4. Basic Audio Profile (BAP) Configuration Engine ([`bap_config.py`](bap_config.py))

`bap_config.py` provides a fool-proof, user-friendly API for constructing Bluetooth SIG compliant **Broadcast Audio Source Endpoint (BASE)** descriptors for Periodic Advertising trains:

### 1. Sampling Frequency Resolution (`get_sampling_frequency_code`)
* **Fool-Proof Rounding**: Non-standard sample rate inputs automatically round to the nearest Bluetooth SIG standard frequency:
  * `44000 Hz` -> `44100 Hz` (CD Audio, Code `0x07`)
  * `47999 Hz` -> `48000 Hz` (Studio/Broadcast, Code `0x08`)
  * `32100 Hz` -> `32000 Hz` (Super-Wideband, Code `0x06`)
* **Range Validation**: Rejects out-of-bounds rates (< 7,000 Hz or > 100,000 Hz) with an informative `ValueError`.

### 2. Presentation Delay in Milliseconds (`get_presentation_delay`)
* **Human-Readable API**: Accepts requested presentation delay in milliseconds (e.g. `10.0 ms`, `40.0 ms`, `100.0 ms`).
* **Format Conversion**: Clamps between `5.0 ms` and `250.0 ms`, converts to integer microseconds, and packs into a 3-byte Little-Endian 24-bit integer.

### 3. Frame Duration Resolution (`get_frame_duration_code`)
* Returns valid Bluetooth SIG byte codes:
  * `7.5 ms` -> Code `0x00` (`DURATION_7500_US`, low-latency/gaming)
  * `10.0 ms` -> Code `0x01` (`DURATION_10000_US`, standard broadcast)

### 4. LTV 3: Octets Per Codec Frame Presets (`get_octets_per_codec_frame`)
Provides pre-calculated, standard bitrate presets or custom byte sizing:
* `"voice"`: 40 octets/frame mono (32 kbps) | 80 octets stereo (64 kbps)
* `"standard"`: 60 octets/frame mono (48 kbps) | 120 octets stereo (96 kbps)
* `"high_quality"`: 100 octets/frame mono (80 kbps) | 200 octets stereo (160 kbps)
* `"audiophile"`: 120 octets/frame mono (96 kbps) | 240 octets stereo (192 kbps)

### 5. Metadata LTV: Audio Context Types & Language (`build_metadata_ltv`)
Supports selecting standard Bluetooth SIG audio contexts, track titles, and ISO-639 language codes:
* Contexts: `media` (`0x0004`), `conversational` (`0x0002`), `game` (`0x0008`), `live` (`0x0040`), `instructional` (`0x0010`), `alerts` (`0x0400`), `emergency_alarm` (`0x0800`).
* Optional Program Info string (LTV 0x03) and 3-letter Language code (LTV 0x04, e.g. `"eng"`, `"swe"`).

---

## 5. On-Board Status LED Feedback (WS2812 on GPIO 8)

The ESP32-C6-WROOM-1 DevKit on-board addressable RGB LED provides real-time visual hardware feedback:

| State | Pattern / Frequency | Color | Duty Cycle | ON / OFF Times | Description |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **IDLE** | **Slow Pulse (0.5 Hz)** | **Green** | **16 / 255** (~6.25%) | 125 ms ON, 1875 ms OFF | Controller initialized, waiting for host H4 commands |
| **ADVERTISING** | **Medium Pulse (1.0 Hz)** | **Cyan** | **16 / 255** (~6.25%) | 63 ms ON, 937 ms OFF | Auracast PA/EA metadata announcement active |
| **TRANSMITTING** | **Fast Flash (3.0 Hz)** | **Blue** | **32 / 255** (~12.5%) | 42 ms ON, 291 ms OFF | Active HCI command processing / ISO audio streaming |
| **RESET / SYNC** | **Double Blink (2.0 Hz)** | **Purple** | **32 / 255** (~12.5%) | 62 ms ON, 438 ms OFF | Host transport reset / re-initialization |
| **ERROR / OVERRUN**| **Fast Strobe (5.0 Hz)** | **Red** | **64 / 255** (~25.0%) | 50 ms ON, 150 ms OFF | Buffer allocation drop or UART FIFO overflow warning |

> **Hardware Color Compensation**: The ESP32-C6 DevKit WS2812 LED hardware uses an RGB die order across its shift register. The firmware automatically applies channel mapping compensation to ensure Green is pure Green (`0, 255, 0`) and Red is pure Red (`255, 0, 0`).

---

## 6. Pitfalls Encountered & Technical Solutions

### Pitfall 1: FreeRTOS Stack Protection Fault
* **Problem**: When running `uart_rx_task`, the CPU immediately panicked with `Guru Meditation Error: Core 0 panic'ed (Stack protection fault)`.
* **Root Cause**: `uart_rx_task` allocated two 4096-byte buffers (`rx_buf`, `pkt_buf`) locally on the task stack, exceeding the 4096-byte FreeRTOS task stack size.
* **Solution**: Relocated packet parsing and UART buffers to static BSS memory (`static uint8_t s_rx_raw_buf[...]`, `static uint8_t s_rx_pkt_buf[...]`).

### Pitfall 2: ESP-IDF VHCI Routing vs Direct NimBLE Transport Binding
* **Problem**: Calling standard `esp_vhci_host_register_callback()` on ESP32-C6 in ESP-IDF v5.2 routed incoming controller HCI events internally to Bluedroid rather than the user callback.
* **Solution**: Bound directly to the NimBLE Link Layer HCI transport interface using `ble_hci_trans_cfg_hs(controller_to_host_evt_cb, NULL, controller_to_host_acl_cb, NULL)`. Outgoing commands use `ble_hci_trans_buf_alloc()` and `ble_hci_trans_hs_cmd_tx()`.

### Pitfall 3: Serial Protocol Purity (UART0 Log Interference)
* **Problem**: ASCII bootloader banners and `ESP_LOG` messages on UART0 corrupt the binary H4 stream (`0x01` CMD, `0x04` EVT, `0x05` ISO), causing the host HCI parser to drop synchronization.
* **Solution**: Fully disabled console output on UART0 via `CONFIG_ESP_CONSOLE_NONE=y`, `CONFIG_BOOTLOADER_LOG_LEVEL_NONE=y`, and `CONFIG_LOG_DEFAULT_LEVEL_NONE=y` in `sdkconfig.defaults`.

### Pitfall 4: PySerial DTR/RTS Reset Handling
* **Problem**: Default serial port opening in Python asserts DTR/RTS, causing an ESP32 hardware reset and emitting ROM bootloader characters.
* **Solution**: In custom probe scripts, set `dtr=False, rts=False` before opening; in Bumble, configured transport spec `serial:COM121,115200,delay` to allow clean bootloader settling.

### Pitfall 5: WS2812 Color Channel Inversion
* **Problem**: Setting Green illuminated the on-board LED in Red.
* **Root Cause**: ESP-IDF `led_strip` default `LED_PIXEL_FORMAT_GRB` sends Green then Red over RMT, but the ESP32-C6 DevKit on-board LED expects Red then Green.
* **Solution**: Implemented channel compensation in `status_led.c` swapping the Red and Green arguments before packing RMT pulses.

### Pitfall 6: Meson-Python Shared Library Packaging on Windows
* **Problem**: `pip install liblc3` failed during metadata generation due to `mesonpy` disallowing internal DLL bundling in Windows wheels.
* **Solution**: Built Google `liblc3` directly into `liblc3.dll` using WinLibs GCC (`gcc -O3 -shared -o liblc3.dll ...`) and loaded it via high-performance Python ctypes.

### Pitfall 7: Host Abort & Link Layer Residual Broadcast
* **Problem**: Aborting the host script left the ESP32 Link Layer actively emitting Extended and Periodic Advertising trains on 2.4 GHz, causing receivers to detect phantom broadcasts, and failing to detect hardware resets.
* **Solution**:
  1. Monitored `hci_source.terminated` on every packet transmission and sleep interval to immediately detect ESP32 disconnects/resets and abort cleanly.
  2. Implemented `stop_broadcast(device)` executing `HCI_LE_Terminate_BIG_Command`, disabling Periodic & Extended Advertising, and issuing `HCI_Reset_Command`, immediately returning the controller to Standby and resetting the status LED to slow 0.5 Hz Green idle.

---

## 7. Verification & Test Plan

### Test 1: HCI Hardware Diagnostic Probe (`test_hci.py`)
Validates that the ESP32-C6 controller link layer is responsive and compliant with Bluetooth 5.3 specifications:
```powershell
python apps\usb_ble_bumble\test_hci.py COM121
```

### Test 2: BAP / PBP BASE Descriptor Unit Tests (`test_bap_config.py`)
Validates fool-proof sample rate rounding, presentation delay byte conversions, frame duration codes, LTV 3 presets, and metadata contexts:
```powershell
python apps\usb_ble_bumble\test_bap_config.py
```

### Test 3: Stereo Broadcaster with Custom BAP Parameters
Validates live LC3 streaming with custom presentation delay (50ms), High-Quality preset (160 kbps), and live context:
```powershell
python apps\usb_ble_bumble\bumble_broadcaster.py --port COM121 --mode stereo --source synth --sample-rate 44000 --presentation-delay 50.0 --quality-preset high_quality --context live --program-info "Live Stage" --language eng --test-duration 3
```

---

## 8. How to Run & CLI Reference

### Display BAP Configuration Reference
```powershell
python apps\usb_ble_bumble\bumble_broadcaster.py --list-bap-options
```

### List All Available Audio Capture Devices
```powershell
python apps\usb_ble_bumble\bumble_broadcaster.py --list-audio-devices
```

### Broadcast LFO Sine Wave Test Tone (Stereo)
```powershell
python apps\usb_ble_bumble\bumble_broadcaster.py --port COM121 --mode stereo --source synth --amplitude 0.25 --lfo-rate 0.2 --lfo-min 220 --lfo-max 880
```

### Broadcast Live from VB-Audio Virtual Cable (Custom BAP Metadata)
```powershell
python apps\usb_ble_bumble\bumble_broadcaster.py --port COM121 --mode stereo --source device --audio-device "CABLE" --quality-preset high_quality --context media --program-info "Living Room Hifi" --language eng
```

### Broadcast 5.1 Multi-Channel Surround (6 Discrete Streams)
```powershell
python apps\usb_ble_bumble\bumble_broadcaster.py --port COM121 --mode multichannel --num-bis 6 --source synth
```

### Flashing Firmware to ESP32-C6
```powershell
. "C:\Users\stefa\OneDrive\Documents\ESP\v6.0.2\esp-idf\export.ps1"

Set-Location apps\usb_ble_bumble
idf.py build
python -m esptool --chip esp32c6 -p COM21 -b 460800 --before default_reset --after hard_reset write_flash --flash_mode dio --flash_size 4MB --flash_freq 40m 0x0 "build/bootloader/bootloader.bin" 0x8000 "build/partition_table/partition-table.bin" 0x10000 "build/usb_ble_bumble.bin"
```
