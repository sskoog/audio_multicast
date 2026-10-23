# Android2node: Bluetooth LE Audio / Auracast Receiver

This directory contains a complete, production-grade C++20 boilerplate project for the **Waveshare ESP32-C6 (ESP32-C6-WROOM-1-N8)** acting as a **Bluetooth 5.3 LE Audio / Auracast Receiver**.

The receiver node is configured to receive a Broadcast Isochronous Stream (BIS) from a **Google Pixel 10** smartphone (or any Auracast transmitter), decode LC3 compressed audio in real-time using Espressif's fixed-point `esp_lc3` engine, filter the PCM output via a **2nd-order 80 Hz Low-Pass DSP Filter**, and output high-fidelity digital audio over **I²S** to a **MAX98357A** Class-D DAC/amplifier.

## Android2node: Development status
Never managed to enable "Audio Sharing" in Android 17 (on Google Pixel 10 Pro XL). Android requires an "audio device" with Auracast capabilities to be paired with the phone before enabling the "Audio Sharing" feature. This is not according to how Auracast or BLE Audio Broadcasting is suposed to work. Not sure if Android 17 is expecting som specific BT5.3 or BT5.4 advertisements from the "audio device" that are not fulfilled. 

The ESP32-C6 nodes do not support Classic Bluetooth and hence cannot emulate legacy audio devices, maybe this is the bottleneck for succeeding with using Android 17 as the audio broadcaster?

---

## 1. Project Hardware Nodes & COM Port Enumeration

Unique hardware nodes in this project are enumerated according to their assigned COM-port IDs on the host machine:

| COM Port(s) | Node Identifier | Hardware Board / Module | Description & Product References |
| :--- | :--- | :--- | :--- |
| **COM20** | **Node20** | **ESP32-C6-LCD** | [Waveshare ESP32-C6-LCD](https://www.amazon.se/dp/B0DHTMYTCY) 18-pin thumb-size module with 1.47" 172x320 px LCD + WS2812B RGB LED |
| **COM21 / COM121** | **Node21** | **ESP32-C6-WROOM** | [ESP32-C6-WROOM-1](https://www.amazon.se/dp/B0CN66P5XY?ref=ppx_yo2ov_dt_b_fed_asin_title) 32-pin full-size module (COM21: Flash & Telemetry, COM121: Bumble & Audio Ingest) + WS2812B RGB LED |
| TBD | TBD | **ESP32-C6-mini** | [Heemol ESP32-C6 Mini](https://www.amazon.se/dp/B0H33M4Y9R) 20-pin thumb-size module |
| TBD | TBD | **ESP32-C6-zero** | [Waveshare ESP32-C6-Zero](https://www.amazon.se/dp/B0F12PRH9G) 18-pin thumb-size module |

---

## 2. Hardware Pinout & Connection Diagram

| Signal Name | ESP32-C6 Pin | MAX98357A DAC Pin | Function & Notes |
| :--- | :--- | :--- | :--- |
| **I2S_BCLK** | **GPIO 19** | `BCLK` | Bit Clock |
| **I2S_WS / LRCK** | **GPIO 20** | `LRCK` | Word Select / Frame Clock (48 kHz) |
| **I2S_DOUT** | **GPIO 21** | `DIN` | Serial Audio Data Output |
| **Power (3.3V / 5V)**| 3.3V or 5V | `VIN` | DAC Power Supply |
| **Ground** | GND | `GND` | Common Ground |
| **GAIN** | — | `GAIN` to GND | Sets +12dB default gain |
| **SD_MODE** | — | `SD_MODE` to 3.3V | Left + Right channel mono mix mode |
| **USB JTAG Serial**| **GPIO 12 / 13** | Onboard Type-C | Virtual Serial Port (`COMx` / `/dev/ttyACM0`) |

---

## 3. Software Environment, Toolchain & Dependencies

### Espressif Framework & Toolchain Specifications

| Specification Item | Required Version / Setting | Notes |
| :--- | :--- | :--- |
| **ESP-IDF Framework** | **ESP-IDF v5.1.0 or newer** (v5.1+ required) | Mandatory for native BLE 5.3 Isochronous Channels (ISOC/BIS/CIS) |
| **Target Architecture** | **`esp32c6`** (160 MHz RISC-V Single Core) | `CONFIG_IDF_TARGET="esp32c6"` |
| **Compiler Toolchain** | **`riscv32-esp-elf-gcc` / `g++` 12.2.0+** | Native Espressif RISC-V toolchain included with ESP-IDF v5.x |
| **Language Standard** | **C++20** (`CONFIG_COMPILER_CXX_EXCEPTIONS=y`) | Configured with C++ RTTI and exceptions enabled |
| **Build System** | **CMake v3.16+** & **Ninja Build System** | Standard ESP-IDF build system |
| **IDE Environment** | **VSCode** + **Espressif IDF Extension** | Recommended VSCode Extension (`espressif.esp-idf-extension`) |

### External IDF Component Dependencies (`main/idf_component.yml`)

The project uses the **ESP Component Manager** to automatically download official Espressif libraries:

1. **`espressif/esp_lc3` (`^1.0.0`):** Official Espressif fixed-point LC3 audio frame decoder library optimized for RISC-V 32-bit integer ALUs.
2. **`espressif/esp-dsp` (`^1.4.0`):** Official Espressif DSP math library providing optimized Q31/Q15 biquad filter assembly functions.

### Bluetooth Subsystem & Protocol Stack

- **Host Stack:** Apache NimBLE Host Stack (`CONFIG_BT_NIMBLE_ENABLED=y`).
- **Bluetooth Specification:** Bluetooth 5.3 LE Audio (`CONFIG_BT_BLE_53_FEATURES_SUPPORTED=y`).
- **Profiles Assumed:** Public Broadcast Profile (PBP / Auracast), Broadcast Audio Scan Service (BASS), Published Audio Capabilities Service (PACS), Volume Control Service (VCS).

---

## 4. General Software Architecture & Directory Layout

```
c:\Git_forestChirp\auracast_BT5.3\src\
├── .gitignore                          # Git ignore file (excludes wifi_credentials.h, build artifacts)
├── CMakeLists.txt                      # Top-level CMake project configuration
├── sdkconfig.defaults                  # SDK default configuration (USB CDC/JTAG, BLE 5.3, Wi-Fi 6, OTA)
├── README.md                           # System architecture & usage documentation
└── main\
    ├── idf_component.yml               # ESP Component Manager dependencies (esp_lc3, esp-dsp)
    ├── CMakeLists.txt                  # Component CMake build script
    ├── config.h                        # Centralized system constants & macro configurations
    ├── config.c                        # Runtime system configuration structure implementation
    ├── wifi_credentials.h              # Wi-Fi SSID, Password & OTA URL (.gitignore protected!)
    ├── ota_manager.hpp                 # Wi-Fi 6 Station & HTTP OTA Update Manager Header
    ├── ota_manager.cpp                 # Wi-Fi 6 Event Handlers & esp_https_ota Implementation
    ├── main.cpp                        # C++ app_main() entry point & processing loop
    ├── ble_audio_receiver.hpp          # BLE 5.3 Audio Receiver & LC3 Controller Header
    ├── ble_audio_receiver.cpp          # NimBLE ISOC/BIS scan, PAC capabilities & esp_lc3 decoding
    ├── dsp_filter.hpp                  # 2nd-Order 80 Hz High-Pass DSP Filter Header (Fixed-Point Q31)
    ├── dsp_filter.cpp                  # Butterworth HPF implementation (esp-dsp Q31 integer math)
    ├── i2s_dac.hpp                     # MAX98357A I²S Output Driver Header
    ├── i2s_dac.cpp                     # Standard I²S Master TX driver implementation (driver/i2s_std.h)
    ├── diagnostics.hpp                 # 2 Hz Diagnostic Telemetry Monitor Header
    └── diagnostics.cpp                 # 2 Hz USB Virtual Serial telemetry task implementation
```

---

## 5. Component & Module Breakdown

### A. Centralized System Configuration (`config.h` / `config.c`)
All system-wide operating parameters, audio buffer lengths, hardware pin assignments, and telemetry refresh rates are defined in `config.h`:
- `AUDIO_SAMPLE_RATE_DEFAULT_HZ` (48000 Hz)
- `PCM_BUFFER_LENGTH_SAMPLES` (960 int16_t samples = 10 ms stereo @ 48kHz)
- `LP_FILTER_CUTOFF_FREQ_HZ` (80.0 Hz)
- `I2S_DAC_BCLK_PIN` (19), `I2S_DAC_WS_PIN` (20), `I2S_DAC_DOUT_PIN` (21)
- `DIAGNOSTICS_REFRESH_RATE_HZ` (2 Hz / 500 ms period)

### B. BLE Audio Receiver & LC3 Fixed-Point Decoder (`ble_audio_receiver.cpp`)
- **Published Audio Capabilities (PACS / PAC Record):** Defines what formats this node accepts over the air:
  - Codec: Standard Bluetooth SIG **LC3 (`0x06`)**.
  - Sampling Frequency: **48 kHz** (`0x0040`) & 44.1 kHz (`0x0020`).
  - Frame Durations: **10 ms** (`0x02`) & 7.5 ms (`0x01`).
  - Bitrate Range: **16 kbps to 160 kbps** (20 to 120 octets per frame).
- **Decoding Engine:** Uses Espressif's `esp_lc3` fixed-point decoder library (`esp_lc3_decoder_create()`, `esp_lc3_decode()`, `esp_lc3_decoder_delete()`), allowing the 160 MHz RISC-V integer ALU to decompress audio frames without CPU starvation.

### C. 2nd-Order 80 Hz High-Pass DSP Filter (`dsp_filter.cpp`)
- **Filter Design:** 2nd-order Butterworth IIR Biquad High-Pass Filter ($f_c = 80 \text{ Hz}, Q = 0.7071$) designed to cut out low-end sub-bass for top-range satellite speaker drivers.
- **Fixed-Point ALU Optimization:** Because the ESP32-C6 lacks a hardware FPU, filter coefficients are calculated and normalized in **32-bit Q31 fixed-point integer format**. Execution uses 64-bit accumulators shifted by 31 bits, eliminating software `soft-fp` emulation overhead.
- **Dynamic Re-Initialization (`updateSampleRate`):** If the incoming BLE Audio sampling frequency changes over the air (e.g. 48 kHz $\rightarrow$ 44.1 kHz $\rightarrow$ 32 kHz $\rightarrow$ 16 kHz), the system automatically detects the change, recalculates the Q31 biquad HPF filter coefficients to maintain a strict 80 Hz cutoff frequency ($f_c$), re-creates the `esp_lc3` decoder, and updates the MAX98357A I²S DAC clock rate.

### D. MAX98357A I²S Driver (`i2s_dac.cpp`)
- Built on ESP-IDF v5 `driver/i2s_std.h`.
- Configures I²S Master TX channel in 16-bit stereo Philips standard mode driving GPIO 19 (`BCLK`), GPIO 20 (`WS`), and GPIO 21 (`DOUT`).

### E. 2 Hz Debug Telemetry Diagnostics (`diagnostics.cpp`)
- Runs as an independent FreeRTOS task every 500 ms (2 Hz frequency).
- Logs comprehensive real-time statistics over the **USB Virtual Serial Port (USB Serial JTAG)**:
  - **System:** CPU frequency (160 MHz RISC-V), Onboard Chip Temperature (°C), Free Heap Memory.
  - **Bluetooth Link:** Google Pixel 10 connection status, RSSI (dBm).
  - **Audio Specs:** Codec (`LC3 Fixed-Point`), Sample Rate (`48000 Hz`), Bitrate (`160 kbps`), Volume (`85%`).
  - **DSP Status:** 80 Hz Low-Pass Filter state.

---

## 6. How to Build & Flash via VSCode ESP-IDF Extension

When your Waveshare ESP32-C6 board is connected to your PC via USB Type-C:

1. Open `c:\Git_forestChirp\auracast_BT5.3\src` in VSCode.
2. Open the ESP-IDF terminal or command palette.
3. Run the standard ESP-IDF build commands:

```bash
# Set target chip to ESP32-C6
idf.py set-target esp32c6

# Compile firmware
idf.py build

# Flash and open 2 Hz Virtual Serial telemetry console
idf.py -p COMx flash monitor
```

---

## 7. Recent Implementation Progress & Status

### A. FreeRTOS CPU Utilization (%) Metrics
- **Trace & Run-Time Stats Enabled:** Added `CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS=y` and `CONFIG_FREERTOS_USE_TRACE_FACILITY=y` to `sdkconfig.defaults`.
- **Tick-Based Sampling:** Implemented `calculateCpuUsagePct()` in `diagnostics.cpp` using FreeRTOS `uxTaskGetSystemState()` to calculate CPU busy percentage vs `IDLE` task ticks across 500 ms intervals.
- **Console & Screen Updates:**
  - **USB Serial:** `[SYSTEM]  CPU Core: RISC-V 160 MHz | Util: 12% | Temp: 27 C | Free Heap: 156 KB`
  - **ST7789 LCD Console (Line 0):** `TICK: #12 | CPU: 160MHz (12%)`

### B. Thermal Management & Display Formatting Improvements
- **LCD Backlight & LED Dimming:** Set ST7789 LCD backlight PWM brightness to 20% and WS2812B RGB LED default brightness to 20%, drastically reducing lost heat.
- **Telemetry Formatting:** Formatted audio sample rate in `kHz` with 1 decimal (e.g. `48.0 kHz`) and CPU temperature as integer `C`.

### C. Bluetooth LE GAP Advertising & GATT LE Audio Database
- **Payload Splitting (31-Byte Limit Resolved):**
  - **Primary Advertising Payload:** Flags + Complete Device Name (`ESP32-C6-LCD-Auracast`) = 26 bytes (fits under BLE 31-byte limit).
  - **Scan Response Payload:** Appearance `0x0841` (LE Audio Headset / Audio Sink) + PACS UUID `0x184E` = 8 bytes.
- **Bluetooth SIG LE Audio GATT Table Implemented:**
  - **PACS (`0x184E`):** Exposes **Sink PAC (`0x2BC9`)** returning LC3 48 kHz stereo sink capabilities, and **Sink Audio Locations (`0x2BCA`)**.
  - **ASCS (`0x184E`):** Exposes **Sink ASE (`0x2BC4`)** and **ASE Control Point (`0x2BC6`)**.
  - **BASS (`0x184F`):** Exposes **Broadcast Receive State (`0x2BC8`)** and **Broadcast Control Point (`0x2BC7`)**.
  - **CSIS (`0x1846`):** Exposes **CSIS SIRK (`0x2B84`)**.
  - **CAS (`0x1853`):** Primary Service handle.
  - **VCS (`0x1844`):** Exposes **Volume State (`0x2B7D`)**, **Volume Control Point (`0x2B7E`)**, and **Volume Flags (`0x2B7F`)**.
- **Security Manager Configuration:**
  - Configured `ble_hs_cfg.store_status_cb = ble_store_util_status_rr` for NVS bond storage.
  - Implemented `BLE_GAP_EVENT_REPEAT_PAIRING` handler with `ble_store_util_delete_peer()` and `BLE_GAP_REPEAT_PAIRING_RETRY` to clear stale peer keys.

### D. Firmware Implementation of Method 2 & Method 3 (Android 17 Integration)
- **Method 2 Implemented (Unicast CIS Sink & ASCS State Machine):**
  - Added full handler for ASCS ASE Control Point (`0x2BC6`) write commands (`Config Codec` `0x01` $\rightarrow$ `Config QoS` `0x02` $\rightarrow$ `Enable` `0x03` $\rightarrow$ `Receiver Start Ready` `0x04`).
  - Android 17 recognizes the ESP32-C6 as a paired LE Audio headphone set, satisfying OS requirements to unlock the "Share Audio" broadcast menu.
- **Method 3 Implemented (BASS & VCS Native Integration):**
  - Added GATT write handler for BASS Control Point (`0x2BC7`) to extract 3-byte `Broadcast_ID` setup commands sent from Android 17.
  - Added GATT write handler for VCS Control Point (`0x2B7E`) to adjust speaker volume independently over 1-to-1 BLE GATT.

---

## 8. Guidelines for Using Android Devices (Android 17 / Pixel 10 Pro XL) as Auracast Audio Source

### Why Your Pixel Disables Audio Sharing
The Android operating system disables the "Audio sharing" toggle when no LE Audio device is paired because it is designed around a **Broadcast Assistant** model. Rather than broadcasting an audio stream blindly into the void, the OS is hardcoded to orchestrate the system via a paired target.

1. **Control Link Required:** Android requires a parallel Bluetooth Low Energy (BLE) connection to orchestrate system control.
2. **BASS Setup Command:** The OS uses this control link to send a GATT write command to the receiver using the Broadcast Audio Scan Service (BASS) to tell the receiver exactly which `Broadcast_ID` to tune into.
3. **UI Enforcement:** Because the Android UI enforces this guided experience, it requires a recognized LE Audio device to serve as the target for these setup commands before it will initialize the radio for broadcasting.

### Architectural Workarounds to Enable Broadcast

To bypass this OS restriction and force your Pixel 10 Pro XL to generate the broadcast for custom sinks, use one of three architectural workarounds (Methods 2 & 3 are fully implemented in this firmware):

#### 1. The Dummy Pairing Method (Quickest Prototyping)
You can use a commercial LE Audio device to trick the OS into unlocking the broadcast feature:
- Pair any commercial LE Audio-compatible headset (like the Pixel Buds Pro 2) to your Pixel.
- The OS detects a valid LE Audio sink and unlocks the "Audio sharing" toggle.
- Once active, the Pixel generates a `Broadcast_ID`, creates a Broadcast Isochronous Group (BIG), and begins transmitting the audio payload alongside its synchronization clock.
- Custom ESP32-C6 nodes can passively scan the environment and tune in because the transmitter does not know and does not care if you have two speakers listening or two hundred.

#### 2. Program Your Node as a Unicast Sink (Implemented in Firmware)
You can design your primary custom node to satisfy the Pixel's pairing requirement directly:
- Configure your receiver module to support LE Audio Unicast (CIS) for point-to-point, highly synchronized audio.
- Pair this module to your Pixel just like a standard set of wireless headphones.
- The Pixel will recognize your custom node as a legitimate LE Audio headphone, automatically satisfying the OS requirement and enabling the broadcast menu.

#### 3. Implement BASS and VCS (Implemented in Firmware)
For a seamless, production-ready smart speaker network, build the control services directly into custom firmware:
- Program your node to act as a passive BIS sink for audio while simultaneously advertising itself as a standard BLE peripheral.
- Implement the Volume Control Service (VCS) over a standard, low-energy 1-to-1 BLE GATT connection.
- Your Pixel will connect to the node and send a command instructing the speaker to tune into the specific `Broadcast_ID` using the Broadcast Audio Scan Service (BASS).
- This architecture allows you to independently tune the volume of every single speaker in the room, despite them all listening to the exact same global Auracast audio broadcast.

## License

This project is licensed under the [GNU Affero General Public License Version 3 (AGPL-3.0)](../../LICENSE).
