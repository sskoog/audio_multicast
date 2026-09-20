# Audio VSAF Broadcast 2: VSAF 3.0 Dual-Frame Redundancy, Round-Robin Telemetry & Hardware ISR Paced Streaming

## 1. Executive Summary & Architectural Overview

The **`audio_VSAF_broadcast2`** application implements an ultra-low-latency, multi-channel, multi-speaker wireless digital audio distribution system over 802.11 Wi-Fi broadcast semantics (`FF:FF:FF:FF:FF:FF`). It is engineered to distribute up to 6 distinct audio channels synchronously from a single **SOURCE** transmitter (ESP32-S3) to **2 to 6 SINK** speaker nodes (ESP32-C6) with microsecond-level presentation timeline alignment and zero audible dropouts.

### Key Architectural Evolution in VSAF 3.0

1. **VSAF 3.0 Dual-Frame Payload Redundancy (t0 + t-1)**:
   - Each 248-byte audio broadcast packet contains **two full LC3 audio frames**: the current frame (t0, 120 bytes) and the preceding frame (t-1, 120 bytes).
   - If a single packet is lost in RF, the SINK recovers the missing audio frame instantly from the subsequent packet without requesting retransmissions, completely eliminating audio dropouts and Soft-ARQ repair windows.
2. **Commanded Round-Robin SINK Telemetry (1 Reply per 10 ms Window)**:
   - Rather than having multiple SINK nodes compete for the RF channel simultaneously, SINK feedback is strictly **commanded by the SOURCE via bit 7 (`REQ_ACK`) of `packet_flags`**.
   - Exactly one SINK is granted an uplink transmission slot per 10 ms window in round-robin fashion (60 ms full cluster status period across 6 channels), eliminating reverse-path collisions and preserving > 9.5 ms of quiet airtime per frame.
3. **Hardware Timer Event Pacing (`esp_timer` + Task Notifications)**:
   - Periodic frame deadlines are triggered directly by an `esp_timer` hardware periodic timer (`m_frame_timer`) calling `frameTimerCb`, waking `bcast_tx_task` via FreeRTOS direct task notifications (`vTaskNotifyGiveFromISR`).
   - Completely eliminates CPU spin-wait loops (`esp_rom_delay_us(20)`), preventing IDLE task starvation and reducing SOURCE CPU load from 94% down to ~59%.
4. **Wi-Fi Baseband "TX Done" ISR Semaphore Slot Timing (`s_tx_done_sem`)**:
   - The forward 6-channel serial broadcast sweep is paced directly by the Wi-Fi MAC hardware "TX Done" interrupt callback (`onEspNowSendCb`).
   - The master presentation timestamp `t_tx1_us` is recorded the exact microsecond the baseband MAC queue is 100% empty, dispatching the next channel packet immediately (~1.5 us) after the previous packet leaves the antenna.
   - All 6 channels burst sequentially across the antenna in ~460 us total.
5. **Deterministic Error & Fault Handling**:
   - Deterministic behavior is established for all thinkable baseband fault modes: immediate API rejections, hardware delivery errors, 2.0 ms semaphore timeouts, and baseband lockups.
6. **Dual-Way PTP Microsecond Time Synchronization**:
   - Both forward audio frames and reverse telemetry packets embed microsecond hardware timestamps (`esp_timer_get_time()`). This enables continuous dual-way Precision Time Protocol (PTP) calculation of true Round-Trip Time (RTT) and clock offset, locking all SINK presentation timelines together within +/- 10 microseconds without external time servers.
7. **Dynamic State-Driven Wi-Fi Power Save Management**:
   - Uses `WIFI_PS_MIN_MODEM` during `IDLE` and `SCANNING` states to silence ambient 2.4 GHz packet filtering and RX DMA interrupts, slashing idle CPU load from 24% down to ~2%.
   - Transitions dynamically to `WIFI_PS_NONE` upon entering `PREFILL`, `STREAM`, and `CAST` states, ensuring 100% continuous RF receiver uptime, deterministic TX pacing, and eliminating sleep-induced frame loss.

```
                  +----------------------------------------------------+
                  |              ESP32-S3 SOURCE (Node 16)             |
                  |  - Xtensa Dual-Core @ 240 MHz + Hardware FPU       |
                  |  - Hardware esp_timer Event Pacing (10.0 ms)       |
                  |  - Wi-Fi TX Done ISR Semaphore Sweep (~460 us)     |
                  |  - Dual-Frame LC3 Encoding (t0 + t-1)              |
                  |  - UAC1 USB Audio Speaker (48 kHz 16-bit Stereo)   |
                  |  - Round-Robin Telemetry Collector (1 per 10 ms)   |
                  |  - Dual-Way PTP Master Clock Time Server           |
                  +-------------------------+--------------------------+
                                            |
                 802.11 Layer-2 Broadcast   |  (HT20 MCS3 / OFDM 24.0 Mbps)
                 0 Hardware ACKs, 0 Backoff |  Audio Downlink + Round-Robin Uplink
                                            |
        +-------------------+---------------+-------------------+-------------------+
        | (Ch 0: Left)      | (Ch 1: Right) | (Ch 2: Center)    | (Ch 5: Subwoofer) |
        v                   v               v                   v                   v
+---------------+   +---------------+   +---------------+   +---------------+   +---------------+
| SINK 0 (Left) |   | SINK 1 (Right)|   | SINK 2        |   | SINK 3..4     |   | SINK 5 (Sub)  |
| ESP32-C6-Zero |   | ESP32-C6-Zero |   | ESP32-C6 Dev  |   | ESP32-C6 Mini |   | ESP32-C6-LCD  |
| (Node 23)     |   | (Node 24)     |   | (Node 21)     |   | (Node 25/26)  |   | (Node 20)     |
| MAX98357A DAC |   | MAX98357A DAC |   | MAX98357A DAC |   | MAX98357A DAC |   | MAX98357A DAC |
| 48kHz Stereo  |   | 48kHz Stereo  |   | 48kHz Stereo  |   | 48kHz Stereo  |   | Native 8kHz   |
| PTP Slave     |   | PTP Slave     |   | PTP Slave     |   | PTP Slave     |   | LR4 LP Filter |
+---------------+   +---------------+   +---------------+   +---------------+   +---------------+
```

---

## 2. Hardware Topology & Node Registry

| Node ID | Board / Form Factor | SoC Architecture | Flash / RAM | Factory MAC Address | Default COM Port | Network Role & Audio Routing |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Node 1**  | Seeed Studio XIAO ESP32-S3 Plus + Wio-SX1262 B2B | ESP32-S3 (Xtensa Dual-Core + FPU) | 8 MB / 8 MB PSRAM | TBD | **COM1** | Audio SINK (PCM5102A DAC + TPA3118 Mono Amp, GPIO 3 Mute control). |
| **Node 2**  | Seeed Studio XIAO ESP32-S3 Plus + Wio-SX1262 B2B | ESP32-S3 (Xtensa Dual-Core + FPU) | 8 MB / 8 MB PSRAM | TBD | **COM2** | Audio SINK (PCM5102A DAC + TPA3118 Mono Amp, GPIO 3 Mute control). |
| **Node 3**  | Seeed Studio XIAO ESP32-S3 Plus + Wio-SX1262 B2B | ESP32-S3 (Xtensa Dual-Core + FPU) | 8 MB / 8 MB PSRAM | TBD | **COM3** | Audio SINK (PCM5102A DAC + TPA3118 Mono Amp, GPIO 3 Mute control). |
| **Node 4**  | Seeed Studio XIAO ESP32-S3 Plus + Wio-SX1262 B2B | ESP32-S3 (Xtensa Dual-Core + FPU) | 8 MB / 8 MB PSRAM | `E8:3D:C1:FB:E8:3C` | **COM4** | **Audio SINK (Ch 4: Surround Right)**: PCM5102A DAC + TPA3118 Mono Amp (GPIO 3 Mute control). |
| **Node 5**  | Seeed Studio XIAO ESP32-S3 Plus + Wio-SX1262 B2B | ESP32-S3 (Xtensa Dual-Core + FPU) | 8 MB / 8 MB PSRAM | TBD | **COM5** | Audio SINK (PCM5102A DAC + TPA3118 Mono Amp, GPIO 3 Mute control). |
| **Node 16** | Seeed Studio XIAO ESP32-S3 | ESP32-S3 (Xtensa Dual-Core + FPU) | 4 MB / 512 KB | `E0:72:A1:D8:4C:D0` | **COM16** (Bootloader)<br>**COM116** (Runtime App) | **Audio SOURCE**: UAC1 USB Audio Speaker, LC3 encoder, 6-slot ISR broadcast sweeper, round-robin telemetry collector, PTP master. |
| **Node 20** | Waveshare ESP32-C6-LCD-1.47 | ESP32-C6 (160 MHz RISC-V) | 8 MB / 512 KB | `AC:EB:E6:23:DC:24` | **COM20** | Audio SINK / Subwoofer (Channel 5) / Console Display with ST7789 LCD. |
| **Node 21** | ESP32-C6-WROOM-1 DevKit | ESP32-C6 (160 MHz RISC-V) | 8 MB / 512 KB | `98:A3:16:9D:57:EC` | **COM21** (Flash)<br>**COM121** (App) | Audio SINK (Channel 2: Center) or USB Host Bridge. |
| **Node 23** | Waveshare ESP32-C6-Zero | ESP32-C6 (160 MHz RISC-V) | 8 MB / 512 KB | `B0:A6:04:99:38:44` | **COM23** | **Audio SINK Left (Channel 0)**: MAX98357A I2S DAC, WS2812B RGB indicator. |
| **Node 24** | Waveshare ESP32-C6-Zero | ESP32-C6 (160 MHz RISC-V) | 8 MB / 512 KB | `B0:A6:04:99:18:E4` | **COM24** | **Audio SINK Right (Channel 1)**: MAX98357A I2S DAC, WS2812B RGB indicator. |
| **Node 25** | Heemol ESP32-C6 Mini | ESP32-C6 (160 MHz RISC-V) | 8 MB / 512 KB | `E8:3D:C1:FB:DC:C4` | **COM25** (or COM10) | Audio SINK (Channel 3: Surround Left) / Test Node. |
| **Node 26** | Heemol ESP32-C6 Mini | ESP32-C6 (160 MHz RISC-V) | 8 MB / 512 KB | `98:A3:16:AC:13:38` | **COM26** (or COM22) | Audio SINK (Channel 4: Surround Right) / Test Node. |

### 2.1 Node 16 (SOURCE) USB VID:PID Registry & Operating Modes

Node 16 (Seeed Studio XIAO ESP32-S3) utilizes native USB connected directly to GPIO 19 (`D-`) and GPIO 20 (`D+`). It exposes distinct USB identities depending on boot mode:

| State / Operating Mode | USB Subsystem / Driver | Windows Device Friendly Name | Hardware Instance ID (VID/PID) | Assigned Port / Endpoint | Function / Usage |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Download / Flash** | Native USB-Serial/JTAG ROM Bootloader | `USB Serial Device (COM16)` | `USB\VID_303A&PID_1001` | **COM16** | Flashing firmware via `esptool` / `s3_flash_and_reset.py`. |
| **Download / Flash** | Native USB-OTG ROM Bootloader | `USB Serial Device (COM3)` | `USB\VID_303A&PID_0009` | **COM3** | Alternative ROM DFU/Serial download port. |
| **Application Runtime** | TinyUSB CDC ACM Console | `USB Serial Device (COM116)` | `USB\VID_303A&PID_4002&MI_00` | **COM116** | High-speed ASCII CLI, runtime commands, and 10 Hz telemetry. |
| **Application Runtime** | TinyUSB UAC1 Stereo Audio | `Node16 audio (USB Speaker)` | `USB\VID_303A&PID_4002&MI_02` | USB Audio Output | 48 kHz / 16-bit PCM digital audio stream from Windows host. |

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
  - Native USB D-: GPIO 19
  - Native USB D+: GPIO 20
  - User Status LED: GPIO 21 (Active LOW discrete LED)
  - BOOT Button: GPIO 0

---

## 3. VSAF 3.0 Protocol Specification & Wire Formats (`vsaf_protocol.hpp`)

All network communication uses the **VSAF 3.0 (Variable-rate Synchronized Audio Frame)** container format. All structures are strictly 32-bit word aligned with little-endian byte ordering.

```
 0                   1                   2                   3
 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|          type_id (u16)        |packet_flags(u8)|   seq (u8)   |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                          t_tx1_us                             |
|             (SOURCE Master Presentation Timestamp)            |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                                                               |
|                 Primary Frame t0 (120 Bytes)                  |
|                 (Offset 8, 32-bit Word Aligned)               |
|                                                               |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                                                               |
|                Redundant Frame t-1 (120 Bytes)                |
|                (Offset 128, 32-bit Word Aligned)              |
|                                                               |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
```

### 3.1 8-Byte Word-Aligned Header

1. **`type_id` (`uint16_t`, 2 Bytes)**: Combination magic word and message discriminator:
   - `0x1337`: `VSAF_TYPE_AUDIO` (Forward Audio Broadcast from SOURCE)
   - `0x1338`: `VSAF_TYPE_CONTROL` (Control Frame from SOURCE)
   - `0x1339`: `VSAF_TYPE_SINK_TELEMETRY` (Reverse Telemetry Reply from SINK)
2. **`packet_flags` (`uint8_t`, 1 Byte)**: Bit-packed stream configuration:
   - **Bit 0 (`FRAME_DUR`)**: Frame duration (`0` = 7.5 ms, `1` = 10.0 ms).
   - **Bits 1..3 (`SAMPLE_RATE`)**: Sample rate code (`0` = 8k, `1` = 16k, `2` = 24k, `3` = 32k, `4` = 48k).
   - **Bits 4..6 (`RECEIVER_ID`)**: Target audio channel (`0` = Left, `1` = Right, `2` = Center, `3` = Surround L, `4` = Surround R, `5` = Sub, `7` = Wildcard Broadcast).
   - **Bit 7 (`REQ_ACK`)**: **Request for ACK / Telemetry Reply**:
     - `1`: Target SINK is explicitly commanded by SOURCE to send a telemetry reply in this 10 ms window.
     - `0`: SINK remains silent.
3. **`seq` (`uint8_t`, 1 Byte)**: Monotonically incrementing 8-bit sequence number (0..255).
4. **`t_tx1_us` (`uint32_t`, 4 Bytes)**: Master microsecond presentation timestamp from `esp_timer_get_time()`.

### 3.2 240-Byte Dual-LC3 Payload

- **`data_t0` (120 Bytes, offset 8)**: Primary LC3 compressed audio frame for current timestamp t0.
- **`data_t_prev` (120 Bytes, offset 128)**: Redundant LC3 compressed audio frame for previous timestamp t-1.
- **Total Packet Length**: Strictly **248 Bytes** (fully compliant with the 250-byte ESP-NOW limit).

### 3.3 Round-Robin SINK Telemetry Frame (`vsaf_sink_telemetry_t`)

```
 0                   1                   2                   3
 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|          type_id (0x1339)     |  sink_id (u8) |  ack_seq (u8) |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                          t_tx1_echo                           |
|             (Echoed SOURCE Timestamp in Microseconds)         |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|         t_dwell_us (u16)      | down_rssi(i8) | fifo_fill(u8) |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|          crc16 (u16)          |         reserved (u16)        |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
```

- **Size**: Strictly 16 bytes.
- **`t_dwell_us`**: Turnaround dwell time inside SINK: `t_dwell_us = t_tx2 - t_rx1`.
- **`crc16`**: CCITT-16 checksum protecting arithmetic fields against bit errors.

---

## 4. Hardware Timer Pacing, ISR Slot Timing & Fault Handling

### 4.1 Periodic Frame Heartbeat (`esp_timer` + Task Notifications)

The broadcast engine relies on absolute hardware timer deadlines rather than relative FreeRTOS ticks:
1. `m_frame_timer` is created with `esp_timer_create()` and started periodically at `m_frame_duration_us` (10,000 us).
2. The ISR-level timer callback `frameTimerCb` delivers a direct task notification to the TX task:
   ```cpp
   void IRAM_ATTR EspNowBroadcastEngine::frameTimerCb(void* arg) {
       auto* engine = static_cast<EspNowBroadcastEngine*>(arg);
       if (engine && engine->m_source_tx_task_handle) {
           BaseType_t xHigherPriorityTaskWoken = pdFALSE;
           vTaskNotifyGiveFromISR(engine->m_source_tx_task_handle, &xHigherPriorityTaskWoken);
           if (xHigherPriorityTaskWoken == pdTRUE) {
               portYIELD_FROM_ISR();
           }
       }
   }
   ```
3. `bcast_tx_task` blocks on `ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20))` with zero spin-waiting, eliminating clock drift and CPU starvation.

### 4.2 Wi-Fi Hardware "TX Done" ISR Semaphore Pacing (`s_tx_done_sem`)

The 6-channel broadcast sweep is paced directly by the Wi-Fi baseband hardware interrupt:
1. Before each channel transmission, stale semaphore tokens are purged.
2. The packet is stamped with `t_tx1_us = esp_timer_get_time()` and handed to `esp_now_send()`.
3. `bcast_tx_task` waits on `s_tx_done_sem`:
   ```cpp
   if (xSemaphoreTake(s_tx_done_sem, pdMS_TO_TICKS(2)) == pdTRUE) {
       // ISR fired: MAC queue is 100% empty, dispatch next channel packet immediately (~1.5 us)
   }
   ```
4. All 6 channel packets are transmitted in ~460 us total.

### 4.3 Deterministic Baseband Fault Modes & Error Handling

| Fault Mode | Condition / Trigger | Deterministic Handling | Error Metric |
| :--- | :--- | :--- | :--- |
| **Fault Mode 1: Immediate API Rejection** | `esp_now_send()` returns != `ESP_OK` | **Bypasses semaphore wait** (ISR will not fire). Proceeds immediately to next channel without stalling frame pacing. | `m_tx_mac_error_count` |
| **Fault Mode 2: Hardware MAC Delivery Failure** | `onEspNowSendCb` reports status != `ESP_NOW_SEND_SUCCESS` | Baseband queue is clear (ISR fired). Increments fail counter and continues sweep. | `m_tx_fail_count` |
| **Fault Mode 3: Semaphore Timeout (2.0 ms)** | `xSemaphoreTake(s_tx_done_sem, 2ms)` expires | Baseband hang / heavy RF collision. **Aborts remainder of the 6-channel sweep for the current frame immediately** so the task does not miss the next frame deadline. | `m_tx_timeout_count` |
| **Fault Mode 4: Subsystem Lockup** | 5 or more consecutive frame timeouts (50 ms) | Calls `handleTxSubsystemHang()`, clears stale semaphore tokens, resets consecutive error counter, and logs a system warning. | `m_consecutive_tx_timeouts` |

### 4.4 Dynamic Wi-Fi Power Save Management (`WIFI_PS_MIN_MODEM` vs `WIFI_PS_NONE`)

Standard 802.11 Wi-Fi modem sleep (`WIFI_PS_MIN_MODEM`) coordinates station sleep windows using Access Point (AP) DTIM beacon frames and hardware TSF timers. Because **ESP-NOW is connectionless Layer-2 without an Access Point or beacons**, the Wi-Fi baseband has no coordinated schedule for incoming broadcast frames.

To achieve maximum energy efficiency when inactive without risking audio dropouts during playback, the system dynamically manages Wi-Fi power save modes across state transitions:

| System State | Power Save Mode (`esp_wifi_set_ps`) | Baseband RF Status | Operational Rationale |
| :--- | :--- | :--- | :--- |
| **`IDLE`** / **`OFF`** | **`WIFI_PS_MIN_MODEM`** | Radio cycles into low-power modem sleep when audio receiver is paused or muted. | Eliminates 2.4 GHz ambient packet filtering overhead and RX DMA bus contention, dropping idle CPU load from **24% down to ~2%** on muted SINK nodes. |
| **`SCANNING`** / **`PREFILL`** / **`STREAM`** / **`CAST`** | **`WIFI_PS_NONE`** | Continuous 100% active radio, baseband ADC, and RF PLL. | Required during `SCANNING` to capture connectionless 802.11 ESP-NOW broadcast frames (no AP beacon timing), and during `STREAM`/`CAST` for 100% packet delivery on 10.0 ms cadence with sub-10 microsecond synchronization. |

> [!NOTE]
> **State-Driven Transition Mechanism**:
> When a SINK node is in `IDLE` (muted), `WIFI_PS_MIN_MODEM` conserves power. When unmuted into `SCANNING` (or when active in `STREAM`/`CAST`), `WIFI_PS_NONE` keeps the RF receiver active 100% of the time so that incoming broadcast frames are immediately detected and captured upon channel hopping without sleep-induced packet drops.

---

## 5. LC3 Codec Pipeline & Inter-Core Architecture

## 5.0. Expected LC3 Encoder performance
Comprehensive benchmark of LC3-encoder CPU walltime evaluated at [lc3_encoder_ESP32_S3_rev2.md](../../docs/lc3_encoder_ESP32_S3_rev2.md).

Reference encoder times:
- liblc3, LTPF OFF, IRAM, 48.0 kHz, 10.0 ms, 120 B, 96 kbps: 0.862 ms
- liblc3, LTPF OFF, IRAM, 8.0 kHz, 10.0 ms, 80 B, 64 kbps: 0.334 ms
- liblc3, LTPF ON, FLASH, 48.0 kHz, 10.0 ms, 120 B, 96 kbps: 3.233 ms <-- VERY SLOW!
- liblc3, LTPF ON, FLASH, 8.0 kHz, 10.0 ms, 80 B, 64 kbps: 2.516 ms <-- VERY SLOW!

Reference decoder times:
- esp_audio_codec (FixP), FLASH, 48.0 kHz, 10.0 ms, 120 B, 96 kbps: 0.953 ms
- esp_audio_codec (FixP), IRAM, 48.0 kHz, 7.5 ms, 120 B, 127 kbps: 0.755 ms
- esp_audio_codec (FixP), FLASH, 8.0 kHz, 10.0 ms, 80 B, 64 kbps: 0.279 ms
- esp_audio_codec (FixP), IRAM, 8.0 kHz, 10.0 ms, 80 B, 64 kbps: 0.213 ms


> [!IMPORTANT]
> ### Critical Codec Optimization & Execution Requirements
> - **IRAM Placement Mandatory (`linker.lf`)**: Google `liblc3` (on ESP32-S3) and Espressif fixed-point LC3 (on ESP32-C6) **must** be hosted in internal fast SRAM / IRAM instead of external SPI flash. Empirical hardware benchmarks demonstrate that executing from IRAM runs **3x to 5x faster**: from ~4.5 ms down to **0.8 - 1.5 ms** per 48 kHz encode pass, eliminating SPI flash cache misses and bus contention during 10 ms audio frames!
> - **LTPF (Long-Term Pitch Filter) Disablement**: Disabling LTPF analysis (`lc3_encoder_disable_ltpf()`) for the LC3 encoder boosts encoding throughput by another **~50%**, cutting execution time from ~1.5 ms down to **0.93 - 0.95 ms** per 48 kHz pass (and ~0.35 ms for 8 kHz subwoofer frames)!

### 5.1 ESP32-S3 SOURCE Audio Engine
- **Core 1 (Priority 6, 8KB Stack)**: Runs `audioDspTask`. Vectorized DSP filters + LC3 encoding:
  - High-Pass Filter (HPF @ 100 Hz LR4) for Left and Right channels.
  - Subwoofer Multirate Decimator (D=6 Polyphase FIR Decimator + 100 Hz LR4 IIR).
  - Encodes Right Channel (Ch 1 @ 48 kHz) in ~0.94 ms and Subwoofer (Ch 5/3 @ 8 kHz) in ~0.35 ms.
  - Overall SOURCE CPU load: **38% - 40%** at 240 MHz.
- **Core 0 (Priority 7, 8KB Stack)**: Runs `sourceTxTask`. Encodes Left Channel (Ch 0 @ 48 kHz) in ~0.93 ms, executes the ~4.2 ms 6-channel 802.11 VSAF broadcast sweep, then sleeps until next frame.

### 5.2 ESP32-C6 SINK Audio Engine
- **Core 0 (Priority 6, 16KB Stack)**: Runs `bcast_snk_task`.
  - Jitter FIFO pre-roll cushion: 8 packets (80 ms) during `SCANNING` -> `PREFILL`.
  - Dual-descriptor I2S DMA with preloaded descriptors.
  - Fixed-point LC3 decoder in IRAM executes in ~1.2 ms per 10 ms frame (24% CPU load on 160 MHz RISC-V).
  - Redundancy recovery: when 1 packet is lost (`seq_diff == 2`), recovers t-1 from current packet before t0, maintaining **0 PLC and 0 audio underruns**.

---

## 6. Precision Time Protocol (PTP) Dual-Way Time Synchronization

```
   SOURCE Timeline (Master)                      SINK Timeline (Slave)
   ========================                      =====================
             |                                             |
   t_tx1 ----* (Audio Frame Sent)                          |
             \                                             |
              \  Propagation Time                          |
               \                                           |
                +----------------------------------------> *---- t_rx1 (Audio Frame Received)
                                                           |
                                                           | t_dwell_us
                                                           |
                * <----------------------------------------*---- t_tx2 (Telemetry Sent)
               /
              /  Propagation Time
             /
   t_rx2 ----* (Telemetry Received)                        |
             |                                             |
```

1. **True Net Round-Trip Time (RTT)**:
   ```text
   net_rtt_us = (t_rx2 - t_tx1) - t_dwell_us
   ```
2. **Dual-Way Master-Slave Clock Offset**:
   ```text
   clock_offset_us = ((t_dwell_us) - (t_rx2 - t_tx1)) / 2
   ```
3. SINK timeline jitter is smoothed via Exponential Moving Average (`EMA_offs`) and Rolling Buffer Median (`RB_med`), synchronizing speaker outputs to within **+/- 10 microseconds**.

---

## 7. Interactive CLI Console Commands

Accessible over USB serial on all nodes (**COM116** for SOURCE, **COM23** for SINK Left, **COM24** for SINK Right) at 115,200 baud:

| Command | Target | Description |
| :--- | :--- | :--- |
| `vol <0..100>` | SOURCE / SINK | Set volume percentage (0 = Mute, 100 = 0 dBFS). Broadcasts from SOURCE to all SINKs. |
| `voldb <-96..0>` | SOURCE / SINK | Set volume directly in decibels (-96.0 dB to 0.0 dB). |
| `volu8 <0..255>` | SOURCE / SINK | Set raw 8-bit volume level directly. |
| `volch <ch> <0..255>` | SOURCE | Set volume for a specific channel (0: Left, 1: Right, 5: Sub). |
| `mute` / `unmute` | Both | Smoothly mute or unmute audio using 96 dB/s slew rate. |
| `peer list` | SOURCE | Display registered SINK peers, online status, RTT, and telemetry statistics. |
| `start` / `play` / `cast` | SOURCE | Begin broadcast audio transmission (`CAST` state). |
| `stop` / `pause` | SOURCE | Stop transmission and enter `IDLE` state. |
| `synth [on/off]` | SOURCE | Toggle internal test sine-wave synthesizer on/off. |
| `scan [auto]` / `survey` | SOURCE | Run passive 802.11 RF sniffer survey across channels 1..13. |
| `wifich <1..13>` | SOURCE / SINK | Set Wi-Fi channel manually. |
| `ch <0..5>` | SINK | Dynamically switch receiving audio channel ID for this speaker node. |
| `diag` | Both | Print instantaneous telemetry and hardware status report. |
| `clear` / `cls` | Both | Reset all frame counters, PLC statistics, and error counters. |
| `bootloader` | Both | Trigger software reboot into the factory ROM bootloader for firmware flashing. |
| `reboot` / `reset` | Both | Software restart of the microcontroller. |

---

## 8. Real-Time Telemetry Specification

Nodes emit formatted 1.0-second telemetry heartbeats over USB serial:

### SOURCE Telemetry (Node 16 - COM116)
```text
+=================================================================== ESP32-S3-SOURCE [SOURCE] ===================================================================+
|    CPU      | STATE | NODES  |    WIFI     |  AUDIO dBFS  |     STAGE TIMINGS (ms)       |  SOURCE      PKTS  ACK%  FAIL   TOT  |             ROUND-TRIP NET (us)        |
|  %   C  MHz |       | 012345 | GAIN Ch PHY |   RMS    Pk  |  DSP   Enc1  Enc2  Enc3   TX  |  INPUT        1/s     %   1/s  pkts   |              L_Net       R_Net         |
| 38  44  240 | CAST  | 11OOOO | +3.0 02 HT3 | -33.5 -30.3 | 0.71  0.93  0.94  0.38  4.24|  TONE       601   93%     2    12K |                727        1790         |
| 38  45  240 | CAST  | 11OOOO | +3.0 02 HT3 | -33.5 -30.3 | 0.72  0.93  0.94  0.30  4.30|  TONE       598   93%     2    13K |               1008         920         |
```

### SINK Telemetry (Node 23 / 24 - COM23 / COM24)
```text
+=================================================================== ESP32-C6-23-LEFT [SINK] ====================================================================+
|    CPU      | STATE |  CHAN  |    WIFI     | AUDIO     dBFS      SR   PD    CODEC ms  | AMP dB   PKTS  RED  PLC  DMA   FIFO    |         TIME & SYNCHRONIZATION (ms)    |
|  %   C  MHz |       |        | RSSI Ch PHY |  Enc    RMS   Pk   kHz   ms   Avg   Pk   |  SW  HW   1/s  rec  tot  UDR   UDR     |  Local  Master  EMA_offs RB_med RB_rng |
| 24  47  160 | STRM  | LEFT   |  -48 02 HT3 |  LC3  -33.7 -27.5    48   10  1.22  1.46 |   0  +3    99   11    5    0     0    |  10089   25693   +1560   +1560   2.33  |

+=================================================================== ESP32-C6-24-RIGHT [SINK] ===================================================================+
|    CPU      | STATE |  CHAN  |    WIFI     | AUDIO     dBFS      SR   PD    CODEC ms  | AMP dB   PKTS  RED  PLC  DMA   FIFO    |         TIME & SYNCHRONIZATION (ms)    |
|  %   C  MHz |       |        | RSSI Ch PHY |  Enc    RMS   Pk   kHz   ms   Avg   Pk   |  SW  HW   1/s  rec  tot  UDR   UDR     |  Local  Master  EMA_offs RB_med RB_rng |
| 24  46  160 | STRM  | RGHT   |  -59 02 HT3 |  LC3  -33.6 -28.2    48   10  1.22  1.47 |   0  +3   100   10    4    0     0    |  10076   25685   +1561   +1561   0.65  |
```

---

## 9. Parallel Build, Flash & Automated Deployment

Multi-target parallel compilation and concurrent multi-node flashing are executed with strict CPU worker thread gating (`-j 6` per target) via PowerShell:

```powershell
# Environment Activation (PowerShell)
$env:IDF_TOOLS_PATH = "C:\Users\stefa\.espressif"
. "C:\Users\stefa\OneDrive\Documents\ESP\v6.0.2\esp-idf\export.ps1"

# Parallel Compilation and Concurrent Multi-Node Flashing (Node 16 on COM116, Node 23 on COM23, Node 24 on COM24)
powershell -ExecutionPolicy Bypass -File apps\audio_VSAF_broadcast2\tools\parallel_build_and_flash.ps1
```
