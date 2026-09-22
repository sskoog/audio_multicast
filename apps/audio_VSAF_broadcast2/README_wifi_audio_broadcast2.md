# Audio VSAF Broadcast 2: VSAF 3.0 Multi-Tier UEP Redundancy, Round-Robin Telemetry & Hardware ISR Paced Streaming

## 1. Executive Summary & Architectural Overview

The **`audio_VSAF_broadcast2`** application implements an ultra-low-latency, multi-channel, multi-speaker wireless digital audio distribution system over 802.11 Wi-Fi broadcast semantics (`FF:FF:FF:FF:FF:FF`). It is engineered to distribute up to 6 distinct audio channels synchronously from a single **SOURCE** transmitter (ESP32-S3) to **2 to 6 SINK** speaker nodes (ESP32-C6 / ESP32-S3) with microsecond-level presentation timeline alignment, multi-packet burst-loss resilience, and zero audible dropouts.

### Key Architectural Evolution in VSAF 3.0

1. **VSAF 3.0 Multi-Tier Unequal Error Protection (UEP) Redundancy**:
   - **Satellite Broadcast Packets (`0x1337`, Ch 0..4)**: Each 248-byte packet carries **3 temporal frames**: primary frame $t_0$ (120 bytes, HQ 96 kbps @ 48 kHz), redundant frame $t_{-1}$ (60 bytes, 48 kbps @ 48 kHz), and redundant frame $t_{-2}$ (60 bytes, 48 kbps @ 48 kHz).
   - **Subwoofer Broadcast Packets (`0x1338`, Ch 5)**: Each 248-byte packet carries **4 temporal frames**: primary frame $t_0$ (60 bytes @ 8 kHz) and 3 historical frames $t_{-1}$, $t_{-2}$, $t_{-3}$ (60 bytes each).
   - If up to 2 consecutive RF packets are dropped for satellites (or 3 consecutive packets for subwoofer), the SINK recovers the missing audio frames instantly from subsequent packets, completely eliminating audio dropouts and Soft-ARQ repair windows.
2. **5-Encoder Multi-Rate LC3 Pipeline (IRAM Fast Path)**:
   - 5 independent Google `liblc3` encoder instances hosted in fast IRAM with LTPF (Long-Term Pitch Filter) analysis disabled for microsecond-level execution.
   - Encoding order:
     - **Pass 1 (HQ Satellites)**: Left HP (120B) and Right HP (120B).
     - **Pass 2 (Subwoofer)**: Sub 8k (60B) encoded *before* redundant satellite passes.
     - **Pass 3 (Redundant Satellites)**: Left Red (60B) and Right Red (60B).
3. **Glitch-Free Dynamic Decoder Adaptation & Symmetric JIT Gap Handling**:
   - SINK LC3 decoder dynamically adapts to variable incoming frame sizes (120B HQ vs. 60B Redundancy) on-the-fly while maintaining continuous MDCT synthesis overlap memory.
   - Unified sequence gap logic across the primary FIFO pop loop and the JIT wait loop ensures clean Packet Loss Concealment (PLC) synthesis without pitch jumps or waveform phase glitches ("blurp" sound eliminated).
4. **Commanded Round-Robin SINK Telemetry (1 Reply per 10 ms Window)**:
   - SINK feedback is strictly commanded by the SOURCE via bit 7 (`REQ_ACK`) of `packet_flags`.
   - Exactly one SINK is granted an uplink transmission slot per 10 ms window in round-robin fashion (60 ms full cluster status period across 6 channels), eliminating reverse-path collisions and preserving > 9.5 ms of quiet airtime per frame.
5. **Hardware Timer Event Pacing (`esp_timer` + Task Notifications)**:
   - Periodic frame deadlines are triggered directly by an `esp_timer` hardware periodic timer (`m_frame_timer`) calling `frameTimerCb`, waking `sourceTxTask` via FreeRTOS direct task notifications (`vTaskNotifyGiveFromISR`).
   - Completely eliminates CPU spin-wait loops, preventing IDLE task starvation and reducing SOURCE CPU load to ~38%.
6. **Wi-Fi Baseband "TX Done" ISR Semaphore Slot Timing (`s_tx_done_sem`)**:
   - The forward 6-channel serial broadcast sweep is paced directly by the Wi-Fi MAC hardware "TX Done" interrupt callback (`onEspNowSendCb`).
   - The master presentation timestamp `t_tx1_us` is recorded the exact microsecond the baseband MAC queue is 100% empty, dispatching the next channel packet immediately (~1.5 us) after the previous packet leaves the antenna.
   - All 6 channels burst sequentially across the antenna in ~4.5 ms total.
7. **Dual-Way PTP Microsecond Time Synchronization**:
   - Both forward audio frames and reverse telemetry packets embed microsecond hardware timestamps (`esp_timer_get_time()`). This enables continuous dual-way Precision Time Protocol (PTP) calculation of true Round-Trip Time (RTT) and clock offset, locking all SINK presentation timelines together within +/- 10 microseconds without external time servers.
8. **Dynamic State-Driven Wi-Fi Power Save Management**:
   - Uses `WIFI_PS_MIN_MODEM` during `IDLE` and `SCANNING` states to silence ambient 2.4 GHz packet filtering and RX DMA interrupts, slashing idle CPU load from 24% down to ~2%.
   - Transitions dynamically to `WIFI_PS_NONE` upon entering `PREFILL`, `STREAM`, and `CAST` states, ensuring 100% continuous RF receiver uptime and deterministic TX pacing.

```
                  +----------------------------------------------------+
                  |              ESP32-S3 SOURCE (Node 16)             |
                  |  - Xtensa Dual-Core @ 240 MHz + Hardware FPU       |
                  |  - Hardware esp_timer Event Pacing (10.0 ms)       |
                  |  - 5-Instance Multi-Rate LC3 Encoder (IRAM)        |
                  |  - Pass 1: Left/Right HQ 120B (48 kHz)             |
                  |  - Pass 2: Subwoofer 60B (8 kHz)                   |
                  |  - Pass 3: Left/Right Redundancy 60B (48 kHz)      |
                  |  - Wi-Fi TX Done ISR Semaphore Sweep (~4.5 ms)     |
                  |  - UAC1 USB Audio Speaker (48 kHz 16-bit Stereo)   |
                  |  - Round-Robin Telemetry Collector (1 per 10 ms)   |
                  |  - Dual-Way PTP Master Clock Time Server           |
                  +-------------------------+--------------------------+
                                            |
                 802.11 Layer-2 Broadcast   |  (HT20 MCS0 / MCS3 / OFDM 24 Mbps)
                 0 Hardware ACKs, 0 Backoff |  Audio Downlink + Round-Robin Uplink
                                            |
        +-------------------+---------------+-------------------+-------------------+
        | (Ch 0: Left)      | (Ch 1: Right) | (Ch 2: Center)    | (Ch 5: Subwoofer) |
        v                   v               v                   v                   v
+---------------+   +---------------+   +---------------+   +---------------+   +---------------+
| SINK 0 (Left) |   | SINK 1 (Right)|   | SINK 2 (Ctr)  |   | SINK 3..4     |   | SINK 5 (Sub)  |
| ESP32-C6-Zero |   | ESP32-C6-Zero |   | ESP32-C6 Dev  |   | ESP32-S3 / C6 |   | ESP32-C6-LCD  |
| (Node 23)     |   | (Node 24)     |   | (Node 21)     |   | (Node 4 / 25) |   | (Node 20)     |
| MAX98357A DAC |   | MAX98357A DAC |   | MAX98357A DAC |   | PCM5102A DAC  |   | MAX98357A DAC |
| 48kHz Stereo  |   | 48kHz Stereo  |   | 48kHz Stereo  |   | 48kHz Stereo  |   | Native 8kHz   |
| 3-Tier UEP    |   | 3-Tier UEP    |   | 3-Tier UEP    |   | 3-Tier UEP    |   | 4-Tier UEP    |
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
| **Node 16** | Seeed Studio XIAO ESP32-S3 | ESP32-S3 (Xtensa Dual-Core + FPU) | 4 MB / 512 KB | `E0:72:A1:D8:4C:D0` | **COM16** (Bootloader)<br>**COM116** (Runtime App) | **Audio SOURCE**: UAC1 USB Audio Speaker, 5-instance LC3 encoder, 6-slot ISR broadcast sweeper, round-robin telemetry collector, PTP master. |
| **Node 20** | Waveshare ESP32-C6-LCD-1.47 | ESP32-C6 (160 MHz RISC-V) | 8 MB / 512 KB | `AC:EB:E6:23:DC:24` | **COM20** | **Audio SINK / Subwoofer (Channel 5)**: 8 kHz LC3 decode, ST7789 LCD Console Display. |
| **Node 21** | ESP32-C6-WROOM-1 DevKit | ESP32-C6 (160 MHz RISC-V) | 8 MB / 512 KB | `98:A3:16:9D:57:EC` | **COM21** (Flash)<br>**COM121** (App) | Audio SINK (Channel 2: Center) or USB Host Bridge. |
| **Node 23** | Waveshare ESP32-C6-Zero | ESP32-C6 (160 MHz RISC-V) | 8 MB / 512 KB | `B0:A6:04:99:38:44` | **COM23** | **Audio SINK Left (Channel 0)**: MAX98357A I2S DAC, WS2812B RGB indicator. |
| **Node 24** | Waveshare ESP32-C6-Zero | ESP32-C6 (160 MHz RISC-V) | 8 MB / 512 KB | `B0:A6:04:99:18:E4` | **COM24** | **Audio SINK Right (Channel 1)**: MAX98357A I2S DAC, WS2812B RGB indicator. |
| **Node 25** | Heemol ESP32-C6 Mini | ESP32-C6 (160 MHz RISC-V) | 8 MB / 512 KB | `E8:3D:C1:FB:DC:C4` | **COM25** (or COM10) | Audio SINK (Channel 3: Surround Left) / Test Node. |
| **Node 26** | Heemol ESP32-C6 Mini | ESP32-C6 (160 MHz RISC-V) | 8 MB / 512 KB | `98:A3:16:AC:13:38` | **COM26** (or COM22) | Audio SINK (Channel 4: Surround Right) / Test Node. |

### 2.1 Channel Assignment & Routing Matrix

```text
Channel 0: Left Satellite (Ch 0, High-Pass @ 100 Hz LR4, 48 kHz, Packet Type 0x1337)
Channel 1: Right Satellite (Ch 1, High-Pass @ 100 Hz LR4, 48 kHz, Packet Type 0x1337)
Channel 2: Center Satellite (Ch 2, High-Pass @ 100 Hz LR4, 48 kHz, Packet Type 0x1337)
Channel 3: Surround Left Satellite (Ch 3, High-Pass @ 100 Hz LR4, 48 kHz, Packet Type 0x1337)
Channel 4: Surround Right Satellite (Ch 4, High-Pass @ 100 Hz LR4, 48 kHz, Packet Type 0x1337)
Channel 5: Subwoofer (Ch 5, Polyphase Decimated LP @ 100 Hz LR4, 8 kHz, Packet Type 0x1338)
```

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

### 3.1 Satellite Broadcast Packet (`0x1337`, 3-Tier UEP, 248 Bytes Total)

```
 0                   1                   2                   3
 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|       type_id (0x1337)        |packet_flags(u8)|   seq (u8)   |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                          t_tx1_us                             |
|             (SOURCE Master Presentation Timestamp)            |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                                                               |
|                 Primary Frame t0 (120 Bytes)                  |
|                 (Offset 8, 32-bit Word Aligned)               |
|                 HQ 96 kbps @ 10 ms 48 kHz                     |
|                                                               |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                                                               |
|                Redundant Frame t-1 (60 Bytes)                 |
|                (Offset 128, 32-bit Word Aligned)              |
|                Redundancy 48 kbps @ 10 ms 48 kHz              |
|                                                               |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                                                               |
|                Redundant Frame t-2 (60 Bytes)                 |
|                (Offset 188, 32-bit Word Aligned)              |
|                Redundancy 48 kbps @ 10 ms 48 kHz              |
|                                                               |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
```

### 3.2 Subwoofer Broadcast Packet (`0x1338`, 4-Tier UEP, 248 Bytes Total)

```
 0                   1                   2                   3
 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|       type_id (0x1338)        |packet_flags(u8)|   seq (u8)   |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                          t_tx1_us                             |
|             (SOURCE Master Presentation Timestamp)            |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                 Primary Frame t0 (60 Bytes)                   |
|                 (Offset 8, 32-bit Word Aligned) 8 kHz         |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                Redundant Frame t-1 (60 Bytes)                 |
|                 (Offset 68, 32-bit Word Aligned) 8 kHz        |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                Redundant Frame t-2 (60 Bytes)                 |
|                 (Offset 128, 32-bit Word Aligned) 8 kHz       |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                Redundant Frame t-3 (60 Bytes)                 |
|                 (Offset 188, 32-bit Word Aligned) 8 kHz       |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
```

### 3.3 8-Byte Word-Aligned Header

1. **`type_id` (`uint16_t`, 2 Bytes)**: Combination magic word and message discriminator:
   - `0x1337`: `VSAF_TYPE_AUDIO_SATELLITE` (Satellite Audio Broadcast: Ch 0..4)
   - `0x1338`: `VSAF_TYPE_AUDIO_SUBWOOFER` (Subwoofer Audio Broadcast: Ch 5, 4x60B frames)
   - `0x1350`: `VSAF_TYPE_CONTROL` (Control Frame from SOURCE)
   - `0x1360`: `VSAF_TYPE_SINK_TELEMETRY` (Reverse Telemetry Reply from SINK)
2. **`packet_flags` (`uint8_t`, 1 Byte)**: Bit-packed stream configuration:
   - **Bit 0 (`FRAME_DUR`)**: Frame duration (`1` = 10.0 ms standard, 7.5 ms deprecated).
   - **Bits 1..3 (`SAMPLE_RATE`)**: Sample rate code (`0` = 8k, `1` = 16k, `2` = 24k, `3` = 32k, `4` = 48k).
   - **Bits 4..6 (`RECEIVER_ID`)**: Target audio channel (`0` = Left, `1` = Right, `2` = Center, `3` = Surround Left, `4` = Surround Right, `5` = Subwoofer, `7` = Wildcard Broadcast).
   - **Bit 7 (`REQ_ACK`)**: **Request for ACK / Telemetry Reply**:
     - `1`: Target SINK is explicitly commanded by SOURCE to send a telemetry reply in this 10 ms window.
     - `0`: SINK remains silent.
3. **`seq` (`uint8_t`, 1 Byte)**: Monotonically incrementing 8-bit sequence number (0..255).
4. **`t_tx1_us` (`uint32_t`, 4 Bytes)**: Master microsecond presentation timestamp from `esp_timer_get_time()`.

### 3.4 Round-Robin SINK Telemetry Frame (`vsaf_sink_telemetry_t`)

```
 0                   1                   2                   3
 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|          type_id (0x1360)     |  sink_id (u8) |  ack_seq (u8) |
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
3. `sourceTxTask` blocks on `ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20))` with zero spin-waiting, eliminating clock drift and CPU starvation.

### 4.2 Wi-Fi Hardware "TX Done" ISR Semaphore Pacing (`s_tx_done_sem`)

The 6-channel broadcast sweep is paced directly by the Wi-Fi baseband hardware interrupt:
1. Before each channel transmission, stale semaphore tokens are purged.
2. The packet is stamped with `t_tx1_us = esp_timer_get_time()` and handed to `esp_now_send()`.
3. `sourceTxTask` waits on `s_tx_done_sem`:
   ```cpp
   if (xSemaphoreTake(s_tx_done_sem, pdMS_TO_TICKS(2)) == pdTRUE) {
       // ISR fired: MAC queue is 100% empty, dispatch next channel packet immediately (~1.5 us)
   }
   ```
4. All 6 channel packets are transmitted in ~4.5 ms total.

### 4.3 Deterministic Baseband Fault Modes & Error Handling

| Fault Mode | Condition / Trigger | Deterministic Handling | Error Metric |
| :--- | :--- | :--- | :--- |
| **Fault Mode 1: Immediate API Rejection** | `esp_now_send()` returns != `ESP_OK` | **Bypasses semaphore wait** (ISR will not fire). Proceeds immediately to next channel without stalling frame pacing. | `m_tx_mac_error_count` |
| **Fault Mode 2: Hardware MAC Delivery Failure** | `onEspNowSendCb` reports status != `ESP_NOW_SEND_SUCCESS` | Baseband queue is clear (ISR fired). Increments fail counter and continues sweep. | `m_tx_fail_count` |
| **Fault Mode 3: Semaphore Timeout (2.0 ms)** | `xSemaphoreTake(s_tx_done_sem, 2ms)` expires | Baseband hang / heavy RF collision. **Aborts remainder of the 6-channel sweep for the current frame immediately** so the task does not miss the next frame deadline. | `m_tx_timeout_count` |
| **Fault Mode 4: Subsystem Lockup** | 5 or more consecutive frame timeouts (50 ms) | Calls `handleTxSubsystemHang()`, clears stale semaphore tokens, resets consecutive error counter, and logs a system warning. | `m_consecutive_tx_timeouts` |

---

## 5. LC3 Codec Pipeline & Inter-Core Architecture

### 5.1 5-Encoder LC3 Architecture on ESP32-S3 SOURCE

The audio pipeline runs on **Core 1 (Priority 6)** with hardware FPU vectorization:

```
 Interleaved Stereo PCM In (480 samples @ 48 kHz)
                      |
        +-------------+-------------+
        |                           |
  Left Crossover             Right Crossover
  (100 Hz LR4 HPF)           (100 Hz LR4 HPF)
        |                           |
  m_pcm_left_hp               m_pcm_right_hp
        |                           |
        +-------------+-------------+
                      |
           Polyphase Decimator (D=6)
           + 100 Hz LR4 LPF @ 8 kHz
                      |
                m_pcm_sub_8k
                      |
       ================================
         3-PASS LC3 ENCODING PIPELINE
       ================================
  [Pass 1: HQ Satellites]
    - Enc 0: m_pcm_left_hp  -> 120B (48k HQ)
    - Enc 1: m_pcm_right_hp -> 120B (48k HQ)
  [Pass 2: Subwoofer]
    - Enc 4: m_pcm_sub_8k   ->  60B (8k Mono)
  [Pass 3: Redundant Satellites]
    - Enc 2: m_pcm_left_hp  ->  60B (48k Redundancy)
    - Enc 3: m_pcm_right_hp ->  60B (48k Redundancy)
```

- **Execution Timing**:
  - DSP Filtering: ~0.70 ms
  - Pass 1 (HQ): ~1.78 ms
  - Pass 2 (Sub): ~0.32 ms
  - Pass 3 (Red): ~0.00 ms (cached / parallel)
  - Broadcast TX Sweep: ~4.50 ms
  - **Total Frame Execution**: **~7.30 ms** (well within 10.0 ms frame deadline).

### 5.2 SINK Multi-Packet Burst Recovery & PLC Synthesis

When packets arrive at the SINK:
1. **In-Sequence ($\Delta\text{seq} = 1$)**: Primary frame $t_0$ pushed to FIFO.
2. **Single Packet Drop ($\Delta\text{seq} = 2$)**: Recovers $t_{-1}$ (60B) from redundancy $\rightarrow$ pushes $t_0$ (120B).
3. **Double Packet Drop ($\Delta\text{seq} = 3$)**: Recovers $t_{-2}$ (60B) $\rightarrow$ recovers $t_{-1}$ (60B) $\rightarrow$ pushes $t_0$ (120B).
4. **Triple Packet Drop for Subwoofer ($\Delta\text{seq} = 4$)**: Recovers $t_{-3}$ (60B) $\rightarrow$ $t_{-2}$ (60B) $\rightarrow$ $t_{-1}$ (60B) $\rightarrow$ pushes $t_0$ (60B).
5. **Irrecoverable Burst Drop ($\Delta\text{seq} > 3$)**: SINK synthesizes LC3 PLC frames for intermediate slots via `lc3_decode(..., NULL, pcm_out)`, preserving seamless phase and pitch overlap without "blurp" sound.

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

Accessible over USB serial on all nodes (**COM116** for SOURCE, **COM4** for SINK Surround Right, **COM23** for SINK Left, **COM24** for SINK Right) at 115,200 baud:

| Command | Target | Description |
| :--- | :--- | :--- |
| `vol <0..100>` | SOURCE / SINK | Set volume percentage (0 = Mute, 100 = 0 dBFS). Broadcasts from SOURCE to all SINKs. |
| `voldb <-96..0>` | SOURCE / SINK | Set volume directly in decibels (-96.0 dB to 0.0 dB). |
| `volu8 <0..255>` | SOURCE / SINK | Set raw 8-bit volume level directly. |
| `volch <ch> <0..255>` | SOURCE | Set volume for a specific channel (0: Left, 1: Right, 2: Center, 3: LSurr, 4: RSurr, 5: Sub). |
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
|  %   C  MHz |       | 012345 | GAIN Ch PHY |   RMS    Pk  |  DSP   EncHQ  EncRed   TX    |  INPUT        1/s     %   1/s  pkts   |              L_Net       R_Net         |
| 36  64  240 | CAST  | OOOO1O | +3.0 10 HT0 | -33.3 -30.3 |  0.70   1.78    0.00   4.49  |  TONE       597   90%     2     7K |                  -           -         |
| 38  62  240 | CAST  | OOOO1O | +3.0 10 HT0 | -33.6 -30.3 |  0.70   1.78    0.00   4.81  |  TONE       588   61%     6     5K |                  -           -         |
```

### SINK Telemetry (Node 4 - COM4 - Surround Right)
```text
+=================================================================== ESP32-S3-04-RSUR [SINK] ====================================================================+
|    CPU      | STATE |  CHAN  |    WIFI     | AUDIO     dBFS      SR   PD    CODEC ms  | AMP dB   PKTS  RED  PLC  DMA   FIFO    |         TIME & SYNCHRONIZATION (ms)    |
|  %   C  MHz |       |        | RSSI Ch PHY |  Enc    RMS   Pk   kHz   ms   Avg   Pk   |  SW  HW   1/s  rec  tot  UDR   UDR     |  Local  Master  EMA_offs RB_med RB_rng |
|  6  47  240 | STRM  | RSUR   |  -53 10 HT0 |  LC3  -33.9 -29.4    48   10  0.65  0.73 | -20   -    99   10    0    0     0    |   5992   14535   +8547   +8546   5.47  |
```

### SINK Telemetry (Node 23 / 24 - COM23 / COM24 - Left & Right)
```text
+=================================================================== ESP32-C6-23-LEFT [SINK] ====================================================================+
|    CPU      | STATE |  CHAN  |    WIFI     | AUDIO     dBFS      SR   PD    CODEC ms  | AMP dB   PKTS  RED  PLC  DMA   FIFO    |         TIME & SYNCHRONIZATION (ms)    |
|  %   C  MHz |       |        | RSSI Ch PHY |  Enc    RMS   Pk   kHz   ms   Avg   Pk   |  SW  HW   1/s  rec  tot  UDR   UDR     |  Local  Master  EMA_offs RB_med RB_rng |
| 13  47  160 | STRM  | LEFT   |  -57 10 HT0 |  LC3  -33.8 -28.7    48   10  1.28  1.58 |   0  +3    99    8    0    0     0    |   6056   14615   +8568   +8568  10.48  |

+=================================================================== ESP32-C6-24-RIGHT [SINK] ===================================================================+
|    CPU      | STATE |  CHAN  |    WIFI     | AUDIO     dBFS      SR   PD    CODEC ms  | AMP dB   PKTS  RED  PLC  DMA   FIFO    |         TIME & SYNCHRONIZATION (ms)    |
|  %   C  MHz |       |        | RSSI Ch PHY |  Enc    RMS   Pk   kHz   ms   Avg   Pk   |  SW  HW   1/s  rec  tot  UDR   UDR     |  Local  Master  EMA_offs RB_med RB_rng |
| 12  46  160 | STRM  | RGHT   |  -45 10 HT0 |  LC3  -34.0 -29.2    48   10  1.21  1.34 |   0  +3   100   19    3    0     0    |   6054   14625   +8570   +8571   5.96  |
```

---

## 9. Parallel Build, Flash & Automated Deployment

Multi-target parallel compilation and concurrent multi-node flashing are executed with strict CPU worker thread gating (`-j 6` per target) via PowerShell:

```powershell
# Environment Activation (PowerShell)
$env:IDF_TOOLS_PATH = "C:\Users\stefa\.espressif"
. "C:\Users\stefa\OneDrive\Documents\ESP\v6.0.2\esp-idf\export.ps1"

# Fast automated build and flash for individual roles
powershell -ExecutionPolicy Bypass -File apps\audio_VSAF_broadcast2\tools\build_and_flash.ps1 -Role SOURCE -Port COM116
powershell -ExecutionPolicy Bypass -File apps\audio_VSAF_broadcast2\tools\build_and_flash.ps1 -Role SINK -Port COM4
```
