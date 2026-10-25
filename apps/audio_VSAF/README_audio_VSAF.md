# audio_VSAF: VSAF 3.0 Multi-Channel Wi-Fi Audio Broadcast

## 1. Overview

The **`audio_VSAF`** application distributes up to 6 audio channels (5.1) from one **SOURCE** (ESP32-S3, Node 16) to up to 6 **SINK** speaker nodes (ESP32-S3 / ESP32-C6) over 802.11 ESP-NOW broadcast (`FF:FF:FF:FF:FF:FF`), with LC3 coding, multi-tier redundancy and a strict 10 ms frame cadence. Project goals and numeric targets (latency, inter-speaker sync, zero glitches per hour) are in [GOALS.md](../../GOALS.md).

Main building blocks:

1. **VSAF 3.0 multi-tier Unequal Error Protection (UEP)**: every 248-byte satellite packet (`0x1337`, ch 0..4) carries the current frame at HQ (120 B, 96 kbps @ 48 kHz) plus the two previous frames at MQ (60 B, 48 kbps). Subwoofer packets (`0x1338`, ch 5) carry 4 frames of 60 B @ 8 kHz. Up to 2 lost satellite packets in a row (3 for the sub) are recovered from later packets without concealment.
2. **Dual-core SOURCE pipeline** (section 4): core 0 runs USB input + DSP and encodes, core 1 encodes and owns the radio. Encode work is a lock-free job queue shared by both cores. A dedicated RF task keeps exactly one packet in flight, stamps `t_tx1_us` right before each `esp_now_send()` and never transmits after the 9.5 ms frame deadline.
3. **Dual-rate LC3 encoding (`liblc3_dual`)** (section 5): HQ and MQ frames of a channel share one LC3 analysis; MQ only adds a quantization pass. Output is bit-exact with two separate encoders.
4. **Commanded round-robin SINK telemetry**: bit 7 (`REQ_ACK`) of `packet_flags` grants exactly one SINK an uplink reply per 10 ms frame; that packet is sent last in the burst so the reply never collides with the downlink.
5. **Self-healing radio**: TX-done timeouts, ESP-NOW queue exhaustion and implausible completions are detected and repaired by re-initialising ESP-NOW (section 4.4). Channel and PHY changes are applied only at frame boundaries with the radio idle.
6. **Fixed channel 13 at boot** (`CONFIG_VSAF_PREFERRED_WIFI_CHANNEL`), boot survey off by default (`CONFIG_VSAF_BOOT_CHANNEL_SCAN`). SINKs start their search on the same channel and stream ~0.35-0.5 s after power-on. Default PHY HT20 MCS3, TX power 10 dBm on all nodes (`CONFIG_VSAF_TX_POWER_QDBM`).
7. **Hardware timer pacing**: the 10 ms frame tick is an `esp_timer` periodic callback with direct task notifications, never FreeRTOS tick delays.

```
                  +-----------------------------------------------------+
                  |              ESP32-S3 SOURCE (Node 16)              |
                  |  UAC1 USB speaker (48 kHz 16-bit stereo)            |
                  |  core 0: USB read + DSP (LR4 crossover, sub         |
                  |          decimation) + LC3 jobs                     |
                  |  core 1: LC3 jobs + RF task (1 packet in flight)    |
                  |  11 LC3 jobs/frame: sub, 5x HQ (+shared analysis),  |
                  |          5x MQ (quantize only)                      |
                  |  Round-robin telemetry collector, RTT / offset      |
                  +-------------------------+---------------------------+
                                            |
                 802.11 broadcast, ch 13    |  HT20 MCS3 (default), 10 dBm
                 no ACKs, 6 packets / 10 ms |  downlink + 1 commanded uplink reply
                                            |
        +-------------------+---------------+-------------------+-------------------+
        | (Ch 0: Left)      | (Ch 1: Right) | (Ch 2: Center)    | (Ch 5: Subwoofer) |
        v                   v               v                   v                   v
+---------------+   +---------------+   +---------------+   +---------------+   +---------------+
| SINK 0 (Left) |   | SINK 1 (Right)|   | SINK 2 (Ctr)  |   | SINK 3..4     |   | SINK 5 (Sub)  |
| XIAO S3 Plus  |   | XIAO S3 Plus  |   | ESP32-C6-Zero |   | Heemol C6     |   | ESP32-C6-Zero |
| (Node 4)      |   | (Node 5)      |   | (Node 23)     |   | (Node 25 / 26)|   | (Node 24 / 20)|
| PCM5102A DAC  |   | PCM5102A DAC  |   | MAX98357A DAC |   | MAX98357A DAC |   | MAX98357A DAC |
| 48kHz 24-bit  |   | 48kHz 24-bit  |   | 48kHz 16-bit  |   | 48kHz 16-bit  |   | Native 8kHz   |
| 3-Tier UEP    |   | 3-Tier UEP    |   | 3-Tier UEP    |   | 3-Tier UEP    |   | 4-Tier UEP    |
+---------------+   +---------------+   +---------------+   +---------------+   +---------------+
```

---

## 2. Hardware Topology & Node Registry

| Node ID | Board / Form Factor | SoC Architecture | Flash / RAM | Factory MAC Address | Default COM Port | Network Role, DAC & Aux Hardware Configuration |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Node 1**  | Seeed Studio XIAO ESP32-S3 Plus + Wio-SX1262 B2B | ESP32-S3 (Xtensa Dual-Core + FPU) | 8 MB / 8 MB PSRAM | TBD | **COM1** | Audio SINK (Ch 0: Left / Spare): PCM5102A 24-bit I2S DAC, TPA3118 Mono Power Amp (GPIO 3 Mute control, -20.0 dB post-gain), User LED (GPIO 21), Wio-SX1262 B2B LoRa header. |
| **Node 2**  | Seeed Studio XIAO ESP32-S3 Plus + Wio-SX1262 B2B | ESP32-S3 (Xtensa Dual-Core + FPU) | 8 MB / 8 MB PSRAM | TBD | **COM2** | Audio SINK (Ch 1: Right / Spare): PCM5102A 24-bit I2S DAC, TPA3118 Mono Power Amp (GPIO 3 Mute control, -20.0 dB post-gain), User LED (GPIO 21), Wio-SX1262 B2B LoRa header. |
| **Node 3**  | Seeed Studio XIAO ESP32-S3 Plus + Wio-SX1262 B2B | ESP32-S3 (Xtensa Dual-Core + FPU) | 8 MB / 8 MB PSRAM | TBD | **COM3** | Audio SINK (Ch 2: Center / Spare): PCM5102A 24-bit I2S DAC, TPA3118 Mono Power Amp (GPIO 3 Mute control, -20.0 dB post-gain), User LED (GPIO 21), Wio-SX1262 B2B LoRa header. |
| **Node 4**  | Seeed Studio XIAO ESP32-S3 Plus + Wio-SX1262 B2B | ESP32-S3 (Xtensa Dual-Core + FPU) | 8 MB / 8 MB PSRAM | `E8:3D:C1:FB:E8:3C` | **COM4** | **Audio SINK (Ch 0: Left)**: PCM5102A 24-bit I2S DAC, TPA3118 Mono Power Amp (GPIO 3 Mute control, -20.0 dB post-gain), User LED (GPIO 21), Wio-SX1262 B2B LoRa header. |
| **Node 5**  | Seeed Studio XIAO ESP32-S3 Plus + Wio-SX1262 B2B | ESP32-S3 (Xtensa Dual-Core + FPU) | 8 MB / 8 MB PSRAM | `E8:3D:C1:FC:8B:50` | **COM5** | **Audio SINK (Ch 1: Right)**: PCM5102A 24-bit I2S DAC, TPA3118 Mono Power Amp (GPIO 3 Mute control, -9.0 dB post-gain), User LED (GPIO 21), Wio-SX1262 B2B LoRa header. |
| **Node 16** | Seeed Studio XIAO ESP32-S3 | ESP32-S3 (Xtensa Dual-Core + FPU) | 4 MB / 512 KB (no PSRAM) | `E0:72:A1:D8:4C:D0` | **COM16** (Bootloader)<br>**COM116** (Runtime App) | **Audio SOURCE**: UAC1 USB Audio Speaker (48 kHz 16-bit Stereo), dual-core DSP + LC3 pipeline (5 satellites dual-rate + sub), RF task, round-robin telemetry collector, User LED (GPIO 21). |
| **Node 20** | Waveshare ESP32-C6-LCD-1.47 | ESP32-C6 (160 MHz RISC-V) | 8 MB / 512 KB | `AC:EB:E6:23:DC:24` | **COM20** | **Audio SINK / Subwoofer (Ch 5)**: MAX98357A I2S Mono DAC (+3 dB gain), ST7789 1.47" LCD Console Display, WS2812B RGB LED (GPIO 8), 8 kHz decimated LC3 decoding. |
| **Node 21** | ESP32-C6-WROOM-1 DevKit | ESP32-C6 (160 MHz RISC-V) | 8 MB / 512 KB | `98:A3:16:9D:57:EC` | **COM21** (Flash)<br>**COM121** (App) | Audio SINK (Ch 2: Center) / USB Host Bridge: MAX98357A I2S DAC (+3 dB gain), CP2102N UART bridge + Native USB. |
| **Node 23** | Waveshare ESP32-C6-Zero | ESP32-C6 (160 MHz RISC-V) | 8 MB / 512 KB | `B0:A6:04:99:38:44` | **COM23** | **Audio SINK (Ch 2: Center)**: MAX98357A I2S Mono DAC (+3 dB hardware gain), WS2812B RGB LED (GPIO 8). |
| **Node 24** | Waveshare ESP32-C6-Zero | ESP32-C6 (160 MHz RISC-V) | 8 MB / 512 KB | `B0:A6:04:99:18:E4` | **COM24** | **Audio SINK (Ch 5: Subwoofer)**: MAX98357A I2S Mono DAC (+3 dB hardware gain), WS2812B RGB LED (GPIO 8), 8 kHz polyphase decimated LC3 decoding. |
| **Node 25** | Heemol ESP32-C6 Mini | ESP32-C6 (160 MHz RISC-V) | 8 MB / 512 KB | `E8:3D:C1:FB:DC:C4` | **COM25** (or COM10) | Audio SINK (Ch 3: Surround Left) / Test Node: MAX98357A I2S DAC (+3 dB gain), discrete User LED (GPIO 15). |
| **Node 26** | Heemol ESP32-C6 Mini | ESP32-C6 (160 MHz RISC-V) | 8 MB / 512 KB | `98:A3:16:AC:13:38` | **COM26** (or COM22) | Audio SINK (Ch 4: Surround Right) / Test Node: MAX98357A I2S DAC (+3 dB gain), discrete User LED (GPIO 15). |

### 2.1 Channel Assignment & Routing Matrix

```text
Channel 0: Left Satellite (Ch 0, High-Pass @ 100 Hz LR4, 48 kHz, Packet Type 0x1337)
Channel 1: Right Satellite (Ch 1, High-Pass @ 100 Hz LR4, 48 kHz, Packet Type 0x1337)
Channel 2: Center Satellite (Ch 2, High-Pass @ 100 Hz LR4, 48 kHz, Packet Type 0x1337)
Channel 3: Surround Left Satellite (Ch 3, High-Pass @ 100 Hz LR4, 48 kHz, Packet Type 0x1337)
Channel 4: Surround Right Satellite (Ch 4, High-Pass @ 100 Hz LR4, 48 kHz, Packet Type 0x1337)
Channel 5: Subwoofer (Ch 5, Polyphase Decimated LP @ 100 Hz LR4, 8 kHz, Packet Type 0x1338)
```

Until the spatial upmix exists (GOALS.md priority 2), the SOURCE fills ch 2 (Center) and ch 3 (Surround L) with the left signal and ch 4 (Surround R) with the right signal. All 5 satellite streams are encoded independently, so the CPU load is already that of real 5.1.

### 2.2 Pinout & Auxiliary Hardware Wiring Reference

- **Node 1, 2, 3, 4, 5 (SINK PCM5102A DAC + TPA3118 Amp on XIAO S3 Plus)**:
  - BCK: D5 / GPIO 6
  - LCK (WS): D3 / GPIO 4
  - DIN (DOUT): D4 / GPIO 5
  - Power Amp Mute / SD: D2 / GPIO 3 (Active LOW Mute, High-Z Unmute)
  - User Status LED: GPIO 21 (Active LOW discrete LED)
  - BOOT Button: GPIO 0
  - Wio-SX1262 LoRa B2B Pins: GPIO 38..42, 7..9 (Reserved)
- **Node 23 & Node 24 (SINK MAX98357A DACs on Waveshare ESP32-C6-Zero)**:
  - BCLK: GPIO 2
  - LRCLK (WS): GPIO 3
  - DIN (DOUT): GPIO 1
  - WS2812B Status LED: GPIO 8
  - BOOT Button: GPIO 9
- **Node 20 (Waveshare ESP32-C6-LCD-1.47)**:
  - BCLK: GPIO 2
  - LRCLK (WS): GPIO 3
  - DIN (DOUT): GPIO 1
  - ST7789 LCD: SPI (MOSI: GPIO 7, SCLK: GPIO 6, CS: GPIO 14, DC: GPIO 15, RST: GPIO 21, BL: GPIO 22)
  - WS2812B Status LED: GPIO 8
  - BOOT Button: GPIO 9
- **Node 21 (ESP32-C6-WROOM-1 DevKit)**:
  - BCLK: GPIO 2
  - LRCLK (WS): GPIO 3
  - DIN (DOUT): GPIO 1
  - BOOT Button: GPIO 9
- **Node 25 & Node 26 (Heemol ESP32-C6 Mini)**:
  - BCLK: GPIO 2
  - LRCLK (WS): GPIO 3
  - DIN (DOUT): GPIO 1
  - User LED: GPIO 15
  - BOOT Button: GPIO 9
- **Node 16 (SOURCE Broadcaster on XIAO ESP32-S3)**:
  - Native USB D-: GPIO 19
  - Native USB D+: GPIO 20
  - User Status LED: GPIO 21 (Active LOW discrete LED)
  - BOOT Button: GPIO 0

---

## 3. VSAF 3.0 Protocol Specification & Wire Formats (`vsaf_protocol.hpp`)

All network communication uses the **VSAF 3.0 (Variable-rate Synchronized Audio Frame)** container format. All structures are strictly 32-bit word aligned with little-endian byte ordering. The source of truth is [main/vsaf_protocol.hpp](main/vsaf_protocol.hpp) (`static_assert`s pin the sizes).

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
4. **`t_tx1_us` (`uint32_t`, 4 Bytes)**: SOURCE `esp_timer_get_time()` in microseconds, written immediately before the packet is handed to `esp_now_send()` (so it excludes encode time and queueing inside the SOURCE).

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

## 4. SOURCE Real-Time Pipeline (`vsaf_source_pipeline.cpp`)

The 10 ms frame is the master metronome (GOALS.md): every task must finish inside it, including putting all 6 packets on air. The SOURCE splits one frame across both ESP32-S3 cores:

| Task | Core | Prio | Role |
| :--- | :--- | :--- | :--- |
| `esp_timer` callback `frameTimerCb` | 0 | 22 | Frame tick: notifies `src_dsp0` and `src_rf` (`SRC_BIT_TICK`) |
| `src_dsp0` | 0 | 5 | USB read (or tone synth) -> LR4 crossover -> sub decimation -> publish frame -> encode jobs |
| `src_enc1` | 1 | 5 | Encode jobs of the published frame |
| `src_rf` | 1 | 7 | Radio owner: radio health, deferred channel / PHY changes, one packet in flight |
| Wi-Fi driver | 0 | 23 | ESP-IDF; calls the ESP-NOW send callback (TX-done) |

### 4.1 Frame timeline

```
 t = 0 ms     frame tick (esp_timer)
 core 0 :  [USB+DSP ~0.45]  publish  [jobs: SUB, HQ, HQ, ... MQ ...]           idle
 core 1 :                   [jobs: HQ, HQ, ... MQ ...]                          idle
 radio  :        [sub][HQ][HQ][HQ][HQ][HQ(ack ch, last)]          | 9.5 ms deadline
                 ^ each packet: t_tx1_us stamped right before esp_now_send()
 measured (music over USB, ch 13, HT3): all jobs done ~5.1 ms, last packet on air ~5.1 ms
```

### 4.2 Job queue (lock-free, both cores)

Per frame `srcPrepareFrame()` fills 11 jobs, claimed in order by whichever core is free through an atomic cursor (`next_job.fetch_add`):

1. `SUB`: 8 kHz sub encode, builds the sub packet (t0 + 3 history frames).
2. `HQ` x5 (satellite order rotates every frame): shared LC3 analysis + 120 B frame (`lc3_encode_dual_begin`), builds the satellite packet with the MQ history of frames N-1 and N-2, marks it ready.
3. `MQ` x5 (same rotation): 60 B frame from the HQ job's analysis (`lc3_encode_dual_end`), stored in a 3-slot history tagged by frame number. An MQ job whose HQ phase is still running on the other core waits for it (counted as `MQ waits`).

PCM and packet buffers are double-buffered by `frame_no & 1`. Before publishing frame N+1, core 0 waits for jobs of frame N still in flight and cancels any it never started (counted). Missing MQ history is replaced by the other slot so a packet always carries valid LC3 frames (counted as `MQ missing`). No allocation happens in steady state.

### 4.3 RF task (`src_rf`)

- Event-driven, no polling. It sleeps on a task notification. It is woken by a TX-done, by a packet becoming ready (encoders notify only while `src_rf` is idle and waiting, via a store/load flag handshake that cannot lose a wake-up), or by the frame tick. The only timed wake-up is the release of a held reply-request packet.
- Exactly one packet in flight: the next ready packet is submitted as soon as the previous TX-done arrives. Nothing piles up in the ESP-NOW queue, so `ESP_ERR_ESPNOW_NO_MEM` cannot build up.
- `t_tx1_us` is written into the packet immediately before `esp_now_send()`.
- The packet carrying `REQ_ACK` goes last (its SINK replies immediately), unless the others are later than 8 ms.
- No submit after **9.5 ms** into the frame; unsent packets are counted as deadline skips.
- Measured: TX-done callback -> `src_rf` running ~14 us average. Submitting the next packet from inside the send callback was evaluated and rejected: it would save under 0.1 ms per frame and move `esp_now_send()` into the Wi-Fi task.

### 4.4 Fault handling and self-healing TX

| Condition | Handling | Counter |
| :--- | :--- | :--- |
| `esp_now_send()` returns an error | Packet dropped, next one tried; 30 consecutive errors -> ESP-NOW re-init | `send_err`, `no_mem` (`txstat`) |
| TX-done status != success | Counted, burst continues | `tx_status_fail` |
| No TX-done within 2.5 ms | Packet given up, burst continues; 5 in a row -> `handleTxSubsystemHang()` (log rate limited to 1/s) | `tx_done_timeouts` |
| No successful completion for 0.5 s while casting | `recoverEspNowTx()`: `esp_now_deinit()` + init + peer re-add | `recoveries` |
| All completions faster than 60 us for 0.5 s (wedged driver) | `recoverEspNowTx()` | `recoveries` |
| Channel / PHY change requested (`wifich`, `phy`) | Deferred to the next frame boundary with no packet in flight (`applyPendingRadioChanges`) | - |

`txfault` (console) breaks the TX path on purpose to test this recovery; it heals in well under 1 s.

---

## 5. LC3 Codec Pipeline

### 5.1 SOURCE encoder bank and `liblc3_dual`

The SOURCE uses [components/liblc3_dual](../../components/liblc3_dual), a fork of Google liblc3 (the original [components/liblc3](../../components/liblc3) stays untouched and is excluded from this app in `CMakeLists.txt`, since both export the same symbols). The fork adds dual-rate encoding:

- `lc3_encode_dual()` / `lc3_encode_dual_begin()` + `lc3_encode_dual_end()` encode one PCM frame at two sizes with a single analysis. The bitrate-independent stages (input load, LTPF, MDCT, band energies, bandwidth detection, and SNS + TNS whenever both rates take the same attack-detector and TNS-weighting decisions) run once; only quantization and the bitstream writer run per rate.
- The second rate needs only a small state (`lc3_encoder_rate_t`: attack detector, quantizer offset, pending side data, spectrum; ~2 KB instead of a 5.4 KB encoder).
- Output is **bit-exact** with two independent encoders (verified on target with `encbench`: 0 differing frames over silence, sine, noise, attack bursts and live music).

Encoder bank in `lc3_codec.cpp` (all state in internal SRAM, code in IRAM via `CONFIG_LC3_CODE_IN_IRAM`, LTPF disabled):

| Slot | Content |
| :--- | :--- |
| 0..4 | Satellite ch 0..4 HQ encoder (48 kHz, 120 B) |
| 5..9 | MQ rate state paired with encoder 0..4 (60 B) |
| 10 | Subwoofer encoder (8 kHz, 60 B) |

Pure CPU cost per 10 ms frame (`encbench`, interrupts masked, 240 MHz, live music): HQ alone 0.99 ms, MQ alone 0.81 ms, dual HQ+MQ 1.49 ms (silence 0.73 ms), sub 0.32 ms. LC3plus (2.5 / 5 ms frames, 96 kHz HR) is compiled out (`CONFIG_LC3_ENABLE_PLUS_MODES=n`), which saves ~36 KB of internal RAM.

### 5.2 SINK Multi-Packet Burst Recovery & PLC Synthesis

When packets arrive at the SINK:
1. **In-Sequence (`Delta_seq = 1`)**: Primary frame `t0` pushed to FIFO.
2. **Single Packet Drop (`Delta_seq = 2`)**: Recovers `t-1` (60B) from redundancy -> pushes `t0` (120B).
3. **Double Packet Drop (`Delta_seq = 3`)**: Recovers `t-2` (60B) -> recovers `t-1` (60B) -> pushes `t0` (120B).
4. **Triple Packet Drop for Subwoofer (`Delta_seq = 4`)**: Recovers `t-3` (60B) -> `t-2` (60B) -> `t-1` (60B) -> pushes `t0` (60B).
5. **Irrecoverable Burst Drop (`Delta_seq > 3`)**: SINK synthesizes LC3 PLC frames for intermediate slots via `lc3_decode(..., NULL, pcm_out)`. Every PLC frame counts as a glitch in the GOALS.md reliability target.

The SINK decoder adapts to the frame size per frame (120 B HQ vs 60 B redundancy) while keeping its MDCT overlap state. Playout is FIFO driven: a SINK leaves `SCANNING` after a prefill of 5 frames (50 ms: 2 in DMA, 3 in the FIFO).

### 5.3 SINK clock-drift compensation (single-sample insert / drop)

Each SINK plays out on its own crystal, the SOURCE produces frames on its own. Without correction a SINK whose clock runs a few ppm fast slowly empties its FIFO (and a slow one fills it). Once the cushion is gone, every late or lost packet that redundancy would have repaired turns into a FIFO underrun + PLC (seen in the first 15-minute soak test, section 10).

`runSinkLoop()` therefore holds the **total buffered audio** constant: frames in the FIFO plus samples still queued in the I2S DMA.
1. After every frame pop: buffered = FIFO frames x 480 + (samples written to I2S - samples played). Samples played come from the DMA `on_sent` interrupt (completed 480-sample descriptors, plus the part of the current descriptor since the last completion), so the measure is re-anchored by hardware every 10 ms and does not depend on an assumed sample rate. If a DMA underrun plays zero-filled descriptors, the written count is rebased.
2. The value is filtered with an EMA in Q16 fixed point (time constant 256 frames, ~2.6 s).
3. 3 s after entering `STREAM` the filtered value becomes the target (at least 2 frames; typically ~4: 2 in the FIFO, ~2 in the DMA).
4. If the filtered value is more than 0.5 frame below the target (SINK clock fast), one interpolated sample is inserted into the decoded frame (481 samples played); more than 0.5 frame above (SINK clock slow), two neighbours are merged (479 samples). The edit point is the smoothest spot of the frame (smallest sample-to-sample step), so a single sample in 480 is inaudible.
5. At most one correction per 16 frames, i.e. up to ~130 ppm of correction range; ESP32 crystals differ by tens of ppm.
6. Every (re-)entry into `STREAM` (power-on, SOURCE reboot, outage) resets the filter, relearns the target and restarts the `ppm` statistics.

`i2s_channel_write()` streams across the two DMA descriptors, so 479 / 481-sample writes need no change to the I2S setup. They do shift the write phase against the descriptors, which moves up to a whole frame between the DMA queue and the FIFO. That is why the controller must not look at the FIFO depth alone: the first version did, and every single inserted sample made the FIFO depth jump by one frame (section 10). The net number of corrections divided by the samples played is the SINK's clock offset against the SOURCE, printed in the heartbeat as `ppm` (positive = SINK clock fast). `driftcorr 0` disables the correction to observe the free-running drift.

---

## 6. Round-Trip Time and Clock Offset

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

1. **Net round-trip time**: `net_rtt_us = (t_rx2 - t_tx1) - t_dwell_us`
2. **Master-slave clock offset**: `clock_offset_us = ((t_dwell_us) - (t_rx2 - t_tx1)) / 2`
3. The SINK smooths the offset with an EMA (`EMA_offs`) and a rolling median (`RB_med`).

**Status:** these values are telemetry today. SINK playout does not yet use them (it is FIFO driven). The drift compensation of section 5.3 keeps every SINK's FIFO depth (and thus its latency) constant, but it does not align the SINKs to a common presentation time. The inter-speaker sync target (GOALS.md: within a few samples, held over long sessions) has not been measured yet; earlier claims of +/- 10 us alignment are unverified.

---

## 7. Interactive CLI Console Commands

USB serial on all nodes (**COM116** for the SOURCE, **COM4** / **COM5** / **COM23** / **COM24** for the SINKs), 115200 baud. Open ports with DTR/RTS low (see the `esp32-usb-serial-debug` skill).

How each node reads its console:
- **SOURCE (S3, Node 16):** TinyUSB CDC (COM116).
- **S3 SINKs (Nodes 4, 5):** the USB-Serial-JTAG port is configured as a secondary, output-only console (the S3 SOURCE needs the USB PHY for TinyUSB), so the CLI task reads the USB-Serial-JTAG RX FIFO directly. Before 2026-10-25 these SINKs ignored all console input.
- **C6 SINKs:** USB-Serial-JTAG primary console (`stdin`), non-blocking.
- On all nodes, output written while no program has the port open is dropped once the 64-byte USB FIFO is full, so the first line after opening a port can start mid-line. That is expected, not a stall.

### 7.1 Audio and network

| Command | Target | Description |
| :--- | :--- | :--- |
| `vol <0..100>` | SOURCE / SINK | Set volume percentage (0 = Mute, 100 = 0 dBFS). Broadcasts from SOURCE to all SINKs. |
| `voldb <-96..0>` | SOURCE / SINK | Set volume directly in decibels. |
| `volu8 <0..255>` | SOURCE / SINK | Set raw 8-bit volume level. |
| `volch <ch> <0..255>` | SOURCE | Set volume for one channel (0: Left, 1: Right, 2: Center, 3: LSurr, 4: RSurr, 5: Sub). |
| `mute` / `unmute` | Both | Smooth mute / unmute (96 dB/s slew). |
| `postgain <dB>` | SINK | Software post-gain (PCM5102A calibration, see AGENTS.md). |
| `peer list` / `peers` | SOURCE | Registered SINK peers, online status, RTT and telemetry. |
| `start` / `play` / `cast` | SOURCE | Begin broadcast (`CAST`). |
| `stop` / `pause` | SOURCE | Stop transmission (`IDLE`). |
| `synth [on/off]` / `tone` | SOURCE | Internal test synthesizer instead of USB audio. |
| `sr <8k..48k>` | SOURCE | Satellite sample rate. |
| `scan` / `survey` | SOURCE | Passive channel survey 1..13 (refused while casting). |
| `wifich <1..13>` / `channel` | SOURCE / SINK | Set Wi-Fi channel (SOURCE: applied at the next frame boundary). |
| `phy <mode>` | SOURCE | TX PHY rate (default HT20 MCS3), applied at the next frame boundary. |
| `txp` / `txpower <dBm>` | Both | TX power in dBm (2..20 or `max`; default 10 dBm). The reply shows ESP-IDF's readback, taken once right after the change. The radio quantizes the setting to fixed levels and no public API reports the applied level (section 10). The SOURCE heartbeat `GAIN` column shows the same cached value; nothing is polled. |
| `ch <0..5>` | SINK | Switch the received audio channel. |
| `driftcorr 0\|1` | SINK | Clock-drift compensation off / on (default on, section 5.3). |
| `clear` / `cls` | Both | Reset frame, PLC and error counters. |
| `bootloader` / `download` | Both | Reboot into the ROM bootloader for flashing (Node 16: also triggered by the 1200-baud touch). |
| `reboot` / `reset` | Both | Software restart. |

### 7.2 Diagnostics on every node

| Command | Description |
| :--- | :--- |
| `txstat` | Channel (set / radio), PHY, TX power setting, recoveries, send errors (incl. `no_mem`, last error), TX-done timeouts, status failures, deadline drops. |
| `usjdiag` | USB-Serial-JTAG console health: console writes accepted / rejected (EIO = firmware believes no host is connected), bytes received, seconds connected / disconnected, connection changes. Kept in RTC memory: after a chip reset (not a power-on) the `previous session` line shows the counters of the session before the reset, e.g. of a stalled console. |

### 7.3 SOURCE timing and debug

| Command | Description |
| :--- | :--- |
| `timing` | Per-stage table of the last 1 s window (avg / max / count): DSP, LC3 HQ / MQ / sub job times, TX submit -> TX-done, TX-done -> `src_rf` wake, `esp_now_send()` CPU, radio idle gap between packets, first submit, TX end, all jobs done, encode CPU per core; plus overruns, cancelled jobs, MQ missing / waits, deadline skips, TX timeouts, `src_rf` wakes and active time per frame, heap. |
| `tasks` | CPU per FreeRTOS task over 1 s (run-time stats; interrupt time counts against the interrupted task). |
| `mem` | Internal heap free / min / largest block, pipeline task creation report, stack high-water marks. |
| `log` | Dump the 4 KB RAM ring of recent `ESP_LOG` output (boot messages are kept even before USB enumerates). |
| `encbench [0\|1] [irq]` | LC3 micro-benchmark on core 0 or 1 in a top-priority task: HQ, MQ, sub and dual-rate encode cost per test signal, plus the dual-rate bit-exactness check. `irq` leaves interrupts enabled (difference = interrupt load). |
| `dspload <us>` | Add a synthetic busy-wait to the DSP stage (headroom test for the future spatial upmix, 0..9000 us). |
| `enccores 1\|2` | Debug: encode on core 0 only (frames are cut at 8 ms so the task watchdog stays fed) or on both cores. |
| `txfault` | Break the ESP-NOW TX path at the next frame boundary to test self-healing. |

---

## 8. Real-Time Telemetry Specification

Nodes print fixed-width 1 s heartbeats over USB serial. Test scripts parse these columns; keep the format stable.

### 8.1 SOURCE Telemetry (Node 16 - COM116)
```text
|    CPU      | STATE | NODES  |    WIFI     |  AUDIO dBFS  |     STAGE TIMINGS (ms)       |  SOURCE      PKTS  ACK%  FAIL   TOT  |             ROUND-TRIP NET (us)        |
|  %   C  MHz |       | 012345 | GAIN Ch PHY |   RMS    Pk  |  DSP   EncHQ  EncRed  TXend  |  INPUT        1/s     %   1/s  pkts   |              L_Net       R_Net         |
| 40  67  240 | CAST  | 111OO1 | +3.0 13 HT3 | -12.4  -0.8 |  0.45   5.80   1.78   4.37  |  USB        598   87%     8   104K |               1087        1324         |
```
- `DSP`: DSP stage per frame (core 0). `EncHQ`: CPU of the 5 HQ jobs + sub per frame, summed over both cores. `EncRed`: CPU of the 5 MQ jobs per frame, summed over both cores.
- `TXend`: when the frame's last packet finished on air, in ms after the frame tick. This is the critical path; it must stay below the 9.5 ms deadline. It is not CPU time and must not be added to the other stages.
- `CPU %`: (DSP + EncHQ + EncRed) / (2 cores x 10 ms).

### 8.2 SINK Telemetry (Node 4 shown: S3, PCM5102A, -20 dB post-gain; C6 SINKs print the same columns)
```text
|    CPU      | STATE |  CHAN  |    WIFI     | AUDIO     dBFS      SR   PD    CODEC ms  | AMP dB   PKTS  RED PLC DMA OVF UDR   | BUF+DMA   DRIFT CORR |         TIME & SYNCHRONIZATION (ms)    |
|  %   C  MHz |       |        | RSSI Ch PHY |  Enc    RMS   Pk   kHz   ms   Avg   Pk   |  SW  HW   1/s  rec tot UDR ovf udr   |  buf min adj    ppm  |  Local  Master  EMA_offs RB_med RB_rng |
|  4  48  240 | STRM  | LEFT   |  -53 13 HT0 |  LC3    off   off    48   10  0.40  0.49 | -20   -    99   12   0   0   0   0  |  3.9   2   +0   +0.0 |  20050   14585   -5693   -5693   0.38  |
```
- `RED`: frames recovered from redundancy; `PLC`: concealed frames (each one counts against the zero-glitch target); `DMA` / `OVF` / `UDR`: I2S DMA underruns, FIFO overflows (drops above 18 frames), FIFO underruns. Running totals, printed modulo 1000 (`soak_test.py` unwraps them).
- `buf`: buffered audio after each frame pop, FIFO + I2S DMA queue, 1 s average (frames; ~4 in steady state). `min`: lowest FIFO depth (FIFO only) in the last second.
- `DRIFT CORR adj`: net clock-drift corrections in the last second (+ inserted, - dropped samples; `lrn` while the target is being learned, `off` when disabled). `ppm`: SINK clock offset against the SOURCE since STREAM start, from the net corrections (+ = SINK clock fast).

---

## 9. Build, Flash & Test

The scripts activate ESP-IDF v6.1 themselves through [tools/idf_env.ps1](../../tools/idf_env.ps1). One image per chip family (`build_s3/` + `sdkconfig.s3`, `build_c6/` + `sdkconfig.c6`). Role and node identity are chosen at runtime from the Wi-Fi MAC (`main/config.c`).

```powershell
# Compile only
powershell -ExecutionPolicy Bypass -File apps\audio_VSAF\tools\compile.ps1 -Target s3   # or -Target c6

# Build + flash one node (chip inferred from Role / NodeId)
powershell -ExecutionPolicy Bypass -File apps\audio_VSAF\tools\build_and_flash.ps1 -Role SOURCE                    # Node 16, hands-free reset
powershell -ExecutionPolicy Bypass -File apps\audio_VSAF\tools\build_and_flash.ps1 -Role SINK -Port COM4 -NodeId 4
powershell -ExecutionPolicy Bypass -File apps\audio_VSAF\tools\build_and_flash.ps1 -Role SINK -Port COM23 -NodeId 23

# Whole cluster: S3 + C6 built in parallel, then all detected nodes flashed concurrently
powershell -ExecutionPolicy Bypass -File apps\audio_VSAF\tools\parallel_build_and_flash.ps1 [-BuildOnly | -FlashOnly] [-Targets s3] [-Nodes "16,4,5"]

# Reliability soak test: music into Node 16 over USB, PASS = 0 PLC on every SINK
.venv_audio_multicast\Scripts\python.exe apps\audio_VSAF\tools\soak_test.py --duration 900
```

Node 16 is flashed hands-free (`tools/s3_flash_and_reset.py` identifies it by MAC, reboots it into the ROM bootloader and flashes over COM16). See the `parallel-compile-and-flash`, `node16-flash-and-reset` and `vsaf2-soak-test` skills.

### 9.1 SOURCE memory and cache configuration (`sdkconfig.s3`)

Node 16 has no PSRAM (512 KB SRAM), so internal RAM and the shared flash cache are the scarce resources:

| Setting | Value | Why |
| :--- | :--- | :--- |
| `CONFIG_ESP32S3_INSTRUCTION_CACHE_32KB` | y (was 16 KB) | The Wi-Fi driver, ESP-NOW and most application code run from flash. With 16 KB the cache thrashed between the two cores; 32 KB cut Wi-Fi task CPU from 38 % to 8 % (section 10). Costs 16 KB of internal RAM. |
| `CONFIG_LC3_CODE_IN_IRAM` | y | LC3 code and tables in internal RAM (no cache misses in the hot loop). |
| `CONFIG_LC3_ENABLE_PLUS_MODES` | n | LC3plus tables not needed (-36 KB). |
| Wi-Fi static RX buffers (SOURCE) | 6 (SINK 32) | The SOURCE receives only telemetry replies. Set in `vsaf_broadcast_engine.cpp`. |
| AMPDU TX/RX, SoftAP, WPA3 / OWE, Enterprise | off | ESP-NOW broadcast in STA mode needs none of them (-3.5 KB heap). |
| Task stacks | `src_rf` 4 KB, `src_enc1` 5 KB, `src_dsp0` 6 KB | Sized from measured high-water marks (`mem`). |
| `CONFIG_VSAF_TX_POWER_QDBM` (`config.h`, all nodes) | 40 (10 dBm requested) | Moderate on purpose: SOURCE heating (600 packets/s) and future battery SINKs. The old value 12 ("+3 dBm") actually transmitted at 2 dBm. |
| `CONFIG_VSAF_SCAN_DWELL_MS` / `CONFIG_VSAF_SCAN_HOLD_MS` (`config.h`, SINK) | 30 ms / 3000 ms | Channel search from the preferred channel upward, and the hold on the last channel after a stream loss (section 10). |

Free internal heap with all pipeline tasks running: ~30 KB (largest block 16 KB).

On the C6 (`sdkconfig.c6`) `CONFIG_LC3_CODE_IN_IRAM` is off: the C6 SINKs decode with Espressif's fixed-point codec and must not link liblc3 at all. All SOURCE-only liblc3 code (`encbench`) is compiled for the S3 only; a stray reference once pulled ~100 KB of LC3 code and tables into C6 internal RAM, and `esp_wifi_init()` then failed with `ESP_ERR_NO_MEM` in a boot loop.

---

## 10. Performance Work Log

### 2026-10-24: dual-core SOURCE pipeline

Starting point: one serial loop on the SOURCE (DSP, 5 encoders, then a TX sweep gated on a TX-done semaphore). It overran the 10 ms frame under channel contention and at low PHY rates, and every overrun showed up as simultaneous PLC bursts on all SINKs.

Changes, in order, each measured on Node 16 while streaming music over USB:

1. **Dual-core pipeline** (section 4): DSP + publish on core 0, lock-free job queue on both cores, dedicated RF task with one packet in flight, timestamp at submit, 9.5 ms TX deadline, reply-request packet last. Encoder bank grew to 11 slots for 5 independent satellite streams + sub.
2. **RAM to make it fit**: LC3plus compiled out (-36 KB), SOURCE Wi-Fi static RX buffers 32 -> 6, smaller task stacks, unused buffers of the old loop removed. The first flash had failed to create the pipeline tasks for lack of internal heap.
3. **Measurement tools**: `timing`, `tasks`, `encbench`, `mem`, `log`, `enccores`, `dspload` (section 7.2). They showed LC3 itself was not the problem: encodes took ~1.7x their pure CPU cost because the Wi-Fi task (38 % of core 0) and `src_rf` (31 % of core 1, ~280 us per `esp_now_send()`) preempted them, and both grew when USB audio was streaming.
4. **Event-driven RF task**: encoders wake `src_rf` only while it waits for a packet; the 1 ms poll is gone (wakes ~20 -> ~13 per frame).
5. **Dual-rate LC3 (`liblc3_dual`)**: HQ and MQ share one analysis, bit-exact. Two-phase form keeps HQ packets as early as before. Saves ~1.6 ms of CPU per frame and 16 KB of RAM (MQ states ~2 KB instead of 5.4 KB encoders).
6. **32 KB instruction cache**: the decisive fix. Flash-resident Wi-Fi and application code had been thrashing the 16 KB cache shared by both cores.
7. **Leaner Wi-Fi stack**: unused features compiled out (+3.5 KB heap).

Tried and reverted: pinning the Wi-Fi task to core 1 (`esp_now_send()` slower, more deadline drops), and `CONFIG_ESP_WIFI_EXTRA_IRAM_OPT` on top of the 32 KB cache (no measurable change: Wi-Fi task 8.9 % vs 9.0 %, TX end 5.23 vs 5.25 ms, for 0.4 KB less heap).

#### Radio CPU cost: how it was found and what fixed it

After the pipeline and the dual-rate encoder, the radio path still cost about 1 ms of CPU per packet (~6 ms per frame for 6 packets), more than all LC3 encodes together. Three steps were planned to cut it, each measured with `timing`, `tasks` and `encbench` while streaming music.

How the cause was narrowed down:

| Observation | Measurement | Conclusion |
| :--- | :--- | :--- |
| Encodes take 1.6-1.7x longer in the pipeline than in `encbench` | HQ 1.6 ms vs 0.95 ms pure; same factor on both cores | Not the codec, something steals time |
| Interrupts are not it | `encbench` with interrupts enabled vs masked, at prio 24: +2-6 % | Task preemption, not ISRs |
| Two cores contending for SRAM are not it | `enccores 1` (core 0 alone) was even slower: 1.8 ms | Not dual-core memory contention |
| The thieves | `tasks`: Wi-Fi task 34-38 % of core 0, `src_rf` 27-31 % of core 1 | The radio stack preempts the encoders |
| Waking `src_rf` is cheap | TX-done -> `src_rf` running 43-55 us | Not FreeRTOS context switches (the 1 kHz tick only affects timeouts) |
| `src_rf`'s own code is slow | Active ~2.2 ms per frame, of which `esp_now_send()` 1.5 ms, leaving ~60 us per wake for a few dozen lines of C | Points to cache misses, not to the amount of work |
| Everything from flash slows down when USB audio streams | Wi-Fi task 20 % -> 34 %, `esp_now_send()` 170 -> 266 us, esp_timer 1.8 -> 4.9 %; LC3 in IRAM unaffected | The 16 KB instruction cache shared by both cores thrashes |

The three steps and their results:

| Step | Change | Result | Kept? |
| :--- | :--- | :--- | :--- |
| 1 | `src_rf` without polling: encoders notify it only while it waits for a packet (flag handshake, no lost wake-ups); the 1 ms poll is replaced by one timed wake for the held reply packet | Wakes ~20 -> ~13 per frame, but `src_rf` CPU unchanged (27 %) | Yes (cleaner, fewer wakes), but not the fix |
| 2a | Instruction cache 16 KB -> 32 KB (`CONFIG_ESP32S3_INSTRUCTION_CACHE_32KB`) | Wi-Fi task 38 % -> 8 %, `src_rf` 31 % -> 6 %, `esp_now_send()` 280 -> 50 us, wake 55 -> 14 us, HQ job 2.0 -> 1.3 ms, MQ job 0.7 -> 0.34 ms, TX end 8.6 -> 5.1 ms, deadline skips 3-4/s -> 0, idle per core 7-20 % -> ~50 %. Costs 16 KB RAM | Yes: the decisive fix |
| 2b | `CONFIG_ESP_WIFI_EXTRA_IRAM_OPT` (more Wi-Fi code in IRAM) on top of 2a | No measurable change (Wi-Fi 8.9 % vs 9.0 %, TX end 5.23 vs 5.25 ms), -0.4 KB heap | No, reverted |
| 3 | Unused Wi-Fi features compiled out: AMPDU TX/RX, SoftAP, WPA3 SAE/OWE, Enterprise (`sdkconfig.s3`) | +3.5 KB heap (26.6 -> 30.1 KB free, largest block 12 -> 16 KB), no timing change | Yes |

Conclusions:
- **On a dual-core ESP32-S3 without PSRAM, the 16 KB instruction cache is the bottleneck for flash-resident code.** Both cores share it, and the Wi-Fi blobs, ESP-NOW, TinyUSB and application code all compete for it. A miss stalls a core for a flash line fill (DIO flash at 80 MHz). The effect grows with every extra code path that runs, which is why it got worse when USB audio streamed.
- **Keep the 32 KB cache.** Do not shrink it back to win RAM. Hot code that must not depend on the cache (LC3) stays in IRAM.
- **Measure before optimizing the obvious suspect.** Wake latency, context switches and LC3 all looked plausible but were not the cause. Only comparing pure cost (`encbench`) against in-pipeline cost, then per-task CPU (`tasks`), then active time per wake, led to the cache.
- **Not worth it:** submitting the next packet from inside the ESP-NOW send callback (saves under 0.1 ms per frame now that a wake takes 14 us), moving the Wi-Fi task to core 1, and more Wi-Fi IRAM.
- **Not tried yet:** QIO flash mode (twice the line-fill bandwidth of DIO). Risky without checking the flash chip, because a board that cannot boot in QIO needs a manual BOOT-button reflash.

| Metric (1 s windows, music over USB, ch 13, HT20 MCS3) | Serial loop / first dual-core build | Final |
| :--- | :--- | :--- |
| Last packet on air (TX end) | 7.8-9.1 ms | **5.1-5.3 ms** |
| All encode jobs done | 9.6 ms (worst 11.7 ms) | **5.1-5.3 ms** (worst ~6 ms) |
| Deadline skips | 2-35 per s | **0** |
| Wi-Fi task CPU (core 0) | 34-38 % | **8-9 %** |
| `src_rf` CPU (core 1) | 27-31 % | **6 %** |
| `esp_now_send()` CPU per packet | 250-290 us | **~55 us** |
| TX-done -> RF task wake | 45-55 us | **14 us** |
| Idle core 0 / core 1 | 4-7 % / 12-20 % | **46 % / 53 %** |
| Internal heap free | 8 KB (tasks not created) -> 27 KB | **30 KB** |

Soak test after all changes (3 min, Nodes 4, 5, 23, 24): **0 PLC, 0 underruns** on every SINK, 0.14-0.49 % primary packets recovered from redundancy, TX end 4.4 ms average.

Also today: fixed channel 13 at boot with the boot survey off, self-healing ESP-NOW TX and frame-boundary radio changes, Node 16 ROM-bootloader reboot via `esp_rom_software_reset_system()` (an `esp_restart()` from the TinyUSB callback used to hang the board), hands-free Node 16 flashing identified by MAC, and the `soak_test.py` / `usb_audio_streamer.py` reliability test.

### 2026-10-24 (evening): 15-minute soak tests and SINK drift compensation

1. **First 15-minute soak on the new SOURCE: FAIL** (Node 4: 9 PLC, Node 24: 7 PLC, Nodes 5 / 23: 0). The SOURCE was clean (0 frames skipped, TX end 4.4 ms). Every PLC was a single event paired with exactly one FIFO underrun, at a normal 99-100 packets/s, on the two SINKs with the weakest links (0.9 % / 0.6 % loss). The SINKs had been streaming for ~20 min before the window, so the most likely cause was clock drift emptying the 30 ms FIFO cushion; with an empty FIFO a late or lost packet can no longer be repaired from the next packet's redundancy in time.
2. **FIFO-depth telemetry** added to the SINK heartbeat (section 8.2) and to `soak_test.py` (FIFO avg / min, net corrections, ppm per SINK).
3. **Drift compensation** (section 5.3): single-sample insert / drop holds the FIFO depth at its start-up level.
4. **C6 boot loop fixed**: the C6 SINKs had stopped booting (`esp_wifi_init()` out of memory) because the S3-only `encbench` code pulled liblc3 into the C6 image (section 9.1).
5. **First version of the controller filtered the depth in Q8** with a `>> 8` EMA step, which cannot rise to a new level, so it never dropped samples. Fixed to Q16.

| Soak test (15 min, Nodes 4 / 5 / 23 / 24, music over USB) | PLC | UDR | Corrections (net samples) | FIFO avg (frames) |
| :--- | :--- | :--- | :--- | :--- |
| No compensation, SINKs streaming ~20 min before the window | 9 / 0 / 0 / 7 | 9 / 0 / 0 / 7 | - | - |
| Compensation with the Q8 filter (effectively inactive), fresh SINK start | 0 / 0 / 0 / 0 | 0 | +2 / +1 / +1 / +1 | 2.56 / 2.65 / 2.72 / 2.57 |
| Compensation with the Q16 filter, fresh SINK start | **0 / 0 / 0 / 0** | **0** | +1 / **-62** / 0 / +1 | **2.02 / 2.06 / 1.98 / 1.88** |

Observations:
- With the working controller the FIFO depth stays at its learned target (~2 frames after pop) on all SINKs; without it the depth wandered by a whole frame within minutes.
- The depth does not only drift slowly. Besides slides of about one frame over 6-12 minutes (on the order of 10-25 ppm), it sometimes stepped by a full frame within a minute. **Cause: the controller itself.** All 9 up-steps in both runs fell in the same second as an inserted sample (`adj=+1`), with normal packet rate and redundancy counts (so no SOURCE-side frame loss: a skipped packet is recovered from redundancy without changing the depth, and a lost frame tick would step the depth down, not up). After one 481-sample write, every following 480-sample write straddles two DMA descriptors, the SINK loop runs one descriptor later and the frame moves from the DMA queue into the FIFO: the FIFO depth jumps although the buffered audio did not change.
- Fix: the controller now measures FIFO + I2S DMA queue (section 5.3). The DMA queue is samples written minus samples played, counted from the DMA `on_sent` interrupt.
- The `ppm` column only counts corrections, so while the buffered audio stays inside the +/- 0.5 frame deadband it reads ~0; it becomes a drift measurement once corrections run continuously (long runs).

**Counter rollover review** (all counters in the VSAF path checked): the 8-bit `seq`, the SINK clock offset and all display counters are wrap safe. One bug: the SOURCE computed RTT and clock offset as 64-bit `esp_timer` minus the echoed 32-bit `t_tx1_us`, so after 71.6 minutes of SOURCE uptime every telemetry reply failed the RTT filter (RTT frozen, offset garbage). Fixed with 32-bit modular subtraction.

### 2026-10-25 (night): 80-minute soak with buffered-audio drift control and the RTT wrap fix

80-minute window (chosen so the SOURCE crosses 71.6 min of uptime during the run), music over USB, ch 13, HT20 MCS3. **Result: FAIL** (Node 24: 108 PLC, Node 4: 6 PLC; Nodes 5 and 23: 0 PLC).

| Node | RSSI | Loss (RED+PLC) | PLC | of which underruns | Buffered avg (frames) | Net corrections | Drift |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| 4 (L, S3) | -54 dBm | 0.73 % | 6 | 0 | 4.01 | +6 | ~0 ppm |
| 5 (R, S3) | -45 dBm | 0.15 % | 0 | 0 | 4.09 | **-2116** | **-9.1 ppm** (SINK slow) |
| 23 (C, C6) | -43 dBm | 0.06 % | 0 | 0 | 3.97 | -5 | ~0 ppm |
| 24 (Sub, C6) | **-67 dBm** | **3.43 %** | 108 | 47 | 3.93 | -63 | -0.3 ppm |

Findings:
- **Drift control works.** Buffered audio stayed at ~4 frames on every SINK for 80 minutes. Node 5 needed 2116 dropped samples (one every ~2.3 s, -9.1 ppm), the others close to none. No PLC was caused by drift: Nodes 5 and 23 had zero, Node 4's six were sequence gaps (no underrun).
- **RTT wrap fix works.** The SOURCE passed 71.6 min of uptime during the run and its RTT columns kept updating with normal values (~0.65-0.95 ms) to the end.
- **Node 24 (sub) had a much weaker link than in the evening runs**: -64 to -68 dBm for the whole run against -58 to -60 dBm before, and ~4 lost packets per second. The node or its surroundings presumably changed. With that loss rate, bursts beyond the sub's 3-frame redundancy caused 61 gap PLCs, and 47 more were underruns.
- **The FIFO cushion, not the redundancy depth, limits recovery.** A lost frame is only recovered when the next packet arrives. After 2-3 lost packets in a row, the 2 frames in the FIFO run out before the recovering packet comes, so the SINK conceals frames that redundancy would have restored 10-20 ms later. A deeper FIFO target (+1 to +2 frames, i.e. +10 to +20 ms latency) would let the full redundancy work. This is a latency trade-off against the < 50 ms soft target.
- **Node 4's 6 PLC** are isolated bursts of 3+ lost packets at a normal -54 dBm (0.73 % loss).
- **SOURCE**: 7 seconds below 590 packets/s, all in one 9-second episode around 9.5 minutes into the run (567-586 packets/s, TX end 4.3-5.0 ms). The CPU was fine, so this was likely a burst of RF contention (TX-done timeouts or deadline skips). The rest of the run was clean.

### 2026-10-25: boot-to-streaming time and TX power

**Boot time.** Measured with a hard reset (USB RTS) and the `reboot` command, 2-3 runs per SINK, device time since reset:

| SINK | Reset | Before | After |
| :--- | :--- | :--- | :--- |
| 4, 5 (S3) | hard reset | 2.17 s | **0.35 s** |
| 23, 24 (C6) | hard reset | 2.29 s | **0.47 s** |
| 23, 24 (C6) | `reboot` command | 2.53 s | **0.71 s** |

~80 % of the old time was the channel search: SINKs started on channel 1 and hopped every 150 ms, while the SOURCE sits on channel 13. Now SINKs start on `CONFIG_VSAF_PREFERRED_WIFI_CHANNEL` and hop upward with wrap (13 -> 1 -> 2 ...) with a 30 ms dwell (`CONFIG_VSAF_SCAN_DWELL_MS`; a full sweep takes ~0.4 s). After losing the stream a SINK keeps its channel for `CONFIG_VSAF_SCAN_HOLD_MS` (3 s) before sweeping. SOURCE reboot test: all SINKs detected the loss within 60 ms, held channel 13 and were streaming again 2.3 s after the reset request, limited only by the SOURCE's own boot. The remaining boot time is mostly the bootloader's image check (~250 ms on the C6), kept for robustness against flash corruption.

**TX power.** `esp_wifi_set_max_tx_power()` is quantized (the documented table maps 8-19 -> 2 dBm, 34-43 -> 8.5 dBm, 44-51 -> 11 dBm, ...), so the old setting of 12 ("+3 dBm") actually transmitted at 2 dBm. `esp_wifi_get_max_tx_power()` returns the requested value on both S3 and C6, so the readback cannot confirm the radiated power. Measured instead through RSSI on the receiving side:
- S3 (SOURCE): 2 -> 13 dBm raised the SINKs' RSSI by +7.5 to +10.5 dB, roughly as set.
- C6 (Node 23): 2 -> 8.5 dBm raised its uplink by +14.5 dB, 8.5 -> 11 dBm by +5.7 dB, 11 -> 13 dBm by +0.8 dB. The C6 transmits far below nominal at the lowest settings, which explained most of its uplink / downlink asymmetry.

New default for all nodes: 10 dBm requested (`CONFIG_VSAF_TX_POWER_QDBM` = 40), kept moderate for SOURCE heating and future battery SINKs. Result at 0.5 m: downlink -30 to -42 dBm, uplink -30 to -53 dBm, ~99 % of telemetry replies received.

**TX power readback.** Neither `esp_wifi_get_max_tx_power()` (echoes the request) nor `wifi_country_t.max_tx_power` (always 20 dBm, the regulatory maximum) reports the applied level, on S3 and C6 alike. The firmware therefore reads the setting back once after each change and caches it for the heartbeat and `txstat`, labelled as the setting; the earlier per-heartbeat API call was removed.

**Console findings and fixes.**
- The S3 SINKs (Nodes 4, 5) accepted no console input, because on the S3 the USB-Serial-JTAG console is a secondary, output-only console. Fixed: the CLI task reads the USB-Serial-JTAG RX FIFO directly on S3 SINKs (section 7).
- Node 24's console once stopped in both directions after repeated USB replugs, while audio kept working. esptool could still use the same port, and a chip reset cleared it, so the host side was alive. The leading suspect is ESP-IDF's "host connected" flag (both `read()` and `write()` fail with EIO while it is false). `usjdiag` (section 7.2) now records the evidence across a reset.
- **Replug experiment (Node 24, 2026-10-25): the stall did not reproduce.**

  | Round | Action | Result |
  | :--- | :--- | :--- |
  | A | 3 quick replugs within ~5 s | Console OK at once, 0 rejected writes, connected the whole session |
  | B | One slow replug (~5 s unplugged) | Console OK at once, 0 rejected writes |
  | C | Replug while a program held COM24 open | The open handle failed (Windows error 22) and never delivered again, as expected; COM24 was back 8.9 s after the unplug and a reopened port delivered the fresh boot log |

  Conclusion: replugging alone, quick or slow or with the port held open, is handled correctly by the firmware console path and by Windows. The original stall needed another condition, e.g. power lost again mid-boot or several programs (Device Manager, test scripts) touching the port at the same time. Topic parked. If a console goes silent again: open the port, try a host-side reopen / DTR toggle, then reset the chip (RTS) and run `usjdiag`; its `previous session` line shows whether the firmware believed no host was connected (`fail(EIO)` > 0, a device-side cause) or wrote normally while the data was lost on the host.

### Next steps
- Decide on a deeper SINK FIFO target (latency vs. recovery) and re-run.
- Check Node 24's placement / antenna (RSSI 8 dB worse than in the evening runs).
- Log `txstat` periodically during soaks to classify SOURCE packet-rate dips.
- Longer soak runs (1 h+, including a SINK that has streamed for a long time) to confirm zero PLC with drift compensation.
- Understand the one-frame FIFO depth steps; measure the real per-SINK drift (e.g. a run with `driftcorr 0` and the FIFO slope, or SOURCE/SINK timestamps).
- Measure end-to-end latency, inter-speaker skew and clock drift (GOALS.md priority 1).
- Spatial upmix on the SOURCE: about half of each core is now free for it; `dspload` can rehearse the extra load.
- Optional RAM: limit `liblc3_dual` to 10 ms frames (~8-12 KB of tables) and, if the runtime `sr` switch can go, to 48 kHz + 8 kHz.
