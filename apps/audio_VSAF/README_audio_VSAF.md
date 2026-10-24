# audio_VSAF: VSAF 3.0 Multi-Channel Wi-Fi Audio Broadcast

## 1. Overview

The **`audio_VSAF`** application distributes up to 6 audio channels (5.1) from one **SOURCE** (ESP32-S3, Node 16) to up to 6 **SINK** speaker nodes (ESP32-S3 / ESP32-C6) over 802.11 ESP-NOW broadcast (`FF:FF:FF:FF:FF:FF`), with LC3 coding, multi-tier redundancy and a strict 10 ms frame cadence. Project goals and numeric targets (latency, inter-speaker sync, zero glitches per hour) are in [GOALS.md](../../GOALS.md).

Main building blocks:

1. **VSAF 3.0 multi-tier Unequal Error Protection (UEP)**: every 248-byte satellite packet (`0x1337`, ch 0..4) carries the current frame at HQ (120 B, 96 kbps @ 48 kHz) plus the two previous frames at MQ (60 B, 48 kbps). Subwoofer packets (`0x1338`, ch 5) carry 4 frames of 60 B @ 8 kHz. Up to 2 lost satellite packets in a row (3 for the sub) are recovered from later packets without concealment.
2. **Dual-core SOURCE pipeline** (section 4): core 0 runs USB input + DSP and encodes, core 1 encodes and owns the radio. Encode work is a lock-free job queue shared by both cores. A dedicated RF task keeps exactly one packet in flight, stamps `t_tx1_us` right before each `esp_now_send()` and never transmits after the 9.5 ms frame deadline.
3. **Dual-rate LC3 encoding (`liblc3_dual`)** (section 5): HQ and MQ frames of a channel share one LC3 analysis; MQ only adds a quantization pass. Output is bit-exact with two separate encoders.
4. **Commanded round-robin SINK telemetry**: bit 7 (`REQ_ACK`) of `packet_flags` grants exactly one SINK an uplink reply per 10 ms frame; that packet is sent last in the burst so the reply never collides with the downlink.
5. **Self-healing radio**: TX-done timeouts, ESP-NOW queue exhaustion and implausible completions are detected and repaired by re-initialising ESP-NOW (section 4.4). Channel and PHY changes are applied only at frame boundaries with the radio idle.
6. **Fixed channel 13 at boot** (`CONFIG_VSAF_PREFERRED_WIFI_CHANNEL`), boot survey off by default (`CONFIG_VSAF_BOOT_CHANNEL_SCAN`). Default PHY HT20 MCS3, TX power +3 dBm.
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
                 802.11 broadcast, ch 13    |  HT20 MCS3 (default), +3 dBm
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

**Status:** these values are telemetry today. SINK playout does not yet use them (it is FIFO driven), and there is no sample-clock correction between SINK crystals. The inter-speaker sync target (GOALS.md: within a few samples, held over long sessions) has not been measured yet; earlier claims of +/- 10 us alignment are unverified.

---

## 7. Interactive CLI Console Commands

USB serial on all nodes (**COM116** for the SOURCE, **COM4** / **COM5** / **COM23** / **COM24** for the SINKs), 115200 baud. Open ports with DTR/RTS low (see the `esp32-usb-serial-debug` skill).

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
| `txp` / `txpower <dBm>` | Both | TX power (default +3 dBm). |
| `ch <0..5>` | SINK | Switch the received audio channel. |
| `clear` / `cls` | Both | Reset frame, PLC and error counters. |
| `bootloader` / `download` | Both | Reboot into the ROM bootloader for flashing (Node 16: also triggered by the 1200-baud touch). |
| `reboot` / `reset` | Both | Software restart. |

### 7.2 SOURCE timing and debug

| Command | Description |
| :--- | :--- |
| `timing` | Per-stage table of the last 1 s window (avg / max / count): DSP, LC3 HQ / MQ / sub job times, TX submit -> TX-done, TX-done -> `src_rf` wake, `esp_now_send()` CPU, radio idle gap between packets, first submit, TX end, all jobs done, encode CPU per core; plus overruns, cancelled jobs, MQ missing / waits, deadline skips, TX timeouts, `src_rf` wakes and active time per frame, heap. |
| `tasks` | CPU per FreeRTOS task over 1 s (run-time stats; interrupt time counts against the interrupted task). |
| `txstat` | Channel, PHY, TX power, recoveries, send errors (incl. `no_mem`, last error), TX-done timeouts, status failures, deadline drops. |
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

### 8.2 SINK Telemetry - Node 4 (COM4 - Left Channel, PCM5102A 24-bit, -20 dB Post-Gain)
```text
+=================================================================== ESP32-S3-04-LEFT [SINK] ====================================================================+
|    CPU      | STATE |  CHAN  |    WIFI     | AUDIO     dBFS      SR   PD    CODEC ms  | AMP dB   PKTS  RED  PLC  DMA   FIFO    |         TIME & SYNCHRONIZATION (ms)    |
|  %   C  MHz |       |        | RSSI Ch PHY |  Enc    RMS   Pk   kHz   ms   Avg   Pk   |  SW  HW   1/s  rec  tot  UDR   UDR     |  Local  Master  EMA_offs RB_med RB_rng |
|  6  47  240 | STRM  | LEFT   |  -50 10 HT0 |  LC3  -33.4 -28.0    48   10  0.62  1.33 | -20   -   100   20    0    0     0    |   7996  127283   +1192   +1192   5.66  |
```

### 8.3 SINK Telemetry - Node 23 (COM23 - Center Channel, MAX98357A, +3 dB Gain)
```text
+=================================================================== ESP32-C6-23-CNTR [SINK] ====================================================================+
|    CPU      | STATE |  CHAN  |    WIFI     | AUDIO     dBFS      SR   PD    CODEC ms  | AMP dB   PKTS  RED  PLC  DMA   FIFO    |         TIME & SYNCHRONIZATION (ms)    |
|  %   C  MHz |       |        | RSSI Ch PHY |  Enc    RMS   Pk   kHz   ms   Avg   Pk   |  SW  HW   1/s  rec  tot  UDR   UDR     |  Local  Master  EMA_offs RB_med RB_rng |
| 12  46  160 | STRM  | CNTR   |  -51 10 HT0 |  LC3  -33.5 -27.6    48   10  1.24  1.53 |   0  +3    99   12    0    0     0    |   8058  127373   +1193   +1193   6.07  |
```

The SINK `RED`, `PLC`, `DMA` and `UDR` counters print modulo 1000; `soak_test.py` unwraps them.

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

Free internal heap with all pipeline tasks running: ~30 KB (largest block 16 KB).

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

### Next steps
- 15-minute soak test (zero-PLC target) on the final build; then longer runs.
- Measure end-to-end latency, inter-speaker skew and clock drift (GOALS.md priority 1).
- Spatial upmix on the SOURCE: about half of each core is now free for it; `dspload` can rehearse the extra load.
- Optional RAM: limit `liblc3_dual` to 10 ms frames (~8-12 KB of tables) and, if the runtime `sr` switch can go, to 48 kHz + 8 kHz.
