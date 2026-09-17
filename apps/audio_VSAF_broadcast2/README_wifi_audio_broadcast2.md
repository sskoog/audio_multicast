# Audio VSAF Broadcast 2: Multi-Channel Soft-ACK, Soft-ARQ & PTP Time-Synchronized Streaming

## 1. Executive Summary & Architectural Overview

The **`audio_VSAF_broadcast2`** application implements a high-fidelity, ultra-low-latency, multi-channel, multi-speaker wireless audio distribution system over 802.11 ESP-NOW broadcast semantics (`FF:FF:FF:FF:FF:FF`). It is engineered to distribute up to 6 distinct audio channels synchronously from a single **SOURCE** transmitter to **2 to 6 SINK** speaker nodes with microsecond-level presentation timeline alignment.

### Key Architectural Evolution over Legacy Unicast
Legacy multi-unicast Wi-Fi audio systems rely on IEEE 802.11 MAC-layer unicast semantics where receivers send immediate hardware-level MAC ACKs. While effective for 1-to-1 connections, this architecture degrades catastrophically in multi-speaker topologies when any node is unpowered or experiences RF fading:
1. **Exponential MAC Backoff Freezes**: In 802.11 unicast, an unacknowledged packet triggers up to 7 hardware retransmissions with exponential backoff, blocking the Wi-Fi baseband TX queue for 3.0 to 5.0 ms per missing node. If two nodes are offline, the queue starves completely, destroying the 10.0 ms audio frame budget and causing audible dropouts on online speakers.
2. **Deterministic Broadcast Channel Access**: `audio_VSAF_broadcast2` replaces all hardware MAC ACKs with **pure Layer-2 Broadcast (`FF:FF:FF:FF:FF:FF`) in both forward (audio) and reverse (feedback) directions**. Broadcast frames require 0 hardware ACKs and trigger 0 exponential backoffs. Every transmission executes in fixed, predictable airtime (~75 us at 24 Mbps OFDM).
3. **Deterministic Soft-ACKs**: SINK nodes acknowledge received packets by broadcasting a compact, 16-byte software acknowledgment (`vsaf_soft_ack_t`) within 50 us of packet reception.
4. **Soft-ARQ Selective Repair**: Instead of hardware MAC retries blocking the channel, the SOURCE executes a scheduled **ARQ Repair Window** within the second half of each 10 ms audio frame, retransmitting only unacknowledged channel payloads.
5. **Dual-Way PTP Microsecond Time Synchronization**: Both forward audio frames and reverse Soft-ACK packets embed microsecond hardware timestamps (`esp_timer_get_time()`). This enables continuous dual-way Precision Time Protocol (PTP) calculation of true Round-Trip Time (RTT) and clock offset, locking all SINK presentation timelines together within +/- 10 microseconds without external time servers.

```
                  +----------------------------------------------------+
                  |              ESP32-S3 SOURCE (Node 16)             |
                  |  - Xtensa Dual-Core @ 240 MHz + Hardware FPU       |
                  |  - Dual-Core Parallel liblc3 Audio Encoders        |
                  |  - UAC1 USB Audio Speaker (48 kHz 16-bit Stereo)   |
                  |  - 6-Slot Broadcast Sweep (750 us per slot)        |
                  |  - Soft-ARQ Repair Window (4.5 ms - 8.0 ms)        |
                  |  - Dual-Way PTP Master Clock Time Server           |
                  +-------------------------+--------------------------+
                                            |
                 802.11 Layer-2 Broadcast   |  (OFDM 24.0 Mbps, FF:FF:FF:FF:FF:FF)
                 0 Hardware ACKs, 0 Backoff |  Audio Downlink + Soft-ACK Uplink
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
| **Node 16** | Seeed Studio XIAO ESP32-S3 | ESP32-S3 (Xtensa Dual-Core + FPU) | 4 MB / 512 KB | `E0:72:A1:D8:4C:D0` | **COM16** (Bootloader)<br>**COM116** (Runtime App) | **Audio SOURCE**: UAC1 USB Audio Speaker, dual-core LC3 encoder, 6-slot broadcast sweeper, soft-ACK collector, PTP master. |
| **Node 20** | Waveshare ESP32-C6-LCD-1.47 | ESP32-C6 (160 MHz RISC-V) | 8 MB / 512 KB | `AC:EB:E6:23:DC:24` | **COM20** | Audio SINK / Subwoofer (Channel 5) / Console Display with ST7789 LCD. |
| **Node 21** | ESP32-C6-WROOM-1 DevKit | ESP32-C6 (160 MHz RISC-V) | 8 MB / 512 KB | `98:A3:16:9D:57:EC` | **COM21** (Flash)<br>**COM121** (App) | Audio SINK (Channel 2: Center) or USB Host Bridge. |
| **Node 23** | Waveshare ESP32-C6-Zero | ESP32-C6 (160 MHz RISC-V) | 8 MB / 512 KB | `B0:A6:04:99:38:44` | **COM23** | **Audio SINK Left (Channel 0)**: MAX98357A I2S DAC, WS2812B RGB indicator. |
| **Node 24** | Waveshare ESP32-C6-Zero | ESP32-C6 (160 MHz RISC-V) | 8 MB / 512 KB | `B0:A6:04:99:18:E4` | **COM24** | **Audio SINK Right (Channel 1)**: MAX98357A I2S DAC, WS2812B RGB indicator. |
| **Node 25** | Heemol ESP32-C6 Mini | ESP32-C6 (160 MHz RISC-V) | 8 MB / 512 KB | `E8:3D:C1:FB:DC:C4` | **COM25** (or COM10) | Audio SINK (Channel 3: Surround Left) / Test Node. |
| **Node 26** | Heemol ESP32-C6 Mini | ESP32-C6 (160 MHz RISC-V) | 8 MB / 512 KB | `98:A3:16:AC:13:38` | **COM26** (or COM22) | Audio SINK (Channel 4: Surround Right) / Test Node. |

### 2.1 Node 16 (SOURCE) USB VID:PID Registry & Operating Modes

Node 16 (Seeed Studio XIAO ESP32-S3) utilizes native USB connected directly to GPIO 19 (`D-`) and GPIO 20 (`D+`). It exposes four distinct USB VID:PID identities depending on boot mode and firmware state:

| State / Operating Mode | USB Subsystem / Driver | Windows Device Friendly Name | Hardware Instance ID (VID/PID) | Assigned Port / Endpoint | Function / Usage |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Download / Flash** | Native USB-Serial/JTAG ROM Bootloader | `USB Serial Device (COM16)` | `USB\VID_303A&PID_1001` | **COM16** | Flashing firmware via `esptool` / `s3_flash_and_reset.py`. |
| **Download / Flash** | Native USB-OTG ROM Bootloader | `USB Serial Device (COM3)` | `USB\VID_303A&PID_0009` | **COM3** | Alternative ROM DFU/Serial download port. |
| **Application Runtime** | TinyUSB CDC ACM Console | `USB Serial Device (COM116)` | `USB\VID_303A&PID_4002&MI_00` | **COM116** | High-speed ASCII CLI, runtime commands, and 10 Hz telemetry. |
| **Application Runtime** | TinyUSB UAC1 Stereo Audio | `Node16 audio (USB Speaker)` | `USB\VID_303A&PID_4002&MI_02` | USB Audio Output | 48 kHz / 16-bit PCM digital audio stream from Windows host. |

### Pinout Reference
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

## 3. Protocol Specification & Wire Formats (`vsaf_protocol.hpp`)

All network communication uses the **VSAF 2.0 (Variable-rate Synchronized Audio Frame)** container. All fields are explicitly packed in little-endian byte order.

### 3.0 802.11 basics

The RF RX amplifier on ESP32 has a Low-Noise Amplifier (LNA) which can easily saturate at high signal strengths. This can cause the RF RX to drop perfectly valid packets due to distortion, leading to corrupt bits or fault CNCs. Optimal RSSI is -60 dBm to -30 dBm for ESP32s. If RSSI is over -20 dB, reduce the RF TX gain.

### 3.1 16-Bit Tag Word (`0xTTRS`)

Every packet begins with a 16-bit Tag word encoding packet routing, type, and operational flags:

```
 15  14  13  12  11  10   9   8   7   6   5   4   3   2   1   0
+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
|  VERSION  |SYNC|RET|   TYPE    |     RX_ID     |     TX_ID     |
+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
```

- **Bits 3:0 (`TX_ID`)**: Sender Node ID:
  - `0x0`: SOURCE (Node 16)
  - `0x1` to `0x6`: SINK Nodes 1 through 6
- **Bits 7:4 (`RX_ID`)**: Target Receiver Node ID:
  - `0x0` to `0x6`: Specific target node / audio channel
  - `0xF`: Wildcard Broadcast (all nodes process)
- **Bits 11:8 (`TYPE`)**: Packet Type:
  - `0x0`: `VSAF_TYPE_AUDIO_PRIMARY` (Primary scheduled audio broadcast)
  - `0x1`: `VSAF_TYPE_SOFT_ACK` (Reverse software acknowledgment)
  - `0x2`: `VSAF_TYPE_AUDIO_REPAIR` (Soft-ARQ retransmitted audio frame)
  - `0x3`: `VSAF_TYPE_DISCOVERY_SYNC` (Periodic discovery & clock sync beacon)
- **Bit 12 (`RET` - Retry Flag)**: `0` = First transmission, `1` = Retransmission.
- **Bit 13 (`SYNC` - Time Sync Valid)**: `1` = High-precision hardware timestamps are valid.
- **Bits 15:14 (`VERSION`)**: Protocol version (`0b00` for VSAF 2.0).

---

### 3.2 Broadcast Audio Frame Wire Format (`vsaf_audio_packet_t`)

Audio frames carry compressed LC3 frames alongside timing and codec configuration:

```
 0                   1                   2                   3
 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|           tag (u16)           |           seq (u16)           |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                          t_tx1_us                             |
|             (SOURCE TX Timestamp in Microseconds)             |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|   sr_khz (u8) |        frame_dur_us (u16)     |payload_len(u8)|
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                 LC3 Compressed Audio Payload                  |
|               (Length = payload_len, e.g. 120 bytes)          |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
```

- **Header Size**: 12 bytes.
- **`seq`**: Monotonically incrementing 16-bit sequence number per channel.
- **`t_tx1_us`**: Master microsecond timestamp taken from `esp_timer_get_time()` immediately before the Wi-Fi baseband driver call.
- **`sr_khz`**: Audio sample rate (8, 16, 24, 32, 48 kHz).
- **`frame_dur_us`**: Frame duration in microseconds (typically 10,000 us = 10.0 ms).
- **`payload_len`**: Number of octets in the LC3 payload (typically 120 octets for 96 kbps per channel, or 80 octets for 64 kbps Subwoofer).
- **Total Packet Length**: 132 bytes (120-byte payload + 12-byte header). Airtime at 24 Mbps OFDM is **~76 us**.

---

### 3.3 Soft-ACK Wire Format (`vsaf_soft_ack_t`)

SINK nodes broadcast this 16-byte structure back to the SOURCE immediately upon receiving an audio frame:

```
 0                   1                   2                   3
 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|           tag (u16)           |          ack_seq (u16)        |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                         t_tx1_echo                            |
|             (Echoed SOURCE Timestamp in Microseconds)         |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|         t_dwell_us (u16)      | down_rssi(u8) | fifo_fill(u8) |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|          crc16 (u16)          |           (Padding)           |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
```

- **Size**: Strictly 16 bytes. Airtime at 24 Mbps OFDM is **~38 us**.
- **`tag`**: Tag word with `TYPE = 0x1` (Soft-ACK), `TX_ID = SINK_ID`, `RX_ID = 0x0` (SOURCE).
- **`ack_seq`**: Sequence number of the audio frame being acknowledged.
- **`t_tx1_echo`**: Direct echo of the SOURCE's `t_tx1_us` timestamp, enabling instantaneous zero-state RTT calculation on the SOURCE.
- **`t_dwell_us`**: Exact elapsed time in microseconds inside the SINK from receiving the frame (`t_rx1`) to transmitting the Soft-ACK (`t_tx2`): `t_dwell_us = t_tx2 - t_rx1`.
- **`downlink_rssi`**: RF signal strength (-dBm) measured by the SINK radio when receiving the audio frame.
- **`fifo_fill`**: SINK jitter buffer fill percentage (0 to 100%), providing proactive flow telemetry.
- **`crc16`**: CCITT-16 checksum protecting the Soft-ACK against bit errors.

### 3.4 VSAF CRC-16 Checksum vs. 802.11 Layer-2 Hardware FCS (CRC-32)

A common question in wireless protocol design is: **If the IEEE 802.11 Wi-Fi MAC layer already computes a 32-bit hardware Frame Check Sequence (FCS / CRC-32) over every radio frame, why does VSAF include an additional application-layer CRC-16?**

The distinction lies in the **End-to-End Principle**, the realities of **broadcast promiscuity**, and **hardware-software boundary protection**:

1. **Physical Layer (FCS) vs. End-to-End Application Integrity**:
   - The 802.11 MAC hardware FCS protects solely the *air interface* between RF demodulator and MAC transceiver. Once the hardware validates the FCS, it strips the FCS and writes the frame into a DMA ring buffer.
   - Any corruption occurring *after* the hardware MAC—such as DMA bus contention, memory copy overrun, pointer aliasing, or FreeRTOS task preemption during ring buffer handoff—is invisible to the 802.11 FCS.
   - VSAF's CRC-16 protects the payload from the instant it is formatted in user memory on the transmitter to the instant it is parsed in the receiving task, providing end-to-end data integrity across the entire hardware and OS pipeline.

2. **Broadcast Promiscuity & Foreign Packet Discrimination**:
   - ESP-NOW broadcasts are received by *all* Wi-Fi nodes listening on the selected 2.4 GHz channel.
   - Any third-party ESP-NOW device, smart bulb, or vendor action frame within RF range will pass hardware FCS because its RF transmission was electrically valid.
   - If an alien packet happens to match the frame length of a Soft-ACK or audio packet, relying solely on tag word matching (`0xTTRS`) creates a non-zero risk of accepting foreign or malformed data.
   - Computing a CCITT-16 checksum across the exact struct payload guarantees that only genuine VSAF frames produced by nodes running this exact protocol version are accepted.

3. **Protection of Critical Arithmetic Fields**:
   - VSAF Soft-ACK frames carry microsecond timestamps (`t_tx1_echo`, `t_dwell_us`, `t_sink_tx_us`) used for closed-loop RTT computation and PTP clock servo synchronization.
   - A single corrupted bit in `t_dwell_us` or `t_tx1_echo` would cause massive false spikes in estimated network jitter (e.g., millions of microseconds) or trigger false ARQ retransmissions during the 4.5 ms repair sweep.
   - The CRC-16 acts as a cryptographic-style sanity gate before any value enters the PTP clock filter or the retransmission state machine.

4. **Zero Computational Penalty**:
   - On the ESP32-S3 (240 MHz) and ESP32-C6 (160 MHz), calculating CCITT-16 over a 14-byte Soft-ACK payload requires **fewer than 100 clock cycles (< 0.5 microseconds)**.
   - Given a 750 us channel slot budget, 0.5 us represents less than 0.07% of the slot time, making the additional safety effectively free.

---

## 4. 10 ms Frame Timing Budget & Channel Scheduling

The system operates on an exact **10,000 us (10.0 ms)** hardware timer period driven by `esp_timer_get_time()`. FreeRTOS tick delays (`vTaskDelay`) are strictly avoided to eliminate cumulative clock drift.

```
0.0 ms                    4.5 ms                     7.8 ms                   10.0 ms
+-------------------------+--------------------------+------------------------+
|   PHASE 1:              |   PHASE 2:               |   PHASE 3:             |
|   Primary Broadcast     |   Soft-ARQ Repair        |   Codec Encode &       |
|   Sweep (Channels 0..5) |   Window (Unacked Chs)   |   Buffer Prep          |
|   [ 6 x 750 us slots ]  |   [ 4 x 750 us slots ]   |   [ 2.2 ms cushion ]   |
+-------------------------+--------------------------+------------------------+
```

### 4.1 Phase 1: Primary Broadcast Sweep (0.0 ms to 4.5 ms)
- Divided into **6 sequential channel slots of 750 microseconds each**:
  - Slot 0 (t = 0 us): Channel 0 (Left - Node 23)
  - Slot 1 (t = 750 us): Channel 1 (Right - Node 24)
  - Slot 2 (t = 1500 us): Channel 2 (Center - Node 21)
  - Slot 3 (t = 2250 us): Channel 3 (Surround Left - Node 25)
  - Slot 4 (t = 3000 us): Channel 4 (Surround Right - Node 26)
  - Slot 5 (t = 3750 us): Channel 5 (Subwoofer - Node 20)
- In each slot:
  1. SOURCE broadcasts the audio frame (`TYPE = 0x0`) to `FF:FF:FF:FF:FF:FF`.
  2. Transmission finishes in ~76 us.
  3. The target SINK filters the tag in software within ~5 ns, timestamps `t_rx1`, and formats its Soft-ACK.
  4. The SINK broadcasts `vsaf_soft_ack_t` back to `FF:FF:FF:FF:FF:FF`.
  5. The entire forward audio + reverse Soft-ACK transaction completes in **under 200 microseconds**, leaving > 550 microseconds of dead-band margin per slot before the next channel transmits.

### 4.2 Phase 2: Soft-ARQ Repair Window (4.5 ms to 7.8 ms)
- At t = 4.5 ms, the SOURCE inspects its acknowledgment bitmap (`acked_in_this_slot[ch]`).
- For any active SINK that failed to return a valid Soft-ACK during Phase 1:
  1. The SOURCE immediately dispatches an ARQ repair broadcast (`TYPE = 0x2`, `RETRY_FLAG = 1`) with the same sequence number.
  2. Up to 4 lost channels can be sequentially repaired with 750 us slot spacing.
  3. If the SINK already received the primary frame (i.e. only the uplink Soft-ACK was lost), it drops the duplicate audio payload from its FIFO and immediately re-broadcasts a Soft-ACK.
  4. If the primary frame was dropped in the downlink, the SINK enqueues the repair frame and transmits its Soft-ACK, completely eliminating packet loss.

### 4.3 Phase 3: Frame Boundary & Encode Preparation (7.8 ms to 10.0 ms)
- The remaining **2.2 ms** is reserved for dual-core LC3 encoding:
  - On the ESP32-S3 (Xtensa @ 240 MHz with FPU):
    - Core 1 encodes Left / Channel 0 (~1.8 ms).
    - Core 0 worker task encodes Right / Channel 1 concurrently (~1.8 ms).
  - Total parallel encode time is < 2.0 ms, guaranteeing completion well before the 10.0 ms deadline.

---

## 5. Precision Time Protocol (PTP) Dual-Way Time Synchronization

To maintain phase-accurate, echo-free audio playback across spatially separated speakers, `audio_VSAF_broadcast2` implements a continuous, dual-way hardware-timestamped PTP synchronization algorithm.

### 5.1 Timestamp Exchange Topology

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
                * <----------------------------------------*---- t_tx2 (Soft-ACK Sent)
               /
              /  Propagation Time
             /
   t_rx2 ----* (Soft-ACK Received)                         |
             |                                             |
```

### 5.2 Microsecond Calculation Formulas
1. **True Round-Trip Time (RTT)**:
   ```text
   rtt_us = (t_rx2 - t_tx1) - t_dwell_us
   ```
   Where `t_dwell_us = t_tx2 - t_rx1` is measured and reported by the SINK. Under normal conditions with 24 Mbps OFDM, `rtt_us` measures between **180 us and 350 us**.

2. **Dual-Way Master-Slave Clock Offset**:
   Assuming symmetric RF propagation delay (T_prop = rtt_us / 2):
   ```text
   clock_offset_us = ((t_rx1 - t_tx1) - (t_rx2 - t_tx2)) / 2
   ```

3. **Moving Window Filtering**:
   To reject RF multipath jitter, SINK nodes pass incoming offset estimates through:
   - An **Exponential Moving Average (EMA)**: `EMA_offs = EMA_offs * 0.95 + sample * 0.05`.
   - A **Rolling Buffer Median & Range Filter**: A 20-sample median filter (`RB_med`) with spread analysis (`RB_rng`) to completely discard outliers.
   - The filtered offset steers the SINK's I2S DMA consumption rate and presentation timestamp (PTS) interpolation, locking inter-speaker synchronization to **within +/- 10 microseconds**.

---

## 6. SINK Audio Processing Pipeline & Volume Control

Each SINK speaker node implements a high-fidelity, click-free audio playback pipeline:

```
[802.11 Broadcast RX]
        |
        v (Tag Filter: RX_ID == MyChannel?)
[VSAF Jitter FIFO] (Pre-roll buffer: 50 ms cushion)
        |
        v (Next Frame Ready)
[liblc3 Decoder] (PLC concealment on missing seq)
        |
        v (480 samples @ 48kHz 16-bit PCM)
[Volume Slew Limiter] (96 dB/s Logarithmic Slew)
        |
        v
[Dual-Descriptor I2S DMA] (Ping-Pong 1920-byte buffers)
        |
        v
[MAX98357A Class-D DAC] (GPIO 1 DOUT, GPIO 2 BCLK, GPIO 3 LRCLK)
```

### 6.1 Volume Control & Logarithmic Slew Rate Limiter (96 dB/s)
- Volume is addressed using an 8-bit unsigned integer (`0` to `255`):
  - `0`: **MUTE** (linear multiplier = `0.0`, -inf dB)
  - `1`: **-96.0 dBFS** (minimum audible floor, linear multiplier = `0.0000158`)
  - `255`: **0.0 dBFS** (unity gain, linear multiplier = `1.0`)
- **96 dB/s Slew Rate**: Volume adjustments are stepped by at most 0.96 dB per 10 ms frame, and linearly interpolated per-sample across the 480 PCM samples. This completely eliminates zipper noise, clicks, or popping during volume changes.
- **Fade Times**:
  - Full scale swing (`-96 dB` to `0 dB`): Exactly **1.00 second**.
  - Moderate adjustment (`-12 dB` to `0 dB`): Exactly **125 ms**.

### 6.2 Packet Loss Concealment (PLC)
If a frame is lost in both the primary sweep and the ARQ repair window, the SINK invokes `lc3_decode(decoder, NULL, pcm_out)` with a `NULL` bitstream pointer. The liblc3 codec standard PLC algorithm interpolates the spectral coefficients and phase of the previous valid frame, masking transient RF dropouts seamlessly.

---

## 7. Dedicated Subwoofer Channel Architecture (Channel ID 5)

To conserve RF airtime while providing uncompromised bass response, Channel 5 is specialized as a dedicated Subwoofer channel:

```
SOURCE DSP Pipeline:
   Stereo Input (48 kHz)
   [Left 480]  ---\  Mono Sum      [480 samples @ 48kHz]    Downsampler (Factor 6)
                   +-------------> [4th-Order LR4 LP]   --> [Decimate 48k -> 8k]
   [Right 480] ---/  (L + R) / 2   [fc = 100 Hz]            [80 samples @ 8kHz]
                                                                     |
                                                                     v
                                                            [LC3 8kHz Encoder]
                                                            [80 Octets / 10ms]
                                                                     |
                                                                     v
                                                            Broadcast (Ch 5)
```

- **4th-Order Linkwitz-Riley Low-Pass Filter (LR4 LP)**:
  - Cascaded pair of 2nd-order Butterworth low-pass biquads (Q = 0.7071).
  - Attenuation: **-6.02 dB** at fc (default 100 Hz), **-24.6 dB** at 200 Hz, and **-128.97 dB** at 4,000 Hz (Nyquist frequency of 8 kHz).
  - The -129 dB stopband rejection completely eliminates aliasing distortion during 6:1 decimation.
- **Native 8 kHz SINK Playback**:
  - Subwoofer SINK nodes (Channel 5) configure their I2S DAC hardware directly to 8 kHz.
  - Decoded 80-sample frames are fed straight into the native 8 kHz DMA descriptors without software upsampling.

---

## 8. USB Audio Class 1.0 (UAC1) Host Interface & Hands-Free Flashing

### 8.1 Dual-Personality Native USB on Node 16 (ESP32-S3)
The Seeed Studio XIAO ESP32-S3 connects its USB-C port directly to the SoC silicon:

| Mode | Active Layer | Assigned COM Port | Description |
| :--- | :--- | :--- | :--- |
| **ROM Bootloader** | Hardware Silicon ROM | **COM16** | Entered via `B`+`R` buttons, `bootloader` CLI command, or 1200-baud touch. Used by `esptool` for flashing. |
| **Runtime Application** | TinyUSB Composite Stack | **COM116** + UAC1 Speaker | Exposes `Node16 audio (USB Speaker)` for PC playback and `COM116` for real-time telemetry and CLI commands. |

### 8.2 Autonomous Post-Flash Reset via RTC Watchdog Timer (WDT)
Because the XIAO ESP32-S3 lacks an external USB-to-UART bridge and has no transistor auto-reset circuit, standard `esptool --after hard-reset` only toggles a virtual register and leaves the board frozen in download mode.

Our automated flasher ([tools/s3_flash_and_reset.py](file:///c:/Git_ble_audio/apps/audio_VSAF_broadcast2/tools/s3_flash_and_reset.py)) executes an atomic hardware reboot:
1. Flashes the binaries (`bootloader.bin`, `partition-table.bin`, `audio_VSAF_broadcast2.bin`) at 921,600 baud.
2. Clears the `RTC_CNTL_FORCE_DOWNLOAD_BOOT` bit in `RTC_CNTL_OPTION1_REG` (`0x6000812C`).
3. Arms the internal **ESP32-S3 RTC Watchdog Timer (WDT)** for 2000 cycles (~50 us) via `RTC_CNTL_WDTCONFIG0_REG` (`0x60008098`).
4. The watchdog triggers a true hardware digital core reset. The chip reboots into flash and re-enumerates as **COM116** and `Node16 audio` completely **hands-free with zero button presses**.

---

## 9. Interactive CLI Console Commands

The diagnostic CLI is accessible over USB serial on all nodes (**COM116** for SOURCE, **COM23** for SINK Left, **COM24** for SINK Right) at 115,200 baud:

| Command | Target | Description |
| :--- | :--- | :--- |
| `vol <0..100>` | SOURCE / SINK | Set volume percentage (0 = Mute, 100 = 0 dBFS). Broadcasts from SOURCE to all SINKs. |
| `voldb <-96..0>` | SOURCE / SINK | Set volume directly in decibels (-96.0 dB to 0.0 dB). |
| `volu8 <0..255>` | SOURCE / SINK | Set raw 8-bit volume level directly. |
| `volch <ch> <0..255>` | SOURCE | Set volume for a specific channel (0: Left, 1: Right, 5: Sub). |
| `mute` / `unmute` | Both | Smoothly mute or unmute audio using 96 dB/s slew rate. |
| `peer list` | SOURCE | Display registered SINK peers, online status, RTT, and Soft-ACK percentages. |
| `start` / `play` | SOURCE | Begin broadcast audio transmission (`CAST` state). |
| `stop` / `pause` | SOURCE | Stop transmission and enter `IDLE` state. |
| `synth` | SOURCE | Toggle internal test sine-wave synthesizer on/off. |
| `sublp <hz>` | SOURCE | Dynamically adjust Linkwitz-Riley subwoofer cutoff frequency (20 to 1,000 Hz). |
| `ch <0..5>` | SINK | Dynamically switch the receiving audio channel ID for this speaker node. |
| `diag` | Both | Print instantaneous telemetry and hardware status report. |
| `clear` / `cls` | Both | Reset all frame counters, PLC statistics, and error counters. |
| `bootloader` | Both | Trigger software reboot into the factory ROM bootloader for firmware flashing. |
| `reboot` / `reset` | Both | Software restart of the microcontroller. |

---

## 10. Real-Time Telemetry Specification

Nodes emit formatted 1.0-second telemetry heartbeats over USB serial:

### SOURCE Telemetry (Node 16 - COM116)
```text
+=================================================================== ESP32-S3-16-SOURCE ====================================================================+
|    CPU      | STATE | PEERS  |    WIFI     | AUDIO     dBFS      SR   PD    CODEC ms  | ARQ   PKTS   ACK%   RTT   |         TIME & SYNCHRONIZATION (ms)     |
|  %   C  MHz |       | Mask   | RSSI Ch PHY |  Enc    RMS   Pk   kHz   ms   Avg   Pk   | Retr   1/s    tot    us   |  Local  Master  EMA_offs RB_med RB_rng  |
|  8  43  240 | CAST  | 11OOOO | +9.0 01 OFD |  LC3  -18.2 -12.1   48 10.0   1.8  2.1   |    0   100   99.8   240   |   12450  12450       0       0      0   |
```

### SINK Telemetry (Node 23 / 24 - COM23 / COM24)
```text
+=================================================================== ESP32-C6-23-LEFT [SINK] ===============================================================+
|    CPU      | STATE |  CHAN  |    WIFI     | AUDIO     dBFS      SR   PD    CODEC ms  | AMP dB   PKTS  PLC  DMA   FIFO   |         TIME & SYNCHRONIZATION (ms)    |
|  %   C  MHz |       |        | RSSI Ch PHY |  Dec    RMS   Pk   kHz   ms   Avg   Pk   |  SW  HW   1/s  tot  UDR  UDR  OVR|  Local  Master  EMA_offs RB_med RB_rng |
|  5  46  160 | STRM  | LEFT   |  -52 01 OFD |  LC3  -18.2 -12.1   48 10.0   2.4  2.8   |  -   +3   100    0    0    0    0|   12450  12450      +2      +1      3 |
```

---

## 11. Automated Build, Flash & Test Workflow

### Step 1: Environment Activation (ESP-IDF v6.0.2)
```powershell
$env:IDF_TOOLS_PATH = "C:\Users\stefa\.espressif"
. "C:\Users\stefa\OneDrive\Documents\ESP\v6.0.2\esp-idf\export.ps1"
```

### Step 2: Compile & Flash SINK Left (Node 23 on COM23)
```powershell
powershell -ExecutionPolicy Bypass -File apps\audio_VSAF_broadcast2\tools\build_and_flash.ps1 -Role SINK -Port COM23 -NodeId 23
```

### Step 3: Compile & Flash SINK Right (Node 24 on COM24)
```powershell
powershell -ExecutionPolicy Bypass -File apps\audio_VSAF_broadcast2\tools\build_and_flash.ps1 -Role SINK -Port COM24 -NodeId 24
```

### Step 4: Compile & Hands-Free Flash SOURCE (Node 16 on COM16 / COM116)
```powershell
powershell -ExecutionPolicy Bypass -File apps\audio_VSAF_broadcast2\tools\flash_s3.ps1
```

### Step 5: Automated Audio Test Matrix
Run the comprehensive test suite in [wifi_audio_tests.md](file:///c:/Git_ble_audio/apps/audio_VSAF_broadcast2/wifi_audio_tests.md) to validate synth audio playback, Soft-ACK delivery rates (> 98%), ARQ packet repair, and PTP dual-way synchronization stability across all active nodes.
