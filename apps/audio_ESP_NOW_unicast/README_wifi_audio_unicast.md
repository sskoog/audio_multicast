# Multi-Unicast ESP-NOW Audio Streaming Architecture & Protocol Specification

## 1. Executive Summary & Hardware Topology

The **`audio_ESP_NOW_unicast`** application implements a high-fidelity, ultra-low-latency, multi-channel, multi-speaker wireless audio distribution system over 802.11 ESP-NOW unicast semantics. The network topology comprises one central **SOURCE** transmitter and **2 to 6 SINK** speaker nodes operating synchronously with microsecond-level presentation timeline alignment.

```
                  +--------------------------------------+
                  |        ESP32-S3 SOURCE (Dongle)      |
                  |  - Xtensa Dual-Core @ 240 MHz        |
                  |  - Dual-Core Parallel liblc3 Encoder |
                  |  - Master PTS Generation (50ms delay)|
                  |  - 0 Hz Circuit Breaker for Offline  |
                  |  - dB-Scale Master & Ch Volume Ctrl  |
                  +-------------------+------------------+
                                      |
       +------------------------------+------------------------------+
       | (Unicast HT20 MCS1)          | (Unicast HT20 MCS1)          | (Unicast HT20 MCS1)
       v                              v                              v
+---------------+              +---------------+              +---------------+
| SINK 0 (Left) |              | SINK 1 (Right)|              | SINK 2..5     |
| ESP32-C6 / S3 |              | ESP32-C6 / S3 |              | ESP32-C6 / S3 |
| MAX98357A DAC |              | MAX98357A DAC |              | MAX98357A DAC |
| 2 DMA Descs   |              | 2 DMA Descs   |              | 2 DMA Descs   |
| 96dB/s Slew   |              | 96dB/s Slew   |              | 96dB/s Slew   |
+---------------+              +---------------+              +---------------+
```

### Hardware Constraints & Assumptions

1. **SOURCE Node**:
   - **SoC**: ESP32-S3 (Xtensa Dual-Core @ 240 MHz).
   - **Compute Capability**: Hardware Single-Precision FPU with SIMD vector extensions.
   - **Dual-Core Encoding**: Core 1 encodes Left / Channel 0 while a dedicated FreeRTOS worker task (`lc3_worker_c0`) on Core 0 encodes Right / Channel 1 concurrently (~3.7 ms total encode time per 10 ms stereo block).
   - **Role**: Dispatches discrete 802.11 unicast frames to registered ONLINE SINKs and sends on-demand volume control commands.
2. **SINK Nodes**:
   - **SoC**: ESP32-C6 (160 MHz 32-bit RISC-V) or ESP32-S3.
   - **Role**: Emits periodic 2 Hz broadcast presence beacons (`SINK_HELLO`), receives unicast packets targeted to its channel, applies 96 dB/s slew-limited volume scaling, decodes LC3 frames, and feeds a dual-descriptor I2S DMA pipeline to a MAX98357A Class-D amplifier.

---

## 2. Audio Volume Control & Slew-Rate Limiter

### 2.1 Scaled 0 to 255 dB-Domain Mapping
Volume is communicated using an 8-bit unsigned integer (`uint8_t volume_u8`), providing 254 active steps across a 96 dB dynamic range (~0.378 dB per step resolution):
- **`0`**: **MUTE** (linear multiplier = `0.0`, $-\infty$ dB)
- **`1`**: **-96.0 dB** (minimum audible sound floor, linear multiplier = `0.0000158`)
- **`255`**: **0.0 dBFS** (maximum volume / unity gain, linear multiplier = `1.0`)

Conversion formula:
```cpp
gain_dB = -96.0f + (float)(vol_u8 - 1) * (96.0f / 254.0f);
linear_gain = powf(10.0f, gain_dB / 20.0f);
```

### 2.2 SINK Slew Rate Limiter (96 dB/s)
To eliminate audible zipper noise, clicks, or pops during volume changes:
1. **Logarithmic Slew in dB Domain**: The SINK steps its internal gain by at most `CONFIG_VOLUME_SLEW_RATE_DB_PER_SEC * (frame_duration_us / 1000000.0f)` (0.96 dB per 10 ms frame at 96 dB/s).
2. **Per-Sample Linear Interpolation**: Across the 480 PCM samples in each 10 ms frame, the gain multiplier is smoothly interpolated from `start_linear` to `end_linear`.
3. **Fade Duration Examples**:
   - Full scale swing (`-96 dB` to `0 dB`): Exactly **1.00 second**.
   - `MUTE` from full volume: Smooth fade to silence in **1.00 second**.
   - Minor adjustment (`-12 dB` to `0 dB`): Seamlessly completes in **125 ms**.

---

## 3. SINK-Initiated Presence & Dynamic Handshake Protocol

### 3.1 The Problem with SOURCE-Side Probing
In naive unicast networks, the SOURCE continuously transmits probe packets to all configured peers to discover when an offline node powers on. However, in 802.11 MAC unicast semantics:
- When a target node is offline or unpowered, the 802.11 MAC hardware controller triggers up to 7 retransmissions per packet, backing off exponentially.
- Each failed unicast attempt locks the Wi-Fi baseband TX queue for **3 to 5 ms**.
- If two or three nodes are powered off simultaneously, the SOURCE baseband queue starves, delaying audio packets to remaining online nodes and causing audible jitter and DMA underruns.

### 3.2 SINK-Initiated Handshake Architecture (`SINK_HELLO`)
To eliminate all SOURCE-side probing overhead and guarantee 100% airtime availability for active nodes, this application implements a **SINK-Initiated Presence Protocol**:

```
+----------------+                                          +----------------+
|  SINK Node     |                                          |  SOURCE Node   |
|  (Boot / Scan) |                                          |  (Core 1 CAST) |
+-------+--------+                                          +-------+--------+
        |                                                           |
        | [State: SCANNING]                                         | (Streaming to active peers)
        |                                                           | (OFFLINE peers get 0 pkts)
        | --- Broadcast SINK_HELLO (2 Hz, octets=0, Ch 0/1) ------> |
        |     (FF:FF:FF:FF:FF:FF - 0 MAC Retries, 0 Block)          |
        |                                                           | SINK_HELLO Received:
        |                                                           | - Mark Peer ONLINE
        |                                                           | - Attach MAC to Stream
        |                                                           |
        | <========= 100 Hz Unicast Audio Stream (LC3) ============ |
        |                                                           |
        | [Buffer 5 frames (50ms)]                                  |
        | [State: PREFILL -> STREAM]                                |
        |                                                           |
```

1. **Zero-Probing SOURCE (0 Hz for Offline Nodes)**:
   - When a SINK is offline, the SOURCE peer table marks it as `OFFLINE`.
   - The SOURCE **completely skips** transmission to `OFFLINE` nodes in `runSourceLoop()`.
   - **0 packets and 0 probes** are dispatched to offline nodes, consuming **0.00 us** of Wi-Fi airtime.
2. **SINK Auto-Announcement (`SINK_HELLO`)**:
   - While in `SCANNING` mode waiting for a stream, the SINK broadcasts a 2 Hz `SINK_HELLO` packet to `FF:FF:FF:FF:FF:FF`.
   - Broadcast packets in 802.11 require no MAC ACKs and trigger **0 hardware retries**, ensuring zero channel blocking.
   - The packet payload contains `octets = 0`, `opcode = SINK_HELLO (0x01)`, and the target audio channel (`channel_id = 0` Left, `1` Right, `2` Center).
3. **Dynamic SOURCE Attachment**:
   - The SOURCE receives `SINK_HELLO` via `onPacketReceived()` on Core 0.
   - The engine automatically adds or updates the SINK peer in the hardware table, marks its status as `ONLINE`, and immediately begins streaming unicast LC3 audio frames at 100 Hz.

### 3.3 Circuit Breaker Mechanism (Fast Failure Detection)
- If a SINK loses power or goes out of range during streaming, the 802.11 hardware MAC fails to receive an ACK.
- In `onPacketSent()`, the SOURCE tracks `consecutive_ack_fails`.
- **Trigger**: Upon **5 consecutive missed ACKs (50 ms)**, the circuit breaker immediately trips the peer status to `OFFLINE`.
- **Result**: Transmission to that SINK drops to **0 Hz** instantly, preventing any channel congestion or frame delivery delays to other active SINKs.

---

## 4. 802.11 Physical Layer (PHY) & Airtime Analysis

### 4.1 PHY Configuration Rationale
By operating in **Multi-Unicast mode** with known MAC addresses, the system leverages:
- Direct negotiation of **802.11n HT20 MCS rates** (up to 72.2 Mbps).
- Mandatory hardware-level **802.11 MAC Acknowledgements (ACKs)** sent by the receiver within 16 us (SIFS).
- Automatic hardware-level MAC retransmissions on transient interference.

| PHY Rate Code | Modulation & Coding | Raw Bitrate | Airtime / Packet (128B) | Airtime (6 Nodes) | Channel Clearance (10ms frame) |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **HT20 MCS1 (Default)** | **QPSK 1/2** | **13.0 Mbps** | **~177 us** | **~1.06 ms** | **89.4% Free** |
| HT20 MCS2 | 16-QAM 1/2 | 19.5 Mbps | ~132 us | ~0.79 ms | 92.1% Free |
| HT20 MCS3 | 16-QAM 3/4 | 26.0 Mbps | ~110 us | ~0.66 ms | 93.4% Free |
| OFDM 12M | QPSK 1/2 | 12.0 Mbps | ~188 us | ~1.13 ms | 88.7% Free |

**Configured Default**:
- **PHY Mode**: `WIFI_PHY_MODE_HT20`
- **Data Rate**: `WIFI_PHY_RATE_MCS1_LGI` (13.0 Mbps)
- **Wi-Fi Channel**: Channel 1 (2412 MHz fixed)
- **TX Power**: `+9.0 dBm` (`36` * 0.25 dBm)

---

## 5. Packet Wire Format (VSAF 8-Byte Compact Header)

To maximize efficiency and maintain 32-bit/64-bit word alignment for LC3 payload decoding, the packet header is streamlined to **8 bytes**:

```
 0                   1                   2                   3
 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|   seq (u8)    |  octets (u8)  |        audio.flags (u16)      |
|               |  (0 = CTRL)   |  (sr_code:3, dur_code:2, res) |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                    audio.pts_us (uint32_t)                    |
|             (Presentation Time Stamp in Microseconds)         |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                 LC3 Compressed Audio Payload                  |
|                 (Length = octets, e.g. 120 bytes)             |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
```

### Control & Volume Packet Overlay (`octets == 0`)
When `octets == 0`, the header is interpreted as a Control/Volume structure:

```cpp
#pragma pack(push, 1)
typedef struct {
    uint8_t  seq;        // Byte 0: Rolling sequence number (0..255)
    uint8_t  octets;     // Byte 1: 0 = Control Packet, 40..240 = LC3 Audio payload length
    union {
        struct {
            uint16_t flags;      // Bytes 2..3: Bit 0..2: SR (8k..96k), Bit 3..4: Dur (10ms)
            uint32_t pts_us;     // Bytes 4..7: Presentation Time Stamp (us timeline)
        } audio;
        struct {
            uint8_t  opcode;     // Byte 2: ControlOpcode (0x01=SINK_HELLO, 0x02=SINK_BYE, 0x04=VOLUME_SET)
            uint8_t  channel_id; // Byte 3: Target Audio Channel (0=Left, 1=Right, 0xFF=All SINKs)
            uint8_t  volume_u8;  // Byte 4: Volume (0=Mute, 1=-96.0dB, 255=0.0dB)
            uint8_t  flags;      // Byte 5: Bit 0: 0=Smooth Slew, 1=Instant
            uint16_t reserved;   // Bytes 6..7: Word padding
        } ctrl;
    };
} vsaf_packet_t;
#pragma pack(pop)
```

---

## 6. Clock Synchronization & 50 ms Presentation Delay

### 6.1 Timeline Alignment
- **Master PTS Clock**: The SOURCE stamps each frame with `pts_us = now_us + 50000` (50 ms in the future).
- **SINK Phase-Lock**: Upon receiving a frame, the SINK compares local reception time with PTS to maintain sub-millisecond sync across all speakers.
- **Jitter Resilience**: The 50 ms cushion absorbs RF bursts, channel retransmissions, and FreeRTOS task scheduling jitter without draining the I2S DMA buffers.

---

## 7. I2S DMA Dual-Descriptor & Prefill Architecture

### 7.1 Strict 2-Descriptor DMA Setup
The I2S driver (`Hardware::I2sAudioDriver`) configures the ESP32 hardware DMA with **strictly 2 descriptors** (`chan_cfg.dma_desc_num = 2`).
- **Descriptor Length**: Exactly one 10 ms stereo audio frame (480 samples * 2 channels * 2 bytes = **1920 bytes** at 16-bit 48 kHz).
- **Interrupt Mode**: `on_sent` callback fires whenever a descriptor completes transmission.

```
 DMA Ping-Pong Ring:
 +---------------------------+---------------------------+
 |    Descriptor 0 (10ms)    |    Descriptor 1 (10ms)    |
 |  [ Currently Playing ]   |  [ Next Frame Preloaded ] |
 +---------------------------+---------------------------+
               |
               v (DMA Done Interrupt)
      "DMA Slot Free" Event
```

### 7.2 Pre-charging Strategy on Startup
1. In `SCANNING`, SINK accumulates incoming frames in `s_rx_fifo` until reaching `CONFIG_ESPNOW_PREFILL_THRESHOLD_FRAMES = 5` (50 ms).
2. SINK transitions to `PREFILL`, decodes Frame 0 into Descriptor 0, and Frame 1 into Descriptor 1 via `i2s_channel_preload_data()`.
3. SINK enables I2S hardware clocks (`i2s_channel_enable()`), achieving seamless, pop-free playback.

---

## 8. Interactive CLI Console Commands

| Command | Description |
| :--- | :--- |
| `vol <0..100>` | Set volume percentage (0 = Mute, 100 = 0 dBFS). Broadcasts from SOURCE or sets local SINK. |
| `voldb <-96..0>` | Set volume directly in dBFS (-96.0 dB to 0.0 dB). |
| `volu8 <0..255>` | Set raw 8-bit volume level directly. |
| `volch <ch> <0..255>` | Set volume for a specific channel (0: Left, 1: Right) from SOURCE. |
| `mute` / `unmute` | Smoothly mute or unmute audio using 96 dB/s slew rate. |
| `peer list` | Display registered SINK peers, session uptime, sent count, and ACK percentages. |
| `peer add <mac> <ch> [name]` | Register a new SINK peer (ch 0..5, e.g. `peer add B0:A6:04:99:38:44 0 Left`). |
| `peer del <mac>` | Unregister a SINK peer from the transmission list. |
| `peer enable <mac>` / `disable` | Temporarily enable or disable unicast transmissions to a specific peer. |
| `start` / `play` / `unicast` | Transition SOURCE to `CAST` / SINK to `SCANNING`. |
| `stop` / `pause` | Stop streaming and transition node to `IDLE`. |
| `phy <mc0..mc7|6m|12m|24m>` | Dynamically switch the 802.11 PHY modulation rate across all peers. |
| `octets <40..240>` | Dynamically reconfigure LC3 frame size (default: 120 bytes). |
| `sr <16k|24k|32k|44k|48k>` | Dynamically change audio sample rate across encoder and I2S DAC. |
| `gain <3|6|9|12>` | Adjust MAX98357A hardware DAC gain level in decibels. |
| `clear` / `cls` | Reset all transmission, ACK, PLC, and DMA error counters. |
| `diag` | Print full instant telemetry report. |
| `reset` / `reboot` | Reboot microcontroller. |

## 9. Dedicated Subwoofer Audio Channel Architecture (Channel ID 5)

To optimize overall network airtime while delivering uncompromised bass response, the system provides a dedicated **Subwoofer Channel** (Channel ID 5):

```
+---------------------------------------------------------------------------------------------+
|                                    SOURCE Subwoofer DSP Pipeline                            |
|                                                                                             |
|   Stereo Stream (48k)                                                                       |
|   [Left 480 samples]  ---\  Mono Sum      [480 samples @ 48kHz]    Downsampler (Factor 6)  |
|                           +-------------> [4th-Order LR4 LP]   --> [Decimate 48k -> 8k]     |
|   [Right 480 samples] ---/  (L + R) / 2   [fc = 100 Hz]            [80 samples @ 8kHz]      |
|                                                                             |               |
|                                                                             v               |
|                                                                     [LC3 8kHz Encoder]      |
|                                                                     [80 Octets / 10ms Frame]|
|                                                                             |               |
|                                                                             v               |
|                                                                    ESP-NOW Unicast (Ch 5)   |
+---------------------------------------------------------------------------------------------+

+---------------------------------------------------------------------------------------------+
|                                    SINK Subwoofer Playback Pipeline                         |
|                                                                                             |
|   ESP-NOW Unicast (Ch 5)                                                                    |
|   [80 Octets @ 8kHz] ---> [LC3 8kHz Decoder] ---> [80 samples @ 8kHz]                       |
|                                                          |                                  |
|                                                          v                                  |
|                                              [Volume Slew Limiter (96 dB/s)]                |
|                                                          |                                  |
|                                                          v                                  |
|                                              [I2S DMA Ring Buffer (Native 8kHz Stereo DAC)] |
+---------------------------------------------------------------------------------------------+
```

### 9.1 4th-Order Linkwitz-Riley Low-Pass Filter (LR4 LP)
- **Topology**: Cascaded pair of 2nd-order Butterworth low-pass biquads ($Q = 1/\sqrt{2} pprox 0.70710678$).
- **Direct Form II Transposed**: Numerically stable implementation processing 16-bit PCM.
- **Magnitude Response**:
  - **-6.02 dB** at cutoff frequency $f_c$ (`CONFIG_ESPNOW_SUB_LP_HZ`, default 100.0 Hz).
  - **-24.6 dB** at 1 octave above cutoff (200 Hz).
  - **-128.97 dB** at 4,000 Hz (Nyquist frequency of 8 kHz).
- **Anti-Aliasing Immunity**: The -129 dB attenuation at the 4 kHz Nyquist boundary ensures zero aliasing distortion when decimating directly by factor of 6.

### 9.2 Multi-Rate Resampling & LC3 Compression
- **10 ms Frame Geometry**: A 10 ms frame contains 480 samples at 48 kHz, which decimates to exactly **80 samples at 8 kHz**.
- **Bitrate & Airtime**: Encoding 80 samples @ 8 kHz into **80 octets per frame** yields a 64 kbps stream, preserving pristine bass definition while drastically reducing Wi-Fi packet airtime.
- **Native 8 kHz SINK Playback**: Subwoofer SINK nodes listening on Channel 5 configure their I2S DAC directly to 8 kHz. The decoded 80-sample 8 kHz frame is fed straight into the native 8 kHz dual-descriptor I2S DMA pipeline without any software upsampling.

### 9.3 Dynamic CLI Control
- `sublp <hz>`: Dynamically adjusts the Linkwitz-Riley cutoff frequency (20 Hz to 1,000 Hz) on the SOURCE in real time.
- `ch 5`: Configures a SINK speaker node to receive and decode the Subwoofer channel.
- `peer add <MAC> 5 [Name]`: Registers a dedicated Subwoofer peer node on the SOURCE.

---

## 10. USB Audio Class (UAC) Native Streaming Firmware

To eliminate scheduling jitter and UART bottlenecks associated with Python-based host streaming scripts, the SOURCE node firmware now fully supports acting as a native **USB Audio Class 1.0 (UAC1)** Speaker device when plugged into a PC.

```
+---------------------------------------------------------------------------------------------+
|                                  UAC1 Native Audio Streaming Pipeline                       |
|                                                                                             |
|   [Windows / macOS / Linux Host PC]                                                         |
|   (Recognizes ESP32-S3 as a 48kHz Stereo USB Speaker)                                       |
|               |                                                                             |
|               v (Isochronous USB Audio Data)                                                |
|                                                                                             |
|   +---------------------------------------+                                                 |
|   | ESP32-S3 SOURCE Node                  |                                                 |
|   |                                       |                                                 |
|   | [TinyUSB UAC1 Endpoint]               |                                                 |
|   |       |                               |                                                 |
|   |       v (48kHz 16-bit PCM Stream)     |                                                 |
|   | [FreeRTOS Stream Buffer]              |                                                 |
|   |       |                               |                                                 |
|   |       v                               |                                                 |
|   | [CAST Mode: usb_audio_read_pcm()]     |                                                 |
|   |       |                               |                                                 |
|   |       v (Synchronous 10ms Blocks)     |                                                 |
|   | [LC3 Dual-Core Encoders]              |                                                 |
|   |       |                               |                                                 |
|   |       v                               |                                                 |
|   | [ESP-NOW Unicast Dispatcher]          |                                                 |
|   +---------------------------------------+                                                 |
|               |                                                                             |
|               v (Wireless 802.11 LC3 Frames)                                                |
|       [SINK Nodes]                                                                          |
+---------------------------------------------------------------------------------------------+
```

### 10.1 Advantages over Python Serial Streamer
- **Zero OS Scheduling Jitter**: Traditional Python serial transmission suffers from non-real-time OS thread suspension. UAC relies on strictly timed USB isochronous hardware endpoints, guaranteeing flawless 10 ms real-time delivery.
- **Driverless Plug-and-Play**: The SOURCE node enumerates as a standard USB audio interface requiring no custom drivers or Python scripts.
- **Combined CDC Interface**: A composite USB device profile is utilized, presenting both the Audio Speaker (UAC1) interface and a Virtual Serial Port (CDC). This allows the standard diagnostic CLI to remain accessible simultaneously alongside the audio stream.
- **Automatic Fallback**: If the USB audio stream stops or underruns, the engine automatically falls back to generating a continuous internal test tone to maintain the 802.11 broadcast rhythm until the USB stream resumes.

### 10.2 Hardware Topology & COM Port Mappings (Node 16)
On the Seeed Studio XIAO ESP32-S3, the physical USB data lines (GPIO 19/20) are shared between the silicon ROM USB-Serial-JTAG controller and the internal USB OTG PHY:
- **Flashing / Download Mode (`COM16`)**:
  - Activated when holding the `B` (Boot) button and tapping `R` (Reset), via the `bootloader` CLI command, or via 1200-baud touch reset.
  - Enumerates as `USB\VID_303A&PID_1001` (ESP32-S3 ROM Bootloader) on **COM16**.
  - Flashed using `esptool.py` or the automated script `flash_s3.ps1`.
- **Runtime Application Mode (`COM116` + UAC1 Speaker)**:
  - After flashing, pressing the `R` button boots into flash firmware.
  - TinyUSB initializes the composite USB device (`USB\VID_303A&PID_4002`).
  - CDC Serial port enumerates on secondary port **COM116** (mapped in Device Manager).
  - Audio interface enumerates as a 48 kHz stereo USB speaker (`Node16 audio`).

### 10.3 Windows Audio Endpoint Configuration
- **Default Device**: In Windows Sound Settings, set `Speakers (Node16 audio)` as the Default Playback Device.
- **Sample Rate**: The UAC1 descriptor advertises 48,000 Hz, 16-bit stereo PCM.
- **Renaming the Audio Endpoint**:
  - If Windows caches the generic device name, open the classic Sound Control Panel (`mmsys.cpl`).
  - Under the **Playback** tab, double-click the speaker device, navigate to the **General** tab, and enter `Node16 audio`. Alternatively, use Windows Settings -> System -> Sound -> Output Properties -> Rename.

