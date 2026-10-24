# Project Goals

Semi-professional hobby project: a low-latency, multi-channel wireless speaker system over 802.11 (ESP-NOW) with LC3 coding.
Home use first (stereo up to 5.1); small PA / event / outdoor use is the later target.

Main track: `apps/audio_VSAF` (VSAF 3.0 broadcast). Everything else is either a support tool or archived.

## Targets

| Area | Target | How it is judged |
| :--- | :--- | :--- |
| End-to-end latency (audio in at SOURCE -> sound out of speaker) | **Hard: < 100 ms** (beat classic Bluetooth codecs). **Soft: < 50 ms** | Acoustic loopback measurement. The TV/analog path needs the low end (video lip-sync); the PC/USB path is less critical |
| Inter-speaker sync | **Within one to a few samples at 48 kHz** (about 20-60 µs), held over long sessions | Measured skew between two speakers, at start and after 10+ minutes (clock drift) |
| Reliability | **0 audible glitches per hour** at home distance (≤ 10 m, normal Wi-Fi around) | Frames recovered from redundancy are fine; every PLC (concealment) event counts as a glitch. Counted per SINK from telemetry |
| Audio quality | **Better than classic Bluetooth codecs, near-transparent in good radio conditions** | Dropout-free transmission takes priority over codec transparency when the two conflict |
| Scale | Home: up to 6 channels (5.1). PA: **≤ 30 m, ≤ 8 speakers** | Range and speaker count beyond this are out of scope |

## Audio sources

1. **PC via USB** (UAC1 speaker on the SOURCE node). Latency less important.
2. **TV / analog input.** Needs low latency for video sync. Needs ADC (or S/PDIF) hardware on the SOURCE.
3. **Network streams** (Spotify/AirPlay/DLNA style). Handled by a separate node that pipes audio to the SOURCE over I2S, not by the SOURCE itself.

## Spatial audio

Multi-channel spatial effects: **stereo -> multi-channel upmix on the SOURCE** (before LC3 encoding), so SINKs stay simple.
This is what the tight inter-speaker sync target exists for.

## Pushing the platform to its limits

We will squeeze every drop of performance out of the ESP32-S3 nodes. One resource-constrained dual-core SoC (Xtensa LX7 @ 240 MHz + FPU) has to do all of this at once, in real time:
- run the VSAF2 protocol rock-solid over Wi-Fi (802.11 broadcast, telemetry, time sync),
- run spatial audio algorithms based on the Hilbert transform (stereo -> multi-channel upmix),
- run multi-channel LC3 encoders, including the multi-bitrate redundant frames (today 5 encoder instances per frame on the SOURCE).

Note that the SOURCE (Node 16, XIAO ESP32-S3) has no PSRAM: 512 KB SRAM and 4 MB flash. The XIAO S3 Plus SINKs have 8 MB PSRAM.

### The 10 ms metronome
Audio frames are sent every 10 ms, and that frame period is the master metronome for all firmware tasks. **Every task must finish strictly within the 10 ms budget**, including sending all VSAF packets over 802.11. Overrunning a frame is a bug, not a performance issue: it shows up as late packets, FIFO underruns and PLC on the SINKs.

### Low-level optimization is a first-class concern
Getting all components to work together seamlessly depends on:
- **Memory allocation:** static, zero-allocation steady state in the audio path (no `malloc`/`new` inside per-frame loops).
- **Memory placement:** deliberate IRAM / internal SRAM / PSRAM / flash placement of hot code and buffers (DMA buffers and per-frame state in internal SRAM; PSRAM only for cold or bulk data).
- **Cache behavior:** keep hot loops and their data cache-friendly; avoid flash/PSRAM cache misses in the frame path.
- **Thread safety:** lock-free or minimal-lock hand-off between ISR, Wi-Fi callbacks and tasks; no priority inversion.
- **Context switches:** deliberate core pinning and task priorities, event-driven wake-ups (task notifications from the frame timer) instead of polling or spin-waits.
- **Compiler optimizations:** per-component optimization levels and target-specific code (FPU, SIMD/DSP instructions, `IRAM_ATTR`), always checked against measurements.

Each optimization should be justified by a measurement (per-task worst-case execution time per frame, CPU load per core, free internal heap), reported in the 1 s heartbeat telemetry. Never trust an average: the worst-case frame decides.

## Priorities (next 1-3 months)

1. **Measure and verify** the targets above, turning current claims into numbers:
   - end-to-end latency,
   - inter-speaker skew and clock drift between SINKs (no sample-clock correction exists today; decide on a fix only after measuring),
   - dropout / PLC rate over long runs.
   Then keep these as repeatable regression tests.
2. **Spatial upmix** on the SOURCE.

Later: TV/analog input hardware, network-stream node, full 5.1 bring-up, PA scaling (≤ 30 m, ≤ 8 speakers).

### Measurement equipment available
- Dayton Audio UMM-6 USB measurement microphone (acoustic latency and speaker-to-speaker arrival skew).
- DATS V3 computer-based audio component test system.
- General electrical instruments, but no multi-channel oscilloscope. Firmware-side GPIO/serial timestamps can complement the acoustic measurements.

## Known gaps / open questions
- **Clock drift:** each SINK runs from its own crystal (20 ppm parts drift by roughly a sample every 1-2 s against each other). Only timestamps are aligned today. Possible fixes after measuring: APLL clock steering (S3 only, C6 has no APLL), adaptive resampling, or sample drop/insert.
- The README claim of ±10 µs PTP sync has not yet been verified with an acoustic or electrical measurement.

## Non-goals
- Bluetooth LE Audio / Auracast transport (BLE apps archived).
- ACK-based multi-unicast transport and the older dual-frame broadcast (archived).
- Runtime provisioning / auto-discovery: node identity, channel and gain stay in MAC-address tables in firmware.
- Range beyond 30 m, more than 8 speakers, 5 GHz operation.

The retired apps live in [`archive/`](archive/README.md) and are no longer maintained (no ESP-IDF bumps, no fixes).
