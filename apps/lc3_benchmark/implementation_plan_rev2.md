# Implementation Plan: ESP32-S3 LC3 Codec Benchmark Suite (Rev 2)

## 1. Overview & Objectives
This plan outlines the architecture, implementation, and automated test execution of a benchmark suite for the **Bluetooth Low Complexity Communication Codec (LC3)** on the **ESP32-S3 (Xtensa LX7 dual-core @ 240 MHz, hardware FPU)**.

The benchmark evaluates the single-core CPU execution time and latency for encoding and decoding under two critical performance optimizations:
1. **IRAM vs SPI Flash Code Placement**: Linker-enforced placement in internal SRAM (`noflash`) vs SPI Flash XIP (`default`) with instruction cache.
2. **LTPF (Long Term Postfilter / Pitch Filter) ON vs OFF**: Quantifying the speedup achieved when disabling the computationally intensive pitch analysis stage (`lc3_encoder_disable_ltpf()`).
3. **Floating-Point (`liblc3` + FPU) vs Fixed-Point (`esp_audio_codec`)**: Direct comparison across all quality profiles.
4. **Encode vs Decode Execution**: Measuring both encoding to LC3 bitstream and decoding back to PCM across 100 consecutive dynamic music frames.

---

## 2. Test Matrix Specifications

### Quality Levels & Bitrate Profiles
| Profile Level | Target Sample Rate | Frame Duration | Octets / Frame | Bitrate (kbps) | Samples / Frame (PCM) |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **VHQ** | 48.0 kHz | 10.0 ms / 7.5 ms | 160 Bytes | 128.0 kbps / 170.67 kbps | 480 / 360 |
| **HQ** | 48.0 kHz | 10.0 ms / 7.5 ms | 120 Bytes | 96.0 kbps / 128.0 kbps | 480 / 360 |
| **MQ** | 32.0 kHz | 10.0 ms / 7.5 ms | 80 Bytes | 64.0 kbps / 85.33 kbps | 320 / 240 |
| **LW** | 24.0 kHz | 10.0 ms / 7.5 ms | 60 Bytes | 48.0 kbps / 64.0 kbps | 240 / 180 |
| **SUB1** | 8.0 kHz | 10.0 ms / 7.5 ms | 80 Bytes | 64.0 kbps / 85.33 kbps | 80 / 60 |
| **SUB2** | 8.0 kHz | 10.0 ms / 7.5 ms | 120 Bytes | 96.0 kbps / 128.0 kbps | 80 / 60 |

### Linear Combinations per Profile
Each of the 6 quality levels will be tested under all linear combinations:
* **Frame Durations**: `[7.5 ms, 10.0 ms]` (2 options)
* **Codec Engines & LTPF Modes**:
  1. `liblc3` (Google Float + Hardware FPU) with **LTPF ON**
  2. `liblc3` (Google Float + Hardware FPU) with **LTPF OFF**
  3. `esp_audio_codec` (Espressif Fixed-Point)
* **Code Memory Placement**:
  1. **IRAM Placement**: Code executed from internal SRAM (`noflash` linker fragment).
  2. **SPI Flash Placement**: Code executed from external SPI Flash via cache (`default` linker fragment).

**Total Benchmark Runs**: 6 quality levels * 2 frame durations * 3 engine modes * 2 memory placements = **72 benchmark runs**.
Each run processes **100 consecutive frames** of dynamic audio for encode, and **100 consecutive frames** for decode.

---

## 3. Audio Test Clips & Preprocessing
* **Source Clip**: [10s 48 kHz mono CLIP Monster.wav](file:///c:/Git_audio_multicast/data/10s%20clips/10s%2048%20kHz%20mono%20CLIP%20Monster.wav) (16-bit mono PCM).
* **Extraction**: Extract a 4.0-second high-energy dynamic middle section (from 2.0s to 6.0s) to avoid silent lead-in.
* **Resampling**: Resample into 4 mono 16-bit PCM streams:
  - 48 kHz (192,000 samples = 384 KB)
  - 32 kHz (128,000 samples = 256 KB)
  - 24 kHz (96,000 samples = 192 KB)
  - 8 kHz (32,000 samples = 64 KB)
* **Packaging**: Pack all 4 clips into `data/benchmark_clips_rev2.bin` with standard 4 KB header structure and burn to the `storage` partition at `0x190000`.

---

## 4. Benchmark Execution Sequence per Configuration
For each test configuration on a dedicated core (Core 1):
1. **Load PCM to RAM**: Read 100 frames worth of raw PCM samples from the flash storage partition into a preallocated static DRAM buffer.
2. **Setup Encoder**: Initialize `lc3_setup_encoder` (or `esp_lc3_enc_open`). If LTPF is disabled, call `lc3_encoder_disable_ltpf()`.
3. **Encode Measurement**:
   - Record `t_enc_start = esp_timer_get_time()`.
   - In a tight loop, encode 100 frames into a preallocated RAM buffer (`100 * octets` bytes).
   - Record `t_enc_end = esp_timer_get_time()`.
   - Calculate total duration, average duration per frame (microseconds), and CPU load percentage.
4. **Setup Decoder**: Initialize `lc3_setup_decoder` (or `esp_lc3_dec_open`).
5. **Decode Measurement**:
   - Record `t_dec_start = esp_timer_get_time()`.
   - In a tight loop, decode 100 LC3 frames from RAM into a preallocated scratch PCM buffer.
   - Record `t_dec_end = esp_timer_get_time()`.
   - Calculate total duration, average duration per frame (microseconds), and CPU load percentage.
6. **Telemetry Output**: Stream structured JSON/CSV records over USB-CDC/Serial for automated capture.

---

## 5. Proposed Changes

### Component 1: Firmware Codec Benchmark Suite (`apps/lc3_benchmark`)
#### [MODIFY] [CMakeLists.txt](file:///c:/Git_audio_multicast/apps/lc3_benchmark/CMakeLists.txt)
- Add target build configuration flags to toggle between IRAM (`noflash`) and Flash (`default`) linker rules.

#### [MODIFY] [main/CMakeLists.txt](file:///c:/Git_audio_multicast/apps/lc3_benchmark/main/CMakeLists.txt)
- Ensure registration of `liblc3` and `espressif__esp_audio_codec` components.

#### [MODIFY] [main/main.cpp](file:///c:/Git_audio_multicast/apps/lc3_benchmark/main/main.cpp)
- Set up dedicated task on Core 1 at high priority (priority 5 or 10).
- Keep Core 0 for serial monitoring and system housekeeping with Wi-Fi disabled.

#### [MODIFY] [main/lc3_benchmark_runner.hpp](file:///c:/Git_audio_multicast/apps/lc3_benchmark/main/lc3_benchmark_runner.hpp) & [lc3_benchmark_runner.cpp](file:///c:/Git_audio_multicast/apps/lc3_benchmark/main/lc3_benchmark_runner.cpp)
- Refactor runner to implement the 6 quality levels (VHQ, HQ, MQ, LW, SUB1, SUB2).
- Add LTPF disable support (`lc3_encoder_disable_ltpf`).
- Add decode benchmark measurement alongside encode benchmark.
- Format results as structured telemetry rows.

### Component 2: Audio Resampling & Flash Packer
#### [NEW] [tools/pack_benchmark_clips_rev2.py](file:///c:/Git_audio_multicast/tools/pack_benchmark_clips_rev2.py)
- Slices the middle section of the reference clip and resamples into 48, 32, 24, and 8 kHz mono 16-bit PCM.
- Writes binary partition image `data/benchmark_clips_rev2.bin`.

### Component 3: Automation, Analysis & Report Generation
#### [NEW] [apps/lc3_benchmark/run_s3_benchmark_suite.py](file:///c:/Git_audio_multicast/apps/lc3_benchmark/run_s3_benchmark_suite.py)
- Orchestrates the full test:
  1. Packs and flashes audio clips to ESP32-S3 `storage` partition.
  2. Compiles & flashes IRAM build to Node 16 (using RTC Watchdog hands-free reset).
  3. Captures IRAM test results via COM116 / COM16.
  4. Compiles & flashes Flash build to Node 16.
  5. Captures Flash test results via COM116 / COM16.
  6. Saves raw log files in `apps/lc3_benchmark/tests/`.
  7. Generates matplotlib comparison plots.
  8. Generates the comprehensive report [docs/lc3_encoder_ESP32_S3_rev2.md](file:///c:/Git_audio_multicast/docs/lc3_encoder_ESP32_S3_rev2.md).

---

## 6. Verification Plan

### Automated Steps
1. **Audio File Packing**: Run `python tools/pack_benchmark_clips_rev2.py` and verify generated clip sizes and sample counts.
2. **Build Verification**: Compile both IRAM and Flash targets using ESP-IDF v6.0.2 with `idf.py -B build_s3 build`.
3. **Flashing & Test Execution**: Run `python apps/lc3_benchmark/run_s3_benchmark_suite.py` to flash Node 16 and capture serial data.
4. **Data Verification**: Verify that all 72 test rows are successfully received, valid, and contain non-zero durations.
5. **Plot & Report Validation**: Verify that matplotlib plots and markdown report are generated in `docs/` and `apps/lc3_benchmark/tests/`.
