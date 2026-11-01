# Embedded C++20 Audio DSP Implementation Walkthrough

## 1. Architecture & Files Created

All embedded C++20 DSP modules are implemented in `components/audio_dsp/` as a self-contained ESP-IDF / CMake component with zero dynamic memory allocation (`malloc`/`new`) in steady-state audio loops and lock-free `AtomicConfig<T>` parameter updates for FreeRTOS thread safety.

### Component Directory (`components/audio_dsp/`)
* [CMakeLists.txt](file:///c:/Git_audio_misc/components/audio_dsp/CMakeLists.txt) — Dual-mode build configuration supporting both ESP-IDF (`idf_component_register` when `ESP_PLATFORM` is set) and host C++20 unit/parity testing (`test_audio_dsp` + shared library `libaudio_dsp`).
* [dsp_primitives.hpp](file:///c:/Git_audio_misc/components/audio_dsp/include/dsp_primitives.hpp) & [dsp_primitives.cpp](file:///c:/Git_audio_misc/components/audio_dsp/src/dsp_primitives.cpp)
  * `g_fast_sin_table[130]`: Single RAM-resident (`DSP_DRAM_ATTR`, `alignas(16)`) 128-interval quarter-wave lookup table (`520` bytes) with a guard tap at index `129` (`1.0f`), shared across all channels and instances.
  * [`FastSinLUT`](file:///c:/Git_audio_misc/components/audio_dsp/include/dsp_primitives.hpp#L118-L207): `uint32_t` full-turn (`2^32 = 360 deg`) phase arithmetic (`sin_u32`, `cos_u32`, `sin_cos_u32`, `hz_to_inc_u32`, `deg_to_u32`, `shortest_arc_diff_s32`). Eliminates the Python `_lookup_q1` pre-clipping `frac` bug and returns exact `1.0f` at `90 deg` (`0x40000000u`) and `-1.0f` at `270 deg` (`0xC0000000u`) with `-94.50 dB` max error across the full circle.
  * [`BiquadDF2T`](file:///c:/Git_audio_misc/components/audio_dsp/include/dsp_primitives.hpp#L251-L295) & [`BiquadCascade`](file:///c:/Git_audio_misc/components/audio_dsp/include/dsp_primitives.hpp#L300-L408): Direct Form II Transposed 1st- and 2nd-order filter sections and cascades supporting Butterworth Low-Pass (`Order 4` with true Butterworth pole Q factors `0.5412` and `1.3066`) and configurable High-Pass (`Order 1..4`, cutoff `50..200 Hz`, default `Q = 0.707`).
  * [`HilbertIIRAllpass`](file:///c:/Git_audio_misc/components/audio_dsp/include/dsp_primitives.hpp#L419-L512): 8-section Niemitalo elliptic IIR Dual All-Pass cascade (`process_block` and `match_in_phase_block`) unrolled in scalar registers.
  * [`RecursiveNCO`](file:///c:/Git_audio_misc/components/audio_dsp/include/dsp_primitives.hpp#L520-L547) & [`EnvelopeFollower`](file:///c:/Git_audio_misc/components/audio_dsp/include/dsp_primitives.hpp#L552-L575): Coupled-form quadrature oscillator with periodic Taylor stabilization and dual-time-constant peak envelope tracker.
  * Saturating `int16_t <-> float` block converters (`pcm16_to_float_block`, `pcm16_stereo_to_float_lr`, `float_to_pcm16_block`) that clamp cleanly to `[-32768, +32767]` so `side_gain` up to `2.0` (`+6 dB`) never wraps around.
* [hilbert_quadrature_ms.hpp](file:///c:/Git_audio_misc/components/audio_dsp/include/hilbert_quadrature_ms.hpp) & [hilbert_quadrature_ms.cpp](file:///c:/Git_audio_misc/components/audio_dsp/src/hilbert_quadrature_ms.cpp)
  * [`HilbertQuadratureMs`](file:///c:/Git_audio_misc/components/audio_dsp/include/hilbert_quadrature_ms.hpp#L70-L281): Multi-channel (`2` to `8` output channels) Mid/Side quadrature spatial processor matching `process_quadrature_ms` in [hilbert_helper.py](file:///c:/Git_audio_misc/Signal_processing/hilbert_helper.py) and [hilbert_part3.ipynb](file:///c:/Git_audio_misc/Signal_processing/hilbert_part3.ipynb).
  * Computes the adjustable input HPF (`hpf_order` in `{1, 2, 3, 4}`, `hpf_cutoff_hz` in `[50, 200] Hz`, `hpf_q = 0.707f`), M/S decomposition, and shared Mid/Side Hilbert pairs **once** per `480`-sample (`10.0 ms`) block.
  * Maintains strictly isolated per-channel `lfo_acc` and `shift_acc` `uint32_t` phase accumulators in `states_[kMaxQuadratureChannels]` and dispatches each speaker channel to one of 3 specialized branchless inner loops (Static Surround, DQS + LFO, or Full DQS + LFO + DFS).
* [subharmonic_pre_stage.hpp](file:///c:/Git_audio_misc/components/audio_dsp/include/subharmonic_pre_stage.hpp) & [subharmonic_pre_stage.cpp](file:///c:/Git_audio_misc/components/audio_dsp/src/subharmonic_pre_stage.cpp)
  * [`SubharmonicPreStage`](file:///c:/Git_audio_misc/components/audio_dsp/include/subharmonic_pre_stage.hpp#L32-L133): Common pre-stage shared by both subharmonic pipelines: `48 kHz` stereo (`480` samples) -> mono (`0.5 * (L + R)`) -> configurable **4th-order Butterworth Low-Pass filter (`70–200 Hz`)** -> `6:1` decimation to **`8 kHz` (`80` samples per `10.0 ms` block)**.
* [subharmonic_zc_flipper.hpp](file:///c:/Git_audio_misc/components/audio_dsp/include/subharmonic_zc_flipper.hpp) & [subharmonic_zc_flipper.cpp](file:///c:/Git_audio_misc/components/audio_dsp/src/subharmonic_zc_flipper.cpp)
  * [`SubharmonicZcFlipper`](file:///c:/Git_audio_misc/components/audio_dsp/include/subharmonic_zc_flipper.hpp#L39-L108): Prototype 1 operating at **`8 kHz`** (`80` samples/block) with adaptive Schmitt-trigger hysteresis, alternating zero-crossing period-doubling sign toggle, and post-flip 8 kHz Butterworth LP smoothing filter.
* [subharmonic_nccf_synth.hpp](file:///c:/Git_audio_misc/components/audio_dsp/include/subharmonic_nccf_synth.hpp) & [subharmonic_nccf_synth.cpp](file:///c:/Git_audio_misc/components/audio_dsp/src/subharmonic_nccf_synth.cpp)
  * [`SubharmonicNccfSynth`](file:///c:/Git_audio_misc/components/audio_dsp/include/subharmonic_nccf_synth.hpp#L65-L202): Prototype 2 operating at **`8 kHz`** (`80` samples/block) with a `320`-sample (`40 ms`) ring buffer, 2-stage coarse-to-fine NCCF pitch detection (`k` in `[64, 229]` for `35–125 Hz`), `3 * k_naive` odd-harmonic disambiguation, parabolic lag refinement, 3-point median filter, `RecursiveNCO` at `f0 / 2`, and `< 45 Hz` sub-bass ducking.
  * [`SubharmonicPipeline`](file:///c:/Git_audio_misc/components/audio_dsp/include/subharmonic_nccf_synth.hpp#L210-L295): Unified wrapper combining the shared `SubharmonicPreStage` with mutually exclusive mode selection (`SubharmonicMode::{kBypass, kZcFlipper, kNccfSynth}`).
* [audio_dsp_c_api.h](file:///c:/Git_audio_misc/components/audio_dsp/include/audio_dsp_c_api.h), [audio_dsp_c_api.cpp](file:///c:/Git_audio_misc/components/audio_dsp/src/audio_dsp_c_api.cpp), [test_audio_dsp.cpp](file:///c:/Git_audio_misc/components/audio_dsp/test/test_audio_dsp.cpp), and [verify_cpp_parity.py](file:///c:/Git_audio_misc/Signal_processing/verify_cpp_parity.py) — C ABI bridge, C++20 unit test & benchmark suite, and Python-to-C++ numerical parity verification script.

---

## 2. Verification & Benchmark Results

### A. C++20 Unit Test & Benchmark Suite (`test_audio_dsp.exe`)
All 5 test suites in [test_audio_dsp.cpp](file:///c:/Git_audio_misc/components/audio_dsp/test/test_audio_dsp.cpp) passed with zero compiler warnings (`-Wall -Wextra -O3 -ffast-math`):

| Test Suite | Metric / Check | Result |
| :--- | :--- | :--- |
| **1. `FastSinLUT` & `uint32_t` Angles** | Cardinal exactness at `0, 90, 180, 270 deg` (`_lookup_q1` edge bug fix) | Exact `0.0f`, `+1.0f`, `0.0f`, `-1.0f` (**PASS**) |
| | Max error across `65,536` angles (`sin` & `cos`) | `1.88e-5` (**`-94.50 dB`**, **PASS**) |
| | `-180 deg == +180 deg` (`0x80000000u`) & shortest-arc `350 -> 10 deg` | Exact `0x80000000u` & `+20.0 deg` (**PASS**) |
| | Single RAM LUT instance address (`g_fast_sin_table`) | Verified (**PASS**) |
| **2. 10-Minute Phase-Drift Test** | `28,800,000` samples (`600 s` at `48 kHz`) at `3.0 Hz` & `0.4 Hz` | `< 1.1 deg` fixed quantization, **`0.0 deg` inter-channel drift** (vs `~107 deg` in `float32`, **PASS**) |
| **3. `HilbertQuadratureMs` (2–8 Ch)** | `dqs_deg = -90 / +90` vs `allpass(L / R)` (`hilbert_part3.ipynb` Cell 7) | L = **`-127.9 dB`**, R = **`-126.2 dB`** (**PASS**) |
| | `wet = 0.0` static surround invariance with LFO & DFS active | `0.0` difference (**PASS**) |
| | `side_gain = 0.0` mono Mid identity across all azimuths | `0.0` difference (**PASS**) |
| | `side_gain = 2.0` (`+6 dB`) `int16_t` saturation & HPF Orders `1..4` | Clean clamping to `[-32768, +32767]` (**PASS**) |
| | 8-channel LFO & DFS phase accumulator isolation | Strictly isolated (**PASS**) |
| **4. Shared `8 kHz` Subharmonic** | `SubharmonicZcFlipper` (`8 kHz`) on `80 Hz` stereo input | Halves `80.0 Hz` -> **`40.0 Hz`** (**PASS**) |
| | `SubharmonicNccfSynth` (`8 kHz`) on `80 Hz` stereo input | Tracks `f0 = 80.00 Hz`, synthesizes **`f_sub = 40.00 Hz`** (**PASS**) |
| **5. Throughput Benchmark** | 6-channel `HilbertQuadratureMs` (`480` samples, `int16` I/O) | **`23.45 us` / `10.0 ms` block** on host (`~0.28–0.34 ms` estimated on 240 MHz ESP32-S3, well under `< 1.0 ms` target) |

### B. Python <-> C++20 Numerical Parity Suite ([verify_cpp_parity.py](file:///c:/Git_audio_misc/Signal_processing/verify_cpp_parity.py))
Executed via `uv run python Signal_processing/verify_cpp_parity.py` loading `libaudio_dsp.dll`:
* **Parity Test 0 (`FastSinLUT` `uint32_t` batch)**: Max error **`-94.50 dB`**, `sin(90 deg) = 1.0000000`, `sin(270 deg) = -1.0000000`.
* **Parity Test 1 (6-Channel `HilbertQuadratureMs` vs `hilbert_helper.process_quadrature_ms` on 6-speaker ring with LFO + DFS)**:
  * Channel 0 (`-90 deg`): **`-89.00 dB`** relative RMS error
  * Channel 1 (`-30 deg`): **`-90.13 dB`** relative RMS error
  * Channel 2 (`+30 deg`): **`-91.89 dB`** relative RMS error
  * Channel 3 (`+90 deg`, `dfs = +0.4 Hz`): **`-92.64 dB`** relative RMS error
  * Channel 4 (`+150 deg`, `dfs = -0.4 Hz`): **`-91.36 dB`** relative RMS error
  * Channel 5 (`-150 deg`): **`-90.83 dB`** relative RMS error
* **Parity Test 2 (Shared `8 kHz` Subharmonic Pipelines)**:
  * `SubharmonicZcFlipper` (`8 kHz`) C++ vs Python relative RMS error: **`-74.90 dB`**
  * `SubharmonicNccfSynth` (`8 kHz`) steady-state tracked `f0`: **`80.001 Hz`** (error **`0.0014 Hz`**)
