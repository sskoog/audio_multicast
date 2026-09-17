# win11audio
A simple Python application for debugging DSP audio alrorithms written in C++ by capturing sound via WASAPI on Windows using pybind11 and sounddevice.

The intended DSP algorithms are partially intended for spatial audio effects, where a mono or stereo input stream is processed and output as a multi-channel (4+) audio streams. See example implementation: [forestChirp](https://github.com/sskoog/forestChirp/). 

A secondary feature of this app is to send processed audio via Bluetooth Low Energy Audio Broadcast (a.k.a. Auracast). This requires support for Bluetooth 5.3 or 5.4 on the target platform in both hardware and driver version.

## DSP functions
Glossary and definitions:
* MS: Mid/Side extraction ( mid = 0.5*(L+R), side = 0.5*(L-R) )
* Dry input: Raw input signal
* Wet output: Output signal after processing
* DQS: Dynamic Quadrature Splitter
* DFS: Dynamic Frequency Shifter
* LFO: Low frequency oscillator
* Hilbert Transform: Digital all-pass filter with a constant 90 degree phase shift across the audible frequency band
* HPF: High-pass filter
* SSB: Single-Sideband


## Spatial audio functions:

### Input mixing (passive matrix) to DRY signal:
1. Mono: Mono input --> dry0 = dry1 = Mono
2. Stereo: Stereo input --> dry0 = Left, dry1 = Right
3. Mid-side (MS): Stereo --> dry0 = 0.5*(L+R), dry1 = 0.5*(L-R)

### Pre-Hilbert conditioning:
The Hilbert transform is implemented through a digital all-pass filter with virtually flat frequency response across the audible band, but with a constant 90 degree phase shift. However, making this filter cover the low-end frequency spectrum is a complex task. Since low-end audio is not expected to contain any stereo imaging information, a HPF of ~150 Hz is applied to the dry input signals before feeding to the Hilbert transform.
To maintain the low-end, a dry LP-signal must be added to the wet output signal after processing. 

Hence:
1. Mono --> dry0_HPF & dry0_LPF
2. Stereo --> dry0_HPF, dry1_HPF, dry0_LPF, dry1_LPF
3. MS --> mid_HPF, side_HPF, mid_LPF, side_LPF

When using MS-mode, only the side channel is processed by the Hilbert transform. The mid channel is passed through untouched to preserve the mono (center) content, which serves as an anchor for the stereo image in all output channels. The mixing factor ''mix_ms'' between center and wet side channel can be adjusted (default is 0.5).

### Spatial audio algorithms 
0. All bypass (raw input --> output)
1. DQS + DFS (fixed-value phase per output channel + frequency shift per output channel)
  - Basic: Fixed phase shift and no frequency shift
  - Basic+: Fixed phase shift and fixed frequency shift for all output channels
  - Swirl/Distributed Chorus: Fixed phase shift and slow frequency shift (~0.3 Hz) for all output channels
  - Spatial rotary: Apply ultra-low freq oscillator to DQS phase with unique prime frequencies for all output channels (e.g. 0.11 - 0.22 Hz)
  - Fractional Heterodyning: No DQS, apply fixed fractional DFS-shift across all outputs, e.g. [0, 0.4, 0.8, 1.2] Hz

### Output mixing
Assuming stereo output:
1. Mono
  - out0 = dry0_LPF + wet0_HPF
2. Stereo
  - out0 = dry0_LPF + wet0_HPF
  - out1 = dry1_LPF + wet1_HPF
3. MS
  - out0 = mid_LPF  + wet0_HPF
  - out1 = side_LPF + wet1_HPF


---

## Architecture Overview

```
                        [ Stereo Input Stream ]
                     (WASAPI Loopback / Mic In)
                                 │
                                 ▼
                     ┌───────────────────────┐
                     │    Biquad 100Hz HPF   │  (Direct Form II Transposed)
                     └───────────┬───────────┘
                                 │
                                 ▼
                     ┌───────────────────────┐
                     │ Mid / Side Extraction │  (Mid = 0.5*(L+R), Side = 0.5*(L-R))
                     └───────────┬───────────┘
                                 │
                                 ▼
                     ┌───────────────────────┐
                     │  Hilbert Transformer  │
                     │                       │
                     │ • FIR: 31-Tap Linear  │  (Exact 15-sample linear phase delay)
                     │ • IIR: 2x3 Allpass    │  (Low-weight cascaded allpass network)
                     └───────────┬───────────┘
                                 │
                                 ▼
                     ┌───────────────────────┐
                     │    Mixing / Modulator │
                     │                       │
                     │  • Mode 0: Phase Shift│  (Static Phase Rotation)
                     │  • Mode 1: LFO Phase  │  (Low Frequency Phase Modulator)
                     │  • Mode 2: SSB DFS    │  (Single-Sideband Frequency Shift)
                     └───────────┬───────────┘
                                 │
                                 ▼
                     ┌───────────────────────┐
                     │ Stereo Reconstruction │  (L = Mid + Side, R = Mid - Side)
                     └───────────┬───────────┘
```

### Components

1. **`dsp_engine.cpp`**:
   - **`HilbertFIR`**: 31-tap Type-III FIR filter with Blackman window and exact 15-sample circular buffer delay line for flat linear group delay.
   - **`HilbertIIRAllpass`**: 2x3-pole cascaded allpass IIR filter network from `forestChirp/audio_output.h` with near-zero latency.
   - **`FastTrigLUT`**: 1024-point precomputed sine/cosine lookup table with linear interpolation.
   - **`RecursiveQuadratureOscillator`**: Phasor oscillator with continuous 1st-order Pade amplitude stabilization.
   - **`BiquadHPF`**: Direct Form II Transposed 2nd-order 100 Hz high-pass Butterworth filter.
   - **`AudioDSPPipeline`**: C++ pipeline exposed to Python through `pybind11`.

2. **`dsp_app.py`**:
   - Python frontend for impulse-response transfer function verification and real-time WASAPI loopback streaming.

---

## Requirements & Virtual Environment Setup

### 1. Activate the Virtual Environment:
```powershell
# From the repository root
.\.venv_audio_multicast\Scripts\Activate.ps1

# Or from the win11audio app folder:
..\..\.venv_audio_multicast\Scripts\Activate.ps1
```

### 2. Install / Verify Dependencies:
```powershell
pip install -r requirements.txt
```

### 3. Compile C++ Extension:
```powershell
powershell -ExecutionPolicy Bypass -File .\build.ps1
```

---

## Running the Application

### 1. List Audio Devices
```powershell
python dsp_app.py --list-devices
```

### 2. Analytical Verification (Transfer Function & Group Delay)
```powershell
# Verify using Linear-Phase FIR Hilbert
python dsp_app.py --mode analyze --hilbert fir --save-plot analysis_fir.png

# Verify using Low-Weight Cascaded Allpass IIR Hilbert
python dsp_app.py --mode analyze --hilbert iir --save-plot analysis_iir.png
```

### 3. Real-Time Streaming with Spatial Algorithms
```powershell
# Intercept PC Audio Output via WASAPI Loopback with MS Matrix, Swirl Algorithm, and IIR Allpass
python dsp_app.py --mode stream --loopback --input-mode ms --algo swirl --hilbert iir

# Spatial Rotary (prime LFO rates per channel) with 4 output channels
python dsp_app.py --mode stream --loopback --input-mode ms --algo rotary --channels 4 --rotary-scale 1.0

# Fractional Heterodyning (stepped DFS offsets: 0, 0.4, 0.8, 1.2 Hz)
python dsp_app.py --mode stream --loopback --input-mode ms --algo heterodyne --channels 4 --dfs-step 0.4
```

---

## CLI Options Reference

| Argument | Description | Default |
|---|---|---|
| `--mode` | Execution mode: `analyze` or `stream` | `analyze` |
| `--input-mode` | Passive matrix: `mono`, `stereo`, or `ms` (Mid/Side) | `ms` |
| `--algo` | Spatial algorithm: `bypass` (0), `basic` (1), `basic+` (2), `swirl` (3), `rotary` (4), `heterodyne` (5) | `swirl` |
| `--hilbert` | Hilbert transform: `fir` (31-tap FIR) or `iir` (2x3 allpass IIR) | `fir` |
| `--channels` | Number of output channels (e.g. 2, 4, 5, 8) | `2` |
| `--mix-ms` | Mixing factor between center anchor and wet side channel (0.0 to 1.0) | `0.5` |
| `--hpf-cutoff` | High-pass / Low-pass crossover frequency in Hz | `150.0` |
| `--phase-shift-deg`| Static phase angle in degrees | `45.0` |
| `--dfs-offset` | Frequency shift offset in Hz (for `basic+` and DFS) | `5.0` |
| `--dfs-step` | Fractional DFS step in Hz across channels (for `heterodyne`) | `0.4` |
| `--rotary-scale` | Speed multiplier for prime LFO rotary rates | `1.0` |
| `--rotary-depth-deg`| Rotary phase modulation depth in degrees | `90.0` |
| `--loopback` | Enables Windows WASAPI loopback capture (captures speaker audio) | `False` |
| `--device` | Input capture device ID number or substring name match | Default input device |
| `--cable` | Auto-detects and binds to VB-Audio Virtual Cable input | `False` |
| `--out-device` | Output playback device ID number or substring name (e.g. `'Speakers'`, `'Bose'`) | Default physical output |
| `--no-playback` | Disables audio output playback (monitor RMS levels only) | `False` |
| `--volume` | Output playback volume multiplier | `1.0` |
| `--save-plot` | Path to save analysis plot image (e.g. `analysis_spatial.png`) | `None` |
| `--list-devices` | Lists all audio input and output devices | `False` |

# PC setups

## Using a virtual audio cable in Win11
1. **Install Virtual Cable**: Download and install [VB-CABLE Virtual Audio Device](https://vb-audio.com/Cable/index.htm).
2. **Route Specific App Audio**:
   - Open Windows 11 **Settings > System > Sound > Volume mixer**.
   - Under **Apps**, find your web browser or media player (e.g. Chrome, Edge, Spotify).
   - Set its **Output device** to **`CABLE Input (VB-Audio Virtual Cable)`**.
3. **Run `dsp_app` with Virtual Cable Input & Real-Time Playback**:
   ```powershell
   # Automatically capture from Virtual Cable and play spatial audio out to speakers/headphones:
   python dsp_app.py --mode stream --cable --algo swirl

   # Route playback to a specific Bluetooth headset:
   python dsp_app.py --mode stream --cable --out-device "Bose" --algo rotary
   ```

## Win11 PC SS-S9 specs
* Device name: SS-S9
* Microsoft Surface Pro 9 for Business Model 2038 i7
* Product ID: 00330-66940-27850-AAOEM
* BIOS Version: 24.103.143 (2026-01-07)
* SAM: 6.502.139
* UEFI: 24.103.143
* OS: Windows 11 Pro 25H2 (26200.9168)
* CPU: 12th Gen Intel (Alder Lake-U) Core i7-1265U (2.70 GHz)
* RAM: 16 GB
* SSD: 0.5 TB
* Graphics card: Intel Iris Xe Graphics (128 MB)
* Wi-Fi driver: 23.160.0.4
* Intel Bluetooth chip: Intel Wi-Fi 6E AX211, driver version: 24.40.11.1

Windows 11 drivers does not natively support BLE Audio Broadcasting with the setup listed above.
Microsoft has introduced experimental Auracast broadcasting in Windows 11 Insider preview builds for specific Copilot+ PCs (Qualcomm Snapdragon X Elite / select Intel Core Ultra).

Best alternative to broadcast PC audio to multiple Bluetooth speakers is to use a dedicated BLE-audio 5.3/5.4 USB dongle or an **ESP32-C6 DevKit HCI Controller** with Google Bumble (see [`apps/usb_ble_bumble`](../usb_ble_bumble)).

---

## BLE Audio & LC3 Codec Engine Details

When broadcasting audio via Bluetooth Low Energy Audio (Auracast / BAP / PBP), the system implements a **Split Host/Controller Architecture**:

```
+-------------------------------------------------------------------------------+
| HOST PC (Windows 11)                                                          |
|                                                                               |
|  1. Audio Capture (WASAPI Loopback or VB-Audio Virtual Cable)                 |
|     * Captures 48 kHz, 16-bit PCM audio                                       |
|                                                                               |
|  2. Spatial DSP Pipeline (Mid/Side + DQS + Hilbert + DFS Modulation)          |
|     * Produces stereo or multi-channel spatial PCM audio                      |
|                                                                               |
|  3. LC3 Codec Engine (Google liblc3 running on PC CPU via native C DLL)      |
|     * Compresses 480 PCM samples (960 B/ch) -> 60 B mono / 120 B stereo SDU   |
|                                                                               |
|  4. Google Bumble Bluetooth Host Stack                                        |
|     * Packs compressed LC3 frames into HCI ISO Data Packets (H4 type 0x05)   |
|     * Manages Extended (EA) and Periodic Advertising (PA + BAP BASE)          |
+---------------------------------------┬---------------------------------------+
                                        │ USB Serial (H4 Transport @ 115200 baud)
+---------------------------------------▼---------------------------------------+
| ESP32-C6 DevKit (HCI Link Layer Controller)                                  |
|                                                                               |
|  * Does NO audio encoding or DSP processing on-chip.                          |
|  * Receives pre-compressed LC3 ISO packets from UART0.                        |
|  * Schedules and transmits packets over 2.4 GHz RF Link Layer (BIS / BIG).    |
|  * Status LED feedback: 0.5 Hz Green (Idle), 3.0 Hz Blue (Transmitting).      |
+-------------------------------------------------------------------------------+
```

### LC3 Codec Implementation

* **Library**: [Google `liblc3`](https://github.com/google/liblc3) (Google's official, high-performance C reference implementation of the **Bluetooth SIG Low Complexity Communication Codec** standard).
* **Location in Repo**: [`apps/usb_ble_bumble/liblc3/`](../usb_ble_bumble/liblc3/).
* **Native Shared Library**: [`apps/usb_ble_bumble/liblc3.dll`](../usb_ble_bumble/liblc3.dll) (compiled for 64-bit Windows with MinGW GCC).
* **Python Binding**: [`apps/usb_ble_bumble/lc3_encoder.py`](../usb_ble_bumble/lc3_encoder.py) via Python `ctypes`.
* **Standard Audio Frame Format**:
  * Sampling Rate: **48,000 Hz** (or 44.1 kHz, 32 kHz, 24 kHz, 16 kHz)
  * Frame Duration: **10.0 ms** (480 PCM samples per frame per channel)
  * Bitrate / Frame Size:
    * **Mono Stream**: 60 octets per frame (48 kbps)
    * **Stereo Stream**: 120 octets per frame (96 kbps total, 60 bytes Left + 60 bytes Right)
    * **High-Quality Stereo**: 200 octets per frame (160 kbps total)
* **Encoding Latency**: Less than **0.1 ms** per 10 ms audio frame on the host PC CPU.







