# SOURCE audio pipeline runs too slow!
2026-09-18

Running 3 encoders, the 6-packet broadcaster and USB audio host is too much for the ESP32-S3 @ 240 MHz!

## Baseline
Input stereo audio via TinyUSB @ 48 kHz 
--> DSP (HP L+R and sub) 
--> LC3-encode ch1 @ core0
--> LC3-encode ch2 @ core1
--> LC3-encode ch3 @ core1
Run VSAF broadcast to 6 targets through semaphore-wait for RF TX module

LC3-encode parameters:
* 48 kHz, 16 bits, 120 octets, 10 ms

Timings:
* DSP: 0.60 ms
* LC3-encode time: 4.0 - 5.0 ms (same for all 3 ch)
* TX: 5.3 - 6.6 ms

```
+=================================================================== ESP32-S3-SOURCE [SOURCE] ===================================================================+
|    CPU      | STATE | NODES  |    WIFI     |  AUDIO dBFS  |     STAGE TIMINGS (ms)       |  SOURCE      PKTS  ACK%  FAIL   TOT  |             ROUND-TRIP NET (us)        |
|  %   C  MHz |       | 012345 | GAIN Ch PHY |   RMS    Pk  |  DSP   Enc1  Enc2  Enc3   TX  |  INPUT        1/s   1/s   1/s  pkts   |              L_Net       R_Net         |
| 97  66  240 | CAST  | 11OOOO | +3.0 04 HT3 | -44.0 -32.2 | 0.59  5.01  4.48  3.93  5.31|  USB        272  100%     0   320K |               2310        2094         |
| 96  66  240 | CAST  | 11OOOO | +3.0 04 HT3 | -46.6 -35.4 | 0.59  4.38  4.08  4.25  5.64|  USB        287  100%     0   320K |               2310        2094         |
| 96  66  240 | CAST  | 11OOOO | +3.0 04 HT3 | -45.1 -33.5 | 0.57  4.62  4.19  4.22  5.65|  USB        281  100%     0   321K |               2310        2094         |
| 98  66  240 | CAST  | 11OOOO | +3.0 04 HT3 | -44.8 -34.9 | 0.61  4.03  4.23  4.46  5.90|  USB        293  100%     0   321K |               2310        2094         |
| 97  66  240 | CAST  | 11OOOO | +3.0 04 HT3 | -44.2 -34.3 | 0.60  4.22  4.36  4.12  5.90|  USB        302  100%     0   321K |               2310        2094         |
| 96  66  240 | CAST  | 11OOOO | +3.0 04 HT3 | -44.8 -34.4 | 0.59  4.54  4.15  4.19  5.59|  USB        290  100%     0   322K |               2310        2094         |
| 96  66  240 | CAST  | 11OOOO | +3.0 04 HT3 | -45.9 -34.6 | 0.59  4.37  4.03  4.11  5.60|  USB        297  100%     0   322K |               2310        2094         |
| 97  66  240 | CAST  | 11OOOO | +3.0 04 HT3 | -48.0 -37.7 | 0.60  4.23  4.05  4.38  5.78|  USB        301  100%     0   322K |               2310        2094         |
| 98  66  240 | CAST  | 11OOOO | +3.0 04 HT3 | -46.4 -33.6 | 0.60  4.34  4.21  4.37  5.55|  USB        297  100%     0   323K |               2310        2094         |
| 97  66  240 | CAST  | 11OOOO | +3.0 04 HT3 | -46.4 -34.0 | 0.59  4.45  4.39  4.25  5.74|  USB        274  100%     0   323K |               2310        2094         |
```

## Moving liblc3 to IRAM instead of SPI flash

SOURCE Timings (streaming PC audio):
* DSP: 0.45 ms
* LC3-encode time: 1.47 - 1.48 ms (0.8 ms for sub)
* TX: 4.5 ms
--> DRAMATIC improvement of LC3-encode speed!

SINK timings before: 2.54 ms
SINK timings after: 1.25 ms
--> Victory! ca x2 faster!

## Disable LTPF (lc3_encoder_disable_ltpf)
"Disable LTPF (Long Term Pitch Filter) analysis for significant encoding speedup"

Timings (streaming PC audio / tone):
* DSP: 0.45 - 0.70 ms  
* LC3-encode: 0.94 ms (0.39 ms for 8 kHz subwoofer)
* TX: 4.0 - 4.3 ms

### Audio Quality Impact Analysis:
1. **At 96 kbps (120 octets @ 48 kHz / 10 ms)**: The MDCT bit budget is plentiful. LTPF is primarily designed for speech pitch prediction at ultra-low bitrates (16–32 kbps). In standard Bluetooth LC3 listening evaluations at 96 kbps, LTPF difference on music is transparent / indistinguishable.
2. **Synthetic Tones & Transients**: Disabling LTPF avoids false pitch lag autocorrelation and transient smearing on synthetic tones and percussion.
3. **Bluetooth Standard Compliance**: Disabling LTPF at the encoder is 100% compliant with the Bluetooth LC3 specification (decoders read the `ltpf_active` bit in the frame header and bypass pitch post-filtering automatically).


## Use esp-dsp SIMD and smart decimation
"Optimization 4" consists of two algorithmic improvements:
* Subwoofer Decimation Pruning (2-Stage Polyphase Decimator & LPF): Decimates 48 kHz mono -> 8 kHz mono (factor-of-6) in two stages:
  - Stage 1: Anti-aliasing LPF at 48 kHz (fc = 2500 Hz) + Factor-of-3 downsampling (480 -> 160 samples @ 16 kHz).
  - Stage 2: 4th-order Linkwitz-Riley Subwoofer LPF at 16 kHz (fc = 200 Hz) + Factor-of-2 downsampling (160 -> 80 samples @ 8 kHz).
  - Eliminates 67% of the LR4 computation while improving low-frequency filter numerical precision.
* SIMD Vectorization (esp-dsp): Left and Right stereo channels use `dsps_biquad_sf32` (stereo assembly SIMD) processing contiguous float arrays.


Timings (streaming PC audio / tone):
* DSP: 0.37 ms  
* LC3-encode: 0.94 ms (0.40 ms for 8 kHz subwoofer)
* TX: 4.5 ms

--> Only tiny improvement in timing of DSP-filters.


