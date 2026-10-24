# liblc3_dual

Fork of Google liblc3 (copied from [../liblc3](../liblc3), which stays untouched) with **dual-rate encoding**: one PCM frame encoded at two frame sizes with a single LC3 analysis. Used by `apps/audio_VSAF` for the SOURCE's HQ (120 B) + MQ redundancy (60 B) frames.

## Added API (`include/lc3.h`)

| Function | Purpose |
| :--- | :--- |
| `lc3_encoder_rate_size(dt_us, sr_hz)` | Bytes needed for the second-rate state (~2 KB at 48 kHz / 10 ms) |
| `lc3_setup_encoder_rate(encoder, mem)` | Second-rate state, paired with one primary encoder for the whole stream |
| `lc3_encode_dual(...)` | Encode both frames in one call |
| `lc3_encode_dual_begin(...)` / `lc3_encode_dual_end(rate, out_b)` | Same, split: `_begin` = shared analysis + primary frame, `_end` = second frame. Lets the caller send the primary frame first. |

## How it works (`src/lc3.c`, `analyze_dual`)

Shared once per frame: input load, LTPF analysis, MDCT, band energies, bandwidth detection. SNS and TNS are shared when both rates take the same attack-detection and TNS LPC-weighting decisions (attack detection keeps one state per rate; at 48 kHz / 10 ms it is enabled only from 100 B, so a 120 B + 60 B pair recomputes SNS/TNS for the second rate on attack frames). Spectral quantization (`lc3_spec_analyze`, with its per-rate bit-offset state) and the bitstream writer run per rate. `tns_lpc_weighting()` mirrors `resolve_lpc_weighting()` of `tns.c`; keep them in sync if upstream changes it.

The output is bit-exact with two independent encoders at the two sizes (checked on target by the audio_VSAF console command `encbench`). High-resolution (LC3plus HR) streams are not supported by the dual API.

## Other differences to upstream

- ESP-IDF component files: `CMakeLists.txt`, `Kconfig` (`CONFIG_LC3_CODE_IN_IRAM`, `CONFIG_LC3_ENABLE_PLUS_MODES`), `linker.lf` (archive `libliblc3_dual.a`).
- Only `src/`, `include/` and the license were copied; upstream tests, tools and bindings stay in `../liblc3`.

Both components export the same symbols, so an app must use one or the other (audio_VSAF lists `liblc3` in `EXCLUDE_COMPONENTS`).
