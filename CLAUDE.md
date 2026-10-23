# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

Project overview, hardware registry (node IDs, MACs, COM ports, pinouts), post-gain calibration table and the ESP-IDF environment activation snippet live in AGENTS.md. Always-on rules live in `.agents/rules/`. Both are imported here so they are not duplicated:

@AGENTS.md
@.agents/rules/01_esp32_specific.md
@.agents/rules/02_audio_protocol_and_timing.md
@.agents/rules/03_diagnostics_and_telemetry.md
@.agents/rules/04_coding_style.md
@.agents/rules/05_user_interaction.md

The repo was previously developed with VSCode + Google Antigravity; `.agents/rules/` is left over from that and is imported above. Task playbooks are Claude Code skills in `.claude/skills/` (`node16-flash-and-reset`, `esp32-usb-serial-debug`, `parallel-compile-and-flash`, `lc3-codec-benchmark`, and the legacy `audio-test-matrix`).

## Which app is active

`apps/audio_VSAF_broadcast2` is the main track and the latest app. Default to it for all firmware work unless told otherwise. Its spec is [apps/audio_VSAF_broadcast2/README_wifi_audio_broadcast2.md](apps/audio_VSAF_broadcast2/README_wifi_audio_broadcast2.md). `audio_ESP_NOW_broadcast` and `audio_ESP_NOW_unicast` are earlier siblings with a near-identical file layout; `node2node`, `android2node`, `usb_ble_bumble` are legacy BLE experiments. Don't port changes between apps unless asked.

The wire format source of truth is [main/vsaf_protocol.hpp](apps/audio_VSAF_broadcast2/main/vsaf_protocol.hpp) (`static_assert`s pin packet sizes). If you change it, update the VSAF section in AGENTS.md and the app README too.

## Build & flash (audio_VSAF_broadcast2)

Scripts live in `apps/audio_VSAF_broadcast2/tools/`. They activate ESP-IDF (v6.1) through the shared helper [tools/idf_env.ps1](tools/idf_env.ps1); the repo-wide IDF version is `$DefaultIdfVersion` there, never hard-code IDF paths in new scripts. Scripts accept `-IdfVersion` to override for one run, and stale build dirs from another IDF version are auto-`fullclean`ed:

```powershell
# Compile only (no hardware needed) - the closest thing to a "lint/check" step
powershell -ExecutionPolicy Bypass -File apps\audio_VSAF_broadcast2\tools\compile.ps1 -Target s3   # or -Target c6
powershell -ExecutionPolicy Bypass -File apps\audio_VSAF_broadcast2\tools\build_and_flash.ps1 -Role SINK -NodeId 23 -OnlyCompile

# Build + flash one node (chip is inferred from Role/Port/NodeId; S3 for SOURCE and Nodes 4/5)
powershell -ExecutionPolicy Bypass -File apps\audio_VSAF_broadcast2\tools\build_and_flash.ps1 -Role SOURCE            # Node 16, auto port
powershell -ExecutionPolicy Bypass -File apps\audio_VSAF_broadcast2\tools\build_and_flash.ps1 -Role SINK -Port COM4 -NodeId 4
powershell -ExecutionPolicy Bypass -File apps\audio_VSAF_broadcast2\tools\build_and_flash.ps1 -Role SINK -Port COM23 -NodeId 23 -OnlyFlash
```

- One firmware image per chip family. `build_s3/` + `sdkconfig.s3` and `build_c6/` + `sdkconfig.c6` are isolated. The script copies the target's `sdkconfig.<chip>` to `sdkconfig` before building and copies it back on success, so edit persistent Kconfig changes in `sdkconfig.<chip>` / `sdkconfig.defaults*`, not in `sdkconfig`.
- Node 16 (SOURCE, native USB TinyUSB) is flashed via `tools/s3_flash_and_reset.py` (hands-free reset into ROM bootloader, COM116 app port -> COM16 boot port). Other nodes use plain `esptool write_flash`.
- **Do not build S3 and C6 concurrently (known race, unresolved).** Both targets share `managed_components/` and `dependencies.lock`; a C6 configure deletes the S3-only `esp_tinyusb` while an S3 build reads it (WinError 32 / missing `tusb_config.h`). Build targets one after the other until this is fixed; one-at-a-time builds work because the shared lock forces a re-solve on each target switch.
- Whole cluster at once (currently affected by the race above when dependencies re-solve; then all detected nodes flashed concurrently): `powershell -ExecutionPolicy Bypass -File apps\audio_VSAF_broadcast2\tools\parallel_build_and_flash.ps1 [-BuildOnly | -FlashOnly] [-Nodes "16,4,5"]`.
- The build script kills lingering serial monitors / streamers before flashing; close any monitor you opened yourself first.

## Testing

There are no unit tests. Validation is hardware-in-the-loop over serial telemetry:
- `apps/audio_VSAF_broadcast2/tools/test_broadcast_cluster.py`: cluster test (cadence, multi-SINK streaming, RTT, drop-out/recovery) that parses 1 s heartbeat lines from SOURCE (COM116) and SINKs.
- `tools/test_audio_matrix.py` / `audio-test-matrix` skill: legacy, targets `audio_ESP_NOW_broadcast` console commands only.
- `scratch/*.py`: ad-hoc one-off probes; not maintained.
- Python runs from `.venv_audio_multicast` (use `uv`). Serial scripts monkey-patch pyserial to survive Windows usbser.sys errors 13/22/31 on TinyUSB CDC ports, and must open ports with DTR/RTS false.

## Firmware architecture (audio_VSAF_broadcast2)

Same binary for SOURCE and SINK of a given chip; role and per-node hardware are chosen at **runtime**:
- [main/config.c](apps/audio_VSAF_broadcast2/main/config.c) `get_system_config()` reads the Wi-Fi STA MAC and fills a `system_config_t` (node_id, role, I2S/LED/mute GPIOs, default channel, `is_pcm5102a`, DAC gain). Adding a node = add a MAC branch here (under the right `CONFIG_IDF_TARGET_*` block) plus, for PCM5102A nodes, a post-gain case in `main.cpp`.
- `main.cpp`: boot, config lookup, wiring of subsystems, SINK post-gain switch.
- `vsaf_broadcast_engine.cpp`: the transport. SOURCE: `esp_timer` periodic callback -> task notification -> `sourceTxTask`, which encodes and sends the per-channel packets back-to-back, each send gated on the Wi-Fi TX-done callback semaphore; sets `REQ_ACK` round-robin so exactly one SINK replies per 10 ms. SINK: RX callback -> LC3 frame FIFO -> decode/playback loop with sequence gap handling, redundant-frame recovery (t-1/t-2/t-3), then PLC.
- `lc3_codec.cpp`: wraps Google `liblc3` (S3 SOURCE encoders and S3 PCM5102A 24-bit decoder, IRAM-placed) and Espressif `esp_audio_codec` LC3 (C6 fixed-point). SOURCE runs 5 encoder instances (L/R HQ 120 B, Sub 8 kHz 60 B, L/R redundancy 60 B) in that order.
- `usb_audio.cpp` (S3 only): UAC1 USB speaker that feeds the SOURCE. `tone_generator.cpp`: internal test synth (`synth on/off`).
- `i2s_audio.cpp`: I2S DAC output (16-bit MAX98357A on C6, 32-bit slot / 24-bit PCM5102A on S3, GPIO 3 amp mute). `audio_filters.hpp` / `biquad_filter.hpp`: LR4 crossover (satellites HP 100 Hz, sub LP + polyphase decimation to 8 kHz).
- `diagnostics.cpp`: fixed-width 1 s heartbeat telemetry; test scripts parse these columns, so changing the format can break them. `console.cpp`: serial CLI (commands listed in README section 7, e.g. `bootloader`).
- `main/linker.lf` (C6 only) places hot paths in IRAM.

Shared ESP-IDF components in `components/` (`liblc3`, `esp-dsp`, `fixp-dsp`, `audio_metering`, `stats`, `led_strip`) are pulled in via `EXTRA_COMPONENT_DIRS` in each app's top-level `CMakeLists.txt`; managed components come from `main/idf_component.yml`.
