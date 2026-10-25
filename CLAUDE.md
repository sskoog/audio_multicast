# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

Project overview, hardware registry (node IDs, MACs, COM ports, pinouts), post-gain calibration table and the ESP-IDF environment activation snippet live in AGENTS.md. Always-on rules live in `.agents/rules/`. Both are imported here so they are not duplicated:

@GOALS.md
@AGENTS.md
@.agents/rules/01_esp32_specific.md
@.agents/rules/02_audio_protocol_and_timing.md
@.agents/rules/03_diagnostics_and_telemetry.md
@.agents/rules/04_coding_style.md
@.agents/rules/05_user_interaction.md

The repo was previously developed with VSCode + Google Antigravity; `.agents/rules/` is left over from that and is imported above. Task playbooks are Claude Code skills in `.claude/skills/` (`node16-flash-and-reset`, `esp32-usb-serial-debug`, `parallel-compile-and-flash`, `vsaf2-soak-test`, `lc3-codec-benchmark`).

## Which app is active

`apps/audio_VSAF` is the main track and the latest app. Default to it for all firmware work unless told otherwise. Its spec is [apps/audio_VSAF/README_audio_VSAF.md](apps/audio_VSAF/README_audio_VSAF.md). Retired apps (old ESP-NOW broadcast/unicast, BLE experiments) are in [archive/](archive/README.md): read-only reference, don't build, fix or IDF-bump them, and don't port changes into them. Goals, numeric targets and the current priority (measure latency / skew / drift / dropouts, then spatial upmix) are in GOALS.md, imported above; judge changes against those targets.

The wire format source of truth is [main/vsaf_protocol.hpp](apps/audio_VSAF/main/vsaf_protocol.hpp) (`static_assert`s pin packet sizes). If you change it, update the VSAF section in AGENTS.md and the app README too.

## Build & flash (audio_VSAF)

Scripts live in `apps/audio_VSAF/tools/`. They activate ESP-IDF (v6.1) through the shared helper [tools/idf_env.ps1](tools/idf_env.ps1); the repo-wide IDF version is `$DefaultIdfVersion` there, never hard-code IDF paths in new scripts. Scripts accept `-IdfVersion` to override for one run, and stale build dirs from another IDF version are auto-`fullclean`ed:

```powershell
# Compile only (no hardware needed) - the closest thing to a "lint/check" step
powershell -ExecutionPolicy Bypass -File apps\audio_VSAF\tools\compile.ps1 -Target s3   # or -Target c6
powershell -ExecutionPolicy Bypass -File apps\audio_VSAF\tools\build_and_flash.ps1 -Role SINK -NodeId 23 -OnlyCompile

# Build + flash one node (chip is inferred from Role/Port/NodeId; S3 for SOURCE and Nodes 4/5)
powershell -ExecutionPolicy Bypass -File apps\audio_VSAF\tools\build_and_flash.ps1 -Role SOURCE            # Node 16, auto port
powershell -ExecutionPolicy Bypass -File apps\audio_VSAF\tools\build_and_flash.ps1 -Role SINK -Port COM4 -NodeId 4
powershell -ExecutionPolicy Bypass -File apps\audio_VSAF\tools\build_and_flash.ps1 -Role SINK -Port COM23 -NodeId 23 -OnlyFlash
```

- One firmware image per chip family. `build_s3/` + `sdkconfig.s3` and `build_c6/` + `sdkconfig.c6` are isolated. The scripts pass `-D SDKCONFIG=sdkconfig.<chip>` to `idf.py`, so Kconfig changes land directly in `sdkconfig.<chip>` (tracked in git); shared defaults go in `sdkconfig.defaults*`. There is no shared `sdkconfig` file.
- Node 16 (SOURCE, native USB TinyUSB) is flashed via `tools/s3_flash_and_reset.py` (hands-free reset into ROM bootloader, COM116 app port -> COM16 boot port). Other nodes use plain `esptool write_flash`.
- S3 and C6 can build concurrently: each target has its own lock (`dependencies.lock.<target>`, set in the app's top-level `CMakeLists.txt`), and the S3-only `esp_tinyusb` (v3.0.0) is vendored in `apps/audio_VSAF/components/esp_tinyusb` (see its `VENDORED.md`) instead of living in the shared `managed_components/`, where every C6 configure would delete it. Don't re-add esp_tinyusb as a registry dependency. In `main/CMakeLists.txt`, gate `REQUIRES` on `idf_build_get_property(target IDF_TARGET)`, never on `CONFIG_*` (Kconfig isn't evaluated yet when requirements are collected).
- Whole cluster at once (S3 + C6 built in parallel, then all detected nodes flashed concurrently; per-target logs in `build_*/parallel_build.log`): `powershell -ExecutionPolicy Bypass -File apps\audio_VSAF\tools\parallel_build_and_flash.ps1 [-BuildOnly | -FlashOnly] [-Nodes "16,4,5"]`.
- The build script kills lingering serial monitors / streamers before flashing; close any monitor you opened yourself first.

## Testing

There are no unit tests. Validation is hardware-in-the-loop over serial telemetry:
- `apps/audio_VSAF/tools/test_broadcast_cluster.py`: cluster test (cadence, multi-SINK streaming, RTT, drop-out/recovery) that parses 1 s heartbeat lines from SOURCE (COM116) and SINKs.
- `apps/audio_VSAF/tools/soak_test.py` + `usb_audio_streamer.py`: reliability soak test (music into Node 16 over USB, per-SINK RED/PLC/underrun accounting, PASS = 0 PLC). See the `vsaf2-soak-test` skill for usage and how to read failures.
- PC-side LC3 encoder (ctypes wrapper + `liblc3.dll`) for streamers: `tools/pc_lc3/`.
- `scratch/*.py`: ad-hoc one-off probes; not maintained.
- Python runs from `.venv_audio_multicast` (use `uv`). Serial scripts monkey-patch pyserial to survive Windows usbser.sys errors 13/22/31 on TinyUSB CDC ports, and must open ports with DTR/RTS false.

## Firmware architecture (audio_VSAF)

Same binary for SOURCE and SINK of a given chip; role and per-node hardware are chosen at **runtime**:
- [main/config.c](apps/audio_VSAF/main/config.c) `get_system_config()` reads the Wi-Fi STA MAC and fills a `system_config_t` (node_id, role, I2S/LED/mute GPIOs, default channel, `is_pcm5102a`, DAC gain). Adding or re-assigning a node = add / change the MAC branch here (under the right `CONFIG_IDF_TARGET_*` block) **and** the node registry at the top of `main.cpp` (it sets the audio channel), plus a post-gain case in `main.cpp` (all DAC types).
- `main.cpp`: boot, config lookup, wiring of subsystems, SINK post-gain switch.
- `vsaf_broadcast_engine.cpp`: the transport (Wi-Fi / ESP-NOW init, TX health and self-healing `recoverEspNowTx`, frame-boundary radio changes, SINK path). SINK: RX callback -> LC3 frame FIFO -> decode/playback loop with sequence gap handling, redundant-frame recovery (t-1/t-2/t-3), then PLC.
- `vsaf_source_pipeline.cpp`: the SOURCE 10 ms pipeline across both cores. `esp_timer` tick -> `src_dsp0` (core 0: USB/DSP, publishes the frame, encodes) + `src_enc1` (core 1: encodes) claiming 11 jobs per frame from a lock-free cursor (sub, 5x HQ, 5x MQ), and `src_rf` (core 1, prio 7: one packet in flight, `t_tx1_us` = frame grid time, 9.5 ms TX deadline, `REQ_ACK` packet sent last and its TX-done kept as PTP t1, control frame every 10th frame). Console `timing` / `tasks` / `encbench` / `mem` measure it. See README section 4 and 10.
- `vsaf_time_sync.cpp`: time sync (VSAF 3.1). SOURCE: two-step PTP-like offset/drift estimate per channel from the telemetry replies (min-RTT sample of 1 s), control-frame builder. SINK: control-frame parser, offset projection. The SINK playout scheduler that uses it is in `runSinkLoop()` (`vsaf_broadcast_engine.cpp`); all SINK I2S output goes through a whole-descriptor staging buffer (`dac_push`). Console `ptp`, `trim`, `latency`, `syncprobe`; host check `tools/sync_probe.py`. README section 6.
- `lc3_codec.cpp`: wraps `components/liblc3_dual` (fork of Google liblc3 with bit-exact dual-rate `lc3_encode_dual_begin/_end`; the original `components/liblc3` is excluded from this app) for the S3 SOURCE encoders and the S3 PCM5102A 24-bit decoder (IRAM-placed), and Espressif `esp_audio_codec` LC3 (C6 fixed-point). SOURCE bank: slots 0..4 satellite HQ encoders (120 B), 5..9 MQ rate states sharing their analysis (60 B), 10 sub (8 kHz, 60 B).
- `sdkconfig.s3` deliberately uses a 32 KB instruction cache (16 KB thrashed between the cores and tripled Wi-Fi CPU cost), LC3plus off and unused Wi-Fi features off; internal RAM on Node 16 (no PSRAM) is the scarce resource, check `mem` after changes.
- `usb_audio.cpp` (S3 only): UAC1 USB speaker that feeds the SOURCE. `tone_generator.cpp`: internal test synth (`synth on/off`).
- `i2s_audio.cpp`: I2S DAC output (16-bit MAX98357A on C6, 32-bit slot / 24-bit PCM5102A on S3, GPIO 3 amp mute). `audio_filters.hpp` / `biquad_filter.hpp`: LR4 crossover (satellites HP 100 Hz, sub LP + polyphase decimation to 8 kHz).
- `diagnostics.cpp`: fixed-width 1 s heartbeat telemetry; test scripts parse these columns, so changing the format can break them. `console.cpp`: serial CLI (commands listed in README section 7, e.g. `bootloader`).
- `main/linker.lf` (C6 only) places hot paths in IRAM.

Shared ESP-IDF components in `components/` (`liblc3`, `liblc3_dual` (used by audio_VSAF instead of `liblc3`), `esp-dsp`, `fixp-dsp`, `audio_metering`, `stats`, `led_strip`) are pulled in via `EXTRA_COMPONENT_DIRS` in each app's top-level `CMakeLists.txt`; registry components come from `main/idf_component.yml` (versions pinned, e.g. `esp_audio_codec` 2.6.2; `'*'` once let esp_tinyusb jump a major version silently).
