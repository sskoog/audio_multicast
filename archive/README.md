# Archive

Retired apps, kept for reference. **Not maintained**: no ESP-IDF version bumps, no fixes, not built in CI or by the repo scripts.
Current work happens in [`apps/audio_VSAF_broadcast2`](../apps/audio_VSAF_broadcast2/); see [GOALS.md](../GOALS.md) for why these were retired.

| Folder | What it was | Why retired |
| :--- | :--- | :--- |
| `audio_ESP_NOW_broadcast/` | First ESP-NOW broadcast app (dual-frame t0 + t-1, runtime sample-rate / frame-duration / mono-stereo switching) | Replaced by VSAF2 (VSAF 3.0 multi-tier redundancy) |
| `audio_ESP_NOW_unicast/` | ESP-NOW multi-unicast with MAC-layer ACKs and retries | Broadcast + redundancy chosen as the transport |
| `node2node/`, `android2node/`, `usb_ble_bumble/` | Bluetooth LE Audio / Auracast experiments (incl. Google Bumble host on ESP32-C6 HCI) | BLE transport is a non-goal (no usable BIG/BIS controller support on ESP32-S3/C6) |
| `tools/` | `test_audio_matrix.py` (mono/stereo/dur/sr matrix), `test_wifi_audio.py` (unicast + PC streamer), `test_live_sample_rates.py` / `measure_all_rates_live.py` (Node 21 as C6 SOURCE, `BROAD` state), `test_sr_*.py` (ad-hoc sample-rate probes), `run_benchmark.py` (`bench` command, disabled in VSAF2), `migrate_to_audio_multicast.ps1` (one-time `C:\Git_ble_audio` -> `C:\Git_audio_multicast` move, done) | Their target apps or commands are archived; LC3 benchmarking now uses `apps/lc3_benchmark` with `tools/run_*_benchmark.py` |
| `skills/audio-test-matrix/` | Agent skill for the 20-case sample-rate x duration x mono/stereo matrix of `audio_ESP_NOW_broadcast` | Its target app is archived |

Notes:
- The PC-side LC3 encoder (`lc3_encoder.py` + `liblc3.dll`) that used to live in `usb_ble_bumble/` was moved to [`tools/pc_lc3/`](../tools/pc_lc3/) because active tools use it. Archived scripts that did `import lc3_encoder` from their own folder need `tools/pc_lc3` on `sys.path` to run.
- The liblc3 source tree used to build that DLL is still at `usb_ble_bumble/liblc3/`.
- The build scripts here were last updated for ESP-IDF v6.1 (via `tools/idf_env.ps1`) but were never built on it.
- `tools/` holds the PC-side test scripts that drove these apps (moved from the top-level `tools/`). Paths inside them still point at the old `apps/...` locations.
