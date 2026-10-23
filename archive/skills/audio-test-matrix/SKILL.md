---
name: audio-test-matrix
description: >-
  LEGACY - only for apps/audio_ESP_NOW_broadcast (not audio_VSAF_broadcast2). Automated test execution and validation of the 20 audio configuration combinations (Sample Rates x Frame Durations x Mono/Stereo) across SOURCE and SINK nodes over ESP-NOW.
---

> **Archived:** `archive/tools/test_audio_matrix.py` drives the `mono`/`stereo`/`dur`/`sr` console commands of `archive/audio_ESP_NOW_broadcast`. `audio_VSAF_broadcast2` has an `sr` command but no `mono`/`stereo`/`dur`, and a different telemetry layout, so the matrix does not apply to it. For VSAF2 cluster validation use `apps/audio_VSAF_broadcast2/tools/test_broadcast_cluster.py`.

# Audio Matrix Verification Skill

This skill executes automated end-to-end matrix tests over serial ports to verify audio stability, CPU load, packet delivery rate, and underrun metrics across all supported configurations.

---

## Test Matrix Definition
- **Sample Rates (5)**: 48 kHz, 32 kHz, 24 kHz, 16 kHz, 8 kHz
- **Frame Durations (2)**: 10.0 ms, 7.5 ms
- **Channel Modes (2)**: Mono, Stereo
- **Total Test Cases**: 20 combinations (10 seconds per case)

---

## Execution Command

```powershell
# Run full 20-combination matrix against SOURCE (COM16) and SINK (COM23)
python archive\tools\test_audio_matrix.py --source-port COM16 --sink-port COM23 --duration 10
```

---

## Pass/Fail Verification Criteria
1. **Packet Delivery Rate**:
   - 10.0 ms Cadence: Must maintain 199.5 - 200.5 pkts/s (Mono: 200 pkts/s total, Stereo: 200 pkts/s per channel).
   - 7.5 ms Cadence: Must maintain 265.5 - 267.5 pkts/s.
2. **Underrun & Recovery Limits**:
   - `FIFO UDR`: Must remain 0 after the initial prefill buffer locks.
   - `DMA UDR`: Must remain 0.
   - `PLC tot`: Must not exceed 1% of total frames under line-of-sight conditions.
3. **Codec Time Headroom**:
   - ESP32-S3 Stereo @ 48 kHz / 10 ms: Codec encode time must remain under 4.0 ms (40% headroom per 10 ms window).
