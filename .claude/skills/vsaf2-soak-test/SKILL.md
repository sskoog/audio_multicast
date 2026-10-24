---
name: vsaf2-soak-test
description: >-
  Run and interpret the VSAF2 reliability soak test: stream music from the PC into the SOURCE (Node 16)
  over TinyUSB UAC1, log every SINK's 1 s heartbeat, and check the GOALS.md target of zero PLC
  (concealed frames) per SINK. Use for any dropout / PLC / packet-loss / RF-channel / Wi-Fi PHY-rate
  investigation on audio_VSAF.
---

# VSAF2 Soak Test (zero-PLC check)

## Tools (apps/audio_VSAF/tools/)
- `usb_audio_streamer.py`: plays a 48 kHz file (MP3/WAV/FLAC, looped) or built-in synthetic music to the
  "Node16 audio (USB Speaker)" WASAPI endpoint (48 kHz, so Windows does not resample). Reports PC-side
  underflows (must be 0, otherwise the PC starved the SOURCE).
- `soak_test.py`: opens SOURCE (COM116) + SINK consoles, starts the streamer, waits for all SINKs to reach
  `STRM`, sends `clear`, logs for `--duration` s, writes `soak_logs/<timestamp>/` (raw per-node logs,
  `streamer.log`, `summary.json`) and exits 0 only on PASS.

```powershell
cd apps\audio_VSAF\tools
..\..\..\.venv_audio_multicast\Scripts\python.exe -u soak_test.py --duration 60     # smoke run first
..\..\..\.venv_audio_multicast\Scripts\python.exe -u soak_test.py --duration 900    # 15-minute run
# options: --sinks 4,5,23,24   --file <48k file> ('' = synthetic)   --no-stream (audio from elsewhere)
```
Needs `soundfile` in the venv for MP3 (`uv pip install --python .venv_audio_multicast\Scripts\python.exe soundfile`).
Default song: `data/mp3/Alan Walker - Monster.mp3` (gitignored, local only).

**PASS** = every SINK: PLC total 0, never left `STRM`, streamed > 90 % of the window.

## Reading the results
SINK heartbeat middle block: `SW HW PKTS RED PLC DMA OVF UDR`.
- **RED**: frames rebuilt from the redundant t-1/t-2 (t-3 for sub) copies. Harmless; it measures raw packet loss.
- **PLC**: frames concealed. Every PLC is a target failure. **UDR** (FIFO underrun) is the subset caused by the
  FIFO running dry (late/missing bursts); PLC - UDR = sequence gaps longer than the redundancy depth.
- **loss%** in the report = (RED + PLC) / frames. Do **not** use the PKTS column for loss: the firmware's 1 s
  rate reads ~0.8 % low on every node (the SOURCE's TOT counter grows at exactly 600 pkt/s while its 1/s column shows ~595).
- Counters are running totals printed modulo 1000 (firmware) and unwrapped by the logger; they reset when a SINK
  re-enters streaming.

**Diagnosing a FAIL**
- PLC events at the *same time on all SINKs* (compare `plc_events` in summary.json): common cause, i.e. channel /
  interference or a SOURCE stall. If the SOURCE's packet rate stays ~600/s during the event, the SOURCE kept
  sending and the packets were delayed/lost in the air.
- PLC on *one SINK only* with high RED: that node's reception (placement, antenna, nearby USB 3 noise), not the protocol.
- High decode time on a SINK (audio block `ms Avg Pk`) approaching 10 ms: CPU-bound (C6 decoding 48 kHz satellites
  takes ~1.2 ms, so this has not been the cause so far).

## Radio knobs (SOURCE console, runtime only, lost on reboot; boot default = ch 13, MCS3)
Boot channel: `CONFIG_VSAF_PREFERRED_WIFI_CHANNEL` (13) in `main/config.h`. The boot RF survey is off
(`CONFIG_VSAF_BOOT_CHANNEL_SCAN 0`) because its decoded-frame score preferred ch 1, which measured worse for TX.
- `wifich <1..13>`: SINKs re-scan and follow. Check neighbours first: `netsh wlan show networks mode=bssid`.
- `phy mcs3 | mcs1 | mcs0 | 12m | ...`: broadcast PHY rate.
  **Do not use MCS0 with the current firmware**: TX stage grows from ~3.6 to ~5.2 ms, the frame (DSP + 2 encode
  passes + TX, all serial on core 1) reaches ~9.3-9.5 ms of the 10 ms budget and the SOURCE skips frames in about
  a quarter of all seconds (seen as `seconds < 590 pkt/s` in the report). MCS3 keeps the frame at ~7.8 ms.
- **Fixed 2026-10-24 (commit 472e8a0): SOURCE "sending" but silent.** Cause: ESP-NOW TX buffers exhausted
  (`ESP_ERR_ESPNOW_NO_MEM`) after frequent TX-done timeouts; old firmware could stay stuck there. Now `wifich`/`phy`
  apply at a frame boundary and the TX loop rebuilds ESP-NOW by itself (send-error run, no TX-done for 0.5 s, or
  impossibly fast sweeps). Check with `txstat` (`recoveries`, `send_err`, `no_mem`, `tx_done_timeouts`);
  `txfault` injects a fault (verified: 30 failed sends = 50 ms, then recovery, SINKs stay in STRM).
- **Channel choice:** the `tx_done_timeouts` rate in `txstat` is the most direct "how hard is it to transmit here"
  measure. 2026-10-24: ch 13 ~35-70/s, ch 1 ~150-300/s at MCS3, although the decoded-frame boot survey ranked ch 1
  cleaner. `soak_test.py` aborts if SINKs do not reach STRM.
- SINK jitter cushion: `CONFIG_ESPNOW_PREFILL_THRESHOLD_FRAMES` (config.h, 5 = 50 ms: 2 DMA + 3 FIFO). Each extra
  frame adds 10 ms latency (GOALS.md soft target < 50 ms end to end).

## Baseline log
| Date | Setup | Window | Result |
| :--- | :--- | :--- | :--- |
| 2026-10-24 | ch 10, MCS3 (boot default) | 60 s | FAIL: 5-8 PLC on every SINK, simultaneous ~50 ms events |
| 2026-10-24 | ch 13, MCS3 | 60 s | Nodes 4/5/24: 0 PLC. Node 23: 38 PLC, ~7 % loss |
| 2026-10-24 | ch 13, MCS0 | 60 s | Nodes 4/5/24: 0 PLC, 0.2-0.35 % loss. Node 23: 49 PLC, 1.8 % loss |
| 2026-10-24 | ch 13, MCS0 | 15 min | FAIL. Nodes 4/5: 0 PLC. Node 24: 2 PLC. Node 23: 695 PLC, 7.4 % loss, 2 SCAN + 2 hidden re-syncs. SOURCE over budget (frame 9.3 ms, 228/804 s with skipped frames) |
| 2026-10-24 | ch 1, MCS0 | 60 s | FAIL: all 4 SINKs dropped to SCAN at the same moment; SOURCE over budget |
| 2026-10-24 | ch 13, MCS3 (Node 23 relocated, RSSI -39) | 15 min | FAIL, but 0 PLC on all 4 SINKs for the first 13 min (loss 0.19-0.36 %). Then one burst on all SINKs at once (10-30 PLC each, all FIFO underruns), exactly when the SOURCE TX stage rose to 4.1-4.8 ms and it skipped frames (554-583 pkt/s). Cause: a busy channel slows the serial per-packet TX waits, the frame overruns 10 ms, `ulTaskNotifyTake(pdTRUE)` merges the missed ticks, and whole frames are never produced. |

**Root cause of the remaining PLC (2026-10-24):** SOURCE frame overrun under channel contention, not SINK reception.
Fix candidates: (1) move the 6 `esp_now_send` calls out of the encode loop into their own task/core so slow airtime
cannot delay the next encode, and stop busy-waiting per packet; (2) never drop a frame on overrun (handle a notify
count > 1); (3) a deeper SINK cushion (`CONFIG_ESPNOW_PREFILL_THRESHOLD_FRAMES` 5 -> 6/7, +10/20 ms latency).
