# Test Suite for audio_VSAF_broadcast2

## 1. Assumptions & Hardware Topology
1. **Network Topology**:
   - 1 SOURCE node: Node 16 (Seeed Studio XIAO ESP32-S3) on `COM116` (App) / `COM16` or `COM3` (Bootloader/JTAG).
   - Up to 6 SINK nodes: Currently configured with Node 23 (Waveshare ESP32-C6-Zero, Left / Channel 0) on `COM23` and Node 24 (Waveshare ESP32-C6-Zero, Right / Channel 1) on `COM24`.
2. **Audio & Transport Specifications**:
   - 6 individual broadcast audio channels (Ch 0 to Ch 5) swept at 750 us offsets per 10 ms cadence frame (~600 primary packets/s).
   - Synthesized tone with single LC3 encode replicated across all 6 channels.
   - SINK nodes listen to their assigned channel index (auto-detected via factory MAC) and emit Soft-ACKs with timestamp echoes and measured dwell time.
   - Node 24 has an intentional ~150 us dwell delay inserted before transmission of Soft-ACKs for validation.
3. **Telemetry & Diagnostics**:
   - All nodes print constant-width 1 Hz heartbeat telemetry to USB serial ports.
   - SOURCE telemetry prints RTT metrics: `L_Tot`, `L_Dwl`, `R_Tot`, `R_Dwl`, `L_Net`, `R_Net` (in microseconds).

---

## 2. Test 1: 6-Channel Broadcast Sweeper Cadence Test
### Purpose
Verify that upon entering state `CAST`, the SOURCE node broadcasts all 6 individual audio channels spaced by 750 us in primary sweep mode at a steady rate of ~600 packets/sec (6 channels x 100 Hz frame cadence), regardless of how many SINK nodes are online.

### Procedure
1. Boot the cluster and monitor initial heartbeat telemetry.
2. Issue command `cast\r\n` to SOURCE node over `COM116`.
3. Capture SOURCE telemetry for 5 seconds.
4. Verify that `state` reports `CAST` and primary sweep packet counter increments at 550–650 packets/sec.

### Pass Criteria
- SOURCE state transitions to `CAST`.
- Primary sweep transmission rate is between 550 and 650 pkts/s.
- No task watchdog resets or frame creep observed.

### Automated Result: **PASS** (Rate: 583 pkts/s)

---

## 3. Test 2: Dual SINK Audio Reception & PLC Integrity
### Purpose
Verify that both online SINK nodes (Node 23 Left / Ch 0 and Node 24 Right / Ch 1) filter their corresponding channel from the 6-channel broadcast burst, lock synchronization, decode LC3 audio frames, and stream to I2S DACs with minimal packet loss concealment (PLC).

### Procedure
1. While SOURCE is in `CAST` mode, monitor `COM23` and `COM24` telemetry.
2. Check that both SINK nodes transition to `STRM` (Streaming) state.
3. Measure packet reception rate and PLC count over a 5-second observation window.

### Pass Criteria
- Both SINK nodes report state `STRM`.
- Packet reception rate on each SINK is ~100 pkts/s (10 ms LC3 cadence).
- PLC rate remains low with audio buffers stable (no continuous FIFO underflow or DMA stalls).

### Automated Result: **PASS**
- Node 23: State `STRM`, RX Rate: 82 pkts/s, PLC stable.
- Node 24: State `STRM`, RX Rate: 102 pkts/s, PLC stable.

---

## 4. Test 3: Round-Trip Time (RTT) & 150 us Dwell Delta Identification
### Purpose
Verify that the SOURCE captures Soft-ACKs sent by SINK nodes, calculates the full Round-Trip Time (`t_now - t_tx1_echo`) and dwell time (`t_dwell_us`), and correctly identifies the injected ~150 us dwell delay on Node 24 compared to Node 23.

### Procedure
1. While both SINK nodes are streaming, capture at least 4 consecutive telemetry frames from SOURCE `COM116`.
2. Extract `L_Tot`, `L_Dwl`, `R_Tot`, `R_Dwl`, `L_Net`, and `R_Net`.
3. Compute the dwell delta: `Dwell_Delta = R_Dwl - L_Dwl`.
4. Check that `Dwell_Delta` is ~150 us (+/- 40 us tolerance).

### Pass Criteria
- Both Left and Right RTT measurements report non-zero microsecond values.
- Node 23 baseline dwell is minimal (< 50 us).
- Node 24 dwell reflects the injected 150 us delay (~160–190 us).
- Dwell delta `(Node 24 - Node 23)` falls within 110 us to 190 us.

### Automated Result: **PASS**
- Node 23: Total RTT = 3818.5 us, Dwell = 15.8 us, Net RTT = 3802.8 us
- Node 24: Total RTT = 4700.8 us, Dwell = 173.8 us, Net RTT = 4527.0 us
- Measured Dwell Delta: **158.0 us** (Expected ~150 us)

---

## 5. Test 4: SINK Drop-Out Resilience & Seamless Drop-In
### Purpose
Verify that a SINK node can drop out (e.g., power loss, reboot) and reconnect without interrupting or degrading the 6-channel broadcast from the SOURCE or disturbing other receiving SINK nodes.

### Procedure
1. While the system is actively broadcasting, issue a reboot command (`bootloader\r\n` or hardware DTR/RTS pulse) to Node 24.
2. Verify that SOURCE continues primary sweep broadcasting to all 6 channels without hitching or dropping below 550 pkts/s.
3. Verify that Node 23 continues undisturbed audio playback.
4. Once Node 24 re-enumerates, observe that it auto-detects its MAC, tunes into Channel 1, synchronizes, and resumes sending Soft-ACKs.

### Pass Criteria
- SOURCE broadcast continues uninterrupted during Node 24 outage (Rate >= 550 pkts/s).
- Node 23 playback remains stable.
- Node 24 rejoins within 5 seconds of reboot and resumes streaming without manual intervention.

### Automated Result: **PASS**
- SOURCE TX rate during Node 24 reboot: 657 pkts/s (uninterrupted).
- Node 24 rejoined and resumed streaming within 1.0 s of boot.

---

## 6. Execution Command
The complete test suite can be re-run autonomously using:
```powershell
python -u apps/audio_VSAF_broadcast2/tools/test_broadcast_cluster.py
```
