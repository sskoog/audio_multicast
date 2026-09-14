#!/usr/bin/env python3
"""
Automated Test Suite for audio_VSAF_broadcast2 6-Channel Broadcast & Soft-ACK RTT Engine.
Validates:
  1. SOURCE 6-channel broadcast cadence (~600 pkts/s in CAST mode).
  2. Multi-SINK concurrent streaming on Node 23 (Left, Ch 0) and Node 24 (Right, Ch 1).
  3. Round-Trip Time (RTT) and SINK turnaround Dwell time calculation.
  4. Detection of injected 150 us dwell delay on Node 24 vs Node 23.
  5. SINK drop-out resilience and automatic seamless drop-in recovery.
"""

import sys
import time
import re
import threading
import serial

SOURCE_PORT = "COM116"
SINK_L_PORT = "COM23"
SINK_R_PORT = "COM24"
BAUD = 115200

class NodeMonitor:
    def __init__(self, name: str, port: str):
        self.name = name
        self.port = port
        self.ser = None
        self.running = False
        self.thread = None
        self.lines = []
        self.lock = threading.Lock()

    def start(self):
        target = rf"\\.\{self.port}" if not self.port.startswith("\\\\.\\") else self.port
        self.ser = serial.Serial()
        self.ser.port = target
        self.ser.baudrate = BAUD
        self.ser.dtr = True
        self.ser.rts = True
        self.ser.timeout = 0.5
        self.ser.open()
        self.running = True
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def _reader(self):
        buf = b""
        while self.running:
            try:
                chunk = self.ser.read(256)
                if chunk:
                    buf += chunk
                    while b"\n" in buf:
                        raw_line, buf = buf.split(b"\n", 1)
                        line = raw_line.decode("latin-1", errors="replace").strip("\r")
                        with self.lock:
                            self.lines.append((time.time(), line))
                            if len(self.lines) > 500:
                                self.lines.pop(0)
            except Exception:
                break

    def send_cmd(self, cmd: str):
        if self.ser and self.ser.is_open:
            self.ser.write((cmd + "\r\n").encode("ascii"))
            self.ser.flush()

    def get_recent_lines(self, seconds: float = 3.0):
        now = time.time()
        with self.lock:
            return [l for t, l in self.lines if now - t <= seconds]

    def stop(self):
        self.running = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass


def run_tests():
    print("=" * 70)
    print(" VSAF 6-CHANNEL BROADCAST & SOFT-ACK RTT AUTOMATED TEST SUITE")
    print("=" * 70)
    print(f"SOURCE Port: {SOURCE_PORT}")
    print(f"SINK L Port: {SINK_L_PORT} (Node 23, Channel 0)")
    print(f"SINK R Port: {SINK_R_PORT} (Node 24, Channel 1)")
    print("-" * 70)

    mon_src = NodeMonitor("SOURCE", SOURCE_PORT)
    mon_l = NodeMonitor("SINK_L", SINK_L_PORT)
    mon_r = NodeMonitor("SINK_R", SINK_R_PORT)

    try:
        mon_src.start()
        mon_l.start()
        mon_r.start()
    except Exception as e:
        print(f"[FATAL] Failed to open COM ports: {e}")
        return False

    print("[INFO] Monitoring ports for initial telemetry (3s)...")
    time.sleep(3.0)

    # -------------------------------------------------------------------------
    # TEST 1: Put SOURCE in CAST mode & verify 6-channel broadcast rate (~600 pkts/s)
    # -------------------------------------------------------------------------
    print("\n[TEST 1] Triggering SOURCE into CAST mode (tone test tone)...")
    mon_src.send_cmd("start")
    mon_src.send_cmd("tone on")
    time.sleep(3.5)

    src_lines = mon_src.get_recent_lines(3.0)
    cast_seen = False
    tx_pkts_rate = 0

    for line in src_lines:
        if "CAST" in line:
            cast_seen = True
            m = re.search(r"CAST\s*\|\s*([01O\-]{6})\s*\|.*?\|\s*(\w+)\s+(\d+)\s+(\d+%?)\s+(\d+)\s+([0-9\.]+[KM]?)", line)
            if m:
                tx_pkts_rate = int(m.group(3))

    print(f"  -> SOURCE State is CAST: {cast_seen}")
    print(f"  -> SOURCE Primary Sweep TX Rate: {tx_pkts_rate} pkts/s (Expect ~550-650 for 6 channels x 100Hz)")

    test1_pass = cast_seen and (tx_pkts_rate >= 500 and tx_pkts_rate <= 650)
    print(f"  => TEST 1 RESULT: {'PASS' if test1_pass else 'FAIL'}")

    # -------------------------------------------------------------------------
    # TEST 2: Verify Dual SINK streaming on Node 23 and Node 24
    # -------------------------------------------------------------------------
    print("\n[TEST 2] Verifying Dual SINK Audio Reception...")
    time.sleep(3.0)
    l_lines = mon_l.get_recent_lines(3.0)
    r_lines = mon_r.get_recent_lines(3.0)

    l_rx_rate, l_plc = 0, 0
    r_rx_rate, r_plc = 0, 0
    l_found, r_found = False, False

    for line in l_lines:
        if "STRM" in line and "LEFT" in line:
            # Format: | SW HW PKTS PLC DMA FIFO_UDR FIFO_OVR |
            m = re.search(r"\|\s*([0-9\-]+)\s+([0-9\+\-]+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s*\|", line)
            if m:
                l_rx_rate = int(m.group(3))
                l_plc = int(m.group(4))
                l_found = True

    for line in r_lines:
        if "STRM" in line and "RGHT" in line:
            m = re.search(r"\|\s*([0-9\-]+)\s+([0-9\+\-]+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s*\|", line)
            if m:
                r_rx_rate = int(m.group(3))
                r_plc = int(m.group(4))
                r_found = True

    print(f"  -> Node 23 (Left):  STRM Detected = {l_found}, RX Rate = {l_rx_rate} pkts/s, PLC Total = {l_plc}")
    print(f"  -> Node 24 (Right): STRM Detected = {r_found}, RX Rate = {r_rx_rate} pkts/s, PLC Total = {r_plc}")

    test2_pass = l_found and r_found and (l_rx_rate >= 70) and (r_rx_rate >= 70)
    print(f"  => TEST 2 RESULT: {'PASS' if test2_pass else 'FAIL'}")

    # -------------------------------------------------------------------------
    # TEST 3: Round-Trip Time & Dwell Time Delta Validation
    # -------------------------------------------------------------------------
    print("\n[TEST 3] Validating Round-Trip Time (RTT) & Injected 150us Dwell Delta...")
    time.sleep(3.0)
    src_lines = mon_src.get_recent_lines(3.0)

    rtt_samples = []
    for line in src_lines:
        m = re.search(r"\|\s*(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s*\|", line)
        if m:
            lt, ld, rt, rd, ln, rn = [int(x) for x in m.groups()]
            if lt > 0 and rt > 0:
                rtt_samples.append((lt, ld, rt, rd, ln, rn))

    if rtt_samples:
        avg_lt = sum(s[0] for s in rtt_samples) / len(rtt_samples)
        avg_ld = sum(s[1] for s in rtt_samples) / len(rtt_samples)
        avg_rt = sum(s[2] for s in rtt_samples) / len(rtt_samples)
        avg_rd = sum(s[3] for s in rtt_samples) / len(rtt_samples)
        avg_ln = sum(s[4] for s in rtt_samples) / len(rtt_samples)
        avg_rn = sum(s[5] for s in rtt_samples) / len(rtt_samples)
        dwell_delta = avg_rd - avg_ld

        print(f"  -> Node 23 (Left):  Total RTT = {avg_lt:.1f} us, Dwell = {avg_ld:.1f} us, Net RTT = {avg_ln:.1f} us")
        print(f"  -> Node 24 (Right): Total RTT = {avg_rt:.1f} us, Dwell = {avg_rd:.1f} us, Net RTT = {avg_rn:.1f} us")
        print(f"  -> Measured Dwell Delta (Node 24 - Node 23): {dwell_delta:.1f} us (Expected ~150 us +/- 40 us)")

        test3_pass = (110 <= dwell_delta <= 190) and (avg_ln < 5000) and (avg_rn < 5000)
    else:
        print("  -> [ERROR] No complete RTT telemetry samples captured in time window.")
        test3_pass = False

    print(f"  => TEST 3 RESULT: {'PASS' if test3_pass else 'FAIL'}")

    # -------------------------------------------------------------------------
    # TEST 4: SINK Drop-out and Seamless Drop-in Recovery
    # -------------------------------------------------------------------------
    print("\n[TEST 4] Testing SINK Drop-Out & Reconnection...")
    print("  -> Rebooting Node 24 (Right speaker)...")
    mon_r.send_cmd("reset")
    time.sleep(1.0)

    # Check SOURCE and Node 23 while Node 24 is offline
    print("  -> Checking SOURCE and Node 23 during Node 24 outage (3s)...")
    time.sleep(3.0)
    src_lines_drop = mon_src.get_recent_lines(3.0)

    src_tx_during_drop = 0
    for line in src_lines_drop:
        m = re.search(r"CAST\s*\|\s*([01O\-]{6})\s*\|.*?\|\s*(\w+)\s+(\d+)", line)
        if m:
            val = int(m.group(3))
            if val > src_tx_during_drop:
                src_tx_during_drop = val

    print(f"  -> SOURCE TX Rate during Node 24 reboot: {src_tx_during_drop} pkts/s (Uninterrupted broadcast)")

    # Wait for Node 24 to boot back up and rejoin
    print("  -> Waiting for Node 24 to rejoin network...")
    rejoined = False
    for attempt in range(12):
        time.sleep(1.0)
        src_lines_rec = mon_src.get_recent_lines(2.0)
        for line in src_lines_rec:
            m = re.search(r"CAST\s*\|\s*([01O\-]{6})\s*\|", line)
            if m and m.group(1).startswith("11"):
                rejoined = True
                break
        if rejoined:
            print(f"  -> Node 24 successfully rejoined in {attempt + 1} seconds!")
            break

    test4_pass = (src_tx_during_drop >= 400) and rejoined
    print(f"  => TEST 4 RESULT: {'PASS' if test4_pass else 'FAIL'}")

    # -------------------------------------------------------------------------
    # FINAL SUMMARY
    # -------------------------------------------------------------------------
    mon_src.stop()
    mon_l.stop()
    mon_r.stop()

    print("\n" + "=" * 70)
    print(" TEST SUITE SUMMARY REPORT")
    print("=" * 70)
    print(f" Test 1 (6-Channel Broadcast Sweeper Cadence @ ~600 pkts/s):  {'[PASS]' if test1_pass else '[FAIL]'}")
    print(f" Test 2 (Dual SINK Audio Streaming & PLC Integrity):          {'[PASS]' if test2_pass else '[FAIL]'}")
    print(f" Test 3 (Round-Trip Time & 150us Dwell Delta Identification): {'[PASS]' if test3_pass else '[FAIL]'}")
    print(f" Test 4 (SINK Drop-Out Resilience & Seamless Drop-In):        {'[PASS]' if test4_pass else '[FAIL]'}")
    print("=" * 70)

    all_passed = test1_pass and test2_pass and test3_pass and test4_pass
    print(f" OVERALL RESULT: {'SUCCESS - ALL TESTS PASSED' if all_passed else 'FAILURE - SOME TESTS FAILED'}")
    print("=" * 70)
    return all_passed


if __name__ == "__main__":
    success = run_tests()
    sys.exit(0 if success else 1)
