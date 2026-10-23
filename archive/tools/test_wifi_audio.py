import serial
import time
import subprocess
import threading
import sys
import re

PORT_SRC = "COM16"
PORT_SINK1 = "COM23"
PORT_SINK2 = "COM24"
BAUD = 115200

def parse_telemetry(line):
    # Typical line: | 27  60  240 | BROAD |  -   01 24M |  LC3  -33.4 -30.3  48.0   10  4.17  4.28  -26   -   197   -    -    -     -  |  14329   14329 |
    # SINK line:    | 26  36  160 | STRM  | -24  01  1M |  LC3  -33.1 -30.3  48.0   10  1.98  2.06  -26  +3    99    0    0     0    0 | 651984    9714 |
    if not line.startswith("|") or line.startswith("+="):
        return None
    parts = [p.strip() for p in line.split("|")]
    if len(parts) < 6:
        return None
    
    try:
        cpu_parts = parts[1].split()
        cpu_pct = int(cpu_parts[0]) if len(cpu_parts) > 0 else 0
        state = parts[2]
        
        audio_parts = parts[5].split()
        sr = audio_parts[3] if len(audio_parts) > 3 else "-"
        
        mid_parts = parts[6].split()
        # For SINK: gain_sw, gain_hw, pkts, plc, dma_udr, fifo_udr, fifo_ovr
        plc = int(mid_parts[3]) if len(mid_parts) > 3 and mid_parts[3] != '-' else 0
        dma_udr = int(mid_parts[4]) if len(mid_parts) > 4 and mid_parts[4] != '-' else 0
        fifo_udr = int(mid_parts[5]) if len(mid_parts) > 5 and mid_parts[5] != '-' else 0
        
        time_parts = parts[7].split()
        local_time_us = int(time_parts[0]) if len(time_parts) > 0 and time_parts[0] != '-' else 0
        master_time_us = int(time_parts[1]) if len(time_parts) > 1 and time_parts[1] != '-' else 0

        return {
            "state": state,
            "sr": sr,
            "plc": plc,
            "dma_udr": dma_udr,
            "fifo_udr": fifo_udr,
            "master_time_us": master_time_us,
            "local_time_us": local_time_us
        }
    except Exception as e:
        return None


def open_ports():
    print("Opening COM ports...")
    s_src = serial.Serial(PORT_SRC, BAUD, timeout=0.1)
    s_snk1 = serial.Serial(PORT_SINK1, BAUD, timeout=0.1)
    s_snk2 = serial.Serial(PORT_SINK2, BAUD, timeout=0.1)
    
    # Ensure they are not in reset
    s_src.dtr = False
    s_src.rts = False
    s_snk1.dtr = False
    s_snk1.rts = False
    s_snk2.dtr = False
    s_snk2.rts = False
    print("Waiting 3.5s for nodes to boot...")
    time.sleep(3.5)
    
    s_src.write(b"\r\n")
    s_snk1.write(b"\r\n")
    s_snk2.write(b"\r\n")
    time.sleep(0.1)
    
    s_src.reset_input_buffer()
    s_snk1.reset_input_buffer()
    s_snk2.reset_input_buffer()
    return s_src, s_snk1, s_snk2

def read_all(s, samples_list, node_name):
    while s.in_waiting > 0:
        l = s.readline().decode('utf-8', errors='replace').rstrip()
        p = parse_telemetry(l)
        if p:
            p['node'] = node_name
            p['ts'] = time.time()
            samples_list.append(p)

def test_1_continuous(s_src, s_snk1, s_snk2, duration=60):
    print("\n--- TEST 1: Continuous Streaming & Clock Drift (60s) ---")
    s_src.read_all()
    s_snk1.read_all()
    s_snk2.read_all()
    s_src.write(b"synth\r\n")
    time.sleep(0.1)
    s_src.write(b"start\r\n")
    s_src.flush()
    time.sleep(1.0)
    s_snk1.write(b"reset\r\n")
    s_snk2.write(b"reset\r\n")
    time.sleep(5.0) # Wait for sinks to connect
    
    # Clear buffers to avoid reading old state transitions
    s_snk1.read_all()
    s_snk2.read_all()
    
    samples = []
    t_end = time.time() + duration
    while time.time() < t_end:
        read_all(s_src, samples, "SOURCE")
        read_all(s_snk1, samples, "SINK1")
        read_all(s_snk2, samples, "SINK2")
        time.sleep(0.05)
    
    snk1 = [s for s in samples if s['node'] == 'SINK1']
    snk2 = [s for s in samples if s['node'] == 'SINK2']
    
    # Check criteria
    snk1_states = set(s['state'] for s in snk1)
    snk2_states = set(s['state'] for s in snk2)
    
    snk1_plc = max([s['plc'] for s in snk1] + [0])
    snk2_plc = max([s['plc'] for s in snk2] + [0])
    
    # Clock drift evaluation
    # We find telemetry logs that arrived at roughly the same host time
    drifts = []
    for s1 in snk1:
        # Find closest s2
        s2 = min(snk2, key=lambda x: abs(x['ts'] - s1['ts']), default=None)
        if s2 and abs(s2['ts'] - s1['ts']) < 0.5:
            # master_time_us is the sync clock
            drift = abs(s1['master_time_us'] - s2['master_time_us'])
            drifts.append(drift)
    
    avg_drift_ms = (sum(drifts) / len(drifts)) / 1000.0 if drifts else 0.0
    
    print(f"SINK1 States: {snk1_states}, max PLC: {snk1_plc}")
    print(f"SINK2 States: {snk2_states}, max PLC: {snk2_plc}")
    print(f"Avg Clock Drift between SINKs: {avg_drift_ms:.3f} ms")
    
    passed = ('STRM' in snk1_states and len(snk1_states) == 1) and \
             ('STRM' in snk2_states and len(snk2_states) == 1) and \
             (snk1_plc < 60) and (snk2_plc < 60) and (avg_drift_ms < 0.5)
    print(f"TEST 1 RESULT: {'PASS' if passed else 'FAIL'}")
    return passed

def test_2_bumble(s_src, s_snk1, s_snk2, duration=60):
    print("\n--- TEST 2: Bumble Streaming (60s) ---")
    s_src.read_all()
    s_snk1.read_all()
    s_snk2.read_all()
    s_src.write(b"stop\r\n")
    time.sleep(1.0)
    s_src.write(b"pc\r\n") # Set to PC STRM mode
    time.sleep(1.0)
    s_src.close()
    
    # Launch bumble in background
    cmd = ["python", r"apps\audio_ESP_NOW_unicast\pc_unicast_streamer.py", "--source", "synth", "--duration", str(duration + 5)]
    proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    
    time.sleep(3.0)
    s_snk1.write(b"reset\r\n")
    s_snk2.write(b"reset\r\n")
    time.sleep(5.0)
    
    # Clear buffers to avoid reading old state transitions
    s_snk1.read_all()
    s_snk2.read_all()
    
    samples = []
    t_end = time.time() + duration
    while time.time() < t_end:
        read_all(s_snk1, samples, "SINK1")
        read_all(s_snk2, samples, "SINK2")
        time.sleep(0.05)
        
    proc.wait()
    s_src.open()
    s_src.dtr = False
    s_src.rts = False
    time.sleep(3.5) # Wait for source to boot after reopen resets it
    s_src.read_all()
    
    snk1 = [s for s in samples if s['node'] == 'SINK1']
    snk2 = [s for s in samples if s['node'] == 'SINK2']
    snk1_states = set(s['state'] for s in snk1)
    snk2_states = set(s['state'] for s in snk2)
    snk1_plc = max([s['plc'] for s in snk1] + [0])
    snk2_plc = max([s['plc'] for s in snk2] + [0])
    
    print(f"SINK1 States: {snk1_states}, max PLC: {snk1_plc}")
    print(f"SINK2 States: {snk2_states}, max PLC: {snk2_plc}")
    
    passed = ('STRM' in snk1_states and len(snk1_states) == 1) and \
             ('STRM' in snk2_states and len(snk2_states) == 1) and \
             (snk1_plc < 60) and (snk2_plc < 60)
    print(f"TEST 2 RESULT: {'PASS' if passed else 'FAIL'}")
    return passed

def test_3_dropout(s_src, s_snk1, s_snk2):
    print("\n--- TEST 3: SINK Drop-out Test ---")
    s_src.read_all()
    s_snk1.read_all()
    s_snk2.read_all()
    s_src.write(b"stop\r\n")
    time.sleep(1.0)
    s_src.write(b"synth\r\n")
    s_src.write(b"start\r\n")
    time.sleep(3.0)
    # Clear buffers so we only read steady-state STRM during dropout
    s_snk1.read_all()
    s_snk2.read_all()
    
    print("Holding SINK2 in reset for 5s...")
    s_snk2.dtr = True
    s_snk2.rts = True
    
    samples_during = []
    t_end = time.time() + 5.0
    while time.time() < t_end:
        read_all(s_snk1, samples_during, "SINK1")
        time.sleep(0.05)
        
    print("Releasing SINK2 from reset...")
    s_snk2.dtr = False
    s_snk2.rts = False
    
    # Wait for recovery
    samples_after = []
    t_end = time.time() + 10.0
    while time.time() < t_end:
        read_all(s_snk1, samples_after, "SINK1")
        read_all(s_snk2, samples_after, "SINK2")
        time.sleep(0.05)
        
    snk1_during_states = set(s['state'] for s in samples_during)
    snk2_after_states = [s['state'] for s in samples_after if s['node'] == 'SINK2']
    snk1_plc = max([s['plc'] for s in samples_after if s['node'] == 'SINK1'] + [0])
    
    print(f"SINK1 states during dropout: {snk1_during_states}")
    print(f"SINK2 recovery states: {set(snk2_after_states)}")
    
    passed = ('STRM' in snk1_during_states and len(snk1_during_states) == 1) and \
             ('STRM' in snk2_after_states)
    print(f"TEST 3 RESULT: {'PASS' if passed else 'FAIL'}")
    return passed

def test_4_sr_change(s_src, s_snk1, s_snk2):
    print("\n--- TEST 4: Live Change of Sample Rate ---")
    s_src.read_all()
    s_snk1.read_all()
    s_snk2.read_all()
    s_src.write(b"synth\r\n")
    s_src.write(b"start\r\n")
    time.sleep(3.0)
    
    rates = ["48k", "32k", "24k", "16k", "8k"]
    overall_pass = True
    for r in rates:
        print(f"Switching to SR = {r}...")
        s_src.write(f"sr {r}\r\n".encode())
        
        # Give it a moment to apply before clearing buffers
        time.sleep(1.0)
        s_src.read_all()
        s_snk1.read_all()
        s_snk2.read_all()
        
        samples = []
        t_end = time.time() + 4.0
        while time.time() < t_end:
            read_all(s_snk1, samples, "SINK1")
            read_all(s_snk2, samples, "SINK2")
            time.sleep(0.05)
            
        snk1 = [s for s in samples if s['node'] == 'SINK1']
        snk1_sr = snk1[-1]['sr'] if snk1 else "-"
        snk1_states = set(s['state'] for s in snk1)
        
        print(f" -> SINK1 reporting SR: {snk1_sr}, States: {snk1_states}")
        # Strip trailing '0' or 'k' for comparison, e.g. 48.0 vs 48k
        expected = r.replace("k", "")
        if not snk1_sr.startswith(expected) or "SCAN" in snk1_states:
            overall_pass = False

    s_src.write(b"stop\r\n")
    print(f"TEST 4 RESULT: {'PASS' if overall_pass else 'FAIL'}")
    return overall_pass


if __name__ == "__main__":
    s_src, s_snk1, s_snk2 = open_ports()
    try:
        t1 = test_1_continuous(s_src, s_snk1, s_snk2)
        t2 = test_2_bumble(s_src, s_snk1, s_snk2)
        t3 = test_3_dropout(s_src, s_snk1, s_snk2)
        t4 = test_4_sr_change(s_src, s_snk1, s_snk2)
        
        print("\n=== SUMMARY ===")
        print(f"Test 1: {'PASS' if t1 else 'FAIL'}")
        print(f"Test 2: {'PASS' if t2 else 'FAIL'}")
        print(f"Test 3: {'PASS' if t3 else 'FAIL'}")
        print(f"Test 4: {'PASS' if t4 else 'FAIL'}")
    finally:
        s_src.close()
        s_snk1.close()
        s_snk2.close()
