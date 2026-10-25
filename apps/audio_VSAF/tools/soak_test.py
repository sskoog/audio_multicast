#!/usr/bin/env python3
"""
VSAF2 soak test: stream music into the SOURCE over USB and log packet loss / PLC on every SINK.

Pass criterion (GOALS.md reliability target): zero PLC (concealed) frames on every SINK and no SINK
leaving STRM during the measurement window. Frames recovered from redundancy (RED) are fine.

Flow:
  1. Open the SOURCE (COM116) and SINK consoles with DTR/RTS held low *before* opening
     (on USB-Serial/JTAG an open with DTR/RTS asserted resets the chip).
  2. Start usb_audio_streamer.py (PC -> Node 16 UAC1 speaker).
  3. Wait until every SINK reports STRM with packets flowing, then send `clear` to zero the
     SINK error counters and start the window.
  4. Log every 1 s heartbeat for --duration seconds; print a summary each minute.
  5. Write raw logs + summary.json to soak_logs/<timestamp>/ and exit 0 only on PASS.

SINK heartbeat middle block (diagnostics.cpp):  SW HW PKTS RED PLC DMA OVF UDR
  PKTS = packets/s for this SINK's channel (100/s expected at 10 ms frames)
  RED, PLC, DMA (I2S DMA underruns), OVF/UDR (FIFO over/underflow) are running totals printed
  modulo 1000; a decrease while the SINK stays in STRM is unwrapped (+1000), a decrease after
  leaving STRM is a reset (the firmware zeroes them when streaming restarts).

Example:
  python soak_test.py --duration 900 --file "../../../data/mp3/Alan Walker - Monster.mp3"
"""

from __future__ import annotations

import argparse
import re
import statistics
import json
import os
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime

import serial
import serial.serialwin32 as sw

# --- usbser.sys error 31 on TinyUSB CDC (COM116) is harmless: ignore it on configure -------------
_orig_reconf = sw.Serial._reconfigure_port


def _safe_reconf(self) -> None:
    try:
        _orig_reconf(self)
    except serial.SerialException as e:
        if "31" not in str(e):
            raise


sw.Serial._reconfigure_port = _safe_reconf

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
APP_DIR = os.path.dirname(TOOLS_DIR)
REPO_ROOT = os.path.dirname(os.path.dirname(APP_DIR))

# node_id -> (port, label); matches AGENTS.md hardware registry
DEFAULT_SINKS = {4: ("COM4", "L  ch0"), 5: ("COM5", "R  ch1"), 23: ("COM23", "C  ch2"), 24: ("COM24", "Sub ch5")}
SOURCE_PORT = "COM116"
FRAMES_PER_S = 100  # 10 ms frames
SINK_STATES = {"IDLE", "SCAN", "PREF", "STRM", "CAST"}
MID_FIELDS = ("sw", "hw", "pkts", "red", "plc", "dma", "ovf", "udr")
COUNTERS = ("red", "plc", "dma", "ovf", "udr")
COUNTER_MODULO = 1000  # diagnostics.cpp prints running totals % 1000 (3-char columns)
WRAP_WINDOW = 150      # a wrap must start within this distance of 1000 (counters grow <= 100/s)


def open_port(port: str) -> serial.Serial:
    """Open a console port at 115200 with DTR/RTS low from the start (no reset on open)."""
    s = serial.Serial()
    s.port = "\\\\.\\" + port
    s.baudrate = 115200
    s.timeout = 0.2
    s.write_timeout = 1.0
    s.dsrdtr = False
    s.rtscts = False
    s.dtr = False
    s.rts = False
    s.open()
    return s


def parse_sink_line(line: str) -> dict | None:
    """
    Parse one SINK heartbeat row.

    Args:
        line: Text line from the console.
    Returns:
        {"state": str, "pkts": int|None, "red": int, "plc": int, ...} or None if not a heartbeat row.
    """
    if not line.startswith("|"):
        return None
    seg = [s.strip() for s in line.split("|")]
    if len(seg) < 8 or seg[2][:4] not in SINK_STATES:
        return None
    mid = seg[6].split()
    if len(mid) != len(MID_FIELDS):
        return None
    row = {"state": seg[2][:4], "role": seg[3]}

    def num(tok: str) -> int | None:
        return int(tok) if tok.isdigit() else None

    for name, tok in zip(MID_FIELDS, mid):
        row[name] = num(tok)
    rssi = seg[4].split()
    row["rssi"] = int(rssi[0]) if rssi and rssi[0].lstrip("-").isdigit() else None
    # FIFO / drift segment (firmware with drift compensation): "avg min adj ppm", e.g. " 2.4   1   +1  +23.4"
    row["fifo_avg"] = row["fifo_min"] = row["drift_adj"] = row["drift_ppm"] = None
    if len(seg) >= 10:
        drift = seg[7].split()
        if len(drift) == 4:
            def flt(tok: str) -> float | None:
                try:
                    return float(tok)
                except ValueError:
                    return None
            row["fifo_avg"], row["drift_ppm"] = flt(drift[0]), flt(drift[3])
            row["fifo_min"] = int(drift[1]) if drift[1].isdigit() else None
            row["drift_adj"] = int(drift[2]) if drift[2].lstrip("+-").isdigit() else None
    # Time-sync segment (VSAF 3.1): "LOCK    +5" / "SYNC  -  " / "FREE  -  " / " -    -  "
    row["sync_state"] = row["sync_err_us"] = None
    if len(seg) >= 11:
        sy = seg[8].split()
        if sy and sy[0] in ("LOCK", "SYNC", "FREE"):
            row["sync_state"] = sy[0]
            if len(sy) > 1 and sy[1].lstrip("+-").isdigit():
                row["sync_err_us"] = int(sy[1])
    return row


PLAY_RE = re.compile(r"\[SYNC\] play f=(\d+) m=(\d+) err=(-?\d+)")
PTP_COUNT_RE = re.compile(r"coarse aligns (\d+), fine corrections (\d+)")
SYNC_BUCKET_S = 600.0        # spread statistics per 10 min
SYNC_SPREAD_LIMIT_US = 60.0  # GOALS.md: within one to a few samples (20-60 us)


class SyncMonitor:
    """
    Cross-SINK sync accounting from the `[SYNC] play` probe lines (README section 6.4).

    Every SINK reports, for the frame whose SOURCE frame time is a full second, the SOURCE-clock time its
    first sample starts playing. Frames seen on two or more SINKs give one spread sample (max - min, us).
    """

    def __init__(self, latency_us: int = 50000) -> None:
        self.latency_us = latency_us
        self.frames: dict[int, dict[int, int]] = {}   # frame time -> {node: play - PT (us)}
        self.frame_t_rel: dict[int, float] = {}       # frame time -> window time (s) of first report

    def add(self, node: int, text: str, t_rel: float) -> None:
        """Record one probe line (t_rel: s since the window opened)."""
        m = PLAY_RE.search(text)
        if not m:
            return
        f, master = int(m[1]), int(m[2])
        d = (master - f) % 2**32
        if d >= 2**31:
            d -= 2**32
        # Frame times repeat after the 71.6 min wrap: key on (frame, wrap epoch) via the window time
        key = (f, int(t_rel // 3600))
        self.frames.setdefault(key, {})[node] = d - self.latency_us
        self.frame_t_rel.setdefault(key, t_rel)

    def summary(self) -> dict:
        """
        Returns: {"frames": n, "spread_us": {n, mean, median, p95, max}, "buckets": [...],
                  "per_node_mean_us": {node: mean}, "per_node_stdev_us": {node: stdev}}.
        """
        samples = sorted((self.frame_t_rel[k], max(v.values()) - min(v.values()))
                         for k, v in self.frames.items() if len(v) >= 2)
        per_node: dict[int, list] = {}
        for v in self.frames.values():
            for n, x in v.items():
                per_node.setdefault(n, []).append(x)

        def stats(xs: list) -> dict:
            if not xs:
                return {}
            xs = sorted(xs)
            return {"n": len(xs), "mean": round(statistics.mean(xs), 1), "median": statistics.median(xs),
                    "p95": xs[min(len(xs) - 1, int(0.95 * len(xs)))], "max": xs[-1]}

        buckets = []
        if samples:
            b = 0.0
            while b <= samples[-1][0]:
                xs = [sp for t, sp in samples if b <= t < b + SYNC_BUCKET_S]
                if xs:
                    buckets.append({"from_min": round(b / 60), **stats(xs)})
                b += SYNC_BUCKET_S
        return {"frames": len(samples), "spread_us": stats([sp for _, sp in samples]), "buckets": buckets,
                "per_node_mean_us": {n: round(statistics.mean(x), 1) for n, x in per_node.items()},
                "per_node_stdev_us": {n: round(statistics.pstdev(x), 1) for n, x in per_node.items()}}


@dataclass
class SinkStats:
    node: int
    port: str
    label: str
    rows: int = 0
    last_state: str = "?"
    pkts_sum: int = 0
    pkts_secs: int = 0
    pkts_min: int | None = None
    rssi: list = field(default_factory=list)
    last: dict = field(default_factory=dict)      # last cumulative counter values
    totals: dict = field(default_factory=lambda: {c: 0 for c in COUNTERS})
    counter_glitches: int = 0                      # counter decreased (reset/truncation)
    state_drops: list = field(default_factory=list)
    plc_events: list = field(default_factory=list)  # (t_rel_s, delta)
    fifo_min: int | None = None                    # lowest FIFO depth seen in the window (frames)
    fifo_avg: list = field(default_factory=list)   # per-second average buffered audio, FIFO + I2S DMA (frames)
    drift_adj_total: int = 0                       # net drift corrections in the window (+ insert / - drop)
    drift_ppm_last: float | None = None            # firmware estimate since STREAM start (+ = SINK clock fast)
    sync_lock_secs: int = 0                        # streaming seconds with SYNC = LOCK
    sync_unlocked_secs: int = 0                    # streaming seconds with SYNC / FREE
    sync_err_max_abs: int = 0                      # largest |filtered playout error| while locked (us)
    ptp_counts: list = field(default_factory=list)  # (coarse aligns, fine corrections) from `ptp` replies
    in_window: bool = False

    def update(self, row: dict, t_rel: float) -> None:
        """Accumulate one heartbeat row (only counted while the measurement window is open)."""
        self.rows += 1
        if not self.in_window:
            self.last_state = row["state"]
            self.last = {c: row[c] for c in COUNTERS}
            return
        prev_state = self.last_state
        if row["state"] != "STRM" and prev_state == "STRM":
            self.state_drops.append((round(t_rel, 1), row["state"]))
        self.last_state = row["state"]
        if row["state"] == "STRM" and row["pkts"] is not None:
            self.pkts_sum += row["pkts"]
            self.pkts_secs += 1
            self.pkts_min = row["pkts"] if self.pkts_min is None else min(self.pkts_min, row["pkts"])
        if row["rssi"] is not None:
            self.rssi.append(row["rssi"])
        if row["state"] == "STRM":
            if row.get("fifo_min") is not None:
                self.fifo_min = row["fifo_min"] if self.fifo_min is None else min(self.fifo_min, row["fifo_min"])
            if row.get("fifo_avg") is not None:
                self.fifo_avg.append(row["fifo_avg"])
            if row.get("drift_adj") is not None:
                self.drift_adj_total += row["drift_adj"]
            if row.get("drift_ppm") is not None:
                self.drift_ppm_last = row["drift_ppm"]
            if row.get("sync_state") == "LOCK":
                self.sync_lock_secs += 1
                if row.get("sync_err_us") is not None:
                    self.sync_err_max_abs = max(self.sync_err_max_abs, abs(row["sync_err_us"]))
            elif row.get("sync_state") in ("SYNC", "FREE"):
                self.sync_unlocked_secs += 1
        hidden_resync = False
        for c in COUNTERS:
            v = row[c]
            if v is None:
                continue
            prev = self.last.get(c)
            if prev is None:
                self.last[c] = v
                continue
            if v >= prev:
                d = v - prev
            elif prev >= COUNTER_MODULO - WRAP_WINDOW and prev_state == "STRM" and row["state"] == "STRM":
                # Firmware prints totals modulo 1000. A counter grows at most 100/s, so a real wrap
                # can only start from the top of the range.
                d = v + COUNTER_MODULO - prev
            else:
                # Counters are zeroed when a SINK (re)enters streaming. If both samples say STRM, the
                # SINK dropped out and re-synced within one heartbeat (an audible gap, not a wrap).
                self.counter_glitches += 1
                if prev_state == "STRM" and row["state"] == "STRM":
                    hidden_resync = True
                d = v
            self.totals[c] += d
            if c == "plc" and d:
                self.plc_events.append((round(t_rel, 1), d))
            self.last[c] = v
        if hidden_resync:
            self.state_drops.append((round(t_rel, 1), "resync"))

    def summary(self) -> dict:
        # Primary-packet loss = frames that needed redundancy (RED) or concealment (PLC), per frame
        # played. Not derived from PKTS: the firmware's per-second rate reads ~0.8 % low on every node
        # (SOURCE TOT counter grows at exactly 600/s while its 1/s column shows ~595).
        frames = self.pkts_secs * FRAMES_PER_S
        loss_pct = 100.0 * (self.totals["red"] + self.totals["plc"]) / frames if frames else None
        return {
            "node": self.node, "port": self.port, "label": self.label, "heartbeats": self.rows,
            "streaming_seconds": self.pkts_secs, "pkts_per_s_avg": round(self.pkts_sum / self.pkts_secs, 2) if self.pkts_secs else None,
            "pkts_per_s_min": self.pkts_min, "packet_loss_pct": round(loss_pct, 3) if loss_pct is not None else None,
            "rssi_avg_dbm": round(sum(self.rssi) / len(self.rssi), 1) if self.rssi else None,
            **{f"{c}_total": self.totals[c] for c in COUNTERS},
            "plc_events": self.plc_events[:200], "state_drops": self.state_drops, "counter_glitches": self.counter_glitches,
            "fifo_min": self.fifo_min,
            "fifo_avg": round(sum(self.fifo_avg) / len(self.fifo_avg), 2) if self.fifo_avg else None,
            "drift_adj_total": self.drift_adj_total, "drift_ppm": self.drift_ppm_last,
            "sync_lock_secs": self.sync_lock_secs, "sync_unlocked_secs": self.sync_unlocked_secs,
            "sync_err_max_abs_us": self.sync_err_max_abs,
            "coarse_aligns_in_window": (self.ptp_counts[-1][0] - self.ptp_counts[0][0])
            if len(self.ptp_counts) >= 2 else None,
            "fine_corrections_in_window": (self.ptp_counts[-1][1] - self.ptp_counts[0][1])
            if len(self.ptp_counts) >= 2 else None,
        }


def parse_source_line(line: str) -> dict | None:
    """
    Parse one SOURCE heartbeat row: STAGE TIMINGS (ms) DSP/EncHQ/EncRed/TXend and SOURCE PKTS (1/s).

    The dual-core pipeline reports DSP, HQ (+ sub) and MQ encode CPU per frame (summed over both cores) and
    TXend = offset of the frame's last TX completion from the frame tick (the critical path, not CPU).

    Returns:
        {"stages_ms": [dsp, enc_hq, enc_red, tx_end], "pkts": int, "phy": "ch PHY"} or None.
    """
    if not line.startswith("|"):
        return None
    seg = [s.strip() for s in line.split("|")]
    if len(seg) < 9 or not seg[2].startswith("CAST"):
        return None
    try:
        stages = [float(x) for x in seg[6].split()[:4]]
        pkts = int(seg[7].split()[1])
    except (ValueError, IndexError):
        return None
    wifi = seg[4].split()
    return {"stages_ms": stages, "pkts": pkts, "phy": " ".join(wifi[1:3]) if len(wifi) >= 3 else "?"}


@dataclass
class SourceStats:
    """
    SOURCE frame-budget accounting (GOALS.md 10 ms metronome), dual-core pipeline.

    tx_end: when the frame's last packet finished on air (ms after the frame tick); must stay below the
    9.5 ms TX deadline. cpu_per_core: (DSP + HQ/sub + MQ encode CPU) / 2 cores, a load figure only.
    """
    rows: int = 0
    tx_end_max_ms: float = 0.0
    tx_end_ms: list = field(default_factory=list)
    cpu_per_core_ms: list = field(default_factory=list)
    low_rate_secs: int = 0  # seconds with < 590 pkt/s (i.e. frames skipped; nominal 600 incl. ~0.8 % reporting bias)
    phy: str = "?"
    in_window: bool = False

    def update(self, row: dict) -> None:
        if not self.in_window:
            return
        self.rows += 1
        dsp, enc_hq, enc_red, tx_end = row["stages_ms"]
        self.cpu_per_core_ms.append((dsp + enc_hq + enc_red) / 2.0)
        self.tx_end_ms.append(tx_end)
        self.tx_end_max_ms = max(self.tx_end_max_ms, tx_end)
        if row["pkts"] < 590:
            self.low_rate_secs += 1
        self.phy = row["phy"]

    def summary(self) -> dict:
        n = len(self.tx_end_ms) or 1
        return {"heartbeats": self.rows, "wifi_ch_phy": self.phy,
                "tx_end_ms_avg": round(sum(self.tx_end_ms) / n, 2),
                "tx_end_ms_max_of_1s_avgs": round(self.tx_end_max_ms, 2),
                "cpu_ms_per_core_avg": round(sum(self.cpu_per_core_ms) / n, 2),
                "low_rate_seconds": self.low_rate_secs}


class PortReader(threading.Thread):
    """Reads one console port line by line, writes a raw log, feeds parsed rows to a callback."""

    def __init__(self, name: str, ser: serial.Serial, log_path: str, on_line) -> None:
        super().__init__(daemon=True)
        self.name, self.ser, self.on_line = name, ser, on_line
        self.log = open(log_path, "w", encoding="utf-8", errors="replace")
        self.stop_evt = threading.Event()
        self.t0 = time.time()

    def run(self) -> None:
        buf = b""
        while not self.stop_evt.is_set():
            try:
                chunk = self.ser.read(4096)
            except serial.SerialException as e:
                self.log.write(f"[{time.time() - self.t0:8.2f}] <<serial error: {e}>>\n")
                time.sleep(0.5)
                continue
            if not chunk:
                continue
            buf += chunk
            *lines, buf = buf.split(b"\n")
            for raw in lines:
                text = raw.decode("utf-8", "replace").rstrip("\r")
                if not text.strip():
                    continue
                t_rel = time.time() - self.t0
                self.log.write(f"[{t_rel:8.2f}] {text}\n")
                self.on_line(text, t_rel)
        self.log.close()

    def send(self, cmd: str, retries: int = 3) -> bool:
        """Send a console command; S3 USB-Serial/JTAG writes sometimes time out, so retry."""
        for attempt in range(retries):
            try:
                self.ser.write(f"\r\n{cmd}\r\n".encode())
                return True
            except serial.SerialException as e:
                last = e
                time.sleep(0.3)
        print(f"[WARN] {self.name}: write '{cmd}' failed after {retries} tries: {last} "
              f"(harmless: counters are measured relative to the window start)")
        return False


def main() -> int:
    ap = argparse.ArgumentParser(description="VSAF2 soak test: zero-PLC check over USB-streamed audio.")
    ap.add_argument("--duration", type=float, default=900.0, help="Measurement window (s), default 900 (15 min).")
    ap.add_argument("--warmup-timeout", type=float, default=60.0, help="Max wait for all SINKs to reach STRM (s).")
    ap.add_argument("--settle", type=float, default=10.0, help="Extra streaming time before the window opens (s).")
    ap.add_argument("--file", default=os.path.join(REPO_ROOT, "data", "mp3", "Alan Walker - Monster.mp3"),
                    help="48 kHz audio file for the streamer ('' = synthetic music).")
    ap.add_argument("--sinks", default=",".join(str(n) for n in DEFAULT_SINKS),
                    help="Comma-separated SINK node IDs (default 4,5,23,24).")
    ap.add_argument("--no-stream", action="store_true", help="Do not start the streamer (audio supplied externally).")
    ap.add_argument("--no-sync", action="store_true", help="Do not run the time-sync probe / checks.")
    ap.add_argument("--latency-ms", type=int, default=50, help="Playout latency set on the SOURCE (ms), for the probe.")
    ap.add_argument("--reanalyze", metavar="RUN_DIR", help="Recompute results from an earlier run's logs.")
    ap.add_argument("--window-start", type=float, help="With --reanalyze: window start in log time (s).")
    args = ap.parse_args()
    if args.reanalyze:
        return reanalyze(args.reanalyze, args.window_start, args.duration if "--duration" in sys.argv else None)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = os.path.join(APP_DIR, "soak_logs", stamp)
    os.makedirs(out_dir, exist_ok=True)
    print(f"[LOG] {out_dir}")

    sinks: dict[int, SinkStats] = {}
    readers: dict[str, PortReader] = {}
    t_window = [None]  # wall time the window opened
    lock = threading.Lock()
    sync = SyncMonitor(args.latency_ms * 1000)

    for node_s in args.sinks.split(","):
        node = int(node_s)
        port, label = DEFAULT_SINKS[node]
        st = SinkStats(node, port, label)
        sinks[node] = st

        def on_line(text: str, _t: float, st: SinkStats = st) -> None:
            row = parse_sink_line(text)
            if row:
                with lock:
                    rel = time.time() - t_window[0] if t_window[0] else 0.0
                    st.update(row, rel)
            elif "[SYNC] play" in text or "coarse aligns" in text:
                with lock:
                    if not st.in_window:
                        return
                    rel = time.time() - t_window[0]
                    sync.add(st.node, text, rel)
                    m = PTP_COUNT_RE.search(text)
                    if m:
                        st.ptp_counts.append((int(m[1]), int(m[2])))

        readers[port] = PortReader(f"Node{node}", open_port(port), os.path.join(out_dir, f"node{node}_{port}.log"), on_line)

    source = SourceStats()

    def on_source_line(text: str, _t: float) -> None:
        row = parse_source_line(text)
        if row:
            with lock:
                source.update(row)

    try:
        readers[SOURCE_PORT] = PortReader("Node16", open_port(SOURCE_PORT), os.path.join(out_dir, "node16_COM116.log"),
                                          on_source_line)
    except serial.SerialException as e:
        print(f"[WARN] SOURCE console {SOURCE_PORT} not available ({e}); continuing with SINK logs only")
    for r in readers.values():
        r.start()

    streamer = None
    if not args.no_stream:
        total = args.warmup_timeout + args.settle + args.duration + 15
        cmd = [sys.executable, "-u", os.path.join(TOOLS_DIR, "usb_audio_streamer.py"), "--duration", str(total)]
        if args.file:
            cmd += ["--file", args.file]
        streamer = subprocess.Popen(cmd, stdout=open(os.path.join(out_dir, "streamer.log"), "w"), stderr=subprocess.STDOUT)
        print(f"[STREAM] started: {os.path.basename(args.file) if args.file else 'synthetic music'}")

    # Wait for every SINK to stream
    t_wait = time.time()
    while time.time() - t_wait < args.warmup_timeout:
        with lock:
            states = {n: s.last_state for n, s in sinks.items()}
        if all(v == "STRM" for v in states.values()):
            break
        if streamer and streamer.poll() is not None:
            print("[ERROR] streamer exited early; see streamer.log")
            break
        time.sleep(1)
    print(f"[WARMUP] states after {time.time() - t_wait:.0f} s: {states}")
    if not all(v == "STRM" for v in states.values()):
        # Don't burn a 15-minute window measuring nothing (happened 2026-10-24 after runtime wifich/phy changes).
        print("[ABORT] not every SINK reached STRM; check the SOURCE (CAST, TX stage time, NODES column) and SINK channel")
        for r in readers.values():
            r.stop_evt.set()
        if streamer and streamer.poll() is None:
            streamer.terminate()
        return 2
    time.sleep(args.settle)

    for st in sinks.values():
        readers[st.port].send("clear")
    time.sleep(1.5)
    with lock:
        t_window[0] = time.time()
        for st in sinks.values():
            st.in_window = True
        source.in_window = True
    if not args.no_sync:
        for st in sinks.values():
            readers[st.port].send("ptp")
            readers[st.port].send("syncprobe 1")
    print(f"[WINDOW] open for {args.duration:.0f} s (counters cleared)")

    next_report = 60
    try:
        while (elapsed := time.time() - t_window[0]) < args.duration:
            time.sleep(1)
            if elapsed >= next_report:
                next_report += 60
                with lock:
                    parts = [f"N{s.node}:{s.last_state} plc={s.totals['plc']} red={s.totals['red']} "
                             f"udr={s.totals['udr']} pk/s={s.pkts_sum / s.pkts_secs if s.pkts_secs else 0:.1f}"
                             for s in sinks.values()]
                if not args.no_sync:
                    with lock:
                        sp = sync.summary()["spread_us"]
                        unl = sum(s.sync_unlocked_secs for s in sinks.values())
                    parts.append(f"sync spread mean {sp.get('mean', '-')} max {sp.get('max', '-')} us, "
                                 f"unlocked s {unl}")
                print(f"[{elapsed / 60:4.1f} min] " + " | ".join(parts), flush=True)
            if streamer and streamer.poll() is not None:
                print("[ERROR] streamer stopped during the window; see streamer.log")
                break
    except KeyboardInterrupt:
        print("[ABORT] interrupted")

    window_s = time.time() - t_window[0]
    if not args.no_sync:
        for st in sinks.values():
            readers[st.port].send("syncprobe 0")
            readers[st.port].send("ptp")
        time.sleep(2.5)
    for r in readers.values():
        r.stop_evt.set()
    if streamer and streamer.poll() is None:
        streamer.terminate()
    time.sleep(0.5)

    with lock:
        results = [s.summary() for s in sinks.values()]
    first_reader = readers[next(iter(sinks.values())).port]
    meta = {"timestamp": stamp, "file": args.file or "synthetic",
            "window_start_log_s": round(t_window[0] - first_reader.t0, 2), "source": source.summary()}
    if not args.no_sync:
        meta["sync"] = sync.summary()
    return finalize(out_dir, results, window_s, meta)


def finalize(out_dir: str, results: list, window_s: float, meta: dict) -> int:
    """
    Decide PASS/FAIL, write summary.json and print the result table.

    Args:
        out_dir: Run directory (soak_logs/<timestamp>).
        results: SinkStats.summary() dicts.
        window_s: Measurement window length (s).
        meta: Extra fields stored in summary.json (timestamp, file, window_start_log_s, ...).
    Returns:
        Process exit code: 0 on PASS, 1 on FAIL.
    """
    passed = all(r["plc_total"] == 0 and not r["state_drops"] and r["streaming_seconds"] > 0.9 * window_s
                 for r in results)
    sync = meta.get("sync")
    sync_pass = None
    if sync:
        sync_pass = (bool(sync["buckets"]) and all(b["mean"] <= SYNC_SPREAD_LIMIT_US for b in sync["buckets"])
                     and all(r.get("sync_unlocked_secs", 0) == 0 and not r.get("coarse_aligns_in_window")
                             for r in results))
        passed = passed and sync_pass
    report = {**meta, "window_s": round(window_s, 1), "pass": passed, "sinks": results}
    with open(os.path.join(out_dir, "summary.json"), "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2)

    print(f"\n=== SOAK RESULT ({window_s / 60:.1f} min) : {'PASS' if passed else 'FAIL'} ===")
    print("(loss% = primary packets lost = (RED + PLC) / frames; pk/s is the firmware's 1 s rate, ~0.8 % low)")
    print(f"{'Node':<6}{'Ch':<9}{'pk/s':>7}{'min':>5}{'loss%':>8}{'RED':>6}{'PLC':>6}{'DMA':>5}{'OVF':>5}{'UDR':>5}{'RSSI':>6}"
          f"{'BUFavg':>8}{'min':>4}{'ADJ':>6}{'ppm':>7}  drops")
    for r in results:
        print(f"{r['node']:<6}{r['label']:<9}{(r['pkts_per_s_avg'] or 0):>7.2f}{(r['pkts_per_s_min'] or 0):>5}"
              f"{(r['packet_loss_pct'] if r['packet_loss_pct'] is not None else float('nan')):>8.3f}"
              f"{r['red_total']:>6}{r['plc_total']:>6}{r['dma_total']:>5}{r['ovf_total']:>5}{r['udr_total']:>5}"
              f"{(r['rssi_avg_dbm'] or 0):>6.0f}"
              f"{(r['fifo_avg'] if r.get('fifo_avg') is not None else float('nan')):>8.2f}"
              f"{(r['fifo_min'] if r.get('fifo_min') is not None else -1):>4}"
              f"{r.get('drift_adj_total', 0):>+6}"
              f"{(r['drift_ppm'] if r.get('drift_ppm') is not None else float('nan')):>+7.1f}  {r['state_drops'] or '-'}")
    src = meta.get("source")
    if src and src.get("heartbeats"):
        over = src["low_rate_seconds"] or src["tx_end_ms_max_of_1s_avgs"] > 9.0
        budget_note = "OVER BUDGET: frames skipped or TX end near the 9.5 ms deadline" if over else "ok"
        print(f"SOURCE {src['wifi_ch_phy']}: TX end avg {src['tx_end_ms_avg']:.2f} ms, worst 1 s avg "
              f"{src['tx_end_ms_max_of_1s_avgs']:.2f} ms, encode+DSP CPU {src['cpu_ms_per_core_avg']:.2f} ms/core/frame, "
              f"seconds < 590 pkt/s: {src['low_rate_seconds']}/{src['heartbeats']}  [{budget_note}]")
    if sync:
        sp = sync["spread_us"]
        print(f"SYNC: {sync['frames']} common frames, SINK-to-SINK spread mean {sp.get('mean')} / median "
              f"{sp.get('median')} / p95 {sp.get('p95')} / max {sp.get('max')} us  "
              f"[{'ok' if sync_pass else 'FAIL'}: every 10-min mean <= {SYNC_SPREAD_LIMIT_US:.0f} us, "
              f"always LOCK, no coarse realign]")
        for b in sync["buckets"]:
            print(f"  {b['from_min']:>3}-{b['from_min'] + 10:<3} min: n {b['n']:>3}  mean {b['mean']:>6}  "
                  f"p95 {b['p95']:>4}  max {b['max']:>4} us")
        for r in results:
            n = r["node"]
            mean = sync["per_node_mean_us"].get(n, sync["per_node_mean_us"].get(str(n)))
            sd = sync["per_node_stdev_us"].get(n, sync["per_node_stdev_us"].get(str(n)))
            print(f"  N{n}: play-PT mean {mean} us stdev {sd} us | lock {r.get('sync_lock_secs')} s, unlocked "
                  f"{r.get('sync_unlocked_secs')} s, |e_filt| max {r.get('sync_err_max_abs_us')} us, coarse "
                  f"{r.get('coarse_aligns_in_window')}, fine {r.get('fine_corrections_in_window')}")
    print(f"[LOG] {out_dir}")
    return 0 if passed else 1


def reanalyze(run_dir: str, start_s: float | None, duration_s: float | None) -> int:
    """
    Replay saved per-node logs of an earlier run through the current accounting logic.

    Args:
        run_dir: soak_logs/<timestamp> directory with node<N>_COM<P>.log files.
        start_s: Window start in log time (s); default from summary.json, else 14.5 s.
        duration_s: Window length (s); default from summary.json, else to the end of the log.
    Returns:
        Exit code as for a live run (0 PASS, 1 FAIL).
    """
    import re
    old = {}
    sp = os.path.join(run_dir, "summary.json")
    if os.path.isfile(sp):
        with open(sp, encoding="utf-8") as f:
            old = json.load(f)
    start = start_s if start_s is not None else old.get("window_start_log_s", 14.5)
    dur = duration_s if duration_s is not None else old.get("window_s", 1e9)
    results = []
    for node, (port, label) in DEFAULT_SINKS.items():
        path = os.path.join(run_dir, f"node{node}_{port}.log")
        if not os.path.isfile(path):
            continue
        st = SinkStats(node, port, label)
        with open(path, encoding="utf-8", errors="replace") as f:
            for line in f:
                m = re.match(r"\[\s*([0-9.]+)\] (.*)$", line.rstrip("\n"))
                if not m:
                    continue
                t, text = float(m.group(1)), m.group(2)
                if t > start + dur:
                    break
                row = parse_sink_line(text)
                if row:
                    st.in_window = t >= start
                    st.update(row, t - start)
        results.append(st.summary())
    source = SourceStats()
    spath = os.path.join(run_dir, "node16_COM116.log")
    if os.path.isfile(spath):
        with open(spath, encoding="utf-8", errors="replace") as f:
            for line in f:
                m = re.match(r"\[\s*([0-9.]+)\] (.*)$", line.rstrip("\n"))
                if not m:
                    continue
                t = float(m.group(1))
                if t > start + dur:
                    break
                row = parse_source_line(m.group(2))
                if row:
                    source.in_window = t >= start
                    source.update(row)
    meta = {k: v for k, v in old.items() if k not in ("sinks", "pass", "window_s")}
    meta.update({"reanalyzed": True, "window_start_log_s": start, "source": source.summary()})
    print(f"[REANALYZE] {run_dir} window {start:.1f} s + {min(dur, 1e6):.0f} s")
    return finalize(run_dir, results, min(dur, 1e6), meta)


if __name__ == "__main__":
    sys.exit(main())
