"""Inter-SINK sync self-test for audio_VSAF time sync (README section 6.4).

Turns on `syncprobe` on several SINKs at once. Every SINK then prints, for the
frame whose SOURCE frame time is a full second, the time its first sample starts
playing converted back to the SOURCE clock. The script compares those times per
frame across SINKs and reports the spread (us).

This checks the SINKs against each other as each one measures itself. It does
not include offset-estimation bias, the DAC pipeline, or acoustic paths.

Usage:
    python sync_probe.py [--duration 30] [--ports COM4,COM5,COM23,COM24]
"""
from __future__ import annotations

import argparse
import re
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import soak_test  # noqa: E402  (pyserial monkey patch + open_port with DTR/RTS low)

PLAY_RE = re.compile(r"\[SYNC\] play f=(\d+) m=(\d+) err=(-?\d+)")


def collect(ports: list[str], duration_s: float) -> dict[str, str]:
    """
    Enable the probe on all ports, read for a while, disable it again.

    Inputs:  ports (COM names), duration_s (s).
    Returns: decoded serial text per port.
    """
    sers = {p: soak_test.open_port(p) for p in ports}
    for ser in sers.values():
        ser.read(65536)
        ser.write(b"\r\nsyncprobe 1\r\n")
    bufs = {p: b"" for p in ports}
    t_end = time.time() + duration_s
    while time.time() < t_end:
        for p, ser in sers.items():
            bufs[p] += ser.read(4096)
    for ser in sers.values():
        ser.write(b"\r\nsyncprobe 0\r\n")
    time.sleep(0.2)
    for ser in sers.values():
        ser.close()
    return {p: b.decode("utf-8", "replace") for p, b in bufs.items()}


def play_offsets(text: str, latency_us: int) -> dict[int, int]:
    """
    Parse `[SYNC] play` lines.

    Inputs:  text (serial output), latency_us (playout latency, us).
    Returns: frame time -> (play time - frame time - latency) in us, SOURCE clock.
    """
    out: dict[int, int] = {}
    for m in PLAY_RE.finditer(text):
        f, master = int(m[1]), int(m[2])
        d = (master - f) % 2**32
        if d >= 2**31:
            d -= 2**32
        out[f] = d - latency_us
    return out


def main() -> int:
    """Run the probe, print a per-frame table and the spread summary. Returns exit code."""
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--duration", type=float, default=30.0, help="Probe time (s).")
    ap.add_argument("--ports", default="COM4,COM5,COM23,COM24", help="SINK COM ports.")
    ap.add_argument("--latency-ms", type=int, default=50, help="Playout latency set on the SOURCE (ms).")
    ap.add_argument("--max-spread-us", type=float, default=60.0, help="PASS limit for the mean spread (us).")
    args = ap.parse_args()
    ports = [p.strip() for p in args.ports.split(",") if p.strip()]

    texts = collect(ports, args.duration)
    per_port = {p: play_offsets(t, args.latency_ms * 1000) for p, t in texts.items()}
    frames = sorted({f for d in per_port.values() for f in d})

    print("frame time   " + "".join(f"{p:>8}" for p in ports) + "  spread_us")
    spreads: list[int] = []
    for f in frames:
        vals = [per_port[p].get(f) for p in ports]
        got = [v for v in vals if v is not None]
        if len(got) < 2:
            continue
        spread = max(got) - min(got)
        spreads.append(spread)
        cells = "".join(f"{v:>8}" if v is not None else "       -" for v in vals)
        print(f"{f:<12} {cells}  {spread:>6}")

    for p in ports:
        v = list(per_port[p].values())
        if v:
            print(f"{p}: {len(v)} frames, play - PT mean {statistics.mean(v):+.1f} us, "
                  f"stdev {statistics.pstdev(v):.1f} us")
        else:
            print(f"{p}: no [SYNC] play lines (not streaming / not synced?)")
    if not spreads:
        print("FAIL: no frame seen on two or more SINKs")
        return 1
    mean = statistics.mean(spreads)
    print(f"frames {len(spreads)}  spread mean {mean:.1f} us  median {statistics.median(spreads):.1f} us  "
          f"max {max(spreads)} us")
    ok = mean <= args.max_spread_us
    print("PASS" if ok else "FAIL", f"(mean spread limit {args.max_spread_us:.0f} us)")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
