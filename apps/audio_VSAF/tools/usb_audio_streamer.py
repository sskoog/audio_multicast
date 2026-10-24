#!/usr/bin/env python3
"""
Stream audio from the PC into the VSAF2 SOURCE (Node 16) through its TinyUSB UAC1 speaker.

The SOURCE enumerates as a Windows playback device ("Node16 audio (USB Speaker)") and
broadcasts whatever it receives, so this script just plays audio to that device:
  - a music file (MP3/WAV/FLAC via soundfile, looped), or
  - a built-in synthetic music loop (chords + bass + drums + noise bursts) when no file is given.

Uses the WASAPI endpoint at 48 kHz so Windows does not resample (the firmware runs 48 kHz).

Examples:
  python usb_audio_streamer.py --file "data/mp3/Alan Walker - Monster.mp3" --duration 900
  python usb_audio_streamer.py --duration 60            # synthetic music
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np
import sounddevice as sd

SAMPLE_RATE_HZ = 48000
CHANNELS = 2
DEVICE_HINT = "Node16"


def find_output_device(hint: str, hostapi_name: str = "Windows WASAPI") -> int:
    """
    Return the sounddevice index of the playback device whose name contains `hint`.

    Prefers the given host API (WASAPI: native 48 kHz, no resampling), falls back to any host API.

    Args:
        hint: Case-insensitive substring of the device name, e.g. "Node16".
        hostapi_name: Preferred PortAudio host API name.
    Returns:
        Device index usable as `sd.OutputStream(device=...)`.
    Raises:
        RuntimeError: If no matching output device exists (SOURCE not enumerated over USB).
    """
    candidates = []
    for idx, dev in enumerate(sd.query_devices()):
        if dev["max_output_channels"] >= CHANNELS and hint.lower() in dev["name"].lower():
            api = sd.query_hostapis(dev["hostapi"])["name"]
            candidates.append((api != hostapi_name, idx, api, dev["name"]))
    if not candidates:
        raise RuntimeError(f"No playback device matching '{hint}'. Is Node 16 running and enumerated over USB?")
    candidates.sort()
    _, idx, api, name = candidates[0]
    print(f"[DEVICE] #{idx} {name} ({api})")
    return idx


def load_file(path: str, gain_db: float) -> np.ndarray:
    """
    Load an audio file as float32 stereo at 48 kHz.

    Args:
        path: Audio file path (MP3/WAV/FLAC; anything libsndfile decodes).
        gain_db: Gain applied to the file (dB, e.g. -6.0 for headroom).
    Returns:
        Array of shape (frames, 2), float32, range [-1, 1].
    Raises:
        ValueError: If the file is not 48 kHz (no resampler here, to keep the test bit-honest).
    """
    import soundfile as sf

    data, sr = sf.read(path, dtype="float32", always_2d=True)
    if sr != SAMPLE_RATE_HZ:
        raise ValueError(f"{path} is {sr} Hz; provide a 48 kHz file (no resampling in this tool).")
    if data.shape[1] == 1:
        data = np.repeat(data, 2, axis=1)
    data = data[:, :2] * np.float32(10.0 ** (gain_db / 20.0))
    print(f"[FILE] {path}: {data.shape[0] / SAMPLE_RATE_HZ:.1f} s, gain {gain_db:+.1f} dB")
    return np.clip(data, -1.0, 1.0)


def synth_music(seconds: float = 16.0, seed: int = 1) -> np.ndarray:
    """
    Generate a loopable synthetic 'song': 120 BPM chord progression, bass line, kick/snare/hi-hat
    and occasional broadband noise bursts. Wide-band and dynamic, so codec and redundancy paths
    see realistic content (pure sines hide PLC artefacts and compress too well).

    Args:
        seconds: Loop length (s); rounded to whole bars at 120 BPM (2 s per bar).
        seed: RNG seed for the noise components (reproducible runs).
    Returns:
        Array of shape (frames, 2), float32, peak about -6 dBFS.
    """
    rng = np.random.default_rng(seed)
    bars = max(1, int(round(seconds / 2.0)))
    n = bars * 2 * SAMPLE_RATE_HZ
    t = np.arange(n) / SAMPLE_RATE_HZ
    out = np.zeros((n, 2), dtype=np.float64)
    beat = SAMPLE_RATE_HZ // 2  # 0.5 s per beat at 120 BPM

    roots = [220.0, 174.61, 261.63, 196.0]  # A, F, C, G
    for b in range(bars):
        s0, s1 = b * 2 * SAMPLE_RATE_HZ, (b + 1) * 2 * SAMPLE_RATE_HZ
        tb = t[s0:s1] - t[s0]
        root = roots[b % len(roots)]
        env = np.minimum(1.0, tb * 20.0) * np.exp(-tb * 0.6)
        for k, ratio in enumerate((1.0, 1.25, 1.5, 2.0)):
            tone = np.sin(2 * np.pi * root * ratio * tb) + 0.3 * np.sin(2 * np.pi * root * ratio * 2 * tb)
            pan = (0.3 + 0.4 * (k % 2), 0.7 - 0.4 * (k % 2))
            out[s0:s1, 0] += 0.07 * env * tone * pan[0]
            out[s0:s1, 1] += 0.07 * env * tone * pan[1]
        bass = np.sign(np.sin(2 * np.pi * root / 4 * tb)) * 0.12 * np.exp(-(tb % 0.5) * 4.0)
        out[s0:s1] += bass[:, None]

    for i in range(0, n, beat):
        seg = min(beat, n - i)
        tt = np.arange(seg) / SAMPLE_RATE_HZ
        if (i // beat) % 2 == 0:  # kick on beats 1 and 3
            kick = np.sin(2 * np.pi * (50 + 120 * np.exp(-tt * 30)) * tt) * np.exp(-tt * 9) * 0.45
            out[i:i + seg] += kick[:, None]
        else:  # snare on 2 and 4
            snare = rng.standard_normal(seg) * np.exp(-tt * 18) * 0.18
            out[i:i + seg] += snare[:, None]
        for h in (0, beat // 2):  # eighth-note hi-hats
            j = i + h
            hs = min(beat // 4, n - j)
            if hs > 0:
                hat = np.diff(rng.standard_normal(hs + 1)) * np.exp(-np.arange(hs) / SAMPLE_RATE_HZ * 60) * 0.05
                out[j:j + hs, 0] += hat * 0.8
                out[j:j + hs, 1] += hat * 1.2

    peak = np.max(np.abs(out))
    out *= 0.5 / peak if peak > 0 else 1.0
    print(f"[SYNTH] {bars} bars ({n / SAMPLE_RATE_HZ:.0f} s loop), peak -6 dBFS")
    return out.astype(np.float32)


def stream(audio: np.ndarray, device: int, duration_s: float, block_frames: int = 480) -> int:
    """
    Play `audio` in a loop on `device` for `duration_s` seconds.

    Args:
        audio: (frames, 2) float32 samples at 48 kHz.
        device: sounddevice output index.
        duration_s: Total playback time (s).
        block_frames: Frames per callback (480 = one 10 ms VSAF frame).
    Returns:
        Number of PortAudio output underflows seen on the PC side (should be 0; a PC-side
        underflow starves the SOURCE and must not be blamed on the radio link).
    """
    pos = 0
    underflows = 0

    def callback(outdata: np.ndarray, frames: int, _time, status: sd.CallbackFlags) -> None:
        nonlocal pos, underflows
        if status.output_underflow:
            underflows += 1
        end = pos + frames
        if end <= len(audio):
            outdata[:] = audio[pos:end]
        else:
            first = len(audio) - pos
            outdata[:first] = audio[pos:]
            outdata[first:] = audio[: frames - first]
        pos = end % len(audio)

    with sd.OutputStream(device=device, samplerate=SAMPLE_RATE_HZ, channels=CHANNELS, dtype="float32",
                         blocksize=block_frames, latency="low", callback=callback):
        t0 = time.time()
        last_print = t0
        while (now := time.time()) - t0 < duration_s:
            time.sleep(0.2)
            if now - last_print >= 30:
                last_print = now
                print(f"[STREAM] {now - t0:6.0f} s / {duration_s:.0f} s, PC underflows: {underflows}", flush=True)
    print(f"[DONE] Streamed {duration_s:.0f} s, PC-side underflows: {underflows}")
    return underflows


def main() -> int:
    ap = argparse.ArgumentParser(description="Stream music into the VSAF2 SOURCE over USB (UAC1).")
    ap.add_argument("--file", help="48 kHz audio file to loop (MP3/WAV/FLAC). Default: synthetic music.")
    ap.add_argument("--duration", type=float, default=900.0, help="Playback time in seconds (default 900).")
    ap.add_argument("--gain-db", type=float, default=-3.0, help="Gain applied to --file (dB).")
    ap.add_argument("--device", default=DEVICE_HINT, help="Substring of the playback device name.")
    args = ap.parse_args()

    audio = load_file(args.file, args.gain_db) if args.file else synth_music()
    dev = find_output_device(args.device)
    return 1 if stream(audio, dev, args.duration) else 0


if __name__ == "__main__":
    sys.exit(main())
