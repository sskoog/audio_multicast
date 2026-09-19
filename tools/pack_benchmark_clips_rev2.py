#!/usr/bin/env python3
"""
pack_benchmark_clips_rev2.py
Extracts a dynamic middle-section from 'data/10s clips/10s 48 kHz mono CLIP Monster.wav'
and resamples into 16-bit mono PCM clips for:
  - 48000 Hz (for VHQ and HQ)
  - 32000 Hz (for MQ)
  - 24000 Hz (for LW)
  - 8000 Hz  (for SUB1 and SUB2)
Each clip contains 4.0 seconds of audio (300-500 LC3 frames, allowing >= 100 frames per pass).
Packages them into data/benchmark_clips_rev2.bin with indexed 4KB header for ESP32 flash partition.
"""

import os
import struct
import subprocess
import wave

SOURCE_WAV = r"data/10s clips/10s 48 kHz mono CLIP Monster.wav"
OUTPUT_BIN = r"data/benchmark_clips_rev2.bin"
SAMPLE_RATES = [48000, 32000, 24000, 8000]
START_OFFSET_SEC = 2.0  # Cut out middle section starting at 2.0s
DURATION_SEC = 4.0      # 4.0 seconds duration (400 frames of 10ms, 533 frames of 7.5ms)

MAGIC = 0x4C433342  # 'LC3B' in hex
VERSION = 2
HEADER_SIZE = 4096   # 4 KB aligned header


def main() -> int:
    """Extract and pack audio benchmark clips.

    Inputs:
        None (uses module constants SOURCE_WAV, SAMPLE_RATES, DURATION_SEC).
    Outputs:
        int: 0 on success, non-zero on failure.
    """
    if not os.path.exists(SOURCE_WAV):
        print(f"Error: Source file not found: {SOURCE_WAV}")
        return 1

    clip_data = []

    for sr in SAMPLE_RATES:
        temp_wav = f"data/temp_{sr}_clip.wav"
        print(f"Generating {sr} Hz 16-bit mono {DURATION_SEC}s dynamic clip (from {START_OFFSET_SEC}s)...")
        cmd = [
            "ffmpeg", "-y", "-ss", str(START_OFFSET_SEC), "-i", SOURCE_WAV,
            "-t", str(DURATION_SEC),
            "-ar", str(sr), "-ac", "1", "-c:a", "pcm_s16le",
            temp_wav
        ]
        res = subprocess.run(cmd, capture_output=True, text=True)
        if res.returncode != 0:
            print(f"FFmpeg error: {res.stderr}")
            return 1

        with wave.open(temp_wav, "rb") as w:
            n_frames = w.getnframes()
            raw_pcm = w.readframes(n_frames)
            target_samples = int(sr * DURATION_SEC)
            target_bytes = target_samples * 2
            if len(raw_pcm) > target_bytes:
                raw_pcm = raw_pcm[:target_bytes]
            elif len(raw_pcm) < target_bytes:
                raw_pcm += b"\x00" * (target_bytes - len(raw_pcm))

            print(f"  -> {sr} Hz: {len(raw_pcm)} bytes ({len(raw_pcm)//2} samples = {len(raw_pcm)/2/sr:.3f}s)")
            clip_data.append((sr, raw_pcm))

        if os.path.exists(temp_wav):
            os.remove(temp_wav)

    # Build Header:
    # uint32_t magic (LC3B)
    # uint32_t version (2)
    # uint32_t clip_count
    # uint32_t header_size
    # entries: (sample_rate, offset, size_bytes, sample_count)
    header = bytearray(HEADER_SIZE)
    struct.pack_into("<IIII", header, 0, MAGIC, VERSION, len(clip_data), HEADER_SIZE)

    current_offset = HEADER_SIZE
    for idx, (sr, pcm) in enumerate(clip_data):
        entry_offset = 16 + idx * 16
        size_bytes = len(pcm)
        sample_count = size_bytes // 2
        struct.pack_into("<IIII", header, entry_offset, sr, current_offset, size_bytes, sample_count)
        current_offset += size_bytes

    with open(OUTPUT_BIN, "wb") as f:
        f.write(header)
        for _, pcm in clip_data:
            f.write(pcm)

    total_size = os.path.getsize(OUTPUT_BIN)
    print(f"\nSuccessfully generated {OUTPUT_BIN}")
    print(f"Total binary size: {total_size} bytes ({total_size / (1024*1024):.2f} MB)")
    print(f"Packed {len(clip_data)} clips with header.")
    return 0


if __name__ == "__main__":
    exit(main())
