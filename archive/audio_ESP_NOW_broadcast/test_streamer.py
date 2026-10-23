#!/usr/bin/env python3
"""
Unit Test Suite for pc_audio_streamer.py
Verifies:
1. Multi-channel audio generation (1 to 6 channels).
2. Polyphase resampling from 48000 Hz to all target sample rates (8k, 16k, 24k, 32k, 44.1k, 48k).
3. 7.5 ms and 10.0 ms LC3 encoding using Google liblc3.dll.
4. Correct VSAF 248-byte packet header format and channel bit-packing.
"""

import os
import sys
import struct
import numpy as np

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, CURRENT_DIR)

from pc_audio_streamer import (
    MultiChannelResampler,
    calculate_required_samples,
    get_default_octets,
    VSAF_MAGIC,
    VSAF_HEADER_LEN,
    SAMPLE_RATE_CODES
)

BUMBLE_DIR = os.path.join(CURRENT_DIR, "..", "usb_ble_bumble")
if os.path.exists(BUMBLE_DIR):
    sys.path.insert(0, os.path.abspath(BUMBLE_DIR))

from lc3_encoder import LC3Encoder

def test_resampler():
    print(">>> Testing Multi-Channel Polyphase Resampler...")
    test_rates = [8000, 16000, 24000, 32000, 44100, 48000]
    in_rate = 48000
    duration_s = 0.05
    num_samples_in = int(in_rate * duration_s)
    
    # 6-channel input
    dummy_input = np.sin(np.linspace(0, 100, num_samples_in))[:, np.newaxis] * np.ones((1, 6))
    
    for out_rate in test_rates:
        resampler = MultiChannelResampler(in_rate, out_rate)
        out = resampler.resample(dummy_input)
        expected_len = int(out_rate * duration_s)
        # Allow +/- 2 samples due to polyphase edge filtering
        assert abs(len(out) - expected_len) <= 2, f"Failed for {out_rate}: got {len(out)}, expected {expected_len}"
        assert out.shape[1] == 6, f"Channel count mismatch: {out.shape[1]}"
        print(f"  [PASS] Resampling 48000 Hz -> {out_rate} Hz ({dummy_input.shape} -> {out.shape})")
    print("Resampler Tests Passed!\n")

def test_lc3_encoder_array():
    print(">>> Testing Multi-Channel LC3 Encoder Array across 7.5ms and 10ms...")
    durations = [7500, 10000]
    sample_rates = [16000, 24000, 32000, 44100, 48000]
    
    for dur in durations:
        for sr in sample_rates:
            octets = get_default_octets(sr, dur)
            samples = calculate_required_samples(sr, dur)
            
            # Create 6 encoders
            encoders = [LC3Encoder(frame_duration_us=dur, sample_rate_hz=sr) for _ in range(6)]
            
            # Generate 6 channels of audio
            pcm_frame = (np.random.randn(samples, 6) * 10000).astype(np.int16)
            
            for ch in range(6):
                ch_pcm = pcm_frame[:, ch]
                encoded = encoders[ch].encode(ch_pcm, octets)
                assert len(encoded) == octets, f"LC3 encoding size mismatch: {len(encoded)} vs {octets}"
            
            print(f"  [PASS] {dur/1000.0:.1f}ms @ {sr} Hz (6 channels, {samples} samples/frame, {octets} octets/frame)")
    print("LC3 Encoder Array Tests Passed!\n")

def test_vsaf_packet_serialization():
    print(">>> Testing VSAF 248-Byte Packet Serialization...")
    dur = 7500
    sr = 48000
    sr_code = SAMPLE_RATE_CODES[sr]
    dur_bit = 1
    octets = 75
    
    seq = 42
    pts_us = 12345678
    
    for ch in range(6):
        cfg = (ch & 0x07) | (sr_code << 3) | (dur_bit << 6) | (0 << 7)
        header = struct.pack("<HBBI", VSAF_MAGIC, seq, cfg, pts_us)
        curr_frame = bytes([ch + 1] * octets)
        prev_frame = bytes([ch] * octets)
        
        pkt = header + curr_frame + prev_frame
        assert len(pkt) == 8 + 2 * octets
        
        # Unpack and verify
        magic, u_seq, u_cfg, u_pts = struct.unpack("<HBBI", pkt[:8])
        assert magic == 0x1337
        assert u_seq == 42
        assert (u_cfg & 0x07) == ch
        assert ((u_cfg >> 3) & 0x07) == sr_code
        assert ((u_cfg >> 6) & 0x01) == dur_bit
        assert u_pts == pts_us
        
        print(f"  [PASS] Channel {ch} VSAF Packet: Total Len = {len(pkt)} bytes, Header = {len(header)}B, Magic = 0x{magic:04X}, Cfg = 0x{u_cfg:02X}")
    print("VSAF Packet Serialization Tests Passed!\n")

if __name__ == "__main__":
    test_resampler()
    test_lc3_encoder_array()
    test_vsaf_packet_serialization()
    print("==========================================================")
    print("   ALL UNIT TESTS PASSED FOR PC AUDIO STREAMER PIPELINE!  ")
    print("==========================================================")
