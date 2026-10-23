"""
Unit Tests for BAP / PBP BASE Descriptor Configuration (bap_config.py)
"""

import sys
import os

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, CURRENT_DIR)

from bap_config import (
    get_sampling_frequency_code,
    list_supported_sampling_frequencies,
    get_presentation_delay,
    get_frame_duration_code,
    list_supported_frame_durations,
    get_octets_per_frame_options,
    get_octets_per_codec_frame,
    list_context_type_options,
    build_metadata_ltv,
    build_base_data
)

def run_tests():
    print("===========================================================")
    print("  BAP / PBP BASE Descriptor Configuration Unit Tests")
    print("===========================================================\n")

    # 1. Sample Rate Fool-Proof Rounding & Bounds
    print("[1/6] Testing Sampling Frequency Resolution...")
    test_rates = [
        (44000, 44100, 0x07),
        (48000, 48000, 0x08),
        (47999, 48000, 0x08),
        (16000, 16000, 0x03),
        (32100, 32000, 0x06),
        (88200, 88200, 0x09),
        (96000, 96000, 0x0A)
    ]
    for in_rate, exp_rate, exp_code in test_rates:
        r_hz, code, desc = get_sampling_frequency_code(in_rate)
        assert r_hz == exp_rate and code == exp_code, f"Failed on {in_rate}: got {r_hz}, code {code}"
        print(f"  * Input: {in_rate:6d} Hz -> Resolved: {r_hz:6d} Hz (Code: 0x{code:02X}) | {desc}")

    # Under 7 kHz test
    try:
        get_sampling_frequency_code(6000)
        assert False, "Should have raised ValueError for 6000 Hz"
    except ValueError as e:
        print(f"  * Caught expected under-limit error: {e}")

    # Over 100 kHz test
    try:
        get_sampling_frequency_code(120000)
        assert False, "Should have raised ValueError for 120000 Hz"
    except ValueError as e:
        print(f"  * Caught expected over-limit error: {e}")

    # 2. Presentation Delay in Milliseconds
    print("\n[2/6] Testing Presentation Delay Calculation...")
    delays = [10.0, 20.0, 40.0, 50.0, 100.0, 300.0]
    for delay_ms in delays:
        delay_us, d_bytes, desc = get_presentation_delay(delay_ms)
        assert len(d_bytes) == 3
        print(f"  * Requested: {delay_ms:5.1f} ms -> {desc} -> Bytes (3-LE): {d_bytes.hex(' ')}")

    # 3. Frame Duration Codes
    print("\n[3/6] Testing Frame Duration Resolution...")
    durations = [(7.5, 7.5, 0x00), (10.0, 10.0, 0x01), (8.0, 7.5, 0x00), (9.5, 10.0, 0x01)]
    for in_dur, exp_dur, exp_code in durations:
        r_dur, code, desc = get_frame_duration_code(in_dur)
        assert r_dur == exp_dur and code == exp_code
        print(f"  * Requested: {in_dur:4.1f} ms -> Resolved: {r_dur:4.1f} ms (Code: 0x{code:02X}) | {desc}")

    # 4. LTV 3: Octets Per Frame Presets
    print("\n[4/6] Testing LTV 3 Octets Per Frame Options...")
    options = get_octets_per_frame_options(48000, 10.0, is_stereo=True)
    for name, opt in options.items():
        print(f"  * Preset '{name:12s}': {opt['description']}")

    # 5. Metadata Audio Context Types
    print("\n[5/6] Testing Metadata Audio Context Types...")
    contexts = list_context_type_options()
    for ctx in contexts[:6]:
        print(f"  * Context '{ctx['name']:16s}' (Bitmask: 0x{ctx['bitmask']:04X}): {ctx['description']}")

    meta_bytes = build_metadata_ltv(context=["media", "live"], program_info="Rock Festival Live", language="eng")
    print(f"  * Combined Metadata Bytes: {meta_bytes.hex(' ')}")

    # 6. Master BASE Construction
    print("\n[6/6] Testing Complete BASE Descriptor Construction...")
    base_bytes = build_base_data(
        num_bis=2,
        is_stereo=False,
        sample_rate=44000, # Will round to 44100
        frame_duration_ms=10.0,
        presentation_delay_ms=40.0,
        quality_preset="high_quality",
        context_type="media",
        program_info="Stereo Tour Broadcast",
        language="swe"
    )
    assert len(base_bytes) > 20
    print(f"  * Generated BASE Descriptor ({len(base_bytes)} bytes): {base_bytes.hex(' ')}")

    print("\n===========================================================")
    print("  ALL BAP CONFIGURATION TESTS COMPLETED SUCCESSFULLY!")
    print("===========================================================")

if __name__ == "__main__":
    run_tests()
