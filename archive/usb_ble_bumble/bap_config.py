"""
Bluetooth SIG Basic Audio Profile (BAP) & Public Broadcast Profile (PBP)
Configuration & BASE Descriptor Builder for BLE 5.3 Auracast
"""

import struct
from typing import Optional, List, Tuple, Dict, Union, Any


# --------------------------------------------------------------------------
# 1. Sampling Frequencies (Bluetooth SIG BAP 4.3.1 / Codec Specific Config)
# --------------------------------------------------------------------------
SUPPORTED_SAMPLING_FREQUENCIES: Dict[int, Tuple[int, str]] = {
    8000:   (0x01, "8.0 kHz (Low Bandwidth Speech / Telephony)"),
    11025:  (0x02, "11.025 kHz (Legacy Audio Sampling)"),
    16000:  (0x03, "16.0 kHz (Wideband Speech Standard)"),
    22050:  (0x04, "22.05 kHz (Intermediate Audio)"),
    24000:  (0x05, "24.0 kHz (Medium Quality Audio)"),
    32000:  (0x06, "32.0 kHz (Super-Wideband Audio)"),
    44100:  (0x07, "44.1 kHz (CD Audio Standard)"),
    48000:  (0x08, "48.0 kHz (Broadcast / Studio Audio Standard)"),
    88200:  (0x09, "88.2 kHz (High-Resolution Audio)"),
    96000:  (0x0A, "96.0 kHz (Ultra High-Resolution Audio)"),
}

def get_sampling_frequency_code(sample_rate_hz: int) -> Tuple[int, int, str]:
    """
    Validates and resolves requested sampling frequency to standard Bluetooth SIG byte code.
    If a non-standard rate is requested, rounds to the nearest supported standard rate
    (e.g., 44000 Hz -> 44100 Hz, 47999 Hz -> 48000 Hz).

    Throws ValueError if requested rate is below 7 000 Hz or above 100 000 Hz.

    Returns:
        (resolved_rate_hz, byte_code, description)
    """
    if sample_rate_hz < 7000 or sample_rate_hz > 100000:
        raise ValueError(
            f"Invalid sample rate: {sample_rate_hz} Hz. "
            f"Allowed range is 7 kHz to 100 kHz."
        )

    # Find nearest supported frequency
    nearest_hz = min(SUPPORTED_SAMPLING_FREQUENCIES.keys(), key=lambda f: abs(f - sample_rate_hz))
    code, desc = SUPPORTED_SAMPLING_FREQUENCIES[nearest_hz]
    return nearest_hz, code, desc

def list_supported_sampling_frequencies() -> List[Dict[str, Any]]:
    """Returns a list of all supported sampling frequencies with descriptions."""
    return [
        {"sample_rate_hz": hz, "code": code, "description": desc}
        for hz, (code, desc) in SUPPORTED_SAMPLING_FREQUENCIES.items()
    ]


# --------------------------------------------------------------------------
# 2. Presentation Delay (Bluetooth SIG BAP Level 1 Field)
# --------------------------------------------------------------------------
def get_presentation_delay(delay_ms: float = 40.0) -> Tuple[int, bytes, str]:
    """
    Converts requested presentation delay in milliseconds to Bluetooth SIG 24-bit
    microsecond representation (3 bytes, Little-Endian).

    Clamps requested delay to valid Bluetooth range (5.0 ms to 250.0 ms).

    Returns:
        (delay_us, delay_bytes_3le, description)
    """
    clamped_ms = max(5.0, min(delay_ms, 250.0))
    delay_us = int(round(clamped_ms * 1000.0))
    # Pack as 24-bit Little Endian (3 bytes)
    delay_bytes = struct.pack('<I', delay_us)[:3]
    desc = f"{clamped_ms:.1f} ms ({delay_us:,} µs presentation delay)"
    return delay_us, delay_bytes, desc


# --------------------------------------------------------------------------
# 3. Frame Duration (Bluetooth SIG BAP LTV Type 0x02)
# --------------------------------------------------------------------------
SUPPORTED_FRAME_DURATIONS: Dict[float, Tuple[int, str]] = {
    7.5:  (0x00, "7.5 ms Frame Duration (Low Latency / Gaming / Interactive)"),
    10.0: (0x01, "10.0 ms Frame Duration (Standard LC3 / High Efficiency / Broadcast)"),
}

def get_frame_duration_code(duration_ms: float = 10.0) -> Tuple[float, int, str]:
    """
    Returns valid Bluetooth SIG frame duration code (0x00 for 7.5ms, 0x01 for 10.0ms).
    Rounds to nearest supported frame duration if an intermediate value is provided.

    Returns:
        (resolved_duration_ms, byte_code, description)
    """
    if abs(duration_ms - 7.5) < abs(duration_ms - 10.0):
        code, desc = SUPPORTED_FRAME_DURATIONS[7.5]
        return 7.5, code, desc
    else:
        code, desc = SUPPORTED_FRAME_DURATIONS[10.0]
        return 10.0, code, desc

def list_supported_frame_durations() -> List[Dict[str, Any]]:
    """Returns all supported frame durations with descriptions."""
    return [
        {"duration_ms": ms, "code": code, "description": desc}
        for ms, (code, desc) in SUPPORTED_FRAME_DURATIONS.items()
    ]


# --------------------------------------------------------------------------
# 4. LTV 3: Octets Per Codec Frame (Target Compressed Size / Bitrate)
# --------------------------------------------------------------------------
def get_octets_per_frame_options(
    sample_rate_hz: int = 48000,
    frame_duration_ms: float = 10.0,
    is_stereo: bool = False
) -> Dict[str, Dict[str, Any]]:
    """
    Calculates standard Bluetooth LC3 quality presets and target octets per frame.
    
    Returns a dictionary of presets ('voice', 'standard', 'high_quality', 'audiophile')
    with byte sizes, resulting bitrates (kbps), and descriptive explanations.
    """
    # Base octets per channel per 10ms frame
    if frame_duration_ms == 7.5:
        presets = {
            "voice": 30,          # ~32 kbps/ch
            "standard": 45,       # 48 kbps/ch
            "high_quality": 60,   # 64 kbps/ch
            "audiophile": 90,     # 96 kbps/ch
        }
    else: # 10.0 ms
        presets = {
            "voice": 40,          # 32 kbps/ch
            "standard": 60,       # 48 kbps/ch
            "high_quality": 100,  # 80 kbps/ch
            "audiophile": 120,    # 96 kbps/ch
        }

    options = {}
    multiplier = 2 if is_stereo else 1
    for name, octets_per_ch in presets.items():
        total_octets = octets_per_ch * multiplier
        bitrate_kbps = int((total_octets * 8 * 1000) / (frame_duration_ms * 1000))
        options[name] = {
            "octets_per_channel": octets_per_ch,
            "total_sdu_octets": total_octets,
            "bitrate_kbps": bitrate_kbps,
            "description": f"{name.replace('_', ' ').title()}: {total_octets} octets ({bitrate_kbps} kbps total)"
        }
    return options

def get_octets_per_codec_frame(
    sample_rate_hz: int = 48000,
    frame_duration_ms: float = 10.0,
    is_stereo: bool = False,
    preset: str = "standard",
    custom_octets: Optional[int] = None
) -> Tuple[int, bytes, str]:
    """
    Resolves target octets per frame for LTV 3.
    
    Returns:
        (octets_per_frame, ltv_bytes, description)
    """
    if custom_octets is not None:
        octets = max(20, min(custom_octets, 400))
        bitrate_kbps = int((octets * 8 * 1000) / (frame_duration_ms * 1000))
        desc = f"Custom: {octets} octets ({bitrate_kbps} kbps)"
    else:
        options = get_octets_per_frame_options(sample_rate_hz, frame_duration_ms, is_stereo)
        preset_clean = preset.lower() if preset.lower() in options else "standard"
        octets = options[preset_clean]["total_sdu_octets"]
        desc = options[preset_clean]["description"]

    # LTV 3: [Length=3, Type=0x04, Octets_Low, Octets_High]
    ltv_bytes = bytes([0x03, 0x04, octets & 0xFF, (octets >> 8) & 0xFF])
    return octets, ltv_bytes, desc


# --------------------------------------------------------------------------
# 5. Metadata LTV: Audio Context Types & Additional Metadata
# --------------------------------------------------------------------------
CONTEXT_TYPE_DEFINITIONS: Dict[str, Tuple[int, str]] = {
    "unspecified":      (0x0001, "Unspecified audio stream"),
    "conversational":   (0x0002, "Conversational audio (telephony, podcast dialog)"),
    "media":            (0x0004, "Media audio (music, movie soundtracks, streaming radio)"),
    "game":             (0x0008, "Game audio (low latency sound effects, game voice)"),
    "instructional":    (0x0010, "Instructional audio (gym/fitness guides, museum tours)"),
    "voice_assistants": (0x0020, "Voice assistant feedback & navigation directions"),
    "live":             (0x0040, "Live performance / concert / stage audio"),
    "sound_effects":    (0x0080, "Sound effects (UI feedback, button clicks)"),
    "notifications":    (0x0100, "Notification alerts & incoming messages"),
    "ringtone":         (0x0200, "Ringtone audio"),
    "alerts":           (0x0400, "Safety warnings and critical alerts"),
    "emergency_alarm":  (0x0800, "Public safety emergency broadcast"),
}

def list_context_type_options() -> List[Dict[str, Any]]:
    """Returns list of all Bluetooth SIG context types with bitmasks and descriptions."""
    return [
        {"name": name, "bitmask": mask, "description": desc}
        for name, (mask, desc) in CONTEXT_TYPE_DEFINITIONS.items()
    ]

def build_metadata_ltv(
    context: Union[str, int, List[Union[str, int]]] = "media",
    program_info: Optional[str] = None,
    language: Optional[str] = None
) -> bytes:
    """
    Builds the Metadata LTV section for BAP Level 2 / Level 3.
    
    Args:
        context: Context name (e.g. 'media', 'live'), bitmask (0x0004), or list of contexts.
        program_info: Optional track/program title string (LTV Type 0x03).
        language: Optional 3-character ISO 639-3 language code (e.g. 'eng', 'swe', 'deu') (LTV Type 0x04).
    """
    ltv = bytearray()

    # 1. Context Type LTV (Type 0x02, Length 3)
    ctx_mask = 0
    if isinstance(context, int):
        ctx_mask = context
    elif isinstance(context, str):
        ctx_mask = CONTEXT_TYPE_DEFINITIONS.get(context.lower(), (0x0004,))[0]
    elif isinstance(context, (list, tuple)):
        for item in context:
            if isinstance(item, int):
                ctx_mask |= item
            elif isinstance(item, str):
                ctx_mask |= CONTEXT_TYPE_DEFINITIONS.get(item.lower(), (0x0004,))[0]
    if ctx_mask == 0:
        ctx_mask = 0x0004 # Default to MEDIA

    ltv.extend([0x03, 0x02, ctx_mask & 0xFF, (ctx_mask >> 8) & 0xFF])

    # 2. Program Info LTV (Type 0x03)
    if program_info:
        p_bytes = program_info.encode('utf-8')
        ltv.extend([len(p_bytes) + 1, 0x03])
        ltv.extend(p_bytes)

    # 3. Language LTV (Type 0x04 - ISO 639-3, 3 ASCII characters)
    if language and len(language) == 3:
        lang_bytes = language.lower().encode('ascii')
        ltv.extend([0x04, 0x04])
        ltv.extend(lang_bytes)

    return bytes(ltv)


# --------------------------------------------------------------------------
# 6. Master BAP Broadcast Audio Source Endpoint (BASE) Builder
# --------------------------------------------------------------------------
def build_base_data(
    num_bis: int = 1,
    is_stereo: bool = True,
    sample_rate: int = 48000,
    frame_duration_ms: float = 10.0,
    presentation_delay_ms: float = 40.0,
    octets_per_frame: Optional[int] = None,
    quality_preset: str = "standard",
    context_type: Union[str, int, List[Union[str, int]]] = "media",
    program_info: Optional[str] = None,
    language: Optional[str] = None,
    channel_allocations: Optional[List[int]] = None
) -> bytes:
    """
    Constructs a fully validated, Bluetooth SIG compliant Broadcast Audio Source Endpoint (BASE)
    data structure for Periodic Advertising in BLE 5.3 Auracast.

    Parameters:
        num_bis: Number of Broadcast Isochronous Streams (BIS) in subgroup (1 to 31).
        is_stereo: True if BIS carries interleaved/joint stereo (Left + Right).
        sample_rate: Requested sample rate in Hz (e.g. 44100, 48000; rounded if needed; 7k-100k).
        frame_duration_ms: Frame duration in ms (7.5 or 10.0 ms).
        presentation_delay_ms: Audio presentation delay in ms (5.0 to 250.0 ms, default 40.0 ms).
        octets_per_frame: Explicit target frame size in bytes, or None to use quality_preset.
        quality_preset: Preset name ('voice', 'standard', 'high_quality', 'audiophile').
        context_type: Bluetooth audio context ('media', 'live', 'conversational', etc.).
        program_info: Optional track/program metadata title string.
        language: Optional 3-letter ISO 639-3 language code (e.g. 'eng', 'swe').
        channel_allocations: Optional custom list of 32-bit channel location bitmasks per BIS.
    """
    # 1. Resolve Sampling Frequency
    resolved_sr, sr_code, _ = get_sampling_frequency_code(sample_rate)

    # 2. Resolve Frame Duration
    resolved_duration, duration_code, _ = get_frame_duration_code(frame_duration_ms)

    # 3. Resolve Presentation Delay (Level 1)
    _, delay_bytes, _ = get_presentation_delay(presentation_delay_ms)

    # 4. Resolve Octets Per Codec Frame (LTV 3)
    _, octets_ltv, _ = get_octets_per_codec_frame(
        sample_rate_hz=resolved_sr,
        frame_duration_ms=resolved_duration,
        is_stereo=is_stereo,
        preset=quality_preset,
        custom_octets=octets_per_frame
    )

    # 5. Build Subgroup Level 2 Codec Specific Configuration
    level2_codec_cfg = bytearray()
    # LTV 1: Sampling Frequency [Len=2, Type=0x01, Value=sr_code]
    level2_codec_cfg.extend([0x02, 0x01, sr_code])
    # LTV 2: Frame Duration [Len=2, Type=0x02, Value=duration_code]
    level2_codec_cfg.extend([0x02, 0x02, duration_code])
    # LTV 3: Octets Per Codec Frame
    level2_codec_cfg.extend(octets_ltv)

    # 6. Build Subgroup Level 2 Metadata
    level2_metadata = build_metadata_ltv(
        context=context_type,
        program_info=program_info,
        language=language
    )

    # Level 1: BASE Header (Presentation Delay + Num Subgroups)
    base = bytearray()
    base.extend(delay_bytes) # 3 bytes Little-Endian presentation delay
    base.append(1)           # Num_Subgroups = 1

    # --- Subgroup 0 (Level 2) ---
    base.append(num_bis)     # Num_BIS in this subgroup

    # Codec ID: LC3 Standard (Coding Format 0x06, Company ID 0x0000, Vendor Codec ID 0x0000)
    base.extend(bytes([0x06, 0x00, 0x00, 0x00, 0x00]))
    base.append(len(level2_codec_cfg))
    base.extend(level2_codec_cfg)
    base.append(len(level2_metadata))
    base.extend(level2_metadata)

    # --- Level 3: Per-BIS Channel Allocations ---
    if channel_allocations is None:
        if is_stereo:
            allocations = [0x00000003] # Front Left (1) | Front Right (2)
        else:
            # Discrete spatial positions (Front Left, Front Right, Front Center, LFE, Surround Left, Surround Right)
            allocations = [1 << i for i in range(num_bis)]
    else:
        allocations = channel_allocations

    for bis_idx in range(1, num_bis + 1):
        alloc = allocations[bis_idx - 1] if bis_idx <= len(allocations) else (1 << (bis_idx - 1))
        base.append(bis_idx) # BIS Index
        # BIS Codec Config (Audio Location: [Len=5, Type=0x03, 32-bit Location])
        bis_cfg = bytes([
            0x05, 0x03,
            alloc & 0xFF,
            (alloc >> 8) & 0xFF,
            (alloc >> 16) & 0xFF,
            (alloc >> 24) & 0xFF
        ])
        base.append(len(bis_cfg))
        base.extend(bis_cfg)

    return bytes(base)
