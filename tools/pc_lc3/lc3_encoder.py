"""
LC3 Audio Codec Ctypes Wrapper (liblc3)
Wraps Google liblc3 for real-time LC3 frame encoding on Windows.
"""

import ctypes
import os
import sys
import numpy as np

# Find compiled liblc3.dll
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
DLL_PATH = os.path.join(CURRENT_DIR, "liblc3.dll")

if not os.path.exists(DLL_PATH):
    raise FileNotFoundError(f"liblc3.dll not found at {DLL_PATH}. Please compile it with GCC.")

# Load DLL
_liblc3 = ctypes.CDLL(DLL_PATH)

# Function signatures
_liblc3.lc3_encoder_size.argtypes = [ctypes.c_int, ctypes.c_int]
_liblc3.lc3_encoder_size.restype = ctypes.c_uint

_liblc3.lc3_setup_encoder.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_void_p]
_liblc3.lc3_setup_encoder.restype = ctypes.c_void_p

_liblc3.lc3_encode.argtypes = [
    ctypes.c_void_p,
    ctypes.c_int,
    ctypes.c_void_p,
    ctypes.c_int,
    ctypes.c_int,
    ctypes.c_void_p
]
_liblc3.lc3_encode.restype = ctypes.c_int

_liblc3.lc3_encoder_disable_ltpf.argtypes = [ctypes.c_void_p]
_liblc3.lc3_encoder_disable_ltpf.restype = None

# Format enum
LC3_PCM_FORMAT_S16 = 0
LC3_PCM_FORMAT_S24 = 1
LC3_PCM_FORMAT_S24_3LE = 2
LC3_PCM_FORMAT_FLOAT = 3


class LC3Encoder:
    """
    High-performance LC3 Audio Encoder for a single channel.
    """
    def __init__(self, frame_duration_us: int = 10000, sample_rate_hz: int = 48000, disable_ltpf: bool = False):
        self.frame_duration_us = frame_duration_us
        self.sample_rate_hz = sample_rate_hz
        self.num_samples = int(sample_rate_hz * frame_duration_us / 1000000)

        CORE_RATES = (8000, 16000, 24000, 32000, 48000, 96000)
        core_sr = sample_rate_hz if sample_rate_hz in CORE_RATES else min(CORE_RATES, key=lambda f: abs(f - sample_rate_hz))

        # Allocate encoder memory context
        self.mem_size = _liblc3.lc3_encoder_size(frame_duration_us, core_sr)
        if self.mem_size == 0:
            raise ValueError(f"Invalid LC3 parameters: dt_us={frame_duration_us}, sr_hz={sample_rate_hz}")

        self._mem = ctypes.create_string_buffer(self.mem_size)
        self._handle = _liblc3.lc3_setup_encoder(frame_duration_us, core_sr, 0, self._mem)
        if not self._handle:
            raise RuntimeError("Failed to initialize liblc3 encoder instance")

        if disable_ltpf:
            _liblc3.lc3_encoder_disable_ltpf(self._handle)

    def encode(self, pcm_data: np.ndarray, num_bytes: int) -> bytes:
        """
        Encodes a 1D NumPy array of int16 PCM samples (length must be self.num_samples)
        into an LC3 frame of length `num_bytes`.
        Guarantees C-contiguous 1D int16 array layout before calling C API.
        """
        # Ensure contiguous 1D int16 array (critical for slices like pcm_frame[:, 0])
        if not isinstance(pcm_data, np.ndarray) or pcm_data.dtype != np.int16 or not pcm_data.flags.c_contiguous:
            pcm_data = np.ascontiguousarray(pcm_data, dtype=np.int16)

        if pcm_data.ndim != 1 or len(pcm_data) != self.num_samples:
            pcm_data = pcm_data.ravel()
            if len(pcm_data) != self.num_samples:
                raise ValueError(f"Expected {self.num_samples} samples, got {len(pcm_data)}")

        # Output buffer
        out_buf = ctypes.create_string_buffer(num_bytes)

        # Encode (stride = 1 for contiguous 1D array)
        pcm_ptr = pcm_data.ctypes.data_as(ctypes.c_void_p)
        ret = _liblc3.lc3_encode(self._handle, LC3_PCM_FORMAT_S16, pcm_ptr, 1, num_bytes, out_buf)
        if ret != 0:
            raise RuntimeError(f"lc3_encode failed with error code {ret}")

        return out_buf.raw
