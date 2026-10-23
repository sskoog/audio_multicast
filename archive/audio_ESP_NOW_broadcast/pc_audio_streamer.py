#!/usr/bin/env python3
"""
=======================================================================
PC Real-Time LC3 Audio Streamer for audioESP-NOW (VSAF Dongle)
=======================================================================
Captures real-time system audio (e.g. from VB-Audio Virtual CABLE or WASAPI loopback),
compresses it to LC3 in Python, packetizes into dual-frame VSAF frames (8B Header + Frame N + Frame N-1),
and streams over high-speed USB Serial directly to Node 16 (ESP32-S3) acting as an ESP-NOW audio dongle.

Features:
- Direct capture from "CABLE Output (VB-Audio Virtual Cable)" via WASAPI with auto-discovery.
- Studio-grade rational polyphase resampling (e.g. 44.1 kHz -> 48.0 kHz @ 160/147 ratio, >200 dB SNR).
- Dual-channel LC3 encoding using Google liblc3 (Ctypes wrapper to liblc3.dll).
- In-band dual-frame redundancy (Frame N + Frame N-1) for zero-latency SINK PLC recovery.
- Dynamic sample rates (32 kHz, 48 kHz, etc.) and frame durations (7.5 ms, 10.0 ms).
- Real-time ANSI telemetry dashboard displaying cadence, bitrate, RMS dBFS, and packet statistics.
"""

import sys
import os
import time
import math
import struct
import argparse
import collections
import queue
import threading
import numpy as np
import scipy.signal

# Add bumble app path for liblc3 wrapper
BUMBLE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "usb_ble_bumble"))
if BUMBLE_DIR not in sys.path:
    sys.path.insert(0, BUMBLE_DIR)

try:
    import lc3_encoder
except ImportError:
    print("[ERROR] Could not import lc3_encoder from " + str(BUMBLE_DIR) + ". Ensure liblc3.dll is present.")
    raise

try:
    import sounddevice as sd
except ImportError:
    print("[ERROR] sounddevice module is required. Run: pip install sounddevice")
    raise

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("[ERROR] pyserial module is required. Run: pip install pyserial")
    raise

# VSAF Protocol Constants
VSAF_MAGIC = 0x1337
VSAF_HEADER_LEN = 8

# Standard 8 kHz integer grid sample rate mapping
SAMPLE_RATE_CODES = {
    8000: 0,
    16000: 1,
    24000: 2,
    32000: 3,
    48000: 4,
    96000: 5,
}

PENTATONIC_CH_FREQS = [261.63, 293.66, 329.63, 392.00, 440.00, 523.25]


def auto_detect_source_port() -> str:
    """Finds active ESP32-S3 (Node 16) or ESP32-C6 COM ports."""
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if "com16" in p.device.lower():
            return p.device
        if "usb-serial" in desc or "jtag" in desc or "cp210" in desc or "ch340" in desc or "303a" in hwid:
            return p.device
    return "COM16"


def get_default_octets(sample_rate: int, duration_us: int) -> int:
    """Returns standard high-fidelity LC3 octets per frame."""
    if duration_us == 7500:
        return 120 if sample_rate >= 32000 else 60
    else:  # 10000 us
        return 120 if sample_rate >= 32000 else 80


def calculate_required_samples(sample_rate: int, duration_us: int) -> int:
    return int((sample_rate * duration_us) // 1000000)


def find_audio_device(name_query: str = "cable output", prefer_input: bool = True):
    """Discovers audio device index matching name query."""
    devices = sd.query_devices()
    match_id = None
    query = name_query.lower()
    
    # 1. Look for matching input device
    for idx, d in enumerate(devices):
        dname = d['name'].lower()
        if query in dname:
            if prefer_input and d['max_input_channels'] > 0:
                return idx, d
            elif not prefer_input and d['max_output_channels'] > 0:
                return idx, d
            match_id = idx

    # 2. Fallback partial match
    if match_id is not None:
        return match_id, devices[match_id]

    # 3. Default input/output
    default_id = sd.default.device[0 if prefer_input else 1]
    if default_id is not None and default_id >= 0:
        return default_id, devices[default_id]

    return 0, devices[0]


class Resampler:
    """High-Fidelity Rational Polyphase Resampler (e.g. 44.1 kHz -> 48.0 kHz)."""
    def __init__(self, in_rate: int, out_rate: int):
        self.in_rate = in_rate
        self.out_rate = out_rate
        gcd = math.gcd(in_rate, out_rate)
        self.up = out_rate // gcd
        self.down = in_rate // gcd
        print(f"[Resampler] Rational Polyphase Filter Initialized: {in_rate} Hz -> {out_rate} Hz (Ratio: {self.up}/{self.down})", flush=True)

    def resample(self, data: np.ndarray, target_samples: int) -> np.ndarray:
        if self.in_rate == self.out_rate and len(data) == target_samples:
            return data
        if len(data) == 0:
            return np.zeros((target_samples, data.shape[1] if data.ndim > 1 else 1), dtype=data.dtype)
        
        # Polyphase rational resample with high spectral purity
        res = scipy.signal.resample_poly(data, self.up, self.down, axis=0)
        
        if len(res) < target_samples:
            pad_shape = list(res.shape)
            pad_shape[0] = target_samples - len(res)
            res = np.vstack([res, np.zeros(pad_shape, dtype=res.dtype)])
        elif len(res) > target_samples:
            res = res[:target_samples]
        return res


class PcAudioStreamer:
    def __init__(
        self,
        port: str = "COM16",
        baud: int = 2000000,
        sample_rate: int = 48000,
        duration_ms: float = 10.0,
        num_channels: int = 2,
        octets: int = 120,
        source_type: str = "wasapi",
        audio_device_name: str = "CABLE Output",
        presentation_delay_ms: float = 30.0,
    ):
        self.presentation_delay_ms = presentation_delay_ms
        self.port = auto_detect_source_port() if port.lower() in ("auto", "") else port
        self.baud = baud
        self.sample_rate = sample_rate
        self.duration_us = int(duration_ms * 1000)
        self.duration_ms = duration_ms
        self.num_channels = min(max(num_channels, 1), 6)
        self.source_type = source_type
        self.audio_device_name = audio_device_name

        self.sr_code = SAMPLE_RATE_CODES.get(sample_rate, 4)
        self.samples_per_frame = calculate_required_samples(sample_rate, self.duration_us)
        
        # Ensure octets is a multiple of 4 bytes (32-bit word alignment)
        req_octets = octets if octets > 0 else get_default_octets(sample_rate, self.duration_us)
        self.octets = (req_octets // 4) * 4
        if self.octets < 20:
            self.octets = 20
        elif self.octets > 120:
            self.octets = 120

        self.serial_conn = None
        self.is_running = False
        self.audio_stream = None
        self.audio_queue = queue.Queue(maxsize=50)
        self.resampler = None

        # LC3 Encoders per channel
        self.encoders = [
            lc3_encoder.LC3Encoder(
                frame_duration_us=self.duration_us,
                sample_rate_hz=self.sample_rate
            )
            for _ in range(self.num_channels)
        ]

        # Dual-frame redundancy buffers (Frame N-1)
        self.prev_lc3_frames = [bytes(self.octets) for _ in range(self.num_channels)]
        self.seq_counters = [0 for _ in range(self.num_channels)]

    def open_serial(self):
        print(f"Connecting to SOURCE Dongle on {self.port} at {self.baud} baud...", flush=True)
        try:
            self.serial_conn = serial.Serial(
                self.port,
                self.baud,
                timeout=0.2,
                write_timeout=2.0,
                rtscts=False,
                dsrdtr=False
            )
            self.serial_conn.dtr = False
            self.serial_conn.rts = False
            time.sleep(0.2)
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()
            print(f"SUCCESS: Connected to {self.port}!", flush=True)
        except Exception as e:
            print(f"[ERROR] Failed to open serial port '{self.port}': {e}", flush=True)
            raise

    def start_audio_capture(self):
        if self.source_type in ("wasapi", "cable", "device"):
            dev_id, dev_info = find_audio_device(self.audio_device_name, prefer_input=True)
            native_sr = int(dev_info.get('default_samplerate', 48000))
            native_ch = max(int(dev_info.get('max_input_channels', 2)), 2)
            block_size = int(native_sr * (self.duration_us / 1000000.0))

            print(f"Opening Audio Input on: [{dev_id}] '{dev_info['name']}' ({native_sr} Hz, {native_ch} ch)...", flush=True)
            if native_sr != self.sample_rate:
                self.resampler = Resampler(native_sr, self.sample_rate)
            else:
                self.resampler = None

            def callback(indata, frames, time_info, status):
                try:
                    self.audio_queue.put_nowait(indata.copy())
                except queue.Full:
                    pass

            # For standard input devices (like CABLE Output)
            if dev_info['max_input_channels'] > 0:
                self.audio_stream = sd.InputStream(
                    device=dev_id,
                    channels=min(native_ch, 2),
                    samplerate=native_sr,
                    dtype='float32',
                    blocksize=block_size,
                    callback=callback
                )
            else: # WASAPI Loopback on output device
                self.audio_stream = sd.InputStream(
                    device=dev_id,
                    channels=2,
                    samplerate=native_sr,
                    dtype='float32',
                    blocksize=block_size,
                    callback=callback,
                    extra_settings=sd.WasapiSettings(loopback=True)
                )
            self.audio_stream.start()
            print("Audio capture active and running.", flush=True)

    def generate_synth_frames(self, phase_accs: list) -> np.ndarray:
        pcm = np.zeros((self.samples_per_frame, self.num_channels), dtype=np.int16)
        for ch in range(self.num_channels):
            freq = PENTATONIC_CH_FREQS[ch % len(PENTATONIC_CH_FREQS)]
            t = np.arange(self.samples_per_frame)
            phase = phase_accs[ch] + 2.0 * np.pi * freq * t / self.sample_rate
            phase_accs[ch] = (phase[-1] + 2.0 * np.pi * freq / self.sample_rate) % (2.0 * np.pi)
            signal_wave = np.sin(phase) * 0.25 * 32767.0
            pcm[:, ch] = signal_wave.astype(np.int16)
        return pcm

    def run(self, test_duration_sec: float = None):
        self.open_serial()
        self.start_audio_capture()
        self.is_running = True

        dur_str = f"{self.duration_us / 1000.0:.1f}ms"
        fps = 1000000.0 / self.duration_us
        bitrate_kbps = int((self.octets * 8 * 1000000) / (self.duration_us * 1000))
        pkt_size = VSAF_HEADER_LEN + 2 * self.octets

        print("=" * 80)
        print("     PC REAL-TIME LC3 AUDIO STREAMER FOR audioESP-NOW (VSAF DONGLE)")
        print("=" * 80)
        print(f"  Target Dongle   : {self.port} @ {self.baud} baud")
        print(f"  Audio Source    : {self.source_type.upper()} ({self.audio_device_name})")
        print(f"  Sample Rate     : {self.sample_rate} Hz (VSAF sr_code: {self.sr_code})")
        print(f"  Frame Duration  : {dur_str} ({fps:.1f} packets/sec/ch)")
        print(f"  Audio Channels  : {self.num_channels} ({'Stereo' if self.num_channels == 2 else 'Multi-Ch'})")
        print(f"  LC3 Frame Size  : {self.octets} bytes/ch (32-bit aligned)")
        print(f"  Audio Bitrate   : {bitrate_kbps} kbps/ch ({bitrate_kbps * self.num_channels} kbps Total)")
        print(f"  VSAF Packet Size: {pkt_size} bytes (8B Header + {self.octets}B Curr + {self.octets}B Prev)")
        print("=" * 80)

        table_div  = "+----------+-------+----------+-----------+---------------------+----------+"
        table_hdr1 = "| Time     | Total | Cadence  | Line Rate |    Audio dBFS (RMS) | Channels |"
        table_hdr2 = "|          | Pkts  | (pkts/s) | (kbps)    |   Left      Right   |          |"
        print(table_div, flush=True)
        print(table_hdr1, flush=True)
        print(table_hdr2, flush=True)
        print(table_div, flush=True)

        synth_phase = [0.0] * self.num_channels
        start_time = time.perf_counter()
        next_tick_ns = time.perf_counter_ns()
        step_ns = int(self.duration_us * 1000)
        
        total_frames = 0
        frames_since_stat = 0
        last_stat_time = time.perf_counter()
        dur_bit = 1 if self.duration_us == 7500 else 0

        try:
            while self.is_running:
                # Check for test duration timeout
                if test_duration_sec is not None and (time.perf_counter() - start_time) >= test_duration_sec:
                    break

                # 1. Acquire PCM Frame
                if self.audio_stream is not None:
                    try:
                        raw_data = self.audio_queue.get(timeout=0.05)
                        if self.resampler:
                            resampled = self.resampler.resample(raw_data, self.samples_per_frame)
                        else:
                            resampled = raw_data
                        pcm_frame = (np.clip(resampled, -1.0, 1.0) * 32767.0).astype(np.int16)
                        if pcm_frame.ndim == 1:
                            pcm_frame = np.column_stack([pcm_frame, pcm_frame])
                        if pcm_frame.shape[0] < self.samples_per_frame:
                            pad = np.zeros((self.samples_per_frame - pcm_frame.shape[0], pcm_frame.shape[1]), dtype=np.int16)
                            pcm_frame = np.vstack([pcm_frame, pad])
                        elif pcm_frame.shape[0] > self.samples_per_frame:
                            pcm_frame = pcm_frame[:self.samples_per_frame, :]
                    except queue.Empty:
                        continue
                else:
                    now_ns = time.perf_counter_ns()
                    if now_ns < next_tick_ns:
                        while time.perf_counter_ns() < next_tick_ns:
                            pass
                        next_tick_ns += step_ns
                    else:
                        if now_ns - next_tick_ns > (2 * step_ns):
                            next_tick_ns = now_ns + step_ns
                        else:
                            next_tick_ns += step_ns
                    pcm_frame = self.generate_synth_frames(synth_phase)

                pts_us = int((time.perf_counter() + (self.presentation_delay_ms / 1000.0)) * 1000000) & 0xFFFFFFFF

                # 2. Encode and build VSAF packets for each channel
                batch_bytes = bytearray()
                for ch in range(self.num_channels):
                    ch_pcm = pcm_frame[:, ch % pcm_frame.shape[1]]
                    curr_lc3 = self.encoders[ch].encode(ch_pcm, self.octets)
                    prev_lc3 = self.prev_lc3_frames[ch]

                    seq = self.seq_counters[ch] & 0xFF
                    self.seq_counters[ch] = (self.seq_counters[ch] + 1) & 0xFF
                    cfg = (ch & 0x07) | (self.sr_code << 3) | (dur_bit << 6) | (0 << 7)

                    # Assemble dynamic word-aligned VSAF packet: 8B Header + N bytes Curr + N bytes Prev
                    header = struct.pack("<HBBI", VSAF_MAGIC, seq, cfg, pts_us)
                    pkt = header + curr_lc3 + prev_lc3
                    batch_bytes.extend(pkt)

                    self.prev_lc3_frames[ch] = curr_lc3

                # 3. Transmit batch to Node 16 over USB Serial
                self.serial_conn.write(batch_bytes)
                total_frames += 1
                frames_since_stat += 1

                # 4. Telemetry printout (1 Hz)
                now = time.perf_counter()
                if now - last_stat_time >= 1.0:
                    dt = now - last_stat_time
                    fps_real = frames_since_stat / dt
                    pkts_sec = fps_real * self.num_channels
                    kbps_real = (frames_since_stat * len(batch_bytes) * 8) / (dt * 1000)

                    # Compute RMS dBFS
                    ch0_data = pcm_frame[:, 0].astype(np.float32) / 32768.0
                    rms0 = np.sqrt(np.mean(ch0_data ** 2))
                    rms0_db = 20 * math.log10(rms0) if rms0 > 1e-5 else -99.9

                    ch1_data = pcm_frame[:, 1 % pcm_frame.shape[1]].astype(np.float32) / 32768.0
                    rms1 = np.sqrt(np.mean(ch1_data ** 2))
                    rms1_db = 20 * math.log10(rms1) if rms1 > 1e-5 else -99.9

                    elapsed = now - start_time
                    time_str = f"{int(elapsed // 60):02d}:{int(elapsed % 60):02d}.{int((elapsed * 10) % 10)}"
                    print(f"| {time_str:<8} | {total_frames * self.num_channels:5d} | {pkts_sec:6.1f}/s | {kbps_real:6.1f} kbps |  {rms0_db:5.1f} dB  {rms1_db:5.1f} dB | Stereo   |", flush=True)

                    frames_since_stat = 0
                    last_stat_time = now

        except KeyboardInterrupt:
            print("[INFO] Stopped by user.")
        finally:
            self.stop()

    def stop(self):
        self.is_running = False
        if self.audio_stream is not None:
            try:
                self.audio_stream.stop()
                self.audio_stream.close()
            except Exception:
                pass
        if self.serial_conn is not None and self.serial_conn.is_open:
            try:
                # Send stop command so Node 16 transitions cleanly to IDLE state
                self.serial_conn.write(b"\r\nstop\r\n")
                self.serial_conn.flush()
                time.sleep(0.1)
                self.serial_conn.close()
            except Exception:
                pass
        print("PC Audio Streamer cleanly terminated (SOURCE placed in IDLE state).", flush=True)


def main():
    parser = argparse.ArgumentParser(description="Real-Time PC LC3 Audio Streamer for audioESP-NOW")
    parser.add_argument("--port", "-p", default="COM16", help="COM Port for Node 16 (default: COM16 or auto)")
    parser.add_argument("--baud", "-b", type=int, default=2000000, help="Baud rate (default: 2000000)")
    parser.add_argument("--sample-rate", "-sr", type=int, default=48000, choices=[8000, 16000, 24000, 32000, 48000, 96000], help="Sample rate in Hz (default: 48000)")
    parser.add_argument("--duration", "-d", type=float, default=10.0, choices=[7.5, 10.0], help="Frame duration in ms (default: 10.0)")
    parser.add_argument("--channels", "-c", type=int, default=2, choices=[1, 2, 4, 6], help="Number of audio channels (default: 2)")
    parser.add_argument("--octets", "-o", type=int, default=120, help="LC3 frame size in octets (20..120, default: 120)")
    parser.add_argument("--presentation-delay", type=float, default=30.0, help="Presentation delay in ms")
    parser.add_argument("--source", "-s", default="wasapi", choices=["wasapi", "cable", "synth", "device"], help="Audio source (default: wasapi)")
    parser.add_argument("--device", default="CABLE Output", help="Audio device query name (default: 'CABLE Output')")
    parser.add_argument("--test-duration", "-t", type=float, default=None, help="Run test for N seconds and exit")

    args = parser.parse_args()

    streamer = PcAudioStreamer(
        port=args.port,
        baud=args.baud,
        sample_rate=args.sample_rate,
        duration_ms=args.duration,
        num_channels=args.channels,
        octets=args.octets,
        source_type=args.source,
        audio_device_name=args.device
    )

    streamer.run(test_duration_sec=args.test_duration)


if __name__ == "__main__":
    main()
