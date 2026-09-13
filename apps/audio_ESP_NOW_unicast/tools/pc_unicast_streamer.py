#!/usr/bin/env python3
"""
========================================================================================
PC Real-Time LC3 Audio Streamer for audioESP-NOW Unicast Multi-Speaker Network
========================================================================================
Encodes and streams LC3 audio packets directly from the host PC over USB-Serial JTAG to
Node 16 (ESP32-S3 SOURCE in 'PC STRM' mode), which forwards:
  - Channel 0: Left Channel @ 48 kHz LC3 (120B) -> Node 23 (ESP32-C6 Left Speaker)
  - Channel 1: Right Channel @ 48 kHz LC3 (120B) -> Node 24 (ESP32-C6 Right Speaker)

Audio Source Modes:
  1. 'wasapi' / 'cable' / 'device': Live capture of Windows PC audio (Spotify, YouTube, Games, VLC).
  2. 'loopback': Direct loopback capture of Windows default speaker output.
  3. 'mp3': MP3 playlist player from data/mp3 folder.
  4. 'synth' / 'lfo': 0.2 Hz LFO dual-channel sine sweep (220-880 Hz Left, 440-1760 Hz Right).
  5. 'stereo-test': 440 Hz Left, 880 Hz Right distinct test tones.
"""

import sys
import os
import time
import math
import struct
import argparse
import queue
import threading
import random
import subprocess
import numpy as np

# Add bumble app path for liblc3 wrapper
BUMBLE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "usb_ble_bumble"))
if BUMBLE_DIR not in sys.path:
    sys.path.insert(0, BUMBLE_DIR)

try:
    import lc3_encoder
except ImportError:
    alt_dirs = [
        r"C:\Git_ble_audio\apps\usb_ble_bumble",
        os.path.join(os.path.dirname(__file__), "..", "..", "apps", "usb_ble_bumble")
    ]
    imported = False
    for ad in alt_dirs:
        if os.path.exists(ad) and ad not in sys.path:
            sys.path.insert(0, ad)
            try:
                import lc3_encoder
                imported = True
                break
            except ImportError:
                pass
    if not imported:
        print("[ERROR] Could not import lc3_encoder. Ensure liblc3.dll is present in apps/usb_ble_bumble.")
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

VSAF_MAGIC = 0x1337
SAMPLE_RATE_48K = 48000
FRAME_DURATION_US = 10000 # 10.0 ms
SAMPLES_48K = 480
OCTETS_PER_FRAME_48K = 120 # 120 octets = 96 kbps per channel


def auto_detect_source_port() -> str:
    """Finds active ESP32-S3 (Node 16) COM port."""
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if "com16" in p.device.lower():
            return p.device
        if "usb-serial" in desc or "jtag" in desc or "303a" in hwid:
            return p.device
    return "COM16"


def find_audio_device(name_query: str = "cable output", prefer_input: bool = True):
    """
    Discovers audio device index matching name query, prioritizing WASAPI (Host API 2).
    """
    devices = sd.query_devices()
    query = (name_query or "cable").lower()

    # If query is an explicit integer ID
    try:
        dev_id = int(name_query)
        if 0 <= dev_id < len(devices):
            return dev_id, devices[dev_id]
    except (ValueError, TypeError):
        pass

    # 1. Look for matching WASAPI (hostapi 2) device
    for idx, d in enumerate(devices):
        dname = d['name'].lower()
        if d.get('hostapi') == 2 and query in dname:
            if prefer_input and d['max_input_channels'] > 0:
                return idx, d
            elif not prefer_input and d['max_output_channels'] > 0:
                return idx, d

    # 2. Look for any matching device
    for idx, d in enumerate(devices):
        dname = d['name'].lower()
        if query in dname:
            if prefer_input and d['max_input_channels'] > 0:
                return idx, d
            elif not prefer_input and d['max_output_channels'] > 0:
                return idx, d

    # 3. Fallback to default input
    default_id = sd.default.device[0 if prefer_input else 1]
    if default_id is not None and default_id >= 0:
        return default_id, devices[default_id]

    return 0, devices[0]


class PcUnicastStreamer:
    def __init__(
        self,
        port: str = "COM16",
        baud: int = 2000000,
        source_type: str = "wasapi",
        audio_device_name: str = "CABLE Output",
        mp3_folder: str = "data/mp3"
    ):
        self.port = auto_detect_source_port() if port.lower() in ("auto", "") else port
        self.baud = baud
        self.source_type = source_type.lower()
        self.audio_device_name = audio_device_name
        self.mp3_folder = mp3_folder

        self.serial_conn = None
        self.is_running = False
        self.audio_stream = None
        self.audio_queue = queue.Queue(maxsize=100)

        # Google liblc3 Encoders:
        # Left Encoder: 48 kHz, 10.0 ms frame duration (480 samples -> 120 octets)
        self.enc_left = lc3_encoder.LC3Encoder(FRAME_DURATION_US, SAMPLE_RATE_48K)
        # Right Encoder: 48 kHz, 10.0 ms frame duration (480 samples -> 120 octets)
        self.enc_right = lc3_encoder.LC3Encoder(FRAME_DURATION_US, SAMPLE_RATE_48K)

        # MP3 Player state
        self.mp3_proc = None
        self.mp3_playlist = []

        # Synth state
        self.lfo_phase = 0.0
        self.carrier_phase_l = 0.0
        self.carrier_phase_r = 0.0
        self.stereo_phase_l = 0.0
        self.stereo_phase_r = 0.0

    def open_serial(self):
        print(f"Connecting to SOURCE Dongle on {self.port} at {self.baud} baud...", flush=True)
        try:
            self.serial_conn = serial.Serial(
                self.port,
                self.baud,
                timeout=0.1,
                write_timeout=1.0,
                rtscts=False,
                dsrdtr=False
            )
            self.serial_conn.dtr = False
            self.serial_conn.rts = False
            time.sleep(0.1)
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()
            print(f"Waiting 3.5s for SOURCE node to boot after DTR toggle...", flush=True)
            time.sleep(3.5)
            # Ensure it is in PC STREAM mode and started
            print("Sending 'pc' and 'start' commands...", flush=True)
            self.serial_conn.write(b"pc\r\n")
            time.sleep(0.5)
            self.serial_conn.write(b"start\r\n")
            time.sleep(1.0)
            self.serial_conn.reset_input_buffer()
            print(f"SUCCESS: Connected to {self.port}!", flush=True)
        except Exception as e:
            print(f"[ERROR] Failed to open serial port '{self.port}': {e}", flush=True)
            raise

    def start_audio_source(self):
        if self.source_type in ("wasapi", "cable", "device", "loopback"):
            is_loopback = (self.source_type == "loopback")
            prefer_input = not is_loopback
            dev_id, dev_info = find_audio_device(self.audio_device_name, prefer_input=prefer_input)

            max_in = dev_info.get('max_input_channels', 0)
            use_wasapi_loopback = is_loopback or (max_in == 0)

            print(f"Opening WASAPI Audio {'Loopback' if use_wasapi_loopback else 'Input'} on: [{dev_id}] '{dev_info['name']}' (48000 Hz, 2 ch)...", flush=True)

            def callback(indata, frames, time_info, status):
                try:
                    self.audio_queue.put_nowait(indata.copy())
                except queue.Full:
                    pass

            extra_settings = sd.WasapiSettings(loopback=True) if use_wasapi_loopback else None

            self.audio_stream = sd.InputStream(
                device=dev_id,
                channels=2,
                samplerate=SAMPLE_RATE_48K,
                dtype='int16',
                blocksize=SAMPLES_48K,
                callback=callback,
                extra_settings=extra_settings
            )
            self.audio_stream.start()
            print("Live audio capture active @ 48000 Hz Stereo / 100 fps.", flush=True)

        elif self.source_type == "mp3":
            self._init_mp3_player()

    def _init_mp3_player(self):
        valid_exts = (".mp3", ".wav", ".flac", ".m4a", ".aac", ".ogg")
        if os.path.exists(self.mp3_folder):
            self.mp3_playlist = [
                os.path.join(self.mp3_folder, f)
                for f in os.listdir(self.mp3_folder)
                if f.lower().endswith(valid_exts) and os.path.isfile(os.path.join(self.mp3_folder, f))
            ]
        if not self.mp3_playlist:
            print(f"[WARN] No audio files found in '{self.mp3_folder}'. Falling back to synth mode.")
            self.source_type = "synth"
            return
        self._play_next_mp3_track()

    def _play_next_mp3_track(self):
        if self.mp3_proc:
            try:
                self.mp3_proc.kill()
                self.mp3_proc.wait(timeout=0.1)
            except Exception:
                pass
            self.mp3_proc = None

        track = random.choice(self.mp3_playlist)
        clean_name = os.path.basename(track).encode('ascii', errors='replace').decode('ascii')
        print(f"\n[MP3 Track] Now Playing: '{clean_name}' (48 kHz Stereo)", flush=True)

        frame_bytes = SAMPLES_48K * 2 * 2
        cmd = [
            "ffmpeg",
            "-i", track,
            "-f", "s16le",
            "-acodec", "pcm_s16le",
            "-ar", "48000",
            "-ac", "2",
            "-loglevel", "quiet",
            "-"
        ]
        self.mp3_proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            bufsize=frame_bytes * 10
        )

    def generate_synth_frame(self) -> np.ndarray:
        """Generates continuous-phase LFO sine sweep (220-880 Hz Left, 440-1760 Hz Right)."""
        t = np.arange(SAMPLES_48K) / SAMPLE_RATE_48K
        dt = SAMPLES_48K / SAMPLE_RATE_48K

        lfo_mod = np.sin(self.lfo_phase + 2.0 * np.pi * 0.2 * t)
        freq_l = 550.0 + 330.0 * lfo_mod
        freq_r = 1100.0 + 660.0 * lfo_mod

        phase_l = self.carrier_phase_l + 2.0 * np.pi * np.cumsum(freq_l) / SAMPLE_RATE_48K
        phase_r = self.carrier_phase_r + 2.0 * np.pi * np.cumsum(freq_r) / SAMPLE_RATE_48K
        self.carrier_phase_l = phase_l[-1] % (2.0 * np.pi)
        self.carrier_phase_r = phase_r[-1] % (2.0 * np.pi)
        self.lfo_phase = (self.lfo_phase + 2.0 * np.pi * 0.2 * dt) % (2.0 * np.pi)

        pcm_l = (np.sin(phase_l) * 0.6 * 32767.0).astype(np.int16)
        pcm_r = (np.sin(phase_r) * 0.6 * 32767.0).astype(np.int16)
        return np.column_stack([pcm_l, pcm_r])

    def generate_stereo_test_frame(self) -> np.ndarray:
        """Generates 440 Hz on Left and 880 Hz on Right."""
        t = np.arange(SAMPLES_48K) / SAMPLE_RATE_48K
        dt = SAMPLES_48K / SAMPLE_RATE_48K

        phase_l = self.stereo_phase_l + 2.0 * np.pi * 440.0 * t
        phase_r = self.stereo_phase_r + 2.0 * np.pi * 880.0 * t
        self.stereo_phase_l = (self.stereo_phase_l + 2.0 * np.pi * 440.0 * dt) % (2.0 * np.pi)
        self.stereo_phase_r = (self.stereo_phase_r + 2.0 * np.pi * 880.0 * dt) % (2.0 * np.pi)

        pcm_l = (np.sin(phase_l) * 0.6 * 32767.0).astype(np.int16)
        pcm_r = (np.sin(phase_r) * 0.6 * 32767.0).astype(np.int16)
        return np.column_stack([pcm_l, pcm_r])

    def get_mp3_frame(self) -> np.ndarray:
        if not self.mp3_proc or not self.mp3_proc.stdout:
            return np.zeros((SAMPLES_48K, 2), dtype=np.int16)

        frame_bytes = SAMPLES_48K * 2 * 2
        raw_bytes = self.mp3_proc.stdout.read(frame_bytes)

        if len(raw_bytes) < frame_bytes:
            self._play_next_mp3_track()
            if self.mp3_proc and self.mp3_proc.stdout:
                extra = self.mp3_proc.stdout.read(frame_bytes - len(raw_bytes))
                raw_bytes += extra

        if len(raw_bytes) < frame_bytes:
            raw_bytes += bytes(frame_bytes - len(raw_bytes))

        return np.frombuffer(raw_bytes, dtype=np.int16).reshape((SAMPLES_48K, 2))

    def run(self, test_duration_sec: float = None):
        self.open_serial()
        self.start_audio_source()
        self.is_running = True

        print("\n" + "=" * 90)
        print("  HOST PC REAL-TIME LC3 STEREO UNICAST STREAMER -> NODE 16 (ESP32-S3 SOURCE)")
        print("  Routing:")
        print("    - Channel 0: Left Channel @ 48 kHz LC3 (120B)  -> Node 23 (Left Speaker)")
        print("    - Channel 1: Right Channel @ 48 kHz LC3 (120B) -> Node 24 (Right Speaker)")
        print("=" * 90)
        print("|  TIME   | FRAME |  TX RATE  | BITRATE   |           VU METERS (Left / Right)            |")
        print("|---------+-------+-----------+-----------+-----------------------------------------------|")

        seq = 0
        total_frames = 0
        start_time = time.perf_counter()
        last_stat_time = start_time
        frames_since_stat = 0

        # Frame pacer for generated / mp3 modes
        step_ns = 10000000 # 10.0 ms
        next_tick_ns = time.perf_counter_ns() + step_ns

        try:
            while self.is_running:
                if test_duration_sec is not None and (time.perf_counter() - start_time) >= test_duration_sec:
                    print(f"\nCompleted test duration ({test_duration_sec}s). Stopping.")
                    break

                # 1. Acquire 10 ms Stereo PCM Frame (480 samples x 2 channels int16)
                if self.audio_stream is not None:
                    try:
                        pcm_frame = self.audio_queue.get(timeout=0.03)
                        if pcm_frame.ndim == 1:
                            pcm_frame = np.column_stack([pcm_frame, pcm_frame])
                        if pcm_frame.shape[0] < SAMPLES_48K:
                            pad = np.zeros((SAMPLES_48K - pcm_frame.shape[0], 2), dtype=np.int16)
                            pcm_frame = np.vstack([pcm_frame, pad])
                        elif pcm_frame.shape[0] > SAMPLES_48K:
                            pcm_frame = pcm_frame[:SAMPLES_48K, :]
                    except queue.Empty:
                        continue
                elif self.source_type == "mp3":
                    now_ns = time.perf_counter_ns()
                    if now_ns < next_tick_ns:
                        while time.perf_counter_ns() < next_tick_ns:
                            pass
                    next_tick_ns += step_ns
                    pcm_frame = self.get_mp3_frame()
                elif self.source_type == "stereo-test":
                    now_ns = time.perf_counter_ns()
                    if now_ns < next_tick_ns:
                        while time.perf_counter_ns() < next_tick_ns:
                            pass
                    next_tick_ns += step_ns
                    pcm_frame = self.generate_stereo_test_frame()
                else: # synth / lfo
                    now_ns = time.perf_counter_ns()
                    if now_ns < next_tick_ns:
                        while time.perf_counter_ns() < next_tick_ns:
                            pass
                    next_tick_ns += step_ns
                    pcm_frame = self.generate_synth_frame()

                master_time_us = int(time.perf_counter() * 1000000) & 0xFFFFFFFF
                current_seq = seq & 0xFF
                seq = (seq + 1) & 0xFF

                # 2. Encode Left Channel (48 kHz LC3 -> 120 octets)
                pcm_left = np.ascontiguousarray(pcm_frame[:, 0], dtype=np.int16)
                lc3_left = self.enc_left.encode(pcm_left, OCTETS_PER_FRAME_48K)

                # 3. Encode Right Channel (48 kHz LC3 -> 120 octets)
                pcm_right = np.ascontiguousarray(pcm_frame[:, 1], dtype=np.int16)
                lc3_right = self.enc_right.encode(pcm_right, OCTETS_PER_FRAME_48K)

                # 4. Assemble VSAF LC3 Packets:
                # Header format (10B): Magic(uint16), Seq(uint8), Channel(uint8), Octets(uint8), Flags(uint8), MasterTime(uint32)
                # flags: Bit 0..2: SR code (4 for 48k), Bit 3: Dur (0 for 10ms), Bit 5..7: Delay code (3 for 50ms -> 3<<5 = 96)
                flags_48k = 4 | (3 << 5)  # 100 (0x64)
                hdr_left  = struct.pack("<HBBBB I", VSAF_MAGIC, current_seq, 0, OCTETS_PER_FRAME_48K, flags_48k, master_time_us)
                pkt_left  = hdr_left + lc3_left

                hdr_right = struct.pack("<HBBBB I", VSAF_MAGIC, current_seq, 1, OCTETS_PER_FRAME_48K, flags_48k, master_time_us)
                pkt_right = hdr_right + lc3_right

                # 5. Transmit both packets in one atomic USB batch (260 bytes total)
                batch = pkt_left + pkt_right
                self.serial_conn.write(batch)

                total_frames += 1
                frames_since_stat += 1

                # 6. Dashboard Telemetry (1 Hz)
                now = time.perf_counter()
                if now - last_stat_time >= 1.0:
                    dt = now - last_stat_time
                    fps_real = frames_since_stat / dt
                    kbps_real = (frames_since_stat * len(batch) * 8) / (dt * 1000)

                    # Compute RMS dBFS
                    ch0 = pcm_frame[:, 0].astype(np.float32) / 32768.0
                    rms0 = np.sqrt(np.mean(ch0 ** 2))
                    rms0_db = 20 * math.log10(rms0) if rms0 > 1e-5 else -99.9

                    ch1 = pcm_frame[:, 1].astype(np.float32) / 32768.0
                    rms1 = np.sqrt(np.mean(ch1 ** 2))
                    rms1_db = 20 * math.log10(rms1) if rms1 > 1e-5 else -99.9

                    elapsed = now - start_time
                    m, s = divmod(int(elapsed), 60)
                    time_str = f"{m:02d}:{s:02d}.{int((elapsed % 1) * 10)}"

                    def vu_meter(val_db, width=8):
                        if val_db < -60: return "." * width
                        frac = max(0.0, min(1.0, (val_db + 60.0) / 60.0))
                        bars = int(round(frac * width))
                        return "#" * bars + "." * (width - bars)

                    print(f"| {time_str:<8} | {total_frames:>5} | {fps_real:>6.1f}/s  | {kbps_real:>6.0f} kbps | "
                          f"[{vu_meter(rms0_db)}] {rms0_db:>5.1f}dB   "
                          f"[{vu_meter(rms1_db)}] {rms1_db:>5.1f}dB |", flush=True)

                    frames_since_stat = 0
                    last_stat_time = now

        except KeyboardInterrupt:
            print("\nStream stopped by user.")
        finally:
            self.close()

    def close(self):
        self.is_running = False
        if self.audio_stream is not None:
            try: self.audio_stream.stop(); self.audio_stream.close()
            except Exception: pass
        if self.mp3_proc is not None:
            try: self.mp3_proc.kill()
            except Exception: pass
        if self.serial_conn is not None and self.serial_conn.is_open:
            try: self.serial_conn.close()
            except Exception: pass
        print("[Streamer] Shutdown complete.")


def list_devices():
    devices = sd.query_devices()
    print("\nAvailable Audio Devices:")
    for idx, d in enumerate(devices):
        hostapi = d.get('hostapi', 0)
        api_name = "WASAPI" if hostapi == 2 else "MME" if hostapi == 0 else f"API{hostapi}"
        in_ch = d.get('max_input_channels', 0)
        out_ch = d.get('max_output_channels', 0)
        sr = int(d.get('default_samplerate', 0))
        print(f"  [{idx:2d}] {d['name']:<45} ({api_name:<6}) | In: {in_ch:2d} | Out: {out_ch:2d} | {sr} Hz")
    print()


def main():
    parser = argparse.ArgumentParser(description="PC Real-Time LC3 Stereo Audio Streamer for audioESP-NOW Unicast")
    parser.add_argument("--port", type=str, default="COM16", help="Target Dongle Port (default: COM16 or 'auto')")
    parser.add_argument("--baud", type=int, default=2000000, help="Serial Baud Rate (default: 2000000)")
    parser.add_argument("--source", type=str, default="wasapi", choices=["wasapi", "cable", "device", "loopback", "mp3", "stereo-test", "synth"],
                        help="Audio Source Mode (default: wasapi)")
    parser.add_argument("--device", type=str, default="CABLE Output", help="Audio device query name for WASAPI/Cable capture")
    parser.add_argument("--list-devices", action="store_true", help="List all available audio input/output devices and exit")
    parser.add_argument("--mp3-folder", type=str, default="data/mp3", help="Folder containing MP3 audio files (default: data/mp3)")
    parser.add_argument("--duration", type=float, default=None, help="Streaming duration in seconds (optional, runs indefinitely if omitted)")
    args = parser.parse_args()

    if args.list_devices:
        list_devices()
        sys.exit(0)

    streamer = PcUnicastStreamer(
        port=args.port,
        baud=args.baud,
        source_type=args.source,
        audio_device_name=args.device,
        mp3_folder=args.mp3_folder
    )
    streamer.run(test_duration_sec=args.duration)


if __name__ == "__main__":
    main()
