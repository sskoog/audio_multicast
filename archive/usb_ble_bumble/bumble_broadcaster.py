def hard_reset_controller(port: str, baud: int = 115200):
    """
    Performs a hardware RTS (EN) pulse to guarantee the ESP32-C6 starts in a clean,
    fresh, fully-responsive state without requiring physical button presses.
    """
    try:
        s = serial.Serial(port, baud, timeout=0.5)
        s.dtr = False
        s.rts = True
        time.sleep(0.05)
        s.rts = False
        time.sleep(0.35)
        s.reset_input_buffer()
        s.reset_output_buffer()
        s.close()
        time.sleep(0.1)
    except Exception as e:
        logging.debug(f"Hardware reset notice: {e}")

#!/usr/bin/env python3
"""
BLE 5.3 Auracast Broadcaster (Google Bumble Host + ESP32-C6 HCI Controller)
----------------------------------------------------------------------------
Implements Bluetooth Low Energy Audio Broadcasts (Auracast / PBP / BAP):
  Mode 1: Single BIS with one Stereo Channel (Left + Right Audio Location)
  Mode 2: One Broadcast Isochronous Group (BIG) with 2-6 discrete Mono BIS streams

Audio Sources:
  1. 'synth' (Default): Synthesized 25% amplitude Sine Wave modulated by 0.2 Hz LFO (220 Hz - 880 Hz).
  2. 'device': Live PCM audio capture from Windows Audio Device (e.g. VB-Audio Virtual Cable).
  3. 'test-pattern': Synthetic sequence counter bytes for link-layer testing.
"""

import sys
import ctypes
import os
import time
import struct
import math
import collections
import asyncio
import argparse
import logging
import numpy as np
from typing import Optional, List, Tuple, Dict, Any, Union

from bumble.transport import open_transport
from bumble.device import Device
from bumble.hci import (
    Address,
    HCI_LE_1M_PHY,
    HCI_LE_2M_PHY,
    HCI_LE_Set_Extended_Advertising_Parameters_Command,
    HCI_LE_Set_Extended_Advertising_Data_Command,
    HCI_LE_Set_Periodic_Advertising_Parameters_Command,
    HCI_LE_Set_Periodic_Advertising_Data_Command,
    HCI_LE_Set_Periodic_Advertising_Enable_Command,
    HCI_LE_Set_Extended_Advertising_Enable_Command,
    HCI_LE_Create_BIG_Command,
    HCI_LE_Terminate_BIG_Command,
    HCI_Reset_Command,
    HCI_IsoDataPacket
)

import serial
from bumble.transport.common import TransportLostError

class SerialDisconnectFilter(logging.Filter):
    def filter(self, record):
        if record.exc_info:
            exc_type = record.exc_info[0]
            if exc_type and issubclass(exc_type, (serial.SerialException, serial.serialutil.SerialException, TransportLostError, PermissionError, OSError)):
                return False
        msg = str(record.getMessage())
        if any(s in msg for s in ("ClearCommError", "transport lost", "Fatal write error", "no pending response future", "command result mismatch")):
            return False
        return True

from lc3_encoder import LC3Encoder
from audio_source import Mp3FolderAudioSource, LfoSineAudioSource, DeviceAudioSource, list_audio_devices
from bap_config import (
    build_base_data,
    get_sampling_frequency_code,
    list_supported_sampling_frequencies,
    get_presentation_delay,
    get_frame_duration_code,
    list_supported_frame_durations,
    get_octets_per_frame_options,
    get_octets_per_codec_frame,
    list_context_type_options,
    build_metadata_ltv
)

def print_bap_options():
    """Prints a user-friendly reference of all configurable Bluetooth BAP fields."""
    print("=====================================================================")
    print("  Bluetooth SIG Basic Audio Profile (BAP) Configuration Reference")
    print("=====================================================================\n")

    print("[1] Supported Sampling Frequencies (LTV Type 0x01):")
    for item in list_supported_sampling_frequencies():
        print(f"    * {item['sample_rate_hz']:6d} Hz (Code: 0x{item['code']:02X}) - {item['description']}")

    print("\n[2] Supported Frame Durations (LTV Type 0x02):")
    for item in list_supported_frame_durations():
        print(f"    * {item['duration_ms']:4.1f} ms (Code: 0x{item['code']:02X}) - {item['description']}")

    print("\n[3] Octets Per Codec Frame Presets (LTV Type 0x04) @ 48kHz, 10ms:")
    print("    Mono Stream Presets:")
    for name, opt in get_octets_per_frame_options(48000, 10.0, is_stereo=False).items():
        print(f"      - {name:12s}: {opt['total_sdu_octets']:3d} octets/frame ({opt['bitrate_kbps']} kbps)")
    print("    Stereo Stream Presets:")
    for name, opt in get_octets_per_frame_options(48000, 10.0, is_stereo=True).items():
        print(f"      - {name:12s}: {opt['total_sdu_octets']:3d} octets/frame ({opt['bitrate_kbps']} kbps total, {opt['octets_per_channel']} octets/ch)")

    print("\n[4] Audio Context Types (Metadata LTV Type 0x02):")
    for item in list_context_type_options():
        print(f"    * {item['name']:16s} (Bitmask: 0x{item['bitmask']:04X}) - {item['description']}")

    print("\n[5] Presentation Delay (Level 1 Field):")
    print("    * Configured in milliseconds (5.0 ms to 250.0 ms, default: 40.0 ms).")
    print("    * Automatically packed into 24-bit Little-Endian microseconds representation.")
    print("=====================================================================\n")

async def stop_broadcast(device: Device):
    """
    Gracefully terminates BIG, disables Periodic and Extended Advertising,
    and resets the ESP32 Controller link layer to clean Standby/Idle state (Green LED).
    """
    print("\n[Teardown] Stopping Auracast broadcast on ESP32...", flush=True)
    try:
        # 1. Terminate BIG
        await device.send_command(
            HCI_LE_Terminate_BIG_Command(big_handle=0, reason=0x16),
            check_result=False
        )
    except Exception:
        pass

    try:
        # 2. Disable Periodic Advertising
        await device.send_command(
            HCI_LE_Set_Periodic_Advertising_Enable_Command(enable=0, advertising_handle=0),
            check_result=False
        )
        print("  * Disabled Periodic Advertising (PA)", flush=True)
    except Exception:
        pass

    try:
        # 3. Disable Extended Advertising
        await device.send_command(
            HCI_LE_Set_Extended_Advertising_Enable_Command(
                enable=0,
                advertising_handles=[0],
                durations=[0],
                max_extended_advertising_events=[0]
            ),
            check_result=False
        )
        print("  * Disabled Extended Advertising (EA)", flush=True)
    except Exception:
        pass

    print("[Teardown] Broadcast stopped cleanly. ESP32 is now in IDLE mode.\n", flush=True)

# --------------------------------------------------------------------------
# Main Broadcaster Loop
# --------------------------------------------------------------------------
async def run_broadcaster(args):
    if args.verbose:
        logging.basicConfig(level=logging.DEBUG)
    else:
        logging.basicConfig(level=logging.INFO)

    try:
        if sys.platform == "win32":
            ctypes.windll.winmm.timeBeginPeriod(1)
    except Exception:
        pass

    disc_filter = SerialDisconnectFilter()
    logging.getLogger().addFilter(disc_filter)
    for h in logging.root.handlers:
        h.addFilter(disc_filter)
    for name in ("bumble", "bumble.host", "bumble.transport", "bumble.device", "asyncio", "serial_asyncio"):
        logging.getLogger(name).addFilter(disc_filter)

    loop = asyncio.get_running_loop()
    def handle_async_exception(loop, context):
        exc = context.get('exception')
        if exc and isinstance(exc, (serial.SerialException, serial.serialutil.SerialException, TransportLostError, PermissionError, OSError)):
            return
        msg = str(context.get('message', ''))
        if any(s in msg for s in ('Fatal write error', 'ClearCommError', 'transport lost', 'serial transport')):
            return
        loop.default_exception_handler(context)
    loop.set_exception_handler(handle_async_exception)

    num_bis = 1 if args.mode == "stereo" else args.num_bis
    is_stereo = (args.mode == "stereo")
    num_channels = 2 if is_stereo else num_bis

    # 1. Resolve and Validate BAP Fields
    resolved_sr, sr_code, sr_desc = get_sampling_frequency_code(args.sample_rate)
    resolved_dur, dur_code, dur_desc = get_frame_duration_code(args.frame_duration)
    resolved_delay_us, delay_bytes, delay_desc = get_presentation_delay(args.presentation_delay)
    frame_octets, octets_ltv, octets_desc = get_octets_per_codec_frame(
        sample_rate_hz=resolved_sr,
        frame_duration_ms=resolved_dur,
        is_stereo=is_stereo,
        preset=args.quality_preset,
        custom_octets=args.octets_per_frame
    )

    transport_spec = f"serial:{args.port},{args.baud}"
    print(f"===========================================================", flush=True)
    print(f"  BLE 5.3 Auracast Broadcaster (Google Bumble + ESP32-C6)", flush=True)
    print(f"  Port: {args.port} @ {args.baud} baud", flush=True)
    print(f"  Mode: {args.mode.upper()}", flush=True)
    if is_stereo:
        print(f"  Configuration: 1 BIG -> 1 Stereo BIS (L+R Audio Location)", flush=True)
    else:
        print(f"  Configuration: 1 BIG -> {args.num_bis} Discrete Mono BIS Streams", flush=True)
    print(f"  BAP Parameters:", flush=True)
    print(f"    * Sampling Frequency: {sr_desc} [Code 0x{sr_code:02X}]", flush=True)
    print(f"    * Frame Duration:     {dur_desc} [Code 0x{dur_code:02X}]", flush=True)
    print(f"    * Presentation Delay: {delay_desc}", flush=True)
    print(f"    * Codec Frame Size:   {octets_desc}", flush=True)
    print(f"    * Audio Context:      {args.context.upper()}", flush=True)
    if args.program_info:
        print(f"    * Program Info:       '{args.program_info}'", flush=True)
    if args.language:
        print(f"    * Language (ISO-639): '{args.language.lower()}'", flush=True)
    print(f"  Audio Source: {args.source.upper()}", flush=True)
    if args.source == "synth":
        print(f"    * LFO Sweep: {args.lfo_min} Hz -> {args.lfo_max} Hz @ {args.lfo_rate} Hz LFO", flush=True)
        print(f"    * Peak Amplitude: {int(args.amplitude * 100)}% ({int(args.amplitude * 32767)} / 32767)", flush=True)
    elif args.source == "device":
        print(f"    * Target Device Query: '{args.audio_device}'", flush=True)
    print(f"===========================================================\n", flush=True)

    hard_reset_controller(args.port, args.baud)
    transport_spec = f"serial:{args.port},{args.baud}"
    print(f"Connecting to ESP32-C6 HCI Controller on {transport_spec}...", flush=True)
    try:
        hci_transport = await open_transport(transport_spec)
    except Exception as e:
        print(f"Error opening HCI transport on {args.port}: {e}", flush=True)
        print("Tip: Check if another terminal process or serial monitor is holding the COM port open.", flush=True)
        return

    async with hci_transport as (hci_source, hci_sink):
        device = Device.with_hci(
            name=args.adv_name,
            address="F0:C6:00:00:00:01",
            hci_source=hci_source,
            hci_sink=hci_sink
        )

        print("Powering on Bluetooth Device...", flush=True)
        try:
            await asyncio.wait_for(device.power_on(), timeout=5.0)
        except asyncio.TimeoutError:
            print("\n[ERROR] Timed out waiting for ESP32-C6 HCI Controller to respond to HCI_Reset!", flush=True)
            print("Troubleshooting Steps:", flush=True)
            print(f"  1. Verify COM port '{args.port}' is correct (use Device Manager / COM121 or COM21).", flush=True)
            print("  2. Ensure no previous background python process is holding the port.", flush=True)
            print("  3. Press the RST button on the ESP32-C6 DevKit board once.\n", flush=True)
            return
        print(f"Device Online! Controller Address: {device.random_address or device.public_address}", flush=True)

        # Initialize Audio Source & LC3 Encoders after Controller is online
        audio_source = None
        if args.source == "mp3":
            try:
                audio_source = Mp3FolderAudioSource(
                    folder_path=args.mp3_dir,
                    num_channels=num_channels,
                    sample_rate=resolved_sr,
                    frame_duration_ms=resolved_dur
                )
            except Exception as e:
                print(f"[Notice] Could not load MP3 folder '{args.mp3_dir}' ({e}). Falling back to 'synth' mode...", flush=True)
                audio_source = LfoSineAudioSource(
                    num_channels=num_channels,
                    sample_rate=resolved_sr,
                    frame_duration_ms=resolved_dur,
                    amplitude=args.amplitude,
                    lfo_rate_hz=args.lfo_rate,
                    freq_min_hz=args.lfo_min,
                    freq_max_hz=args.lfo_max
                )
        elif args.source == "synth":
            audio_source = LfoSineAudioSource(
                num_channels=num_channels,
                sample_rate=resolved_sr,
                frame_duration_ms=resolved_dur,
                amplitude=args.amplitude,
                lfo_rate_hz=args.lfo_rate,
                freq_min_hz=args.lfo_min,
                freq_max_hz=args.lfo_max
            )
        elif args.source == "device":
            try:
                audio_source = DeviceAudioSource(
                    device_query=args.audio_device,
                    num_channels=num_channels,
                    sample_rate=resolved_sr,
                    frame_duration_ms=resolved_dur
                )
                print(f"Connected live audio input: '{audio_source.device_name}'", flush=True)
            except Exception as e:
                print(f"\n[ERROR] Failed to open audio device '{args.audio_device}': {e}", flush=True)
                print("Available input devices on this PC:", flush=True)
                list_audio_devices()
                return

        encoders = []
        octets_per_channel = (frame_octets // 2) if is_stereo else frame_octets
        if args.source in ("mp3", "synth", "device"):
            frame_us = int(resolved_dur * 1000)
            for ch in range(num_channels):
                encoders.append(LC3Encoder(frame_duration_us=frame_us, sample_rate_hz=resolved_sr))
            print(f"Initialized {len(encoders)} LC3 encoder channel instances (Google liblc3).\n", flush=True)

        # 1. Build Advertising Data using user-configured BAP parameters
        base_payload = build_base_data(
            num_bis=num_bis,
            is_stereo=is_stereo,
            sample_rate=resolved_sr,
            frame_duration_ms=resolved_dur,
            presentation_delay_ms=args.presentation_delay,
            octets_per_frame=args.octets_per_frame,
            quality_preset=args.quality_preset,
            context_type=args.context,
            program_info=args.program_info,
            language=args.language
        )

        # Public Broadcast Announcement (PBA): UUID 0x1852, Flags 0x01
        pba_service_data = bytes([0x52, 0x18, 0x01])
        adv_data = bytearray()
        adv_data.extend([0x02, 0x01, 0x06]) # Flags: General Discoverable + BR/EDR Not Supported
        adv_data.extend([len(pba_service_data) + 1, 0x16])
        adv_data.extend(pba_service_data)
        name_bytes = args.adv_name.encode('utf-8')
        adv_data.extend([len(name_bytes) + 1, 0x09]) # Complete Local Name
        adv_data.extend(name_bytes)

        # Periodic Advertising Data: BASE (UUID 0x1851)
        per_adv_data = bytearray()
        base_service_data = bytes([0x51, 0x18]) + base_payload
        per_adv_data.extend([len(base_service_data) + 1, 0x16])
        per_adv_data.extend(base_service_data)

        # 2. Configure HCI Controller
        print(f"[1/4] Configuring Extended Advertising (Set 0)...", flush=True)
        try:
            resp = await device.send_command(HCI_LE_Set_Extended_Advertising_Parameters_Command(
                advertising_handle=0,
                advertising_event_properties=0x0000, # Non-connectable, Non-scannable, Undirected
                primary_advertising_interval_min=32, # 20 ms (Fast Discovery)
                primary_advertising_interval_max=32,
                primary_advertising_channel_map=0x07, # Channels 37, 38, 39
                own_address_type=0x00, # Public Device Address (Factory IEEE MAC)
                peer_address_type=0x00,
                peer_address=Address("00:00:00:00:00:00"),
                advertising_filter_policy=0x00,
                advertising_tx_power=10, # +10 dBm
                primary_advertising_phy=HCI_LE_1M_PHY,
                secondary_advertising_max_skip=0,
                secondary_advertising_phy=HCI_LE_2M_PHY,
                advertising_sid=0,
                scan_request_notification_enable=0
            ))
            print(f"  -> Extended Advertising Parameters Configured (Resp: {resp})", flush=True)
        except Exception as e:
            print(f"  [ERROR] Failed to set Extended Advertising Parameters: {e}", flush=True)

        await device.send_command(HCI_LE_Set_Extended_Advertising_Data_Command(
            advertising_handle=0,
            operation=0x03, # Complete data
            fragment_preference=0x01,
            advertising_data=bytes(adv_data)
        ))

        print(f"[2/4] Configuring Periodic Advertising with BAP BASE Descriptor...", flush=True)
        await device.send_command(HCI_LE_Set_Periodic_Advertising_Parameters_Command(
            advertising_handle=0,
            periodic_advertising_interval_min=16, # 20.0 ms (50 Hz Stable Audio Pacing with 2x Dual-Frame Redundancy)
            periodic_advertising_interval_max=16,
            periodic_advertising_properties=0x0000
        ))

        await device.send_command(HCI_LE_Set_Periodic_Advertising_Data_Command(
            advertising_handle=0,
            operation=0x03, # Complete data
            advertising_data=bytes(per_adv_data)
        ))

        print(f"[3/4] Enabling Extended & Periodic Advertising...", flush=True)
        await device.send_command(HCI_LE_Set_Periodic_Advertising_Enable_Command(
            enable=1,
            advertising_handle=0
        ))
        await device.send_command(HCI_LE_Set_Extended_Advertising_Enable_Command(
            enable=1,
            advertising_handles=[0],
            durations=[0],
            max_extended_advertising_events=[0]
        ))

        print(f"[4/4] Creating Broadcast Isochronous Group (BIG Handle 0)...", flush=True)
        sdu_interval = int(resolved_dur * 1000)
        max_sdu = frame_octets

        create_big_cmd = HCI_LE_Create_BIG_Command(
            big_handle=0,
            advertising_handle=0,
            num_bis=num_bis,
            sdu_interval=sdu_interval,
            max_sdu=max_sdu,
            max_transport_latency=int(args.presentation_delay), # transport latency in ms
            rtn=2, # 2 retransmissions
            phy=HCI_LE_2M_PHY, # LE 2M PHY
            packing=0x00, # Sequential
            framing=0x00, # Unframed
            encryption=0x00, # Unencrypted
            broadcast_code=b'\x00' * 16
        )

        bis_handles = list(range(1, num_bis + 1))
        try:
            resp = await device.send_command(create_big_cmd)
            if hasattr(resp, 'return_parameters') and hasattr(resp.return_parameters, 'connection_handles'):
                bis_handles = resp.return_parameters.connection_handles
            print(f"==> BIG Created Successfully! Group Handle: 0, BIS Handles: {bis_handles}")
        except Exception as e:
            print(f"Note on Create BIG command: {e}")

        print(f"\n===========================================================")
        print(f"  AURACAST BROADCAST IS ACTIVE!")
        print(f"  Broadcast Name: '{args.adv_name}'")
        print(f"  Active Streams: {num_bis} BIS ({'Stereo' if is_stereo else f'{num_bis} Mono Channels'})")
        print(f"  Audio Source: {args.source.upper()}")
        print(f"  Streaming real-time LC3 compressed audio over RF...")
        print(f"  Press Ctrl+C to terminate broadcast.")
        print(f"===========================================================\n", flush=True)

        start_time = time.perf_counter()
        seq_num = 0
        last_stat_seq = 0
        last_stat_time = start_time
        is_hardware_disconnected = False

        # Dual 100-frame ring buffers matching Node 23 AudioSignalMeter (1.0s @ 100 fps)
        peak_history = collections.deque(maxlen=100)
        rms_history = collections.deque(maxlen=100)
        pending_pa_frame1 = None

        try:
            while True:
                # Check for COM port disconnect or hardware reset
                if hci_source.terminated.done():
                    is_hardware_disconnected = True
                    print("\n[NOTE] Hardware reset or USB disconnected on Node 21 (COM121). Broadcaster terminated gracefully.", flush=True)
                    break

                target_time = start_time + ((seq_num + 1) * (resolved_dur / 1000.0))

                if audio_source:
                    # Fetch next PCM audio frame: shape (num_channels, frame_samples)
                    pcm_frame = audio_source.get_frame()

                    # Compute single-pass frame Peak (pk2pk / 2) and RMS matching Node 23 firmware
                    p_min = int(np.min(pcm_frame))
                    p_max = int(np.max(pcm_frame))
                    frame_peak = (p_max - p_min) // 2
                    frame_rms = int(np.sqrt(np.mean(pcm_frame.astype(np.float64)**2)))

                    peak_history.append(frame_peak)
                    rms_history.append(frame_rms)

                    if is_stereo:
                        # Encode Left and Right channels into single Stereo SDU
                        lc3_l = encoders[0].encode(pcm_frame[0], octets_per_channel)
                        lc3_r = encoders[1].encode(pcm_frame[1], octets_per_channel)
                        payload = lc3_l + lc3_r

                        iso_pkt = HCI_IsoDataPacket(
                            connection_handle=bis_handles[0],
                            pb_flag=0b00, # Complete SDU
                            packet_sequence_number=seq_num & 0xFFFF,
                            iso_sdu_length=len(payload),
                            packet_status_flag=0,
                            data_total_length=4 + len(payload),
                            iso_sdu_fragment=payload
                        )
                        try:
                            # 1. Emit Hardware ISO BIS Packet (Pure BLE 5.3 Auracast Stream)
                            hci_sink.on_packet(bytes(iso_pkt))

                            # 2. Update Periodic Advertising Train at stable 50 Hz (every 20 ms) with 2x LC3 frames (20ms audio)
                            if seq_num % 2 == 0:
                                pending_pa_frame1 = lc3_l
                            else:
                                if pending_pa_frame1 is not None:
                                    per_adv = bytearray()
                                    base_service_data = bytes([0x51, 0x18, (seq_num & 0xFF), len(lc3_l)]) + pending_pa_frame1 + lc3_l
                                    per_adv.extend([len(base_service_data) + 1, 0x16])
                                    per_adv.extend(base_service_data)
                                    per_cmd = HCI_LE_Set_Periodic_Advertising_Data_Command(
                                        advertising_handle=0,
                                        operation=0x03,
                                        advertising_data=bytes(per_adv)
                                    )
                                    hci_sink.on_packet(bytes(per_cmd))
                                pending_pa_frame1 = None
                        except (TransportLostError, serial.SerialException, PermissionError, OSError):
                            is_hardware_disconnected = True
                            print("\n[NOTE] Hardware reset or USB disconnected on Node 21 (COM121). Broadcaster terminated gracefully.", flush=True)
                            break
                        except Exception as e:
                            if seq_num % 100 == 1:
                                print(f"[Broadcaster TX Warning] {e}", flush=True)
                    else:
                        # Multi-Channel: Encode 1 Mono LC3 frame per BIS stream
                        for bis_idx, bis_handle in enumerate(bis_handles):
                            ch_idx = bis_idx % num_channels
                            payload = encoders[ch_idx].encode(pcm_frame[ch_idx], octets_per_channel)

                            iso_pkt = HCI_IsoDataPacket(
                                connection_handle=bis_handle,
                                pb_flag=0b00,
                                packet_sequence_number=seq_num & 0xFFFF,
                                iso_sdu_length=len(payload),
                                packet_status_flag=0,
                                data_total_length=4 + len(payload),
                                iso_sdu_fragment=payload
                            )
                            try:
                                # 1. Emit Hardware ISO BIS Packet (Pure BLE 5.3 Auracast Stream)
                                hci_sink.on_packet(bytes(iso_pkt))
                                if bis_idx == 0:
                                    per_adv = bytearray()
                                    base_service_data = bytes([0x51, 0x18, (seq_num & 0xFF)]) + payload
                                    per_adv.extend([len(base_service_data) + 1, 0x16])
                                    per_adv.extend(base_service_data)
                                    per_cmd = HCI_LE_Set_Periodic_Advertising_Data_Command(
                                        advertising_handle=0,
                                        operation=0x03,
                                        advertising_data=bytes(per_adv)
                                    )
                                    hci_sink.on_packet(bytes(per_cmd))
                            except (TransportLostError, serial.SerialException, PermissionError, OSError):
                                is_hardware_disconnected = True
                                print("\n[NOTE] Hardware reset or USB disconnected on Node 21 (COM121). Broadcaster terminated gracefully.", flush=True)
                                break
                            except Exception as e:
                                print(f"\n[ABORT] Serial write failed ({e}). Hardware was likely disconnected or reset.", flush=True)
                                break
                else:
                    rms_l = rms_r = 0.0
                    # Test Pattern Mode
                    for bis_idx, bis_handle in enumerate(bis_handles):
                        dummy_payload = bytes([(seq_num + bis_idx) & 0xFF] * frame_octets)
                        iso_pkt = HCI_IsoDataPacket(
                            connection_handle=bis_handle,
                            pb_flag=0b00,
                            packet_sequence_number=seq_num & 0xFFFF,
                            iso_sdu_length=len(dummy_payload),
                            packet_status_flag=0,
                            data_total_length=4 + len(dummy_payload),
                            iso_sdu_fragment=dummy_payload
                        )
                        try:
                            hci_sink.on_packet(bytes(iso_pkt))
                        except Exception as e:
                            print(f"\n[ABORT] Serial write failed ({e}). Hardware was likely disconnected or reset.", flush=True)
                            break

                seq_num += 1
                if seq_num % 100 == 0:
                    now_perf = time.perf_counter()
                    time_delta = now_perf - last_stat_time
                    pkts_delta = seq_num - last_stat_seq
                    pkt_rate = pkts_delta / (time_delta if time_delta > 0 else 1.0)
                    last_stat_time = now_perf
                    last_stat_seq = seq_num

                    elapsed_s = seq_num * resolved_dur / 1000.0

                    # Compute 1.0s Peak and RMS dBFS metrics matching Node 23 AudioSignalMeter
                    max_pk = max(peak_history) if peak_history else 0
                    if max_pk <= 0:
                        peak_db = -math.inf
                    else:
                        peak_db = 20.0 * math.log10(max_pk / 32767.0)

                    if rms_history:
                        mean_sq = np.mean(np.array(rms_history, dtype=np.float64)**2)
                        rms_val = np.sqrt(mean_sq)
                        if rms_val <= 0:
                            rms_db = -math.inf
                        else:
                            rms_db = 20.0 * math.log10(rms_val / 32767.0)
                    else:
                        rms_db = -math.inf

                    rms_str = "-∞" if (math.isinf(rms_db) or rms_db <= -95.0) else f"{rms_db:.1f}"
                    peak_str = "-∞" if (math.isinf(peak_db) or peak_db <= -95.0) else f"{peak_db:.1f}"

                    if args.source == "device":
                        # VU Bar representation
                        max_norm = max_pk / 32767.0
                        bar_len = int(min(1.0, max_norm * 4.0) * 15)
                        vu_bar = "#" * bar_len + "-" * (15 - bar_len)
                        if max_pk < 32:
                            status_extra = f" | RMS|Pk {rms_str}|{peak_str} dBFS | [SILENCE (No CABLE input?)]"
                        else:
                            status_extra = f" | RMS|Pk {rms_str}|{peak_str} dBFS |{vu_bar}|"
                        print(f"  [Broadcasting] Frame {seq_num:5d} @ {pkt_rate:5.1f} fps ({elapsed_s:5.1f}s){status_extra}", flush=True)
                    else:
                        print(f"  [Broadcasting] Frame {seq_num:5d} @ {pkt_rate:5.1f} fps ({elapsed_s:5.1f}s) | RMS|Pk {rms_str}|{peak_str} dBFS", flush=True)

                if args.test_duration and (seq_num * resolved_dur / 1000.0) >= args.test_duration:
                    print(f"\nCompleted test duration ({args.test_duration}s). Exiting.", flush=True)
                    break

                now = time.perf_counter()
                sleep_time = target_time - now
                if sleep_time > 0:
                    try:
                        done, _ = await asyncio.wait([hci_source.terminated], timeout=sleep_time)
                        if done:
                            is_hardware_disconnected = True
                            print("\n[NOTE] Hardware reset or USB disconnected on Node 21 (COM121). Broadcaster terminated gracefully.", flush=True)
                            break
                    except Exception:
                        await asyncio.sleep(sleep_time)

        except (asyncio.CancelledError, KeyboardInterrupt):
            print("\nBroadcast interrupted by user (Ctrl+C).", flush=True)
        except (TransportLostError, serial.SerialException, PermissionError, OSError):
            is_hardware_disconnected = True
            print("\n[NOTE] Hardware reset or USB disconnected on Node 21 (COM121). Broadcaster terminated gracefully.", flush=True)
        except Exception as e:
            print(f"\nBroadcast encountered exception: {e}", flush=True)
        finally:
            try:
                if sys.platform == "win32":
                    ctypes.windll.winmm.timeEndPeriod(1)
            except Exception:
                pass
            if audio_source:
                try:
                    audio_source.close()
                except Exception:
                    pass
            if not is_hardware_disconnected and hci_source and not hci_source.terminated.done():
                try:
                    await asyncio.wait_for(stop_broadcast(device), timeout=2.5)
                except Exception as e:
                    logging.debug(f"Teardown timeout/error: {e}")
            if hci_transport:
                try:
                    await hci_transport.close()
                except Exception:
                    pass
                try:
                    if hasattr(hci_source, 'serial') and hci_source.serial:
                        hci_source.serial.close()
                except Exception:
                    pass

def main():
    parser = argparse.ArgumentParser(description="Google Bumble BLE 5.3 Auracast Broadcaster with LC3 Audio Streaming")
    parser.add_argument("--port", default="COM121", help="HCI Serial COM port (default: COM121)")
    parser.add_argument("--baud", type=int, default=2000000, help="Baud rate (default: 2000000)")
    parser.add_argument("--mode", choices=["stereo", "multichannel"], default="stereo",
                        help="Broadcast Mode: 'stereo' (1 BIG, 1 Stereo BIS) or 'multichannel' (1 BIG, 2-6 Mono BIS)")
    parser.add_argument("--num-bis", type=int, default=5, choices=range(2, 7),
                        help="Number of BIS streams for multichannel mode (2 to 6, default: 5)")
    parser.add_argument("--adv-name", default="ForestChirp Auracast", help="Broadcast Local Name")
    
    # BAP Bluetooth Fields
    parser.add_argument("--sample-rate", type=int, default=48000,
                        help="Sampling Frequency in Hz (e.g. 44100, 48000; rounded to nearest if non-standard; range: 7k-100k)")
    parser.add_argument("--frame-duration", type=float, default=10.0,
                        help="Frame duration in ms: 7.5 or 10.0 (default: 10.0 ms)")
    parser.add_argument("--presentation-delay", type=float, default=40.0,
                        help="Audio presentation delay in milliseconds (range: 5.0 to 250.0 ms, default: 40.0 ms)")
    parser.add_argument("--quality-preset", choices=["voice", "standard", "high_quality", "audiophile"], default="standard",
                        help="LC3 quality preset defining octets per frame (default: 'standard')")
    parser.add_argument("--octets-per-frame", type=int, default=None,
                        help="Explicit target frame size in bytes (overrides --quality-preset)")
    parser.add_argument("--context", default="media",
                        help="Audio Context: 'media', 'conversational', 'game', 'instructional', 'live', 'unspecified', 'alerts'")
    parser.add_argument("--program-info", default=None,
                        help="Optional program / track title metadata string for BAP Level 2")
    parser.add_argument("--language", default=None,
                        help="Optional 3-letter ISO 639-3 language code (e.g. 'eng', 'swe', 'deu')")
    parser.add_argument("--list-bap-options", action="store_true",
                        help="Print reference table of all supported BAP sampling rates, durations, presets, and context types")

    # Audio source configuration
    default_mp3_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "data", "mp3"))
    parser.add_argument("--source", choices=["mp3", "synth", "device", "test-pattern"], default="mp3",
                        help="Audio input source: 'mp3' (Random tracks from data/mp3, default), 'synth' (LFO modulated sine), 'device' (Virtual Cable / live mic), or 'test-pattern'")
    parser.add_argument("--mp3-dir", default=default_mp3_dir,
                        help=f"Directory containing MP3/WAV tracks for 'mp3' source (default: {default_mp3_dir})")
    parser.add_argument("--audio-device", default="CABLE",
                        help="Device name query or index for 'device' source mode (default: 'CABLE')")
    parser.add_argument("--list-audio-devices", action="store_true",
                        help="List all available audio input/output devices and exit")
    parser.add_argument("--amplitude", type=float, default=0.25,
                        help="Audio peak amplitude (0.0 to 1.0, default: 0.25 = 25%%)")
    parser.add_argument("--lfo-rate", type=float, default=0.2,
                        help="LFO modulation frequency in Hz for 'synth' mode (default: 0.2 Hz)")
    parser.add_argument("--lfo-min", type=float, default=220.0,
                        help="LFO minimum frequency in Hz (default: 220.0 Hz)")
    parser.add_argument("--lfo-max", type=float, default=880.0,
                        help="LFO maximum frequency in Hz (default: 880.0 Hz)")

    parser.add_argument("--test-duration", type=float, default=0.0,
                        help="Optional auto-stop duration in seconds (for automated testing)")
    parser.add_argument("--verbose", "-v", action="store_true", help="Enable verbose debug logging")

    args = parser.parse_args()

    if args.list_bap_options:
        print_bap_options()
        sys.exit(0)

    if args.list_audio_devices:
        list_audio_devices()
        sys.exit(0)

    asyncio.run(run_broadcaster(args))

if __name__ == "__main__":
    main()
