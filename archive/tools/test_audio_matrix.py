import serial
import time
import re
import sys

SAMPLE_RATES = [48000, 32000, 24000, 16000, 8000]
DURATIONS = [10.0, 7.5]
MODES = ["mono", "stereo"]

def parse_table_row(line):
    # Matches a row like:
    # | 27  60  240 | BROAD |  -   01 24M |  LC3  -33.4 -30.3  48.0   10  4.17  4.28  -26   -   197   -    -    -     -  |  14329   14329 |
    # or SINK:
    # | 26  36  160 | STRM  | -24  01  1M |  LC3  -33.1 -30.3  48.0   10  1.98  2.06  -26  +3    99    0    0     0    0 | 651984    9714 |
    if not line.startswith("|") or line.startswith("+="):
        return None
    parts = [p.strip() for p in line.split("|")]
    if len(parts) < 6:
        return None
    
    try:
        # Part 1: CPU %  Temp  MHz
        cpu_parts = parts[1].split()
        cpu_pct = int(cpu_parts[0]) if len(cpu_parts) > 0 else 0
        temp_c = int(cpu_parts[1]) if len(cpu_parts) > 1 else 0
        
        # Part 2: State
        state = parts[2]
        
        # Part 3: Wifi RSSI Ch PHY
        wifi_str = parts[3]
        
        # Part 4: Audio block
        # Enc RMS Pk SR PD Avg Pk SW HW PKTS PLC DMA FIFO PREV
        audio_parts = parts[4].split()
        
        # Check if we have audio metrics
        enc = audio_parts[0] if len(audio_parts) > 0 else "-"
        sr = audio_parts[3] if len(audio_parts) > 3 else "-"
        pd = audio_parts[4] if len(audio_parts) > 4 else "-"
        codec_avg = audio_parts[5] if len(audio_parts) > 5 else "-"
        codec_pk = audio_parts[6] if len(audio_parts) > 6 else "-"
        
        # SINK columns at the end of audio block:
        # SINK: pkts(9), plc(10), dma(11), fifo(12), prev(13)
        plc = int(audio_parts[10]) if len(audio_parts) > 10 and audio_parts[10] != '-' else 0
        dma_udr = int(audio_parts[11]) if len(audio_parts) > 11 and audio_parts[11] != '-' else 0
        fifo_udr = int(audio_parts[12]) if len(audio_parts) > 12 and audio_parts[12] != '-' else 0
        prev_rec = int(audio_parts[13]) if len(audio_parts) > 13 and audio_parts[13] != '-' else 0

        return {
            "cpu_pct": cpu_pct,
            "temp_c": temp_c,
            "state": state,
            "enc": enc,
            "sr": sr,
            "pd": pd,
            "codec_avg": codec_avg,
            "codec_pk": codec_pk,
            "plc": plc,
            "dma_udr": dma_udr,
            "fifo_udr": fifo_udr,
            "prev_rec": prev_rec,
        }
    except Exception as e:
        return None

def run_test_case(ser_src, ser_snk, sr_hz, dur_ms, mode, duration_sec=10.0):
    print(f"\n=======================================================")
    print(f" TEST CASE: {sr_hz} Hz | {dur_ms} ms | {mode.upper()}")
    print(f"=======================================================")

    # 1. Configure SOURCE
    ser_src.write(f"{mode}\r\n".encode())
    time.sleep(0.1)
    ser_src.write(f"dur {dur_ms}\r\n".encode())
    time.sleep(0.1)
    ser_src.write(f"sr {sr_hz}\r\n".encode())
    time.sleep(0.1)
    ser_src.write(b"start\r\n")
    ser_src.flush()

    # 2. Reset counters on SINK
    time.sleep(0.2)
    ser_snk.write(b"reset\r\n")
    ser_snk.flush()

    # 3. Wait 2 seconds for settling
    time.sleep(2.0)
    ser_src.reset_input_buffer()
    ser_snk.reset_input_buffer()

    src_samples = []
    snk_samples = []

    t_end = time.time() + duration_sec
    while time.time() < t_end:
        # Read SOURCE
        while ser_src.in_waiting > 0:
            l = ser_src.readline().decode('utf-8', errors='replace').rstrip()
            parsed = parse_table_row(l)
            if parsed and parsed["state"] == "BROAD":
                src_samples.append(parsed)
        
        # Read SINK
        while ser_snk.in_waiting > 0:
            l = ser_snk.readline().decode('utf-8', errors='replace').rstrip()
            parsed = parse_table_row(l)
            if parsed:
                snk_samples.append(parsed)

        time.sleep(0.05)

    # Summarize SOURCE stats
    if src_samples:
        src_cpus = [s["cpu_pct"] for s in src_samples]
        src_codecs = [float(s["codec_avg"]) for s in src_samples if s["codec_avg"] != "-"]
        src_cpu_avg = sum(src_cpus) / len(src_cpus)
        src_codec_avg = sum(src_codecs) / len(src_codecs) if src_codecs else 0.0
    else:
        src_cpu_avg = 0
        src_codec_avg = 0.0

    # Summarize SINK stats
    if snk_samples:
        # Check states
        states = [s["state"] for s in snk_samples]
        is_strm_only = all(st == "STRM" for st in states)
        snk_cpus = [s["cpu_pct"] for s in snk_samples if s["state"] == "STRM"]
        snk_codecs = [float(s["codec_avg"]) for s in snk_samples if s["state"] == "STRM" and s["codec_avg"] != "-"]
        snk_cpu_avg = sum(snk_cpus) / len(snk_cpus) if snk_cpus else 0
        snk_codec_avg = sum(snk_codecs) / len(snk_codecs) if snk_codecs else 0.0

        last_snk = snk_samples[-1]
        sr_rep = last_snk["sr"]
        pd_rep = last_snk["pd"]
        dma_udr_max = max(s["dma_udr"] for s in snk_samples)
        fifo_udr_max = max(s["fifo_udr"] for s in snk_samples)
        prev_rec_max = max(s["prev_rec"] for s in snk_samples)
        plc_max = max(s["plc"] for s in snk_samples)
    else:
        is_strm_only = False
        sr_rep = "-"
        pd_rep = "-"
        snk_cpu_avg = 0
        snk_codec_avg = 0.0
        dma_udr_max = 0
        fifo_udr_max = 0
        prev_rec_max = 0
        plc_max = 0

    result = {
        "sr_hz": sr_hz,
        "dur_ms": dur_ms,
        "mode": mode,
        "src_cpu_pct": round(src_cpu_avg, 1),
        "src_codec_ms": round(src_codec_avg, 2),
        "snk_sr": sr_rep,
        "snk_pd": pd_rep,
        "snk_cpu_pct": round(snk_cpu_avg, 1),
        "snk_codec_ms": round(snk_codec_avg, 2),
        "snk_dma_udr": dma_udr_max,
        "snk_fifo_udr": fifo_udr_max,
        "snk_prev_rec": prev_rec_max,
        "snk_plc": plc_max,
        "strm_stable": is_strm_only
    }
    print(f" -> SRC: CPU {result['src_cpu_pct']}%, Codec {result['src_codec_ms']} ms | SINK: SR {result['snk_sr']}k, PD {result['snk_pd']}ms, CPU {result['snk_cpu_pct']}%, Codec {result['snk_codec_ms']} ms, DMA_UDR {result['snk_dma_udr']}, PLC {result['snk_plc']}, Stable: {result['strm_stable']}")
    return result

def main():
    print("Opening COM16 (SOURCE) and COM23 (SINK)...")
    try:
        ser_src = serial.Serial("COM16", 115200, timeout=0.2)
        ser_snk = serial.Serial("COM23", 115200, timeout=0.2)
    except Exception as e:
        print(f"Error opening serial ports: {e}")
        return

    time.sleep(1.0)
    ser_src.reset_input_buffer()
    ser_snk.reset_input_buffer()

    results = []

    for mode in MODES:
        for dur in DURATIONS:
            for sr in SAMPLE_RATES:
                res = run_test_case(ser_src, ser_snk, sr, dur, mode, duration_sec=10.0)
                results.append(res)

    ser_src.write(b"stop\r\n")
    ser_src.flush()
    ser_src.close()
    ser_snk.close()

    print("\n\n" + "="*80)
    print("                      FULL AUDIO SETTINGS MATRIX RESULTS")
    print("="*80)
    header = "| Sample Rate | Duration | Mode | SRC CPU % | SRC Codec (ms) | SNK SR (kHz) | SNK PD (ms) | SNK CPU % | SNK Codec (ms) | DMA UDR | FIFO UDR | PREV REC | PLC 1/s | Status |"
    sep = "| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |"
    print(header)
    print(sep)
    for r in results:
        status_str = "PASS" if (r["strm_stable"] and r["snk_dma_udr"] == 0) else "FAIL"
        line = f"| {r['sr_hz']} Hz | {r['dur_ms']} ms | {r['mode'].upper()} | {r['src_cpu_pct']}% | {r['src_codec_ms']} ms | {r['snk_sr']} | {r['snk_pd']} | {r['snk_cpu_pct']}% | {r['snk_codec_ms']} ms | {r['snk_dma_udr']} | {r['snk_fifo_udr']} | {r['snk_prev_rec']} | {r['snk_plc']} | {status_str} |"
        print(line)

if __name__ == "__main__":
    main()
