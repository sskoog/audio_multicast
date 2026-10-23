---
name: lc3-codec-benchmark
description: >-
  On-device benchmarking and cross-SoC performance analysis of the Google LC3 audio codec across ESP32-S3 (FPU), ESP32-C6 (RISC-V), and ESP32-WROOM-32 (Xtensa dual-core).
---

# LC3 Codec Benchmark Skill

Procedures for executing on-device execution time benchmarks, measuring cycle counts, and generating comparative matplotlib plots across Espressif SoCs.

---

## Benchmarking Workflow

1. **Flash Benchmark Firmware**:
   ```powershell
   # Flash to ESP32-S3 on COM16
   python tools\run_s3_benchmark.py --port COM16
   
   # Flash to ESP32-C6 on COM23
   python tools\run_c6_benchmark.py --port COM23
   
   # Flash to ESP32-WROOM-32 on COM21
   python tools\run_wroom32_benchmark.py --port COM21
   ```

2. **Capture Benchmark Outputs**:
   Run serial captures to record execution time (microseconds) and CPU load across:
   - Floating-point vs Fixed-point implementations
   - Frame durations (7.5 ms vs 10.0 ms)
   - Sample rates (8k, 16k, 24k, 32k, 48k)
   - Target octet bitrates (20 to 155 octets/frame)

3. **Plot Generation**:
   ```powershell
   python tools\generate_all_nodes_plots.py
   ```
