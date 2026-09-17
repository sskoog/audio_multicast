---
name: parallel-compile-and-flash
description: >-
  Procedures, rules, and automated tooling for multi-core parallel firmware compilation (ESP32-S3 + ESP32-C6 concurrently with CPU thread gating: max 8 threads single-build, 6 threads per dual-target build) and simultaneous multi-node flashing across USB Virtual COM ports.
---

# Parallel Firmware Compilation & Multi-Node Flashing Skill

This skill documents the architecture, thread-allocation limits, isolation constraints, and automated scripts for compiling multi-architecture ESP-IDF firmware in parallel and concurrently flashing all cluster nodes over multiple USB COM ports in Windows PowerShell.

---

## 1. Hardware & Host Threading Architecture

### Host Machine Constraints
- **CPU Topology**: 12 logical threads (2 Performance Cores + 8 Efficiency Cores).
- **Single-Target Build Limit**: Max **8 ninja worker threads** (`ninja -j 8`).
- **Dual-Target Parallel Build Limit**: Exactly **6 ninja worker threads per target** (`ninja -j 6` for S3, `ninja -j 6` for C6; total 12 threads combined).

### Target Firmware Architecture Matrix
- **ESP32-S3 (Xtensa Dual-Core + FPU)**: `build_s3/` with `sdkconfig.s3` (Node 16 SOURCE).
- **ESP32-C6 (RISC-V Single-Core)**: `build_c6/` with `sdkconfig.c6` (Nodes 20, 21, 23, 24, 25, 26 SINKs).
  *(All C6 SINK nodes dynamically detect their role/channel based on factory MAC address, so building `build_c6` once produces the binary for all C6 nodes).*

---

## 2. Strict Build Isolation Rules

When compiling S3 and C6 targets simultaneously:
1. **Dedicated Build Folders**: S3 must use `build_s3/` and C6 must use `build_c6/`. Never mix build artifacts or toolchains.
2. **Dedicated `sdkconfig` Files**: `Copy-Item sdkconfig.s3 sdkconfig` and `Copy-Item sdkconfig.c6 sdkconfig` per isolated subprocess context.
3. **No Cross-Target Contention**: Multi-threaded Python/PowerShell job runners invoke independent ESP-IDF subshells.

---

## 3. Automated Parallel Build & Flash Tool

The repository provides an automated coordinator:
- Python script: `c:\Git_ble_audio\.agents\skills\parallel-compile-and-flash\scripts\parallel_build_and_flash.py`
- PowerShell wrapper: `apps\audio_VSAF_broadcast2\tools\parallel_build_and_flash.ps1`

### CLI Usage & Workflow

#### A. Full Cluster Parallel Build & Flash (Default: All Targets + All Detected Nodes)
```powershell
powershell -ExecutionPolicy Bypass -File apps\audio_VSAF_broadcast2\tools\parallel_build_and_flash.ps1
```

#### B. Parallel Build Only (No Flashing)
```powershell
powershell -ExecutionPolicy Bypass -File apps\audio_VSAF_broadcast2\tools\parallel_build_and_flash.ps1 -BuildOnly
```

#### C. Parallel Flash Only (Fast Re-Flash of Existing Binaries)
```powershell
powershell -ExecutionPolicy Bypass -File apps\audio_VSAF_broadcast2\tools\parallel_build_and_flash.ps1 -FlashOnly
```

#### D. Selective Nodes Flashing
```powershell
powershell -ExecutionPolicy Bypass -File apps\audio_VSAF_broadcast2\tools\parallel_build_and_flash.ps1 -Nodes "16,23,24"
```

---

## 4. Multi-Port Parallel Flash Mechanics

Because each embedded device is wired to an independent physical USB CDC / Serial JTAG interface (`COM16`, `COM23`, `COM24`), the host USB controller transmits serial data concurrently across all USB endpoints:

```text
                  +---------------------------------------+
                  | Parallel Build & Flash Coordinator    |
                  +-------------------+-------------------+
                                      |
         +----------------------------+----------------------------+
         |                            |                            |
  [Flash COM16]                 [Flash COM23]                [Flash COM24]
  Node 16 (ESP32-S3)            Node 23 (ESP32-C6)           Node 24 (ESP32-C6)
  Thread 1                      Thread 2                     Thread 3
         |                            |                            |
  (3.5s upload + WDT reset)     (3.5s upload + RTS reset)    (3.5s upload + RTS reset)
         |                            |                            |
         +----------------------------+----------------------------+
                                      |
                        Total Elapsed Time: ~3.5 s
```

### Reset Handlers
1. **Node 16 (ESP32-S3 SOURCE)**: Uses the internal RTC Watchdog Timer (WDT) register sequence via `s3_flash_and_reset.py` to trigger a true hardware reboot without manual button pressing.
2. **Nodes 20..26 (ESP32-C6 SINKs)**: Reset via standard hardware `RTS` pin toggle (`--after hard-reset`).
