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
2. **Dedicated `sdkconfig` Files**: each build passes `-D SDKCONFIG=sdkconfig.<s3|c6>` to `idf.py` (no shared `sdkconfig` copy).
3. **No Cross-Target Contention**: Multi-threaded Python/PowerShell job runners invoke independent ESP-IDF subshells. Per-target dependency locks (`dependencies.lock.<target>`) and the vendored `components/esp_tinyusb` keep the two builds from touching shared files.

---

## 3. Automated Parallel Build & Flash Tool

The repository provides an automated coordinator:
- Python script: `.claude\skills\parallel-compile-and-flash\scripts\parallel_build_and_flash.py` (defaults `--app-dir` to `apps\audio_VSAF`)
- PowerShell wrapper: `apps\audio_VSAF\tools\parallel_build_and_flash.ps1`

### CLI Usage & Workflow

#### A. Full Cluster Parallel Build & Flash (Default: All Targets + All Detected Nodes)
```powershell
powershell -ExecutionPolicy Bypass -File apps\audio_VSAF\tools\parallel_build_and_flash.ps1
```

#### B. Parallel Build Only (No Flashing)
```powershell
powershell -ExecutionPolicy Bypass -File apps\audio_VSAF\tools\parallel_build_and_flash.ps1 -BuildOnly
```

#### C. Parallel Flash Only (Fast Re-Flash of Existing Binaries)
```powershell
powershell -ExecutionPolicy Bypass -File apps\audio_VSAF\tools\parallel_build_and_flash.ps1 -FlashOnly
```

#### D. Selective Nodes Flashing
```powershell
powershell -ExecutionPolicy Bypass -File apps\audio_VSAF\tools\parallel_build_and_flash.ps1 -Nodes "16,23,24"
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
2. **SINKs (S3 Nodes 1-5, C6 Nodes 20-26)**: plain `esptool` on their USB-Serial/JTAG port, reset via `--after hard-reset`.

---

## 5. Field Notes (measured 2026-10-24, ESP-IDF v6.1, 5-node cluster)

**Timing.** Incremental parallel build S3 + C6: ~25-35 s. Parallel flash of 4 SINKs: 7 s (C6) / 11 s (S3) per node, 11 s wall clock. Build + flash 4 SINKs end to end: ~47 s. Use `-Nodes "4,5,23,24"` to leave Node 16 alone when only SINK code changed (it is the slow, fragile one).

**Success detection.** A build only counts as passed if `idf.py` exits 0 *and* prints `Project build complete`. The old check (`.bin` exists) once reported PASS for a failed S3 build because a stale `.bin` was lying around. Full per-target output: `build_s3/parallel_build.log`, `build_c6/parallel_build.log`.

**Node identity is the MAC, never the VID:PID.** Every S3/C6 SINK and Node 16's ROM bootloader enumerate as `303A:1001`. The USB serial number is the chip MAC (e.g. `E0:72:A1:D8:4C:D0` = Node 16), and `s3_flash_and_reset.py` now only accepts a bootloader port with Node 16's MAC and aborts if the connected chip's MAC differs (`--any-mac` to override). Before this, a parallel run could in principle have flashed the SOURCE image onto a SINK that happened to be enumerating.

**Node 16 failure modes and recovery.**
1. *Reboot-to-bootloader silently failing after an IDF bump*: the CLI `bootloader` path used `win32file` (pywin32), present in `idf6.0_py3.13_env` but not in `idf6.1_py3.13_env`. Now uses `ctypes` (stdlib) with a 1 s write timeout. Don't rely on packages that aren't in a fresh IDF Python env.
2. *Firmware hung but still enumerated* (COM116 + "USB Speaker" both `OK` in Windows, yet zero console bytes and writes time out): no software path can reset it. Ask the user to **hold B, tap R, release B**; it appears as COM16 (or COM3) and the waiting flasher continues.
3. *Flash dies mid-write on COM3* (ROM USB-OTG port, "Waiting for the chip to reconnect"): the chip re-enumerates as COM16; rerun with `--port COM16`. The USB-Serial/JTAG port is the reliable one.
4. A flasher "waiting" much longer than its ~3 min timeout is itself hung: kill `s3_flash_and_reset` processes and restart it.

**Serial ports.** Opening a SINK port with pyserial's default settings raises DTR/RTS and **resets the chip**; set `dtr=False, rts=False` *before* `open()` (see `soak_test.py:open_port`). COM116 (TinyUSB CDC) needs the usbser.sys error-31 patch on configure.
