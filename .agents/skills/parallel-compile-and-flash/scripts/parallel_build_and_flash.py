#!/usr/bin/env python3
"""
Parallel Build and Multi-Node Flash Coordinator for ESP32 Multi-Architecture Audio Systems.

Key Capabilities:
  1. CPU-Aware Parallel Compilation:
     - Total Host Cores: 12 threads (2 P-cores + 8 E-cores).
     - Single-target build: max 8 ninja worker threads (-j 8).
     - Dual-target parallel build: exactly 6 ninja worker threads per build (-j 6 for S3, -j 6 for C6; total 12 threads).
     - Full config and directory isolation (build_s3 + sdkconfig.s3, build_c6 + sdkconfig.c6).
  2. Concurrent Multi-Node Flashing:
     - Automatically scans and identifies connected ESP32-S3 and ESP32-C6 nodes.
     - Concurrently uploads firmware at 921,600 baud to all target COM ports in parallel threads.
     - Autonomous hardware reset: RTC Watchdog system reset for Node 16 (S3), hardware RTS reset for C6 SINKs.
"""

import os
import sys
import time
import argparse
import subprocess
import threading
from concurrent.futures import ThreadPoolExecutor, as_completed
import serial.tools.list_ports

# Known node registry
KNOWN_NODES = {
    16: {"chip": "esp32s3", "role": "SOURCE", "name": "Node 16 (XIAO ESP32-S3 SOURCE)", "default_port": "COM16", "app_port": "COM116"},
    20: {"chip": "esp32c6", "role": "SINK",   "name": "Node 20 (C6 LCD Console)",        "default_port": "COM20"},
    21: {"chip": "esp32c6", "role": "SINK",   "name": "Node 21 (C6 DevKit / Host)",       "default_port": "COM21", "app_port": "COM121"},
    23: {"chip": "esp32c6", "role": "SINK",   "name": "Node 23 (C6 SINK Left Ch 0)",     "default_port": "COM23"},
    24: {"chip": "esp32c6", "role": "SINK",   "name": "Node 24 (C6 SINK Right Ch 1)",    "default_port": "COM24"},
    25: {"chip": "esp32c6", "role": "SINK",   "name": "Node 25 (C6 Surround Left Ch 3)", "default_port": "COM25"},
    26: {"chip": "esp32c6", "role": "SINK",   "name": "Node 26 (C6 Surround Right Ch 4)","default_port": "COM26"},
}


def print_banner(text, color="\033[96m"):
    reset = "\033[0m"
    print(f"\n{color}{'=' * 78}\n {text}\n{'=' * 78}{reset}")


def kill_lingering_processes():
    """Kill lingering serial monitors or flashing processes holding COM ports."""
    ps_cmd = (
        'Get-CimInstance Win32_Process -Filter "CommandLine LIKE \'%device monitor%\' OR '
        'CommandLine LIKE \'%idf_monitor%\' OR CommandLine LIKE \'%test_audio_matrix%\' OR '
        'CommandLine LIKE \'%pc_audio_streamer%\' OR CommandLine LIKE \'%test_broadcast_cluster%\'" | '
        'ForEach-Object { Stop-Process -Id $_.ProcessId -Force } -ErrorAction SilentlyContinue'
    )
    subprocess.run(["powershell", "-NoProfile", "-Command", ps_cmd], capture_output=True)
    time.sleep(0.5)


def scan_connected_nodes():
    """Scan and match connected COM ports with registered cluster nodes."""
    connected = {}
    com_ports = {p.device.upper(): p for p in serial.tools.list_ports.comports()}

    for node_id, info in KNOWN_NODES.items():
        def_port = info.get("default_port", "").upper()
        app_port = info.get("app_port", "").upper()

        matched_port = None
        if def_port in com_ports:
            matched_port = def_port
        elif app_port and app_port in com_ports:
            matched_port = app_port

        if matched_port:
            connected[node_id] = {
                "node_id": node_id,
                "port": matched_port,
                "chip": info["chip"],
                "role": info["role"],
                "name": info["name"]
            }

    return connected


def compile_target(app_dir, target, threads=6):
    """Compile firmware for a specific target with isolated sdkconfig and build dir."""
    t0 = time.time()
    build_dir = f"build_{'s3' if target == 'esp32s3' else 'c6'}"
    sdkconfig_file = f"sdkconfig.{'s3' if target == 'esp32s3' else 'c6'}"

    print(f"[\033[93mBUILD START\033[0m] Compiling {target.upper()} in {build_dir} using {threads} ninja threads...")

    cmd = (
        f"$env:IDF_TOOLS_PATH = 'C:\\Users\\stefa\\.espressif'; "
        f"if (Test-Path 'C:\\Users\\stefa\\.espressif\\python_env\\idf6.0_py3.13_env') {{ "
        f"  $env:IDF_PYTHON_ENV_PATH = 'C:\\Users\\stefa\\.espressif\\python_env\\idf6.0_py3.13_env' "
        f"}} else {{ "
        f"  $env:IDF_PYTHON_ENV_PATH = 'C:\\Users\\stefa\\.espressif\\python_env\\idf6.0_py3.11_env' "
        f"}}; "
        f"$env:PATH = '$env:IDF_PYTHON_ENV_PATH\\Scripts;' + $env:PATH; "
        f". 'C:\\Users\\stefa\\OneDrive\\Documents\\ESP\\v6.0.2\\esp-idf\\export.ps1'; "
        f"Copy-Item '{sdkconfig_file}' 'sdkconfig' -Force; "
        f"idf.py -B {build_dir} -D IDF_TARGET={target} build -- -j {threads}"
    )

    proc = subprocess.run(
        ["powershell", "-NoProfile", "-Command", cmd],
        cwd=app_dir,
        capture_output=True,
        text=True
    )

    duration = time.time() - t0
    if proc.returncode == 0:
        print(f"[\033[92mBUILD SUCCESS\033[0m] {target.upper()} compiled successfully in {duration:.2f} s ({build_dir})")
        return True, target, duration, ""
    else:
        err_msg = proc.stderr if proc.stderr else proc.stdout
        print(f"[\033[91mBUILD FAILED\033[0m] {target.upper()} failed with code {proc.returncode} in {duration:.2f} s")
        return False, target, duration, err_msg


def flash_single_node(app_dir, node_info, baud=921600):
    """Flash a single ESP32 node via esptool or s3_flash_and_reset.py."""
    node_id = node_info["node_id"]
    port = node_info["port"]
    chip = node_info["chip"]
    role = node_info["role"]
    name = node_info["name"]
    t0 = time.time()

    print(f"[\033[93mFLASH START\033[0m] Node {node_id} ({role}) on {port} [{chip}] at {baud} baud...")

    if chip == "esp32s3":
        # S3 hands-free RTC Watchdog system reset
        s3_script = os.path.join(app_dir, "tools", "s3_flash_and_reset.py")
        cmd = [sys.executable, "-u", s3_script, "--port", port, "--baud", str(baud), "--bin-dir", os.path.join(app_dir, "build_s3")]
        proc = subprocess.run(cmd, cwd=app_dir, capture_output=True, text=True)
    else:
        # C6 standard high-speed flash with hardware reset
        build_dir = os.path.join(app_dir, "build_c6")
        bootloader = os.path.join(build_dir, "bootloader", "bootloader.bin")
        partition = os.path.join(build_dir, "partition_table", "partition-table.bin")
        app_bin = os.path.join(build_dir, "audio_VSAF_broadcast2.bin")
        if not os.path.isfile(app_bin):
            app_bin = os.path.join(build_dir, "audio_ESP_NOW_unicast.bin")

        cmd = [
            sys.executable, "-m", "esptool",
            "--chip", "esp32c6",
            "-p", port,
            "-b", str(baud),
            "--connect-attempts", "10",
            "--before", "default-reset",
            "--after", "hard-reset",
            "write_flash",
            "--flash_mode", "dio",
            "--flash_size", "8MB",
            "--flash_freq", "80m",
            "0x0", bootloader,
            "0x8000", partition,
            "0x10000", app_bin
        ]
        proc = subprocess.run(cmd, cwd=app_dir, capture_output=True, text=True)

    duration = time.time() - t0
    if proc.returncode == 0:
        print(f"[\033[92mFLASH SUCCESS\033[0m] Node {node_id} ({name}) on {port} flashed in {duration:.2f} s")
        return True, node_id, port, duration, ""
    else:
        err_msg = proc.stderr if proc.stderr else proc.stdout
        print(f"[\033[91mFLASH FAILED\033[0m] Node {node_id} on {port} failed in {duration:.2f} s")
        return False, node_id, port, duration, err_msg


def main():
    parser = argparse.ArgumentParser(description="Parallel Build & Multi-Node Flash Coordinator")
    parser.add_argument("--app-dir", default=r"C:\Git_ble_audio\apps\audio_VSAF_broadcast2", help="Application root directory")
    parser.add_argument("--build-only", action="store_true", help="Only compile without flashing")
    parser.add_argument("--flash-only", action="store_true", help="Only flash without compiling")
    parser.add_argument("--targets", default="all", choices=["all", "s3", "c6"], help="Build target selection")
    parser.add_argument("--nodes", default="auto", help="Comma-separated node IDs to flash (e.g. '16,23,24' or 'auto')")
    parser.add_argument("--baud", type=int, default=921600, help="Flash baud rate (default 921600)")
    args = parser.parse_args()

    app_dir = os.path.abspath(args.app_dir)
    kill_lingering_processes()

    # 1. PARALLEL COMPILATION PHASE
    if not args.flash_only:
        print_banner("1. CPU-AWARE PARALLEL COMPILATION PHASE")
        build_targets = []
        if args.targets in ["all", "s3"]:
            build_targets.append("esp32s3")
        if args.targets in ["all", "c6"]:
            build_targets.append("esp32c6")

        # Thread allocation rule:
        # Max 8 threads for single-build, 6 threads per target for dual parallel builds (total 12 threads)
        threads_per_target = 6 if len(build_targets) > 1 else 8

        t_build_start = time.time()
        build_results = []

        with ThreadPoolExecutor(max_workers=len(build_targets)) as executor:
            future_to_target = {
                executor.submit(compile_target, app_dir, target, threads_per_target): target
                for target in build_targets
            }
            for future in as_completed(future_to_target):
                build_results.append(future.result())

        total_build_time = time.time() - t_build_start
        all_builds_ok = all(r[0] for r in build_results)

        print(f"\n--- Compilation Summary (Total Parallel Wall-Clock Time: {total_build_time:.2f} s) ---")
        for ok, target, dur, _ in build_results:
            status = "\033[92mPASS\033[0m" if ok else "\033[91mFAIL\033[0m"
            print(f"  * {target.upper():<10}: {status} in {dur:.2f} s (threads: {threads_per_target})")

        if not all_builds_ok:
            print("\n[\033[91mABORT\033[0m] One or more firmware builds failed. Aborting flash phase.")
            sys.exit(1)

    if args.build_only:
        print("\n[DONE] Build-only completed successfully.")
        return

    # 2. PARALLEL MULTI-NODE FLASHING PHASE
    print_banner("2. CONCURRENT MULTI-NODE FLASHING PHASE")
    detected_nodes = scan_connected_nodes()

    if not detected_nodes:
        print("[\033[91mERROR\033[0m] No known cluster nodes detected on available COM ports!")
        sys.exit(1)

    # Filter target nodes if specified
    target_nodes = {}
    if args.nodes != "auto":
        requested_ids = [int(x.strip()) for x in args.nodes.split(",") if x.strip().isdigit()]
        for nid in requested_ids:
            if nid in detected_nodes:
                target_nodes[nid] = detected_nodes[nid]
            else:
                print(f"[\033[93mWARNING\033[0m] Requested Node {nid} not found on any active COM port.")
    else:
        target_nodes = detected_nodes

    print(f"Found {len(target_nodes)} node(s) ready for parallel flashing:")
    for nid, info in target_nodes.items():
        print(f"  * Node {nid:<2} -> {info['port']} ({info['name']})")

    t_flash_start = time.time()
    flash_results = []

    with ThreadPoolExecutor(max_workers=len(target_nodes)) as executor:
        future_to_node = {
            executor.submit(flash_single_node, app_dir, info, args.baud): info
            for info in target_nodes.values()
        }
        for future in as_completed(future_to_node):
            flash_results.append(future.result())

    total_flash_time = time.time() - t_flash_start

    # 3. FINAL SUMMARY TABLE
    print_banner("3. EXECUTION SUMMARY & DEPLOYMENT REPORT")
    print(f"Total Parallel Flash Wall-Clock Time: \033[92m{total_flash_time:.2f} s\033[0m for {len(target_nodes)} nodes\n")
    print(f"{'Node ID':<10} {'Port':<10} {'Role / Target':<25} {'Flash Duration':<18} {'Status'}")
    print("-" * 75)

    for ok, nid, port, dur, _ in sorted(flash_results, key=lambda x: x[1]):
        info = KNOWN_NODES.get(nid, {})
        role_str = f"{info.get('role', '')} ({info.get('chip', '')})"
        status = "\033[92mSUCCESS\033[0m" if ok else "\033[91mFAILED\033[0m"
        print(f"Node {nid:<5} {port:<10} {role_str:<25} {dur:.2f} s{'':<11} {status}")

    all_flash_ok = all(r[0] for r in flash_results)
    if all_flash_ok:
        print("\n\033[92m[CLUSTER DEPLOYED SUCCESSFULLY]\033[0m All nodes are rebooted and operational.")
    else:
        print("\n\033[91m[WARNING]\033[0m One or more nodes failed flashing.")
        sys.exit(1)


if __name__ == "__main__":
    main()
