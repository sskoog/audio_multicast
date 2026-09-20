---
trigger: always_on
---

# Rule: Build and Flash Operations (ESP-IDF)

1. **Toolchain version**: Generally use the latest available/installed toolchains (ESP-IDF) unless explicitly agreed with user. Prompt user if newer toolchains are available locally
   - Official source for ESP-IDF: https://github.com/espressif/esp-idf
   - Additional ESP components: https://components.espressif.com/ .
   - Prompt user if new version of ESP-IDF or ESP-ADF is available to download.
   - Local repository for all ESP-IDF versions: C:\Users\stefa\OneDrive\Documents\ESP\ .
   - ESP-IDF tools in C:\Users\stefa\.espressif .
   - Activate the environment using:
     ```powershell
     $env:IDF_TOOLS_PATH = "C:\Users\stefa\.espressif"
     . "[ESP-IDF-path]\esp-idf\export.ps1"
     ```

2. **COM-ports**
   - Embedded devices are normally connected to this PC via COM-port, where the COM-port number is assigned according to the node ID. Some nodes have two COM-ports, in which case 100 is added to the node number, e.g. node 16 has COM16 & COM116.
   - COM-ports are used for both flashing and for debugging via virtual serial port. Be aware of resource conflicts when flashing, e.g. locked COM-port from a serial interface.
   - If a COM-port resource is locked at the time for experiments or flashing, it is probably due to other serial monitors accidentally being active. If so, run a windows script to force close the com-port to free it up in order to proceed with your task.
   - Always add time-outs in scripts monitoring COM traffic.

3. **Port Locking Mitigation**:
   - Before invoking `esptool.py` or flash scripts, terminate all lingering Python scripts and serial monitors holding COM port handles:
     `Get-CimInstance Win32_Process -Filter "CommandLine LIKE '%device monitor%' OR CommandLine LIKE '%idf_monitor%' OR CommandLine LIKE '%test_audio_matrix%' OR CommandLine LIKE '%pc_audio_streamer%'" | ForEach-Object { Stop-Process -Id $_.ProcessId -Force }`

4. **Flashing Parameters**:
   - Flash Baud Rates: Default to high-speed flashing (--baud 921600) per port with --chip <target_chip>.   
   - Use `--connect-attempts 10`, `--before default_reset`, and `--after hard_reset`.
   - After flashing, allow a 2.5 to 3.0 second delay for Windows USB CDC / Serial JTAG re-enumeration before attempting to open the serial port.
   - Automated test runners and serial monitors must open COM ports with DTR=False and RTS=False (along with firmware-level USB CDC reset disabling) to prevent cross-node reset interference during connection.

5. **Compile-Time Architecture Gating**: Use target-specific macros only for silicon-dependent features (e.g., hardware FPU, SIMD vector instructions, multi-core core pinning):
```
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    // Xtensa Dual-Core + Hardware FPU / DSP SIMD
    xTaskCreatePinnedToCore(task_name, "task_name", 8192, nullptr, 3, nullptr, 1);
#elif defined(CONFIG_IDF_TARGET_ESP32C6)
    // Single-Core RISC-V (No FPU)
    xTaskCreate(task_name, "task_name", 8192, nullptr, 3, nullptr);
#endif
```

6. **Parallel compile and flash**

6.1. **Strict Build Folder Mapping**: Never share or alternate build directories across different target silicon architectures (IDF_TARGET). Always assign a dedicated, isolated build folder per chip family (Xtensa, RISC-V, etc):
    * ESP32-S3: build_s3/
    * ESP32-C6: build_c6/
    * ESP32-C3: build_c3/

6.2. **Multi-Architecture Toolchains**: When Xtensa (xtensa-esp-elf) and RISC-V (riscv32-esp-elf) toolchains reside on the machine, verify that toolchain binaries match the exact compiler version used by ESP-IDF (e.g., GCC 15.2.0).
    * **Purge Legacy Paths**: Subshells and build scripts must strip conflicting or outdated toolchain paths (e.g., older GCC 13.2.0 entries) from the PATH environment variable before launching CMake or Ninja.

6.2. **Isolated sdkconfig Instances**: Never overwrite a single sdkconfig when switching targets. Use target-specific configuration files:
    * Target configs: sdkconfig.s3, sdkconfig.c6
    * Shared baseline: sdkconfig.defaults

6.3. **Avoid Ninja Regeneration Traps**: Ninja will invoke CMake automatically when source or config files change. If the build folder was configured with target A, running a build for target B without a dedicated directory will cause silent target corruption or toolchain mismatch errors.

6.4. **Explicit CMake & Toolchain Invocation**: When configuring CMake programmatically or via scripts, always explicitly pass the full isolation parameters.

6.5. **CPU Thread Allocation Gating**: On 12-thread host machines (2 P-cores + 8 E-cores):
    * **Single-Target Build**: Cap Ninja worker threads at max **8 threads** (`-j 8`).
    * **Dual-Target Parallel Build**: Allocate exactly **6 threads per target** (`-j 6` for S3, `-j 6` for C6; total 12 threads) to prevent CPU starvation and thread thrashing.