---
trigger: always_on
---

# Rule: Coding Standards & Response Guidelines

**Python style**:
- Generally follow PEP 8 Style Guide for Python Code
- For functions longer than 10 rows of code, include multi-line comment describing input and output parameters for all functions, including units for parameters.
- Use type hints, also for function return values.
- Use 'uv' as package manager.
- Use virtual Python environment at .venv_audio_multicast.

**C++ style**:
- Use modern C++20 with RAII for memory and FreeRTOS wrapper handles.
- For functions longer than 10 rows of code, include multi-line comment describing input and output parameters for all functions, including units for parameters.
- All component in audio pipeline (LC3 encode/decode, DMA I2S pump) must be statically assigned (zero-allocation) in steady-state operation (no dynamic `malloc`/`new` inside tick loops).
- Use thread-safe design pattern for FreeRTOS unless the task or function is explicitly NOT intended for an embedded system.