param(
    [string]$Port = "AUTO",
    [int]$Baud = 921600
)

$ErrorActionPreference = "Stop"

# Terminate lingering monitors
Get-CimInstance Win32_Process -Filter "CommandLine LIKE '%device monitor%' OR CommandLine LIKE '%idf_monitor%' OR CommandLine LIKE '%test_audio_matrix%' OR CommandLine LIKE '%pc_audio_streamer%'" | ForEach-Object { Stop-Process -Id $_.ProcessId -Force } -ErrorAction SilentlyContinue

# Environment Setup
if (Test-Path "C:\Users\stefa\OneDrive\Documents\ESP\.esptools") {
    $env:IDF_TOOLS_PATH = "C:\Users\stefa\OneDrive\Documents\ESP\.esptools"
} else {
    $env:IDF_TOOLS_PATH = "C:\Users\stefa\.espressif"
}

if (Test-Path "$env:IDF_TOOLS_PATH\python_env\idf6.0_py3.13_env") {
    $env:IDF_PYTHON_ENV_PATH = "$env:IDF_TOOLS_PATH\python_env\idf6.0_py3.13_env"
} else {
    $env:IDF_PYTHON_ENV_PATH = "$env:IDF_TOOLS_PATH\python_env\idf6.0_py3.11_env"
}
$env:PATH = "$env:IDF_PYTHON_ENV_PATH\Scripts;" + $env:PATH

if (Test-Path "C:\Users\stefa\OneDrive\Documents\ESP\v6.0.2\esp-idf\export.ps1") {
    . "C:\Users\stefa\OneDrive\Documents\ESP\v6.0.2\esp-idf\export.ps1"
} else {
    . "C:\Users\stefa\OneDrive\Documents\ESP\v6.0.2\export.ps1"
}

$toolsDir = Split-Path -Parent $MyInvocation.MyCommand.Path
python -u "$toolsDir\s3_flash_and_reset.py" --port $Port --baud $Baud
