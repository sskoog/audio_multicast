param(
    [string]$Port = "AUTO",
    [int]$Baud = 921600
)

$ErrorActionPreference = "Stop"

# Terminate lingering monitors
Get-CimInstance Win32_Process -Filter "CommandLine LIKE '%device monitor%' OR CommandLine LIKE '%idf_monitor%' OR CommandLine LIKE '%test_audio_matrix%' OR CommandLine LIKE '%pc_audio_streamer%'" | ForEach-Object { Stop-Process -Id $_.ProcessId -Force } -ErrorAction SilentlyContinue

# Environment
$env:IDF_TOOLS_PATH = "C:\Users\stefa\.espressif"
. "C:\Users\stefa\OneDrive\Documents\ESP\v6.0.2\esp-idf\export.ps1"

python -u apps\audio_ESP_NOW_unicast\s3_flash_and_reset.py --port $Port --baud $Baud


