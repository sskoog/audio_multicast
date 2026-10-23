param(
    [string]$Port = "AUTO",
    [int]$Baud = 921600,
    [string]$IdfVersion = ""
)

$ErrorActionPreference = "Stop"

# Terminate lingering monitors
Get-CimInstance Win32_Process -Filter "CommandLine LIKE '%device monitor%' OR CommandLine LIKE '%idf_monitor%' OR CommandLine LIKE '%test_audio_matrix%' OR CommandLine LIKE '%pc_audio_streamer%' OR CommandLine LIKE '%test_broadcast_cluster%'" | ForEach-Object { Stop-Process -Id $_.ProcessId -Force } -ErrorAction SilentlyContinue

$toolsDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $toolsDir))

# Environment Setup (shared helper, default ESP-IDF version set in tools\idf_env.ps1)
. "$repoRoot\tools\idf_env.ps1" -IdfVersion $IdfVersion

python -u "$toolsDir\s3_flash_and_reset.py" --port $Port --baud $Baud
