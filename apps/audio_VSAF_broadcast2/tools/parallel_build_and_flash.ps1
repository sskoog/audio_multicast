param(
    [Parameter(Mandatory=$false)]
    [ValidateSet("all", "s3", "c6")]
    [string]$Targets = "all",

    [Parameter(Mandatory=$false)]
    [string]$Nodes = "auto",

    [Parameter(Mandatory=$false)]
    [switch]$BuildOnly,

    [Parameter(Mandatory=$false)]
    [switch]$FlashOnly,

    [Parameter(Mandatory=$false)]
    [int]$Baud = 921600
)

$ErrorActionPreference = "Stop"

$toolsDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$appDir = Split-Path -Parent $toolsDir
$repoRoot = Split-Path -Parent (Split-Path -Parent $appDir)
$scriptPath = Join-Path $repoRoot ".claude\skills\parallel-compile-and-flash\scripts\parallel_build_and_flash.py"

# Coordinator only needs pyserial; the repo venv has it. Each build subprocess activates
# ESP-IDF itself via tools\idf_env.ps1.
$pythonExe = Join-Path $repoRoot ".venv_audio_multicast\Scripts\python.exe"
if (-not (Test-Path $pythonExe)) { $pythonExe = "python" }

$cmdArgs = @(
    "-u",
    $scriptPath,
    "--app-dir", $appDir,
    "--targets", $Targets,
    "--nodes", $Nodes,
    "--baud", $Baud
)

if ($BuildOnly) {
    $cmdArgs += "--build-only"
}
if ($FlashOnly) {
    $cmdArgs += "--flash-only"
}

& $pythonExe $cmdArgs
exit $LASTEXITCODE
