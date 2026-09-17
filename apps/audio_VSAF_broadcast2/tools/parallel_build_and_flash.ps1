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
$scriptPath = "c:\Git_ble_audio\.agents\skills\parallel-compile-and-flash\scripts\parallel_build_and_flash.py"

$pythonExe = "python"
if (Test-Path "C:\Users\stefa\.espressif\python_env\idf6.0_py3.13_env\Scripts\python.exe") {
    $pythonExe = "C:\Users\stefa\.espressif\python_env\idf6.0_py3.13_env\Scripts\python.exe"
} elseif (Test-Path "C:\Users\stefa\.espressif\python_env\idf6.0_py3.11_env\Scripts\python.exe") {
    $pythonExe = "C:\Users\stefa\.espressif\python_env\idf6.0_py3.11_env\Scripts\python.exe"
}

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
