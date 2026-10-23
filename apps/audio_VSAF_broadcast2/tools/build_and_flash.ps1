param(
    [Parameter(Mandatory=$true)]
    [ValidateSet("SOURCE", "SINK")]
    [string]$Role,

    [Parameter(Mandatory=$false)]
    [string]$Port = "",

    [Parameter(Mandatory=$false)]
    [int]$NodeId = 0,

    [Parameter(Mandatory=$false)]
    [string]$Chip = "",

    [Parameter(Mandatory=$false)]
    [int]$Baud = 921600,

    [Parameter(Mandatory=$false)]
    [switch]$OnlyFlash,

    [Parameter(Mandatory=$false)]
    [switch]$OnlyCompile,

    [Parameter(Mandatory=$false)]
    [string]$IdfVersion = ""
)

$ErrorActionPreference = "Stop"

$toolsDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$appDir = Split-Path -Parent $toolsDir

# Default port and chip selection based on role
if ($Port -eq "") {
    if ($Role -eq "SOURCE") {
        $targetPort = "AUTO"
    } else {
        $targetPort = "COM23"
    }
} else {
    $targetPort = $Port
}

if ($Chip -eq "") {
    if ($Role -eq "SOURCE" -or $targetPort -eq "COM16" -or $targetPort -eq "COM116" -or $targetPort -eq "COM4" -or $targetPort -eq "COM5" -or $NodeId -eq 4 -or $NodeId -eq 5) {
        $Chip = "esp32s3"
    } else {
        $Chip = "esp32c6"
    }
}

if ($NodeId -eq 0) {
    if ($targetPort -eq "COM16" -or $targetPort -eq "COM116") {
        $NodeId = 16
    } elseif ($targetPort -eq "COM4") {
        $NodeId = 4
    } elseif ($targetPort -eq "COM5") {
        $NodeId = 5
    } elseif ($targetPort -eq "COM24") {
        $NodeId = 24
    } elseif ($targetPort -eq "COM23") {
        $NodeId = 23
    } elseif ($targetPort -eq "COM20") {
        $NodeId = 20
    } elseif ($Role -eq "SOURCE") {
        $NodeId = 16
    } else {
        $NodeId = 23
    }
}

$buildDir = if ($Chip -eq "esp32s3") { "$appDir\build_s3" } else { "$appDir\build_c6" }
$flashSize = if ($Chip -eq "esp32s3") { "4MB" } else { "8MB" }

Write-Host "==========================================================" -ForegroundColor Cyan
Write-Host " Audio VSAF Broadcast 2: $Role (Chip: $Chip, Node $NodeId on $targetPort)" -ForegroundColor Cyan
Write-Host "==========================================================" -ForegroundColor Cyan

# 1. Environment Setup (shared helper, default ESP-IDF version set in tools\idf_env.ps1)
$repoRoot = Split-Path -Parent (Split-Path -Parent $appDir)
. "$repoRoot\tools\idf_env.ps1" -IdfVersion $IdfVersion

# 2. Compile Firmware (unless -OnlyFlash)
# Per-target sdkconfig (Rule 01) is passed directly, so S3 and C6 builds never share a 'sdkconfig' file.
$sdkConfig = if ($Chip -eq "esp32s3") { "$appDir\sdkconfig.s3" } else { "$appDir\sdkconfig.c6" }
if (-not $OnlyFlash) {
    Write-Host "[BUILD] Compiling firmware for $Role ($Chip) in $buildDir..." -ForegroundColor Yellow
    Push-Location $appDir
    try {
        Reset-StaleIdfBuildDir $buildDir
        & idf.py -B "$buildDir" -D "IDF_TARGET=$Chip" -D "SDKCONFIG=$sdkConfig" build
        $buildRc = $LASTEXITCODE
    } finally {
        Pop-Location
    }

    if ($buildRc -ne 0) {
        Write-Error "Build failed with exit code $buildRc"
        exit $buildRc
    }
}

if ($OnlyCompile) {
    Write-Host "[SUCCESS] Compilation successful for $Role ($Chip)!" -ForegroundColor Green
    exit 0
}

# 3. Kill lingering serial monitors
Get-CimInstance Win32_Process -Filter "CommandLine LIKE '%device monitor%' OR CommandLine LIKE '%idf_monitor%' OR CommandLine LIKE '%test_audio_matrix%' OR CommandLine LIKE '%pc_unicast_streamer%' OR CommandLine LIKE '%pc_audio_streamer%'" | ForEach-Object { 
    Stop-Process -Id $_.ProcessId -Force 
} -ErrorAction SilentlyContinue
Start-Sleep -Milliseconds 1000

# 4. Flash Firmware
if ($Chip -eq "esp32s3" -and ($NodeId -eq 16 -or $targetPort -eq "COM16" -or $targetPort -eq "COM116" -or $targetPort -eq "AUTO")) {
    # Use hands-free RTC Watchdog reset tool for Node 16
    Write-Host "[FLASH] Executing hands-free S3 flash & reset for Node $NodeId on $targetPort..." -ForegroundColor Yellow
    python -u "$toolsDir\s3_flash_and_reset.py" --port $targetPort --baud $Baud --bin-dir "$buildDir"
} else {
    # Standard flashing for ESP32-C6 and ESP32-S3 (Node 4 on USB-Serial/JTAG)
    $bootloader = "$buildDir\bootloader\bootloader.bin"
    $partition = "$buildDir\partition_table\partition-table.bin"
    $appBin = "$buildDir\audio_VSAF_broadcast2.bin"

    Write-Host "[FLASH] Flashing $Role to $targetPort at $Baud baud..." -ForegroundColor Yellow
    python -m esptool `
        --chip $Chip `
        -p $targetPort `
        -b $Baud `
        --connect-attempts 10 `
        --before default_reset `
        --after hard_reset `
        write_flash `
        --flash_mode dio `
        --flash_size $flashSize `
        --flash_freq 80m `
        0x0 $bootloader `
        0x8000 $partition `
        0x10000 $appBin

    if ($LASTEXITCODE -ne 0) {
        Write-Error "Flashing failed on $targetPort"
        exit $LASTEXITCODE
    }
    Write-Host "[SUCCESS] Successfully flashed $Role to $targetPort!" -ForegroundColor Green
}
