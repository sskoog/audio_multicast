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
    [switch]$OnlyFlash
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
    if ($Role -eq "SOURCE" -or $targetPort -eq "COM16" -or $targetPort -eq "COM116") {
        $Chip = "esp32s3"
    } else {
        $Chip = "esp32c6"
    }
}

if ($NodeId -eq 0) {
    if ($targetPort -eq "COM16" -or $targetPort -eq "COM116") {
        $NodeId = 16
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
Write-Host " Audio Unicast: $Role (Chip: $Chip, Node $NodeId on $targetPort)" -ForegroundColor Cyan
Write-Host "==========================================================" -ForegroundColor Cyan

# 1. Environment Setup (ESP-IDF v6.0.2)
$env:IDF_TOOLS_PATH = "C:\Users\stefa\.espressif"

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

# 1.5 Target sdkconfig isolation (Rule 01_esp32_specific.md)
if ($Chip -eq "esp32s3") {
    if (Test-Path "$appDir\sdkconfig.s3") {
        Copy-Item "$appDir\sdkconfig.s3" "$appDir\sdkconfig" -Force
    } elseif (Test-Path "$appDir\sdkconfig") {
        Remove-Item "$appDir\sdkconfig" -Force
    }
} else {
    if (Test-Path "$appDir\sdkconfig.c6") {
        Copy-Item "$appDir\sdkconfig.c6" "$appDir\sdkconfig" -Force
    }
}

# 2. Compile Firmware (unless -OnlyFlash)
if (-not $OnlyFlash) {
    Write-Host "[BUILD] Compiling firmware for $Role ($Chip) in $buildDir..." -ForegroundColor Yellow
    Push-Location $appDir
    try {
        & idf.py -B "$buildDir" -D "IDF_TARGET=$Chip" build
        if ($LASTEXITCODE -eq 0) {
            if ($Chip -eq "esp32s3") {
                Copy-Item "$appDir\sdkconfig" "$appDir\sdkconfig.s3" -Force
            } else {
                Copy-Item "$appDir\sdkconfig" "$appDir\sdkconfig.c6" -Force
            }
        }
    } finally {
        Pop-Location
    }

    if ($LASTEXITCODE -ne 0) {
        Write-Error "Build failed with exit code $LASTEXITCODE"
        exit $LASTEXITCODE
    }
}

# 3. Kill lingering serial monitors
Get-CimInstance Win32_Process -Filter "CommandLine LIKE '%device monitor%' OR CommandLine LIKE '%idf_monitor%' OR CommandLine LIKE '%test_audio_matrix%' OR CommandLine LIKE '%pc_unicast_streamer%' OR CommandLine LIKE '%pc_audio_streamer%'" | ForEach-Object { 
    Stop-Process -Id $_.ProcessId -Force 
} -ErrorAction SilentlyContinue
Start-Sleep -Milliseconds 1000

# 4. Flash Firmware
if ($Chip -eq "esp32s3") {
    # Use hands-free RTC Watchdog reset tool for Node 16
    Write-Host "[FLASH] Executing hands-free S3 flash & reset for Node $NodeId on $targetPort..." -ForegroundColor Yellow
    python -u "$toolsDir\s3_flash_and_reset.py" --port $targetPort --baud $Baud --bin-dir "$buildDir"
} else {
    # Standard ESP32-C6 flashing
    $bootloader = "$buildDir\bootloader\bootloader.bin"
    $partition = "$buildDir\partition_table\partition-table.bin"
    $appBin = "$buildDir\audio_ESP_NOW_unicast.bin"

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
