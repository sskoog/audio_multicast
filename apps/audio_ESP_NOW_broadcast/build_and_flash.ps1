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
    [int]$Baud = 460800
)

$targetPort = if ($Port -ne "") { $Port } elseif ($Role -eq "SOURCE") { "COM16" } else { "COM23" }

if ($Chip -eq "") {
    if ($targetPort -eq "COM16" -or $Role -eq "SOURCE") {
        $Chip = "esp32s3"
    } else {
        $Chip = "esp32c6"
    }
}

if ($NodeId -eq 0) {
    if ($targetPort -eq "COM16") {
        $NodeId = 16
    } elseif ($targetPort -eq "COM24") {
        $NodeId = 24
    } elseif ($targetPort -eq "COM23") {
        $NodeId = 23
    } elseif ($targetPort -eq "COM21" -or $targetPort -eq "COM121") {
        $NodeId = 21
    } elseif ($Role -eq "SOURCE") {
        $NodeId = 16
    } else {
        $NodeId = 23
    }
}
$nodeRole = if ($Role -eq "SOURCE") { "NODE_ROLE_SOURCE" } else { "NODE_ROLE_SINK" }

Write-Host "==========================================================" -ForegroundColor Cyan
Write-Host " Building & Flashing audioESP-NOW for Role: $Role (Chip: $Chip, Node $NodeId on $targetPort)" -ForegroundColor Cyan
Write-Host "==========================================================" -ForegroundColor Cyan

# 1. Environment Setup (ESP-IDF v6.0.2 with full Xtensa & RISC-V support)
if (Test-Path "C:\Users\stefa\OneDrive\Documents\ESP\.esptools") {
    $env:IDF_TOOLS_PATH="C:\Users\stefa\OneDrive\Documents\ESP\.esptools"
} else {
    $env:IDF_TOOLS_PATH="C:\Users\stefa\.espressif"
}

if (Test-Path "$env:IDF_TOOLS_PATH\python_env\idf6.0_py3.13_env") {
    $env:IDF_PYTHON_ENV_PATH="$env:IDF_TOOLS_PATH\python_env\idf6.0_py3.13_env"
} else {
    $env:IDF_PYTHON_ENV_PATH="$env:IDF_TOOLS_PATH\python_env\idf6.0_py3.11_env"
}
$env:PATH="$env:IDF_PYTHON_ENV_PATH\Scripts;" + $env:PATH

if (Test-Path "C:\Users\stefa\OneDrive\Documents\ESP\v6.0.2\esp-idf\export.ps1") {
    . "C:\Users\stefa\OneDrive\Documents\ESP\v6.0.2\esp-idf\export.ps1"
} else {
    . "C:\Users\stefa\OneDrive\Documents\ESP\v6.0.2\export.ps1"
}

# 2. Select target build dir and sdkconfig
$appDir = $PSScriptRoot
if ($Chip -eq "esp32s3") {
    $buildDir = "$appDir\build_s3"
    if (Test-Path "$appDir\sdkconfig.s3") {
        Copy-Item "$appDir\sdkconfig.s3" "$appDir\sdkconfig" -Force
    }
    $flashSize = "4MB"
} else {
    $buildDir = "$appDir\build_c6"
    if (Test-Path "$appDir\sdkconfig.c6") {
        Copy-Item "$appDir\sdkconfig.c6" "$appDir\sdkconfig" -Force
    }
    $flashSize = "8MB"
}

# 3. Build firmware
Push-Location $appDir
try {
    Write-Host "Compiling firmware for $Role ($Chip)..." -ForegroundColor Yellow
    & idf.py -B "$buildDir" -D "IDF_TARGET=$Chip" build
    if ($LASTEXITCODE -ne 0) {
        Write-Error "Build failed with exit code $LASTEXITCODE"
        exit $LASTEXITCODE
    }
} finally {
    Pop-Location
}

# 4. Check Port & Kill stale monitors / streamers
Get-CimInstance Win32_Process -Filter "CommandLine LIKE '%device monitor%' OR CommandLine LIKE '%idf_monitor%' OR CommandLine LIKE '%pc_audio_streamer%'" | ForEach-Object { 
    Stop-Process -Id $_.ProcessId -Force 
}
Start-Sleep -Milliseconds 1500

# 5. Flash Firmware via esptool.py
$bootloader = "$buildDir\bootloader\bootloader.bin"
$partition = "$buildDir\partition_table\partition-table.bin"
$appBin = "$buildDir\esp32_espnow_audio.bin"

Write-Host "Flashing $Role to $targetPort at $Baud baud..." -ForegroundColor Yellow
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

Write-Host "Successfully flashed $Role to $targetPort!" -ForegroundColor Green

