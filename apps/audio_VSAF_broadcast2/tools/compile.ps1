param(
    [ValidateSet("esp32s3", "esp32c6", "s3", "c6")]
    [string]$Target = "s3",
    [int]$Jobs = 8
)

$ErrorActionPreference = "Stop"

$toolsDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$appDir = Split-Path -Parent $toolsDir

if ($Target -eq "s3" -or $Target -eq "esp32s3") {
    $chip = "esp32s3"
    $buildDir = "$appDir\build_s3"
    $sdkSrc = "$appDir\sdkconfig.s3"
} else {
    $chip = "esp32c6"
    $buildDir = "$appDir\build_c6"
    $sdkSrc = "$appDir\sdkconfig.c6"
}

if (Test-Path "C:\Users\stefa\.espressif") {
    $env:IDF_TOOLS_PATH = "C:\Users\stefa\.espressif"
} else {
    $env:IDF_TOOLS_PATH = "C:\Users\stefa\OneDrive\Documents\ESP\.esptools"
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

if (Test-Path $sdkSrc) {
    Copy-Item $sdkSrc "$appDir\sdkconfig" -Force
}

Push-Location $appDir
try {
    & idf.py -B "$buildDir" -D "IDF_TARGET=$chip" build
    if ($LASTEXITCODE -eq 0) {
        if ($chip -eq "esp32s3") {
            Copy-Item "$appDir\sdkconfig" "$appDir\sdkconfig.s3" -Force
        } else {
            Copy-Item "$appDir\sdkconfig" "$appDir\sdkconfig.c6" -Force
        }
        Write-Host "[SUCCESS] Build succeeded for $chip in $buildDir" -ForegroundColor Green
    }
} finally {
    Pop-Location
}
