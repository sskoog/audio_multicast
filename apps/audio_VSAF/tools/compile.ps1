param(
    [ValidateSet("esp32s3", "esp32c6", "s3", "c6")]
    [string]$Target = "s3",
    [int]$Jobs = 8,
    [string]$IdfVersion = ""
)

$ErrorActionPreference = "Stop"

$toolsDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$appDir = Split-Path -Parent $toolsDir
$repoRoot = Split-Path -Parent (Split-Path -Parent $appDir)

if ($Target -eq "s3" -or $Target -eq "esp32s3") {
    $chip = "esp32s3"
    $buildDir = "$appDir\build_s3"
    $sdkConfig = "$appDir\sdkconfig.s3"
} else {
    $chip = "esp32c6"
    $buildDir = "$appDir\build_c6"
    $sdkConfig = "$appDir\sdkconfig.c6"
}

. "$repoRoot\tools\idf_env.ps1" -IdfVersion $IdfVersion

# Per-target sdkconfig is passed directly (no shared 'sdkconfig' copy), so S3 and C6 can build concurrently.
$env:CMAKE_BUILD_PARALLEL_LEVEL = $Jobs
Push-Location $appDir
try {
    Reset-StaleIdfBuildDir $buildDir
    & idf.py -B "$buildDir" -D "IDF_TARGET=$chip" -D "SDKCONFIG=$sdkConfig" build
    $rc = $LASTEXITCODE
} finally {
    Pop-Location
}

if ($rc -ne 0) {
    Write-Host "[FAILED] Build failed for $chip (exit $rc)" -ForegroundColor Red
    exit $rc
}
Write-Host "[SUCCESS] Build succeeded for $chip in $buildDir" -ForegroundColor Green
