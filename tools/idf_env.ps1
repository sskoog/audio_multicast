# Shared ESP-IDF environment activation for all build/flash scripts in this repo.
#
# Usage (dot-source so the environment lands in the caller's scope):
#   . "$repoRoot\tools\idf_env.ps1"                      # default version below
#   . "$repoRoot\tools\idf_env.ps1" -IdfVersion 6.0.2     # override for one run
#
# Bumping the repo to a new ESP-IDF release = change $DefaultIdfVersion here.

param(
    [string]$IdfVersion = ""
)

$DefaultIdfVersion = "6.1"
if ($IdfVersion -eq "") {
    $IdfVersion = if ($env:AUDIO_MC_IDF_VERSION) { $env:AUDIO_MC_IDF_VERSION } else { $DefaultIdfVersion }
}

$espRoot = "C:\Users\stefa\OneDrive\Documents\ESP"
$env:IDF_TOOLS_PATH = "C:\Users\stefa\.espressif"

# IDF checkout: <espRoot>\v<ver>\esp-idf (standard) or <espRoot>\v<ver> (flat)
$idfPath = "$espRoot\v$IdfVersion\esp-idf"
if (-not (Test-Path "$idfPath\export.ps1")) { $idfPath = "$espRoot\v$IdfVersion" }
if (-not (Test-Path "$idfPath\export.ps1")) {
    throw "ESP-IDF v$IdfVersion not found under $espRoot (expected v$IdfVersion\esp-idf\export.ps1)"
}

# Python env is named by major.minor only, e.g. idf6.1_py3.13_env. Pick the newest Python.
$majorMinor = (($IdfVersion -split '\.')[0..1]) -join '.'
$pyEnv = Get-ChildItem "$env:IDF_TOOLS_PATH\python_env" -Directory -Filter "idf${majorMinor}_py*_env" -ErrorAction SilentlyContinue |
    Sort-Object { [version](($_.Name -replace "^idf${majorMinor}_py", '') -replace '_env$', '') } -Descending |
    Select-Object -First 1
if (-not $pyEnv) {
    throw "No Python env idf${majorMinor}_py*_env in $env:IDF_TOOLS_PATH\python_env. Run install.ps1 in $idfPath first."
}
$env:IDF_PYTHON_ENV_PATH = $pyEnv.FullName
$env:PATH = "$env:IDF_PYTHON_ENV_PATH\Scripts;" + $env:PATH

. "$idfPath\export.ps1" | Out-Null

# Exposed for callers that need the IDF Python interpreter directly
$IdfPythonExe = "$env:IDF_PYTHON_ENV_PATH\Scripts\python.exe"

# Call before 'idf.py build': if the build dir was configured by a different ESP-IDF
# Python env (i.e. another IDF version), run 'idf.py fullclean' so CMake reconfigures.
function Reset-StaleIdfBuildDir([string]$BuildDir) {
    $cache = Join-Path $BuildDir "CMakeCache.txt"
    if (-not (Test-Path $cache)) { return }
    $line = Select-String -Path $cache -Pattern '^PYTHON:[A-Z]+=(.*)$' | Select-Object -First 1
    if ($line -and ($line.Matches[0].Groups[1].Value -ne $IdfPythonExe)) {
        Write-Host "[ENV] $BuildDir was configured with another ESP-IDF version -> idf.py fullclean" -ForegroundColor Yellow
        & idf.py -B "$BuildDir" fullclean | Out-Null
    }
}
Write-Host "[ENV] ESP-IDF v$IdfVersion ($idfPath), Python env $($pyEnv.Name)" -ForegroundColor DarkCyan
