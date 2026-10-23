<#
.SYNOPSIS
    Migrates the local repository root directory from 'Git_ble_audio' to 'Git_audio_multicast'.
.DESCRIPTION
    1. Terminates any active serial monitors or background Python streamers.
    2. Switches PowerShell working directory outside of 'Git_ble_audio'.
    3. Renames 'C:\Git_ble_audio' to 'C:\Git_audio_multicast'.
    4. Updates git remote origin URL to 'https://github.com/sskoog/audio_multicast.git'.
    5. Provides instructions for reopening the workspace.
.EXAMPLE
    # Close Antigravity IDE / VS Code, then in PowerShell run:
    powershell -ExecutionPolicy Bypass -File "C:\Git_ble_audio\tools\migrate_to_audio_multicast.ps1"
#>

[CmdletBinding()]
param(
    [string]$OldPath = "C:\Git_ble_audio",
    [string]$NewPath = "C:\Git_audio_multicast",
    [string]$NewRemote = "https://github.com/sskoog/audio_multicast.git"
)

Write-Host "========================================================" -ForegroundColor Cyan
Write-Host " Migration Tool: Git_ble_audio -> Git_audio_multicast" -ForegroundColor Cyan
Write-Host "========================================================" -ForegroundColor Cyan

# 1. Terminate conflicting processes that might lock files/ports
Write-Host "[1/5] Checking for background Python streamers and monitors..." -ForegroundColor Yellow
$processes = Get-CimInstance Win32_Process -Filter "CommandLine LIKE '%pc_audio_streamer%' OR CommandLine LIKE '%pc_unicast_streamer%' OR CommandLine LIKE '%bumble_broadcaster%'"
if ($processes) {
    foreach ($p in $processes) {
        Write-Host "  Terminating process: $($p.ProcessId) ($($p.Name))" -ForegroundColor DarkYellow
        Stop-Process -Id $p.ProcessId -Force -ErrorAction SilentlyContinue
    }
} else {
    Write-Host "  No conflicting processes found." -ForegroundColor Green
}

# 2. Change current location out of target directory to prevent lock
Write-Host "[2/5] Switching working directory to C:\..." -ForegroundColor Yellow
Set-Location "C:\"

# 3. Check source directory existence
if (-not (Test-Path $OldPath)) {
    if (Test-Path $NewPath) {
        Write-Host "Notice: '$OldPath' does not exist, but '$NewPath' already exists!" -ForegroundColor Green
    } else {
        Write-Error "Error: Neither '$OldPath' nor '$NewPath' could be found."
        exit 1
    }
} else {
    # 4. Attempt folder rename
    Write-Host "[3/5] Renaming '$OldPath' to '$NewPath'..." -ForegroundColor Yellow
    try {
        Rename-Item -Path $OldPath -NewName (Split-Path -Leaf $NewPath) -ErrorAction Stop
        Write-Host "  Successfully renamed directory to '$NewPath'!" -ForegroundColor Green
    } catch {
        Write-Host "  Could not rename folder directly: $($_.Exception.Message)" -ForegroundColor Red
        Write-Host "  Please ensure all IDE windows, terminals, and editor processes in '$OldPath' are closed." -ForegroundColor Yellow
        Write-Host "  Then run in PowerShell:" -ForegroundColor Yellow
        Write-Host "    Rename-Item -Path '$OldPath' -NewName '$(Split-Path -Leaf $NewPath)'" -ForegroundColor White
        exit 1
    }
}

# 5. Update Git Remote URL
Write-Host "[4/5] Updating Git Remote Origin URL..." -ForegroundColor Yellow
Set-Location $NewPath
git remote set-url origin $NewRemote
$currentRemote = git remote get-url origin
Write-Host "  Git remote origin set to: $currentRemote" -ForegroundColor Green

# 6. Virtual environment notice
Write-Host "[5/5] Checking Python Virtual Environment..." -ForegroundColor Yellow
Write-Host "  Note: If using Python virtual environments, paths in pyvenv.cfg may need update." -ForegroundColor DarkCyan
Write-Host "  To create a clean environment in the new folder:" -ForegroundColor DarkCyan
Write-Host "    cd $NewPath" -ForegroundColor White
Write-Host "    python -m venv .venv_audio_multicast" -ForegroundColor White
Write-Host "    .\.venv_audio_multicast\Scripts\pip install -r apps\audio_ESP_NOW_broadcast\requirements.txt" -ForegroundColor White

Write-Host "`nMigration Complete!" -ForegroundColor Green
Write-Host "Open the workspace using:" -ForegroundColor Cyan
Write-Host "  code $NewPath\audio_multicast.code-workspace" -ForegroundColor White
Write-Host "========================================================" -ForegroundColor Cyan
