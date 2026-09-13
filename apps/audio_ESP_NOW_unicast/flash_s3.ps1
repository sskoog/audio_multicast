param(
    [string]$Port = "COM16",
    [int]$Baud = 921600
)

$ErrorActionPreference = "Stop"

# Terminate lingering monitors
Get-CimInstance Win32_Process -Filter "CommandLine LIKE '%device monitor%' OR CommandLine LIKE '%idf_monitor%' OR CommandLine LIKE '%test_audio_matrix%' OR CommandLine LIKE '%pc_audio_streamer%'" | ForEach-Object { Stop-Process -Id $_.ProcessId -Force } -ErrorAction SilentlyContinue

Write-Host "Waiting for $Port (ESP32-S3 ROM bootloader)..." -ForegroundColor Cyan
$timeout = 180
$found = $false
for ($i = 0; $i -lt $timeout; $i++) {
    $ports = [System.IO.Ports.SerialPort]::GetPortNames()
    if ($ports -contains $Port) {
        $found = $true
        break
    }
    Start-Sleep -Seconds 1
}

if (-not $found) {
    Write-Error "Port $Port not found within ${timeout}s. Please hold the 'B' button and tap 'R' on the XIAO ESP32-S3 to enter bootloader mode."
}

Write-Host "Found $Port! Flashing firmware..." -ForegroundColor Green

# Environment
$env:IDF_TOOLS_PATH = "C:\Users\stefa\.espressif"
. "C:\Users\stefa\OneDrive\Documents\ESP\v6.0.2\esp-idf\export.ps1"

python -m esptool --chip esp32s3 -p $Port -b $Baud --before default-reset --after no-reset write-flash --flash-mode dio --flash-size 4MB --flash-freq 80m 0x0 apps\audio_ESP_NOW_unicast\build_s3\bootloader\bootloader.bin 0x8000 apps\audio_ESP_NOW_unicast\build_s3\partition_table\partition-table.bin 0x10000 apps\audio_ESP_NOW_unicast\build_s3\audio_ESP_NOW_unicast.bin

Write-Host "Firmware flashed! Performing clean hardware reset into flash boot..." -ForegroundColor Cyan
python -c "import serial, time; s = serial.Serial('$Port', 115200); s.dtr = False; s.rts = False; time.sleep(0.05); s.dtr = False; s.rts = True; time.sleep(0.1); s.rts = False; s.dtr = False; time.sleep(0.2); s.close()"

Write-Host "Reset complete. Waiting for TinyUSB re-enumeration..." -ForegroundColor Green
Start-Sleep -Seconds 3

Get-PnpDevice | Where-Object { $_.InstanceId -like '*PID_4002*' } | Select-Object Status, Problem, FriendlyName, InstanceId | Format-Table -AutoSize
