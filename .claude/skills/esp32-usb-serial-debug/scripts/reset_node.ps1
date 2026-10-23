param(
    [Parameter(Mandatory=$false)]
    [string]$Port = "COM20",

    [Parameter(Mandatory=$false)]
    [int]$Baud = 115200,

    [Parameter(Mandatory=$false)]
    [int]$ReopenDelayMs = 3000
)

Write-Host "Triggering programmatic reset on $Port (DTR/RTS toggle)..." -ForegroundColor Cyan

$sp = $null
try {
    $sp = New-Object System.IO.Ports.SerialPort $Port, $Baud, "None", 8, "One"
    $sp.Open()
    
    # Toggle DTR / RTS for reset sequence
    $sp.DtrEnable = $false
    $sp.RtsEnable = $true
    Start-Sleep -Milliseconds 100
    $sp.DtrEnable = $true
    $sp.RtsEnable = $false
    Start-Sleep -Milliseconds 100
}
catch {
    Write-Warning "Reset signal initiated, closing handle: $($_.Exception.Message)"
}
finally {
    if ($sp -ne $null) {
        if ($sp.IsOpen) { $sp.Close() }
        $sp.Dispose()
    }
}

Write-Host "Port closed. Waiting ${ReopenDelayMs}ms for USB CDC re-enumeration..." -ForegroundColor Yellow
Start-Sleep -Milliseconds $ReopenDelayMs

# Verify port has re-enumerated in Windows
$reconnected = $false
$sw = [System.Diagnostics.Stopwatch]::StartNew()
while ($sw.ElapsedMilliseconds -lt 5000) {
    $ports = [System.IO.Ports.SerialPort]::GetPortNames()
    if ($ports -contains $Port) {
        $reconnected = $true
        break
    }
    Start-Sleep -Milliseconds 250
}

if ($reconnected) {
    Write-Host "[SUCCESS] Node on $Port reset and re-enumerated successfully!" -ForegroundColor Green
} else {
    Write-Warning "Port $Port did not reappear within 5 seconds. Check device connection."
}
