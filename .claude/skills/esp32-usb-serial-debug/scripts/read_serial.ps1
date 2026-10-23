param(
    [Parameter(Mandatory=$false)]
    [string]$Port = "COM20",

    [Parameter(Mandatory=$false)]
    [int]$Baud = 115200,

    [Parameter(Mandatory=$false)]
    [int]$DurationSeconds = 10,

    [Parameter(Mandatory=$false)]
    [int]$TimeoutMs = 1500,

    [Parameter(Mandatory=$false)]
    [int]$MaxRetries = 3,

    [Parameter(Mandatory=$false)]
    [int]$RetryDelayMs = 1000
)

# 1. Validate COM Port existence
$availablePorts = [System.IO.Ports.SerialPort]::GetPortNames()
if ($availablePorts -notcontains $Port) {
    Write-Error "Port $Port not found! Available ports: $($availablePorts -join ', ')"
    exit 1
}

# 2. Attempt to Open Serial Port with Retry on Permission Denied / Busy
$sp = $null
$opened = $false
$attempt = 0

while (-not $opened -and $attempt -lt $MaxRetries) {
    $attempt++
    try {
        $sp = New-Object System.IO.Ports.SerialPort $Port, $Baud, "None", 8, "One"
        $sp.ReadTimeout = $TimeoutMs
        $sp.WriteTimeout = $TimeoutMs
        $sp.DtrEnable = $true  # Prevent hanging DTR lines on CDC ACM
        $sp.RtsEnable = $false # Avoid accidental reset triggering
        $sp.Open()
        $opened = $true
        Write-Host "Successfully opened $Port @ $Baud baud (ReadTimeout: ${TimeoutMs}ms, Max Duration: ${DurationSeconds}s)." -ForegroundColor Green
    }
    catch [System.UnauthorizedAccessException] {
        Write-Warning "Attempt $attempt of $MaxRetries - Permission Denied for $Port. Port is currently locked by another process, subagent, or monitor."
        if ($attempt -lt $MaxRetries) {
            Write-Host "Waiting ${RetryDelayMs}ms before retrying..." -ForegroundColor Yellow
            Start-Sleep -Milliseconds $RetryDelayMs
        } else {
            Write-Error "Failed to open $Port after $MaxRetries attempts: Permission Denied. Please ensure all open monitors or subagents release $Port."
            exit 1
        }
    }
    catch [System.IO.IOException] {
        $msg = $_.Exception.Message
        Write-Warning "Attempt $attempt of $MaxRetries - I/O Error opening ${Port}: $msg"
        if ($attempt -lt $MaxRetries) {
            Start-Sleep -Milliseconds $RetryDelayMs
        } else {
            Write-Error "Fatal I/O Error accessing ${Port}: $msg"
            exit 1
        }
    }
    catch {
        $msg = $_.Exception.Message
        Write-Error "Unexpected error opening ${Port}: $msg"
        exit 1
    }
}

# 3. Read Loop with Timeout & Exception Protection
$startTime = [DateTime]::UtcNow
$lineCount = 0
try {
    while (([DateTime]::UtcNow - $startTime).TotalSeconds -lt $DurationSeconds) {
        try {
            $line = $sp.ReadLine()
            Write-Output $line
            $lineCount++
        }
        catch [System.TimeoutException] {
            # Expected when no data received within TimeoutMs; continue until DurationSeconds expires
        }
        catch [System.IO.IOException] {
            $msg = $_.Exception.Message
            Write-Warning "Serial connection lost on ${Port} (Device unplugged or reset): $msg"
            break
        }
        catch {
            $msg = $_.Exception.Message
            Write-Warning "Read error on ${Port}: $msg"
            break
        }
    }
}
finally {
    if ($sp -ne $null) {
        if ($sp.IsOpen) {
            $sp.Close()
        }
        $sp.Dispose()
    }
    Write-Host "`n[Summary] Captured $lineCount lines. Port $Port closed and resources released." -ForegroundColor DarkGray
}
