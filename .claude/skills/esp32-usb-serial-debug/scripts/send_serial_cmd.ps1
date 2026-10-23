param(
    [Parameter(Mandatory=$true)]
    [string]$Command,

    [Parameter(Mandatory=$false)]
    [string]$Port = "COM20",

    [Parameter(Mandatory=$false)]
    [int]$Baud = 115200,

    [Parameter(Mandatory=$false)]
    [int]$ListenSeconds = 5,

    [Parameter(Mandatory=$false)]
    [int]$TimeoutMs = 1500
)

# 1. Validate COM Port existence
$availablePorts = [System.IO.Ports.SerialPort]::GetPortNames()
if ($availablePorts -notcontains $Port) {
    Write-Error "Port $Port not found! Available ports: $($availablePorts -join ', ')"
    exit 1
}

$sp = $null
try {
    $sp = New-Object System.IO.Ports.SerialPort $Port, $Baud, "None", 8, "One"
    $sp.ReadTimeout = $TimeoutMs
    $sp.WriteTimeout = $TimeoutMs
    $sp.DtrEnable = $true
    $sp.RtsEnable = $false
    $sp.Open()

    Write-Host "Sending command '$Command' to $Port..." -ForegroundColor Cyan
    $sp.WriteLine($Command)

    Write-Host "Listening for response for $ListenSeconds seconds..." -ForegroundColor DarkGray
    $startTime = [DateTime]::UtcNow
    while (([DateTime]::UtcNow - $startTime).TotalSeconds -lt $ListenSeconds) {
        try {
            $line = $sp.ReadLine()
            Write-Output $line
        } catch [System.TimeoutException] {
            # Continue listening
        }
    }
}
catch [System.UnauthorizedAccessException] {
    Write-Error "PERMISSION DENIED: $Port is locked by another process or monitor."
    exit 1
}
catch {
    Write-Error "Error during serial transaction on ${Port}: $($_.Exception.Message)"
    exit 1
}
finally {
    if ($sp -ne $null) {
        if ($sp.IsOpen) { $sp.Close() }
        $sp.Dispose()
    }
    Write-Host "`nClosed $Port." -ForegroundColor DarkGray
}
