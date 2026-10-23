# List active COM ports with device descriptions
$env:IDF_TOOLS_PATH = "C:\Users\stefa\OneDrive\Documents\ESP\.esptools"
$env:IDF_PYTHON_ENV_PATH = "C:\Users\stefa\OneDrive\Documents\ESP\.esptools\python_env\idf5.2_py3.11_env"
$env:PATH = "C:\Users\stefa\OneDrive\Documents\ESP\.esptools\python_env\idf5.2_py3.11_env\Scripts;" + $env:PATH
$py = "$env:IDF_PYTHON_ENV_PATH\Scripts\python.exe"

Write-Host "Scanning available COM ports with device descriptions..." -ForegroundColor Cyan

& $py -c "import serial.tools.list_ports; ports = list(serial.tools.list_ports.comports()); print('\n'.join(['  * ' + p.device + ' : ' + p.description + ' [' + (p.hwid or 'Unknown HWID') + ']' for p in ports]) if ports else 'No COM ports detected.')"
