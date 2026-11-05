param([int]$Port = 8766, [switch]$NoBrowser)
$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath $PSScriptRoot
$env:PYTHONPATH = Join-Path $PSScriptRoot 'src'
$env:PYTHONUTF8 = '1'
$wbPython = if (Test-Path -LiteralPath '.venv/Scripts/python.exe') { '.venv/Scripts/python.exe' } else { 'python' }
& $wbPython -c 'import numpy, scipy, serial'
if ($LASTEXITCODE -ne 0) { throw 'Install dependencies first: python -m pip install -r requirements-workbench.txt' }
if (!(Test-Path -LiteralPath 'src/pid_agent/workbench/native/pt_sdk.dll')) {
    & $wbPython tools/build_workbench.py
    if ($LASTEXITCODE -ne 0) { throw 'Native simulation build failed; see docs/WORKBENCH.md.' }
}
$wbArguments = @('-m', 'pid_agent.workbench', '--port', "$Port")
if (!$NoBrowser) { $wbArguments += '--open-browser' }
& $wbPython @wbArguments
exit $LASTEXITCODE
