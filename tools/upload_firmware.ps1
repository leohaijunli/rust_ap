# Build, enter the original PX4 bootloader, upload/verify and restart (Windows).
# Requires exactly one FMUv6X; does not touch the bootloader or update the IO MCU.
param([switch]$DryRun)
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
Push-Location $root
try {
    & (Join-Path $PSScriptRoot 'build_firmware.ps1')
    $target = if ($env:CARGO_TARGET_DIR) { $env:CARGO_TARGET_DIR } else { Join-Path $root 'target' }
    $release = Join-Path $target 'thumbv7em-none-eabihf/release'
    $venv = Join-Path $root '.venv/Scripts/python.exe'
    $python = if (Test-Path $venv) { $venv } else { 'py' }
    [string[]]$pythonArgs = if ($python -eq 'py') { @('-3') } else { @() }
    $uploadArgs = @('tools/run_host_tool.py', 'tools/upload_windows.py', '--release', $release)
    if ($DryRun) { $uploadArgs += '--dry-run' }
    & $python @pythonArgs @uploadArgs
    if ($LASTEXITCODE -ne 0) { throw 'upload/preflight failed' }
} finally { Pop-Location }
