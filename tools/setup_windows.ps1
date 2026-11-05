# Optional host-tool setup. Does not touch a connected board.
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
Push-Location $root
try {
    rustup show
    if ($LASTEXITCODE -ne 0) { throw 'pinned Rust setup failed' }
    if (-not (Test-Path '.venv/Scripts/python.exe')) {
        py -3 -m venv .venv
        if ($LASTEXITCODE -ne 0) { throw 'Python venv creation failed' }
    }
    & '.venv/Scripts/python.exe' -m pip install -r tools/requirements.txt
    if ($LASTEXITCODE -ne 0) { throw 'host requirements install failed' }
    $uploader = Join-Path $root '.venv/px4_uploader.py'
    $expected = 'f05929e1035e693005677e088e4e7085503b2e8ab3d7c8785f7bd84cd48880f8'
    if (-not (Test-Path $uploader)) {
        Invoke-WebRequest -UseBasicParsing -Uri 'https://raw.githubusercontent.com/PX4/PX4-Autopilot/a2be9197b57b2d065e4c47ca7fc5b7565ee07282/Tools/px4_uploader.py' -OutFile $uploader
    }
    if ((Get-FileHash $uploader -Algorithm SHA256).Hash.ToLowerInvariant() -ne $expected) { throw 'PX4 uploader SHA-256 mismatch' }
    Write-Host 'Windows host tools ready; no board operation performed.'
} finally { Pop-Location }
