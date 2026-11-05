# Compile the release H753 image and create the validated .px4 package (Windows).
# Equivalent to `bash tools/firmware.sh build`. Errors exit non-zero.
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
Push-Location $root
$target = if ($env:CARGO_TARGET_DIR) { $env:CARGO_TARGET_DIR } else { Join-Path $root 'target' }
$release = Join-Path $target 'thumbv7em-none-eabihf/release'
$venv = Join-Path $root '.venv/Scripts/python.exe'
$python = if (Test-Path $venv) { $venv } else { 'py' }
[string[]]$pythonArgs = if ($python -eq 'py') { @('-3') } else { @() }

try {
cargo build --release --locked --target thumbv7em-none-eabihf --bin v6x-rust-led
if ($LASTEXITCODE -ne 0) { throw 'cargo build failed' }

$host_target = (rustc -vV) | Select-String -Pattern '^host: ' | ForEach-Object { $_.Line -replace '^host: ', '' }
$llvm = Join-Path (rustc --print sysroot) "lib/rustlib/$host_target/bin"
& (Join-Path $llvm 'llvm-objcopy.exe') -O binary (Join-Path $release 'v6x-rust-led') (Join-Path $release 'v6x-rust-led.bin')
if ($LASTEXITCODE -ne 0) { throw 'llvm-objcopy failed' }

& $python @pythonArgs tools/package.py --binary (Join-Path $release 'v6x-rust-led.bin') --output (Join-Path $release 'v6x-rust-led.px4')
if ($LASTEXITCODE -ne 0) { throw 'package.py failed' }

& $python @pythonArgs tools/run_host_tool.py tools/build_report.py --release $release
if ($LASTEXITCODE -ne 0) { throw 'build_report.py failed' }
& $python @pythonArgs tools/check_stack_frames.py --elf (Join-Path $release 'v6x-rust-led')
if ($LASTEXITCODE -ne 0) { throw 'ARM stack-frame budget failed' }
} finally { Pop-Location }
