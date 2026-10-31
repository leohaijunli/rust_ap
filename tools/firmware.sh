#!/usr/bin/env bash
# Shared entry point for RustRover run configurations and terminal use.
set -euo pipefail

root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root"
if [[ -r "$HOME/.cargo/env" ]]; then
    source "$HOME/.cargo/env"
fi

python="$root/.venv/bin/python"
release="${CARGO_TARGET_DIR:-target}/thumbv7em-none-eabihf/release"
action="${1:-help}"
if (( $# )); then shift; fi

usage() {
    echo "Usage: $0 {build|build-injection|build-dump|upload [--dry-run]|upload-dump [--dry-run]|monitor [seconds]|ports|check}"
}

case "$action" in
    help|-h|--help) usage; exit 0 ;;
    build|build-injection|build-dump|ports|check) (( $# == 0 )) || { usage >&2; exit 2; } ;;
    monitor) (( $# <= 1 )) || { usage >&2; exit 2; } ;;
    upload|upload-dump)
        if (( $# > 1 )) || [[ "${1:-}" != "" && "$1" != "--dry-run" ]]; then
            usage >&2
            exit 2
        fi
        ;;
    *) usage >&2; exit 2 ;;
esac

if [[ ! -x "$python" ]]; then
    echo "Create .venv and install tools/requirements.txt; see README.md." >&2
    exit 1
fi

build_firmware() {
    local mode="${1:-normal}"
    local features=()
    local report_flags=()
    case "$mode" in
        injection)
            export CARGO_TARGET_DIR="${CARGO_TARGET_DIR:-target}/p1-fault-injection"
            release="$CARGO_TARGET_DIR/thumbv7em-none-eabihf/release"
            features=(--features fault-injection)
            report_flags=(--fault-injection)
            ;;
        dump)
            export CARGO_TARGET_DIR="${CARGO_TARGET_DIR:-target}/p1-fault-dump"
            release="$CARGO_TARGET_DIR/thumbv7em-none-eabihf/release"
            features=(--features fault-dump)
            report_flags=(--fault-dump)
            ;;
        normal) ;;
        *)
            echo "Unknown firmware mode: $mode" >&2
            exit 2
            ;;
    esac
    cargo build --release --locked --bin v6x-rust-led "${features[@]}"
    local host llvm
    host="$(rustc -vV | sed -n 's/^host: //p')"
    llvm="$(rustc --print sysroot)/lib/rustlib/$host/bin"
    # Direct LLVM tools avoid cargo-binutils' temporary-file allocation.
    "$llvm/llvm-objcopy" -O binary "$release/v6x-rust-led" "$release/v6x-rust-led.bin"
    "$python" tools/package.py
    "$python" tools/build_report.py --release "$release" "${report_flags[@]}"
    "$llvm/llvm-size" -A "$release/v6x-rust-led" > "$release/sections.txt"
    "$llvm/llvm-objdump" --disassemble "$release/v6x-rust-led" > "$release/disassembly.txt"
}

upload_firmware() {
    local mode="$1"
    shift
    uploader="${PX4_UPLOADER:-$HOME/PX4-Autopilot/Tools/px4_uploader.py}"
    ports="${PX4_PORTS:-/dev/serial/by-id/usb-Rust_FMUv6X_V6X_Rust_CDC_V6X-RUST-001-if00,/dev/serial/by-id/usb-Auterion_PX4_FMU_v6X.x_*-if00,/dev/serial/by-id/*PX4_BL*}"
    if [[ ! -f "$uploader" ]]; then
        echo "Set PX4_UPLOADER to the path of PX4's Tools/px4_uploader.py." >&2
        exit 1
    fi
    "$python" "$uploader" --help >/dev/null
    "$python" - "$ports" <<'PY'
import glob
import os
import sys

ports = sorted({port for pattern in sys.argv[1].split(",") for port in glob.glob(pattern)})
if len(ports) != 1:
    sys.exit(f"Expected one FMUv6X serial port, found {ports}. Connect one board or set PX4_PORTS.")
if not os.access(ports[0], os.R_OK | os.W_OK):
    sys.exit(f"No read/write access to {ports[0]}. Install tools/60-fmuv6x.rules; see README.md.")
print(f"USB device: {ports[0]}", flush=True)
PY
    build_firmware "$mode"
    if [[ "${1:-}" == "--dry-run" ]]; then
        echo "Upload prerequisites and firmware package checked; dry run complete."
        exit 0
    fi
    # The uploader sends MAVLink reboot-to-bootloader (parameter 1 = 3),
    # rescans Rust/PX4 application and bootloader identities, uploads,
    # verifies the CRC, and reboots.
    exec "$python" -u "$uploader" --port "$ports" --baud-flightstack 115200 \
        --verbose --noninteractive "$release/v6x-rust-led.px4"
}

case "$action" in
    build) build_firmware ;;
    build-injection) build_firmware injection ;;
    build-dump) build_firmware dump ;;
    upload) upload_firmware normal "$@" ;;
    upload-dump) upload_firmware dump "$@" ;;
    monitor)
        exec "$python" -u "$root/tools/check_mavlink.py" --monitor "${1:-10}"
        ;;
    ports)
        exec "$python" -u "$root/tools/check_mavlink.py" --list-ports
        ;;
    check)
        cargo fmt --check
        cargo clippy --release --locked -- -D warnings
        cargo clippy --release --locked --features fault-injection -- -D warnings
        cargo clippy --release --locked --features fault-dump -- -D warnings
        host="$(rustc -vV | sed -n 's/^host: //p')"
        cargo test --lib --target "$host" --locked
        cargo clippy --lib --target "$host" --locked -- -D warnings
        "$python" -m unittest discover -s tools -p 'test_*.py'
        build_firmware
        build_firmware dump
        ;;
esac
