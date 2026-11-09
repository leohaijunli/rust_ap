#!/usr/bin/env python3
"""Qualified H7 upload through the original PX4 bootloader, no fixed COM number."""

import argparse
import hashlib
import importlib.util
import json
import os
import subprocess
import sys
import time
from pathlib import Path

from build_report import Elf32, source_identity, validate
from check_mavlink import receive
from enter_bootloader import PX4_USB_ID, scan_target_port, send_bootloader_command
from pymavlink import mavutil

ROOT = Path(__file__).resolve().parents[1]
UPLOADER_SHA256 = "f05929e1035e693005677e088e4e7085503b2e8ab3d7c8785f7bd84cd48880f8"


def uploader_path():
    paths = [Path(os.environ["PX4_UPLOADER"])] if os.environ.get("PX4_UPLOADER") else [ROOT / ".venv/px4_uploader.py", ROOT / "target/px4_uploader.py"]
    for path in paths:
        if path.is_file():
            if hashlib.sha256(path.read_bytes()).hexdigest() != UPLOADER_SHA256:
                raise ValueError("PX4 uploader does not match the pinned source/hash")
            return path
    raise ValueError("pinned PX4 uploader missing; run Setup Windows tools")


def ensure_disarmed(port):
    link = mavutil.mavlink_connection(port.device, baud=115200, source_system=255, source_component=190, autoreconnect=False)
    try:
        heartbeat = receive(link, "HEARTBEAT")
        if heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            raise ValueError("upload blocked: board is armed")
    finally:
        link.close()


def load_uploader(path):
    spec = importlib.util.spec_from_file_location("pinned_px4_uploader", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def probe_bootloader(port, path):
    """Resolve ambiguous Windows descriptors with the pinned read-only protocol."""
    uploader = load_uploader(path)
    try:
        with uploader.SerialTransport(port.device, timeout=0.5) as transport:
            protocol = uploader.BootloaderProtocol(transport)
            protocol.sync()
            revision = protocol._get_device_info(uploader.DeviceInfo.BL_REV)
            board = protocol._get_device_info(uploader.DeviceInfo.BOARD_ID)
    except uploader.UploadError:
        return False
    if board != 53 or not 2 <= revision <= 6:
        raise ValueError(f"unsupported bootloader: board={board} protocol={revision}")
    print(f"Bootloader confirmed: board={board} protocol={revision}", flush=True)
    return True


def wait_for_px4_usb(seconds=15):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        port, _, _ = scan_target_port()
        if port and (port.vid, port.pid) == PX4_USB_ID:
            return port
        time.sleep(0.25)
    raise ValueError("PX4 USB did not enumerate; uploader was not started")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--release", type=Path, required=True)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    try:
        release = args.release.resolve()
        package = release / "v6x-rust-ap.px4"
        identity, _ = source_identity(ROOT)
        validate(Elf32((release / "v6x-rust-ap").read_bytes()), (release / "v6x-rust-ap.bin").read_bytes(),
                 json.loads(package.read_text(encoding="utf-8")), identity)
        uploader = uploader_path()
        port, kind, _ = scan_target_port()
        if port is None:
            raise ValueError("no FMUv6X USB device")
        if kind == "px4-application" and probe_bootloader(port, uploader):
            kind = "bootloader"
        if kind in ("rust", "px4-application"):
            # A failed bootloader probe still requires a disarmed heartbeat;
            # silence never authorizes an application reboot.
            ensure_disarmed(port)
        print(f"Qualified source={identity}; device={port.device}; mode={kind}")
        if args.dry_run:
            print("Dry run: no reboot, erase or upload")
            return 0
        if kind == "rust":
            send_bootloader_command(port)
            port = wait_for_px4_usb()
        result = subprocess.run([sys.executable, str(uploader), "--port", port.device,
                                 "--baud-flightstack", "115200", "--noninteractive", str(package)], cwd=ROOT, check=False)
        return result.returncode
    except (OSError, ValueError, RuntimeError, TimeoutError) as error:
        print(f"Upload failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
