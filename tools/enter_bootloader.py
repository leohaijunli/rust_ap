#!/usr/bin/env python3
"""Enter the original PX4 bootloader from the normal V6X Rust CDC."""

import argparse
import math
import sys
import time

import serial
import serial.tools.list_ports


RUST_USB_ID = (0x1209, 0x0001)
RUST_SERIAL = "V6X-RUST-001"
PX4_USB_ID = (0x3185, 0x0035)
BOOTLOADER_COMMAND = b"\n!bootloader\n"
DEFAULT_WAIT = 15.0
POLL_INTERVAL = 0.25


class BootloaderTransitionError(RuntimeError):
    """The board could not be prepared for upload."""


def port_kind(port):
    identity = (port.vid, port.pid)
    if identity == RUST_USB_ID and port.serial_number == RUST_SERIAL:
        return "rust"
    if identity == PX4_USB_ID:
        product = (port.product or "").upper()
        return "bootloader" if "PX4 BL" in product else "px4-application"
    return None


def describe_port(port):
    if port.vid is None or port.pid is None:
        identity = "no USB ID"
    else:
        identity = f"{port.vid:04x}:{port.pid:04x}"
    product = port.product or port.description or "unknown product"
    return f"{port.device} ({product}, {identity}, serial={port.serial_number})"


def scan_target_port():
    try:
        ports = sorted(serial.tools.list_ports.comports(), key=lambda port: port.device)
    except (OSError, serial.SerialException) as error:
        raise BootloaderTransitionError(f"Cannot enumerate serial ports: {error}") from error

    targets = []
    for port in ports:
        kind = port_kind(port)
        if kind is not None:
            targets.append((port, kind))
    if len(targets) > 1:
        found = "; ".join(describe_port(port) for port, _ in targets)
        raise BootloaderTransitionError(
            f"Multiple FMUv6X boards or USB ports detected; connect one board only. Found: {found}"
        )
    if targets:
        port, kind = targets[0]
        return port, kind, ports
    return None, None, ports


def send_bootloader_command(port):
    try:
        device = serial.Serial(
            port.device,
            115200,
            timeout=0.25,
            write_timeout=2.0,
            exclusive=True,
        )
    except (OSError, serial.SerialException) as error:
        raise BootloaderTransitionError(
            f"Cannot open Rust CDC {port.device}: {error}. Check read/write permission and stop other serial readers."
        ) from error

    command_sent = False
    try:
        try:
            written = device.write(BOOTLOADER_COMMAND)
        except (OSError, serial.SerialException) as error:
            raise BootloaderTransitionError(
                f"Failed to send the bootloader command to {port.device}: {error}"
            ) from error
        if written != len(BOOTLOADER_COMMAND):
            raise BootloaderTransitionError(
                f"Incomplete bootloader command on {port.device}: wrote {written} of {len(BOOTLOADER_COMMAND)} bytes"
            )
        command_sent = True
        try:
            device.flush()
        except (OSError, serial.SerialException) as error:
            print(
                f"Rust CDC disconnected after the complete command ({error}); waiting for bootloader enumeration.",
                flush=True,
            )
    finally:
        try:
            device.close()
        except (OSError, serial.SerialException) as error:
            if command_sent:
                print(
                    f"Rust CDC closed during reset ({error}); waiting for bootloader enumeration.",
                    flush=True,
                )


def wait_for_bootloader(timeout, poll_interval=POLL_INTERVAL):
    deadline = time.monotonic() + timeout
    last_seen = "no matching USB device"
    while True:
        port, kind, _ = scan_target_port()
        if port is not None:
            last_seen = describe_port(port)
            if kind == "bootloader":
                print(f"PX4 bootloader ready: {last_seen}", flush=True)
                return "bootloader"

        now = time.monotonic()
        if now >= deadline:
            raise BootloaderTransitionError(
                f"Timed out after {timeout:g}s waiting for the original PX4 bootloader "
                "(3185:0035 with product containing 'PX4 BL'). In VMware, configure automatic guest "
                "connection for both 1209:0001 and 3185:0035. Physical unplug/replug is recovery only. "
                f"Last matching device: {last_seen}"
            )
        time.sleep(min(poll_interval, deadline - now))


def prepare_for_upload(timeout=DEFAULT_WAIT, poll_interval=POLL_INTERVAL):
    port, kind, ports = scan_target_port()
    if port is None:
        observed = "; ".join(describe_port(candidate) for candidate in ports) or "none"
        raise BootloaderTransitionError(
            "No normal V6X Rust CDC, PX4 application, or PX4 bootloader found. "
            f"Detected serial ports: {observed}"
        )
    if kind == "bootloader":
        print(f"PX4 bootloader already ready: {describe_port(port)}", flush=True)
        return "bootloader"
    if kind == "px4-application":
        print(
            f"Official PX4 application detected, not bootloader: {describe_port(port)}. "
            "Skipping the Rust CDC command; px4_uploader will use its MAVLink reboot path.",
            flush=True,
        )
        return "px4-application"

    print(f"Normal Rust CDC detected: {describe_port(port)}", flush=True)
    send_bootloader_command(port)
    print("Complete !bootloader command sent; waiting for the original PX4 bootloader...", flush=True)
    return wait_for_bootloader(timeout, poll_interval)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--wait",
        type=float,
        default=DEFAULT_WAIT,
        metavar="SECONDS",
        help=f"wait for PX4 bootloader enumeration (default: {DEFAULT_WAIT:g})",
    )
    args = parser.parse_args(argv)
    if not math.isfinite(args.wait) or args.wait <= 0:
        parser.error("--wait must be a positive, finite number of seconds")
    try:
        prepare_for_upload(args.wait)
    except BootloaderTransitionError as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("Interrupted", file=sys.stderr)
        return 130
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
