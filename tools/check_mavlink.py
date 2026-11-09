#!/usr/bin/env python3
"""Check FMUv6X Rust MAVLink or monitor normal/fault-dump USB firmware."""

import argparse
import hashlib
import json
import math
import os
import sys
import time
import tomllib
from datetime import datetime, timezone
from pathlib import Path

import serial.tools.list_ports
from pymavlink import mavutil
from pymavlink.dialects.v10 import common as mav1
from pymavlink.dialects.v20 import common as mav2

import decode_fault_dump

DEFAULT_WAIT = 10.0
# 2020-01-01T00:00:00Z; earlier RTC dates are reported as "unknown" by design.
MIN_PLAUSIBLE_UNIX_US = 1_577_836_800 * 1_000_000


class PortNotReady(RuntimeError):
    """No usable application port yet; USB may be re-enumerating."""


def port_mode(port):
    if (port.vid, port.pid) == (0x3185, 0x0035):
        return "bootloader"
    if (port.vid, port.pid) == (0x1209, 0x0001):
        return {"V6X-RUST-001": "mavlink", "V6X-RUST-DUMP": "dump"}.get(port.serial_number)
    return None


def describe_port(port):
    usb = f"{port.vid:04x}:{port.pid:04x}" if port.vid is not None and port.pid is not None else "no USB ID"
    return f"{port.device} ({port.product or port.description}, {usb}, serial={port.serial_number})"


def describe_ports(ports):
    usb = [port for port in ports if port.vid is not None]
    descriptions = [describe_port(port) for port in usb]
    if len(usb) != len(ports):
        descriptions.append(f"{len(ports) - len(usb)} other serial entries without USB identity")
    return "; ".join(descriptions) or "none"


def select_port(device=None, *, allow_dump=False, on_scan=None):
    """Resolve USB identity before opening a port; explicit adapters may use MAVLink."""
    ports = sorted(serial.tools.list_ports.comports(), key=lambda port: port.device)
    if on_scan is not None:
        on_scan(ports)
    observed = describe_ports(ports)
    if device is not None:
        matches = [port for port in ports if os.path.realpath(port.device) == os.path.realpath(device)]
        if not matches:
            raise PortNotReady(f"Serial port {device} not found. Detected: {observed}")
        selected = matches[0]
    else:
        modes = {"mavlink", "dump"} if allow_dump else {"mavlink"}
        matches = [port for port in ports if port_mode(port) in modes]
        if len(matches) > 1:
            raise RuntimeError(f"Multiple Rust CDC devices; select one with --port. Detected: {observed}")
        if not matches:
            hint = "Check the board's USB attachment."
            if any(port_mode(port) == "dump" for port in ports):
                hint = "V6X Rust Dump provides diagnostic text only; use check_mavlink.py --monitor 10."
            elif any(port_mode(port) == "bootloader" for port in ports):
                hint = "PX4 bootloader is present; application telemetry is unavailable."
            raise PortNotReady(f"No matching Rust CDC device. {hint} Detected: {observed}")
        selected = matches[0]

    mode = port_mode(selected)
    if mode == "bootloader":
        raise PortNotReady(f"PX4 bootloader has no application telemetry: {describe_port(selected)}")
    if mode == "dump" and not allow_dump:
        raise RuntimeError("V6X Rust Dump provides diagnostic text only; use check_mavlink.py --monitor 10.")
    return selected


def wait_for_port(device=None, *, allow_dump=False, seconds=DEFAULT_WAIT, on_scan=None):
    deadline = None
    while True:
        try:
            return select_port(device, allow_dump=allow_dump, on_scan=on_scan)
        except PortNotReady:
            if seconds == 0:
                raise
            now = time.monotonic()
            if deadline is None:
                deadline = now + seconds
                print(f"Waiting up to {seconds:g}s for Rust CDC to enumerate...", flush=True)
            remaining = deadline - now
            if remaining <= 0:
                raise
            time.sleep(min(0.25, remaining))


def find_port():
    """Normal Rust CDC only: p1_capture sends commands unavailable in Dump."""
    return select_port().device


def receive(link, kind, predicate=lambda message: True, timeout=6):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        message = link.recv_match(type=kind, blocking=True, timeout=0.2)
        if message is not None and predicate(message):
            assert (message.get_srcSystem(), message.get_srcComponent()) == (1, 1)
            return message
    raise TimeoutError(f"No matching {kind} received")


def request(link, message_id):
    link.mav.command_long_send(
        1, 1, mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
        message_id, 0, 0, 0, 0, 0, 0,
    )


def console_command(link, command, timeout=3):
    payload = (command + "\n").encode()
    if len(payload) > 70:
        raise ValueError("MAVLink console command exceeds 70 bytes")
    link.mav.serial_control_send(
        mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL,
        mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND,
        0,
        0,
        len(payload),
        payload + bytes(70 - len(payload)),
    )
    output = bytearray()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        message = link.recv_match(type="SERIAL_CONTROL", blocking=True, timeout=0.2)
        if message is None or message.device != mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL:
            continue
        output.extend(bytes(message.data[:message.count]))
        if not message.flags & mavutil.mavlink.SERIAL_CONTROL_FLAG_MULTI:
            return output.decode("utf-8")
    raise TimeoutError("No complete MAVLink console response received")


def expected_source_id(expected_version, source_id, report):
    if source_id is not None:
        if len(source_id) != 16 or any(c not in "0123456789abcdefABCDEF" for c in source_id):
            raise ValueError("source-id must contain exactly 16 hexadecimal digits")
        return source_id.lower()
    if expected_version == "0.1.0":
        # The preserved recovery baseline predates the source-ID wire format.
        return f"{int.from_bytes(b'RUSTFMU1', 'little'):016x}"
    if report.exists():
        data = json.loads(report.read_text())
        if data["version"] == expected_version:
            return expected_source_id(expected_version, data["source_id"], report)
    return None


def check(link, expected_version, source_id=None):
    link.port.reset_input_buffer()
    heartbeat = receive(link, "HEARTBEAT")
    assert heartbeat.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4
    assert not heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    timestamps = [time.monotonic()]
    for _ in range(4):
        receive(link, "HEARTBEAT")
        timestamps.append(time.monotonic())
    period = (timestamps[-1] - timestamps[0]) / 4
    assert 0.8 < period < 1.2, f"Heartbeat period is {period:.3f}s"
    print(f"HEARTBEAT: system=1 component=1, PX4-compatible, mean period={period:.3f}s")

    request(link, 253)
    text = receive(link, "STATUSTEXT")
    assert text.text.startswith(f"Rust FMUv6X v{expected_version} running"), text
    ack = receive(link, "COMMAND_ACK", lambda msg: msg.command == 512)
    assert ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
    print(f"STATUSTEXT: {text.text}")

    request(link, 148)
    version = receive(link, "AUTOPILOT_VERSION")
    major, minor, patch = map(int, expected_version.split("."))
    assert version.flight_sw_version == major << 24 | minor << 16 | patch << 8
    actual_source = int.from_bytes(bytes(version.flight_custom_version), "little")
    if source_id is not None:
        assert actual_source == int(source_id, 16), f"Firmware source mismatch: {actual_source:016x}"
    else:
        assert actual_source != 0
    assert version.uid != 0
    ack = receive(link, "COMMAND_ACK", lambda msg: msg.command == 512)
    assert ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
    print(f"AUTOPILOT_VERSION: Rust {expected_version}, source={actual_source:016x}, MCU UID={version.uid:016x}")

    link.mav.param_request_list_send(1, 1)
    parameters = {}
    message = receive(link, "PARAM_VALUE")
    count = message.param_count
    assert count > 0
    indices = set()
    for index in range(count):
        if index:
            message = receive(link, "PARAM_VALUE")
        assert message.param_count == count and 0 <= message.param_index < count
        assert message.param_index not in indices and message.param_id not in parameters
        assert math.isfinite(message.param_value)
        indices.add(message.param_index)
        parameters[message.param_id] = message.param_value
    assert parameters["MAV_SYS_ID"] == 1.0, parameters
    assert 0.1 <= parameters["LINK_LED_HZ"] <= 10.0, parameters
    print(f"Parameter discovery: {parameters}")
    link.mav.param_request_read_send(1, 1, b"LINK_LED_HZ", -1)
    message = receive(link, "PARAM_VALUE", lambda msg: msg.param_id == "LINK_LED_HZ")
    assert message.param_value == parameters["LINK_LED_HZ"]

    request(link, 999999)
    ack = receive(link, "COMMAND_ACK", lambda msg: msg.command == 512)
    assert ack.result == mavutil.mavlink.MAV_RESULT_UNSUPPORTED
    print("Unsupported read request: correctly acknowledged")

    for sequence, dialect in enumerate((mav1, mav2), start=10):
        encoder = dialect.MAVLink(None, srcSystem=255, srcComponent=190)
        frame = dialect.MAVLink_ping_message(123456789, sequence, 0, 0).pack(encoder)
        for chunk in (frame[:2], frame[2:7], frame[7:]):
            link.port.write(chunk)
            link.port.flush()
            time.sleep(0.02)
        response = receive(link, "PING", lambda msg: msg.seq == sequence)
        assert response.time_usec == 123456789
        assert (response.target_system, response.target_component) == (255, 190)
    print("Fragmented MAVLink v1 and v2 requests: PASS")

    encoder = mav2.MAVLink(None, srcSystem=255, srcComponent=190)
    bad = bytearray(mav2.MAVLink_ping_message(0, 90, 0, 0).pack(encoder))
    bad[-1] ^= 0x80
    good = mav2.MAVLink_ping_message(0, 91, 0, 0).pack(encoder)
    link.port.write(bad + good)
    link.port.flush()
    while True:
        response = receive(link, "PING")
        assert response.seq != 90, "CRC-corrupted request was accepted"
        if response.seq == 91:
            break
    print("Corrupted CRC discarded; next valid frame decoded: PASS")

    partial = mav2.MAVLink_ping_message(0, 92, 0, 0).pack(encoder)
    link.port.write(partial[:7])
    link.port.flush()
    time.sleep(1.2)
    fresh = mav2.MAVLink_ping_message(0, 93, 0, 0).pack(encoder)
    link.port.write(fresh)
    link.port.flush()
    receive(link, "PING", lambda msg: msg.seq == 93)
    print("Incomplete old frame expired; fresh request decoded: PASS")

    link.mav.timesync_send(0, 456789)
    sync = receive(link, "TIMESYNC", lambda msg: msg.ts1 == 456789)
    assert sync.tc1 > 0
    first = receive(link, "SYSTEM_TIME")
    second = receive(link, "SYSTEM_TIME")
    assert second.time_boot_ms > first.time_boot_ms
    print(f"MCU boot time advances: {first.time_boot_ms} -> {second.time_boot_ms} ms")
    # The RTC may still hold a pre-2020 date when no GNSS/GCS time has arrived,
    # so 0 is valid; anything else must look like a UNIX epoch stamp.
    for message in (first, second):
        assert message.time_unix_usec == 0 or message.time_unix_usec >= MIN_PLAUSIBLE_UNIX_US, message
    print(f"SYSTEM_TIME.time_unix_usec={first.time_unix_usec} us")

    request(link, 105)
    imu = receive(link, "HIGHRES_IMU")
    required = 0x103F  # accel XYZ, gyro XYZ and temperature
    assert imu.fields_updated & required == required
    assert imu.id == 0
    values = (imu.xacc, imu.yacc, imu.zacc, imu.xgyro, imu.ygyro, imu.zgyro, imu.temperature)
    assert all(math.isfinite(value) for value in values)
    assert imu.time_usec > 0
    print(
        "HIGHRES_IMU: "
        f"accel=({imu.xacc:.3f},{imu.yacc:.3f},{imu.zacc:.3f})m/s^2 "
        f"gyro=({imu.xgyro:.4f},{imu.ygyro:.4f},{imu.zgyro:.4f})rad/s "
        f"temperature={imu.temperature:.2f}C"
    )
    logger = console_command(link, "logger status")
    assert "LOGGER schema=2" in logger and "sync_period_ms=1000" in logger, logger
    print("MAVLink console logger status: PASS")
    rtc = console_command(link, "!time")
    assert "RTC schema=3" in rtc and "ticking=true" in rtc, rtc
    assert "counting=true" in rtc and "prescaler_ok=true" in rtc, rtc
    print(f"MAVLink console RTC status: {rtc.strip()}")
    print("MAVLink wire checks: PASS")


def monitor(link, seconds):
    link.port.reset_input_buffer()
    request(link, 253)
    deadline = time.monotonic() + seconds
    received = 0
    while time.monotonic() < deadline:
        message = link.recv_match(
            type=["HEARTBEAT", "STATUSTEXT", "SYSTEM_TIME", "HIGHRES_IMU", "NAMED_VALUE_FLOAT", "DEBUG_VECT", "COMMAND_ACK"],
            blocking=True, timeout=0.2,
        )
        if message is not None:
            received += 1
            print(f"sys={message.get_srcSystem()} comp={message.get_srcComponent()} {message}")
    if not received:
        raise TimeoutError(f"No MAVLink telemetry received in {seconds:g}s")


def monitor_dump(port, seconds, raw):
    print(f"V6X Rust Dump: capturing diagnostic text for {seconds:g}s (no MAVLink/QGC heartbeat)", flush=True)
    raw.parent.mkdir(parents=True, exist_ok=True)
    # Preserve exact bytes, including partial captures if USB disconnects.
    with raw.open("xb") as output:
        print(f"Raw dump: {raw}", flush=True)
        text = decode_fault_dump.read_port(port, seconds, raw=output)
        byte_count = output.tell()
    print(f"Captured {byte_count} bytes", flush=True)
    if byte_count == 0:
        raise TimeoutError(f"Dump CDC opened but received 0 bytes in {seconds:g}s")
    result = decode_fault_dump.decode_text(text)
    decode_fault_dump.print_report(result)
    if not result["complete"]:
        raise RuntimeError("Incomplete V6X_DUMP report; capture again with --monitor 10 or a longer duration")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="Serial port or by-id path; defaults to one normal or Dump Rust CDC device")
    parser.add_argument("--wait", type=float, default=DEFAULT_WAIT, metavar="SECONDS", help="Wait for USB enumeration before capture/checks (default: 10; 0: one scan)")
    parser.add_argument("--list-ports", action="store_true", help="Print USB serial identities and save diagnostics without opening a port")
    parser.add_argument("--monitor", type=float, metavar="SECONDS", help="Monitor MAVLink or capture/decode Dump text for this duration")
    parser.add_argument("--raw", type=Path, help="New Dump capture file; default: target/fault-dumps/<timestamp>.txt (honors CARGO_TARGET_DIR)")
    root = Path(__file__).resolve().parents[1]
    parser.add_argument("--expected-version", default=tomllib.loads((root / "Cargo.toml").read_text())["package"]["version"])
    parser.add_argument("--source-id", help="Expected hexadecimal !platform source ID")
    args = parser.parse_args(argv)
    if args.monitor is not None and (not math.isfinite(args.monitor) or args.monitor <= 0):
        parser.error("--monitor must be a positive, finite number of seconds")
    if not math.isfinite(args.wait) or args.wait < 0:
        parser.error("--wait must be a nonnegative, finite number of seconds")
    if args.list_ports and (args.port is not None or args.monitor is not None or args.raw is not None):
        parser.error("--list-ports cannot be combined with --port, --monitor or --raw")
    if args.raw and args.monitor is None:
        parser.error("--raw requires --monitor and a Dump device")
    target = Path(os.environ.get("CARGO_TARGET_DIR", str(root / "target")))
    started = datetime.now(timezone.utc)
    run_name = started.strftime("%Y%m%dT%H%M%S.%fZ")
    script = Path(__file__).resolve()
    diagnostics = {
        "schema": 1, "started_utc": started.isoformat(),
        "script": str(script), "script_sha256": hashlib.sha256(script.read_bytes()).hexdigest(),
        "python": sys.executable, "arguments": vars(args), "scans": [], "outcome": "incomplete",
    }

    def record_scan(ports):
        entries = [{"device": port.device, "vid": port.vid, "pid": port.pid,
                    "serial": port.serial_number, "product": port.product, "mode": port_mode(port)}
                   for port in ports]
        scans = diagnostics["scans"]
        if not scans or scans[-1]["ports"] != entries:
            scans.append({"at_utc": datetime.now(timezone.utc).isoformat(), "ports": entries})

    print(f"Tool: {script} sha256={diagnostics['script_sha256'][:12]}", flush=True)
    try:
        if args.list_ports:
            ports = sorted(serial.tools.list_ports.comports(), key=lambda port: port.device)
            record_scan(ports)
            print(f"Python: {sys.executable}")
            print(f"Detected: {describe_ports(ports)}")
            for port in ports:
                if port_mode(port) is not None:
                    print(f"mode={port_mode(port)} {port.device}")
            diagnostics["outcome"] = "listed"
            return 0
        port = wait_for_port(args.port, allow_dump=True, seconds=args.wait, on_scan=record_scan)
        diagnostics["selected_device"] = port.device
        diagnostics["selected_mode"] = port_mode(port) or "mavlink"
        print(f"Device: {describe_port(port)}", flush=True)
        if port_mode(port) == "dump":
            if args.monitor is None:
                raise RuntimeError("V6X Rust Dump has no MAVLink; use --monitor 10 to capture/decode diagnostics")
            raw = args.raw or target / "fault-dumps" / f"{run_name}.txt"
            diagnostics["dump_raw"] = str(raw)
            monitor_dump(port.device, args.monitor, raw)
            diagnostics["outcome"] = "ok"
            return 0
        if args.raw:
            diagnostics.update(outcome="invalid-arguments", error="--raw is only supported for a Dump device")
            parser.error("--raw is only supported for a Dump device")
        if args.monitor is None:
            report = target / "thumbv7em-none-eabihf/release/v6x-rust-ap.build.json"
            args.source_id = expected_source_id(args.expected_version, args.source_id, report)
        link = mavutil.mavlink_connection(
            port.device, baud=115200, source_system=255, source_component=190,
            autoreconnect=False,
        )
        try:
            if args.monitor is None:
                check(link, args.expected_version, args.source_id)
            else:
                monitor(link, args.monitor)
        finally:
            link.close()
        diagnostics["outcome"] = "ok"
    except (OSError, RuntimeError, ValueError, KeyError, AssertionError) as error:
        diagnostics.update(outcome="error", error=str(error) or "MAVLink check failed")
        print(f"Error: {error or 'MAVLink check failed'}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        diagnostics.update(outcome="interrupted", error="KeyboardInterrupt")
        print("Interrupted", file=sys.stderr)
        return 130
    finally:
        diagnostics["finished_utc"] = datetime.now(timezone.utc).isoformat()
        log = target / "monitor-logs" / f"{run_name}.json"
        try:
            log.parent.mkdir(parents=True, exist_ok=True)
            with log.open("x", encoding="utf-8") as output:
                json.dump(diagnostics, output, indent=2, default=str)
                output.write("\n")
            print(f"Diagnostics: {log}", flush=True)
        except OSError as error:
            print(f"Cannot save diagnostics to {log}: {error}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
