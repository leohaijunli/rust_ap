#!/usr/bin/env python3
"""Verify the FMUv6X Rust firmware using an independent MAVLink decoder."""

import argparse
import time

import serial.tools.list_ports
from pymavlink import mavutil
from pymavlink.dialects.v10 import common as mav1
from pymavlink.dialects.v20 import common as mav2


def find_port():
    ports = [
        port.device
        for port in serial.tools.list_ports.comports()
        if (port.vid, port.pid, port.serial_number)
        == (0x1209, 0x0001, "V6X-RUST-001")
    ]
    if len(ports) != 1:
        raise RuntimeError(f"Expected one Rust CDC device, found {ports}")
    return ports[0]


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


def check(link):
    link.port.reset_input_buffer()
    heartbeat = receive(link, "HEARTBEAT")
    assert heartbeat.autopilot == mavutil.mavlink.MAV_AUTOPILOT_GENERIC
    assert not heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    timestamps = [time.monotonic()]
    for _ in range(4):
        receive(link, "HEARTBEAT")
        timestamps.append(time.monotonic())
    period = (timestamps[-1] - timestamps[0]) / 4
    assert 0.8 < period < 1.2, f"Heartbeat period is {period:.3f}s"
    print(f"HEARTBEAT: system=1 component=1, Generic, mean period={period:.3f}s")

    request(link, 253)
    text = receive(link, "STATUSTEXT")
    assert text.text.startswith("Rust FMUv6X v0.1.0 running"), text
    ack = receive(link, "COMMAND_ACK", lambda msg: msg.command == 512)
    assert ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
    print(f"STATUSTEXT: {text.text}")

    request(link, 148)
    version = receive(link, "AUTOPILOT_VERSION")
    assert version.flight_sw_version == 1 << 16
    assert bytes(version.flight_custom_version) == b"RUSTFMU1"
    assert version.uid != 0
    ack = receive(link, "COMMAND_ACK", lambda msg: msg.command == 512)
    assert ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
    print(f"AUTOPILOT_VERSION: Rust 0.1.0, MCU UID={version.uid:016x}")

    link.mav.param_request_list_send(1, 1)
    parameters = {}
    for _ in range(2):
        message = receive(link, "PARAM_VALUE")
        assert message.param_count == 2
        parameters[message.param_id] = message.param_value
    assert parameters == {"MAV_SYS_ID": 1.0, "LINK_LED_HZ": 1.0}, parameters
    print(f"Parameter discovery: {parameters}")
    link.mav.param_request_read_send(1, 1, b"LINK_LED_HZ", -1)
    message = receive(link, "PARAM_VALUE", lambda msg: msg.param_id == "LINK_LED_HZ")
    assert message.param_value == 1.0

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
    print("MAVLink wire checks: PASS")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="Serial port; defaults to the Rust CDC device")
    parser.add_argument("--monitor", type=float, metavar="SECONDS", help="Print decoded telemetry for this duration instead of running checks")
    args = parser.parse_args()
    port = args.port or find_port()
    print(f"Checking {port}")
    link = mavutil.mavlink_connection(
        port, baud=115200, source_system=255, source_component=190,
        autoreconnect=False,
    )
    try:
        if args.monitor is None:
            check(link)
        else:
            link.port.reset_input_buffer()
            request(link, 253)
            deadline = time.monotonic() + args.monitor
            while time.monotonic() < deadline:
                message = link.recv_match(
                    type=["HEARTBEAT", "STATUSTEXT", "SYSTEM_TIME", "COMMAND_ACK"],
                    blocking=True, timeout=0.2,
                )
                if message is not None:
                    print(f"sys={message.get_srcSystem()} comp={message.get_srcComponent()} {message}")
    finally:
        link.close()


if __name__ == "__main__":
    main()
