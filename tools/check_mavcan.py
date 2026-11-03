#!/usr/bin/env python3
"""Check USB MAVCAN on CAN2: handshake, traffic, GetNodeInfo, and session recovery."""

import argparse
import math
import os
import sys
import time

from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mav2

from check_mavlink import console_command, receive, request, wait_for_port

BUS = 2
FRAME_BUS = BUS - 1
HOST_SYSTEM = 250
HOST_COMPONENT = mav2.MAV_COMP_ID_MAVCAN
EXTENDED_FLAG = 1 << 31
RTR_FLAG = 1 << 30
ID_MASK = 0x1FFFFFFF


def send_forward(link, bus):
    # Match pydronecan's initial broadcast command, before it discovers system 1.
    link.mav.command_long_send(0, 0, mav2.MAV_CMD_CAN_FORWARD, 0, bus, 0, 0, 0, 0, 0, 0)


def set_forward(link, bus):
    send_forward(link, bus)
    ack = receive(link, "COMMAND_ACK", lambda msg: msg.command == mav2.MAV_CMD_CAN_FORWARD)
    if ack.result != mav2.MAV_RESULT_ACCEPTED:
        raise RuntimeError(f"CAN_FORWARD bus={bus} rejected: result={ack.result}")
    if (ack.target_system, ack.target_component) != (HOST_SYSTEM, HOST_COMPONENT):
        raise RuntimeError(f"CAN_FORWARD ACK has the wrong target: {ack}")
    print(f"CAN_FORWARD bus={bus}: ACCEPTED", flush=True)


def diagnostics(link):
    text = console_command(link, "!mavcan")
    print(text.strip(), flush=True)
    lines = text.splitlines()
    if not any(line.startswith("MAVCAN schema=1 ready=true ") for line in lines):
        raise RuntimeError("MAVCAN is not ready; check the firmware source and CAN2 initialization")
    if not any(line.startswith("MAVCAN_HW registers_ok=true ") for line in lines):
        raise RuntimeError("CAN2 clock, mode, filters, or message RAM configuration is incorrect")
    return text


def validate_classic(message):
    if message.get_type() != "CAN_FRAME":
        raise RuntimeError("Classic CAN was sent as CANFD_FRAME; update the MAVCAN firmware")
    if (message.get_srcSystem(), message.get_srcComponent()) != (1, 1):
        raise RuntimeError(f"Unexpected MAVLink source: {message}")
    if (message.target_system, message.target_component) != (HOST_SYSTEM, HOST_COMPONENT):
        raise RuntimeError(f"CAN_FRAME has the wrong forwarding target: {message}")
    if message.bus != FRAME_BUS or not 0 <= message.len <= 8:
        raise RuntimeError(f"CAN_FRAME has an invalid bus or DLC: {message}")
    can_id = message.id & ID_MASK
    if message.id & ~(ID_MASK | EXTENDED_FLAG | RTR_FLAG):
        raise RuntimeError(f"CAN_FRAME contains a reserved ID bit: {message}")
    if not message.id & EXTENDED_FLAG and can_id > 0x7FF:
        raise RuntimeError(f"CAN_FRAME lost the extended-ID flag: {message}")


def node_status_source(message):
    can_id = message.id & ID_MASK
    if (message.id & (EXTENDED_FLAG | RTR_FLAG) == EXTENDED_FLAG
            and not can_id & 0x80 and (can_id >> 8) & 0xFFFF == 341
            and message.len == 8 and message.data[7] & 0xE0 == 0xC0):
        return can_id & 0x7F or None
    return None


def observe(link, seconds, *, renew):
    deadline = time.monotonic() + seconds
    next_renewal = time.monotonic() + 1
    count = 0
    nodes = set()
    while time.monotonic() < deadline:
        if renew and time.monotonic() >= next_renewal:
            send_forward(link, BUS)
            next_renewal = time.monotonic() + 1
        message = link.recv_match(type=["CAN_FRAME", "CANFD_FRAME"], blocking=True, timeout=0.05)
        if message is None:
            continue
        validate_classic(message)
        count += 1
        node = node_status_source(message)
        if node is not None:
            nodes.add(node)
        if count <= 3:
            print(f"CAN2 RX id={message.id:08x} dlc={message.len} "
                  f"data={bytes(message.data[:message.len]).hex()}", flush=True)
    print(f"CAN2 frames={count} NodeStatus nodes={sorted(nodes)}", flush=True)
    return count, nodes


def node_info_request(node_id, local_node_id, transfer_id):
    can_id = (EXTENDED_FLAG | (24 << 24) | (1 << 16) | (1 << 15)
              | (node_id << 8) | 0x80 | local_node_id)
    return can_id, [0xC0 | transfer_id] + [0] * 7


def check_node_info(link, node_id, local_node_id):
    transfer_id = int(time.monotonic() * 1000) & 31
    can_id, payload = node_info_request(node_id, local_node_id, transfer_id)
    link.mav.can_frame_send(0, 0, FRAME_BUS, 1, can_id, payload)
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        message = link.recv_match(type=["CAN_FRAME", "CANFD_FRAME"], blocking=True, timeout=0.05)
        if message is None:
            continue
        validate_classic(message)
        expected = (1 << 16) | (local_node_id << 8) | 0x80 | node_id
        if (message.id & (EXTENDED_FLAG | RTR_FLAG) == EXTENDED_FLAG
                and message.id & 0xFFFFFF == expected and message.len > 0
                and message.data[message.len - 1] & 0xA0 == 0x80
                and message.data[message.len - 1] & 31 == transfer_id):
            print(f"GetNodeInfo node={node_id}: response frame received (CAN2 bidirectional)", flush=True)
            return
    raise TimeoutError(f"No GetNodeInfo response from node {node_id}; inspect tx_frames/tx_errors/ecr")


def check(link, args):
    heartbeat = receive(link, "HEARTBEAT")
    if heartbeat.base_mode & mav2.MAV_MODE_FLAG_SAFETY_ARMED:
        raise RuntimeError("MAVCAN requires disarmed outputs")
    request(link, 148)
    version = receive(link, "AUTOPILOT_VERSION")
    source = int.from_bytes(bytes(version.flight_custom_version), "little")
    print(f"HEARTBEAT disarmed=true source={source:016x}", flush=True)
    if args.source_id is not None and source != int(args.source_id, 16):
        raise RuntimeError(f"Firmware source mismatch: expected {args.source_id}, received {source:016x}")
    diagnostics(link)
    set_forward(link, BUS)
    try:
        link.mav.can_filter_modify_send(0, 0, BUS, mav2.CAN_FILTER_REPLACE, 0, [0] * 16)
        count, nodes = observe(link, args.seconds, renew=True)
        diagnostics(link)
        if count == 0:
            raise RuntimeError("No CAN2 frames forwarded; compare rx_frames/usb_frames and verify node power/bitrate")
        candidates = nodes - {args.local_node_id}
        node_id = args.node_id if args.node_id is not None else min(candidates, default=None)
        if node_id is None:
            raise RuntimeError("No static NodeStatus node found; select a known node with --node-id")
        if args.local_node_id in nodes:
            raise RuntimeError(f"Local node ID {args.local_node_id} is in use; select a free --local-node-id")
        try:
            check_node_info(link, node_id, args.local_node_id)
        except TimeoutError:
            diagnostics(link)
            raise
        if args.check_timeout:
            # Five-second lease with no renewal. Frames during the lease are expected.
            observe(link, 5.2, renew=False)
        else:
            set_forward(link, 0)
        state = diagnostics(link)
        if " active=false " not in state:
            raise RuntimeError("Forwarding session did not close or expire")
        if observe(link, 0.3, renew=False)[0] != 0:
            raise RuntimeError("CAN frames continued after session close/expiry")
        set_forward(link, BUS)
        if observe(link, 1.5, renew=True)[0] == 0:
            raise RuntimeError("CAN2 forwarding did not recover after reopening")
        receive(link, "HEARTBEAT")
    finally:
        set_forward(link, 0)
    print("MAVCAN traffic, bidirectional request, session recovery, and USB heartbeat: PASS", flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="Rust CDC device or COM port (auto-detected by default)")
    parser.add_argument("--seconds", type=float, default=3, help="Initial capture duration (default: 3)")
    parser.add_argument("--node-id", type=int, help="GetNodeInfo target; default: first observed NodeStatus node")
    parser.add_argument("--local-node-id", type=int, default=127, help="Unused DroneCAN request source ID (default: 127)")
    parser.add_argument("--source-id", help="Expected 16-digit firmware source ID")
    parser.add_argument("--check-timeout", action="store_true", help="Check lease expiry instead of explicit close")
    args = parser.parse_args(argv)
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error("--seconds must be positive and finite")
    for node_id in (args.node_id, args.local_node_id):
        if node_id is not None and not 1 <= node_id <= 127:
            parser.error("DroneCAN node IDs must be in 1..127")
    if args.node_id == args.local_node_id:
        parser.error("--node-id and --local-node-id must differ")
    if args.source_id is not None and (len(args.source_id) != 16
            or any(c not in "0123456789abcdefABCDEF" for c in args.source_id)):
        parser.error("--source-id must contain 16 hexadecimal digits")
    try:
        port = wait_for_port(args.port).device
        # mavlink_connection's dialect setup must select MAVLink 2 for ID 386.
        os.environ["MAVLINK20"] = "1"
        link = mavutil.mavlink_connection(port, baud=115200, source_system=HOST_SYSTEM,
                                         source_component=HOST_COMPONENT, dialect="common", autoreconnect=False)
        try:
            check(link, args)
        finally:
            link.close()
    except (OSError, RuntimeError, ValueError, AssertionError) as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
