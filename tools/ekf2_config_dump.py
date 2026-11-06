#!/usr/bin/env python3
"""Read the board parameter snapshot (read-only) to diagnose EKF2 configuration faults."""

import argparse
import json
import struct
from pathlib import Path
import time

from pymavlink import mavutil

from check_mavlink import find_port


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="Serial port; defaults to the single Rust CDC device")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=30)
    args = parser.parse_args()
    link = mavutil.mavlink_connection(
        args.port or find_port(), baud=115200, source_system=255, source_component=190,
        autoreconnect=False,
    )
    params = {}
    indices = set()
    count = None
    retry_count = 0
    try:
        link.mav.param_request_list_send(1, 1)
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline and (count is None or len(params) < count):
            message = link.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if message is None:
                # P0 makes PARAM_REQUEST_LIST stream the whole registry, so a
                # ground station should never need per-index re-requests. If the
                # stream stalls we re-send the list rather than chip away at it.
                retry_count += 1
                if retry_count > 4:
                    missing = sorted(set(range(count)) - indices) if count else []
                    raise ValueError(
                        f"stream stalled; received {len(params)} of {count} "
                        f"missing_indices={missing}"
                    )
                link.mav.param_request_list_send(1, 1)
                continue
            count = message.param_count
            indices.add(message.param_index)
            # MAVLink carries INT32 parameters as the raw bit pattern; restore
            # the integer value so the dump reads like the registry does.
            if message.param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
                params[message.param_id] = (
                    struct.unpack("<i", struct.pack("<f", message.param_value))[0]
                )
            else:
                params[message.param_id] = message.param_value
        if count is None or len(params) != count:
            raise ValueError(f"received {len(params)} of {count} parameters")
    finally:
        link.close()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(params, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"parameters={len(params)} ekf2={sum(name.startswith('EKF2_') for name in params)}"
          f" single_list_stall_retries={retry_count}")
    print(f"Evidence: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
