#!/usr/bin/env python3
"""Verify H-04 recovery: heartbeat, fault text, log list, newest ULog download and parse.

Read-only apart from LOG_REQUEST_DATA/END. Never writes parameters, never arms.
Used after controlled power loss, missing-card boot, or card removal.
"""

import argparse
import hashlib
import json
import struct
import sys
import time
from contextlib import closing
from datetime import datetime, timezone
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from check_mavlink import console_command, receive, request, select_port
from pymavlink import mavutil
from pyulog import ULog

CHUNK = 8192
ULOG_MAGIC = b"\x55\x4c\x6f\x67"


def logger_status(link, timeout=5):
    text = console_command(link, "logger status", timeout=timeout)
    lines = [line for line in text.splitlines() if line.strip() and line != "END"]
    fields = {"raw": text}
    for line in lines:
        if not line.startswith("LOGGER "):
            continue
        for item in line.split()[1:]:
            if "=" in item:
                key, value = item.split("=", 1)
                fields[key] = value
    return fields


def list_logs(link):
    link.mav.log_request_list_send(1, 1, 0, 0xFFFF)
    first = receive(link, "LOG_ENTRY", timeout=10)
    if first.num_logs == 0:
        return []
    entries = [first.to_dict()]
    while len(entries) < first.num_logs:
        entries.append(receive(link, "LOG_ENTRY", timeout=10).to_dict())
    return sorted(entries, key=lambda entry: entry["id"])


def download(link, entry):
    output = bytearray()
    size = entry["size"]
    while len(output) < size:
        wanted = min(CHUNK, size - len(output))
        link.mav.log_request_data_send(1, 1, entry["id"], len(output), wanted)
        got = 0
        while got < wanted:
            message = receive(link, "LOG_DATA", timeout=20)
            assert message.id == entry["id"], "LOG_DATA id mismatch"
            assert message.ofs == len(output), (
                f"LOG_DATA offset {message.ofs} != {len(output)}"
            )
            assert 0 < message.count <= 90
            output.extend(bytes(message.data[: message.count]))
            got += message.count
    link.mav.log_request_end_send(1, 1)
    return bytes(output)


def parameter_snapshot(link):
    link.mav.param_request_list_send(1, 1)
    first = receive(link, "PARAM_VALUE")
    values = {}
    for index in range(first.param_count):
        message = first if index == 0 else receive(link, "PARAM_VALUE")
        if message.param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
            value = struct.unpack("<i", struct.pack("<f", message.param_value))[0]
        else:
            value = round(message.param_value, 9)
        values[message.param_id] = value
    return values


def summarize(data):
    log = ULog(str(Path(data["path"])))
    span_us = (int(log.last_timestamp) - int(log.start_timestamp))
    dropouts = [int(item.duration) for item in log.dropouts]
    records = sum(len(item.data.get("timestamp", [])) for item in log.data_list)
    health = None
    try:
        health = log.get_dataset("rust_runtime_health")
    except KeyError:
        pass
    return {
        "bytes": data["bytes"],
        "sha256": data["sha256"],
        "span_s": round(span_us / 1_000_000, 3),
        "records": records,
        "topics": len(log.data_list),
        "dropouts_ms": dropouts,
        "dropout_count": len(dropouts),
        "dropout_max_ms": max(dropouts, default=0),
        "dropout_total_ms": sum(dropouts),
        "file_corruption": bool(log.file_corruption),
        "logger_normal_dropped": int(health.data["logger_normal_dropped"][-1])
        if health is not None and len(health.data["logger_normal_dropped"])
        else None,
        "logger_event_dropped": int(health.data["logger_event_dropped"][-1])
        if health is not None and len(health.data["logger_event_dropped"])
        else None,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True, help="evidence directory")
    parser.add_argument("--log-id", type=int, help="download this id instead of the newest")
    parser.add_argument("--expect-logs", type=int, help="fail if the list length differs")
    parser.add_argument("--no-download", action="store_true")
    parser.add_argument("--parameters", action="store_true", help="read all parameters")
    parser.add_argument("--port", help="explicit COM port")
    args = parser.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    result = {
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "scope": "H-04 read-only recovery verification",
        "passed": False,
    }
    try:
        port = select_port(args.port)
        result["port"] = port.device
        with closing(
            mavutil.mavlink_connection(
                port.device,
                baud=115200,
                source_system=255,
                source_component=190,
                autoreconnect=False,
            )
        ) as link:
            heartbeat = receive(link, "HEARTBEAT", timeout=10)
            result["armed"] = bool(
                heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            )
            result["system_status"] = int(heartbeat.system_status)
            request(link, 148)
            version = receive(link, "AUTOPILOT_VERSION", timeout=10)
            result["source"] = (
                f"{int.from_bytes(bytes(version.flight_custom_version), 'little'):016x}"
            )
            result["fault"] = console_command(link, "!fault", timeout=5)
            result["logger"] = logger_status(link)

            logs = list_logs(link)
            result["logs"] = logs
            if args.expect_logs is not None:
                assert len(logs) == args.expect_logs, (
                    f"expected {args.expect_logs} logs, got {len(logs)}"
                )

            if not args.no_download and logs:
                entry = next(
                    (item for item in logs if item["id"] == args.log_id), None
                ) if args.log_id is not None else logs[-1]
                assert entry is not None, f"log id {args.log_id} not found"
                data = download(link, entry)
                assert data[:4] == ULOG_MAGIC, "downloaded file is not a ULog"
                path = args.out / f"recover-{entry['id']:03d}.ulg"
                path.write_bytes(data)
                result["ulog"] = summarize(
                    {
                        "path": str(path),
                        "bytes": len(data),
                        "sha256": hashlib.sha256(data).hexdigest(),
                    }
                )
                result["ulog"]["id"] = entry["id"]
                result["ulog"]["listed_size"] = entry["size"]

            if args.parameters:
                result["parameters"] = parameter_snapshot(link)
                result["parameter_count"] = len(result["parameters"])
        result["passed"] = True
    except (OSError, ValueError, RuntimeError, KeyError, AssertionError, TimeoutError) as error:
        result["error"] = str(error) or type(error).__name__
        print(result["error"], file=sys.stderr)
    finally:
        result["finished_utc"] = datetime.now(timezone.utc).isoformat()
        target = args.out / "verify.json"
        with target.open("w", encoding="utf-8") as output:
            json.dump(result, output, indent=2, ensure_ascii=False)
            output.write("\n")
    printable = {key: value for key, value in result.items() if key != "parameters"}
    print(json.dumps(printable, indent=2, ensure_ascii=False))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
