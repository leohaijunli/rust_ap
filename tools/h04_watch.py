#!/usr/bin/env python3
"""Watch heartbeat and logger status while an engineer pulls the SD card.

Read-only: polls HEARTBEAT and the `logger status` console command. It never
starts/stops the logger. Use it alongside a logger that the engineer or another
script already started. Records heartbeat gaps and time_boot_ms resets so a
panic/reboot during card removal is visible.
"""

import argparse
import json
import sys
import time
from contextlib import closing
from datetime import datetime, timezone
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from check_mavlink import console_command, receive, select_port
from pymavlink import mavutil


def logger_fields(text):
    for line in text.splitlines():
        if line.startswith("LOGGER "):
            fields = {}
            for item in line.split()[1:]:
                if "=" in item:
                    key, value = item.split("=", 1)
                    fields[key] = value
            return fields
    return {"raw": text}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True, help="evidence directory")
    parser.add_argument("--seconds", type=float, default=45.0)
    parser.add_argument("--status-period", type=float, default=1.0)
    parser.add_argument("--port")
    args = parser.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    result = {
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "scope": "watch heartbeat and logger status during card removal",
        "samples": [],
        "events": [],
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
            receive(link, "HEARTBEAT", timeout=10)
            result["logger_before"] = logger_fields(
                console_command(link, "logger status", timeout=5)
            )
            deadline = time.monotonic() + args.seconds
            next_status = time.monotonic()
            last_beat = None
            seen_boot = None
            while time.monotonic() < deadline:
                message = link.recv_match(
                    type=["HEARTBEAT", "SYSTEM_TIME"], blocking=True, timeout=0.5
                )
                now = time.monotonic()
                if message is not None and message.get_type() == "SYSTEM_TIME":
                    boot_ms = int(message.time_boot_ms)
                    if seen_boot is not None and boot_ms + 1000 < seen_boot:
                        result["events"].append(
                            {
                                "t": round(now, 3),
                                "event": "boot_ms_reset",
                                "boot_ms": boot_ms,
                                "previous": seen_boot,
                            }
                        )
                    seen_boot = boot_ms
                    result["samples"].append(
                        {"t": round(now, 3), "boot_ms": boot_ms}
                    )
                elif message is not None:
                    sample = {
                        "t": round(now, 3),
                        "armed": bool(
                            message.base_mode
                            & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                        ),
                        "system_status": int(message.system_status),
                    }
                    if last_beat is not None and sample["t"] - last_beat > 2.0:
                        result["events"].append(
                            {"t": sample["t"], "event": "heartbeat_gap",
                             "gap_s": round(sample["t"] - last_beat, 3)}
                        )
                    last_beat = sample["t"]
                    result["samples"].append(sample)
                if now >= next_status:
                    try:
                        status = logger_fields(
                            console_command(link, "logger status", timeout=5)
                        )
                    except TimeoutError as error:
                        status = {"error": str(error)}
                        result["events"].append(
                            {"t": round(now, 3), "event": "status_timeout"}
                        )
                    status["t"] = round(now, 3)
                    result["status"] = result.get("status", [])
                    if result["status"] and any(
                        key in status
                        for key in ("status", "stage", "errors")
                    ):
                        previous = result["status"][-1]
                        for key in ("status", "stage", "errors", "event_dropped"):
                            if status.get(key) != previous.get(key):
                                result["events"].append(
                                    {
                                        "t": round(now, 3),
                                        "event": f"logger_{key}",
                                        "from": previous.get(key),
                                        "to": status.get(key),
                                    }
                                )
                    result["status"].append(status)
                    next_status = now + args.status_period
            result["logger_after"] = logger_fields(
                console_command(link, "logger status", timeout=5)
            )
            try:
                result["fault"] = console_command(link, "!fault", timeout=5)
            except TimeoutError as error:
                result["fault"] = f"timeout: {error}"
        result["passed"] = len(result["samples"]) > 0
    except (OSError, ValueError, RuntimeError, KeyError, AssertionError, TimeoutError) as error:
        result["error"] = str(error) or type(error).__name__
        print(result["error"], file=sys.stderr)
    finally:
        result["finished_utc"] = datetime.now(timezone.utc).isoformat()
        target = args.out / "watch.json"
        with target.open("w", encoding="utf-8") as output:
            json.dump(result, output, indent=2, ensure_ascii=False)
            output.write("\n")
    print(
        json.dumps(
            {
                key: value
                for key, value in result.items()
                if key not in {"samples", "status"}
            },
            indent=2,
            ensure_ascii=False,
        )
    )
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
