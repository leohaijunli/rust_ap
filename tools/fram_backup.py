#!/usr/bin/env python3
"""Read and verify all FM25V02A bytes through the Rust USB console."""

import argparse
import hashlib
import json
import os
import time
import zlib
from datetime import datetime, timezone
from pathlib import Path

import serial

from check_mavlink import find_port
from p1_capture import Capture, PREFIXES

PREFIXES.update({"FRAM", "FRAM_DATA"})
CAPACITY = 32768
CHUNK = 256
DEVICE_ID = "7f7f7f7f7f7fc22208"


def read_chunk(capture, address):
    record = capture.request(f"!fram-read {address:04x}")["fields"]["FRAM_DATA"]
    data = bytes.fromhex(record["hex"])
    if int(record["addr"], 16) != address or int(record["len"]) != CHUNK or len(data) != CHUNK:
        raise ValueError(f"Invalid FRAM block at {address:04x}")
    return data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port")
    parser.add_argument("--source-id", required=True)
    parser.add_argument("--output", type=Path, required=True, help="New evidence directory")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    result = {"schema": 1, "started_utc": datetime.now(timezone.utc).isoformat(),
              "outcome": "incomplete", "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    try:
        device = args.port or find_port()
        result["device"] = device
        with serial.Serial(device, 115200, timeout=0.005, write_timeout=0.5, exclusive=True) as port, (args.output / "wire.jsonl").open("x") as raw:
            capture = Capture(port, raw, 0)
            until = time.monotonic() + 2
            while time.monotonic() < until:
                capture.poll()
            capture.write(b"\n")
            result["platform"] = capture.request("!platform")
            platform = result["platform"]["fields"]["P1_PLATFORM"]
            if platform["source"] != args.source_id or platform["fault_injection"] != "false":
                raise ValueError("Unexpected firmware identity")
            result["before"] = capture.request("!fram")
            info = result["before"]["fields"]["FRAM"]
            if info["id"] != DEVICE_ID or info["supported"] != "true" or int(info["bytes"]) != CAPACITY:
                raise ValueError("Unrecognized FRAM; capacity not assumed")
            image = bytearray()
            with (args.output / "fram.bin").open("xb") as backup:
                for address in range(0, CAPACITY, CHUNK):
                    data = read_chunk(capture, address)
                    backup.write(data)
                    image.extend(data)
                backup.flush()
                os.fsync(backup.fileno())
            if (args.output / "fram.bin").read_bytes() != image:
                raise ValueError("Saved backup differs from capture")
            for address in range(0, CAPACITY, CHUNK):
                if read_chunk(capture, address) != image[address:address + CHUNK]:
                    raise ValueError(f"Verification differs at {address:04x}")
            result["after"] = capture.request("!fram")
            if result["after"]["fields"]["FRAM"] != info:
                raise ValueError("FRAM identity/status changed")
            result.update(bytes=len(image), sha256=hashlib.sha256(image).hexdigest(),
                          crc32=f"{zlib.crc32(image):08x}", verified_blocks=CAPACITY // CHUNK,
                          bad_mavlink_frames=capture.bad_frames, outcome="pass")
    except (OSError, ValueError, RuntimeError, KeyError) as error:
        result.update(outcome="failed", error=f"{type(error).__name__}: {error}")
    finally:
        result["finished_utc"] = datetime.now(timezone.utc).isoformat()
        with (args.output / "result.json").open("x") as output:
            json.dump(result, output, indent=2)
            output.write("\n")
            output.flush()
            os.fsync(output.fileno())
    print(json.dumps({k: v for k, v in result.items() if k not in ("platform", "before", "after")}, indent=2))
    return 0 if result["outcome"] == "pass" else 1


if __name__ == "__main__":
    raise SystemExit(main())
