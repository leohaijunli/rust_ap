#!/usr/bin/env python3
"""P3a parameter acceptance: validated set, explicit save, independent FRAM decode.

`save` performs the one permitted write and records the expected slot/generation.
`verify` is the read-back after the engineer has power-cycled the board.
"""

import argparse
import hashlib
import json
import math
import os
import struct
import time
import zlib
from datetime import datetime, timezone
from pathlib import Path

import serial
from pymavlink.dialects.v20 import common as mav2

from check_mavlink import find_port
from p1_capture import Capture, PREFIXES

PREFIXES.update({"FRAM", "FRAM_DATA", "PARAMS", "PARAM_VALUES", "PARAM_SAVE", "PARAM_LOAD"})
CAPACITY = 32768
CHUNK = 256
DEVICE_ID = "7f7f7f7f7f7fc22208"
START = 0x7800
SLOT_SIZE = 1024
HEADER = 20
MAGIC = b"RPAR"
COMMITTED = 0xA5
FORMAT = 1
TARGET_HZ = 2.0


def decode_slot(data):
    """Return {'generation', 'values', 'count'} or raise ValueError."""
    if len(data) < HEADER + 4:
        raise ValueError("short slot")
    if data[0] != COMMITTED:
        raise ValueError("uncommitted")
    if data[1:5] != MAGIC or data[5] != 0:
        raise ValueError("magic")
    version, schema, generation, count, payload = struct.unpack_from("<HIIHH", data, 6)
    if version != FORMAT:
        raise ValueError(f"version {version}")
    if payload != count * 4 or HEADER + payload + 4 > min(len(data), SLOT_SIZE):
        raise ValueError("length")
    crc = struct.unpack_from("<I", data, HEADER + payload)[0]
    if zlib.crc32(data[1:HEADER + payload]) != crc:
        raise ValueError("crc")
    values = struct.unpack_from(f"<{count}f", data, HEADER)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("values")
    return {"generation": generation, "schema": schema, "values": values, "count": count}


class Link:
    def __init__(self, device, path):
        self.raw = path.open("x", buffering=1)
        try:
            self.port = serial.Serial(device, 115200, timeout=0.005,
                                      write_timeout=0.5, exclusive=True)
        except OSError:
            self.raw.close()
            raise
        self.capture = Capture(self.port, self.raw, 0)
        self.messages = []
        self.capture.decoder.set_callback(self.messages.append)

    def close(self):
        self.port.close()
        self.raw.close()

    def settle(self):
        self.drain(2)
        self.capture.write(b"\n")
        self.drain(0.05)
        heartbeat = self.wait("HEARTBEAT")
        if heartbeat.base_mode & mav2.MAV_MODE_FLAG_SAFETY_ARMED:
            raise ValueError("Expected disarmed firmware")

    def drain(self, seconds=0.5):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            self.capture.poll()

    def command(self, name):
        return self.capture.request(name)["fields"]

    def fram(self):
        info = self.command("!fram")["FRAM"]
        if info["id"] != DEVICE_ID or info["supported"] != "true" or int(info["bytes"]) != CAPACITY:
            raise ValueError("Unexpected FRAM identity")
        return info

    def read_image(self):
        image = bytearray()
        for address in range(0, CAPACITY, CHUNK):
            record = self.command(f"!fram-read {address:04x}")["FRAM_DATA"]
            data = bytes.fromhex(record["hex"])
            if int(record["addr"], 16) != address or int(record["len"]) != CHUNK or len(data) != CHUNK:
                raise ValueError(f"bad block {address:04x}")
            image.extend(data)
        return bytes(image)

    def params(self):
        fields = self.command("!params")
        header = fields["PARAMS"]
        values = {name: float(value) for name, value in fields["PARAM_VALUES"].items()}
        return header, values

    def send(self, message):
        start = len(self.messages)
        self.capture.write(message.pack(self.capture.encoder))
        self.capture.encoder.seq = (self.capture.encoder.seq + 1) & 255
        return start

    def wait(self, kind, predicate=lambda message: True, *, start=0, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for message in self.messages[start:]:
                if message.get_type() == kind and predicate(message):
                    return message
            self.capture.poll()
        raise TimeoutError(f"no {kind}")


def set_param(link, name, value, param_type=mav2.MAV_PARAM_TYPE_REAL32):
    start = link.send(mav2.MAVLink_param_set_message(1, 1, name.encode(), value, param_type))
    echo = link.wait("PARAM_VALUE", lambda message: message.param_id == name, start=start)
    link.drain(0.3)
    warnings = [message.text for message in link.messages[start:]
                if message.get_type() == "STATUSTEXT" and "PARAM_SET" in message.text]
    return float(echo.param_value), warnings


def storage_command(link, action):
    start = link.send(mav2.MAVLink_command_long_message(
        1, 1, mav2.MAV_CMD_PREFLIGHT_STORAGE, 0, action, 0, 0, 0, 0, 0, 0))
    return link.wait("COMMAND_ACK",
                     lambda message: message.command == mav2.MAV_CMD_PREFLIGHT_STORAGE, start=start)


def region_matches_backup(image, backup):
    return len(image) == len(backup) == CAPACITY and image[:START] == backup[:START]


def slot_records(image):
    records = []
    for index in (0, 1):
        slot = image[START + index * SLOT_SIZE:START + (index + 1) * SLOT_SIZE]
        records.append(decode_slot(slot) if slot[0] == COMMITTED else None)
    return records


def validate_record(image, header, values):
    records = slot_records(image)
    active = int(header["slot"])
    if active not in (0, 1) or records[active] is None:
        raise ValueError("Missing active record")
    record = records[active]
    if (record["generation"] != int(header["generation"])
            or record["schema"] != int(header["registry"], 16)
            or record["count"] != len(values)
            or record["values"] != (values["MAV_SYS_ID"], values["LINK_LED_HZ"])):
        raise ValueError("Stored record differs from RAM/status")
    return records, {"slot": active, "generation": record["generation"]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port")
    parser.add_argument("--source-id", required=True)
    parser.add_argument("--backup", type=Path, required=True, help="Pre-write fram.bin")
    parser.add_argument("--output", type=Path, required=True, help="New evidence directory")
    parser.add_argument("--mode", choices=("save", "verify"), required=True)
    parser.add_argument("--save-result", type=Path, help="save result.json for verify mode")
    args = parser.parse_args()
    if args.mode == "verify" and args.save_result is None:
        parser.error("verify requires --save-result")
    args.output.mkdir(parents=True, exist_ok=False)
    result = {"schema": 1, "mode": args.mode, "started_utc": datetime.now(timezone.utc).isoformat(),
              "outcome": "incomplete", "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    link = None
    try:
        backup = args.backup.read_bytes()
        backup_hash = hashlib.sha256(backup).hexdigest()
        if len(backup) != CAPACITY or backup_hash != "600969004d68715044b55828cf5288c7af02d1bfb0c97e437f6480d3e5a45097":
            raise ValueError("Expected the verified pre-write FRAM backup")
        result["backup_sha256"] = backup_hash
        result["source"] = args.source_id
        link = Link(args.port or find_port(), args.output / "wire.jsonl")
        link.settle()
        platform = link.command("!platform")["P1_PLATFORM"]
        if platform["source"] != args.source_id or platform["fault_injection"] != "false":
            raise ValueError("Unexpected firmware identity")
        result["platform"] = platform
        result["fram"] = link.fram()
        header, values = link.params()
        result["params_before"] = {"header": header, "values": values}
        if args.mode == "save":
            before = link.read_image()
            if before != backup:
                raise ValueError("FRAM changed before the first save")
            if header["dirty"] != "true" or values["LINK_LED_HZ"] != 1.0:
                raise ValueError("expected dirty defaults before the first save")
            if header["storage"] != "Defaults" or header["valid"] != "false":
                raise ValueError("expected an empty Rust region before the first save")

            start = link.send(mav2.MAVLink_param_request_list_message(1, 1))
            table = {}
            indices = set()
            for _ in range(len(values)):
                message = link.wait("PARAM_VALUE", lambda message: message.param_id not in table, start=start)
                if message.param_count != len(values) or message.param_index not in range(len(values)) or message.param_index in indices:
                    raise ValueError("Invalid parameter count/index")
                indices.add(message.param_index)
                table[message.param_id] = float(message.param_value)
            if table != values:
                raise ValueError(f"parameter list mismatch: {table}")
            result["parameter_table"] = table

            echo, warnings = set_param(link, "LINK_LED_HZ", TARGET_HZ)
            if echo != TARGET_HZ or warnings:
                raise ValueError(f"valid set rejected: {echo} {warnings}")
            checks = {}
            for name, value in (("LINK_LED_HZ", 0.0), ("LINK_LED_HZ", 11.0), ("LINK_LED_HZ", math.nan),
                                ("MAV_SYS_ID", 2.0)):
                echo, warnings = set_param(link, name, value)
                checks[f"{name}={value}"] = {"echo": echo, "warnings": warnings}
                expected = TARGET_HZ if name == "LINK_LED_HZ" else 1.0
                if echo != expected or not warnings:
                    raise ValueError(f"invalid set not rejected: {name}={value} -> {echo} {warnings}")
            result["invalid_sets"] = checks

            header, values = link.params()
            if header["dirty"] != "true" or values["LINK_LED_HZ"] != TARGET_HZ:
                raise ValueError("valid change not visible before save")
            result["params_dirty"] = {"header": header, "values": values}

            ack = storage_command(link, 1.0)
            result["save_ack"] = ack.result
            if ack.result != mav2.MAV_RESULT_ACCEPTED:
                raise ValueError("save not accepted")
            header, values = link.params()
            result["params_saved"] = {"header": header, "values": values}
            if header["dirty"] != "false" or header["storage"] != "Stored" or values["LINK_LED_HZ"] != TARGET_HZ:
                raise ValueError("save did not commit the expected snapshot")

            image = link.read_image()
            (args.output / "fram-after.bin").write_bytes(image)
            records, active = validate_record(image, header, values)
            if not region_matches_backup(image, backup):
                raise ValueError("write escaped the Rust region")
            offset = START + active["slot"] * SLOT_SIZE
            size = HEADER + len(values) * 4 + 4
            if image[:offset] != backup[:offset] or image[offset + size:] != backup[offset + size:]:
                raise ValueError("Bytes outside the active record changed")
            result["slots"] = records
            result["active"] = active
            result["fram_sha256"] = hashlib.sha256(image).hexdigest()
            result["region"] = [START, START + 2 * SLOT_SIZE]
        else:
            saved = json.loads(args.save_result.read_text())
            if saved["outcome"] != "pass" or saved["mode"] != "save" or saved["source"] != args.source_id:
                raise ValueError("save step did not pass")
            if header["dirty"] != "false" or header["storage"] != "Stored":
                raise ValueError("power-cycled state is not the saved state")
            if values["LINK_LED_HZ"] != TARGET_HZ or values["MAV_SYS_ID"] != 1.0:
                raise ValueError("saved values were not restored")
            if header["slot"] != str(saved["active"]["slot"]) or \
                    header["generation"] != str(saved["active"]["generation"]):
                raise ValueError("unexpected slot/generation after reboot")
            image = link.read_image()
            (args.output / "fram-after.bin").write_bytes(image)
            records, active = validate_record(image, header, values)
            if not region_matches_backup(image, backup):
                raise ValueError("bytes outside the Rust region changed")
            result["slots"] = records
            result["active"] = active
            result["fram_sha256"] = hashlib.sha256(image).hexdigest()
            if result["fram_sha256"] != saved["fram_sha256"]:
                raise ValueError("FRAM changed across the power cycle")
        result["bad_mavlink_frames"] = link.capture.bad_frames
        if link.capture.bad_frames:
            raise ValueError("Corrupt MAVLink frames")
        result["outcome"] = "pass"
    except (OSError, ValueError, RuntimeError, KeyError, TimeoutError, serial.SerialException) as error:
        result.update(outcome="failed", error=f"{type(error).__name__}: {error}")
    finally:
        if link:
            link.close()
        result["finished_utc"] = datetime.now(timezone.utc).isoformat()
        with (args.output / "result.json").open("x") as output:
            json.dump(result, output, indent=2)
            output.write("\n")
            output.flush()
            os.fsync(output.fileno())
    print(json.dumps({key: value for key, value in result.items()
                      if key not in ("platform", "params_before", "params_dirty", "params_saved")}, indent=2))
    return 0 if result["outcome"] == "pass" else 1


if __name__ == "__main__":
    raise SystemExit(main())
