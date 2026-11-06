#!/usr/bin/env python3
"""Authorized data-only EKF runtime capture, reusing the P1 wire demultiplexer."""

import argparse
from collections import Counter
from datetime import datetime, timezone
import json
from pathlib import Path
import time

import serial

from check_mavlink import find_port
from p1_capture import Capture, PREFIXES, analyze

PREFIXES.update({"EKF2_RUNTIME", "EKF2_STATE", "EKF2_AIDS", "EKF2_TIMING", "EKF2_CLOCK",
                 "EKF2_PUBLISH", "EKF2_CONFIG", "EKF2_CONFIG_PARAMS",
                 "EST_CONTROL", "IMU", "IMU_CLOCK", "BARO", "BARO_DATA",
                 "LOGGER", "LOGGER_CONTROL", "LOGGER_SWITCH"})


def records(lines):
    result = {}
    for line in lines:
        parts = line.split()
        if not parts or parts[0] == "END":
            continue
        values = dict(part.split("=", 1) for part in parts[1:] if "=" in part)
        key = parts[0]
        if key == "EKF2_STATE":
            key += ":" + values["instance"]
        elif key == "EKF2_TIMING":
            key += ":" + values["path"]
        if key in result:
            raise ValueError(f"duplicate diagnostic record: {key}")
        result[key] = values
    return result


class RuntimeCapture(Capture):
    def __init__(self, port, raw):
        super().__init__(port, raw, 0)
        self.types = Counter()
        self.armed_seen = False
        self.decoder.set_callback(self.message)

    def message(self, message):
        self.types[f"{message.get_srcComponent()}:{message.get_type()}"] += 1
        if message.get_type() == "HEARTBEAT" and message.base_mode & 128:
            self.armed_seen = True

    def request(self, name):
        sent = time.monotonic_ns()
        self.write(name.encode() + b"\n")
        lines = []
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            for line in self.poll():
                if line == "END":
                    return {"sent_ns": sent, "received_ns": time.monotonic_ns(),
                            "lines": lines, "fields": records(lines)}
                lines.append(line)
        raise TimeoutError(f"no complete response to {name}")

    def observe(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.poll()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port")
    parser.add_argument("--source-id", required=True)
    parser.add_argument("--seconds", type=float, default=65)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--log", action="store_true", help="also start/stop the existing SD logger")
    args = parser.parse_args()
    if not 60 <= args.seconds <= 120:
        parser.error("capture duration must be 60..120 seconds")
    args.output.mkdir(parents=True, exist_ok=False)
    result = {"schema": 1, "source_id": args.source_id, "utc": datetime.now(timezone.utc).isoformat(),
              "samples": [], "issues": [], "scope": "Data-only board test, not navigation/flight qualification"}
    try:
        with serial.Serial(args.port or find_port(), 115200, timeout=0.005, write_timeout=0.5,
                           exclusive=True) as port, (args.output / "wire.jsonl").open("x") as raw:
            capture = RuntimeCapture(port, raw)
            capture.observe(2)
            if capture.armed_seen or not capture.types["1:HEARTBEAT"]:
                raise ValueError("expected live disarmed heartbeat")
            result["platform"] = capture.request("!platform")
            if args.source_id not in "\n".join(result["platform"]["lines"]):
                raise ValueError("firmware source ID does not match the requested package")
            result["fault_before"] = capture.request("!fault")
            result["stop_before"] = capture.request("!ekf2 stop")
            capture.observe(0.1)
            result["metrics_before"] = capture.request("!metrics")
            if int(result["metrics_before"]["fields"]["STACK"]["untouched_bytes"]) < 16384:
                raise ValueError("boot stack margin below 16 KiB")
            try:
                result["start"] = capture.request("!ekf2 start")
                if result["start"]["fields"].get("EST_CONTROL", {}).get("accepted") != "true":
                    raise ValueError("estimator start rejected")
                if args.log:
                    result["logger_start"] = capture.request("!logger start")
                capture.request("!metrics-reset")
                capture.observe(2)
                result["activation"] = capture.request("!ekf2")
                runtime = result["activation"]["fields"]["EKF2_RUNTIME"]
                if runtime["active"] != "true":
                    raise ValueError(f"estimator did not activate: config_errors={runtime['config_errors']}, fault={runtime['last_config_fault']}")
                until = time.monotonic() + args.seconds
                next_sample = time.monotonic()
                while time.monotonic() < until:
                    capture.poll()
                    if time.monotonic() >= next_sample:
                        sample = {"metrics": capture.request("!metrics"), "ekf": capture.request("!ekf2"),
                                  "imu": capture.request("!imu")}
                        result["samples"].append(sample)
                        stack = sample["metrics"]["fields"]["STACK"]
                        if stack["guard_ok"] != "true" or int(stack["untouched_bytes"]) < 16384:
                            raise ValueError("active estimator stack margin/guard failed")
                        if capture.armed_seen:
                            raise ValueError("unexpected armed heartbeat")
                        next_sample = time.monotonic() + 1
                result["samples"].append({"metrics": capture.request("!metrics"),
                                          "ekf": capture.request("!ekf2"), "imu": capture.request("!imu")})
                if args.log:
                    result["logger"] = capture.request("!logger")
            finally:
                result["stop_after"] = capture.request("!ekf2 stop")
                if args.log:
                    result["logger_stop"] = capture.request("!logger stop")
                capture.observe(1)
                result["stopped"] = capture.request("!ekf2")
            result["fault_after"] = capture.request("!fault")
            metrics = [sample["metrics"] for sample in result["samples"]]
            result["p1_analysis"] = analyze(metrics, result["platform"]["fields"],
                                            release_latency_us=None)
            result["issues"].extend(result["p1_analysis"]["issues"])
            last = result["samples"][-1]["ekf"]["fields"]
            first = result["samples"][0]["ekf"]["fields"]
            if any(sample["ekf"]["fields"]["EKF2_RUNTIME"]["active"] != "true" for sample in result["samples"]):
                result["issues"].append("runtime inactive during capture; cached snapshots are not a live baseline")
            state = last["EKF2_STATE:main"]
            if state["fault"] != "None" or state["selected"] != "Filter" or int(state["updates"]) == 0:
                result["issues"].append("main: not running healthy filter output")
            before = first["EKF2_STATE:main"]
            if int(state["samples"]) <= int(before["samples"]) or int(state["updates"]) <= int(before["updates"]):
                result["issues"].append("main: sample/update counters did not advance")
            publish = last["EKF2_PUBLISH"]
            if int(publish["missed"]) != 0:
                result["issues"].append("1-kHz publication skipped periods")
            clock = last["EKF2_CLOCK"]
            if int(clock["windows"]) == 0 or abs(int(clock["integration_error_ppm"])) > 100:
                result["issues"].append("IMU integration/TIM8 disagreement exceeds 100 ppm")
            if capture.bad_frames:
                result["issues"].append("MAVLink parse errors")
            result["wire"] = {"types": dict(capture.types), "bad_frames": capture.bad_frames,
                              "discarded_bytes": capture.demux.discarded}
    except (OSError, ValueError, KeyError, TimeoutError) as error:
        result["error"] = str(error)
    finally:
        result["passed"] = not result["issues"] and "error" not in result
        (args.output / "summary.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({key: result[key] for key in ("source_id", "passed", "issues")}
                     | {key: result[key] for key in ("error", "p1_analysis", "wire") if key in result}, indent=2))
    print(f"Evidence: {args.output}")
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
