#!/usr/bin/env python3
"""Engineer-run P1 USB capture; never uploads, reboots, clears faults or injects them."""

import argparse
import json
import math
import time
from pathlib import Path

import serial
from pymavlink.dialects.v20 import common as mav2

from backup_domain import decode_backup
from check_mavlink import find_port

PREFIXES = {
    "P1_PLATFORM", "BACKUP", "CLOCK", "RCC", "POWER", "CORE", "TIMER",
    "P1_METRICS", "PERIOD", "LATE_HIST", "EXEC", "CPU", "STACK", "RAM",
    "P1_FAULT", "FRAME", "FAULT_REG", "OK", "ERR",
}


class Demux:
    """Separate complete MAVLink wire frames from newline-delimited diagnostics."""

    def __init__(self):
        self.buffer = bytearray()
        self.discarded = 0

    def feed(self, data):
        self.buffer.extend(data)
        events = []
        while self.buffer:
            if self.buffer[0] in (0xFD, 0xFE):
                if len(self.buffer) < 3:
                    break
                size = self.buffer[1] + (12 if self.buffer[0] == 0xFD else 8)
                if self.buffer[0] == 0xFD and self.buffer[2] & 1:
                    size += 13
                if len(self.buffer) < size:
                    break
                events.append(("mavlink", bytes(self.buffer[:size])))
                del self.buffer[:size]
                continue
            if not 32 <= self.buffer[0] < 127:
                self.discarded += 1
                del self.buffer[0]
                continue
            end = self.buffer.find(b"\n")
            # Recover from starting in the middle of a binary frame.
            magic = next((i for i, b in enumerate(self.buffer) if b in (0xFD, 0xFE)), None)
            if magic is not None and (end < 0 or magic < end):
                self.discarded += magic
                del self.buffer[:magic]
                continue
            if end < 0:
                if len(self.buffer) > 4096:
                    raise ValueError("overlong console line")
                break
            raw = bytes(self.buffer[:end]).rstrip(b"\r")
            del self.buffer[:end + 1]
            try:
                line = raw.decode("ascii")
            except UnicodeDecodeError:
                self.discarded += len(raw)
                continue
            if line == "END" or line.split(" ", 1)[0] in PREFIXES:
                events.append(("console", line))
            else:
                self.discarded += len(raw)
        return events


def fields(lines):
    result = {}
    for line in lines:
        parts = line.split()
        if not parts:
            continue
        if parts[0] in result:
            raise ValueError("duplicate console record")
        values = {}
        for item in parts[1:]:
            if "=" not in item:
                continue
            key, value = item.split("=", 1)
            if key in values:
                raise ValueError("duplicate field")
            values[key] = value
        result[parts[0]] = values
    if "ERR" in result:
        raise ValueError("firmware returned ERR")
    return result


def analyze(samples, platform, *, stack_margin_bytes=16384, release_latency_us=10):
    """P1-era acceptance analysis.

    `stack_margin_bytes` and `release_latency_us` reflect the calibrated
    criteria (2026-11-06): the 16 KiB untouched margin follows the PX4 WQ
    stack comparison in docs/next-steps.md. Callers that already enforce a
    stronger load-specific 1 kHz contract (e.g. the E7 runtime capture checks
    EKF2_PUBLISH missed=0 and PERIOD deadline_misses here) may pass
    `release_latency_us=None` to skip the empty-system 10-us gate.
    """
    if len(samples) < 2:
        raise ValueError("need at least two metric samples")
    first, last = samples[0], samples[-1]
    a, b = first["fields"], last["fields"]
    for sample in samples:
        for prefix in ("P1_METRICS", "PERIOD", "LATE_HIST", "EXEC", "CPU", "STACK"):
            if prefix not in sample["fields"]:
                raise ValueError(f"missing {prefix}")
    board_seconds = (int(b["P1_METRICS"]["now_us"]) - int(a["P1_METRICS"]["now_us"])) / 1e6
    lower = (last["sent_ns"] - first["received_ns"]) / 1e9
    upper = (last["received_ns"] - first["sent_ns"]) / 1e9
    if lower <= 0 or board_seconds <= 0:
        raise ValueError("invalid/regressing capture window")
    drift = [board_seconds / upper - 1, board_seconds / lower - 1]
    midpoint = board_seconds / ((lower + upper) / 2) - 1
    issues = []
    if board_seconds < 60:
        issues.append("less than 60 seconds of board time")
    if drift[0] >= -0.001 and drift[1] <= 0.001:
        timebase = "pass"
    elif drift[1] < -0.001 or drift[0] > 0.001:
        timebase = "fail"
        issues.append("timebase exceeds 0.1% target")
    else:
        timebase = "indeterminate: reduce USB/host RTT or use an instrument"
        issues.append("timebase measurement uncertainty overlaps threshold")
    epochs = {s["fields"]["P1_METRICS"]["epoch"] for s in samples}
    if len(epochs) != 1:
        issues.append("metrics epoch changed during capture")
    for previous, current in zip(samples, samples[1:]):
        p, c = previous["fields"], current["fields"]
        if (int(c["P1_METRICS"]["now_us"]) <= int(p["P1_METRICS"]["now_us"])
                or int(c["P1_METRICS"]["snapshot_us"]) < int(p["P1_METRICS"]["snapshot_us"])
                or int(c["PERIOD"]["samples"]) < int(p["PERIOD"]["samples"])):
            issues.append("time or sample count regressed during capture")
            break
    for key in ("skipped", "deadline_misses", "scheduling_overruns", "time_errors"):
        if any(int(s["fields"]["PERIOD"][key]) != 0 for s in samples):
            issues.append(f"nonzero {key}")
    if any(sum(int(x) for x in s["fields"]["LATE_HIST"].values()) != int(s["fields"]["PERIOD"]["samples"]) for s in samples):
        issues.append("latency histogram/sample mismatch")
    if release_latency_us is not None and any(
            int(s["fields"]["PERIOD"]["late_max_us"]) > release_latency_us for s in samples):
        issues.append(f"task release latency >{release_latency_us} us (not DRDY edge jitter)")
    if any(int(s["fields"]["EXEC"]["invalid_cycles"]) != 0 for s in samples):
        issues.append("invalid DWT cycle observations")
    if any(s["fields"]["CPU"]["valid"] != "true" for s in samples):
        issues.append("invalid CPU window")
    cpu_max = max(int(s["fields"]["CPU"]["busy_permille"]) for s in samples)
    if any(int(s["fields"]["CPU"]["window_us"]) <= 0 for s in samples):
        raise ValueError("nonpositive CPU window")
    quantization = max(
        math.ceil(int(s["fields"]["CPU"]["idle_calls"]) * 1000 / int(s["fields"]["CPU"]["window_us"]))
        for s in samples
    )
    # Firmware's integer permille division can round busy load down by <1.
    if cpu_max + quantization + 1 >= 500:
        issues.append("CPU including tick/integer rounding bounds >=50%")
    stack_min = min(int(s["fields"]["STACK"]["untouched_bytes"]) for s in samples)
    if stack_min < stack_margin_bytes or any(s["fields"]["STACK"]["guard_ok"] != "true" for s in samples):
        issues.append(f"shared-stack margin <{stack_margin_bytes} B or guard failed")
    if platform.get("CLOCK", {}).get("registers_ok") != "true":
        issues.append("clock register verification failed")
    backup = decode_backup(platform.get("BACKUP"))
    if backup is None:
        issues.append("missing backup-domain verification (schema 2 firmware required)")
    elif not backup["preserved"]:
        issues.append("backup-domain configuration or BKP preservation failed")
    snapshot_delta = (int(b["P1_METRICS"]["snapshot_us"]) - int(a["P1_METRICS"]["snapshot_us"])) / 1e6
    count_delta = int(b["PERIOD"]["samples"]) - int(a["PERIOD"]["samples"])
    rate = count_delta / snapshot_delta if snapshot_delta > 0 else 0
    if not 990 <= rate <= 1010:
        issues.append("period substitute rate outside 1 kHz ±1%")
    for s in samples:
        m = s["fields"]["P1_METRICS"]
        if not 0 <= int(m["now_us"]) - int(m["snapshot_us"]) < 2_000_000:
            issues.append("stale metric snapshot")
            break
        if not 0 <= int(m["now_us"]) - int(s["fields"]["STACK"]["at_us"]) < 2_000_000:
            issues.append("stale stack snapshot")
            break
    return {
        "board_seconds": board_seconds, "host_delta_bounds_seconds": [lower, upper],
        "timebase_drift_midpoint_ppm": midpoint * 1e6,
        "timebase_drift_bounds_ppm": [v * 1e6 for v in drift], "timebase": timebase,
        "period_rate_hz": rate, "cpu_max_permille": cpu_max, "stack_min_untouched_bytes": stack_min,
        "cpu_quantization_bound_permille": quantization,
        "cpu_integer_rounding_bound_permille": 1,
        "backup_domain": backup,
        "issues": issues, "software_measurement_gate": "pass" if not issues else "not passed",
        "scope": "CPU excludes masked-WFE bracket (including MMIO/DSB overhead); tick bound excludes that hardware overhead. Not WCET/DRDY/IMU evidence. Engineer recovery/LED/QGC checks still required.",
    }


class Capture:
    def __init__(self, port, raw, pressure):
        self.port, self.raw, self.pressure = port, raw, pressure
        self.demux = Demux()
        self.decoder = mav2.MAVLink(None)
        self.encoder = mav2.MAVLink(None, srcSystem=255, srcComponent=190)
        self.sequence = 0
        self.next_ping = time.monotonic()
        self.valid_frames = 0
        self.bad_frames = 0
        self.pings_sent = 0
        self.pings_received = 0

    def log(self, direction, data):
        self.raw.write(json.dumps({"host_ns": time.monotonic_ns(), "direction": direction, "hex": data.hex()}) + "\n")

    def write(self, data):
        self.log("tx", data)
        if self.port.write(data) != len(data):
            raise IOError("short serial write")

    def poll(self):
        now = time.monotonic()
        if self.pressure and now >= self.next_ping:
            message = mav2.MAVLink_ping_message(time.monotonic_ns() // 1000, self.sequence, 0, 0)
            self.write(message.pack(self.encoder))
            self.encoder.seq = (self.encoder.seq + 1) & 255
            self.sequence += 1
            self.pings_sent += 1
            self.next_ping = now + 1 / self.pressure
        data = self.port.read(4096)
        if data:
            self.log("rx", data)
        console = []
        for kind, value in self.demux.feed(data):
            if kind == "console":
                console.append(value)
            else:
                try:
                    messages = self.decoder.parse_buffer(value) or []
                    self.valid_frames += len(messages)
                    self.pings_received += sum(m.get_type() == "PING" for m in messages)
                except mav2.MAVError:
                    self.bad_frames += 1
        return console

    def request(self, name):
        sent = time.monotonic_ns()
        self.write(name.encode() + b"\n")
        lines = []
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            for line in self.poll():
                if line == "END":
                    return {"sent_ns": sent, "received_ns": time.monotonic_ns(), "lines": lines, "fields": fields(lines)}
                lines.append(line)
        raise TimeoutError(f"no complete response to {name}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port")
    parser.add_argument("--seconds", type=float, default=65)
    parser.add_argument("--pressure-hz", type=float, default=0, help="PING requests/s, 0 for idle; max 200")
    parser.add_argument("--output", type=Path, required=True, help="New directory for raw/summary evidence")
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds < 60 or not 0 <= args.pressure_hz <= 200:
        parser.error("seconds >=60; pressure-hz between 0 and 200")
    args.output.mkdir(parents=True, exist_ok=False)
    samples = []
    result = {"schema": 1, "mode": "usb-pressure" if args.pressure_hz else "idle", "requested_pressure_hz": args.pressure_hz}
    try:
        with serial.Serial(args.port or find_port(), 115200, timeout=0.005, write_timeout=0.2, exclusive=True) as port, (args.output / "wire.jsonl").open("w") as raw:
            capture = Capture(port, raw, args.pressure_hz)
            # Settle the CDC session and discard startup partial frames only.
            until = time.monotonic() + 1
            while time.monotonic() < until:
                capture.poll()
            result["platform"] = capture.request("!platform")
            result["fault"] = capture.request("!fault")
            capture.request("!metrics-reset")
            until = time.monotonic() + 2
            while time.monotonic() < until:
                capture.poll()
            capture.valid_frames = capture.bad_frames = 0
            capture.pings_sent = capture.pings_received = 0
            first = capture.request("!metrics")
            samples.append(first)
            deadline = time.monotonic() + args.seconds
            next_sample = time.monotonic() + 1
            while time.monotonic() < deadline:
                capture.poll()
                if time.monotonic() >= next_sample:
                    samples.append(capture.request("!metrics"))
                    next_sample = time.monotonic() + 1
            samples.append(capture.request("!metrics"))
            result["analysis"] = analyze(samples, result["platform"]["fields"])
            result["wire"] = {"valid_frames": capture.valid_frames, "bad_frames": capture.bad_frames, "discarded_bytes": capture.demux.discarded, "pings_sent": capture.pings_sent, "pings_received": capture.pings_received}
            if (not capture.valid_frames or capture.bad_frames
                    or capture.pressure and (not capture.pings_sent or capture.pings_received < capture.pings_sent * 0.95)):
                result["analysis"]["issues"].append("MAVLink corruption or >5% unobserved pressure PING replies")
                result["analysis"]["software_measurement_gate"] = "not passed"
    except (OSError, ValueError, RuntimeError, KeyError, TimeoutError) as error:
        result["error"] = str(error)
    finally:
        result["samples"] = samples
        (args.output / "summary.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result.get("analysis", result.get("error")), indent=2))
    print(f"Evidence: {args.output}")
    return 0 if result.get("analysis", {}).get("software_measurement_gate") == "pass" and "error" not in result else 1


if __name__ == "__main__":
    raise SystemExit(main())
