#!/usr/bin/env python3
"""Read-only board probe: repeat start/stop cycles and sample each state several times.

The estimator diagnostics are produced by the console task while the estimator
worker may still be finishing an epoch, so every state is read multiple times
and the repeated readings must agree before a stage code is trusted.
"""

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path
import time

import serial

from check_mavlink import find_port
from ekf2_runtime_capture import RuntimeCapture

FIELDS = ("requested", "active", "config_errors", "last_config_fault", "config_stage", "configure_max_us")
STABLE_FIELDS = ("requested", "active", "config_errors", "last_config_fault", "config_stage")
INSTANCE_FIELDS = ("samples", "updates", "mode", "fault")


def read_once(capture):
    return {"fields": capture.request("!ekf2")["fields"], "metrics": capture.request("!metrics")}


def summary(sample):
    fields = sample["fields"]
    runtime = fields["EKF2_RUNTIME"]
    return {
        "runtime": {key: runtime.get(key) for key in FIELDS},
        "config": fields.get("EKF2_CONFIG", {}),
        "params": fields.get("EKF2_CONFIG_PARAMS", {}),
        "main": {key: fields["EKF2_STATE:main"].get(key) for key in INSTANCE_FIELDS},
        "independent": {key: fields["EKF2_STATE:independent"].get(key) for key in INSTANCE_FIELDS},
        "stack": sample["metrics"]["fields"]["STACK"],
    }


def read_consistent(capture, reads, gap):
    """Read one state several times and prove every part of it is stable.

    Reading parameters is not a loop body: the estimator keeps running, so a
    few extra reads cost nothing and let a transient value be rejected instead
    of mistaken for the real configuration.
    """
    readings = []
    for index in range(reads):
        if index:
            time.sleep(gap)
        readings.append(summary(read_once(capture)))

    def distinct_of(extract):
        return [list(value) for value in sorted({extract(row) for row in readings})]

    runtime = distinct_of(lambda row: tuple(row["runtime"][field] for field in STABLE_FIELDS))
    params = distinct_of(lambda row: tuple(sorted(row["params"].items())))
    config = distinct_of(lambda row: tuple(sorted(row["config"].items())))
    modes = distinct_of(lambda row: (row["main"]["mode"], row["independent"]["mode"]))
    stable = [len(part) == 1 for part in (runtime, params, config, modes)]
    return {
        "readings": readings,
        "runtime_stable": stable[0],
        "params_stable": stable[1],
        "config_stable": stable[2],
        "modes_stable": stable[3],
        "stable": all(stable),
        "distinct_runtime": runtime,
        "distinct_params": params,
        "distinct_config": config,
        "distinct_modes": modes,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port")
    parser.add_argument("--source-id")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seconds", type=float, default=3.0)
    parser.add_argument("--cycles", type=int, default=4)
    parser.add_argument("--reads", type=int, default=5)
    parser.add_argument("--read-gap", type=float, default=0.2)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    result = {"schema": 1, "utc": datetime.now(timezone.utc).isoformat(), "cycles": []}
    try:
        with serial.Serial(args.port or find_port(), 115200, timeout=0.005, write_timeout=0.5,
                           exclusive=True) as port, (args.output / "wire.jsonl").open("x") as raw:
            capture = RuntimeCapture(port, raw)
            capture.observe(2)
            platform = capture.request("!platform")
            result["platform"] = platform["lines"]
            if args.source_id and args.source_id not in "\n".join(platform["lines"]):
                raise ValueError("firmware source ID does not match")
            capture.request("!ekf2 stop")
            capture.observe(0.3)
            result["baseline"] = read_consistent(capture, args.reads, args.read_gap)
            for index in range(args.cycles):
                start = capture.request("!ekf2 start")
                capture.observe(args.seconds)
                running = read_consistent(capture, args.reads, args.read_gap)
                stop = capture.request("!ekf2 stop")
                capture.observe(0.3)
                result["cycles"].append({
                    "index": index,
                    "start": start["lines"],
                    "running": running,
                    "stop": stop["lines"],
                    "stopped": read_consistent(capture, args.reads, args.read_gap),
                })
            result["fault"] = capture.request("!fault")["lines"]
            result["passed"] = all(
                cycle["running"]["stable"]
                and cycle["running"]["readings"][0]["runtime"]["active"] == "true"
                and int(cycle["running"]["readings"][0]["runtime"]["config_errors"]) == 0
                for cycle in result["cycles"]
            ) and result["baseline"]["stable"]
    except (OSError, ValueError, KeyError, TimeoutError) as error:
        result["error"] = str(error)
        result["passed"] = False
    finally:
        (args.output / "summary.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({
        "passed": result.get("passed"),
        "error": result.get("error"),
        "baseline_stable": result.get("baseline", {}).get("stable"),
        "baseline_config": result.get("baseline", {}).get("readings", [{}])[0].get("config"),
        "baseline_params": result.get("baseline", {}).get("readings", [{}])[0].get("params"),
        "cycles": [
            {
                "index": cycle["index"],
                "runtime_stable": cycle["running"]["runtime_stable"],
                "params_stable": cycle["running"]["params_stable"],
                "config_stable": cycle["running"]["config_stable"],
                "modes_stable": cycle["running"]["modes_stable"],
                "distinct_runtime": cycle["running"]["distinct_runtime"],
                "distinct_params": cycle["running"]["distinct_params"],
                "distinct_config": cycle["running"]["distinct_config"],
                "config": cycle["running"]["readings"][0]["config"],
                "params": cycle["running"]["readings"][0]["params"],
            }
            for cycle in result.get("cycles", [])
        ],
    }, indent=2))
    print(f"Evidence: {args.output}")
    return 0 if result.get("passed") else 1


if __name__ == "__main__":
    raise SystemExit(main())
