#!/usr/bin/env python3
"""Qualify E4 full-rate output against fixed C++ and immutable output baselines."""

import argparse
import csv
import gzip
import hashlib
import json
import math
import subprocess
import sys
import tempfile
from pathlib import Path

from check_ekf2_host import ROOT, record_baseline, run_json
from check_ekf2_imu import TOLERANCES
from ekf2_output import write_inputs
from gen_params import DEFAULT_REF

CONFIGURATION = {"seconds": 20.0, "imu_rate_hz": 400, "yaw_rate_deg_s": 5.0,
                 "gnss_delay_ms": 110.0, "range_delay_ms": 5.0, "seed": 0}
CPP_TOLERANCES = TOLERANCES | {"quaternion_norm": 1e-6, "state_abs": 1e-6, "state_relative": 1e-5}


def rows(path):
    with (gzip.open(path, "rt", encoding="utf-8") if path.suffix == ".gz" else path.open(encoding="utf-8")) as file:
        reader = csv.DictReader(file)
        return reader.fieldnames, list(reader)


def compare_output(actual, expected, tolerances=TOLERANCES):
    header, left = rows(actual)
    expected_header, right = rows(expected)
    if header != expected_header or len(header) != 22 or len(left) != len(right):
        raise ValueError("E4 output schema/count differs")
    maximum_q = 0.0
    maximum_state = 0.0
    maximum_global = 0.0
    for i, (row, reference) in enumerate(zip(left, right)):
        for name in header[:4]:
            if int(row[name]) != int(reference[name]): raise ValueError(f"E4 time/status differs at row {i + 1}, {name}")
        q = [float(row[name]) for name in header[4:8]]
        qr = [float(reference[name]) for name in header[4:8]]
        if sum(a * b for a, b in zip(q, qr)) < 0.0: q = [-a for a in q]
        difference = math.sqrt(sum((a - b) ** 2 for a, b in zip(q, qr)))
        if not math.isfinite(difference) or difference > tolerances["quaternion_norm"]:
            raise ValueError(f"E4 quaternion differs at row {i + 1}: {difference}")
        maximum_q = max(maximum_q, difference)
        if abs(math.sqrt(sum(a * a for a in q)) - 1.0) > 1e-4: raise ValueError("E4 output quaternion not normalized")
        for name in header[8:]:
            a, b = float(row[name]), float(reference[name])
            if not math.isfinite(a) or not math.isfinite(b): raise ValueError(f"non-finite E4 output {name}")
            if name in ("latitude_deg", "longitude_deg"):
                scale = 111_320.0 * (math.cos(math.radians(float(reference["latitude_deg"]))) if name == "longitude_deg" else 1.0)
                error = abs(a - b) * scale
                maximum_global = max(maximum_global, error)
                matches = error <= tolerances["state_abs"]
            elif name == "unaided_yaw":
                error = abs(math.remainder(a - b, math.tau))
                # This is a state angle: use the same already-fixed absolute
                # OR relative criterion, while accounting for angle wrapping.
                matches = error <= tolerances["state_abs"] or error <= tolerances["state_relative"] * max(abs(a), abs(b))
            else:
                error = abs(a - b)
                maximum_state = max(maximum_state, error)
                matches = math.isclose(a, b, abs_tol=tolerances["state_abs"], rel_tol=tolerances["state_relative"])
            if not matches: raise ValueError(f"E4 output differs at row {i + 1}, {name}: {a} vs {b}")
    return {"rows": len(left), "max_quaternion_norm": maximum_q, "max_state_abs": maximum_state,
            "max_horizontal_global_error_m": maximum_global, "tolerances": tolerances}


def inspect_output(path, reset_path, expected_count):
    _, output = rows(path)
    _, resets = rows(reset_path)
    if len(output) != expected_count or len(resets) != expected_count:
        raise ValueError("output predictor did not run at the full input cadence")
    maximum_position_step = 0.0
    maximum_angle_step = 0.0
    intentional_resets = 0
    for before, after, old_reset, new_reset in zip(output, output[1:], resets, resets[1:]):
        if int(after["timestamp_us"]) <= int(before["timestamp_us"]) or int(after["delayed_timestamp_us"]) > int(after["timestamp_us"]):
            raise ValueError("E4 output time is not causal/strictly increasing")
        reset = any(old_reset[name] != new_reset[name] for name in old_reset if name != "timestamp_us")
        intentional_resets += reset
        if reset: continue
        step = math.sqrt(sum((float(after[f"position_{axis}"]) - float(before[f"position_{axis}"])) ** 2 for axis in ("n", "e", "d")))
        q_step = math.sqrt(sum((float(after[f"q_{axis}"]) - float(before[f"q_{axis}"])) ** 2 for axis in ("w", "x", "y", "z")))
        maximum_position_step = max(maximum_position_step, step)
        maximum_angle_step = max(maximum_angle_step, q_step)
    # Fixed before first E4 parity: stationary position and 5deg/s yaw, with
    # explicitly recorded initialization/reset events excluded, not hidden.
    if maximum_position_step > 0.02 or maximum_angle_step > 0.01:
        raise ValueError(f"unexplained E4 output step: position={maximum_position_step}, quaternion={maximum_angle_step}")
    return {"rows": len(output), "intentional_reset_rows": intentional_resets,
            "max_nonreset_position_step_m": maximum_position_step, "max_nonreset_quaternion_step": maximum_angle_step,
            "step_limits": {"position_m": 0.02, "quaternion_norm": 0.01}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", choices=("nominal", "jitter"), default="nominal")
    parser.add_argument("--root", type=Path, default=ROOT / "target/ekf2-host-output")
    parser.add_argument("--record-baseline", action="store_true")
    parser.add_argument("--rust", type=Path)
    args = parser.parse_args()
    baseline = ROOT / f"tests/data/ekf2/output_{args.case}"
    configuration = CONFIGURATION | {"case": args.case}
    try:
        manifest = json.loads((args.root / "source_manifest.json").read_text(encoding="utf-8"))
        cpp = args.root / "build/golden_cpp_output"
        if manifest["px4_ref"] != DEFAULT_REF or manifest["profile"] != "gnss_range" or hashlib.sha256(cpp.read_bytes()).hexdigest() != manifest["output_executable_sha256"]:
            raise ValueError("E4 fixed output executable checksum/profile differs")
        output = args.root / f"check-{args.case}"
        output.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(prefix="e4-", dir=output) as temporary:
            directory = Path(temporary)
            expected_count = write_inputs(directory, args.case)
            reference = run_json([cpp.resolve(), directory / "imu.csv", directory / "output.csv", directory / "aids.csv"])
            inspection = inspect_output(directory / "output.csv", directory / "output.resets.csv", expected_count)
            report = {"schema": 1, "stage": "E4-H reference preparation", "px4_ref": DEFAULT_REF, "cpp": reference,
                      "inspection": inspection, "hardware": "not run", "rust_qualification": "not exercised"}
            files = {name: directory / name for name in ("imu.csv", "truth.csv", "aids.csv", "output.csv", "output.resets.csv")}
            if args.record_baseline: record_baseline(baseline, files, report, manifest, configuration)
            stored = json.loads((baseline / "manifest.json").read_text(encoding="utf-8"))
            if stored["configuration"] != configuration or stored["reference_build"]["applied_parameters"] != manifest["applied_parameters"]:
                raise ValueError("E4 baseline configuration/parameters drift")
            for name, path in files.items():
                data = (baseline / f"{name}.gz").read_bytes()
                info = stored["artifacts"][f"{name}.gz"]
                if hashlib.sha256(data).hexdigest() != info["sha256"] or hashlib.sha256(gzip.decompress(data)).hexdigest() != info["raw_sha256"]:
                    raise ValueError("E4 immutable baseline checksum differs")
                if name not in ("output.csv",) and hashlib.sha256(path.read_bytes()).hexdigest() != info["raw_sha256"]:
                    raise ValueError(f"E4 input/reset-event drift: {name}")
            report["cpp_regression"] = compare_output(directory / "output.csv", baseline / "output.csv.gz", CPP_TOLERANCES)
            if args.rust:
                report["rust"] = run_json([args.rust.resolve(), directory / "imu.csv", directory / "rust.csv", directory / "aids.csv", "--output"])
                (directory / "rust.csv").replace(output / "rust.csv")
                (directory / "output.csv").replace(output / "output.csv")
                (directory / "output.resets.csv").replace(output / "output.resets.csv")
                report["rust_qualification"] = compare_output(output / "rust.csv", output / "output.csv")
                report["rust_inspection"] = inspect_output(output / "rust.csv", output / "output.resets.csv", expected_count)
                report["stage"] = "E4-H numerical qualification"
            for name, path in files.items():
                if path.exists(): path.replace(output / name)
            (output / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, indent=2))
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as error:
        print(f"check_ekf2_output: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
