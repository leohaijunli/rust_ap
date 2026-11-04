#!/usr/bin/env python3
"""Qualify the Rust E1 IMU core against a committed fixed-PX4 baseline."""

import argparse
import csv
import gzip
import hashlib
import itertools
import json
import math
import os
import subprocess
import struct
import sys
import tempfile
from pathlib import Path

from check_ekf2_host import BASELINE, ROOT, run_json
from ekf2_replay import GOLDEN_FIELDS, check_golden, open_csv
from gen_params import DEFAULT_REF

# Fixed in docs/ekf2-port.md before the first Rust comparison. Keep independent
# from the tighter C++ rebuild regression tolerances and never rewrite a golden.
TOLERANCES = {"quaternion_norm": 1e-5, "state_abs": 1e-5, "state_relative": 1e-5,
              "covariance_abs": 1e-7, "covariance_relative": 1e-4, "psd_scaled_pivot_abs": 1e-5}


def require_psd(covariance):
    """LDL check in unit-diagonal coordinates, allowing only f32 roundoff."""
    size = len(covariance)
    if any(not math.isfinite(value) for row in covariance for value in row):
        raise ValueError("non-finite covariance")
    diagonal = [covariance[i][i] for i in range(size)]
    if any(value < 0 for value in diagonal):
        raise ValueError("negative covariance diagonal")
    active = [i for i, value in enumerate(diagonal) if value > 0]
    for i, value in enumerate(diagonal):
        if value == 0 and any(covariance[i][j] != 0 or covariance[j][i] != 0 for j in range(size)):
            raise ValueError("zero variance with nonzero covariance")
    scales = [math.sqrt(diagonal[i]) for i in active]
    count = len(active)
    lower = [[0.0] * count for _ in active]
    pivots = [0.0] * count
    tolerance = TOLERANCES["psd_scaled_pivot_abs"]
    for i, source_i in enumerate(active):
        for j in range(i + 1):
            source_j = active[j]
            value = covariance[source_i][source_j] / (scales[i] * scales[j])
            value -= sum(lower[i][k] * pivots[k] * lower[j][k] for k in range(j))
            if i == j:
                if value < -tolerance:
                    raise ValueError("covariance is not positive semidefinite")
                pivots[i] = max(value, 0.0)
                lower[i][i] = 1.0
            elif pivots[j] > tolerance:
                lower[i][j] = value / pivots[j]
            elif abs(value) > tolerance:
                raise ValueError("inconsistent semidefinite covariance pivot")


def compare_outputs(actual_path, reference_path):
    check_golden(actual_path)
    count = 0
    different_f32_fields = 0
    max_quaternion = max_state = max_covariance = max_diagonal_relative = 0.0
    with open_csv(actual_path) as actual, open_csv(reference_path) as reference:
        actual_rows, reference_rows = csv.reader(actual), csv.reader(reference)
        if tuple(next(actual_rows)) != GOLDEN_FIELDS or tuple(next(reference_rows)) != GOLDEN_FIELDS:
            raise ValueError("E1 output schema differs")
        for row, baseline in itertools.zip_longest(actual_rows, reference_rows):
            if row is None or baseline is None or len(row) != len(GOLDEN_FIELDS) or len(baseline) != len(GOLDEN_FIELDS):
                raise ValueError("E1 output row count/width differs")
            if [int(value) for value in row[:4]] != [int(value) for value in baseline[:4]]:
                raise ValueError(f"E1 timestamp/control/fault differs at row {count + 1}")
            values = [float(value) for value in row[4:]]
            expected = [float(value) for value in baseline[4:]]
            if any(not math.isfinite(value) for value in (*values, *expected)):
                raise ValueError("non-finite E1 output")
            sign = -1 if sum(a * b for a, b in zip(values[:4], expected[:4])) < 0 else 1
            values[:4] = [sign * value for value in values[:4]]
            different_f32_fields += sum(struct.pack("<f", a) != struct.pack("<f", b) for a, b in zip(values, expected))
            quaternion_error = math.sqrt(sum((a - b) ** 2 for a, b in zip(values[:4], expected[:4])))
            if quaternion_error > TOLERANCES["quaternion_norm"]:
                raise ValueError(f"E1 quaternion differs at row {count + 1}")
            max_quaternion = max(max_quaternion, quaternion_error)
            for index, (value, baseline_value) in enumerate(zip(values, expected)):
                kind = "state" if index < 25 else "covariance"
                error = abs(value - baseline_value)
                if not math.isclose(value, baseline_value, rel_tol=TOLERANCES[f"{kind}_relative"],
                                    abs_tol=TOLERANCES[f"{kind}_abs"]):
                    raise ValueError(f"E1 differs at row {count + 1}, {GOLDEN_FIELDS[index + 4]}: {value} vs {baseline_value}")
                if kind == "state":
                    max_state = max(max_state, error)
                else:
                    max_covariance = max(max_covariance, error)
            for i in range(24):
                index = 25 + i * 25
                error = abs(values[index] - expected[index]) / max(abs(expected[index]), TOLERANCES["covariance_abs"])
                max_diagonal_relative = max(max_diagonal_relative, error)
            for data in (values, expected):
                covariance = [data[25 + i * 24:25 + (i + 1) * 24] for i in range(24)]
                require_psd(covariance)
            count += 1
    return {"compared_rows": count, "max_quaternion_norm": max_quaternion,
            "different_f32_fields": different_f32_fields,
            "max_state_abs": max_state, "max_covariance_abs": max_covariance,
            "max_covariance_diagonal_relative_with_floor": max_diagonal_relative,
            "tolerances": TOLERANCES}


def load_baseline(directory):
    manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
    build = manifest["reference_build"]
    if (manifest["schema"] != 1 or manifest["px4_ref"] != DEFAULT_REF or build["profile"] != "core_imu"
            or build["px4_ref"] != DEFAULT_REF or build["error_state_count"] != 24 or build["nominal_state_count"] != 25):
        raise ValueError("E1 baseline source/profile differs")
    for name, info in manifest["artifacts"].items():
        if Path(name).name != name:
            raise ValueError("invalid baseline artifact path")
        compressed = (directory / name).read_bytes()
        raw = gzip.decompress(compressed)
        if (hashlib.sha256(compressed).hexdigest() != info["sha256"]
                or hashlib.sha256(raw).hexdigest() != info["raw_sha256"]):
            raise ValueError(f"baseline artifact checksum mismatch: {name}")
    check_golden(directory / "golden.csv.gz")
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, default=BASELINE)
    parser.add_argument("--output", type=Path, default=ROOT / "target/ekf2-host/rust-imu")
    parser.add_argument("--rust", type=Path, help="use an already-built host example")
    args = parser.parse_args()
    try:
        directory = args.baseline.resolve()
        output = args.output.resolve()
        if output == directory or directory in output.parents:
            raise ValueError("outputs must not modify the baseline directory")
        manifest = load_baseline(directory)
        output.mkdir(parents=True, exist_ok=True)
        executable = args.rust
        if executable is None:
            host = next(line.removeprefix("host: ") for line in
                        subprocess.check_output(["rustc", "-vV"], text=True).splitlines() if line.startswith("host: "))
            subprocess.run(["cargo", "build", "--release", "--locked", "--target", host,
                            "--example", "ekf2_input"], cwd=ROOT, check=True)
            executable = ROOT / f"target/{host}/release/examples/ekf2_input{'.exe' if os.name == 'nt' else ''}"
        with tempfile.TemporaryDirectory(prefix="e1-", dir=output) as temporary:
            imu = Path(temporary) / "imu.csv"
            rust_dump = Path(temporary) / "rust.csv"
            imu.write_bytes(gzip.decompress((directory / "imu.csv.gz").read_bytes()))
            rust = run_json([executable.resolve(), imu, rust_dump])
            if rust["samples"] != manifest["cpp"]["samples"] or rust["input_digest"] != manifest["cpp"]["input_digest"]:
                raise ValueError("E1 input digest/count differs")
            comparison = compare_outputs(rust_dump, directory / "golden.csv.gz")
            if comparison["compared_rows"] != rust["updates"] or rust["updates"] != manifest["cpp"]["updates"]:
                raise ValueError("E1 update count differs")
            # Replace derived paths, never truncate an existing inode that may
            # be a hard link to an input or committed artifact.
            imu.replace(output / "imu.csv")
            rust_dump.replace(output / "rust.csv")
        report = {"schema": 1, "stage": "E1-H", "px4_ref": DEFAULT_REF,
                  "baseline": directory.name, "rust": rust, "comparison": comparison,
                  "hardware_validation": "not run; delayed core only, no board integration"}
        (output / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, indent=2))
    except (OSError, ValueError, KeyError, StopIteration, subprocess.CalledProcessError) as error:
        print(f"check_ekf2_imu: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
