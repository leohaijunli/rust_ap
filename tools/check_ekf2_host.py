#!/usr/bin/env python3
"""Check one synthetic input through PX4 C++ and the Rust DTO intake probe."""

import argparse
import csv
import gzip
import hashlib
import itertools
import json
import math
import os
import subprocess
import sys
from pathlib import Path

from ekf2_replay import check_golden, open_csv, write_synthetic
from gen_params import DEFAULT_REF

ROOT = Path(__file__).resolve().parents[1]
BASELINE = ROOT / "tests/data/ekf2/imu_yaw"
CONFIGURATION = {"seconds": 10.0, "rate_hz": 400, "yaw_rate_deg_s": 5.0, "seed": 0,
                 "gyro_noise_rad_s": 0.0, "accel_noise_m_s2": 0.0}
CONFIGURATIONS = {
    "imu_yaw": CONFIGURATION,
    "imu_noisy_clipped": CONFIGURATION | {"seed": 42, "gyro_noise_rad_s": 0.02, "accel_noise_m_s2": 0.1,
                                          "clipping_start_s": 3.0, "clipping_end_s": 3.15, "clipping_mask": 2},
}
# Fixed before the first comparison. These are C++ host-regression tolerances,
# not the future Rust/PX4 E1 numerical qualification thresholds.
TOLERANCES = {"state_abs": 1e-6, "covariance_abs": 1e-7, "relative": 1e-5}


def run_json(command):
    result = subprocess.run([str(arg) for arg in command], check=True, text=True, capture_output=True)
    return json.loads(result.stdout.strip().splitlines()[-1])


def read_reference_manifest(executable):
    manifest = json.loads((executable.parent.parent / "source_manifest.json").read_text(encoding="utf-8"))
    if (manifest["px4_ref"] != DEFAULT_REF or manifest["profile"] != "core_imu"
            or manifest["error_state_count"] != 24 or manifest["nominal_state_count"] != 25
            or hashlib.sha256(executable.read_bytes()).hexdigest() != manifest["executable_sha256"]):
        raise ValueError("golden executable does not match its fixed-reference manifest")
    return manifest


def record_baseline(directory, files, report, source_manifest, configuration=CONFIGURATION):
    targets = [directory / f"{name}.gz" for name in files] + [directory / "manifest.json"]
    if any(path.exists() for path in targets):
        raise ValueError("golden baseline already exists; recording never overwrites it")
    directory.mkdir(parents=True, exist_ok=True)
    artifacts = {}
    for name, source in files.items():
        data = source.read_bytes()
        compressed = gzip.compress(data, mtime=0)
        with (directory / f"{name}.gz").open("xb") as output:
            output.write(compressed)
        artifacts[f"{name}.gz"] = {"raw_bytes": len(data), "bytes": len(compressed),
                                   "raw_sha256": hashlib.sha256(data).hexdigest(),
                                   "sha256": hashlib.sha256(compressed).hexdigest()}
    manifest = report | {"configuration": configuration, "tolerances": TOLERANCES,
                         "artifacts": artifacts, "reference_build": source_manifest,
                         "license": "BSD-3-Clause; see tools/px4_msgs/LICENSE"}
    with (directory / "manifest.json").open("x", encoding="utf-8") as output:
        output.write(json.dumps(manifest, indent=2) + "\n")


def compare_baseline(directory, files, reference_build, configuration=CONFIGURATION, allowed_fault_mask=0):
    manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
    if (manifest["px4_ref"] != DEFAULT_REF or manifest["configuration"] != configuration
            or manifest["tolerances"] != TOLERANCES
            or manifest["reference_build"]["applied_parameters"] != reference_build["applied_parameters"]
            or manifest.get("allowed_fault_mask", 0) != allowed_fault_mask):
        raise ValueError("golden baseline configuration/parameters differ")
    for name, source in files.items():
        expected = manifest["artifacts"][f"{name}.gz"]
        compressed = (directory / f"{name}.gz").read_bytes()
        if (hashlib.sha256(compressed).hexdigest() != expected["sha256"]
                or hashlib.sha256(gzip.decompress(compressed)).hexdigest() != expected["raw_sha256"]):
            raise ValueError(f"golden artifact checksum mismatch: {name}")
        if name != "golden.csv" and hashlib.sha256(source.read_bytes()).hexdigest() != expected["raw_sha256"]:
            raise ValueError(f"generated input/truth changed: {name}")
    check_golden(directory / "golden.csv.gz", allowed_fault_mask=allowed_fault_mask)
    with open_csv(files["golden.csv"]) as actual, open_csv(directory / "golden.csv.gz") as expected:
        actual_rows, expected_rows = csv.reader(actual), csv.reader(expected)
        if next(actual_rows) != next(expected_rows):
            raise ValueError("golden output columns changed")
        count = 0
        for actual_row, expected_row in itertools.zip_longest(actual_rows, expected_rows):
            if actual_row is None or expected_row is None or len(actual_row) != len(expected_row):
                raise ValueError("golden output row count/width changed")
            for index, (value, baseline) in enumerate(zip(actual_row, expected_row)):
                if index < 4:
                    matches = int(value) == int(baseline)
                else:
                    absolute = TOLERANCES["state_abs" if index < 29 else "covariance_abs"]
                    matches = math.isclose(float(value), float(baseline), rel_tol=TOLERANCES["relative"], abs_tol=absolute)
                if not matches:
                    raise ValueError(f"golden output differs at row {count + 1}, column {index}")
            count += 1
    return {"baseline": directory.relative_to(ROOT).as_posix() if directory.is_relative_to(ROOT) else str(directory),
            "compared_rows": count, "tolerances": TOLERANCES}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--golden", type=Path, default=ROOT / "target/ekf2-host/build/golden_cpp")
    parser.add_argument("--output", type=Path, default=ROOT / "target/ekf2-host/check")
    parser.add_argument("--case", choices=CONFIGURATIONS, default="imu_yaw")
    parser.add_argument("--baseline", type=Path)
    parser.add_argument("--record-baseline", action="store_true", help="create a new baseline; never overwrite existing files")
    args = parser.parse_args()
    try:
        configuration = CONFIGURATIONS[args.case]
        if args.baseline is None:
            args.baseline = BASELINE.parent / args.case
        source_manifest = read_reference_manifest(args.golden)
        args.output.mkdir(parents=True, exist_ok=True)
        imu = args.output / "imu.csv"
        dump = args.output / "golden.csv"
        truth = args.output / "truth.csv"
        expected = write_synthetic(imu, truth, **configuration)
        reference = run_json([args.golden, imu, dump])
        host = next(line.removeprefix("host: ") for line in
                    subprocess.check_output(["rustc", "-vV"], text=True).splitlines() if line.startswith("host: "))
        subprocess.run(["cargo", "build", "--release", "--locked", "--target", host,
                        "--example", "ekf2_input"], cwd=ROOT, check=True)
        rust = ROOT / f"target/{host}/release/examples/ekf2_input{'.exe' if os.name == 'nt' else ''}"
        intake = run_json([rust, imu])
        if (reference["px4_ref"] != DEFAULT_REF or reference["samples"] != expected
                or intake["samples"] != expected or reference["input_digest"] != intake["input_digest"]):
            raise ValueError("PX4/Rust IMU intake mismatch")
        inspection = check_golden(dump)

        # One critical bad-input path on both binaries; parser unit tests cover
        # the remaining field bounds. No repeated simulations or hardware work.
        bad = args.output / "invalid.csv"
        lines = imu.read_text(encoding="utf-8").splitlines()[:2]
        fields = lines[1].split(',')
        fields[3] = "NaN"
        bad.write_text(lines[0] + "\n" + ",".join(fields) + "\n", encoding="utf-8")
        for command in ([args.golden, bad, args.output / "invalid-golden.csv"], [rust, bad]):
            result = subprocess.run([str(arg) for arg in command], text=True, capture_output=True)
            if result.returncode != 1 or "non-finite" not in result.stderr:
                raise ValueError("non-finite input was not rejected")

        # A path alias must never truncate the input file.
        before = imu.read_bytes()
        alias = f"{imu.parent}/./{imu.name}"
        result = subprocess.run([str(args.golden), str(imu), alias],
                                text=True, capture_output=True)
        if result.returncode != 1 or imu.read_bytes() != before:
            raise ValueError("input/output alias was not protected")

        report = {"schema": 1, "px4_ref": DEFAULT_REF, "input_sha256": hashlib.sha256(before).hexdigest(),
                  "cpp": reference, "rust_intake": intake, "golden": inspection,
                  "rust_math": "not exercised by DTO-only mode; E1 qualification uses tools/check_ekf2_imu.py",
                  "hardware_validation": "not run; host fixtures only"}
        files = {"imu.csv": imu, "truth.csv": truth, "golden.csv": dump}
        if args.record_baseline:
            record_baseline(args.baseline, files, report, source_manifest, configuration)
        report["regression"] = compare_baseline(args.baseline, files, source_manifest, configuration)
        (args.output / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, indent=2))
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as error:
        print(f"check_ekf2_host: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
