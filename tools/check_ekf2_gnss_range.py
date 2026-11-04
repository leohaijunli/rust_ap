#!/usr/bin/env python3
"""Exercise only the E3 fixed C++ reference. Rust qualification is not implied."""

import csv
import argparse
import hashlib
import json
import sys
import tempfile
import gzip
import itertools
import math
import subprocess
from pathlib import Path

from check_ekf2_host import ROOT, run_json, record_baseline, compare_baseline
from ekf2_replay import write_synthetic, check_golden
from ekf2_gnss_range import FIELDS, EXTENDED_FIELDS, observations
from gen_params import DEFAULT_REF
from check_ekf2_imu import compare_outputs
from check_ekf2_baro_mag import NIS_TOLERANCES

BASELINE = ROOT / "tests/data/ekf2/gnss_range_stationary"
NIS_BASELINE = ROOT / "tests/data/ekf2/gnss_range_stationary_nis"
CONFIGURATION = {"seconds": 20.0, "imu_rate_hz": 400, "aid_rate_hz": 50, "gnss_rate_hz": 5,
                 "latitude_deg": 47.397742, "longitude_deg": 8.545594, "altitude_m": 100.0,
                 "range_m": 1.0, "range_quality": 100, "field_ned_gauss": [0.2, 0.0, 0.4]}


def compare_aids(actual_path, expected_path):
    def opened(path):
        return gzip.open(path, "rt", encoding="utf-8") if path.suffix == ".gz" else path.open(encoding="utf-8")
    with opened(actual_path) as actual, opened(expected_path) as expected:
        left, right = csv.reader(actual), csv.reader(expected)
        header = next(left)
        if header != next(right) or len(header) != 56:
            raise ValueError("E3 aid status schema differs")
        integers = {i for i, name in enumerate(header) if name.endswith(("timestamp_us", "_fused", "_rejected"))
                    or name in ("control_status", "fault_status", "gnss_quality_failure_bits")}
        count = 0
        for row, reference in itertools.zip_longest(left, right):
            if row is None or reference is None or len(row) != len(header) or len(reference) != len(header):
                raise ValueError("E3 aid status row count/width differs")
            count += 1
            for i, (value, baseline) in enumerate(zip(row, reference)):
                if i in integers: matches = int(value) == int(baseline)
                else: matches = math.isfinite(float(value)) and math.isfinite(float(baseline)) and math.isclose(
                    float(value), float(baseline), abs_tol=NIS_TOLERANCES["absolute"], rel_tol=NIS_TOLERANCES["relative"])
                if not matches:
                    raise ValueError(f"E3 aid status differs at row {count}, {header[i]}: {value} vs {baseline}")
    return {"rows": count, "tolerances": NIS_TOLERANCES}


def failure_events(path, case):
    if case == "nominal": return {}
    with path.open(encoding="utf-8") as file: rows = list(csv.DictReader(file))
    if case == "outliers":
        rejected = {name: any(int(row[f"{name}_rejected"]) for row in rows
                    if low < int(row[f"{name}_timestamp_us"]) < high) for name, low, high in
                    [("gnss_pos", 11_880_000, 12_400_000), ("gnss_vel", 11_880_000, 12_400_000), ("range", 12_990_000, 13_200_000)]}
        if not all(rejected.values()): raise ValueError(f"E3 outlier rejection paths not exercised: {rejected}")
        return {"position_velocity_range_rejected": True}
    sources = [("gnss_pos", 2), ("gnss_vel", 44), ("gnss_hgt", 11)]
    if case == "dropout": sources += [("range_height", 10), ("range_terrain", 39)]
    for name, bit in sources:
        seen = False
        stopped = False
        for row in rows:
            active = bool(int(row["control_status"]) & (1 << bit))
            if seen and not active: stopped = True
            seen |= active
        if not seen or not stopped or not int(rows[-1]["control_status"]) & (1 << bit):
            raise ValueError(f"E3 {name} did not start, stop and recover")
    if case == "quality" and not any(int(row["gnss_quality_failure_bits"]) & 1 for row in rows):
        raise ValueError("E3 GNSS fix-quality failure was not exercised")
    return {"sources_stopped_and_recovered": [name for name, _ in sources]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--record-baseline", action="store_true")
    parser.add_argument("--record-nis-baseline", action="store_true")
    parser.add_argument("--rust", type=Path)
    parser.add_argument("--case", choices=("nominal", "outliers", "dropout", "quality", "lever_arm"), default="nominal")
    parser.add_argument("--root", type=Path)
    args = parser.parse_args()
    duration = 30.0 if args.case in ("dropout", "quality") else 20.0
    configuration = CONFIGURATION if args.case == "nominal" else CONFIGURATION | {"seconds": duration, "failure_case": args.case}
    baseline = BASELINE if args.case == "nominal" else BASELINE.parent / f"gnss_range_{args.case}"
    nis_baseline = NIS_BASELINE if args.case == "nominal" else baseline
    if not (args.record_baseline or args.record_nis_baseline) and (not baseline.exists() or not nis_baseline.exists()):
        parser.error("committed E3 state/NIS baseline is missing; recording must be explicit")
    root = args.root or ROOT / ("target/ekf2-host-gnss-range-lever-arm" if args.case == "lever_arm" else "target/ekf2-host-gnss-range")
    try:
        manifest = json.loads((root / "source_manifest.json").read_text())
        executable = root / "build/golden_cpp"
        if manifest["px4_ref"] != DEFAULT_REF or manifest["profile"] != "gnss_range" or hashlib.sha256(executable.read_bytes()).hexdigest() != manifest["executable_sha256"]:
            raise ValueError("E3 reference executable/profile mismatch")
        if manifest.get("fixture", "nominal") != ("lever_arm" if args.case == "lever_arm" else "nominal"):
            raise ValueError("E3 reference fixture mismatch")
        output = root / ("check" if args.case == "nominal" else f"check-{args.case}")
        output.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(prefix="e3-", dir=output) as temporary:
            directory = Path(temporary)
            write_synthetic(directory / "imu.csv", directory / "truth.csv", seconds=duration, yaw_rate_deg_s=0.0)
            with (directory / "aids.csv").open("x", newline="") as file:
                writer = csv.writer(file, lineterminator="\n")
                writer.writerow(FIELDS if args.case in ("nominal", "lever_arm") else EXTENDED_FIELDS)
                writer.writerows(observations(duration, case="nominal" if args.case == "lever_arm" else args.case))
            reference = run_json([executable, directory / "imu.csv", directory / "golden.csv", directory / "aids.csv"])
            inspection = check_golden(directory / "golden.csv")
            with (directory / "golden.csv").open() as file:
                rows = list(csv.DictReader(file))
            status = int(rows[-1]["control_status"])
            flags = {name: bool(status & (1 << bit)) for name, bit in [("gnss_position", 2), ("gnss_velocity", 44), ("gnss_height", 11), ("range_terrain", 39), ("range_height", 10)]}
            if not all(flags.values()):
                raise ValueError(f"E3 reference did not activate GNSS/range fusion: {flags}")
            report = {"schema": 1, "px4_ref": DEFAULT_REF, "stage": "E3-H reference preparation", "cpp": reference, "inspection": inspection,
                      "last_flags": flags, "rust_qualification": "not implemented", "hardware": "not run"}
            files = {name: directory / name for name in ("imu.csv", "truth.csv", "aids.csv", "golden.csv")}
            if args.record_baseline:
                record_baseline(baseline, files, report, manifest, configuration)
            if baseline.exists():
                report["cpp_regression"] = compare_baseline(baseline, files, manifest, configuration)
            aid_executable = root / "build/golden_cpp_aids"
            if hashlib.sha256(aid_executable.read_bytes()).hexdigest() != manifest["aids_executable_sha256"]:
                raise ValueError("E3 aid executable checksum mismatch")
            run_json([aid_executable, directory / "imu.csv", directory / "nis.csv", directory / "aids.csv"])
            report["failure_events"] = failure_events(directory / "nis.csv", "nominal" if args.case == "lever_arm" else args.case)
            if args.record_nis_baseline:
                record_baseline(nis_baseline, files | {"nis.csv": directory / "nis.csv"}, report, manifest, configuration)
            if nis_baseline.exists():
                stored = json.loads((nis_baseline / "manifest.json").read_text())
                compressed = (nis_baseline / "nis.csv.gz").read_bytes()
                info = stored["artifacts"]["nis.csv.gz"]
                if hashlib.sha256(compressed).hexdigest() != info["sha256"] or hashlib.sha256(gzip.decompress(compressed)).hexdigest() != info["raw_sha256"]:
                    raise ValueError("E3 aid baseline checksum mismatch")
                report["nis_regression"] = compare_aids(directory / "nis.csv", nis_baseline / "nis.csv.gz")
            if args.rust:
                report["stage"] = "E3-H numerical qualification"
                flags = ["--lever-arm"] if args.case == "lever_arm" else []
                report["rust"] = run_json([args.rust.resolve(), directory / "imu.csv", directory / "rust.csv", directory / "aids.csv", *flags])
                (directory / "rust.csv").replace(output / "rust.csv")
                (directory / "golden.csv").replace(output / "golden.csv")
                report["rust_qualification"] = compare_outputs(output / "rust.csv", output / "golden.csv")
                run_json([args.rust.resolve(), directory / "imu.csv", directory / "rust-nis.csv", directory / "aids.csv", "--nis", *flags])
                (directory / "rust-nis.csv").replace(output / "rust-nis.csv")
                (directory / "nis.csv").replace(output / "nis.csv")
                report["rust_nis"] = compare_aids(output / "rust-nis.csv", output / "nis.csv")
                report["rust_failure_events"] = failure_events(output / "rust-nis.csv", "nominal" if args.case == "lever_arm" else args.case)
            for name in ("imu.csv", "truth.csv", "aids.csv", "golden.csv"):
                if (directory / name).exists(): (directory / name).replace(output / name)
            if (directory / "nis.csv").exists(): (directory / "nis.csv").replace(output / "nis.csv")
            (output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report, indent=2))
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as error:
        print(f"check_ekf2_gnss_range: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
