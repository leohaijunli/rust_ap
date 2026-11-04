#!/usr/bin/env python3
"""Check fixed E2 golden/NIS data and optional Rust parity, never hardware."""

import argparse
import csv
import hashlib
import gzip
import itertools
import json
import math
import sys
import tempfile
from pathlib import Path

from check_ekf2_host import ROOT, run_json, record_baseline, compare_baseline
from check_ekf2_imu import compare_outputs
from ekf2_baro_mag import FIELDS, samples
from ekf2_replay import check_golden, write_synthetic
from gen_params import DEFAULT_REF

BASELINE = ROOT / "tests/data/ekf2/baro_mag_yaw"
NIS_BASELINE = ROOT / "tests/data/ekf2/baro_mag_yaw_nis"
NIS_TOLERANCES = {"absolute": 1e-6, "relative": 1e-4}
CONFIGURATION = {"seconds": 10.0, "imu_rate_hz": 400, "aid_rate_hz": 50,
                 "yaw_rate_deg_s": 5.0, "altitude_m": 100.0, "field_ned_gauss": [0.2, 0.0, 0.4]}


def compare_nis(actual_path, expected_path):
    def open_file(path):
        return gzip.open(path, "rt", encoding="utf-8") if path.suffix == ".gz" else path.open(encoding="utf-8")
    integers = {0, 1, 2, 3, 4, 8, 9, 10, 20, 21}
    with open_file(actual_path) as actual, open_file(expected_path) as expected:
        left, right = csv.reader(actual), csv.reader(expected)
        header = next(left)
        if header != next(right) or len(header) != 22:
            raise ValueError("aid status schema differs")
        count = 0
        for row, reference in itertools.zip_longest(left, right):
            if row is None or reference is None or len(row) != 22 or len(reference) != 22:
                raise ValueError("aid status count/width differs")
            for i, (value, baseline) in enumerate(zip(row, reference)):
                if i in integers:
                    matches = int(value) == int(baseline)
                else:
                    matches = math.isfinite(float(value)) and math.isfinite(float(baseline)) and math.isclose(
                        float(value), float(baseline), abs_tol=NIS_TOLERANCES["absolute"], rel_tol=NIS_TOLERANCES["relative"])
                if not matches:
                    raise ValueError(f"aid status differs at row {count + 1}, {header[i]}: {value} vs {baseline}")
            count += 1
    return {"rows": count, "tolerances": NIS_TOLERANCES}


def check_failure_events(path, case):
    with path.open(encoding="utf-8") as file:
        rows = list(csv.DictReader(file))
    if case == "outliers":
        baro = any(2_490_000 < int(row["baro_timestamp_us"]) < 2_660_000 and int(row["baro_rejected"]) for row in rows)
        mag = any(3_490_000 < int(row["mag_timestamp_us"]) < 3_660_000 and int(row["mag_rejected"]) for row in rows)
        if not baro or not mag:
            raise ValueError("outlier fixture did not exercise both rejection paths")
        return {"baro_outlier_rejected": True, "mag_outlier_rejected": True}
    if case in ("dropout", "dropout_after_alignment"):
        seen = {"baro": False, "mag": False}
        stopped = {"baro": False, "mag": False}
        for row in rows:
            status = int(row["control_status"])
            for name, bit in [("baro", 9), ("mag", 35)]:
                active = bool(status & (1 << bit))
                if seen[name] and not active: stopped[name] = True
                seen[name] |= active
        if case == "dropout" and stopped["baro"] and int(rows[-1]["control_status"]) & (1 << 9):
            return {"baro_stopped_and_recovered": True, "mag_stop_not_exercised": not stopped["mag"]}
        if not all(stopped.values()):
            raise ValueError("dropout fixture did not stop both aid sources")
        last = int(rows[-1]["control_status"])
        if not last & (1 << 9) or not last & (1 << 35):
            raise ValueError("aid sources did not recover after dropout")
        return {"both_sources_stopped_and_recovered": True}
    return {}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT / "target/ekf2-host-baro-mag")
    parser.add_argument("--record-baseline", action="store_true")
    parser.add_argument("--record-nis-baseline", action="store_true")
    parser.add_argument("--rust", type=Path)
    parser.add_argument("--case", choices=("nominal", "outliers", "dropout", "dropout_after_alignment"), default="nominal")
    args = parser.parse_args()
    baseline = BASELINE if args.case == "nominal" else BASELINE.parent / f"baro_mag_{args.case}"
    nis_baseline = NIS_BASELINE if args.case == "nominal" else baseline
    configuration = CONFIGURATION if args.case == "nominal" else CONFIGURATION | {"failure_case": args.case}
    duration = 12.0 if args.case == "dropout_after_alignment" else 10.0
    if duration != 10.0: configuration = configuration | {"seconds": duration}
    try:
        manifest = json.loads((args.root / "source_manifest.json").read_text(encoding="utf-8"))
        executable = args.root / "build/golden_cpp"
        if (manifest["profile"] != "baro_mag" or manifest["px4_ref"] != DEFAULT_REF
                or hashlib.sha256(executable.read_bytes()).hexdigest() != manifest["executable_sha256"]):
            raise ValueError("E2 reference executable/profile mismatch")
        output = args.root / ("check" if args.case == "nominal" else f"check-{args.case}")
        output.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(prefix="e2-", dir=output) as temporary:
            directory = Path(temporary)
            expected = write_synthetic(directory / "imu.csv", directory / "truth.csv", seconds=duration)
            with (directory / "aids.csv").open("x", newline="", encoding="utf-8") as file:
                writer = csv.writer(file, lineterminator="\n")
                writer.writerow(FIELDS)
                writer.writerows(samples(case=args.case, seconds=duration))
            reference = run_json([executable.resolve(), directory / "imu.csv", directory / "golden.csv", directory / "aids.csv"])
            inspection = check_golden(directory / "golden.csv")
            with (directory / "golden.csv").open(encoding="utf-8") as file:
                last = None
                for row in csv.DictReader(file):
                    last = row
            status = int(last["control_status"])
            flags = {name: bool(status & (1 << bit)) for name, bit in
                     [("tilt_aligned", 0), ("yaw_aligned", 1), ("baro_fused", 9), ("mag_fused", 35)]}
            expected_aids = int(duration * 50) - (50 if args.case in ("dropout", "dropout_after_alignment") else 0)
            if reference["samples"] != expected or reference["aid_samples"] != expected_aids or not all(flags.values()):
                raise ValueError(f"E2 reference did not consume/align/fuse expected inputs: {flags}")
            report = {"schema": 1, "px4_ref": DEFAULT_REF, "stage": "E2-H reference preparation", "cpp": reference, "inspection": inspection,
                      "last_flags": flags, "rust_qualification": "not exercised in reference-only mode", "hardware": "not run"}
            files = {name: directory / name for name in ("imu.csv", "truth.csv", "aids.csv", "golden.csv")}
            if args.record_baseline:
                record_baseline(baseline, files, report, manifest, configuration)
            if baseline.exists():
                report["cpp_regression"] = compare_baseline(baseline, files, manifest, configuration)
            aid_executable = args.root / "build/golden_cpp_aids"
            if hashlib.sha256(aid_executable.read_bytes()).hexdigest() != manifest["aids_executable_sha256"]:
                raise ValueError("aid status executable checksum differs")
            run_json([aid_executable.resolve(), directory / "imu.csv", directory / "nis.csv", directory / "aids.csv"])
            report["failure_events"] = check_failure_events(directory / "nis.csv", args.case)
            if args.record_nis_baseline:
                record_baseline(nis_baseline, files | {"nis.csv": directory / "nis.csv"}, report, manifest, configuration)
            if nis_baseline.exists():
                stored = json.loads((nis_baseline / "manifest.json").read_text())
                compressed = (nis_baseline / "nis.csv.gz").read_bytes()
                info = stored["artifacts"]["nis.csv.gz"]
                if hashlib.sha256(compressed).hexdigest() != info["sha256"] or hashlib.sha256(gzip.decompress(compressed)).hexdigest() != info["raw_sha256"]:
                    raise ValueError("aid status baseline checksum mismatch")
                report["nis_regression"] = compare_nis(directory / "nis.csv", nis_baseline / "nis.csv.gz")
            if args.rust:
                report["stage"] = "E2-H numerical qualification"
                rust = run_json([args.rust.resolve(), directory / "imu.csv", directory / "rust.csv", directory / "aids.csv"])
                report["rust_qualification"] = compare_outputs(directory / "rust.csv", directory / "golden.csv")
                report["rust"] = rust
                run_json([args.rust.resolve(), directory / "imu.csv", directory / "rust-nis.csv", directory / "aids.csv", "--nis"])
                report["rust_nis"] = compare_nis(directory / "rust-nis.csv", directory / "nis.csv")
                report["rust_failure_events"] = check_failure_events(directory / "rust-nis.csv", args.case)
                (directory / "rust-nis.csv").replace(output / "rust-nis.csv")
                (directory / "rust.csv").replace(output / "rust.csv")
            for name in ("imu.csv", "truth.csv", "aids.csv", "golden.csv"):
                (directory / name).replace(output / name)
            (directory / "nis.csv").replace(output / "nis.csv")
            (output / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, indent=2))
    except (OSError, ValueError, KeyError) as error:
        print(f"check_ekf2_baro_mag: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
