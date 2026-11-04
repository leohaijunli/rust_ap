#!/usr/bin/env python3
"""E5 fixed C++ resets/state/covariance/NIS qualification; never hardware."""

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

from check_ekf2_host import ROOT, record_baseline, compare_baseline, run_json
from check_ekf2_imu import compare_outputs, TOLERANCES
from check_ekf2_gnss_range import compare_aids
from ekf2_replay import check_golden
from ekf2_resets import write_inputs, VERTICAL_FAULT_ACCEL_M_S2
from gen_params import DEFAULT_REF

CONFIGURATION = {"seconds": 30.0, "imu_rate_hz": 400, "gnss_rate_hz": 5, "aid_rate_hz": 50, "seed": 0}


def rows(path):
    with (gzip.open(path, "rt", encoding="utf-8") if path.suffix == ".gz" else path.open(encoding="utf-8")) as file:
        reader = csv.DictReader(file)
        return reader.fieldnames, list(reader)


def compare_resets(actual, expected):
    header, left = rows(actual)
    expected_header, right = rows(expected)
    if header != expected_header or len(header) != 21 or len(left) != len(right):
        raise ValueError("E5 reset schema/count differs")
    for i, (row, reference) in enumerate(zip(left, right)):
        for name in header[:10]:
            if int(row[name]) != int(reference[name]): raise ValueError(f"E5 reset time/status/count differs at row {i + 1}, {name}: {row[name]} vs {reference[name]}")
        q = [float(row[name]) for name in header[10:14]]
        qr = [float(reference[name]) for name in header[10:14]]
        if sum(a * b for a, b in zip(q, qr)) < 0.0: q = [-a for a in q]
        difference = math.sqrt(sum((a - b) ** 2 for a, b in zip(q, qr)))
        if not math.isfinite(difference) or difference > TOLERANCES["quaternion_norm"]:
            raise ValueError(f"E5 reset quaternion differs at row {i + 1}")
        for name in header[14:]:
            a, b = float(row[name]), float(reference[name])
            if not math.isfinite(a) or not math.isfinite(b) or not math.isclose(a, b, abs_tol=TOLERANCES["state_abs"], rel_tol=TOLERANCES["state_relative"]):
                raise ValueError(f"E5 reset delta differs at row {i + 1}, {name}: {a} vs {b}")
    return {"rows": len(left), "tolerances": TOLERANCES}


def reset_events(path, case):
    _, data = rows(path)
    changes = []
    names = ("quat_resets", "velocity_ne_resets", "velocity_d_resets", "position_ne_resets", "height_resets", "hagl_resets")
    for before, after in zip(data, data[1:]):
        changed = [name for name in names if before[name] != after[name]]
        if changed: changes.append({"time_us": int(after["timestamp_us"]), "counts": changed})
    if case == "gnss_reset":
        for name in ("velocity_ne_resets", "position_ne_resets"):
            if sum(name in change["counts"] and change["time_us"] > 12_000_000 for change in changes) < 2:
                raise ValueError(f"E5 did not exercise timeout reset and recovery: {name}")
    if case == "baro_reset":
        if sum("height_resets" in change["counts"] and change["time_us"] > 12_000_000 for change in changes) < 2:
            raise ValueError("E5 did not exercise reference-height timeout resets")
    if case == "gsf_yaw":
        if not any("quat_resets" in change["counts"] and change["time_us"] > 12_000_000 for change in changes):
            raise ValueError("E5 GSF yaw never aligned/reset without magnetometer")
        if not int(data[-1]["control_status"]) & (1 << 44): raise ValueError("E5 GSF yaw case did not start GNSS velocity aiding")
    if case == "vertical_clipping":
        if not any(int(row["fault_status"]) & (1 << 10) for row in data):
            raise ValueError("E5 vertical acceleration fault was not exercised")
        if int(data[-1]["fault_status"]) & (1 << 10):
            raise ValueError("E5 vertical acceleration fault did not recover")
    return changes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", choices=("gnss_reset", "baro_reset", "gsf_yaw", "vertical_clipping"), default="gnss_reset")
    parser.add_argument("--root", type=Path)
    parser.add_argument("--record-baseline", action="store_true")
    parser.add_argument("--rust", type=Path)
    args = parser.parse_args()
    fixture = "nominal" if args.case in ("gnss_reset", "vertical_clipping") else args.case
    root = args.root or ROOT / ("target/ekf2-host-resets" if fixture == "nominal" else f"target/ekf2-host-gnss-range-{fixture.replace('_', '-')}")
    baseline = ROOT / f"tests/data/ekf2/resets_{args.case}"
    configuration = CONFIGURATION | {"case": args.case}
    if args.case == "vertical_clipping":
        configuration["synthetic_vertical_accel_error_m_s2"] = VERTICAL_FAULT_ACCEL_M_S2
    allowed_fault_mask = (1 << 10) | (1 << 11) if args.case == "vertical_clipping" else 0
    try:
        manifest = json.loads((root / "source_manifest.json").read_text(encoding="utf-8"))
        if manifest["px4_ref"] != DEFAULT_REF or manifest["profile"] != "gnss_range" or manifest.get("fixture", "nominal") != fixture:
            raise ValueError("E5 reference source/profile/fixture differs")
        binaries = {"golden.csv": ("golden_cpp", "executable_sha256"), "nis.csv": ("golden_cpp_aids", "aids_executable_sha256"), "resets.csv": ("golden_cpp_resets", "resets_executable_sha256")}
        for name, key in binaries.values():
            if hashlib.sha256((root / "build" / name).read_bytes()).hexdigest() != manifest[key]: raise ValueError("E5 reference executable checksum differs")
        output = root / f"check-{args.case}"
        output.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(prefix="e5-", dir=output) as temporary:
            directory = Path(temporary)
            count = write_inputs(directory, args.case)
            for name, (binary, _) in binaries.items():
                result = run_json([root / "build" / binary, directory / "imu.csv", directory / name, directory / "aids.csv"])
            report = {"schema": 1, "stage": "E5-H reference preparation", "px4_ref": DEFAULT_REF, "cpp": result,
                "inspection": check_golden(directory / "golden.csv", allowed_fault_mask=allowed_fault_mask), "events": reset_events(directory / "resets.csv", args.case), "hardware": "not run", "allowed_fault_mask": allowed_fault_mask}
            if result["samples"] != count: raise ValueError("E5 input count differs")
            files = {name: directory / name for name in ("imu.csv", "truth.csv", "aids.csv", *binaries)}
            if args.record_baseline: record_baseline(baseline, files, report, manifest, configuration)
            report["cpp_regression"] = compare_baseline(baseline, {name: files[name] for name in ("imu.csv", "truth.csv", "aids.csv", "golden.csv")}, manifest, configuration, allowed_fault_mask=allowed_fault_mask)
            stored = json.loads((baseline / "manifest.json").read_text(encoding="utf-8"))
            for name in ("nis.csv", "resets.csv"):
                data = (baseline / f"{name}.gz").read_bytes()
                info = stored["artifacts"][f"{name}.gz"]
                if hashlib.sha256(data).hexdigest() != info["sha256"] or hashlib.sha256(gzip.decompress(data)).hexdigest() != info["raw_sha256"]: raise ValueError("E5 baseline checksum drift")
            report["cpp_nis"] = compare_aids(files["nis.csv"], baseline / "nis.csv.gz")
            report["cpp_resets"] = compare_resets(files["resets.csv"], baseline / "resets.csv.gz")
            if args.rust:
                fixture_flag = [] if fixture == "nominal" else [f"--{fixture.replace('_', '-')}" ]
                for name, flags in {"rust.csv": [], "rust-nis.csv": ["--nis"], "rust-resets.csv": ["--resets"]}.items():
                    run_json([args.rust.resolve(), directory / "imu.csv", directory / name, directory / "aids.csv", *flags, *fixture_flag])
                    (directory / name).replace(output / name)
                for name in binaries: (directory / name).replace(output / name)
                report["rust_state"] = compare_outputs(output / "rust.csv", output / "golden.csv", allowed_fault_mask=allowed_fault_mask)
                report["rust_nis"] = compare_aids(output / "rust-nis.csv", output / "nis.csv")
                report["rust_resets"] = compare_resets(output / "rust-resets.csv", output / "resets.csv")
                report["rust_events"] = reset_events(output / "rust-resets.csv", args.case)
                report["stage"] = "E5-H numerical qualification"
            for name, path in files.items():
                if path.exists(): path.replace(output / name)
            (output / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, indent=2))
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as error:
        print(f"check_ekf2_resets: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
