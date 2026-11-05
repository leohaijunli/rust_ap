#!/usr/bin/env python3
"""Replay the split Rust API against committed PX4 evidence, without rebuilding C++."""

import argparse
import gzip
import hashlib
import json
import subprocess
import tempfile
from pathlib import Path

from check_ekf2_baro_mag import compare_nis
from check_ekf2_gnss_range import compare_aids
from check_ekf2_host import ROOT, run_json
from check_ekf2_imu import compare_outputs
from check_ekf2_output import compare_output, inspect_output
from check_ekf2_resets import compare_resets
from gen_params import DEFAULT_REF

CASES = (
    "imu_yaw", "imu_noisy_clipped",
    "baro_mag_yaw", "baro_mag_outliers", "baro_mag_dropout_after_alignment",
    "gnss_range_stationary", "gnss_range_outliers", "gnss_range_dropout",
    "gnss_range_quality", "gnss_range_lever_arm",
    "output_nominal", "output_jitter",
    "resets_gnss_reset", "resets_baro_reset", "resets_gsf_yaw", "resets_vertical_clipping",
)


def artifacts(directory):
    manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
    if manifest["px4_ref"] != DEFAULT_REF or manifest["reference_build"]["px4_ref"] != DEFAULT_REF:
        raise ValueError(f"unpinned reference: {directory}")
    result = {}
    for name, expected in manifest["artifacts"].items():
        if Path(name).name != name or not name.endswith(".gz"):
            raise ValueError(f"invalid artifact name: {name}")
        compressed = (directory / name).read_bytes()
        raw = gzip.decompress(compressed)
        if (hashlib.sha256(compressed).hexdigest() != expected["sha256"]
                or hashlib.sha256(raw).hexdigest() != expected["raw_sha256"]):
            raise ValueError(f"committed artifact checksum differs: {directory / name}")
        result[name[:-3]] = raw
    return manifest, result


def check_case(executable, case, destination, entry):
    baseline = ROOT / "tests/data/ekf2" / case
    manifest, files = artifacts(baseline)
    if case in ("baro_mag_yaw", "gnss_range_stationary"):
        _, nis_files = artifacts(baseline.with_name(case + "_nis"))
        if nis_files["imu.csv"] != files["imu.csv"] or nis_files["aids.csv"] != files["aids.csv"]:
            raise ValueError("NIS and state baselines have different inputs")
        files["nis.csv"] = nis_files["nis.csv"]
    flags = ["--split"] if entry == "split" else []
    for name, flag in (("gnss_range_lever_arm", "--lever-arm"),
                       ("resets_baro_reset", "--baro-reset"), ("resets_gsf_yaw", "--gsf-yaw")):
        if case == name:
            flags.append(flag)
    destination.mkdir(parents=True, exist_ok=True)
    report = {"case": case, "entry": entry, "px4_ref": DEFAULT_REF, "hardware": "not run"}
    with tempfile.TemporaryDirectory(dir=destination) as temporary:
        work = Path(temporary)
        for name, data in files.items():
            (work / name).write_bytes(data)

        def run(name, mode=()):
            output = work / ("rust-" + name)
            command = [executable, work / "imu.csv", output]
            if "aids.csv" in files:
                command.append(work / "aids.csv")
            intake = run_json([*command, *mode, *flags])
            expected = manifest.get("cpp", {})
            for field in ("samples", "input_digest"):
                if field in expected and intake[field] != expected[field]:
                    raise ValueError(f"{case}: {field} differs from the recorded input")
            return output, intake

        if case.startswith("output_"):
            output, report["intake"] = run("output.csv", ["--output"])
            report["output"] = compare_output(output, work / "output.csv")
            report["continuity"] = inspect_output(output, work / "output.resets.csv", report["intake"]["samples"])
            output.replace(destination / output.name)
        else:
            output, report["intake"] = run("state.csv")
            mask = (1 << 10) | (1 << 11) if case == "resets_vertical_clipping" else 0
            report["state_covariance"] = compare_outputs(output, work / "golden.csv", allowed_fault_mask=mask)
            output.replace(destination / output.name)
            if "nis.csv" in files:
                output, _ = run("nis.csv", ["--nis"])
                compare = compare_nis if case.startswith("baro_mag_") else compare_aids
                report["innovations"] = compare(output, work / "nis.csv")
                output.replace(destination / output.name)
            if "resets.csv" in files:
                output, _ = run("resets.csv", ["--resets"])
                report["resets"] = compare_resets(output, work / "resets.csv")
                output.replace(destination / output.name)
    report["passed"] = True
    (destination / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rust", type=Path, required=True)
    parser.add_argument("--case", action="append", choices=CASES)
    parser.add_argument("--entry", choices=("split", "legacy"), default="split")
    parser.add_argument("--output", type=Path, default=ROOT / "target/ekf2-refactor")
    args = parser.parse_args()
    output = args.output.resolve()
    baseline = (ROOT / "tests/data/ekf2").resolve()
    if output == baseline or baseline in output.parents:
        parser.error("outputs must not modify the committed baseline directory")
    executable = args.rust.resolve()
    report = {"schema": 1, "entry": args.entry, "px4_ref": DEFAULT_REF,
              "rust_sha256": hashlib.sha256(executable.read_bytes()).hexdigest(), "results": []}
    output.mkdir(parents=True, exist_ok=True)
    try:
        for case in args.case or CASES:
            result = check_case(executable, case, output / case, args.entry)
            report["results"].append(result)
            print(f"PASS {case}: {args.entry}", flush=True)
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as error:
        report["failure"] = str(error)
        print(f"FAIL {case}: {error}", flush=True)
    report["passed"] = "failure" not in report
    (output / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
