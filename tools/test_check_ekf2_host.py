import csv
import gzip
import hashlib
import json
import tempfile
import unittest
from pathlib import Path

import check_ekf2_host as check
from ekf2_replay import GOLDEN_FIELDS


def fixture_files(directory):
    files = {name: directory / name for name in ("imu.csv", "truth.csv", "golden.csv")}
    files["imu.csv"].write_text("input fixture\n", encoding="utf-8")
    files["truth.csv"].write_text("truth fixture\n", encoding="utf-8")
    row = dict.fromkeys(GOLDEN_FIELDS, 0)
    row.update(timestamp_us=10000, imu_timestamp_us=12500, state_0=1)
    with files["golden.csv"].open("w", encoding="utf-8", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=GOLDEN_FIELDS)
        writer.writeheader()
        writer.writerow(row)
    return files


class BaselineTests(unittest.TestCase):
    def test_archive_roundtrips_is_immutable_and_rejects_tampering(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            files = fixture_files(directory)
            baseline = directory / "baseline"
            source = {"applied_parameters": {"EKF2_DELAY_MAX": 200.0}}
            report = {"px4_ref": check.DEFAULT_REF}
            check.record_baseline(baseline, files, report, source)
            self.assertEqual(check.compare_baseline(baseline, files, source)["compared_rows"], 1)
            for name, path in files.items():
                self.assertEqual(gzip.decompress((baseline / f"{name}.gz").read_bytes()), path.read_bytes())

            saved = {path.name: path.read_bytes() for path in baseline.iterdir()}
            with self.assertRaisesRegex(ValueError, "never overwrites"):
                check.record_baseline(baseline, files, report, source)
            self.assertEqual(saved, {path.name: path.read_bytes() for path in baseline.iterdir()})
            (baseline / "golden.csv.gz").write_bytes(saved["golden.csv.gz"] + b"tampered")
            with self.assertRaisesRegex(ValueError, "checksum"):
                check.compare_baseline(baseline, files, source)

    def test_reference_manifest_rejects_wrong_binary_before_execution(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "build").mkdir()
            executable = root / "build/golden_cpp"
            executable.write_bytes(b"reference executable fixture")
            manifest = {"px4_ref": check.DEFAULT_REF, "profile": "core_imu", "error_state_count": 24,
                        "nominal_state_count": 25,
                        "executable_sha256": hashlib.sha256(executable.read_bytes()).hexdigest()}
            (root / "source_manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
            self.assertEqual(check.read_reference_manifest(executable), manifest)
            executable.write_bytes(b"wrong binary")
            with self.assertRaisesRegex(ValueError, "fixed-reference"):
                check.read_reference_manifest(executable)


if __name__ == "__main__":
    unittest.main()
