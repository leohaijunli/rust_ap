import csv
import tempfile
import unittest
from pathlib import Path

from ekf2_output import write_inputs
from check_ekf2_output import compare_output, CPP_TOLERANCES


class OutputInputTests(unittest.TestCase):
    def test_wrapped_yaw_uses_the_already_fixed_relative_state_criterion(self):
        import gzip
        baseline = Path(__file__).resolve().parents[1] / "tests/data/ekf2/output_nominal/output.csv.gz"
        with gzip.open(baseline, "rt", encoding="utf-8") as file:
            reader = csv.reader(file)
            header, row = next(reader), next(reader)
        i = header.index("unaided_yaw")
        row[i] = "0.758576274"
        with tempfile.TemporaryDirectory() as temporary:
            expected, actual = Path(temporary) / "expected.csv", Path(temporary) / "actual.csv"
            expected.write_text(",".join(header) + "\n" + ",".join(row) + "\n", encoding="utf-8")
            changed = row.copy()
            changed[i] = "0.758575261" # Exact CI discrepancy; relative error remains below 1e-5.
            actual.write_text(",".join(header) + "\n" + ",".join(changed) + "\n", encoding="utf-8")
            self.assertEqual(compare_output(actual, expected, CPP_TOLERANCES)["rows"], 1)
            changed[i] = "0.7585"
            actual.write_text(",".join(header) + "\n" + ",".join(changed) + "\n", encoding="utf-8")
            with self.assertRaises(ValueError): compare_output(actual, expected, CPP_TOLERANCES)

    def test_output_comparison_rejects_time_numeric_and_global_position_errors(self):
        baseline = Path(__file__).resolve().parents[1] / "tests/data/ekf2/output_nominal/output.csv.gz"
        import gzip
        with gzip.open(baseline, "rt", encoding="utf-8") as file:
            reader = csv.reader(file)
            header, row = next(reader), next(reader)
        with tempfile.TemporaryDirectory() as temporary:
            expected, actual = Path(temporary) / "expected.csv", Path(temporary) / "actual.csv"
            text = ",".join(header) + "\n" + ",".join(row) + "\n"
            expected.write_text(text, encoding="utf-8")
            actual.write_text(text, encoding="utf-8")
            self.assertEqual(compare_output(actual, expected)["rows"], 1)
            for name in ("timestamp_us", "velocity_n", "latitude_deg"):
                changed = row.copy()
                i = header.index(name)
                changed[i] = str(int(changed[i]) + 1) if name == "timestamp_us" else str(float(changed[i]) + 0.001)
                actual.write_text(",".join(header) + "\n" + ",".join(changed) + "\n", encoding="utf-8")
                with self.subTest(name=name), self.assertRaises(ValueError):
                    compare_output(actual, expected)

    def test_jitter_preserves_elapsed_time_integrals_and_causal_multi_rate_samples(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            self.assertEqual(write_inputs(directory, "jitter"), 8000)
            with (directory / "imu.csv").open() as file: imu = list(csv.DictReader(file))
            with (directory / "truth.csv").open() as file: truth = list(csv.DictReader(file))
            with (directory / "aids.csv").open() as file: aids = list(csv.DictReader(file))
        self.assertEqual(int(imu[-1]["timestamp_us"]), 20_000_000)
        self.assertEqual(int(truth[-1]["timestamp_us"]), 20_000_000)
        self.assertEqual({int(row["delta_angle_dt_us"]) for row in imu}, {2000, 3000, 2250, 2750})
        self.assertEqual(sum(int(row["gps_valid"]) for row in aids), 100)
        self.assertEqual(len(aids), 1000)
        self.assertTrue(all(int(b["timestamp_us"]) > int(a["timestamp_us"]) for a, b in zip(aids, aids[1:])))
        self.assertTrue(any(int(row["timestamp_us"]) % 20_000 != 0 for row in aids))


if __name__ == "__main__":
    unittest.main()
