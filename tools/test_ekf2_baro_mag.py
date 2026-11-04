import csv
import tempfile
import unittest
from pathlib import Path

from check_ekf2_baro_mag import compare_nis
from ekf2_baro_mag import samples


class AidRegressionTests(unittest.TestCase):
    def test_failure_inputs_are_explicit_and_preserve_canonical_times(self):
        dropout = list(samples(case="dropout"))
        self.assertEqual(len(dropout), 450)
        self.assertFalse(any(5_000_000 < row[0] <= 6_000_000 for row in dropout))
        aligned = list(samples(seconds=12, case="dropout_after_alignment"))
        self.assertEqual(len(aligned), 550)
        self.assertFalse(any(9_000_000 < row[0] <= 10_000_000 for row in aligned))
        outliers = list(samples(case="outliers"))
        self.assertTrue(any(row[1] == 120.0 for row in outliers))
        self.assertTrue(any(row[2] > 1.0 for row in outliers))

    def test_nis_comparison_rejects_boolean_and_numeric_drift(self):
        header = ["timestamp_us", "imu_timestamp_us", "control_status", "fault_status",
                  "baro_timestamp_us", "baro_innovation", "baro_variance", "baro_ratio", "baro_fused", "baro_rejected", "mag_timestamp_us",
                  *(f"mag_{kind}_{i}" for kind in ("innovation", "variance", "ratio") for i in range(3)), "mag_fused", "mag_rejected"]
        row = [0.0] * 22
        for i in (0, 1, 2, 3, 4, 8, 9, 10, 20, 21): row[i] = 0
        with tempfile.TemporaryDirectory() as directory:
            first, second = Path(directory) / "reference.csv", Path(directory) / "rust.csv"
            def write(path, values):
                with path.open("w", newline="") as file:
                    writer = csv.writer(file)
                    writer.writerow(header)
                    writer.writerow(values)
            write(first, row)
            write(second, row)
            self.assertEqual(compare_nis(second, first)["rows"], 1)
            for index, value in [(8, 1), (5, 0.01), (14, float("nan"))]:
                values = row.copy()
                values[index] = value
                write(second, values)
                with self.assertRaises(ValueError): compare_nis(second, first)


if __name__ == "__main__":
    unittest.main()
