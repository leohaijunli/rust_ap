import csv
import tempfile
import unittest
from pathlib import Path

import check_ekf2_imu as check
from ekf2_replay import GOLDEN_FIELDS


def fixture():
    row = ["1000", "1000", "128", "0", "1", *("0" for _ in range(24))]
    row.extend("1" if i == j else "0" for i in range(24) for j in range(24))
    return row


def write(path, row):
    with path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.writer(output)
        writer.writerow(GOLDEN_FIELDS)
        writer.writerow(row)


class QualificationTests(unittest.TestCase):
    def test_comparison_accepts_quaternion_sign_and_rejects_numeric_and_event_drift(self):
        with tempfile.TemporaryDirectory() as directory:
            actual, reference = Path(directory) / "rust.csv", Path(directory) / "reference.csv"
            baseline = fixture()
            write(reference, baseline)
            row = baseline.copy()
            row[4] = "-1"
            write(actual, row)
            self.assertEqual(check.compare_outputs(actual, reference)["compared_rows"], 1)
            for index, value in [(1, "2000"), (2, "129"), (10, "0.01"), (29, "1.01"), (30, "NaN")]:
                row = baseline.copy()
                row[index] = value
                write(actual, row)
                with self.subTest(index=index), self.assertRaises(ValueError):
                    check.compare_outputs(actual, reference)

    def test_psd_check_rejects_indefinite_and_invalid_zero_variance(self):
        check.require_psd([[1.0, 1.0], [1.0, 1.0]])
        check.require_psd([[0.0, 0.0], [0.0, 1.0]])
        for covariance in [[[1.0, 2.0], [2.0, 1.0]], [[-1.0]], [[0.0, 0.1], [0.1, 1.0]]]:
            with self.subTest(covariance=covariance), self.assertRaises(ValueError):
                check.require_psd(covariance)


if __name__ == "__main__":
    unittest.main()
