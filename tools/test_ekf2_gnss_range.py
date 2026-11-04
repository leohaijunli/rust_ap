import unittest
import csv
import gzip
import tempfile
from pathlib import Path

from ekf2_gnss_range import FIELDS, EXTENDED_FIELDS, observations
from check_ekf2_gnss_range import compare_aids, NIS_BASELINE


class FiveSensorInputTests(unittest.TestCase):
    def test_e3_nis_comparison_rejects_numeric_flags_and_timestamp_drift(self):
        with gzip.open(NIS_BASELINE / "nis.csv.gz", "rt", encoding="utf-8") as file:
            reader = csv.reader(file)
            header, row = next(reader), next(reader)
        with tempfile.TemporaryDirectory() as directory:
            expected, actual = Path(directory) / "expected.csv", Path(directory) / "actual.csv"
            text = ",".join(header) + "\n" + ",".join(row) + "\n"
            expected.write_text(text, encoding="utf-8")
            actual.write_text(text, encoding="utf-8")
            self.assertEqual(compare_aids(actual, expected)["rows"], 1)
            for name in ("range_innovation_0", "range_rejected", "gnss_pos_timestamp_us", "gnss_quality_failure_bits"):
                changed = row.copy()
                i = header.index(name)
                changed[i] = str(float(changed[i]) + 1.0) if name == "range_innovation_0" else str(int(changed[i]) + 1)
                actual.write_text(",".join(header) + "\n" + ",".join(changed) + "\n", encoding="utf-8")
                with self.subTest(name=name), self.assertRaises(ValueError):
                    compare_aids(actual, expected)
    def test_stationary_input_keeps_units_and_independent_gnss_rate(self):
        rows = list(observations())
        self.assertEqual(len(rows), 1000)
        self.assertEqual(sum(row[5] for row in rows), 100)
        self.assertTrue(all(len(row) == len(FIELDS) == 14 for row in rows))
        self.assertEqual(rows[0][6:12], (47.397742, 8.545594, 100.0, 0.0, 0.0, 0.0))
        self.assertEqual(rows[0][12:], (1.0, 100))
        self.assertEqual([row[0] for row in rows if row[5]][:3], [200_000, 400_000, 600_000])

    def test_failure_cases_preserve_the_canonical_times_and_independent_source_gaps(self):
        outliers = list(observations(case="outliers"))
        dropout = list(observations(seconds=30, case="dropout"))
        quality = list(observations(seconds=30, case="quality"))
        self.assertEqual(len(outliers), 1000)
        self.assertTrue(all(len(row) == len(EXTENDED_FIELDS) == 22 for row in outliers + dropout + quality))
        self.assertTrue(any(row[6] > 47.398 and row[9] == 20 for row in outliers))
        self.assertTrue(any(row[12] == 35.0 for row in outliers))
        self.assertFalse(any(row[5] for row in dropout if 12_000_000 < row[0] <= 21_000_000))
        self.assertFalse(any(row[21] for row in dropout if 13_000_000 < row[0] <= 15_000_000))
        self.assertTrue(all(row[14] == 2 for row in quality if 12_000_000 < row[0] <= 20_000_000))
        self.assertTrue(all(row[14] == 3 for row in quality if row[0] > 20_000_000))


if __name__ == "__main__":
    unittest.main()
