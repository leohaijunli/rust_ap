import csv
import tempfile
import unittest
from pathlib import Path

from ekf2_resets import motion, write_inputs
from check_ekf2_resets import compare_resets
from ekf2_replay import check_golden


class ResetInputTests(unittest.TestCase):
    def test_fault_injection_allows_only_explicit_fault_bits_without_weakening_numeric_checks(self):
        import gzip
        baseline = Path(__file__).resolve().parents[1] / "tests/data/ekf2/imu_yaw/golden.csv.gz"
        with gzip.open(baseline, "rt", encoding="utf-8") as file:
            reader = csv.reader(file)
            header, row = next(reader), next(reader)
        with tempfile.TemporaryDirectory() as temporary:
            actual = Path(temporary) / "fault.csv"
            row[header.index("fault_status")] = str(1 << 10)
            actual.write_text(",".join(header) + "\n" + ",".join(row) + "\n", encoding="utf-8")
            with self.assertRaises(ValueError): check_golden(actual)
            self.assertEqual(check_golden(actual, allowed_fault_mask=(1 << 10) | (1 << 11))["steps"], 1)
            row[header.index("fault_status")] = str(1 << 3)
            actual.write_text(",".join(header) + "\n" + ",".join(row) + "\n", encoding="utf-8")
            with self.assertRaises(ValueError): check_golden(actual, allowed_fault_mask=(1 << 10) | (1 << 11))

    def test_reset_comparison_rejects_counter_delta_and_nan_quaternion_changes(self):
        import gzip
        baseline = Path(__file__).resolve().parents[1] / "tests/data/ekf2/resets_gnss_reset/resets.csv.gz"
        with gzip.open(baseline, "rt", encoding="utf-8") as file:
            reader = csv.reader(file)
            header, row = next(reader), next(reader)
        with tempfile.TemporaryDirectory() as temporary:
            expected, actual = Path(temporary) / "expected.csv", Path(temporary) / "actual.csv"
            text = ",".join(header) + "\n" + ",".join(row) + "\n"
            expected.write_text(text, encoding="utf-8")
            actual.write_text(text, encoding="utf-8")
            self.assertEqual(compare_resets(actual, expected)["rows"], 1)
            for name, value in (("height_resets", "5"), ("delta_height", "100"), ("delta_q_w", "NaN")):
                changed = row.copy()
                changed[header.index(name)] = value
                actual.write_text(",".join(header) + "\n" + ",".join(changed) + "\n", encoding="utf-8")
                with self.subTest(name=name), self.assertRaises(ValueError): compare_resets(actual, expected)

    def test_motion_and_fault_windows_are_explicit_and_deterministic(self):
        self.assertEqual(motion(12_000_000), (0.0, 0.0))
        self.assertEqual(motion(16_000_000), (4.0, 8.0))
        for case in ("gnss_reset", "baro_reset", "gsf_yaw", "vertical_clipping"):
            with tempfile.TemporaryDirectory() as temporary:
                directory = Path(temporary)
                self.assertEqual(write_inputs(directory, case), 12000)
                with (directory / "aids.csv").open() as file: aids = list(csv.DictReader(file))
                with (directory / "imu.csv").open() as file: imu = list(csv.DictReader(file))
            self.assertEqual(len(aids), 1500)
            if case == "gnss_reset":
                self.assertTrue(any(float(row["gps_vn_m_s"]) == 20.0 for row in aids))
            elif case == "baro_reset":
                self.assertTrue(any(float(row["baro_altitude_m"]) == 150.0 for row in aids))
                self.assertEqual(float(aids[-1]["baro_altitude_m"]), 100.0)
            elif case == "gsf_yaw":
                self.assertEqual(float(aids[-1]["gps_vn_m_s"]), 4.0)
                self.assertTrue(any(float(row["delta_velocity_x_m_s"]) == 0.0025 for row in imu))
            else:
                self.assertTrue(any(int(row["delta_velocity_clipping"]) == 4 for row in imu))
                self.assertEqual(int(imu[-1]["delta_velocity_clipping"]), 0)


if __name__ == "__main__":
    unittest.main()
