import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from golden_cpp import build
import ekf2_baro_mag


class ProfileTests(unittest.TestCase):
    def test_synthetic_field_tracks_body_yaw_without_changing_strength(self):
        import math

        rows = list(ekf2_baro_mag.samples())
        self.assertEqual(len(rows), 500)
        self.assertEqual(rows[0], (20_000, 100.0, 0.2, -0.0, 0.4))
        yaw = math.radians(45)
        self.assertAlmostEqual(rows[-1][2], 0.2 * math.cos(yaw))
        self.assertAlmostEqual(rows[-1][3], -0.2 * math.sin(yaw))
        self.assertTrue(all(abs(sum(v * v for v in row[2:]) - 0.2) < 1e-12 for row in rows))
        for arguments in [{"rate_hz": 0}, {"seconds": 0}, {"altitude_m": float("nan")}]:
            with self.assertRaises(ValueError):
                list(ekf2_baro_mag.samples(**arguments))

    def test_only_e2_profile_applies_sensor_parameters(self):
        names = set(build.PROFILES["baro_mag"]) | {"EKF2_TAU_POS", "EKF2_TAU_VEL"}
        definitions = [SimpleNamespace(name=name, default=0.25, kind="Float") for name in names]
        imu_header, imu = build.configure_header(definitions)
        e2_header, e2 = build.configure_header(definitions, "baro_mag")
        self.assertNotIn("params.ekf2_mag_noise", imu_header)
        self.assertIn("params.ekf2_mag_noise = 0.25f;", e2_header)
        self.assertEqual(set(e2) - set(imu), set(build.BARO_MAG_PARAMS))

    def test_wrong_profile_is_rejected_before_source_or_cache_changes(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "source_manifest.json"
            saved = json.dumps({"profile": "core_imu"})
            path.write_text(saved, encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "separate output"):
                build.prepare(Path("missing-source"), Path(directory), "baro_mag")
            self.assertEqual(path.read_text(encoding="utf-8"), saved)

    def test_five_sensor_fixture_applies_health_delay_and_preserves_a_separate_lever_arm_cache(self):
        names = set(build.PROFILES["gnss_range"]) | {"EKF2_TAU_POS", "EKF2_TAU_VEL", "EKF2_REQ_GPS_H"}
        definitions = [SimpleNamespace(name=name, default=0.25, kind="Float") for name in names]
        normal, applied = build.configure_header(definitions, "gnss_range")
        lever, lever_applied = build.configure_header(definitions, "gnss_range", "lever_arm")
        self.assertIn("set_min_required_gps_health_time(250000u)", normal)
        self.assertEqual(applied["EKF2_REQ_GPS_H"], 0.25)
        self.assertNotIn("params.gps_pos_body", normal)
        self.assertIn("params.gps_pos_body(0) = 0.2f", lever)
        self.assertEqual(lever_applied["EKF2_IMU_POS_Y"], -0.02)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "source_manifest.json"
            saved = json.dumps({"profile": "gnss_range", "fixture": "nominal"})
            path.write_text(saved, encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "fixture differs"):
                build.prepare(Path("missing-source"), Path(directory), "gnss_range", "lever_arm")
            self.assertEqual(path.read_text(encoding="utf-8"), saved)


if __name__ == "__main__":
    unittest.main()
