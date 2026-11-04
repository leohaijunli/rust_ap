import csv
import math
import struct
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import ekf2_replay as replay
from golden_cpp.build import aid_header


def vehicle_log(**updates):
    data = {
        "timestamp_sample": [2500, 5000], "delta_angle_dt": [2500, 2500],
        "delta_velocity_dt": [2500, 2500], "delta_angle_clipping": [0, 0],
        "delta_velocity_clipping": [0, 5], "accel_device_id": [1, 1], "gyro_device_id": [2, 2],
        "accel_calibration_count": [0, 0], "gyro_calibration_count": [0, 0],
    }
    for kind in ("angle", "velocity"):
        for axis in range(3):
            data[f"delta_{kind}[{axis}]"] = [0.01 * axis, 0.02 * axis]
    data.update(updates)
    return SimpleNamespace(data_list=[SimpleNamespace(name="vehicle_imu", multi_id=0, data=data)])


def combined_log(**updates):
    data = {"timestamp": [2500, 5000], "gyro_integral_dt": [1000, 1000],
            "accelerometer_integral_dt": [2000, 2000], "accelerometer_timestamp_relative": [-100, -100],
            "accelerometer_clipping": [0, 5], "gyro_clipping": [0, 0],
            "accel_calibration_count": [0, 0], "gyro_calibration_count": [0, 0]}
    for kind in ("gyro_rad", "accelerometer_m_s2"):
        for axis in range(3):
            data[f"{kind}[{axis}]"] = [0.1 * (axis + 1), 0.2 * (axis + 1)]
    data.update(updates)
    selection = {"timestamp": [1000], "accel_device_id": [1], "gyro_device_id": [2]}
    return SimpleNamespace(data_list=[SimpleNamespace(name="sensor_combined", multi_id=0, data=data),
                                     SimpleNamespace(name="sensor_selection", multi_id=0, data=selection)])


class InputTests(unittest.TestCase):
    def test_synthetic_stationary_and_yaw_trajectory_has_si_units_and_truth(self):
        samples = list(replay.synthetic_imu(seconds=1.2, rate_hz=100, yaw_rate_deg_s=10))
        self.assertEqual(len(samples), 120)
        first, _ = samples[0]
        last, truth = samples[-1]
        self.assertEqual(first["timestamp_us"], 10000)
        self.assertEqual(first["delta_angle_z_rad"], 0.0)
        self.assertAlmostEqual(first["delta_velocity_z_m_s"], -replay.GRAVITY * 0.01)
        self.assertAlmostEqual(last["delta_angle_z_rad"], math.radians(10) * 0.01)
        self.assertAlmostEqual(truth["q_z"], math.sin(math.radians(2) / 2))

    def test_noise_is_seeded_and_invalid_time_or_delta_is_rejected(self):
        kwargs = dict(seconds=0.02, gyro_noise_rad_s=0.01, accel_noise_m_s2=0.02)
        self.assertEqual(list(replay.synthetic_imu(seed=3, **kwargs)), list(replay.synthetic_imu(seed=3, **kwargs)))
        self.assertNotEqual(list(replay.synthetic_imu(seed=3, **kwargs)), list(replay.synthetic_imu(seed=4, **kwargs)))
        row, _ = next(replay.synthetic_imu())
        for field, value in [("delta_angle_dt_us", 0), ("delta_velocity_clipping", 8),
                             ("delta_angle_x_rad", float("nan")), ("timestamp_us", -1)]:
            with self.subTest(field=field), self.assertRaises(ValueError):
                replay.imu_row(row | {field: value})
        with self.assertRaises(ValueError):
            replay.imu_row(row, previous=row["timestamp_us"])
        with self.assertRaises(ValueError):
            list(replay.synthetic_imu(rate_hz=333))

    def test_vehicle_imu_preserves_sample_time_intervals_and_clipping(self):
        rows = list(replay.vehicle_imu_rows(vehicle_log()))
        self.assertEqual([row["timestamp_us"] for row in rows], [2500, 5000])
        self.assertEqual(rows[1]["delta_velocity_clipping"], 5)
        self.assertEqual(rows[1]["delta_velocity_z_m_s"], replay.f32(0.04))

    def test_integrated_vehicle_imu_roundtrips_through_a_real_ulog_file(self):
        from pyulog import ULog

        fields = (
            "uint64_t timestamp;uint64_t timestamp_sample;uint32_t accel_device_id;uint32_t gyro_device_id;"
            "float[3] delta_angle;float[3] delta_velocity;uint32_t delta_angle_dt;uint32_t delta_velocity_dt;"
            "uint8_t delta_angle_clipping;uint8_t delta_velocity_clipping;"
            "uint8_t accel_calibration_count;uint8_t gyro_calibration_count;"
        )

        def message(kind, payload):
            return struct.pack("<HB", len(payload), ord(kind)) + payload

        image = b"ULog\x01\x12\x35\x01" + struct.pack("<Q", 0)
        image += message("B", bytes(40))
        image += message("F", ("vehicle_imu:" + fields).encode())
        image += message("A", struct.pack("<BH", 0, 1) + b"vehicle_imu")
        data = vehicle_log().data_list[0].data
        for index in range(2):
            sample_time = data["timestamp_sample"][index]
            vectors = [data[f"delta_{kind}[{axis}]"][index]
                       for kind in ("angle", "velocity") for axis in range(3)]
            payload = struct.pack(
                "<QQII6fIIBBBB", sample_time + 100, sample_time, 1, 2, *vectors,
                2500, 2500, 0, data["delta_velocity_clipping"][index], 0, 0,
            )
            image += message("D", struct.pack("<H", 1) + payload)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "vehicle_imu.ulg"
            path.write_bytes(image)
            log = ULog(str(path))
            self.assertFalse(log.file_corruption)
            rows = list(replay.vehicle_imu_rows(log))
            self.assertEqual([row["timestamp_us"] for row in rows], [2500, 5000])
            self.assertEqual(rows[1]["delta_velocity_clipping"], 5)
            self.assertEqual(rows[1]["delta_angle_z_rad"], replay.f32(0.04))

    def test_replay_refuses_missing_integrals_source_changes_and_gyro_clipping(self):
        cases = [vehicle_log(accel_device_id=[1, 3]), vehicle_log(gyro_calibration_count=[0, 1]),
                 vehicle_log(delta_angle_clipping=[0, 1]), vehicle_log(timestamp_sample=[2500, 2500]),
                 SimpleNamespace(data_list=[])]
        for log in cases:
            with self.subTest(log=log), self.assertRaises(ValueError):
                list(replay.vehicle_imu_rows(log))

    def test_sensor_combined_matches_px4_f32_rate_times_dt(self):
        rows = list(replay.sensor_combined_rows(combined_log()))
        self.assertEqual([row["timestamp_us"] for row in rows], [2500, 5000])
        self.assertEqual(rows[1]["delta_velocity_clipping"], 5)
        self.assertEqual(rows[0]["delta_angle_z_rad"], replay.f32(replay.f32(0.3) * replay.f32(1000 * replay.f32(1e-6))))
        self.assertEqual(rows[0]["delta_velocity_z_m_s"], replay.f32(replay.f32(0.3) * replay.f32(2000 * replay.f32(1e-6))))

    def test_sensor_combined_requires_selection_and_rejects_reset_boundaries(self):
        switched = combined_log()
        switched.data_list[1].data.update(timestamp=[1000, 4000], accel_device_id=[1, 3], gyro_device_id=[2, 2])
        cases = [switched, combined_log(gyro_calibration_count=[0, 1]), combined_log(gyro_clipping=[0, 1]),
                 combined_log(accelerometer_timestamp_relative=[-100, 2**31 - 1]),
                 SimpleNamespace(data_list=combined_log().data_list[:1])]
        for log in cases:
            with self.subTest(log=log), self.assertRaises(ValueError):
                list(replay.sensor_combined_rows(log))

    def test_aid_payload_headers_keep_types_arrays_and_reject_unknown_schema(self):
        header = aid_header("uint64 timestamp\nfloat64 observation\nfloat32[3] innovation\nbool fused", "EstimatorAidSource3d")
        self.assertIn("double observation{};", header)
        self.assertIn("float innovation[3]{};", header)
        with self.assertRaises(ValueError):
            aid_header("Vector3 value", "EstimatorAidSource3d")


class GoldenTests(unittest.TestCase):
    def test_nominal_25_and_error_24_dump_checks_normal_and_failed_paths(self):
        row = dict.fromkeys(replay.GOLDEN_FIELDS, 0)
        row.update(timestamp_us=10000, imu_timestamp_us=12500, state_0=1)
        for axis in range(24):
            row[f"cov_{axis}_{axis}"] = 1.0
        cases = [({}, None), ({"state_0": float("nan")}, "non-finite"),
                 ({"cov_0_0": -1}, "negative"), ({"cov_0_1": 0.1}, "symmetric"),
                 ({"state_0": 0}, "normalized"), ({"fault_status": 1}, "fault")]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "golden.csv"
            for changes, error in cases:
                with self.subTest(changes=changes):
                    with path.open("w", newline="", encoding="utf-8") as output:
                        writer = csv.DictWriter(output, fieldnames=replay.GOLDEN_FIELDS)
                        writer.writeheader()
                        writer.writerow(row | changes)
                    if error:
                        with self.assertRaisesRegex(ValueError, error):
                            replay.check_golden(path)
                    else:
                        self.assertEqual(replay.check_golden(path)["steps"], 1)

    def test_delay_buffer_warmup_plateau_is_preserved_but_later_repeats_fail(self):
        row = dict.fromkeys(replay.GOLDEN_FIELDS, 0)
        row.update(timestamp_us=10000, imu_timestamp_us=12500, state_0=1)
        rows = [row, row | {"imu_timestamp_us": 22500},
                row | {"timestamp_us": 20000, "imu_timestamp_us": 32500}]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "warmup.csv"
            with path.open("w", newline="", encoding="utf-8") as output:
                writer = csv.DictWriter(output, fieldnames=replay.GOLDEN_FIELDS)
                writer.writeheader()
                writer.writerows(rows)
            self.assertEqual(replay.check_golden(path)["warmup_repeated_timestamps"], 1)
            with path.open("a", newline="", encoding="utf-8") as output:
                csv.DictWriter(output, fieldnames=replay.GOLDEN_FIELDS).writerow(
                    row | {"timestamp_us": 20000, "imu_timestamp_us": 42500}
                )
            with self.assertRaisesRegex(ValueError, "after warm-up"):
                replay.check_golden(path)


if __name__ == "__main__":
    unittest.main()
