import math
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from check_ekf2_standard import (
    MESSAGES, check_covariance, check_frames, check_schema, equal, global_int_fields,
    mavlink, px4_fields, read_mavlink,
)


class StandardOutputTests(unittest.TestCase):
    def test_complete_pinned_schemas_reject_renamed_or_mistyped_fields(self):
        formats = {name: SimpleNamespace(fields=px4_fields(filename)) for name, filename in MESSAGES.items()}
        log = SimpleNamespace(message_formats=formats)
        check_schema(log)
        formats["vehicle_local_position"].fields[2] = ("uint8_t", 0, "xy_valid")
        with self.assertRaisesRegex(ValueError, "field order/type/array"):
            check_schema(log)

    def test_two_dimensional_gnss_observation_is_double_and_dimensions_stay_distinct(self):
        self.assertIn(("double", 2, "observation"), px4_fields("EstimatorAidSource2d.msg"))
        self.assertIn(("float", 25, "states"), px4_fields("EstimatorStates.msg"))
        self.assertIn(("float", 24, "covariances"), px4_fields("EstimatorStates.msg"))

    def test_unknown_and_unbounded_are_not_accepted_as_finite_zero(self):
        equal(math.nan, math.nan, "unknown")
        equal(math.inf, math.inf, "unbounded")
        for actual, expected in ((0, math.nan), (0, math.inf), (-math.inf, math.inf), (math.nan, 0)):
            with self.assertRaises(ValueError):
                equal(actual, expected, "invalid substitution")

    def test_integer_flags_and_microseconds_are_exact_not_float_tolerant(self):
        for actual, expected in (((1 << 44) | 1, 1 << 44), (65_000_001, 65_000_000)):
            with self.assertRaises(ValueError):
                equal(actual, expected, "integer field")

    def test_global_double_coordinates_reject_loss_of_precision(self):
        with self.assertRaises(ValueError):
            equal(47.397743, 47.397742987, "latitude", rel_tol=0, abs_tol=1e-10)

    def test_global_integer_units_truncate_instead_of_rounding(self):
        actual = global_int_fields(
            {"lat": 47.397742987, "lon": -8.545594987, "alt": 100.125},
            {"vx": 1.239, "vy": -1.239, "vz": 0.125, "heading": 0.0}, home_altitude=90.0,
        )
        self.assertEqual(actual, {"lat": 473977429, "lon": -85455949, "alt": 100125,
                                  "relative_alt": 10125, "vx": 123, "vy": -123, "vz": 12, "hdg": 0})

    def test_unknown_cross_covariance_is_not_zero_filled(self):
        values = [math.nan] * 21
        values[0] = math.inf
        values[6] = math.inf
        values[11] = 4.0
        check_covariance(values, {0: math.inf, 6: math.inf, 11: 4.0}, "covariance")
        values[1] = 0.0
        with self.assertRaises(ValueError):
            check_covariance(values, {0: math.inf, 6: math.inf, 11: 4.0}, "covariance")

    def test_mavlink_crc_and_truncation_are_rejected(self):
        encoder = mavlink.MAVLink(None, srcSystem=1, srcComponent=197)
        message = mavlink.MAVLink_estimator_status_message(1234567, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        frame = bytearray(message.pack(encoder))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "mavlink.bin"
            path.write_bytes(frame)
            self.assertEqual(read_mavlink(path)[0].time_usec, 1234567)
            path.write_bytes(frame[:-1])
            with self.assertRaisesRegex(ValueError, "truncated"):
                read_mavlink(path)
            frame[10] ^= 1
            path.write_bytes(frame)
            with self.assertRaisesRegex(ValueError, "CRC/frame"):
                read_mavlink(path)

    def test_no_gps_rejects_a_global_position_message(self):
        log = SimpleNamespace(data_list=[SimpleNamespace(name=name, data={"timestamp": [100]}) for name in
                                        ("vehicle_attitude", "vehicle_angular_velocity", "vehicle_local_position", "estimator_status")])
        message = SimpleNamespace(get_type=lambda: "GLOBAL_POSITION_INT", get_seq=lambda: 0,
                                  get_srcSystem=lambda: 1, get_srcComponent=lambda: 197)
        with self.assertRaisesRegex(ValueError, "forbidden"):
            check_frames(log, {100: {}}, [message], no_gps=True)


if __name__ == "__main__":
    unittest.main()
