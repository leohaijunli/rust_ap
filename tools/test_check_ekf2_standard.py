import math
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from check_ekf2_standard import (
    MESSAGES, check_schema, equal, global_int_fields,
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

    def test_mavlink_crc_and_truncation_are_rejected(self):
        encoder = mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
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


if __name__ == "__main__":
    unittest.main()
