import unittest

from pymavlink.dialects.v20 import common as mav2

from check_mavcan import node_info_request, node_status_source, validate_classic


def decoded_frame(*, can_id=0x9801552A, bus=1, length=8, target=250, canfd=False):
    encoder = mav2.MAVLink(None, srcSystem=1, srcComponent=1)
    frame_type = mav2.MAVLink_canfd_frame_message if canfd else mav2.MAVLink_can_frame_message
    data = [1, 0, 0, 0, 0, 0, 0, 0xC3] + ([0] * 56 if canfd else [])
    frame = frame_type(target, 189, bus, length, can_id, data).pack(encoder)
    return mav2.MAVLink(None).parse_char(frame)


class MavcanChecks(unittest.TestCase):
    def test_classic_node_status_survives_mavlink2_wire_encoding(self):
        message = decoded_frame()
        validate_classic(message)
        self.assertEqual(node_status_source(message), 42)
        self.assertEqual(message.id, 0x9801552A)

    def test_old_canfd_encoding_and_missing_extended_flag_fail(self):
        for message in (decoded_frame(canfd=True), decoded_frame(can_id=0x1801552A)):
            with self.subTest(message=message):
                with self.assertRaises(RuntimeError):
                    validate_classic(message)

    def test_wrong_bus_target_dlc_and_reserved_id_fail(self):
        for fields in ({"bus": 0}, {"target": 1}, {"length": 9}, {"can_id": 0xB801552A}):
            with self.subTest(fields=fields):
                with self.assertRaises(RuntimeError):
                    validate_classic(decoded_frame(**fields))

    def test_remote_and_incomplete_frames_do_not_identify_nodes(self):
        for fields in ({"can_id": 0xD801552A}, {"length": 7}):
            with self.subTest(fields=fields):
                message = decoded_frame(**fields)
                validate_classic(message)
                self.assertIsNone(node_status_source(message))

    def test_get_node_info_request_matches_dronecan_v0_wire_vector(self):
        # Priority 24, service 1, request to node 42 from node 127, TID 3.
        self.assertEqual(node_info_request(42, 127, 3), (0x9801AAFF, [0xC3, 0, 0, 0, 0, 0, 0, 0]))


if __name__ == "__main__":
    unittest.main()
