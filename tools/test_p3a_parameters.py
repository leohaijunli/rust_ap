import unittest
from unittest.mock import Mock

from pymavlink.dialects.v20 import common as mav2

from p3a_parameters import CAPACITY, START, Link, decode_slot, set_param, slot_records


RECORD = bytes.fromhex("a5525041520001007856341201000000020008000000803f00000040502009ec")


class ParameterAcceptanceTests(unittest.TestCase):
    def test_first_commit_accepts_an_empty_inactive_slot_and_checks_crc(self):
        image = bytearray(CAPACITY)
        image[START:START + len(RECORD)] = RECORD
        records = slot_records(image)
        self.assertEqual(records[0]["values"], (1.0, 2.0))
        self.assertEqual(records[0]["schema"], 0x12345678)
        self.assertIsNone(records[1])
        image[START + 24] ^= 1
        with self.assertRaisesRegex(ValueError, "crc"):
            slot_records(image)
        with self.assertRaisesRegex(ValueError, "length"):
            decode_slot(RECORD[:-1])

    def test_wait_keeps_batched_replies_and_excludes_old_messages(self):
        link = Link.__new__(Link)
        old = mav2.MAVLink_param_value_message(b"LINK_LED_HZ", 1.0, 9, 2, 1)
        first = mav2.MAVLink_param_value_message(b"MAV_SYS_ID", 1.0, 9, 2, 0)
        second = mav2.MAVLink_param_value_message(b"LINK_LED_HZ", 2.0, 9, 2, 1)
        link.messages = [old]
        link.capture = Mock()
        link.capture.poll.side_effect = lambda: link.messages.extend((first, second))
        self.assertIs(link.wait("PARAM_VALUE", start=1), first)
        self.assertIs(link.wait("PARAM_VALUE", lambda m: m.param_id == "LINK_LED_HZ", start=1), second)
        link.capture.poll.assert_called_once()

    def test_invalid_set_warning_must_belong_to_that_request(self):
        link = Link.__new__(Link)
        old_warning = mav2.MAVLink_statustext_message(4, b"PARAM_SET: out of range")
        echo = mav2.MAVLink_param_value_message(b"LINK_LED_HZ", 2.0, 9, 2, 1)
        link.messages = [old_warning]
        link.send = Mock(return_value=1)
        link.wait = Mock(return_value=echo)
        link.drain = Mock()
        self.assertEqual(set_param(link, "LINK_LED_HZ", 0.0), (2.0, []))
        link.messages.append(old_warning)
        self.assertEqual(set_param(link, "LINK_LED_HZ", 0.0), (2.0, [old_warning.text]))


if __name__ == "__main__":
    unittest.main()
