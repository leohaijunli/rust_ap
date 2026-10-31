import copy
import unittest

from pymavlink.dialects.v20 import common as mav2

from p1_capture import Demux, analyze, fields


PLATFORM = {
    "CLOCK": {"registers_ok": "true"},
    "BACKUP": {
        "bdcr_before": "0000811b", "bdcr_after": "0000811b",
        "csr_before": "00000000", "csr_after": "00000000",
        "config_preserved": "true", "bkp_preserved": "true",
    },
}


def sample(second):
    return {
        "sent_ns": second * 1_000_000_000,
        "received_ns": second * 1_000_000_000 + 1_000_000,
        "fields": {
            "P1_METRICS": {"epoch": "1", "now_us": str(second * 1_000_000 + 500), "snapshot_us": str(second * 1_000_000)},
            "PERIOD": {"samples": str(second * 1000), "skipped": "0", "deadline_misses": "0", "scheduling_overruns": "0", "time_errors": "0", "late_max_us": "5"},
            "LATE_HIST": {"le5": str(second * 1000)},
            "EXEC": {"invalid_cycles": "0"},
            "CPU": {"valid": "true", "busy_permille": "100", "idle_calls": "2000", "window_us": "1000000"},
            "STACK": {"at_us": str(second * 1_000_000), "untouched_bytes": "60000", "guard_ok": "true"},
        },
    }


class CaptureTests(unittest.TestCase):
    def test_fragmented_console_and_binary_payload_newlines_are_separated(self):
        encoder = mav2.MAVLink(None)
        packet = mav2.MAVLink_statustext_message(6, b"END\nP1_FAULT valid=true").pack(encoder)
        wire = packet + b"P1_FAULT schema=1 valid=false\nEND\n" + packet
        demux = Demux()
        events = []
        for byte in wire:
            events.extend(demux.feed(bytes([byte])))
        self.assertEqual(events, [("mavlink", packet), ("console", "P1_FAULT schema=1 valid=false"), ("console", "END"), ("mavlink", packet)])

    def test_signed_frame_is_not_mistaken_for_console(self):
        encoder = mav2.MAVLink(None)
        encoder.signing.secret_key = bytes(32)
        encoder.signing.sign_outgoing = True
        packet = mav2.MAVLink_ping_message(10, 1, 0, 0).pack(encoder)
        demux = Demux()
        self.assertEqual(demux.feed(packet[:-1]), [])
        self.assertEqual(demux.feed(packet[-1:] + b"END\n"), [("mavlink", packet), ("console", "END")])

    def test_duplicate_fields_and_response_overflow_fail(self):
        for lines in (["CPU valid=true valid=false"], ["CPU a=1", "CPU a=2"], ["ERR response-overflow"]):
            with self.subTest(lines=lines), self.assertRaises(ValueError):
                fields(lines)

    def test_60_second_window_and_rtt_bounds(self):
        result = analyze([sample(2), sample(67)], PLATFORM)
        self.assertEqual(result["software_measurement_gate"], "pass")
        self.assertEqual(result["period_rate_hz"], 1000)
        self.assertEqual(result["cpu_quantization_bound_permille"], 2)

    def test_bad_metrics_never_pass(self):
        baseline = [sample(2), sample(67)]
        for prefix, key, value in (
            ("PERIOD", "skipped", "1"), ("CPU", "valid", "false"),
            ("STACK", "untouched_bytes", "100"), ("EXEC", "invalid_cycles", "1"),
            ("P1_METRICS", "epoch", "2"), ("PERIOD", "late_max_us", "11"),
        ):
            samples = copy.deepcopy(baseline)
            samples[-1]["fields"][prefix][key] = value
            with self.subTest(prefix=prefix, key=key):
                self.assertNotEqual(analyze(samples, PLATFORM)["software_measurement_gate"], "pass")

    def test_timebase_uncertainty_is_not_silently_a_pass(self):
        samples = [sample(2), sample(67)]
        samples[-1]["received_ns"] += 1_000_000_000
        self.assertTrue(analyze(samples, PLATFORM)["timebase"].startswith("indeterminate"))

    def test_stale_stack_and_integer_cpu_rounding_cannot_pass(self):
        for prefix, key, value in (("STACK", "at_us", "1"), ("CPU", "busy_permille", "497")):
            samples = [sample(2), sample(67)]
            samples[-1]["fields"][prefix][key] = value
            with self.subTest(key=key):
                self.assertNotEqual(analyze(samples, PLATFORM)["software_measurement_gate"], "pass")

    def test_zero_cpu_window_is_rejected(self):
        samples = [sample(2), sample(67)]
        samples[-1]["fields"]["CPU"]["window_us"] = "0"
        with self.assertRaises(ValueError):
            analyze(samples, PLATFORM)

    def test_backup_evidence_survives_mixed_stream_capture(self):
        line = b"BACKUP bdcr_before=0000811b bdcr_after=0000811b csr_before=00000000 csr_after=00000000 config_preserved=true bkp_preserved=true\r\n"
        demux = Demux()
        self.assertEqual(demux.feed(line[:21]), [])
        events = demux.feed(line[21:] + b"END\n")
        self.assertEqual(events[0], ("console", line.decode().strip()))
        self.assertEqual(fields([events[0][1]])["BACKUP"], PLATFORM["BACKUP"])

    def test_missing_or_failed_backup_verification_cannot_pass(self):
        samples = [sample(2), sample(67)]
        self.assertNotEqual(analyze(samples, {"CLOCK": PLATFORM["CLOCK"]})["software_measurement_gate"], "pass")
        for key, value in (
            ("bdcr_after", "0000821b"), ("bdcr_after", "0000810b"),
            ("bdcr_after", "0000011b"), ("bkp_preserved", "false"),
            ("config_preserved", "false"),
        ):
            platform = copy.deepcopy(PLATFORM)
            platform["BACKUP"][key] = value
            with self.subTest(key=key, value=value):
                self.assertNotEqual(analyze(samples, platform)["software_measurement_gate"], "pass")
