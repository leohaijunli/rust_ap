import contextlib
import io
import json
import os
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from check_mavlink import (
    PortNotReady,
    console_command,
    expected_source_id,
    find_port,
    main,
    wait_for_port,
)
from test_decode_fault_dump import report, valid_words


class MavlinkIdentityTests(unittest.TestCase):
    def test_preserved_baseline_does_not_use_p1_build_identity(self):
        source = expected_source_id("0.1.0", None, Path("unused.json"))
        self.assertEqual(int(source, 16).to_bytes(8, "little"), b"RUSTFMU1")

    def test_explicit_identity_is_validated(self):
        self.assertEqual(expected_source_id("0.2.0", "0123456789ABCDEF", Path("unused.json")), "0123456789abcdef")
        for source in ("bad", "x" * 16, "1" * 17):
            with self.subTest(source=source), self.assertRaises(ValueError):
                expected_source_id("0.2.0", source, Path("unused.json"))

    def test_mavlink_console_reassembles_multi_reply(self):
        link = SimpleNamespace(mav=SimpleNamespace(serial_control_send=Mock()))
        link.recv_match = Mock(side_effect=[
            SimpleNamespace(device=10, count=5, data=b"LOGGE" + bytes(65), flags=16),
            SimpleNamespace(device=10, count=2, data=b"R\n" + bytes(68), flags=1),
        ])
        self.assertEqual(console_command(link, "logger status"), "LOGGER\n")
        args = link.mav.serial_control_send.call_args.args
        self.assertEqual(args[:5], (10, 2, 0, 0, len(b"logger status\n")))


def usb_port(mode, device="/dev/ttyACM_TEST0"):
    vid, pid, serial, product = {
        "mavlink": (0x1209, 0x0001, "V6X-RUST-001", "V6X Rust CDC"),
        "dump": (0x1209, 0x0001, "V6X-RUST-DUMP", "V6X Rust Dump"),
        "bootloader": (0x3185, 0x0035, "0", "PX4 BL FMU v6X.x"),
        "other": (0x0403, 0x6001, "OTHER", "USB serial adapter"),
    }[mode]
    return SimpleNamespace(device=device, vid=vid, pid=pid, serial_number=serial,
                           product=product, description=product)


class MonitorTests(unittest.TestCase):
    """Exercise CLI routing and capture with mocked enumeration/serial only."""

    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.target = Path(self.directory.name)
        self.ports = self.enterContext(patch("serial.tools.list_ports.comports", return_value=[]))
        self.serial = self.enterContext(patch("serial.Serial"))
        self.connection = self.enterContext(patch("check_mavlink.mavutil.mavlink_connection"))
        self.enterContext(patch("check_mavlink.DEFAULT_WAIT", 0))
        self.enterContext(patch.dict(os.environ, {"CARGO_TARGET_DIR": str(self.target)}))
        self.stdout, self.stderr = io.StringIO(), io.StringIO()
        self.enterContext(contextlib.redirect_stdout(self.stdout))
        self.enterContext(contextlib.redirect_stderr(self.stderr))

    def no_connection(self):
        self.serial.assert_not_called()
        self.connection.assert_not_called()
        self.assertNotIn("Traceback", self.stderr.getvalue())

    def test_p1_capture_discovery_stays_normal_only(self):
        self.ports.return_value = [usb_port("dump"), usb_port("mavlink", "/dev/ttyACM_TEST1")]
        self.assertEqual(find_port(), "/dev/ttyACM_TEST1")
        self.ports.return_value = [usb_port("dump")]
        with self.assertRaisesRegex(RuntimeError, "V6X Rust Dump.*--monitor 10"):
            find_port()
        self.no_connection()

    def test_no_device_and_bootloader_have_actionable_errors(self):
        for ports, text in (([], "Detected: none"), ([usb_port("bootloader")], "PX4 bootloader"),
                            ([usb_port("other")], "USB serial adapter")):
            with self.subTest(ports=ports):
                self.ports.return_value = ports
                self.assertEqual(main(["--monitor", "10"]), 1)
                self.assertIn(text, self.stderr.getvalue())
                self.no_connection()

    def test_multiple_candidates_require_an_explicit_port(self):
        self.ports.return_value = [usb_port("mavlink"), usb_port("dump", "/dev/ttyACM_TEST1")]
        self.assertEqual(main(["--monitor", "10"]), 1)
        self.assertIn("Multiple Rust CDC devices", self.stderr.getvalue())
        self.assertIn("--port", self.stderr.getvalue())
        self.assertIn("/dev/ttyACM_TEST1", self.stderr.getvalue())
        self.no_connection()

    def test_explicit_dump_and_by_id_alias_use_dump_path(self):
        self.ports.return_value = [usb_port("dump"), usb_port("mavlink", "/dev/ttyACM_TEST1")]
        alias = self.target / "by-id-dump"
        alias.symlink_to("/dev/ttyACM_TEST0")
        for device in ("/dev/ttyACM_TEST0", str(alias)):
            with self.subTest(device=device), patch("check_mavlink.monitor_dump") as capture:
                self.assertEqual(main(["--port", device, "--monitor", "10"]), 0)
                self.assertEqual(capture.call_args.args[:2], ("/dev/ttyACM_TEST0", 10.0))
        self.no_connection()

    def test_explicit_bootloader_and_missing_port_are_rejected(self):
        self.ports.return_value = [usb_port("bootloader")]
        for device, error in (("/dev/ttyACM_TEST0", "PX4 bootloader"),
                              ("/dev/ttyACM_MISSING", "not found")):
            with self.subTest(device=device):
                self.assertEqual(main(["--port", device, "--monitor", "10"]), 1)
                self.assertIn(error, self.stderr.getvalue())
        self.no_connection()

    def test_explicit_serial_adapter_preserves_mavlink_override(self):
        self.ports.return_value = [usb_port("other")]
        with patch("check_mavlink.monitor") as monitor:
            self.assertEqual(main(["--port", "/dev/ttyACM_TEST0", "--monitor", "10"]), 0)
        monitor.assert_called_once_with(self.connection.return_value, 10.0)
        self.connection.return_value.close.assert_called_once()
        self.serial.assert_not_called()

    def test_dump_default_check_does_not_claim_mavlink_success(self):
        self.ports.return_value = [usb_port("dump")]
        self.assertEqual(main([]), 1)
        self.assertIn("has no MAVLink; use --monitor 10", self.stderr.getvalue())
        self.no_connection()

    def test_dump_monitor_reads_fragments_saves_exact_bytes_and_decodes(self):
        self.ports.return_value = [usb_port("dump")]
        wire = b"\xff\n" + report(valid_words(kind=1)).encode()
        device = self.serial.return_value.__enter__.return_value
        device.read.side_effect = [wire[:41], wire[41:380], wire[380:]]
        with (patch("decode_fault_dump.time.monotonic", side_effect=[0, 0, 0.1, 0.2, 10]),
              patch("check_mavlink.expected_source_id", side_effect=AssertionError("normal build identity read"))):
            self.assertEqual(main(["--monitor", "10"]), 0)
        captures = list((self.target / "fault-dumps").glob("*.txt"))
        self.assertEqual(len(captures), 1)
        self.assertEqual(captures[0].read_bytes(), wire)
        self.assertIn("retained fault: valid=YES kind=1 (PANIC)", self.stdout.getvalue())
        self.assertIn("no MAVLink/QGC heartbeat", self.stdout.getvalue())
        self.assertIn(str(captures[0]), self.stdout.getvalue())
        self.serial.assert_called_once_with("/dev/ttyACM_TEST0", 115200, timeout=0.2)
        device.write.assert_not_called()
        self.connection.assert_not_called()
        self.assertEqual(self.stderr.getvalue(), "")

    def test_empty_partial_and_invalid_fault_captures_have_distinct_results(self):
        self.ports.return_value = [usb_port("dump")]
        damaged = valid_words()
        damaged[6] ^= 1
        cases = [
            ("", 1, "Dump CDC opened but received 0 bytes"),
            ("unrelated serial data\n", 1, "No V6X_DUMP record found"),
            (report(valid_words(), complete=False), 1, "Incomplete V6X_DUMP report"),
            (report(damaged), 0, "retained fault: valid=NO"),
        ]
        device = self.serial.return_value.__enter__.return_value
        for index, (text, status, message) in enumerate(cases):
            with self.subTest(index=index):
                raw = self.target / f"capture-{index}.txt"
                device.read.return_value = text.encode()
                with patch("decode_fault_dump.time.monotonic", side_effect=[0, 0, 10]):
                    self.assertEqual(main(["--monitor", "10", "--raw", str(raw)]), status)
                self.assertEqual(raw.read_bytes(), text.encode())
                self.assertIn(message, self.stdout.getvalue() + self.stderr.getvalue())
        device.write.assert_not_called()
        self.connection.assert_not_called()
        self.assertNotIn("Traceback", self.stderr.getvalue())

    def test_usb_disconnect_preserves_partial_capture(self):
        self.ports.return_value = [usb_port("dump")]
        raw = self.target / "interrupted.txt"
        device = self.serial.return_value.__enter__.return_value
        device.read.side_effect = [b"V6X_DUMP schema=1\r\n", OSError("USB disconnected")]
        with patch("decode_fault_dump.time.monotonic", side_effect=[0, 0, 0.1]):
            self.assertEqual(main(["--monitor", "10", "--raw", str(raw)]), 1)
        self.assertEqual(raw.read_bytes(), b"V6X_DUMP schema=1\r\n")
        self.assertIn("USB disconnected", self.stderr.getvalue())
        self.serial.return_value.__exit__.assert_called_once()
        self.connection.assert_not_called()

    def test_existing_capture_is_not_overwritten(self):
        self.ports.return_value = [usb_port("dump")]
        raw = self.target / "existing.txt"
        raw.write_text("previous capture")
        self.assertEqual(main(["--monitor", "10", "--raw", str(raw)]), 1)
        self.assertEqual(raw.read_text(), "previous capture")
        self.no_connection()

    def test_normal_check_keeps_firmware_identity_validation(self):
        self.ports.return_value = [usb_port("mavlink")]
        with patch("check_mavlink.check") as check:
            self.assertEqual(main(["--expected-version", "0.2.0", "--source-id", "0123456789ABCDEF"]), 0)
        link = self.connection.return_value
        check.assert_called_once_with(link, "0.2.0", "0123456789abcdef")
        link.close.assert_called_once()
        self.serial.assert_not_called()

    def test_normal_monitor_requests_and_prints_telemetry(self):
        self.ports.return_value = [usb_port("mavlink")]
        link = self.connection.return_value
        link.recv_match.return_value.get_srcSystem.return_value = 1
        link.recv_match.return_value.get_srcComponent.return_value = 1
        with patch("check_mavlink.time.monotonic", side_effect=[0, 0, 10]):
            self.assertEqual(main(["--monitor", "10"]), 0)
        link.mav.command_long_send.assert_called_once_with(1, 1, 512, 0, 253, 0, 0, 0, 0, 0, 0)
        self.assertIn("sys=1 comp=1", self.stdout.getvalue())
        link.close.assert_called_once()
        self.serial.assert_not_called()

    def test_no_mavlink_telemetry_fails_and_closes_port(self):
        self.ports.return_value = [usb_port("mavlink")]
        link = self.connection.return_value
        link.recv_match.return_value = None
        with patch("check_mavlink.time.monotonic", side_effect=[0, 0, 10]):
            self.assertEqual(main(["--monitor", "10"]), 1)
        self.assertIn("No MAVLink telemetry", self.stderr.getvalue())
        link.close.assert_called_once()

    def test_failed_wire_check_closes_port_without_traceback(self):
        self.ports.return_value = [usb_port("mavlink")]
        with patch("check_mavlink.check", side_effect=AssertionError("firmware mismatch")):
            self.assertEqual(main(["--source-id", "0123456789abcdef"]), 1)
        self.assertIn("firmware mismatch", self.stderr.getvalue())
        self.assertNotIn("Traceback", self.stderr.getvalue())
        self.connection.return_value.close.assert_called_once()

    def test_serial_open_error_is_reported_without_traceback(self):
        self.ports.return_value = [usb_port("mavlink")]
        self.connection.side_effect = OSError("Permission denied")
        self.assertEqual(main(["--monitor", "10"]), 1)
        self.assertIn("Permission denied", self.stderr.getvalue())
        self.assertNotIn("Traceback", self.stderr.getvalue())

    def test_bad_duration_is_rejected_before_discovery(self):
        for seconds in ("0", "-1", "nan", "inf", "-inf"):
            with self.subTest(seconds=seconds), self.assertRaises(SystemExit) as exit:
                main([f"--monitor={seconds}"])
            self.assertEqual(exit.exception.code, 2)
        self.ports.assert_not_called()
        self.no_connection()

    def test_raw_option_is_dump_monitor_only(self):
        self.ports.return_value = [usb_port("mavlink")]
        for args in (["--raw", "unused.txt"], ["--monitor", "10", "--raw", "unused.txt"]):
            with self.subTest(args=args), self.assertRaises(SystemExit) as exit:
                main(args)
            self.assertEqual(exit.exception.code, 2)
        self.no_connection()

    def test_wait_tracks_absent_bootloader_then_dump_without_opening(self):
        self.ports.side_effect = [[], [usb_port("bootloader")], [usb_port("dump")]]
        with (patch("check_mavlink.time.monotonic", side_effect=[0, 0.25]),
              patch("check_mavlink.time.sleep") as sleep):
            port = wait_for_port(allow_dump=True, seconds=5)
        self.assertEqual(port.serial_number, "V6X-RUST-DUMP")
        self.assertEqual(self.ports.call_count, 3)
        self.assertEqual(sleep.call_count, 2)
        self.no_connection()

    def test_wait_expires_with_last_bootloader_state(self):
        self.ports.return_value = [usb_port("bootloader")]
        with (patch("check_mavlink.time.monotonic", side_effect=[0, 0.25, 0.5]),
              patch("check_mavlink.time.sleep") as sleep,
              self.assertRaisesRegex(PortNotReady, "PX4 bootloader")):
            wait_for_port(allow_dump=True, seconds=0.5)
        self.assertEqual(self.ports.call_count, 3)
        self.assertEqual(sleep.call_count, 2)
        self.no_connection()

    def test_ambiguous_devices_are_not_retried(self):
        self.ports.return_value = [usb_port("mavlink"), usb_port("dump", "/dev/ttyACM_TEST1")]
        with (patch("check_mavlink.time.sleep") as sleep,
              self.assertRaisesRegex(RuntimeError, "Multiple Rust CDC")):
            wait_for_port(allow_dump=True, seconds=10)
        sleep.assert_not_called()
        self.assertEqual(self.ports.call_count, 1)
        self.no_connection()

    def test_capture_duration_starts_after_port_wait(self):
        self.ports.side_effect = [[], [usb_port("dump")]]
        wire = report(valid_words()).encode()
        device = self.serial.return_value.__enter__.return_value
        device.read.return_value = wire
        with (patch("check_mavlink.time.monotonic", side_effect=[0, 0.25, 0.25, 10.25]),
              patch("check_mavlink.time.sleep")):
            self.assertEqual(main(["--monitor", "10", "--wait", "1"]), 0)
        device.read.assert_called_once()
        self.assertEqual(next((self.target / "fault-dumps").glob("*.txt")).read_bytes(), wire)
        diagnostics = json.loads(next((self.target / "monitor-logs").glob("*.json")).read_text())
        self.assertEqual(diagnostics["outcome"], "ok")
        self.assertEqual(diagnostics["selected_mode"], "dump")
        self.assertEqual(len(diagnostics["scans"]), 2)
        self.assertEqual(diagnostics["scans"][0]["ports"], [])
        self.assertEqual(diagnostics["scans"][1]["ports"][0]["serial"], "V6X-RUST-DUMP")
        device.write.assert_not_called()
        self.connection.assert_not_called()

    def test_failed_discovery_saves_script_and_device_evidence(self):
        self.ports.return_value = [usb_port("bootloader")]
        self.assertEqual(main(["--monitor", "10"]), 1)
        diagnostics = json.loads(next((self.target / "monitor-logs").glob("*.json")).read_text())
        self.assertEqual(diagnostics["outcome"], "error")
        self.assertIn("PX4 bootloader", diagnostics["error"])
        self.assertEqual(diagnostics["scans"][0]["ports"][0]["mode"], "bootloader")
        self.assertEqual(Path(diagnostics["script"]).name, "check_mavlink.py")
        self.assertRegex(diagnostics["script_sha256"], r"^[0-9a-f]{64}$")
        self.assertIn("Diagnostics:", self.stdout.getvalue())
        self.no_connection()

    def test_list_ports_records_usb_identity_without_opening(self):
        self.ports.return_value = [usb_port("dump")]
        self.assertEqual(main(["--list-ports"]), 0)
        self.assertIn("mode=dump /dev/ttyACM_TEST0", self.stdout.getvalue())
        diagnostics = json.loads(next((self.target / "monitor-logs").glob("*.json")).read_text())
        self.assertEqual(diagnostics["outcome"], "listed")
        self.assertEqual(diagnostics["scans"][0]["ports"][0]["serial"], "V6X-RUST-DUMP")
        self.no_connection()

    def test_invalid_wait_is_rejected_before_discovery(self):
        for seconds in ("-1", "nan", "inf", "-inf"):
            with self.subTest(seconds=seconds), self.assertRaises(SystemExit) as exit:
                main(["--monitor", "10", f"--wait={seconds}"])
            self.assertEqual(exit.exception.code, 2)
        self.ports.assert_not_called()
        self.no_connection()
