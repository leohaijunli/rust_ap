import contextlib
import io
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import serial

import enter_bootloader


def usb_port(kind, device="/dev/ttyACM_TEST0"):
    vid, pid, serial_number, product = {
        "rust": (0x1209, 0x0001, "V6X-RUST-001", "V6X Rust CDC"),
        "px4": (0x3185, 0x0035, "0", "PX4 FMU v6X.x"),
        "bootloader": (0x3185, 0x0035, "0", "PX4 BL FMU v6X.x"),
        "other": (0x0403, 0x6001, "OTHER", "USB serial adapter"),
    }[kind]
    return SimpleNamespace(
        device=device,
        vid=vid,
        pid=pid,
        serial_number=serial_number,
        product=product,
        description=product,
    )


class EnterBootloaderTests(unittest.TestCase):
    def setUp(self):
        self.comports = self.enterContext(
            patch("enter_bootloader.serial.tools.list_ports.comports")
        )
        self.serial = self.enterContext(patch("enter_bootloader.serial.Serial"))
        self.sleep = self.enterContext(patch("enter_bootloader.time.sleep"))
        self.monotonic = self.enterContext(patch("enter_bootloader.time.monotonic"))
        self.stdout = io.StringIO()
        self.stderr = io.StringIO()
        self.enterContext(contextlib.redirect_stdout(self.stdout))
        self.enterContext(contextlib.redirect_stderr(self.stderr))

    def assert_no_hardware_action(self):
        self.serial.assert_not_called()
        self.sleep.assert_not_called()

    def test_rust_cdc_sends_complete_line_and_accepts_disconnect_before_ack(self):
        rust = usb_port("rust")
        bootloader = usb_port("bootloader", "/dev/ttyACM_TEST1")
        self.comports.side_effect = [[rust], [], [bootloader]]
        device = Mock()
        device.write.return_value = len(enter_bootloader.BOOTLOADER_COMMAND)
        device.flush.side_effect = serial.SerialException("device disconnected")
        self.serial.return_value = device
        self.monotonic.side_effect = [0.0, 0.0]

        result = enter_bootloader.prepare_for_upload(timeout=2.0)

        self.assertEqual(result, "bootloader")
        self.serial.assert_called_once_with(
            rust.device,
            115200,
            timeout=0.25,
            write_timeout=2.0,
            exclusive=True,
        )
        device.write.assert_called_once_with(b"\n!bootloader\n")
        device.flush.assert_called_once_with()
        device.close.assert_called_once_with()
        self.sleep.assert_called_once_with(0.25)
        self.assertIn("disconnected after the complete command", self.stdout.getvalue())
        self.assertIn("PX4 bootloader ready", self.stdout.getvalue())

    def test_already_in_bootloader_is_idempotent(self):
        self.comports.return_value = [usb_port("bootloader")]

        self.assertEqual(enter_bootloader.prepare_for_upload(), "bootloader")

        self.assertIn("already ready", self.stdout.getvalue())
        self.assert_no_hardware_action()
        self.monotonic.assert_not_called()

    def test_official_px4_application_is_not_bootloader_and_is_skipped(self):
        self.comports.return_value = [usb_port("px4")]

        self.assertEqual(enter_bootloader.prepare_for_upload(), "px4-application")

        output = self.stdout.getvalue()
        self.assertIn("not bootloader", output)
        self.assertIn("px4_uploader", output)
        self.assert_no_hardware_action()
        self.monotonic.assert_not_called()

    def test_multiple_boards_fail_before_opening_a_port(self):
        self.comports.return_value = [
            usb_port("rust"),
            usb_port("bootloader", "/dev/ttyACM_TEST1"),
        ]

        with self.assertRaises(enter_bootloader.BootloaderTransitionError) as caught:
            enter_bootloader.prepare_for_upload()

        self.assertIn("Multiple FMUv6X", str(caught.exception))
        self.assertIn("/dev/ttyACM_TEST0", str(caught.exception))
        self.assertIn("/dev/ttyACM_TEST1", str(caught.exception))
        self.assert_no_hardware_action()

    def test_permission_error_is_clear(self):
        self.comports.return_value = [usb_port("rust")]
        self.serial.side_effect = PermissionError("permission denied")

        self.assertEqual(enter_bootloader.main(["--wait", "1"]), 1)

        error = self.stderr.getvalue()
        self.assertIn("Cannot open Rust CDC", error)
        self.assertIn("permission denied", error)
        self.assertIn("read/write permission", error)
        self.sleep.assert_not_called()

    def test_serial_write_error_is_clear(self):
        self.comports.return_value = [usb_port("rust")]
        device = Mock()
        device.write.side_effect = serial.SerialException("I/O error")
        self.serial.return_value = device

        self.assertEqual(enter_bootloader.main(["--wait", "1"]), 1)

        self.assertIn("Failed to send the bootloader command", self.stderr.getvalue())
        self.assertIn("I/O error", self.stderr.getvalue())
        device.close.assert_called_once_with()
        self.sleep.assert_not_called()

    def test_timeout_explains_vmware_and_recovery_only_unplug(self):
        rust = usb_port("rust")
        self.comports.side_effect = [[rust], [], []]
        device = Mock()
        device.write.return_value = len(enter_bootloader.BOOTLOADER_COMMAND)
        self.serial.return_value = device
        self.monotonic.side_effect = [0.0, 0.0, 1.0]

        with self.assertRaises(enter_bootloader.BootloaderTransitionError) as caught:
            enter_bootloader.prepare_for_upload(timeout=0.5)

        error = str(caught.exception)
        self.assertIn("1209:0001", error)
        self.assertIn("3185:0035", error)
        self.assertIn("automatic guest connection", error)
        self.assertIn("Physical unplug/replug is recovery only", error)
        self.sleep.assert_called_once_with(0.25)

    def test_no_matching_board_fails_without_opening_unrelated_serial(self):
        self.comports.return_value = [usb_port("other")]

        with self.assertRaisesRegex(
            enter_bootloader.BootloaderTransitionError, "No normal V6X Rust CDC"
        ):
            enter_bootloader.prepare_for_upload()

        self.assert_no_hardware_action()


class FirmwareUploadIntegrationTests(unittest.TestCase):
    def test_helper_runs_after_build_and_dry_run_but_before_uploader(self):
        script = (Path(__file__).parent / "firmware.sh").read_text()
        upload = script.split("upload_firmware() {", 1)[1].split("\n}\n", 1)[0]

        build = upload.index('build_firmware "$mode"')
        dry_run = upload.index('if [[ "${1:-}" == "--dry-run" ]]')
        helper = upload.index('"$root/tools/enter_bootloader.py"')
        uploader = upload.index('exec "$python" -u "$uploader"')
        self.assertLess(build, dry_run)
        self.assertLess(dry_run, helper)
        self.assertLess(helper, uploader)


if __name__ == "__main__":
    unittest.main()
