import json
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, Mock, patch

try:
    import tomllib
except ModuleNotFoundError:
    import tomli
    sys.modules["tomllib"] = tomli

import upload_windows


class WindowsWorkflowTests(unittest.TestCase):
    def test_vs_code_default_build_and_upload_do_not_pin_a_com_port(self):
        root = Path(__file__).resolve().parents[1]
        tasks = json.loads((root / ".vscode/tasks.json").read_text())
        build = next(task for task in tasks["tasks"] if task["label"] == "Build firmware (.px4)")
        upload = next(task for task in tasks["tasks"] if task["label"] == "Upload firmware (USB)")
        self.assertTrue(build["group"]["isDefault"])
        self.assertEqual(upload["type"], "process")
        self.assertNotIn("COM12", (root / "tools/upload_firmware.ps1").read_text())
        self.assertIn("[string[]]$pythonArgs", (root / "tools/build_firmware.ps1").read_text())

    def test_changed_com_number_is_selected_after_usb_reset(self):
        rust = SimpleNamespace(device="COM26", vid=0x1209, pid=1)
        bootloader = SimpleNamespace(device="COM39", vid=0x3185, pid=0x0035)
        with patch.object(upload_windows, "scan_target_port", side_effect=[(rust, "rust", []), (bootloader, "px4-application", [])]), patch.object(upload_windows.time, "sleep"):
            self.assertEqual(upload_windows.wait_for_px4_usb().device, "COM39")

    def test_rustrover_windows_profiles_use_powershell_not_a_unix_bash_path(self):
        root = Path(__file__).resolve().parents[1]
        for filename in ["Build firmware.run.xml", "Upload firmware (USB).run.xml"]:
            config = ET.parse(root / ".run" / filename).find("configuration")
            options = {entry.attrib["name"]: entry.attrib["value"] for entry in config.findall("option")}
            self.assertTrue(options["INTERPRETER_PATH"].endswith("powershell.exe"))
            self.assertIn("-File", options["INTERPRETER_OPTIONS"])
            self.assertEqual(options["SCRIPT_OPTIONS"], "")
            self.assertTrue(options["SCRIPT_PATH"].endswith(".ps1"))

    def test_armed_preflight_fails_and_always_closes_serial(self):
        link = Mock()
        port = SimpleNamespace(device="COM26")
        with patch.object(upload_windows.mavutil, "mavlink_connection", return_value=link), patch.object(upload_windows, "receive", return_value=SimpleNamespace(base_mode=128)):
            with self.assertRaisesRegex(ValueError, "armed"):
                upload_windows.ensure_disarmed(port)
        link.close.assert_called_once()

    def test_timeout_or_missing_device_never_selects_an_unrelated_port(self):
        with patch.object(upload_windows, "scan_target_port", return_value=(None, None, [])), patch.object(upload_windows.time, "monotonic", side_effect=[0, 16]):
            with self.assertRaisesRegex(ValueError, "did not enumerate"):
                upload_windows.wait_for_px4_usb()

    def test_ambiguous_windows_bootloader_requires_positive_protocol_identity(self):
        protocol = Mock()
        module = SimpleNamespace(
            SerialTransport=MagicMock(), BootloaderProtocol=Mock(return_value=protocol),
            DeviceInfo=SimpleNamespace(BL_REV=1, BOARD_ID=2), UploadError=RuntimeError,
        )
        port = SimpleNamespace(device="COM39")
        with patch.object(upload_windows, "load_uploader", return_value=module):
            protocol._get_device_info.side_effect = [5, 53]
            self.assertTrue(upload_windows.probe_bootloader(port, Path("pinned.py")))
            protocol._get_device_info.side_effect = [5, 99]
            with self.assertRaisesRegex(ValueError, "board=99"):
                upload_windows.probe_bootloader(port, Path("pinned.py"))
            protocol.sync.side_effect = RuntimeError("no bootloader response")
            self.assertFalse(upload_windows.probe_bootloader(port, Path("pinned.py")))


if __name__ == "__main__":
    unittest.main()
