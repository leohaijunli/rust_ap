import contextlib
import fnmatch
import io
import os
import subprocess
import sys
import unittest
from pathlib import Path
from unittest.mock import patch


SCRIPT = (Path(__file__).parent / "firmware.sh").read_text()
PORTS_LINE = next(line for line in SCRIPT.splitlines() if line.strip().startswith('ports="${PX4_PORTS:-'))
PREFLIGHT = compile(SCRIPT.split("<<'PY'\n", 1)[1].split("\nPY\n", 1)[0], "firmware.sh port preflight", "exec")


class FirmwarePortTests(unittest.TestCase):
    """Exercise the actual defaults/preflight without building or opening a tty."""

    def patterns(self, override=None):
        env = dict(os.environ)
        env.pop("PX4_PORTS", None)
        if override is not None:
            env["PX4_PORTS"] = override
        return subprocess.check_output(
            ["bash", "--noprofile", "--norc", "-c", PORTS_LINE + '\nprintf "%s" "$ports"'],
            env=env,
            text=True,
        )

    def preflight(self, devices, override=None, writable=True):
        patterns = self.patterns(override)
        output = io.StringIO()
        with (
            patch.object(sys, "argv", ["-", patterns]),
            patch("glob.glob", side_effect=lambda pattern: [p for p in devices if fnmatch.fnmatchcase(p, pattern)]),
            patch("os.access", return_value=writable),
            contextlib.redirect_stdout(output),
        ):
            exec(PREFLIGHT, {"__name__": "__main__"})
        return output.getvalue()

    def test_default_accepts_rust_px4_application_and_bootloader(self):
        for device in (
            "/dev/serial/by-id/usb-Rust_FMUv6X_V6X_Rust_CDC_V6X-RUST-001-if00",
            "/dev/serial/by-id/usb-Auterion_PX4_FMU_v6X.x_0-if00",
            "/dev/serial/by-id/usb-Auterion_PX4_BL_FMU_v6X.x_0-if00",
        ):
            with self.subTest(device=device):
                self.assertEqual(self.preflight([device]), f"USB device: {device}\n")

    def test_does_not_match_other_px4_applications(self):
        with self.assertRaisesRegex(SystemExit, "Expected one FMUv6X serial port"):
            self.preflight(["/dev/serial/by-id/usb-Auterion_PX4_FMU_v5.x_0-if00"])

    def test_rejects_no_device(self):
        with self.assertRaisesRegex(SystemExit, r"found \[\]"):
            self.preflight([])

    def test_rejects_multiple_devices(self):
        with self.assertRaisesRegex(SystemExit, "Expected one FMUv6X serial port"):
            self.preflight([
                "/dev/serial/by-id/usb-Auterion_PX4_FMU_v6X.x_0-if00",
                "/dev/serial/by-id/usb-Auterion_PX4_FMU_v6X.x_1-if00",
            ])

    def test_rejects_no_access(self):
        with self.assertRaisesRegex(SystemExit, "No read/write access"):
            self.preflight(["/dev/serial/by-id/usb-Auterion_PX4_FMU_v6X.x_0-if00"], writable=False)

    def test_explicit_patterns_override_defaults(self):
        self.assertEqual(self.preflight(["/dev/custom"], "/dev/custom,/dev/custom"), "USB device: /dev/custom\n")
        with self.assertRaisesRegex(SystemExit, r"found \[\]"):
            self.preflight(["/dev/serial/by-id/usb-Auterion_PX4_FMU_v6X.x_0-if00"], "/dev/custom")
