import json
import tempfile
import unittest
import zlib
from pathlib import Path
from unittest.mock import patch

import gen_parameter_metadata as metadata
from gen_params import DEFAULT_REF


CSV = """EKF2_GYR_NOISE,EKF2,Float,0.015,0.0001,0.1,rad/s,mapped,next-cycle
EKF2_GPS_CTRL,EKF2,Int32,7,0,15,,conditional:gps,next-cycle
EKF2_LOG_VERBOSE,EKF2,Int32,1,0,1,,unmapped,next-cycle
MAV_RATE_HB,Link,Float,1,0,5,Hz,runtime,next-cycle
"""


class MetadataTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        self.help_file = self.root / "help.json"
        self.help = {
            "ref": DEFAULT_REF,
            "parameters": {
                "EKF2_GYR_NOISE": {"type": "float", "shortDesc": "Rate gyro noise", "options": {}},
                "EKF2_GPS_CTRL": {"type": "bitmask", "shortDesc": "GNSS aiding", "options": {"0": "Lon/lat", "3": "Heading"}},
                "EKF2_LOG_VERBOSE": {"type": "boolean", "shortDesc": "Verbose logging", "options": {}},
            },
        }
        self.help_file.write_text(json.dumps(self.help), encoding="utf-8")
        self.output = self.enterContext(patch.object(metadata.subprocess, "check_output", return_value=CSV))

    def generate(self):
        return metadata.generated("catalog", self.help_file)

    def test_labels_setting_guides_and_crc_preserve_the_registry_values(self):
        files = self.generate()
        entries = {entry["name"]: entry for entry in json.loads(files["parameters.json"])["parameters"]}
        gyro = entries["EKF2_GYR_NOISE"]
        self.assertIn("已接入配置", gyro["shortDesc"])
        self.assertEqual((gyro["default"], gyro["min"], gyro["max"], gyro["type"], gyro["units"]), (0.015, 0.0001, 0.1, "Float", "rad/s"))
        self.assertIn("默认 0.015 rad/s", gyro["longDesc"])
        gps = entries["EKF2_GPS_CTRL"]
        self.assertIn("条件生效", gps["shortDesc"])
        self.assertIn("1=Lon/lat", gps["longDesc"])
        self.assertIn("8=Heading", gps["longDesc"])
        self.assertIn("当前仅支持 0..7", gps["longDesc"])
        self.assertEqual(gps["max"], 15)
        verbose = entries["EKF2_LOG_VERBOSE"]
        self.assertIn("未接入", verbose["shortDesc"])
        self.assertIn("不读取", verbose["longDesc"])
        self.assertIn("重新初始化", verbose["longDesc"])
        heartbeat = entries["MAV_RATE_HB"]
        self.assertIn("QGC 连接", heartbeat["longDesc"])
        self.assertIn("覆盖优先", heartbeat["longDesc"])
        self.assertNotIn("unmapped EKF options", files["parameters.json"])
        general = json.loads(files["general.json"])
        self.assertEqual(general["metadataTypes"][0]["fileCrc"], zlib.crc32(files["parameters.json"].encode()))

    def test_unknown_or_missing_integration_information_is_not_silently_accepted(self):
        for text in [CSV.replace(",mapped,", ",unknown,"), CSV.replace(",runtime,", ",unmapped,"), CSV.replace(",rad/s,mapped,next-cycle", "")]:
            with self.subTest(text=text):
                self.output.return_value = text
                with self.assertRaises(ValueError):
                    self.generate()
        self.help["ref"] = "wrong-source"
        self.help_file.write_text(json.dumps(self.help), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "pinned"):
            self.generate()

    def test_help_coverage_and_other_parameter_guides_are_explicit(self):
        self.output.return_value = CSV.replace("EKF2_GYR_NOISE", "EKF2_UNKNOWN")
        with self.assertRaisesRegex(ValueError, "Missing"):
            self.generate()
        with self.assertRaises(ValueError):
            metadata.runtime_help("NEW_PARAMETER_WITHOUT_HELP")
        for name in ["MAV_SYS_ID", "EST_EN", "EST_ALGO", "LOG_EN_EVENT", "LOG_CAN_HZ", "LOG_EN_MAG", "LOG_MAG_HZ", "MAV_RATE_POWER"]:
            short, description = metadata.runtime_help(name)
            self.assertTrue(short and description)
        self.assertIn("不解锁", metadata.runtime_help("EST_EN")[1])
        self.assertIn("只读", metadata.runtime_help("MAV_SYS_ID")[1])
        self.assertIn("地磁", metadata.runtime_help("LOG_EN_MAG")[0])

    def test_check_detects_stale_files_without_overwriting_them(self):
        files = self.generate()
        for name, text in files.items():
            (self.root / name).write_text(text, encoding="utf-8", newline="\n")
        with patch.object(metadata, "DIRECTORY", self.root), patch.object(metadata, "generated", return_value=files), patch("sys.argv", ["gen_parameter_metadata.py", "--rust", "catalog", "--check"]):
            metadata.main()
            target = self.root / "parameters.json"
            target.write_text("old metadata", encoding="utf-8")
            with self.assertRaisesRegex(SystemExit, "differs"):
                metadata.main()
            self.assertEqual(target.read_text(encoding="utf-8"), "old metadata")


if __name__ == "__main__":
    unittest.main()
