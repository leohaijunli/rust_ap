import contextlib
import io
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import gen_params


def float_definition(**overrides):
    meta = {"type": "float", "default": 0.5, "min": 0.0, "max": 1.0, "unit": "m"}
    meta.update(overrides)
    return meta


class DefinitionParsingTests(unittest.TestCase):
    def test_px4_types_map_to_the_rust_parameter_kinds(self):
        cases = [
            (
                "float", float_definition(),
                gen_params.Definition("P", "Float", 0.5, 0.0, 1.0, "m", "Planned"),
            ),
            (
                "int32", {"type": "int32", "default": 10, "min": 1, "max": 20, "unit": "us"},
                gen_params.Definition("P", "Int32", 10, 1.0, 20.0, "us", "Planned"),
            ),
            (
                "boolean", {"type": "boolean", "default": 1},
                gen_params.Definition("P", "Boolean", 1, None, None, "", "Planned"),
            ),
            (
                "enum", {"type": "enum", "default": 2, "values": {0: "A", 1: "B"}},
                gen_params.Definition("P", "Enum", 2, None, None, "", "Planned"),
            ),
            (
                "bitmask", {"type": "bitmask", "default": 7, "min": 0, "max": 15},
                gen_params.Definition("P", "Bitmask", 7, 0.0, 15.0, "", "Planned"),
            ),
        ]
        for name, meta, expected in cases:
            with self.subTest(name):
                self.assertEqual(gen_params.parse_definition("P", meta), expected)

    def test_invalid_definitions_are_rejected(self):
        cases = [
            ("EKF2_PARAMETER_TOO_LONG", float_definition(), "param_id"),
            ("ekf2_lower", float_definition(), "style"),
            ("EKF2_TYPE", float_definition(type="double"), "unsupported"),
            ("EKF2_DEFAULT", {"type": "float"}, "missing default"),
            ("EKF2_NAN", float_definition(default=float("nan")), "non-finite"),
        ]
        for name, meta, message in cases:
            with self.subTest(name):
                with self.assertRaisesRegex(ValueError, message):
                    gen_params.parse_definition(name, meta)

    def test_duplicates_across_files_are_rejected(self):
        document = {"parameters": [{"definitions": {"EKF2_EN": {"type": "boolean", "default": 1}}}]}
        with self.assertRaisesRegex(ValueError, "duplicate"):
            gen_params.collect([("module.yaml", document), ("params_selector.yaml", document)])

    def test_collect_keeps_source_order(self):
        documents = [
            ("module.yaml", {"parameters": [{"definitions": {"EKF2_B": {"type": "boolean", "default": 0}}}]}),
            ("params_gnss.yaml", {"parameters": [{"definitions": {"EKF2_A": {"type": "int32", "default": 3}}}]}),
        ]
        names = [definition.name for definition in gen_params.collect(documents)]
        self.assertEqual(names, ["EKF2_B", "EKF2_A"])


class RenderingTests(unittest.TestCase):
    def test_help_preserves_descriptions_and_bit_positions_without_copying_values(self):
        documents = [("module.yaml", {"parameters": [{"definitions": {
            "EKF2_CTRL": {"type": "bitmask", "description": {"short": "Sensor control"}, "bit": {0: "First", 3: "Fourth"}, "default": 1, "min": 0, "max": 15},
        }}]})]
        help_data = json.loads(gen_params.render_help(documents, "fixed-ref"))
        entry = help_data["parameters"]["EKF2_CTRL"]
        self.assertEqual(entry["options"], {"0": "First", "3": "Fourth"})
        self.assertEqual(entry["shortDesc"], "Sensor control")
        self.assertEqual(help_data["ref"], "fixed-ref")
        self.assertNotIn("default", entry)
        self.assertNotIn("min", entry)
        self.assertNotIn("max", entry)

    def test_render_emits_one_rust_entry_per_parameter(self):
        definitions = [
            gen_params.Definition("EKF2_F", "Float", 0.5, 0.0, 1.0, "m/s", "Planned"),
            gen_params.Definition("EKF2_I", "Int32", 7, None, None, "", "Planned"),
        ]
        rendered = gen_params.render(definitions, "deadbeef")

        self.assertIn("PX4-Autopilot deadbeef", rendered)
        self.assertIn("Copyright (c) 2012 - 2023, PX4 Development Team", rendered)
        self.assertIn("SPDX-License-Identifier: BSD-3-Clause", rendered)
        self.assertIn("tools/px4_msgs/LICENSE", rendered)
        self.assertIn('name: "EKF2_F"', rendered)
        self.assertIn("default: ParamValue::Float(0.5)", rendered)
        self.assertIn("min: Some(0.0)", rendered)
        self.assertIn("default: ParamValue::Int(7)", rendered)
        self.assertIn("min: None", rendered)
        self.assertIn('unit: "m/s"', rendered)
        self.assertEqual(rendered.count("    ParamDef {"), 2)
        self.assertTrue(rendered.endswith("];\n"))

    def test_float_literals_stay_rust_parseable(self):
        self.assertEqual(gen_params.float_literal(1.0), "1.0")
        self.assertEqual(gen_params.float_literal(0.015), "0.015")
        self.assertEqual(gen_params.float_literal(-0.0), "0.0")
        self.assertEqual(gen_params.float_literal(1e-05), "1e-05")

    def test_units_and_names_are_escaped(self):
        definition = gen_params.Definition('EKF2_"Q"', "Float", 0.0, None, None, 'a"b', "Planned")
        rendered = gen_params.render([definition], "deadbeef")
        self.assertIn('name: "EKF2_\\"Q\\""', rendered)
        self.assertIn('unit: "a\\"b"', rendered)


class CheckTests(unittest.TestCase):
    def test_check_includes_fixed_source_help_without_rewriting_stale_files(self):
        documents = [("module.yaml", {"parameters": [{"definitions": {
            "EKF2_F": {**float_definition(), "description": {"short": "Noise"}},
        }}]})]
        catalog = gen_params.render(gen_params.collect(documents), "fixed-ref")
        help_text = gen_params.render_help(documents, "fixed-ref")
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "catalog.rs"
            help_file = Path(directory) / "help.json"
            output.write_text(catalog, encoding="utf-8")
            help_file.write_text(help_text, encoding="utf-8")
            argv = ["gen_params.py", "--ref", "fixed-ref", "--output", str(output), "--help-output", str(help_file), "--check"]
            with patch("sys.argv", argv), patch.object(gen_params, "load_documents", return_value=documents), contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(gen_params.main(), 0)
                help_file.write_text("stale help", encoding="utf-8")
                self.assertEqual(gen_params.main(), 1)
                self.assertEqual(help_file.read_text(encoding="utf-8"), "stale help")

    def test_check_accepts_current_table_and_rejects_drift_without_rewriting(self):
        document = {"parameters": [{"definitions": {"EKF2_F": float_definition()}}]}
        documents = [("module.yaml", document)]
        current = gen_params.render(gen_params.collect(documents), "deadbeef")
        cases = [(current, 0), (current.replace("Float(0.5)", "Float(0.6)"), 1)]
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "params_generated.rs"
            argv = ["gen_params.py", "--ref", "deadbeef", "--output", str(output), "--check"]
            with (
                patch("sys.argv", argv),
                patch.object(gen_params, "load_documents", return_value=documents),
            ):
                for content, expected in cases:
                    with self.subTest(expected=expected):
                        output.write_text(content, encoding="utf-8")
                        with (
                            contextlib.redirect_stdout(io.StringIO()),
                            contextlib.redirect_stderr(io.StringIO()),
                        ):
                            self.assertEqual(gen_params.main(), expected)
                        self.assertEqual(output.read_text(encoding="utf-8"), content)


if __name__ == "__main__":
    unittest.main()
