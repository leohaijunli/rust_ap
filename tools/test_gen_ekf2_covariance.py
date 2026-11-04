import unittest
import contextlib
import io
import tempfile
from pathlib import Path
from unittest.mock import patch

import gen_ekf2_covariance as generator


class ExpressionTests(unittest.TestCase):
    def test_translation_preserves_f32_order_and_indices(self):
        known = {"dt", "gyro_var", "_tmp0"}
        self.assertEqual(generator.expression("P(2, 3) + 2 * (state(0, 0) - accel(1, 0))", known),
                         "p[2][3] + 2.0 * (state[0] - accel[1])")
        self.assertEqual(generator.expression("std::pow(_tmp0, Scalar(2)) / (dt * gyro_var)", known),
                         "square(_tmp0) / (dt * gyro_var)")
        self.assertEqual(generator.expression("dt - (gyro_var - dt)", known), "dt - (gyro_var - dt)")

    def test_unknown_functions_symbols_and_indices_are_rejected(self):
        for expression in ["std::pow(dt, Scalar(3))", "_tmp1 + dt", "state(25, 0)", "P(24, 0)",
                           "accel(0, 1)", "__import__('os')", "dt ** 2", "dt[0]", "P(dt, 0)"]:
            with self.subTest(expression=expression), self.assertRaises((ValueError, SyntaxError)):
                generator.expression(expression, {"dt"})

    def test_full_render_requires_all_upper_triangle_entries(self):
        document = (
            "matrix::Matrix<Scalar, 24, 24> PredictCovariance(const matrix::Matrix<Scalar, 25, 1>& state, "
            "const matrix::Matrix<Scalar, 24, 24>& P) {\n// Intermediate terms (144)\n"
            + "\n".join(f"const Scalar _tmp{i} = dt * 2;" for i in range(144))
            + "\nmatrix::Matrix<Scalar, 24, 24> _res;\n_res.setZero();\n"
            + "\n".join(f"_res({i}, {j}) = P({i}, {j});" for j in range(24) for i in range(j + 1))
            + "\nreturn _res;\n}"
        )
        with patch.object(generator.subprocess, "check_output", side_effect=lambda *args, **kwargs: kwargs["input"]):
            rendered = generator.render(document)
            self.assertEqual(rendered.count("let _tmp"), 144)
            self.assertEqual(rendered.count("    res["), 300)
            for invalid in [document.replace("_res(23, 23) = P(23, 23);", ""),
                            document.replace("_tmp0 = dt * 2", "_tmp0 = _tmp1 * 2")]:
                with self.assertRaises(ValueError):
                    generator.render(invalid)

    def test_check_rejects_drift_without_rewriting(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "covariance.rs"
            output.write_text("old equations\n", encoding="utf-8")
            with patch.object(generator.sys, "argv", ["gen_ekf2_covariance.py", "--check", "--output", str(output)]), \
                    patch.object(generator, "git_show", return_value="fixed source"), \
                    patch.object(generator, "render", return_value="new equations\n"), \
                    contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(generator.main(), 1)
            self.assertEqual(output.read_text(encoding="utf-8"), "old equations\n")


if __name__ == "__main__":
    unittest.main()
