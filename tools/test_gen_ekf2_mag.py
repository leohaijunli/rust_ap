import unittest

import gen_ekf2_mag as generator


class MagGeneratorTests(unittest.TestCase):
    def test_scalar_casts_and_squares_keep_float_division(self):
        self.assertEqual(generator.scalar("Scalar(1) / Scalar(2) * std::pow(state(3, 0), Scalar(2))", set()),
                         "1.0 / 2.0 * square(state[3])")

    def test_pointer_alias_declaration_is_not_an_equation(self):
        document = "// Intermediate terms (0)\n// Output terms (2)\n"
        document += "Scalar& _innov_var = (*innov_var);\n_innov_var = P(2, 2) + R;\n"
        document += "\n".join(f"_h({i}, 0) = 1;" for i in range(7))
        output = generator.render_axis(document, "y")
        self.assertIn("let variance = p[2][2] + R;", output)
        with self.assertRaises(ValueError):
            generator.render_axis(document.replace("_h(6, 0) = 1;", ""), "y")


if __name__ == "__main__":
    unittest.main()
