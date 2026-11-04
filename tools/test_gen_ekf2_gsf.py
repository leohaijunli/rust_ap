import unittest

from gen_ekf2_gsf import scalar


class GsfGeneratorTests(unittest.TestCase):
    def test_scalar_translation_preserves_bounds_order_and_determinant_regularization(self):
        self.assertEqual(scalar("std::cos(state(2, 0)) + P(0, 2) * d_vel(1, 0)", set()),
                         "libm::cosf(state[2]) + p[0][2] * d_vel[1]")
        self.assertEqual(scalar("std::pow(d_ang, Scalar(2))", {"d_ang"}), "square(d_ang)")
        expression = "math::min<Scalar>(0, (((_tmp2) > 0) - ((_tmp2) < 0)))"
        self.assertEqual(scalar(expression, {"_tmp2"}), "negative_sign(_tmp2)")
        for unsupported in ("P(3, 0)", "state(3, 0)", "unknown + 1", "std::sqrt(P(0,0))"):
            with self.assertRaises((ValueError, SyntaxError)):
                scalar(unsupported, set())


if __name__ == "__main__":
    unittest.main()
