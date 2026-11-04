// @generated from fixed PX4 d6f12ad1c4f70ad3230afd7d86e971421e02fef4 by tools/gen_ekf2_mag.py.
// Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
// License: tools/px4_msgs/LICENSE. Preserve expression order.
#![allow(non_snake_case)]
use super::{math::square, Covariance, ErrorStateVector, StateVector};
// Source: src/modules/ekf2/EKF/python/ekf_derivation/generated/compute_mag_y_innov_var_and_h.h; SHA-256 feab438df4dfdd8dd318e575f78b0d7f4f79f16c00d0e8a2fc5f49c0f4512e3b
pub(super) fn mag_y(state: &StateVector, p: &Covariance, R: f32) -> (f32, ErrorStateVector) {
    let _tmp0 = 2.0 * state[2];
    let _tmp1 = 2.0 * state[1];
    let _tmp2 = _tmp0 * state[3] + _tmp1 * state[0];
    let _tmp3 = 2.0 * state[3];
    let _tmp4 = _tmp0 * state[1] - _tmp3 * state[0];
    let _tmp5 = -2.0 * square(state[1]) - 2.0 * square(state[3]) + 1.0;
    let _tmp6 = _tmp1 * state[18] - _tmp3 * state[16];
    let _tmp7 = 1.0 / 2.0 * _tmp6;
    let _tmp8 = 1.0 / 2.0 * _tmp1 * state[16] + 1.0 / 2.0 * _tmp3 * state[18];
    let _tmp9 = 4.0 * state[17];
    let _tmp10 = 2.0 * state[0];
    let _tmp11 = _tmp0 * state[16] + _tmp10 * state[18] - _tmp9 * state[1];
    let _tmp12 = 1.0 / 2.0 * _tmp11;
    let _tmp13 = 1.0 / 2.0 * _tmp0 * state[18]
        - 1.0 / 2.0 * _tmp10 * state[16]
        - 1.0 / 2.0 * _tmp9 * state[3];
    let _tmp14 = _tmp12 * state[0] + _tmp13 * state[2] - _tmp7 * state[1] - _tmp8 * state[3];
    let _tmp15 = 1.0 / 2.0 * state[2];
    let _tmp16 = -_tmp11 * _tmp15 + _tmp13 * state[0] - _tmp7 * state[3] + _tmp8 * state[1];
    let _tmp17 = _tmp12 * state[3] - _tmp13 * state[1] - _tmp15 * _tmp6 + _tmp8 * state[0];
    let variance = p[0][19] * _tmp14
        + p[1][19] * _tmp17
        + p[15][19] * _tmp4
        + p[16][19] * _tmp5
        + p[17][19] * _tmp2
        + p[19][19]
        + p[2][19] * _tmp16
        + R
        + _tmp14
            * (p[0][0] * _tmp14
                + p[1][0] * _tmp17
                + p[15][0] * _tmp4
                + p[16][0] * _tmp5
                + p[17][0] * _tmp2
                + p[19][0]
                + p[2][0] * _tmp16)
        + _tmp16
            * (p[0][2] * _tmp14
                + p[1][2] * _tmp17
                + p[15][2] * _tmp4
                + p[16][2] * _tmp5
                + p[17][2] * _tmp2
                + p[19][2]
                + p[2][2] * _tmp16)
        + _tmp17
            * (p[0][1] * _tmp14
                + p[1][1] * _tmp17
                + p[15][1] * _tmp4
                + p[16][1] * _tmp5
                + p[17][1] * _tmp2
                + p[19][1]
                + p[2][1] * _tmp16)
        + _tmp2
            * (p[0][17] * _tmp14
                + p[1][17] * _tmp17
                + p[15][17] * _tmp4
                + p[16][17] * _tmp5
                + p[17][17] * _tmp2
                + p[19][17]
                + p[2][17] * _tmp16)
        + _tmp4
            * (p[0][15] * _tmp14
                + p[1][15] * _tmp17
                + p[15][15] * _tmp4
                + p[16][15] * _tmp5
                + p[17][15] * _tmp2
                + p[19][15]
                + p[2][15] * _tmp16)
        + _tmp5
            * (p[0][16] * _tmp14
                + p[1][16] * _tmp17
                + p[15][16] * _tmp4
                + p[16][16] * _tmp5
                + p[17][16] * _tmp2
                + p[19][16]
                + p[2][16] * _tmp16);
    let mut h = [0.0; 24];
    h[0] = _tmp14;
    h[1] = _tmp17;
    h[2] = _tmp16;
    h[15] = _tmp4;
    h[16] = _tmp5;
    h[17] = _tmp2;
    h[19] = 1.0;
    (variance, h)
}
// Source: src/modules/ekf2/EKF/python/ekf_derivation/generated/compute_mag_z_innov_var_and_h.h; SHA-256 8b0fc384eb55a7178530082bd98b80bf488e4de2291bd4c5689df4d66bfd47b7
pub(super) fn mag_z(state: &StateVector, p: &Covariance, R: f32) -> (f32, ErrorStateVector) {
    let _tmp0 = 2.0 * state[16];
    let _tmp1 = 2.0 * state[17];
    let _tmp2 = 1.0 / 2.0 * _tmp0 * state[2] - 1.0 / 2.0 * _tmp1 * state[1];
    let _tmp3 = 2.0 * state[1];
    let _tmp4 = 1.0 / 2.0 * _tmp1 * state[2] + 1.0 / 2.0 * _tmp3 * state[16];
    let _tmp5 = 4.0 * state[18];
    let _tmp6 =
        1.0 / 2.0 * _tmp0 * state[0] + 1.0 / 2.0 * _tmp1 * state[3] - 1.0 / 2.0 * _tmp5 * state[2];
    let _tmp7 =
        1.0 / 2.0 * _tmp0 * state[3] - 1.0 / 2.0 * _tmp1 * state[0] - 1.0 / 2.0 * _tmp5 * state[1];
    let _tmp8 = -_tmp2 * state[3] + _tmp4 * state[0] + _tmp6 * state[1] - _tmp7 * state[2];
    let _tmp9 = -_tmp2 * state[1] + _tmp4 * state[2] - _tmp6 * state[3] + _tmp7 * state[0];
    let _tmp10 = -_tmp2 * state[2] - _tmp4 * state[1] + _tmp6 * state[0] + _tmp7 * state[3];
    let _tmp11 = -2.0 * square(state[1]) - 2.0 * square(state[2]) + 1.0;
    let _tmp12 = 2.0 * state[2];
    let _tmp13 = _tmp12 * state[3] - _tmp3 * state[0];
    let _tmp14 = _tmp12 * state[0] + _tmp3 * state[3];
    let variance = p[0][20] * _tmp9
        + p[1][20] * _tmp10
        + p[15][20] * _tmp14
        + p[16][20] * _tmp13
        + p[17][20] * _tmp11
        + p[2][20] * _tmp8
        + p[20][20]
        + R
        + _tmp10
            * (p[0][1] * _tmp9
                + p[1][1] * _tmp10
                + p[15][1] * _tmp14
                + p[16][1] * _tmp13
                + p[17][1] * _tmp11
                + p[2][1] * _tmp8
                + p[20][1])
        + _tmp11
            * (p[0][17] * _tmp9
                + p[1][17] * _tmp10
                + p[15][17] * _tmp14
                + p[16][17] * _tmp13
                + p[17][17] * _tmp11
                + p[2][17] * _tmp8
                + p[20][17])
        + _tmp13
            * (p[0][16] * _tmp9
                + p[1][16] * _tmp10
                + p[15][16] * _tmp14
                + p[16][16] * _tmp13
                + p[17][16] * _tmp11
                + p[2][16] * _tmp8
                + p[20][16])
        + _tmp14
            * (p[0][15] * _tmp9
                + p[1][15] * _tmp10
                + p[15][15] * _tmp14
                + p[16][15] * _tmp13
                + p[17][15] * _tmp11
                + p[2][15] * _tmp8
                + p[20][15])
        + _tmp8
            * (p[0][2] * _tmp9
                + p[1][2] * _tmp10
                + p[15][2] * _tmp14
                + p[16][2] * _tmp13
                + p[17][2] * _tmp11
                + p[2][2] * _tmp8
                + p[20][2])
        + _tmp9
            * (p[0][0] * _tmp9
                + p[1][0] * _tmp10
                + p[15][0] * _tmp14
                + p[16][0] * _tmp13
                + p[17][0] * _tmp11
                + p[2][0] * _tmp8
                + p[20][0]);
    let mut h = [0.0; 24];
    h[0] = _tmp9;
    h[1] = _tmp10;
    h[2] = _tmp8;
    h[15] = _tmp14;
    h[16] = _tmp13;
    h[17] = _tmp11;
    h[20] = 1.0;
    (variance, h)
}
// Source: src/modules/ekf2/EKF/python/ekf_derivation/generated/compute_mag_innov_innov_var_and_hx.h; SHA-256 f90e274888c117e1bb3accac73d9eb9aa0edb6a781dedd611936dd104562626d
pub(super) fn mag_x(
    state: &StateVector,
    p: &Covariance,
    meas: [f32; 3],
    R: f32,
) -> ([f32; 3], [f32; 3], ErrorStateVector) {
    let _tmp0 = -2.0 * square(state[3]);
    let _tmp1 = 1.0 - 2.0 * square(state[2]);
    let _tmp2 = _tmp0 + _tmp1;
    let _tmp3 = 2.0 * state[3];
    let _tmp4 = _tmp3 * state[0];
    let _tmp5 = 2.0 * state[2];
    let _tmp6 = _tmp5 * state[1];
    let _tmp7 = _tmp4 + _tmp6;
    let _tmp8 = _tmp5 * state[0];
    let _tmp9 = 2.0 * state[1];
    let _tmp10 = _tmp9 * state[3];
    let _tmp11 = _tmp10 - _tmp8;
    let _tmp12 = -2.0 * square(state[1]);
    let _tmp13 = _tmp0 + _tmp12 + 1.0;
    let _tmp14 = _tmp5 * state[3];
    let _tmp15 = _tmp9 * state[0];
    let _tmp16 = _tmp14 + _tmp15;
    let _tmp17 = -_tmp4 + _tmp6;
    let _tmp18 = _tmp1 + _tmp12;
    let _tmp19 = _tmp14 - _tmp15;
    let _tmp20 = _tmp10 + _tmp8;
    let _tmp21 = _tmp3 * state[18];
    let _tmp22 = _tmp5 * state[17];
    let _tmp23 = _tmp21 + _tmp22;
    let _tmp24 = 1.0 / 2.0 * state[2];
    let _tmp25 = 2.0 * state[17];
    let _tmp26 = _tmp25 * state[3];
    let _tmp27 = _tmp5 * state[18];
    let _tmp28 = _tmp26 - _tmp27;
    let _tmp29 = 1.0 / 2.0 * _tmp28;
    let _tmp30 = 4.0 * state[2];
    let _tmp31 = _tmp9 * state[17];
    let _tmp32 = 2.0 * state[0];
    let _tmp33 = _tmp32 * state[18];
    let _tmp34 = -_tmp30 * state[16] + _tmp31 - _tmp33;
    let _tmp35 = 1.0 / 2.0 * state[1];
    let _tmp36 = _tmp9 * state[18];
    let _tmp37 = _tmp25 * state[0];
    let _tmp38 = 1.0 / 2.0 * _tmp36 + 1.0 / 2.0 * _tmp37 - 2.0 * state[16] * state[3];
    let _tmp39 = -_tmp23 * _tmp24 - _tmp29 * state[3] + _tmp34 * _tmp35 + _tmp38 * state[0];
    let _tmp40 = 1.0 / 2.0 * _tmp23;
    let _tmp41 = 1.0 / 2.0 * state[0];
    let _tmp42 = -_tmp24 * _tmp28 + _tmp34 * _tmp41 - _tmp38 * state[1] + _tmp40 * state[3];
    let _tmp43 = 1.0 / 2.0 * state[3];
    let _tmp44 = -_tmp29 * state[1] - _tmp34 * _tmp43 + _tmp38 * state[2] + _tmp40 * state[0];
    let _tmp45 = _tmp3 * state[16];
    let _tmp46 = _tmp36 - _tmp45;
    let _tmp47 = 1.0 / 2.0 * _tmp46;
    let _tmp48 = _tmp9 * state[16];
    let _tmp49 = 1.0 / 2.0 * _tmp21 + 1.0 / 2.0 * _tmp48;
    let _tmp50 = _tmp5 * state[16];
    let _tmp51 = 4.0 * state[17];
    let _tmp52 = _tmp33 + _tmp50 - _tmp51 * state[1];
    let _tmp53 = _tmp32 * state[16];
    let _tmp54 = _tmp27 - _tmp51 * state[3] - _tmp53;
    let _tmp55 = _tmp24 * _tmp54 + _tmp41 * _tmp52 - _tmp47 * state[1] - _tmp49 * state[3];
    let _tmp56 = -_tmp24 * _tmp52 + _tmp41 * _tmp54 - _tmp47 * state[3] + _tmp49 * state[1];
    let _tmp57 = -_tmp24 * _tmp46 - _tmp35 * _tmp54 + _tmp43 * _tmp52 + _tmp49 * state[0];
    let _tmp58 = -_tmp31 + _tmp50;
    let _tmp59 = 1.0 / 2.0 * _tmp58;
    let _tmp60 = _tmp22 + _tmp48;
    let _tmp61 = 1.0 / 2.0 * _tmp60;
    let _tmp62 = _tmp26 - _tmp30 * state[18] + _tmp53;
    let _tmp63 = -1.0 / 2.0 * _tmp37 + 1.0 / 2.0 * _tmp45 - 2.0 * state[1] * state[18];
    let _tmp64 = _tmp35 * _tmp62 - _tmp59 * state[3] + _tmp61 * state[0] - _tmp63 * state[2];
    let _tmp65 = 1.0 / 2.0 * _tmp62;
    let _tmp66 = _tmp24 * _tmp60 - _tmp59 * state[1] + _tmp63 * state[0] - _tmp65 * state[3];
    let _tmp67 = -_tmp24 * _tmp58 - _tmp61 * state[1] + _tmp63 * state[3] + _tmp65 * state[0];
    let mut h = [0.0; 24];
    let mut innovation = [0.0; 3];
    let mut variance = [0.0; 3];
    innovation[0] =
        _tmp11 * state[18] + _tmp2 * state[16] + _tmp7 * state[17] - meas[0] + state[19];
    innovation[1] =
        _tmp13 * state[17] + _tmp16 * state[18] + _tmp17 * state[16] - meas[1] + state[20];
    innovation[2] =
        _tmp18 * state[18] + _tmp19 * state[17] + _tmp20 * state[16] - meas[2] + state[21];
    variance[0] = p[0][18] * _tmp44
        + p[1][18] * _tmp42
        + p[15][18] * _tmp2
        + p[16][18] * _tmp7
        + p[17][18] * _tmp11
        + p[18][18]
        + p[2][18] * _tmp39
        + R
        + _tmp11
            * (p[0][17] * _tmp44
                + p[1][17] * _tmp42
                + p[15][17] * _tmp2
                + p[16][17] * _tmp7
                + p[17][17] * _tmp11
                + p[18][17]
                + p[2][17] * _tmp39)
        + _tmp2
            * (p[0][15] * _tmp44
                + p[1][15] * _tmp42
                + p[15][15] * _tmp2
                + p[16][15] * _tmp7
                + p[17][15] * _tmp11
                + p[18][15]
                + p[2][15] * _tmp39)
        + _tmp39
            * (p[0][2] * _tmp44
                + p[1][2] * _tmp42
                + p[15][2] * _tmp2
                + p[16][2] * _tmp7
                + p[17][2] * _tmp11
                + p[18][2]
                + p[2][2] * _tmp39)
        + _tmp42
            * (p[0][1] * _tmp44
                + p[1][1] * _tmp42
                + p[15][1] * _tmp2
                + p[16][1] * _tmp7
                + p[17][1] * _tmp11
                + p[18][1]
                + p[2][1] * _tmp39)
        + _tmp44
            * (p[0][0] * _tmp44
                + p[1][0] * _tmp42
                + p[15][0] * _tmp2
                + p[16][0] * _tmp7
                + p[17][0] * _tmp11
                + p[18][0]
                + p[2][0] * _tmp39)
        + _tmp7
            * (p[0][16] * _tmp44
                + p[1][16] * _tmp42
                + p[15][16] * _tmp2
                + p[16][16] * _tmp7
                + p[17][16] * _tmp11
                + p[18][16]
                + p[2][16] * _tmp39);
    variance[1] = p[0][19] * _tmp55
        + p[1][19] * _tmp57
        + p[15][19] * _tmp17
        + p[16][19] * _tmp13
        + p[17][19] * _tmp16
        + p[19][19]
        + p[2][19] * _tmp56
        + R
        + _tmp13
            * (p[0][16] * _tmp55
                + p[1][16] * _tmp57
                + p[15][16] * _tmp17
                + p[16][16] * _tmp13
                + p[17][16] * _tmp16
                + p[19][16]
                + p[2][16] * _tmp56)
        + _tmp16
            * (p[0][17] * _tmp55
                + p[1][17] * _tmp57
                + p[15][17] * _tmp17
                + p[16][17] * _tmp13
                + p[17][17] * _tmp16
                + p[19][17]
                + p[2][17] * _tmp56)
        + _tmp17
            * (p[0][15] * _tmp55
                + p[1][15] * _tmp57
                + p[15][15] * _tmp17
                + p[16][15] * _tmp13
                + p[17][15] * _tmp16
                + p[19][15]
                + p[2][15] * _tmp56)
        + _tmp55
            * (p[0][0] * _tmp55
                + p[1][0] * _tmp57
                + p[15][0] * _tmp17
                + p[16][0] * _tmp13
                + p[17][0] * _tmp16
                + p[19][0]
                + p[2][0] * _tmp56)
        + _tmp56
            * (p[0][2] * _tmp55
                + p[1][2] * _tmp57
                + p[15][2] * _tmp17
                + p[16][2] * _tmp13
                + p[17][2] * _tmp16
                + p[19][2]
                + p[2][2] * _tmp56)
        + _tmp57
            * (p[0][1] * _tmp55
                + p[1][1] * _tmp57
                + p[15][1] * _tmp17
                + p[16][1] * _tmp13
                + p[17][1] * _tmp16
                + p[19][1]
                + p[2][1] * _tmp56);
    variance[2] = p[0][20] * _tmp66
        + p[1][20] * _tmp67
        + p[15][20] * _tmp20
        + p[16][20] * _tmp19
        + p[17][20] * _tmp18
        + p[2][20] * _tmp64
        + p[20][20]
        + R
        + _tmp18
            * (p[0][17] * _tmp66
                + p[1][17] * _tmp67
                + p[15][17] * _tmp20
                + p[16][17] * _tmp19
                + p[17][17] * _tmp18
                + p[2][17] * _tmp64
                + p[20][17])
        + _tmp19
            * (p[0][16] * _tmp66
                + p[1][16] * _tmp67
                + p[15][16] * _tmp20
                + p[16][16] * _tmp19
                + p[17][16] * _tmp18
                + p[2][16] * _tmp64
                + p[20][16])
        + _tmp20
            * (p[0][15] * _tmp66
                + p[1][15] * _tmp67
                + p[15][15] * _tmp20
                + p[16][15] * _tmp19
                + p[17][15] * _tmp18
                + p[2][15] * _tmp64
                + p[20][15])
        + _tmp64
            * (p[0][2] * _tmp66
                + p[1][2] * _tmp67
                + p[15][2] * _tmp20
                + p[16][2] * _tmp19
                + p[17][2] * _tmp18
                + p[2][2] * _tmp64
                + p[20][2])
        + _tmp66
            * (p[0][0] * _tmp66
                + p[1][0] * _tmp67
                + p[15][0] * _tmp20
                + p[16][0] * _tmp19
                + p[17][0] * _tmp18
                + p[2][0] * _tmp64
                + p[20][0])
        + _tmp67
            * (p[0][1] * _tmp66
                + p[1][1] * _tmp67
                + p[15][1] * _tmp20
                + p[16][1] * _tmp19
                + p[17][1] * _tmp18
                + p[2][1] * _tmp64
                + p[20][1]);
    h[0] = _tmp44;
    h[1] = _tmp42;
    h[2] = _tmp39;
    h[15] = _tmp2;
    h[16] = _tmp7;
    h[17] = _tmp11;
    h[18] = 1.0;
    (innovation, variance, h)
}
// Source: src/modules/ekf2/EKF/python/ekf_derivation/generated/compute_mag_declination_pred_innov_var_and_h.h; SHA-256 084079d3c00f4e5654d87ae6fd34d1f28f2af6a7c080549cb7c0cba04d694885
pub(super) fn declination(
    state: &StateVector,
    p: &Covariance,
    R: f32,
    epsilon: f32,
) -> (f32, f32, ErrorStateVector) {
    let _tmp0 = epsilon * (sign(state[16]) + 0.5) + state[16];
    let _tmp1 = 1.0 / (square(_tmp0) + square(state[17]));
    let _tmp2 = _tmp1 * state[17];
    let _tmp3 = _tmp0 * _tmp1;
    let mut h = [0.0; 24];
    let prediction = libm::atan2f(state[17], _tmp0);
    let variance = R - _tmp2 * (-p[15][15] * _tmp2 + p[16][15] * _tmp3)
        + _tmp3 * (-p[15][16] * _tmp2 + p[16][16] * _tmp3);
    h[15] = -_tmp2;
    h[16] = _tmp3;
    (prediction, variance, h)
}
fn sign(value: f32) -> f32 {
    if value > 0.0 {
        1.0
    } else if value < 0.0 {
        -1.0
    } else {
        0.0
    }
}
