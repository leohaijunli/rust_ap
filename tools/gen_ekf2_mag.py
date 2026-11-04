#!/usr/bin/env python3
"""Translate fixed PX4 Y/Z magnetometer Jacobian and variance expressions."""

import argparse
import hashlib
import re
import subprocess
import sys
from pathlib import Path

from gen_params import DEFAULT_REF, ROOT, git_show, require
from gen_ekf2_covariance import expression

DIRECTORY = "src/modules/ekf2/EKF/python/ekf_derivation/generated"
OUTPUT = ROOT / "src/core/estimator/mag_generated.rs"


def scalar(text, known):
    text = re.sub(r"Scalar\((\d+(?:\.\d+)?)\)", r"\1", text)
    text = text.replace("std::atan2", "atan2")
    text = text.replace("(((state(16, 0)) > 0) - ((state(16, 0)) < 0))", "sign(state(16, 0))")
    text = re.sub(r"meas\((\d+),\s*0\)", r"meas[\1]", text)
    # The covariance expression renderer requires explicit Scalar(2) powers.
    text = re.sub(r"std::pow\((state\(\d+,\s*0\)|_tmp\d+|dt),\s*2\)", r"std::pow(\1, Scalar(2))", text)
    return expression(text, known, functions={"atan2": ("libm::atan2f", 2), "sign": ("sign", 1)}, arrays={"meas": (3,)})


def render_axis(document, axis):
    body = document.split("// Intermediate terms", 1)[1].split("// Output terms", 1)
    temporaries = re.findall(r"const Scalar (_tmp\d+)\s*=\s*(.*?);", body[0], re.S)
    known = {"R"}
    lines = [f"pub(super) fn mag_{axis}(state: &StateVector, p: &Covariance, R: f32) -> (f32, ErrorStateVector) {{"]
    for index, (name, value) in enumerate(temporaries):
        require(name == f"_tmp{index}", "invalid magnetometer temporary sequence")
        lines.append(f"    let {name} = {scalar(value, known)};")
        known.add(name)
    variance = re.search(r"^\s*_innov_var\s*=\s*(.*?);", body[1], re.S | re.M)
    require(variance is not None, "missing magnetometer variance")
    lines.append(f"    let variance = {scalar(variance[1], known)};")
    lines.append("    let mut h = [0.0; 24];")
    seen = set()
    for row, value in re.findall(r"_h\((\d+),\s*0\)\s*=\s*(.*?);", body[1], re.S):
        require(0 <= int(row) < 24 and row not in seen, "invalid magnetometer H index")
        seen.add(row)
        lines.append(f"    h[{row}] = {scalar(value, known)};")
    require(len(seen) == 7, "incomplete magnetometer Jacobian")
    return "\n".join([*lines, "    (variance, h)", "}"])


def render_extra(document, kind):
    parts = document.split("// Intermediate terms", 1)[1].split("// Output terms", 1)
    known = {"R", "epsilon"}
    if kind == "x":
        signature = "pub(super) fn mag_x(state: &StateVector, p: &Covariance, meas: [f32; 3], R: f32) -> ([f32; 3], [f32; 3], ErrorStateVector) {"
    else:
        signature = "pub(super) fn declination(state: &StateVector, p: &Covariance, R: f32, epsilon: f32) -> (f32, f32, ErrorStateVector) {"
    lines = [signature]
    terms = re.findall(r"const Scalar (_tmp\d+)\s*=\s*(.*?);", parts[0], re.S)
    require(len(terms) == (68 if kind == "x" else 4), "incomplete magnetic equations")
    for index, (name, value) in enumerate(terms):
        require(name == f"_tmp{index}", "non-sequential magnetic intermediate")
        lines.append(f"    let {name} = {scalar(value, known)};")
        known.add(name)
    lines.append("    let mut h = [0.0; 24];")
    if kind == "x":
        lines.extend(["    let mut innovation = [0.0; 3];", "    let mut variance = [0.0; 3];"])
        for source, target in [("_innov", "innovation"), ("_innov_var", "variance"), ("_hx", "h")]:
            outputs = re.findall(rf"{source}\((\d+),\s*0\)\s*=\s*(.*?);", parts[1], re.S)
            require(len(outputs) == (7 if target == "h" else 3), "incomplete magnetic output")
            expected = {0, 1, 2, 15, 16, 17, 18} if target == "h" else {0, 1, 2}
            require({int(row) for row, _ in outputs} == expected, "invalid magnetic output indices")
            for row, value in outputs:
                lines.append(f"    {target}[{row}] = {scalar(value, known)};")
        lines.append("    (innovation, variance, h)")
    else:
        for source, target in [("_pred", "prediction"), ("_innov_var", "variance")]:
            result = re.search(rf"^\s*{source}\s*=\s*(.*?);", parts[1], re.S | re.M)
            require(result is not None, "missing declination output")
            lines.append(f"    let {target} = {scalar(result[1], known)};")
        outputs = re.findall(r"_h\((\d+),\s*0\)\s*=\s*(.*?);", parts[1], re.S)
        require([int(row) for row, _ in outputs] == [15, 16], "invalid declination Jacobian")
        for row, value in outputs:
            lines.append(f"    h[{row}] = {scalar(value, known)};")
        lines.append("    (prediction, variance, h)")
    return "\n".join([*lines, "}"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path("/home/dev/PX4-Autopilot"))
    parser.add_argument("--output", type=Path, default=OUTPUT)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    try:
        lines = [f"// @generated from fixed PX4 {DEFAULT_REF} by tools/gen_ekf2_mag.py.",
                 "// Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.",
                 "// SPDX-License-Identifier: BSD-3-Clause",
                 "// License: tools/px4_msgs/LICENSE. Preserve expression order.",
                 "#![allow(non_snake_case)]", "use super::{math::square, Covariance, StateVector, ErrorStateVector};"]
        for axis in ("y", "z"):
            source = f"{DIRECTORY}/compute_mag_{axis}_innov_var_and_h.h"
            document = git_show(args.root, DEFAULT_REF, source)
            lines.extend([f"// Source: {source}; SHA-256 {hashlib.sha256(document.encode()).hexdigest()}",
                          render_axis(document, axis)])
        for kind, filename in [("x", "compute_mag_innov_innov_var_and_hx.h"),
                               ("declination", "compute_mag_declination_pred_innov_var_and_h.h")]:
            source = f"{DIRECTORY}/{filename}"
            document = git_show(args.root, DEFAULT_REF, source)
            lines.extend([f"// Source: {source}; SHA-256 {hashlib.sha256(document.encode()).hexdigest()}",
                          render_extra(document, kind)])
        lines.append("fn sign(value: f32) -> f32 { if value > 0.0 { 1.0 } else if value < 0.0 { -1.0 } else { 0.0 } }")
        result = subprocess.check_output(["rustfmt", "--edition", "2021", "--emit", "stdout"],
                                         input="\n".join(lines), text=True)
        if args.check:
            require(args.output.read_text(encoding="utf-8") == result, "magnetometer equation drift")
        else:
            args.output.write_text(result, encoding="utf-8", newline="\n")
        print("magnetometer: fixed XYZ innovations/Jacobians and declination generated")
    except (OSError, ValueError, SyntaxError, IndexError, subprocess.CalledProcessError) as error:
        print(f"gen_ekf2_mag: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
