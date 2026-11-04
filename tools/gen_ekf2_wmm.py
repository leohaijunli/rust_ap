#!/usr/bin/env python3
"""Translate shipped WMM tables from pinned PX4; never fetch a newer model."""

import argparse
import hashlib
import re
import subprocess
import struct
import sys
from pathlib import Path

from gen_params import DEFAULT_REF, ROOT, git_show, require

SOURCE = "src/lib/world_magnetic_model/geo_magnetic_tables.hpp"
OUTPUT = ROOT / "src/core/estimator/wmm_generated.rs"


def render(document):
    lines = [f"// @generated from fixed PX4 {DEFAULT_REF} by tools/gen_ekf2_wmm.py.",
             f"// Source: {SOURCE}; SHA-256 {hashlib.sha256(document.encode()).hexdigest()}",
             "// Copyright (c) 2020-2024 PX4 Development Team. All rights reserved.",
             "// SPDX-License-Identifier: BSD-3-Clause; license: tools/px4_msgs/LICENSE."]
    for name in ("declination", "inclination", "totalintensity"):
        match = re.search(rf"int16_t {name}_table\[19\]\[37\]\s*\{{(.*?)\n\}};", document, re.S)
        require(match is not None, f"missing fixed WMM {name} table")
        body = re.sub(r"/\*.*?\*/|//[^\n]*", "", match[1], flags=re.S)
        rows = [[int(value) for value in re.findall(r"-?\d+", row)] for row in re.findall(r"\{([^{}]*)\}", body)]
        require(len(rows) == 19 and all(len(row) == 37 for row in rows), "invalid fixed WMM dimensions")
        require(all(-32768 <= value <= 32767 for row in rows for value in row), "WMM value outside int16")
        lines.append(f"pub(super) static {name.upper()}: [[i16; 37]; 19] = [")
        lines.extend("    [" + ", ".join(map(str, row)) + "]," for row in rows)
        lines.append("];")
        scale = re.search(rf"WMM_{name.upper()}_SCALE_TO_\w+\s*=\s*([\d.]+)f;", document)
        require(scale is not None, "missing fixed WMM scale")
        bits = struct.unpack("<I", struct.pack("<f", float(scale[1])))[0]
        lines.append(f"// Source f32 scale: {scale[1]}f")
        lines.append(f"pub(super) const {name.upper()}_SCALE: f32 = f32::from_bits(0x{bits:08x});")
    return subprocess.check_output(["rustfmt", "--edition", "2021", "--emit", "stdout"], input="\n".join(lines), text=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path("/home/dev/PX4-Autopilot"))
    parser.add_argument("--output", type=Path, default=OUTPUT)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    try:
        result = render(git_show(args.root, DEFAULT_REF, SOURCE))
        if args.check: require(args.output.read_text(encoding="utf-8") == result, "fixed WMM table drift")
        else: args.output.write_text(result, encoding="utf-8", newline="\n")
        print("WMM: fixed 3 x 19 x 37 int16 tables and source scales")
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        print(f"gen_ekf2_wmm: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
