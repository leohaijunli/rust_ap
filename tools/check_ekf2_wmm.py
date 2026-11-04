#!/usr/bin/env python3
"""Compare fixed PX4 and Rust WMM lookup bits at interior/globe boundary points."""

import argparse
import hashlib
import json
import struct
import subprocess
import sys
from pathlib import Path

from check_ekf2_host import ROOT
from gen_params import DEFAULT_REF


def values(command):
    result = subprocess.run([str(arg) for arg in command], check=True, text=True, capture_output=True)
    data = [float(value) for value in result.stdout.strip().split(",")]
    if len(data) != 3:
        raise ValueError("WMM probe output width differs")
    return data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rust", type=Path, required=True)
    args = parser.parse_args()
    root = ROOT / "target/ekf2-host-gnss-range"
    try:
        manifest = json.loads((root / "source_manifest.json").read_text())
        cpp = root / "build/wmm_probe"
        if manifest["px4_ref"] != DEFAULT_REF or hashlib.sha256(cpp.read_bytes()).hexdigest() != manifest["wmm_probe_sha256"]:
            raise ValueError("fixed WMM probe checksum/ref mismatch")
        points = [(lat, lon) for lat in (-90, -89.9, 0, 47.397742, 90) for lon in (-180, -170, 8.545594, 179.99, 180, 190)]
        maximum = 0.0
        for lat, lon in points:
            reference = values([cpp, lat, lon])
            actual = values([args.rust.resolve(), lat, lon])
            if any(struct.pack("<f", a) != struct.pack("<f", b) for a, b in zip(reference, actual)):
                raise ValueError(f"WMM f32 bits differ at {lat}/{lon}: {actual} vs {reference}")
            maximum = max(maximum, *(abs(a - b) for a, b in zip(reference, actual)))
        print(json.dumps({"points": len(points), "different_f32_fields": 0, "max_text_parse_error": maximum,
                          "hardware_validation": "not run; mathematical table probe"}, indent=2))
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as error:
        print(f"check_ekf2_wmm: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
