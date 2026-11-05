#!/usr/bin/env python3
"""Generate QGC JSON from the compiled single runtime registry, never a second table."""

import argparse
import csv
import io
import json
import subprocess
import zlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DIRECTORY = ROOT / "src/core/metadata"


def generated(executable):
    rows = csv.reader(io.StringIO(subprocess.check_output([str(executable)], text=True)))
    entries = []
    for name, kind, default, minimum, maximum in rows:
        entry = {"name": name, "type": kind,
                 "group": "SDLog" if name.startswith("LOG_") else "EKF2" if name.startswith("EKF2_") else "Estimator" if name.startswith("EST_") else "System" if name == "MAV_SYS_ID" else "Status",
                 "shortDesc": f"{name} (unmapped EKF options are reported as unused)" if name.startswith("EKF2_") else name,
                 "min": float(minimum), "max": float(maximum), "default": float(default)}
        # Integer values stay JSON integers; scientific Float limits retain their
        # full f32 range rather than being saturated to i32 by formatting.
        for key in ("min", "max", "default"):
            if kind == "Int32" or abs(entry[key]) < 2**31 and entry[key].is_integer():
                entry[key] = int(entry[key])
        entries.append(entry)
    parameters = '{\n  "version": 1,\n  "parameters": [\n' + ",\n".join(
        "    " + json.dumps(entry, ensure_ascii=False) for entry in entries) + "\n  ]\n}\n"
    crc = zlib.crc32(parameters.encode())
    general = json.dumps({"version": 1, "metadataTypes": [{"type": 1, "uri": "mftp:///component_metadata/parameters.json", "fileCrc": crc}]}, indent=2) + "\n"
    return {"parameters.json": parameters, "general.json": general}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rust", type=Path, required=True)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    for name, text in generated(args.rust).items():
        path = DIRECTORY / name
        if args.check:
            if not path.exists() or path.read_text(encoding="utf-8") != text:
                raise SystemExit(f"QGC metadata differs: {name}")
        else:
            DIRECTORY.mkdir(parents=True, exist_ok=True)
            with path.open("w", encoding="utf-8", newline="\n") as output:
                output.write(text)


if __name__ == "__main__":
    main()
