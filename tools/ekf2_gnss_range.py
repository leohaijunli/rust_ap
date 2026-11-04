#!/usr/bin/env python3
"""Synthetic stationary five-sensor input for the E3 reference, not hardware."""

import argparse
import csv
import json
import sys
from pathlib import Path

from ekf2_baro_mag import FIELDS as BARO_MAG_FIELDS, samples

FIELDS = (*BARO_MAG_FIELDS, "gps_valid", "gps_latitude_deg", "gps_longitude_deg", "gps_altitude_m",
          "gps_vn_m_s", "gps_ve_m_s", "gps_vd_m_s", "range_distance_m", "range_quality")
EXTENDED_FIELDS = (*FIELDS, "gps_fix_type", "gps_hacc_m", "gps_vacc_m", "gps_sacc_m_s",
                   "gps_satellites", "gps_pdop", "gps_spoofed", "range_valid")


def observations(seconds=20.0, case="nominal"):
    if case not in ("nominal", "outliers", "dropout", "quality"):
        raise ValueError("unsupported five-sensor case")
    for row in samples(seconds=seconds, yaw_rate_deg_s=0.0):
        # Explicit synthetic location/datum, stationary velocity, 1 m above
        # a flat surface. This constant field is not a measured/WMM field.
        time = row[0]
        values = [*row, int(time % 200_000 == 0), 47.397742, 8.545594, 100.0, 0.0, 0.0, 0.0, 1.0, 100]
        if case == "nominal":
            yield tuple(values)
            continue
        fix = 2 if case == "quality" and 12_000_000 < time <= 20_000_000 else 3
        range_valid = 1
        if case == "dropout":
            if 12_000_000 < time <= 21_000_000: values[5] = 0
            if 13_000_000 < time <= 15_000_000: range_valid = 0
        if case == "outliers":
            if 12_000_000 < time <= 12_400_000:
                values[6] += 0.001
                values[9:12] = [20.0, 10.0, 10.0]
            if 13_000_000 < time <= 13_200_000:
                values[12] = 35.0 # Explicit out-of-range spike; normal 1m returns resume.
        yield (*values, fix, 0.3, 0.5, 0.1, 12, 1.0, 0, range_valid)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--seconds", type=float, default=20.0)
    args = parser.parse_args()
    try:
        rows = list(observations(args.seconds))
        with args.output.open("x", newline="", encoding="utf-8") as file:
            writer = csv.writer(file, lineterminator="\n")
            writer.writerow(FIELDS)
            writer.writerows(rows)
        print(json.dumps({"samples": len(rows), "gps_samples": sum(row[5] for row in rows), "source": "synthetic"}))
    except (OSError, ValueError) as error:
        print(f"ekf2_gnss_range: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
