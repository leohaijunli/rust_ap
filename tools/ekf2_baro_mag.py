#!/usr/bin/env python3
"""Generate synthetic E2 aid samples paired with ekf2_replay's level yaw IMU."""

import argparse
import csv
import json
import math
import sys
from pathlib import Path

FIELDS = ("timestamp_us", "baro_altitude_m", "mag_x_gauss", "mag_y_gauss", "mag_z_gauss")


def samples(seconds=10.0, rate_hz=50, yaw_rate_deg_s=5.0, altitude_m=100.0, case="nominal"):
    if (not math.isfinite(seconds) or seconds <= 0 or type(rate_hz) is not int or rate_hz <= 0
            or 1_000_000 % rate_hz or not math.isfinite(yaw_rate_deg_s) or not math.isfinite(altitude_m)
            or case not in ("nominal", "outliers", "dropout", "dropout_after_alignment")):
        raise ValueError("invalid E2 synthetic configuration")
    duration = round(seconds * 1e6)
    interval = 1_000_000 // rate_hz
    if duration < interval or duration % interval:
        raise ValueError("duration must contain whole aid periods")
    for time in range(interval, duration + 1, interval):
        if case == "dropout" and 5_000_000 < time <= 6_000_000:
            continue
        if case == "dropout_after_alignment" and 9_000_000 < time <= 10_000_000:
            continue
        yaw = max(0, time - 1_000_000) * 1e-6 * math.radians(yaw_rate_deg_s)
        # Fixed synthetic NED field [0.2, 0, 0.4] G, transformed to body FRD.
        # Height is already pressure altitude, matching PX4 baroSample.hgt;
        # this is not a real BMP388 pressure conversion or field measurement.
        height = altitude_m + (20.0 if case == "outliers" and 2_500_000 < time <= 2_650_000 else 0.0)
        field_x = 0.2 * math.cos(yaw) + (1.0 if case == "outliers" and 3_500_000 < time <= 3_650_000 else 0.0)
        yield (time, height, field_x, -0.2 * math.sin(yaw), 0.4)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--yaw-rate-deg-s", type=float, default=5.0)
    args = parser.parse_args()
    try:
        rows = list(samples(seconds=args.seconds, yaw_rate_deg_s=args.yaw_rate_deg_s))
        with args.output.open("x", newline="", encoding="utf-8") as output:
            writer = csv.writer(output, lineterminator="\n")
            writer.writerow(FIELDS)
            writer.writerows(rows)
        print(json.dumps({"samples": len(rows), "source": "synthetic; not hardware"}))
    except (OSError, ValueError) as error:
        print(f"ekf2_baro_mag: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
