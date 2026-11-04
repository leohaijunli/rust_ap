#!/usr/bin/env python3
"""Synthetic E4 yaw motion with full-rate timing and multi-rate aid jitter."""

import csv
import math

from ekf2_replay import IMU_FIELDS, write_synthetic
from ekf2_gnss_range import FIELDS


def write_inputs(directory, case):
    if case not in ("nominal", "jitter"):
        raise ValueError("unsupported E4 output fixture")
    count = write_synthetic(directory / "imu.csv", directory / "truth.csv", seconds=20.0)
    if case == "jitter":
        with (directory / "imu.csv").open(encoding="utf-8") as file:
            rows = list(csv.DictReader(file))
        time = 0
        for i, row in enumerate(rows):
            dt = (2000, 3000, 2250, 2750)[i % 4]
            time += dt
            row["timestamp_us"] = time
            row["delta_angle_dt_us"] = dt
            row["delta_velocity_dt_us"] = dt
            row["delta_angle_z_rad"] = 0.0 if time <= 1_000_000 else math.radians(5.0) * dt * 1e-6
            row["delta_velocity_z_m_s"] = -9.80665 * dt * 1e-6
        with (directory / "imu.csv").open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=IMU_FIELDS, lineterminator="\n")
            writer.writeheader()
            writer.writerows(rows)
        # Re-time the analytical truth as well; keep it visibly synthetic.
        with (directory / "truth.csv").open(encoding="utf-8") as file:
            reader = csv.DictReader(file)
            truth_fields, truth = reader.fieldnames, list(reader)
        for actual, row in zip(rows, truth):
            time = actual["timestamp_us"]
            row["timestamp_us"] = time
            yaw = math.radians(5.0) * max(time * 1e-6 - 1.0, 0.0)
            for key, value in zip(("q_w", "q_x", "q_y", "q_z"), (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))):
                if key in row: row[key] = value
        with (directory / "truth.csv").open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=truth_fields, lineterminator="\n")
            writer.writeheader()
            writer.writerows(truth)
    with (directory / "aids.csv").open("x", newline="", encoding="utf-8") as file:
        writer = csv.writer(file, lineterminator="\n")
        writer.writerow(FIELDS)
        for i, canonical in enumerate(range(20_000, 20_000_001, 20_000)):
            time = canonical - ((0, 900, 1700, 300)[i % 4] if case == "jitter" else 0)
            yaw = math.radians(5.0) * max(time * 1e-6 - 1.0, 0.0)
            writer.writerow((time, 100.0, 0.2 * math.cos(yaw), -0.2 * math.sin(yaw), 0.4,
                int(canonical % 200_000 == 0), 47.397742, 8.545594, 100.0, 0.0, 0.0, 0.0, 1.0, 100))
    return count
