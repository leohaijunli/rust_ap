#!/usr/bin/env python3
"""Deterministic E5 reset cases, not real sensor logs or flight qualification."""

import csv
import math

from ekf2_replay import IMU_FIELDS, write_synthetic
from ekf2_gnss_range import EXTENDED_FIELDS, observations

VERTICAL_FAULT_ACCEL_M_S2 = 30.0


def motion(time_us):
    elapsed = max(time_us * 1e-6 - 12.0, 0.0)
    accelerating = min(elapsed, 4.0)
    return accelerating, 0.5 * accelerating * accelerating + 4.0 * max(elapsed - 4.0, 0.0)


def write_inputs(directory, case):
    if case not in ("gnss_reset", "baro_reset", "gsf_yaw", "vertical_clipping"):
        raise ValueError("unsupported reset case")
    count = write_synthetic(directory / "imu.csv", directory / "truth.csv", seconds=30.0, yaw_rate_deg_s=0.0)
    if case in ("gsf_yaw", "vertical_clipping"):
        with (directory / "imu.csv").open(encoding="utf-8") as file: imu = list(csv.DictReader(file))
        for row in imu:
            time = int(row["timestamp_us"])
            if case == "gsf_yaw":
                row["delta_velocity_x_m_s"] = 0.0025 if 12_000_000 < time <= 16_000_000 else 0.0
            elif 12_000_000 < time <= 14_000_000:
                row["delta_velocity_z_m_s"] = (-9.80665 + VERTICAL_FAULT_ACCEL_M_S2) * 0.0025
                row["delta_velocity_clipping"] = 4
        with (directory / "imu.csv").open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=IMU_FIELDS, lineterminator="\n")
            writer.writeheader(); writer.writerows(imu)
        with (directory / "truth.csv").open(encoding="utf-8") as file:
            reader = csv.DictReader(file)
            fields, truth = reader.fieldnames, list(reader)
        if case == "gsf_yaw":
            for row in truth:
                velocity, position = motion(int(row["timestamp_us"]))
                row["v_n"], row["p_n"] = velocity, position
        with (directory / "truth.csv").open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=fields, lineterminator="\n")
            writer.writeheader(); writer.writerows(truth)
    latitude = math.radians(47.397742)
    eccentricity = 0.0818191908425
    factor = 1.0 - (eccentricity * math.sin(latitude)) ** 2
    # Use the exact pinned LatLonAlt curvature convention, not a newer model.
    north_radius = 6_378_137.0 * (1.0 - eccentricity * eccentricity) / (factor * factor * math.sqrt(factor))
    with (directory / "aids.csv").open("x", newline="", encoding="utf-8") as file:
        writer = csv.writer(file, lineterminator="\n")
        writer.writerow(EXTENDED_FIELDS)
        for row in observations(30.0):
            values = list(row)
            time = values[0]
            if case == "gnss_reset" and 12_000_000 < time <= 21_000_000:
                values[6] += 0.001
                values[9:12] = [20.0, 10.0, 0.0]
            if case == "baro_reset" and 12_000_000 < time <= 20_000_000:
                values[1] = 150.0
            if case == "gsf_yaw":
                velocity, position = motion(time)
                values[6] += math.degrees(position / (north_radius + 100.0))
                values[9] = velocity
            writer.writerow((*values, 3, 0.3, 0.5, 0.1, 12, 1.0, 0, 1))
    return count
