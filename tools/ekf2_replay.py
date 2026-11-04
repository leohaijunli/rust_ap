#!/usr/bin/env python3
"""Author/replay canonical IMU deltas and inspect fixed PX4 golden output."""

import argparse
import csv
import gzip
import json
import math
import random
import struct
import sys
from pathlib import Path

IMU_FIELDS = (
    "timestamp_us", "delta_angle_dt_us", "delta_velocity_dt_us",
    "delta_angle_x_rad", "delta_angle_y_rad", "delta_angle_z_rad",
    "delta_velocity_x_m_s", "delta_velocity_y_m_s", "delta_velocity_z_m_s",
    "delta_velocity_clipping",
)
GOLDEN_FIELDS = (
    "timestamp_us", "imu_timestamp_us", "control_status", "fault_status",
    *(f"state_{index}" for index in range(25)),
    *(f"cov_{row}_{column}" for row in range(24) for column in range(24)),
)
GRAVITY = 9.80665
TRUTH_FIELDS = ("timestamp_us", "q_w", "q_x", "q_y", "q_z", "v_n", "v_e", "v_d", "p_n", "p_e", "p_d")


def integer(value):
    text = str(value)
    if not text or not text.isascii() or not text.isdecimal():
        raise ValueError(f"invalid unsigned integer: {text}")
    return int(text)


def f32(value):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("non-finite IMU delta")
    try:
        return struct.unpack("<f", struct.pack("<f", value))[0]
    except OverflowError as error:
        raise ValueError("IMU delta outside f32 range") from error


def imu_row(row, previous=0):
    result = {name: integer(row[name]) for name in (*IMU_FIELDS[:3], IMU_FIELDS[-1])}
    result.update({name: f32(row[name]) for name in IMU_FIELDS[3:9]})
    time = result["timestamp_us"]
    angle_dt = result["delta_angle_dt_us"]
    velocity_dt = result["delta_velocity_dt_us"]
    if (not previous < time <= 2**64 - 1 or not 1 <= angle_dt <= 1_000_000
            or not 1 <= velocity_dt <= 1_000_000 or time < max(angle_dt, velocity_dt)
            or result["delta_velocity_clipping"] > 7):
        raise ValueError("invalid IMU time or clipping mask")
    return result


def write_imu(path, rows):
    count = 0
    previous = 0
    with Path(path).open("w", encoding="utf-8", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=IMU_FIELDS, lineterminator="\n")
        writer.writeheader()
        for row in rows:
            row = imu_row(row, previous)
            previous = row["timestamp_us"]
            writer.writerow(row)
            count += 1
    if count == 0:
        raise ValueError("no IMU samples")
    return count


def synthetic_imu(seconds=10.0, rate_hz=400, yaw_rate_deg_s=5.0, seed=0,
                  gyro_noise_rad_s=0.0, accel_noise_m_s2=0.0,
                  clipping_start_s=0.0, clipping_end_s=0.0, clipping_mask=0):
    if (not math.isfinite(seconds) or seconds <= 0 or rate_hz <= 0
            or 1_000_000 % rate_hz or not math.isfinite(yaw_rate_deg_s)
            or not math.isfinite(gyro_noise_rad_s) or gyro_noise_rad_s < 0
            or not math.isfinite(accel_noise_m_s2) or accel_noise_m_s2 < 0
            or not math.isfinite(clipping_start_s) or not math.isfinite(clipping_end_s)
            or not 0 <= clipping_start_s <= clipping_end_s <= seconds
            or type(clipping_mask) is not int or not 0 <= clipping_mask <= 7):
        raise ValueError("invalid synthetic IMU configuration")
    dt_us = 1_000_000 // rate_hz
    duration_us = round(seconds * 1_000_000)
    if duration_us < dt_us or duration_us % dt_us:
        raise ValueError("duration must contain whole IMU sample periods")
    dt_s = dt_us * 1e-6
    yaw_rate = math.radians(yaw_rate_deg_s)
    rng = random.Random(seed)
    for timestamp in range(dt_us, duration_us + 1, dt_us):
        # One second at rest permits reference tilt initialization. Yaw then
        # turns about body Z; a level stationary vehicle still senses -g in Z.
        yaw = max(0, timestamp - 1_000_000) * 1e-6 * yaw_rate
        yaw_before = max(0, timestamp - dt_us - 1_000_000) * 1e-6 * yaw_rate
        angle = [rng.gauss(0, gyro_noise_rad_s) * dt_s for _ in range(3)]
        angle[2] += yaw - yaw_before
        velocity = [rng.gauss(0, accel_noise_m_s2) * dt_s for _ in range(3)]
        velocity[2] -= GRAVITY * dt_s
        # Synthetic clipping tests flag propagation/noise handling only; they
        # do not pretend to simulate a real sensor's rail-induced corruption.
        clipping = clipping_mask if clipping_start_s * 1e6 < timestamp <= clipping_end_s * 1e6 else 0
        row = dict(zip(IMU_FIELDS, [timestamp, dt_us, dt_us, *angle, *velocity, clipping]))
        truth = {"timestamp_us": timestamp, "q_w": math.cos(yaw / 2), "q_x": 0,
                 "q_y": 0, "q_z": math.sin(yaw / 2), "v_n": 0, "v_e": 0,
                 "v_d": 0, "p_n": 0, "p_e": 0, "p_d": 0}
        yield row, truth


def vehicle_imu_rows(log, instance=0):
    matches = [dataset for dataset in log.data_list
               if dataset.name == "vehicle_imu" and dataset.multi_id == instance]
    if len(matches) != 1:
        raise ValueError("requires one selected vehicle_imu dataset; decimated sensor_accel/gyro is not replay data")
    data = matches[0].data
    vector_fields = [f"delta_{kind}[{axis}]" for kind in ("angle", "velocity") for axis in range(3)]
    metadata = ("accel_device_id", "gyro_device_id", "accel_calibration_count", "gyro_calibration_count")
    required = ("timestamp_sample", "delta_angle_dt", "delta_velocity_dt",
                "delta_angle_clipping", "delta_velocity_clipping", *vector_fields, *metadata)
    missing = [name for name in required if name not in data]
    if missing:
        raise ValueError(f"vehicle_imu missing fields: {missing}")
    count = len(data["timestamp_sample"])
    if count == 0 or any(len(data[name]) != count for name in required):
        raise ValueError("empty or inconsistent vehicle_imu arrays")
    identity = tuple(data[name][0] for name in metadata)
    previous = 0
    for index in range(count):
        if tuple(data[name][index] for name in metadata) != identity:
            raise ValueError("IMU device/calibration changed: explicit reset replay is not implemented")
        if data["delta_angle_clipping"][index] != 0:
            raise ValueError("gyro clipping is not represented by the PX4 imuSample contract")
        values = [data["timestamp_sample"][index], data["delta_angle_dt"][index],
                  data["delta_velocity_dt"][index], *(data[name][index] for name in vector_fields),
                  data["delta_velocity_clipping"][index]]
        row = imu_row(dict(zip(IMU_FIELDS, values)), previous)
        previous = row["timestamp_us"]
        yield row


def sensor_combined_rows(log, instance=0):
    """Mirror the pinned single-instance EKF2 rate-times-integral-dt path."""
    if instance != 0:
        raise ValueError("sensor_combined only supports the single-instance path")
    datasets = {}
    for name in ("sensor_combined", "sensor_selection"):
        matches = [dataset for dataset in log.data_list if dataset.name == name and dataset.multi_id == 0]
        if len(matches) != 1:
            raise ValueError(f"requires one {name} dataset")
        datasets[name] = matches[0].data
    data = datasets["sensor_combined"]
    selection = datasets["sensor_selection"]
    vectors = [f"{kind}[{axis}]" for kind in ("gyro_rad", "accelerometer_m_s2") for axis in range(3)]
    calibration = ("accel_calibration_count", "gyro_calibration_count")
    required = ("timestamp", "gyro_integral_dt", "accelerometer_integral_dt", "accelerometer_timestamp_relative",
                "accelerometer_clipping", "gyro_clipping", *vectors, *calibration)
    for name in required:
        if name not in data or len(data[name]) != len(data.get("timestamp", [])):
            raise ValueError(f"missing/inconsistent sensor_combined field: {name}")
    for name in ("timestamp", "accel_device_id", "gyro_device_id"):
        if name not in selection or len(selection[name]) != len(selection.get("timestamp", [])):
            raise ValueError(f"missing/inconsistent sensor_selection field: {name}")
    times = [integer(time) for time in selection["timestamp"]]
    if not times or any(second <= first for first, second in zip(times, times[1:])):
        raise ValueError("invalid sensor_selection time order")
    cursor = 0
    identity = None
    previous = 0
    for index, raw_time in enumerate(data["timestamp"]):
        time = integer(raw_time)
        if times[0] > time:
            raise ValueError("sensor selection is unavailable at sample time")
        while cursor + 1 < len(times) and times[cursor + 1] <= time:
            cursor += 1
        selected = tuple(integer(selection[name][cursor]) for name in ("accel_device_id", "gyro_device_id"))
        if not all(0 < device <= 2**32 - 1 for device in selected):
            raise ValueError("invalid selected IMU device ID")
        current = (*selected, *(data[name][index] for name in calibration))
        if identity is None:
            identity = current
        elif current != identity:
            raise ValueError("IMU device/calibration changed: explicit reset replay is not implemented")
        if data["gyro_clipping"][index] != 0 or int(data["accelerometer_timestamp_relative"][index]) == 2**31 - 1:
            raise ValueError("unsupported gyro clipping or invalid accelerometer timestamp")
        angle_dt = integer(data["gyro_integral_dt"][index])
        velocity_dt = integer(data["accelerometer_integral_dt"][index])
        # Match C++'s f32 multiplication, rather than doing the conversion in
        # f64 and changing the delta bits at the shared input boundary.
        angle_s = f32(angle_dt * f32(1e-6))
        velocity_s = f32(velocity_dt * f32(1e-6))
        deltas = [f32(f32(data[name][index]) * (angle_s if axis < 3 else velocity_s))
                  for axis, name in enumerate(vectors)]
        values = [time, angle_dt, velocity_dt, *deltas, data["accelerometer_clipping"][index]]
        row = imu_row(dict(zip(IMU_FIELDS, values)), previous)
        previous = time
        yield row


def open_csv(path):
    path = Path(path)
    if path.suffix == ".gz":
        return gzip.open(path, "rt", encoding="utf-8", newline="")
    return path.open(encoding="utf-8", newline="")


def check_golden(path):
    count = 0
    previous = 0
    previous_input = 0
    first_delayed = None
    warmup_repeats = 0
    symmetry_error = 0.0
    max_variance = 0.0
    with open_csv(path) as source:
        reader = csv.DictReader(source)
        if tuple(reader.fieldnames or ()) != GOLDEN_FIELDS:
            raise ValueError("unexpected golden state/covariance layout")
        for row in reader:
            if None in row or None in row.values():
                raise ValueError("incomplete golden row")
            time = integer(row["timestamp_us"])
            input_time = integer(row["imu_timestamp_us"])
            if time == 0 or time < previous or input_time <= previous_input or time > input_time:
                raise ValueError("invalid golden time order")
            # PX4 keeps returning the first oldest sample while its delay
            # buffer fills. Preserve that warm-up plateau, never invent times.
            if first_delayed is None:
                first_delayed = time
            elif time == previous:
                if time != first_delayed:
                    raise ValueError("repeated delayed timestamp after warm-up")
                warmup_repeats += 1
            if integer(row["fault_status"]) != 0:
                raise ValueError("PX4 reference reports a fault")
            state = [float(row[f"state_{index}"]) for index in range(25)]
            covariance = [[float(row[f"cov_{r}_{c}"]) for c in range(24)] for r in range(24)]
            if not all(math.isfinite(value) for values in [state, *covariance] for value in values):
                raise ValueError("non-finite golden state/covariance")
            if abs(sum(value * value for value in state[:4]) - 1.0) > 1e-5:
                raise ValueError("golden quaternion is not normalized")
            for r in range(24):
                if covariance[r][r] < 0:
                    raise ValueError("negative golden covariance diagonal")
                max_variance = max(max_variance, covariance[r][r])
                for c in range(r):
                    difference = abs(covariance[r][c] - covariance[c][r])
                    symmetry_error = max(symmetry_error, difference)
                    tolerance = 1e-6 + 1e-5 * max(abs(covariance[r][c]), abs(covariance[c][r]))
                    if difference > tolerance:
                        raise ValueError("golden covariance is not symmetric")
            previous = time
            previous_input = input_time
            count += 1
    if count == 0:
        raise ValueError("no reference update rows")
    return {"steps": count, "last_timestamp_us": previous,
            "warmup_repeated_timestamps": warmup_repeats,
            "max_symmetry_error": symmetry_error, "max_variance": max_variance}


def distinct_paths(first, second):
    first, second = Path(first), Path(second)
    if first.resolve() == second.resolve() or (first.exists() and second.exists() and first.samefile(second)):
        raise ValueError("input/truth/output paths must be distinct files")


def write_synthetic(imu_path, truth_path=None, **configuration):
    rows = synthetic_imu(**configuration)
    if truth_path is None:
        return write_imu(imu_path, (row for row, _ in rows))
    distinct_paths(imu_path, truth_path)
    with Path(truth_path).open("w", encoding="utf-8", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=TRUTH_FIELDS, lineterminator="\n")
        writer.writeheader()
        def imu_with_truth():
            for row, truth in rows:
                writer.writerow(truth)
                yield row
        return write_imu(imu_path, imu_with_truth())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    synth = commands.add_parser("synth", help="level stationary/yaw trajectory; noise is per-sample rate RMS, not PSD")
    synth.add_argument("output", type=Path)
    synth.add_argument("--truth", type=Path)
    synth.add_argument("--seconds", type=float, default=10.0)
    synth.add_argument("--rate-hz", type=int, default=400)
    synth.add_argument("--yaw-rate-deg-s", type=float, default=5.0)
    synth.add_argument("--seed", type=int, default=0)
    synth.add_argument("--gyro-noise-rad-s", type=float, default=0.0)
    synth.add_argument("--accel-noise-m-s2", type=float, default=0.0)
    ulog = commands.add_parser("ulog", help="extract PX4 IMU integration data; never infer deltas from decimated topics")
    ulog.add_argument("input", type=Path)
    ulog.add_argument("output", type=Path)
    ulog.add_argument("--instance", type=int, default=0)
    ulog.add_argument("--source", choices=("vehicle_imu", "sensor_combined"), default="vehicle_imu")
    check = commands.add_parser("check", help="inspect the reference dump, not Rust/PX4 numerical equivalence")
    check.add_argument("input", type=Path)
    args = parser.parse_args()
    try:
        if args.command == "check":
            result = check_golden(args.input)
        elif args.command == "ulog":
            from pyulog import ULog
            distinct_paths(args.input, args.output)
            adapter = vehicle_imu_rows if args.source == "vehicle_imu" else sensor_combined_rows
            result = {"samples": write_imu(args.output, adapter(ULog(str(args.input)), args.instance))}
        else:
            result = {"samples": write_synthetic(
                args.output, args.truth, seconds=args.seconds, rate_hz=args.rate_hz,
                yaw_rate_deg_s=args.yaw_rate_deg_s, seed=args.seed,
                gyro_noise_rad_s=args.gyro_noise_rad_s, accel_noise_m_s2=args.accel_noise_m_s2,
            )}
        print(json.dumps(result))
    except (OSError, ValueError, KeyError, OverflowError) as error:
        print(f"ekf2_replay: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
