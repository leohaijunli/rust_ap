#!/usr/bin/env python3
"""E6-H: independently parse synthetic PX4 ULog and standard MAVLink v2 output."""

import argparse
import csv
import hashlib
import json
import math
import os
import subprocess
import sys
from collections import Counter
from pathlib import Path

import numpy as np
from pymavlink.dialects.v20 import common as mavlink
from pyulog import ULog

from check_ulog import PX4_REVISION, px4_fields

ROOT = Path(__file__).resolve().parents[1]
MESSAGES = {
    "vehicle_attitude": "VehicleAttitude.msg",
    "vehicle_angular_velocity": "VehicleAngularVelocity.msg",
    "vehicle_local_position": "VehicleLocalPosition.msg",
    "vehicle_global_position": "VehicleGlobalPosition.msg",
    "estimator_status": "EstimatorStatus.msg",
    "estimator_states": "EstimatorStates.msg",
    "estimator_aid_src_baro_hgt": "EstimatorAidSource1d.msg",
    "estimator_aid_src_mag": "EstimatorAidSource3d.msg",
    "estimator_aid_src_rng_hgt": "EstimatorAidSource1d.msg",
    "estimator_aid_src_gnss_pos": "EstimatorAidSource2d.msg",
    "estimator_aid_src_gnss_vel": "EstimatorAidSource3d.msg",
    "estimator_aid_src_gnss_hgt": "EstimatorAidSource1d.msg",
}
AID_TOPICS = list(MESSAGES)[6:]
UNKNOWN_RATIOS = ["hdg_test_ratio", "vel_test_ratio", "pos_test_ratio", "hgt_test_ratio",
                  "tas_test_ratio", "hagl_test_ratio", "beta_test_ratio"]


def require(condition, message):
    if not condition:
        raise ValueError(message)


def equal(actual, expected, label, *, rel_tol=1e-6, abs_tol=2e-7):
    """NaN stays unknown; +inf is unbounded, not zero or a finite fake sigma."""
    if isinstance(expected, (int, bool, np.integer, np.bool_)):
        require(actual == expected, f"{label}: {actual!r} != {expected!r}")
        return
    a, b = float(actual), float(expected)
    if math.isnan(b):
        matches = math.isnan(a)
    elif math.isinf(b):
        matches = a == b
    else:
        matches = math.isclose(a, b, rel_tol=rel_tol, abs_tol=abs_tol)
    require(matches, f"{label}: {actual!r} != {expected!r}")


def columns(data, prefix, count):
    return [data[f"{prefix}[{i}]"] for i in range(count)]


def trace_vector(row, prefix, count):
    return [float(row[f"{prefix}{i}"]) for i in range(count)]


def rotation(q):
    w, x, y, z = q
    return [[1 - 2 * (y*y + z*z), 2 * (x*y - w*z), 2 * (x*z + w*y)],
            [2 * (x*y + w*z), 1 - 2 * (x*x + z*z), 2 * (y*z - w*x)],
            [2 * (x*z - w*y), 2 * (y*z + w*x), 1 - 2 * (x*x + y*y)]]


def yaw(q):
    r = rotation(q)
    return math.atan2(r[1][0], r[0][0])


def body_rotation_variance(row):
    r = rotation(trace_vector(row, "state", 4))
    covariance = trace_vector(row, "rot_cov", 9)
    return [sum(r[i][axis] * covariance[i * 3 + j] * r[j][axis]
                for i in range(3) for j in range(3)) for axis in range(3)]


def read_csv(path):
    with path.open(encoding="utf-8", newline="") as file:
        return list(csv.DictReader(file))


def dataset_rows(log, name):
    dataset = next((item for item in log.data_list if item.name == name), None)
    require(dataset is not None, f"missing estimator dataset: {name}")
    return [{key: values[index] for key, values in dataset.data.items()}
            for index in range(len(dataset.data["timestamp"]))]


def check_schema(log):
    require(set(log.message_formats) == set(MESSAGES), "ULog format registry mismatch")
    for topic, filename in MESSAGES.items():
        require(log.message_formats[topic].fields == px4_fields(filename),
                f"{topic}: field order/type/array does not match pinned PX4 .msg")


def verify_upstream(root):
    """CI also compares the field-only extracts with the fixed upstream commit."""
    ref = subprocess.check_output(["git", "-C", str(root), "rev-parse", "HEAD"], text=True).strip()
    require(ref == PX4_REVISION, "PX4 checkout is not the fixed E6 message source")
    for filename in sorted(set(MESSAGES.values())):
        path = root / "msg" / ("versioned" if filename.startswith("Vehicle") else "") / filename
        declarations = [line.partition("#")[0].strip() for line in path.read_text(encoding="utf-8").splitlines()]
        declarations = [line.split() for line in declarations if line and "=" not in line]
        local = ROOT / "tools" / "px4_msgs" / filename
        extracted = [line.split() for line in local.read_text(encoding="utf-8").splitlines()
                     if line and not line.startswith("#")]
        require(declarations == extracted, f"pinned message extract changed: {filename}")


def check_local(row, raw, no_gps):
    healthy = bool(int(raw["healthy"]))
    control = int(raw["control"])
    horizontal = healthy and not no_gps and bool(control & ((1 << 2) | (1 << 44)))
    vertical = healthy and bool(control & ((1 << 9) | (1 << 10) | (1 << 11) | (1 << 44)))
    expected = {
        "timestamp": int(raw["timestamp"]), "timestamp_sample": int(raw["latest"]),
        "xy_valid": horizontal, "v_xy_valid": horizontal, "z_valid": vertical, "v_z_valid": vertical,
        "heading": yaw(trace_vector(raw, "q", 4)),
        "heading_var": float(raw["cov2"]), "tilt_var": float(raw["cov0"]) + float(raw["cov1"]),
        "delta_heading": yaw(trace_vector(raw, "delta_q", 4)), "heading_reset_counter": int(raw["reset0"]),
        "xy_reset_counter": int(raw["reset3"]), "z_reset_counter": int(raw["reset4"]),
        "vxy_reset_counter": int(raw["reset1"]), "vz_reset_counter": int(raw["reset2"]),
        "delta_z": float(raw["delta_p2"]), "delta_vz": float(raw["delta_v2"]),
        "dist_bottom": max(float(raw["state24"]) - float(raw["state9"]), 0),
        "dist_bottom_var": float(raw["cov23"]), "delta_dist_bottom": float(raw["delta_hagl"]),
        "dist_bottom_reset_counter": int(raw["reset5"]),
        "dist_bottom_sensor_bitfield": bool(control & (1 << 39)),
        "eph": math.sqrt(float(raw["cov6"]) + float(raw["cov7"])),
        "epv": math.sqrt(float(raw["cov8"])),
        "evh": math.sqrt(float(raw["cov3"]) + float(raw["cov4"])),
        "evv": math.sqrt(float(raw["cov5"])),
        "dead_reckoning": bool(control & ((1 << 29) | (1 << 30))),
        "vxy_max": math.inf, "vz_max": math.inf, "hagl_max_xy": math.inf,
    }
    relying_on_range = control & ((1 << 9) | (1 << 10) | (1 << 11)) == (1 << 10)
    expected |= {"hagl_min": 0.1 if relying_on_range else math.inf,
                 "hagl_max_z": 27.0 if relying_on_range else math.inf}
    for i, field in enumerate(("x", "y", "z")):
        expected[field] = float(raw[f"p{i}"])
    for i, field in enumerate(("vx", "vy", "vz")):
        expected[field] = float(raw[f"v{i}"])
    for prefix, raw_prefix in (("delta_xy", "delta_p"), ("delta_vxy", "delta_v")):
        for i in range(2):
            expected[f"{prefix}[{i}]"] = float(raw[f"{raw_prefix}{i}"])
    for field, value in expected.items():
        equal(row[field], value, f"vehicle_local_position.{field}")
    for field in ("ax", "ay", "az", "z_deriv", "unaided_heading"):
        require(math.isfinite(row[field]), f"local {field} not finite")
    if not row["xy_global"]:
        require(not row["z_global"] and row["ref_timestamp"] == 0, "unknown local reference advertised")
        for field in ("ref_lat", "ref_lon", "ref_alt"):
            equal(row[field], math.nan, f"local.{field}")
    else:
        require(not no_gps and row["z_global"] and 0 < row["ref_timestamp"] <= row["timestamp_sample"],
                "invalid global-reference timestamp/flags")
        for field in ("ref_lat", "ref_lon", "ref_alt"):
            require(math.isfinite(row[field]), f"global reference {field} not finite")
    require(not row["heading_good_for_control"] or healthy, "unhealthy control heading advertised")


def check_status(row, raw, local, gnss_glitch=False):
    expected = {"timestamp_sample": int(raw["delayed"]), "control_mode_flags": int(raw["control"]),
                "filter_fault_flags": int(raw["fault"]), "health_flags": 0, "timeout_flags": 0,
                "time_slip": math.nan}
    for field in UNKNOWN_RATIOS + ["mag_inclination_deg", "mag_inclination_ref_deg", "mag_strength_gs", "mag_strength_ref_gs"]:
        expected[field] = math.nan
    for field in ("accel_device_id", "gyro_device_id", "baro_device_id", "mag_device_id"):
        expected[field] = 0
    for field in ("pre_flt_fail_innov_heading", "pre_flt_fail_innov_height", "pre_flt_fail_innov_pos_horiz",
                  "pre_flt_fail_innov_vel_horiz", "pre_flt_fail_innov_vel_vert"):
        expected[field] = 0
    expected["pre_flt_fail_mag_field_disturbed"] = bool(int(raw["control"]) & (1 << 16))
    for field, index in (("reset_count_vel_ne", 1), ("reset_count_vel_d", 2), ("reset_count_pos_ne", 3),
                         ("reset_count_pod_d", 4), ("reset_count_quat", 0)):
        expected[field] = int(raw[f"reset{index}"])
    for i in range(3):
        expected[f"output_tracking_error[{i}]"] = float(raw[f"tracking{i}"])
    expected["pos_horiz_accuracy"] = local["eph"] if local["xy_global"] else math.inf
    expected["pos_vert_accuracy"] = local["epv"] if local["xy_global"] else math.inf
    for field, value in expected.items():
        equal(row[field], value, f"estimator_status.{field}")
    flags = int(row["solution_status_flags"])
    for bit, valid in ((0, bool(int(raw["healthy"]))), (1, local["xy_valid"]), (2, local["v_z_valid"]),
                       (3, local["xy_valid"]), (4, local["xy_global"] and local["xy_valid"]),
                       (6, local["dist_bottom_valid"])):
        require(bool(flags & (1 << bit)) == bool(valid), f"solution validity bit {bit} mismatch")
    require(bool(flags & (1 << 10)) == gnss_glitch, "GNSS glitch flag does not match raw aid test ratios")


def check_aids(log, directory, instance):
    raw = read_csv(directory / "aids.csv")
    expected = {(int(row["slot"]), int(row["timestamp"])): row for row in raw}
    checked = 0
    for slot, name in enumerate(AID_TOPICS):
        if name not in {item.name for item in log.data_list}:
            continue
        dimensions = 2 if slot == 3 else 3 if slot in (1, 4) else 1
        rows = dataset_rows(log, name)
        samples = [int(row["timestamp_sample"]) for row in rows]
        require(all(a < b for a, b in zip(samples, samples[1:])), f"{name}: duplicate/stale aid published")
        for row in rows:
            key = (slot, int(row["timestamp"]))
            require(key in expected, f"{name}: aid not present in core trace")
            source = expected.pop(key)
            for field, value in {"timestamp_sample": int(source["sample"]), "estimator_instance": instance,
                                 "device_id": 0, "time_last_fuse": 0, "fused": int(source["fused"]),
                                 "innovation_rejected": int(source["rejected"])}.items():
                equal(row[field], value, f"{name}.{field}")
            for axis in range(dimensions):
                suffix = f"[{axis}]" if dimensions > 1 else ""
                for field, prefix in (("innovation", "innovation"), ("innovation_variance", "variance"), ("test_ratio", "ratio")):
                    equal(row[field + suffix], float(source[f"{prefix}{axis}"]), f"{name}.{field}{suffix}")
                for field in ("observation", "observation_variance", "innovation_filtered", "test_ratio_filtered"):
                    equal(row[field + suffix], math.nan, f"{name}.{field}{suffix}")
            checked += 1
    require(not expected, "core aids missing from ULog")
    return checked


def read_mavlink(path):
    parser = mavlink.MAVLink(None)
    try:
        messages = parser.parse_buffer(path.read_bytes()) or []
    except mavlink.MAVError as error:
        raise ValueError(f"MAVLink CRC/frame decode failed: {error}") from error
    require(parser.buf_len() == 0, "truncated MAVLink frame")
    require(messages and all(message.get_type() != "BAD_DATA" for message in messages), "invalid/empty MAVLink stream")
    return messages


def global_int_fields(global_row, local, home_altitude=None):
    """Independent PX4 casts, including f32 arithmetic before integer truncation."""
    scaled = lambda value, scale: int(np.float32(np.float32(value) * np.float32(scale)))
    heading = np.float32(local["heading"])
    tau = np.float32(2 * math.pi)
    wrapped = np.float32(heading % tau)
    degrees = np.float32(wrapped * np.float32(180 / math.pi))
    return {"lat": int(float(global_row["lat"]) * 1e7), "lon": int(float(global_row["lon"]) * 1e7),
            "alt": scaled(global_row["alt"], 1000),
            "relative_alt": scaled(np.float32(global_row["alt"]) - np.float32(home_altitude or 0), 1000),
            "vx": scaled(local["vx"], 100), "vy": scaled(local["vy"], 100), "vz": scaled(local["vz"], 100),
            "hdg": scaled(degrees, 100)}


def check_covariance(actual, values, label):
    expected = [math.nan] * 21
    for index, value in values.items():
        expected[index] = value
    for index, (a, b) in enumerate(zip(actual, expected, strict=True)):
        equal(a, b, f"{label}[{index}]")


def check_frames(log, trace, messages, no_gps):
    rows = {name: {int(row["timestamp"]): row for row in dataset_rows(log, name)}
            for name in ("vehicle_attitude", "vehicle_angular_velocity", "vehicle_local_position", "estimator_status")}
    global_rows = {} if no_gps else {int(row["timestamp"]): row for row in dataset_rows(log, "vehicle_global_position")}
    by_ms = {time // 1000: time for time in rows["vehicle_local_position"]}
    counts = Counter()
    for sequence, message in enumerate(messages):
        kind = message.get_type()
        counts[kind] += 1
        require(message.get_seq() == sequence % 256, "MAVLink v2 sequence/wrap mismatch")
        require(message.get_srcSystem() == 1 and message.get_srcComponent() == (197 if no_gps else 1),
                "MAVLink estimator source/independent component mismatch")
        if kind == "ESTIMATOR_STATUS":
            require(message.time_usec in rows["estimator_status"], "ESTIMATOR_STATUS lost microsecond timestamp")
            row = rows["estimator_status"][message.time_usec]
            require(message.flags == row["solution_status_flags"], "MAVLink solution flags mismatch")
            for field, source in (("vel_ratio", "vel_test_ratio"), ("pos_horiz_ratio", "pos_test_ratio"),
                                  ("pos_vert_ratio", "hgt_test_ratio"), ("mag_ratio", "hdg_test_ratio"),
                                  ("hagl_ratio", "hagl_test_ratio"), ("tas_ratio", "tas_test_ratio"),
                                  ("pos_horiz_accuracy", "pos_horiz_accuracy"), ("pos_vert_accuracy", "pos_vert_accuracy")):
                equal(getattr(message, field), row[source], f"MAVLink ESTIMATOR_STATUS.{field}")
            continue
        require(kind in ({"ODOMETRY"} if no_gps else {"ATTITUDE_QUATERNION", "LOCAL_POSITION_NED", "GLOBAL_POSITION_INT"}),
                f"unexpected/forbidden estimator stream: {kind}")
        timestamp = message.time_usec + 100 if kind == "ODOMETRY" else by_ms.get(message.time_boot_ms)
        require(timestamp in trace, f"{kind}: sample/publication timestamp mismatch")
        local = rows["vehicle_local_position"][timestamp]
        if kind in ("ATTITUDE_QUATERNION", "ODOMETRY"):
            require(timestamp in rows["vehicle_attitude"], "unhealthy attitude sent")
            att = rows["vehicle_attitude"][timestamp]
            angular = rows["vehicle_angular_velocity"][timestamp]
            for index, value in enumerate(columns(att, "q", 4)):
                actual = message.q[index] if kind == "ODOMETRY" else getattr(message, f"q{index + 1}")
                equal(actual, value, f"{kind}.quaternion[{index}]")
            for index, field in enumerate(("rollspeed", "pitchspeed", "yawspeed")):
                equal(getattr(message, field), angular[f"xyz[{index}]"], f"{kind}.{field}")
        if kind == "ATTITUDE_QUATERNION":
            require(message.repr_offset_q == [0.0] * 4, "ordinary attitude representation offset mismatch")
        elif kind in ("LOCAL_POSITION_NED", "ODOMETRY"):
            for field in ("x", "y", "z", "vx", "vy", "vz"):
                equal(getattr(message, field), local[field], f"{kind}.{field}")
            if kind == "LOCAL_POSITION_NED":
                require(all(local[field] for field in ("xy_valid", "z_valid", "v_xy_valid", "v_z_valid")),
                        "LOCAL_POSITION_NED published an invalid axis")
            else:
                require(message.frame_id == mavlink.MAV_FRAME_LOCAL_NED and message.child_frame_id == mavlink.MAV_FRAME_LOCAL_NED,
                        "ODOMETRY frame semantics mismatch")
                require(message.estimator_type == mavlink.MAV_ESTIMATOR_TYPE_AUTOPILOT and message.quality == 0,
                        "ODOMETRY source/quality mismatch")
                raw = trace[timestamp]
                require(message.reset_counter == sum(int(raw[f"reset{i}"]) for i in range(5)) % 256,
                        "ODOMETRY reset sum/wrap mismatch")
                orientation = body_rotation_variance(raw)
                check_covariance(message.pose_covariance, {0: math.inf, 6: math.inf, 11: float(raw["cov8"]),
                                 15: orientation[0], 18: orientation[1], 20: orientation[2]}, "ODOMETRY.pose_covariance")
                check_covariance(message.velocity_covariance, {0: math.inf, 6: math.inf, 11: float(raw["cov5"])}, "ODOMETRY.velocity_covariance")
        else:
            require(timestamp in global_rows, "GLOBAL_POSITION_INT without a valid global snapshot")
            expected = global_int_fields(global_rows[timestamp], local)
            for field, value in expected.items():
                require(getattr(message, field) == value, f"GLOBAL_POSITION_INT.{field}: unit/cast mismatch")
    expected = {"ESTIMATOR_STATUS": len(rows["estimator_status"])}
    if no_gps:
        expected["ODOMETRY"] = len(rows["vehicle_attitude"])
    else:
        expected |= {"ATTITUDE_QUATERNION": len(rows["vehicle_attitude"]), "GLOBAL_POSITION_INT": len(global_rows),
                     "LOCAL_POSITION_NED": sum(all(row[key] for key in ("xy_valid", "z_valid", "v_xy_valid", "v_z_valid"))
                                                for row in rows["vehicle_local_position"].values())}
    require(dict(counts) == expected, "standard MAVLink stream counts/validity suppression mismatch")
    return dict(counts)


def check(directory, mode):
    require(mode in ("gnss", "no-gps"), "unsupported E6 mode")
    no_gps = mode == "no-gps"
    fixture = json.loads((directory / "fixture.json").read_text(encoding="utf-8"))
    require(fixture["schema"] == 1 and fixture["px4_ref"] == PX4_REVISION and fixture["mode"] == mode
            and fixture["synthetic"] is True and fixture["hardware_validation"] == "not run", "fixture provenance mismatch")
    log = ULog(str(directory / "estimator.ulg"))
    require(not log.file_corruption and not log.dropouts, "ULog corruption/dropout in host fixture")
    require(log.msg_info_dict["px4_msg_revision"] == PX4_REVISION and log.msg_info_dict["body_frame"] == "FRD"
            and log.msg_info_dict["world_frame"] == "NED", "ULog revision/frame metadata mismatch")
    require("synthetic host only" in log.msg_info_dict["rust_e6_fixture"], "synthetic fixture not labelled")
    require(len(log.initial_parameters) == 142, "EKF parameter snapshot missing")
    check_schema(log)
    names = {item.name for item in log.data_list}
    expected_names = set(MESSAGES) - ({"vehicle_global_position", *AID_TOPICS[3:]} if no_gps else set())
    require(names == expected_names and len(log.data_list) == len(expected_names),
            "standard ULog topic set/forbidden no-GPS global output mismatch")
    for dataset in log.data_list:
        require(dataset.multi_id == int(no_gps), "estimator ULog multi_id mismatch")
        times = dataset.data["timestamp"].tolist()
        require(times and all(a < b for a, b in zip(times, times[1:])), f"{dataset.name}: publication time regressed/repeated")
        require(all(0 < int(sample) <= int(time) for time, sample in zip(times, dataset.data["timestamp_sample"], strict=True)),
                f"{dataset.name}: sample/publication clock mismatch")
    raw_rows = read_csv(directory / "core.csv")
    trace = {int(row["timestamp"]): row for row in raw_rows}
    require(len(trace) == fixture["publications"] == 1300 and fixture["samples"] == 26000, "E6 fixture duration/count mismatch")
    locals_ = dataset_rows(log, "vehicle_local_position")
    require(len(locals_) == len(trace), "local output omitted diagnostic snapshots")
    local_by_time = {int(row["timestamp"]): row for row in locals_}
    for row in locals_:
        check_local(row, trace[int(row["timestamp"])], no_gps)
    aid_rows = read_csv(directory / "aids.csv")
    aid_index = 0
    gnss_ratios = {3: [0.0, 0.0], 4: [0.0, 0.0, 0.0]}
    for row in dataset_rows(log, "estimator_status"):
        timestamp = int(row["timestamp"])
        while aid_index < len(aid_rows) and int(aid_rows[aid_index]["timestamp"]) <= timestamp:
            aid = aid_rows[aid_index]
            slot = int(aid["slot"])
            if slot in gnss_ratios:
                gnss_ratios[slot] = trace_vector(aid, "ratio", len(gnss_ratios[slot]))
            aid_index += 1
        glitch = any(ratio > 1 for ratios in gnss_ratios.values() for ratio in ratios)
        check_status(row, trace[timestamp], local_by_time[timestamp], glitch)
    for row in dataset_rows(log, "estimator_states"):
        raw = trace[int(row["timestamp"])]
        require(row["n_states"] == 25 and row["timestamp_sample"] == int(raw["delayed"]), "state dimension/delayed clock mismatch")
        for name, prefix, count in (("states", "state", 25), ("covariances", "cov", 24)):
            for index in range(count):
                equal(row[f"{name}[{index}]"], float(raw[f"{prefix}{index}"]), f"estimator_states.{name}[{index}]")
        require(all(row[f"covariances[{i}]"] >= 0 for i in range(24)), "negative covariance in estimator_states")
    for row in dataset_rows(log, "vehicle_attitude"):
        raw = trace[int(row["timestamp"])]
        require(int(raw["healthy"]) == 1 and row["timestamp_sample"] == int(raw["latest"]), "invalid attitude publication")
        for name, prefix in (("q", "q"), ("delta_q_reset", "delta_q")):
            for index in range(4):
                equal(row[f"{name}[{index}]"], float(raw[f"{prefix}{index}"]), f"vehicle_attitude.{name}[{index}]")
        equal(row["quat_reset_counter"], int(raw["reset0"]), "attitude reset counter")
        require(abs(math.sqrt(sum(value * value for value in columns(row, "q", 4))) - 1) <= 1e-5, "attitude quaternion norm")
    for row in dataset_rows(log, "vehicle_angular_velocity"):
        require(row["timestamp_sample"] == int(trace[int(row["timestamp"])]["latest"]), "angular velocity sample clock")
        require(all(math.isfinite(value) for value in columns(row, "xyz", 3)), "non-finite angular velocity")
        for value in columns(row, "xyz_derivative", 3):
            equal(value, math.nan, "unknown angular acceleration")
    if not no_gps:
        for row in dataset_rows(log, "vehicle_global_position"):
            raw = trace[int(row["timestamp"])]
            local = local_by_time[int(row["timestamp"])]
            require(row["lat_lon_valid"] and row["alt_valid"] and local["xy_valid"] and local["z_valid"], "invalid global output")
            for name, source in (("lat", "global_lat"), ("lon", "global_lon"), ("alt", "global_alt")):
                if name in ("lat", "lon"):
                    equal(row[name], float(raw[source]), f"vehicle_global_position.{name}", rel_tol=0, abs_tol=1e-10)
                else:
                    equal(row[name], float(raw[source]), f"vehicle_global_position.{name}")
            for name, value in {"alt_ellipsoid": math.nan, "delta_alt": -float(raw["delta_p2"]),
                                "delta_terrain": -float(raw["delta_p2"]), "terrain_alt": -float(raw["state24"]),
                                "lat_lon_reset_counter": int(raw["reset3"]), "alt_reset_counter": int(raw["reset4"]),
                                "terrain_reset_counter": int(raw["reset5"]), "eph": local["eph"], "epv": local["epv"],
                                "terrain_alt_valid": local["dist_bottom_valid"], "dead_reckoning": local["dead_reckoning"]}.items():
                equal(row[name], value, f"vehicle_global_position.{name}")
    aid_count = check_aids(log, directory, int(no_gps))
    messages = read_mavlink(directory / "mavlink.bin")
    require(len(messages) == fixture["mavlink_frames"], "MAVLink frame count mismatch")
    counts = check_frames(log, trace, messages, no_gps)
    latest = raw_rows[-1]
    theoretical = 0.5 * 0.05 * 60**2
    equal(float(latest["truth_north"]), theoretical, "60-second free integral")
    drift_error = math.hypot(float(latest["p0"]) - theoretical, float(latest["p1"]))
    stationary_bias_error = math.hypot(float(latest["p0"]), float(latest["p1"]))
    covered = sum(math.hypot(float(row["p0"]) - float(row["truth_north"]), float(row["p1"]))
                  <= 3 * math.sqrt(message.pose_covariance[0] + message.pose_covariance[6])
                  for message in messages if message.get_type() == "ODOMETRY"
                  for row in [trace[message.time_usec + 100]])
    if no_gps:
        require(0.5 * theoretical <= drift_error <= 1.5 * theoretical, "ambiguous no-GPS motion error is not the expected drift scale")
        require(covered == counts["ODOMETRY"], "published no-GPS uncertainty does not cover drift")
    return {"schema": 1, "px4_ref": PX4_REVISION, "mode": mode, "synthetic": True,
            "ulog_topics": {item.name: len(item.data["timestamp"]) for item in log.data_list},
            "schema_fields_checked": sum(len(px4_fields(name)) for name in set(MESSAGES.values())),
            "publications": len(trace), "aid_records_checked": aid_count, "mavlink": counts,
            "free_integral_60s_m": theoretical, "final_motion_error_m": drift_error,
            "final_stationary_bias_error_m": stationary_bias_error,
            "conditioned_local_horizontal_sigma_m": float(locals_[-1]["eph"]),
            "odometry_horizontal_sigma": "unbounded (+infinity); not a finite accuracy claim" if no_gps else "not emitted in main stream",
            "odometry_3sigma_covered": covered, "hardware_validation": "not run",
            "artifacts": {name: hashlib.sha256((directory / name).read_bytes()).hexdigest()
                          for name in ("estimator.ulg", "mavlink.bin", "core.csv", "aids.csv", "fixture.json")}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", choices=("no-gps", "gnss"), default="no-gps")
    parser.add_argument("--rust", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--check-only", action="store_true")
    parser.add_argument("--px4-root", type=Path)
    args = parser.parse_args()
    directory = args.output or ROOT / "target/ekf2-host-standard" / args.case
    try:
        if args.px4_root:
            verify_upstream(args.px4_root)
        if not args.check_only:
            host = next(line.removeprefix("host: ") for line in subprocess.check_output(["rustc", "-vV"], text=True).splitlines()
                        if line.startswith("host: "))
            if args.rust is None:
                subprocess.run(["cargo", "build", "--release", "--locked", "--target", host, "--example", "ekf2_standard"], cwd=ROOT, check=True)
                args.rust = ROOT / f"target/{host}/release/examples/ekf2_standard{'.exe' if os.name == 'nt' else ''}"
            subprocess.run([str(args.rust), args.case, str(directory)], cwd=ROOT, check=True)
        report = check(directory, args.case)
        (directory / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        print(json.dumps(report, sort_keys=True, allow_nan=False))
    except (ValueError, OSError, subprocess.CalledProcessError) as error:
        print(f"E6-H check failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
