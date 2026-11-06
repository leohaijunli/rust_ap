#!/usr/bin/env python3
"""Validate the Rust-generated minimal ULog with pinned pyulog."""

import argparse
import math
import tempfile
from pathlib import Path

from pyulog import ULog


TOPICS = {
    "sensor_accel",
    "sensor_gyro",
    "actuator_armed",
    "dronecan_esc_status",
    "dronecan_node_status",
    "rust_can_stats",
    "rust_runtime_health",
    "rust_parameter_change",
    "rust_current_sensor",
}

PX4_REVISION = "d6f12ad1c4f70ad3230afd7d86e971421e02fef4"
PX4_MESSAGES = {
    "sensor_accel": "SensorAccel.msg",
    "sensor_gyro": "SensorGyro.msg",
    "actuator_armed": "ActuatorArmed.msg",
}


def require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def px4_fields(filename: str) -> list[tuple[str, int, str]]:
    types = {"uint64": "uint64_t", "uint32": "uint32_t", "uint16": "uint16_t", "uint8": "uint8_t",
             "float32": "float", "float64": "double", "bool": "bool"}
    fields = []
    for line in (Path(__file__).parent / "px4_msgs" / filename).read_text(encoding="utf-8").splitlines():
        line = line.partition("#")[0].strip()
        if not line or "=" in line:
            continue
        field_type, name = line.split()
        base, _, suffix = field_type.partition("[")
        count = int(suffix.removesuffix("]")) if suffix else 0
        fields.append((types[base], count, name))
    return fields


def check_sensor(log: ULog, name: str, expected: dict[str, list[float]]) -> None:
    dataset = log.get_dataset(name)
    require(dataset.multi_id == 0, f"wrong sensor instance for {name}")
    data = dataset.data
    for field, values in {
        "timestamp": [1_001_000, 1_002_000],
        "timestamp_sample": [1_000_900, 1_001_900],
        "device_id": [0x260012, 0x260012],
        "temperature": [36.5, 36.5],
        "error_count": [7, 7],
        "samples": [1, 1],
        "clip_counter[0]": [0, 1],
        "clip_counter[1]": [0, 1],
        "clip_counter[2]": [0, 0],
    }.items():
        require(data[field].tolist() == values, f"wrong {name}.{field}")
    for field, values in expected.items():
        require(all(math.isclose(actual, value, rel_tol=1e-6, abs_tol=1e-4)
                    for actual, value in zip(data[field], values, strict=True)), f"wrong {name}.{field}")


def check(path: Path, expect_dropout: bool) -> None:
    log = ULog(str(path))
    require(log.start_timestamp == 1_000_000, "wrong start timestamp")
    require(log.msg_info_dict["sys_name"] == "rust_ap", "wrong system name")
    require(log.msg_info_dict["px4_msg_revision"] == PX4_REVISION, "wrong PX4 message revision")
    require(log.msg_info_dict["body_frame"] == "FRD", "wrong body frame")
    require(log.msg_info_dict["world_frame"] == "NED", "wrong world frame")
    require(log.initial_parameters["MAV_SYS_ID"] == 1, "wrong integer parameter")
    require(log.initial_parameters["LINK_LED_HZ"] == 2.0, "wrong real parameter")
    require({item.name for item in log.data_list} == TOPICS, "topic set mismatch")
    for name, filename in PX4_MESSAGES.items():
        require(log.message_formats[name].fields == px4_fields(filename),
                f"{name} does not match the pinned PX4 message")
    for item in log.data_list:
        count = 2 if item.name in {"sensor_accel", "sensor_gyro", "rust_current_sensor"} else 1
        require(len(item.data["timestamp"]) == count, f"wrong sample count for {item.name}")
    check_sensor(log, "sensor_accel", {"x": [1.25, 156.9], "y": [2.5, 156.9], "z": [-9.75, -10.0]})
    check_sensor(log, "sensor_gyro", {"x": [0.125, 34.9], "y": [0.25, 34.9], "z": [-0.5, 0.0]})
    current_sensor = log.get_dataset("rust_current_sensor")
    require(current_sensor.multi_id == 0, "current sensor subscription mismatch")
    require(log.message_formats["rust_current_sensor"].fields == [
        ("uint64_t", 0, "timestamp"), ("uint64_t", 0, "timestamp_sample"),
        ("float", 0, "voltage_v"), ("float", 0, "current_a"), ("float", 0, "power_w"),
    ], "current sensor schema mismatch")
    for field, values in {
        "timestamp": [1_010_100, 1_110_100],
        "timestamp_sample": [1_010_000, 1_110_000],
        "voltage_v": [12.0, 16.0],
        "current_a": [1.0, -0.5],
        "power_w": [12.0, -8.0],
    }.items():
        require(all(math.isclose(actual, value, rel_tol=1e-6, abs_tol=1e-5)
                    for actual, value in zip(current_sensor.data[field], values, strict=True)),
                f"wrong rust_current_sensor.{field}")
    dataset = log.get_dataset("actuator_armed")
    require(dataset.multi_id == 0, "arming subscription mismatch")
    data = dataset.data
    require(data["timestamp"].tolist() == [1_000_103], "arming timestamp mismatch")
    for field, value in {"armed": 0, "prearmed": 1, "ready_to_arm": 0, "lockdown": 1,
                         "kill": 0, "termination": 1, "in_esc_calibration_mode": 0}.items():
        require(data[field].tolist() == [value], f"wrong actuator_armed.{field}")
    require(len(log.dropouts) == (1 if expect_dropout else 0), "dropout count mismatch")
    if expect_dropout:
        require(log.dropouts[0].duration == 25, "dropout duration mismatch")
    require(not log.file_corruption, "pyulog marked file corrupt")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", type=Path)
    args = parser.parse_args()
    check(args.path, expect_dropout=True)

    data = args.path.read_bytes()
    with tempfile.TemporaryDirectory() as directory:
        truncated = Path(directory) / "truncated.ulg"
        truncated.write_bytes(data[:-1])
        check(truncated, expect_dropout=False)
    print("PX4 schemas, FRD/INA219 fields, pyulog full and truncated-tail checks: PASS")


if __name__ == "__main__":
    main()
