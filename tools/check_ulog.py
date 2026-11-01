#!/usr/bin/env python3
"""Validate the Rust-generated minimal ULog with pinned pyulog."""

import argparse
import tempfile
from pathlib import Path

from pyulog import ULog


TOPICS = {
    "imu_icm42688",
    "ahrs",
    "motor_command_4_slots",
    "arming_status",
    "esc_status",
    "gnss_fix",
    "gnss_auxiliary",
    "node_status",
    "can_stats",
    "runtime_health",
    "event",
    "parameter_change",
    "mavlink_stats",
}


def require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def check(path: Path, expect_dropout: bool) -> None:
    log = ULog(str(path))
    require(log.start_timestamp == 1_000_000, "wrong start timestamp")
    require(log.msg_info_dict["sys_name"] == "rust_ap", "wrong system name")
    require(log.initial_parameters["MAV_SYS_ID"] == 1, "wrong integer parameter")
    require(log.initial_parameters["LINK_LED_HZ"] == 2.0, "wrong real parameter")
    require({item.name for item in log.data_list} == TOPICS, "topic set mismatch")
    for item in log.data_list:
        require(len(item.data["timestamp"]) == 1, f"wrong sample count for {item.name}")
    datasets = [item for item in log.data_list if item.name == "arming_status"]
    require(len(datasets) == 1 and datasets[0].multi_id == 0, "arming subscription mismatch")
    data = datasets[0].data
    require(data["timestamp"].tolist() == [1_000_103], "arming timestamp mismatch")
    require(data["sequence"].tolist() == [1], "arming sequence mismatch")
    require(data["status"].tolist() == [0], "arming status mismatch")
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
    print("pyulog full and truncated-tail checks: PASS")


if __name__ == "__main__":
    main()
