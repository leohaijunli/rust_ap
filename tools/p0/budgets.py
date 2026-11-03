#!/usr/bin/env python3
"""P0 planning arithmetic, not a hardware throughput or latency test."""

import argparse
import json
import math


def frame_bits(dlc):
    # Extended classic CAN: includes intermission; excludes error/retry frames.
    # Stuffing applies from SOF through the CRC sequence.
    return 67 + 8 * dlc + (53 + 8 * dlc) // 4


def transfer_bits(payload_bytes):
    remaining = payload_bytes + (2 if payload_bytes > 7 else 0)
    dlcs = []
    while True:
        part = min(remaining, 7)
        dlcs.append(part + 1)  # DroneCAN tail byte
        remaining -= part
        if not remaining:
            break
    return {"payload_bytes": payload_bytes, "dlcs": dlcs,
            "frames": len(dlcs), "bits": sum(map(frame_bits, dlcs))}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", choices=("m1", "exploration"), default="m1",
                        help="M1 data acquisition, or the earlier full-system exploration")
    parser.add_argument("--esc-count", type=int, default=4)
    parser.add_argument("--esc-slots", type=int, default=4,
                        help="highest esc_index + 1, including zero-filled gaps")
    parser.add_argument("--command-hz", type=int, default=400)
    parser.add_argument("--status-hz", type=int, default=50)
    parser.add_argument("--arming-hz", type=int, default=10)
    parser.add_argument("--fix-hz", type=int, default=5)
    args = parser.parse_args()
    if not (1 <= args.esc_count <= 8 and args.esc_count <= args.esc_slots <= 20
            and 1 <= args.command_hz <= 1000 and 1 <= args.status_hz <= 100
            and 1 <= args.fix_hz <= 20 and 1 <= args.arming_hz <= 20):
        parser.error("configuration outside P0 exploration range")

    kinds = {name: transfer_bits(size) for name, size in {
        "raw_command": math.ceil(14 * args.esc_slots / 8),
        "arming_status": 1, "esc_status": 14,
        "fix2_max_arrays": 221, "gnss_auxiliary": 16,
        "node_status": 7, "rtcm_full_chunk": 129,
    }.items()}
    rates = {"raw_command": args.command_hz,
             "arming_status": args.arming_hz,
             "esc_status": args.esc_count * args.status_hz,
             "fix2_max_arrays": args.fix_hz,
             "gnss_auxiliary": 1,
             "node_status": args.esc_count + 2 + 1 + 1}
    traffic = {name: kinds[name]["bits"] * rate for name, rate in rates.items()}
    traffic.update({"rtcm_frame_limit": (0 if args.profile == "m1" else 400) * frame_bits(8),
                    "services_frame_limit": 200 * frame_bits(8),
                    "other_gnss_broadcast_allowance": 20_000})
    subtotal = sum(traffic.values())
    # Payload limits are proposed explicit wire schemas, not Rust struct sizes.
    topics = [
        ("imu_icm42688", 56, 1000),
        ("state_estimate", 76, 250), ("rate_debug", 64, 250),
        (f"motor_command_{args.esc_slots}_slots", 24 + 4 * args.esc_slots, args.command_hz),
        ("arming_status", 24, args.arming_hz),
        ("esc_status", 40, args.esc_count * args.status_hz),
        ("mag", 32, 50), ("baro", 24, 50),
        ("gnss_fix", 256, args.fix_hz), ("gnss_auxiliary", 64, 1),
        ("rtcm_status", 64, 1), ("node_status", 20, 16),
        ("can_stats", 80, 10), ("runtime_health", 64, 10),
        ("events_allowance", 96, 10),
        ("mpc_debug_future", 128, 50), ("path_future", 128, 20),
    ]
    if args.profile == "m1":
        topics = [
            ("sensor_accel", 44, 200), ("sensor_gyro", 44, 200),
            ("vehicle_attitude", 49, 250), ("vehicle_angular_velocity", 40, 250),
            ("dronecan_raw_command", 24 + 4 * args.esc_slots, args.command_hz),
            ("actuator_armed", 15, args.arming_hz),
            ("esc_status", 352, args.status_hz),
            ("sensor_gps", 256, args.fix_hz), ("dronecan_gnss_auxiliary", 64, 1),
            ("dronecan_node_status", 20, 16), ("rust_can_stats", 80, 10),
            ("rust_runtime_health", 64, 10), ("events_allowance", 96, 10),
            ("rust_parameter_change", 48, 20), ("rust_mavlink_stats", 64, 2),
        ]
    logs = [{"topic": name, "payload_max_bytes": size, "hz_total": rate,
             "ulog_data_bytes_per_second": (size + 5) * rate}
            for name, size, rate in topics]
    log_rate = sum(row["ulog_data_bytes_per_second"] for row in logs)
    record_rate = sum(row["hz_total"] for row in logs)
    result = {
        "scope": "proposed budgets; record adopted test values and validate in HIL",
        "assumptions": vars(args) | {"can_bitrate": 1_000_000,
                                    "gnss_node_allowance": 2,
                                    "rtcm_input_bytes_per_second": 0 if args.profile == "m1" else 2048},
        "transfer_sizes": kinds, "transfer_rates_hz": rates,
        "can_bits_per_second": traffic,
        "can_subtotal_percent": subtotal / 10_000,
        "error_retry_reserve_percent": 10,
        "can_with_reserve_percent": (subtotal + 100_000) / 10_000,
        "within_60_percent_envelope": subtotal + 100_000 <= 600_000,
        "log_topics": logs, "log_bytes_per_second": log_rate,
        "log_with_25_percent_margin": math.ceil(log_rate * 1.25),
        "normal_ring_bytes": 128 * 1024, "event_reserve_bytes": 8 * 1024,
    }
    if args.profile == "m1":
        result.update({"log_records_per_second": record_rate,
                       "normal_ring_slots": 960,
                       "conservative_ring_seconds": 960 / record_rate})
    else:
        result["ring_seconds_at_256000_bytes_per_second"] = 128 * 1024 / 256000
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
