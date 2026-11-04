#!/usr/bin/env python3
"""Quantify ULog dropouts, per-topic production rates, and implied SD throughput.

Reads a board ULog with pyulog and reports:
- dropout records ('O') with start time and duration
- rust_runtime_health counters (normal/event dropped, high-water bytes)
- per-topic record counts, achieved rates, and median periods
- implied SD write throughput when records were dropped

The board records per-source enable/rate parameters and an event reserve;
rates here reflect what was actually logged, not configured limits.
"""

import argparse
import json
import statistics
import sys
from pathlib import Path

from pyulog import ULog

HEALTH_TOPIC = "rust_runtime_health"
US_PER_S = 1_000_000


def per_topic_stats(data_list):
    rows = []
    for dataset in data_list:
        timestamps = dataset.data.get("timestamp")
        if timestamps is None or len(timestamps) == 0:
            continue
        count = len(timestamps)
        first = int(timestamps[0])
        last = int(timestamps[-1])
        span_us = max(last - first, 1)
        periods = [(b - a) for a, b in zip(timestamps, timestamps[1:])]
        median_period = statistics.median(periods) if periods else 0
        rows.append({
            "topic": dataset.name,
            "count": count,
            "rate_hz": count * US_PER_S / span_us,
            "median_period_us": median_period,
            "max_period_us": max(periods) if periods else 0,
            "span_s": span_us / US_PER_S,
        })
    return rows


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ulog", type=Path, help="board ULog file")
    parser.add_argument("--record-bytes", help="topic=bytes overrides, e.g. sensor_accel=49")
    parser.add_argument("--json", type=Path, help="also write a JSON summary")
    args = parser.parse_args(argv)

    ulog = ULog(str(args.ulog))
    rows = per_topic_stats(ulog.data_list)
    if not rows:
        print("no timestamped topics found", file=sys.stderr)
        return 1

    overrides = {}
    if args.record_bytes:
        for item in args.record_bytes.split(","):
            name, value = item.split("=")
            overrides[name.strip()] = int(value)

    health = next((d for d in ulog.data_list if d.name == HEALTH_TOPIC), None)
    normal_dropped = event_dropped = 0
    high_water = 0
    if health is not None:
        normal_dropped = int(health.data["logger_normal_dropped"][-1])
        event_dropped = int(health.data["logger_event_dropped"][-1])
        high_water = int(health.data["logger_high_water_bytes"][-1])

    # DATA overhead is 5 bytes (3-byte header + 2-byte topic id).
    produced_bytes = sum(
        row["count"] * (overrides.get(row["topic"], 0) + 5) for row in rows
    )
    total_records = sum(row["count"] for row in rows)
    span_s = max((row["span_s"] for row in rows), default=0.0)
    avg_record_bytes = produced_bytes / total_records if total_records else 0

    print(f"file: {args.ulog.name}")
    print(f"span: {span_s:.2f} s  records: {total_records}  "
          f"produced: {produced_bytes / 1000:.1f} kB")
    print(f"normal_dropped: {normal_dropped}  event_dropped: {event_dropped}  "
          f"high_water_bytes: {high_water}")
    for row in rows:
        rate = f"{row['rate_hz']:.1f} Hz" if row["count"] > 1 else "single"
        print(f"  {row['topic']}: n={row['count']} rate={rate} "
              f"median_period={row['median_period_us']:.0f} us "
              f"max_period={row['max_period_us']:.0f} us")
    for dropout in ulog.dropouts:
        print(f"  dropout: t={dropout.timestamp / US_PER_S:.3f} s "
              f"duration={dropout.duration / 1000:.0f} ms")

    if normal_dropped > 0 and span_s > 0:
        # Dropped records never hit the file, so the file-side rate is the
        # best available lower bound on sustained SD throughput.
        written_bytes = produced_bytes
        written_records = total_records
        produced_records = written_records + normal_dropped
        produced_rate = produced_records / span_s
        written_rate = written_records / span_s
        implied_throughput = written_bytes / span_s
        print(f"implied: produced~{produced_rate:.0f} rec/s "
              f"({produced_rate * avg_record_bytes / 1000:.0f} kB/s), "
              f"file-side {written_rate:.0f} rec/s -> "
              f"sustained SD >= {implied_throughput / 1000:.0f} kB/s")
        deficit = produced_rate * avg_record_bytes - implied_throughput
        print(f"deficit at capture time: ~{deficit / 1000:.0f} kB/s")

    if args.json:
        summary = {
            "file": args.ulog.name,
            "span_s": span_s,
            "records": total_records,
            "produced_bytes": produced_bytes,
            "normal_dropped": normal_dropped,
            "event_dropped": event_dropped,
            "high_water_bytes": high_water,
            "topics": rows,
            "dropouts": [
                {"start_us": d.timestamp, "duration_ms": d.duration / 1000}
                for d in ulog.dropouts
            ],
        }
        args.json.write_text(json.dumps(summary, indent=2))
        print(f"summary: {args.json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())