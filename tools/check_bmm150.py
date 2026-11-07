#!/usr/bin/env python3
"""Compare BMM150 compensation with pinned PX4 and parse synthetic ULog fields."""

import argparse
import hashlib
import json
import math
import random
import struct
import subprocess
import urllib.request
from pathlib import Path

from pyulog import ULog
from check_ulog import px4_fields

ROOT = Path(__file__).resolve().parents[1]
REF = "d6f12ad1c4f70ad3230afd7d86e971421e02fef4"
SOURCE = "src/drivers/magnetometer/bosch/bmm150/BMM150.cpp"
SOURCE_SHA256 = "058a58d79276fd87877c127069caa9ae00d7bc2f4b5354c61cc772c1b09b2921"


def function(source, name):
    start = source.index(f"float BMM150::{name}(")
    brace = source.index("{", start)
    depth, end = 1, brace + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end].replace("BMM150::", "")


def reference(directory, compiler):
    upstream = directory / "px4-bmm150.cpp"
    if not upstream.exists():
        url = f"https://raw.githubusercontent.com/PX4/PX4-Autopilot/{REF}/{SOURCE}"
        upstream.write_bytes(urllib.request.urlopen(url, timeout=30).read())
    if hashlib.sha256(upstream.read_bytes()).hexdigest() != SOURCE_SHA256:
        raise ValueError("cached BMM150 source differs from the fixed PX4 commit")
    source = upstream.read_text(encoding="utf-8")
    header = source[:source.index('#include "BMM150.hpp"')]
    wrappers = r'''
#include <cstdio>
#include <cstdint>
#include <cstring>
static constexpr int16_t OVERFLOW_XYAXES = -4096;
static constexpr int16_t OVERFLOW_ZAXIS = -16384;
struct trim_registers {
int8_t dig_x1, dig_y1, dig_x2, dig_y2;
uint16_t dig_z1;
int16_t dig_z2, dig_z3, dig_z4;
uint8_t dig_xy1;
int8_t dig_xy2;
uint16_t dig_xyz1;
} _trim_data;
'''
    functions = "\n".join(function(source, name) for name in ("compensate_x", "compensate_y", "compensate_z"))
    main = r'''
int main() {
char hex[33]; int x,y,z; unsigned hall;
while (std::scanf("%32s %d %d %d %u",hex,&x,&y,&z,&hall)==5) {
uint8_t b[16]; for (unsigned i=0;i<16;i++){unsigned v;std::sscanf(hex+2*i,"%2x",&v);b[i]=uint8_t(v);}
auto u=[&](int i){return uint16_t(b[i]|uint16_t(b[i+1])<<8);};
auto &t = _trim_data;
t.dig_x1=int8_t(b[0]);t.dig_y1=int8_t(b[1]);t.dig_z4=int16_t(u(2));t.dig_x2=int8_t(b[4]);t.dig_y2=int8_t(b[5]);
t.dig_z2=int16_t(u(6));t.dig_z1=u(8);t.dig_xyz1=u(10)&0x7fff;t.dig_z3=int16_t(u(12));t.dig_xy2=int8_t(b[14]);t.dig_xy1=b[15];
float values[3]={compensate_x(int16_t(x),uint16_t(hall))*0.01f,compensate_y(int16_t(y),uint16_t(hall))*0.01f,compensate_z(int16_t(z),uint16_t(hall))*0.01f};
uint32_t bits[3];std::memcpy(bits,values,sizeof(bits));std::printf("%08x %08x %08x\n",bits[0],bits[1],bits[2]);
}
}
'''
    program, binary = directory / "reference.cpp", directory / "reference.exe"
    program.write_text(header + wrappers + functions + main, encoding="utf-8")
    if Path(compiler).stem.lower() == "cl":
        command = [compiler, "/nologo", "/O2", "/fp:strict", "/EHsc", f"/Fo:{directory / 'reference.obj'}", f"/Fe:{binary}", str(program)]
    else:
        command = [compiler, "-std=c++11", "-O2", "-ffp-contract=off", str(program), "-o", str(binary)]
    subprocess.run(command, check=True)
    return binary


def inputs():
    random_source = random.Random(0)
    rows = []
    for _ in range(256):
        trim = struct.pack("<bbhbbhHHhbB", random_source.randrange(-50, 50), random_source.randrange(-50, 50),
                           random_source.randrange(-1000, 1000), random_source.randrange(-30, 30), random_source.randrange(-30, 30),
                           random_source.randrange(1000, 8000), random_source.randrange(20000, 32000), random_source.randrange(5000, 14000),
                           random_source.randrange(-200, 200), random_source.randrange(-20, 20), random_source.randrange(20, 80))
        rows.append(f"{trim.hex()} {random_source.randrange(-1500,1500)} {random_source.randrange(-1500,1500)} {random_source.randrange(-5000,5000)} {random_source.randrange(5000,14000)}\n")
    return "".join(rows)


def check_ulog(path):
    log = ULog(str(path))
    assert not log.file_corruption, "pyulog marked file corrupt"
    assert log.msg_info_dict["px4_msg_revision"] == REF
    assert {item.name for item in log.data_list} == {"sensor_mag", "vehicle_magnetometer"}
    for topic, filename in [("sensor_mag", "SensorMag.msg"), ("vehicle_magnetometer", "VehicleMagnetometer.msg")]:
        assert log.message_formats[topic].fields == px4_fields(filename), topic
        dataset = log.get_dataset(topic)
        assert dataset.multi_id == 0
        data = dataset.data
        assert data["timestamp"].tolist() == [1_001_100, 1_051_100]
        assert data["timestamp_sample"].tolist() == [1_001_000, 1_051_000]
        assert data["device_id"].tolist() == [0x431021] * 2
        fields = ["x", "y", "z"] if topic == "sensor_mag" else [f"magnetometer_ga[{i}]" for i in range(3)]
        for field, expected in zip(fields, [0.22, -0.07, 0.43]):
            assert all(math.isclose(float(v), expected, rel_tol=1e-6) for v in data[field]), field
        if topic == "sensor_mag":
            assert all(math.isnan(float(v)) for v in data["temperature"])
            assert data["error_count"].tolist() == [7] * 2
        else:
            assert data["calibration_count"].tolist() == [0] * 2
    return {"topics": 2, "records": 4, "file_corruption": False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rust", type=Path, required=True)
    parser.add_argument("--compiler", default="g++")
    parser.add_argument("--output", type=Path, default=ROOT / "target/bmm150-host")
    args = parser.parse_args()
    directory = args.output.resolve()
    directory.mkdir(parents=True, exist_ok=True)
    binary = reference(directory, args.compiler)
    data = inputs()
    left = subprocess.check_output([str(binary)], input=data, text=True).splitlines()
    right = subprocess.check_output([str(args.rust.resolve())], input=data, text=True).splitlines()
    if left != right or len(left) != 256:
        first = next((i for i, pair in enumerate(zip(left, right)) if pair[0] != pair[1]), None)
        raise SystemExit(f"BMM150 compensation parity failed at row {first}: PX4={left[first] if first is not None else len(left)} Rust={right[first] if first is not None else len(right)}")
    path = directory / "magnetometer.ulg"
    subprocess.run([str(args.rust.resolve()), str(path)], check=True)
    parsed = check_ulog(path)
    report = {"px4_ref": REF, "source": SOURCE, "source_sha256": SOURCE_SHA256, "synthetic": True,
              "seed": 0, "cases": 256, "tolerance": "published f32 Gauss bits exactly equal", "ulog": parsed,
              "input_sha256": hashlib.sha256(data.encode()).hexdigest(), "passed": True, "hardware_validation": "not exercised"}
    (directory / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
