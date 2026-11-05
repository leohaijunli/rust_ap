#!/usr/bin/env python3
"""Compare fixed PX4 integer compensation/trim CRC with safe Rust on synthetic data."""

import argparse
import hashlib
import json
import random
import struct
import subprocess
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
REF = "d6f12ad1c4f70ad3230afd7d86e971421e02fef4"
SOURCE = "src/drivers/barometer/bmp388/bmp388.cpp"
TOLERANCE = "published f32 pressure/temperature bits and trim CRC exactly equal"


def function(source, name):
    start = source.index("static ", source.index(name) - 24)
    brace = source.index("{", source.index(name, start))
    depth = 1
    end = brace + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


def reference(directory, compiler):
    directory.mkdir(parents=True, exist_ok=True)
    upstream = directory / "px4-bmp388.cpp"
    if not upstream.exists():
        upstream.write_bytes(urllib.request.urlopen(f"https://raw.githubusercontent.com/PX4/PX4-Autopilot/{REF}/{SOURCE}", timeout=30).read())
    source = upstream.read_text(encoding="utf-8")
    if hashlib.sha256(upstream.read_bytes()).hexdigest() != "c23be8af051f0bb1c3ac29e151f65995ebf143d4d5b65fa5a5a3d423ca1602a3":
        raise ValueError("cached BMP388 source differs from the fixed PX4 commit")
    header = source[:source.index("/**")]
    wrappers = r'''
#include <cstdio>
#include <cstdint>
#include <cstring>
struct bmp3_uncomp_data { uint32_t pressure, temperature; };
struct bmp3_reg_calib_data {
uint16_t par_t1, par_t2; int8_t par_t3; int16_t par_p1, par_p2;
int8_t par_p3, par_p4; uint16_t par_p5, par_p6; int8_t par_p7, par_p8;
int16_t par_p9; int8_t par_p10, par_p11; int64_t t_lin;
};
struct bmp3_calib_data { bmp3_reg_calib_data reg_calib_data; };
'''
    body = "\n".join(function(source, name) for name in ("cal_crc(", "compensate_temperature(", "compensate_pressure("))
    main = r'''
int main() {
char hex[43]; unsigned p,t;
while (std::scanf("%42s %u %u",hex,&p,&t)==3) {
uint8_t b[21]; for(unsigned i=0;i<21;i++){unsigned v; std::sscanf(hex+2*i,"%2x",&v); b[i]=v;}
bmp3_calib_data cal{}; auto &c=cal.reg_calib_data;
auto u=[&](int i){return uint16_t(b[i]|uint16_t(b[i+1])<<8);};
c.par_t1=u(0);c.par_t2=u(2);c.par_t3=int8_t(b[4]);c.par_p1=int16_t(u(5));c.par_p2=int16_t(u(7));
c.par_p3=int8_t(b[9]);c.par_p4=int8_t(b[10]);c.par_p5=u(11);c.par_p6=u(13);c.par_p7=int8_t(b[15]);
c.par_p8=int8_t(b[16]);c.par_p9=int16_t(u(17));c.par_p10=int8_t(b[19]);c.par_p11=int8_t(b[20]);
bmp3_uncomp_data raw{p,t}; float temperature=float(compensate_temperature(&raw,&cal)/100.0f);
float pressure=float(compensate_pressure(&raw,&cal)/100.0f); uint8_t crc=255;
for(auto v:b) crc=uint8_t(cal_crc(crc,v)); crc^=255;
uint32_t pb,tb;std::memcpy(&pb,&pressure,4);std::memcpy(&tb,&temperature,4);
std::printf("%02x %08x %08x\n",unsigned(crc),pb,tb);
}
}
'''
    program = directory / "reference.cpp"
    program.write_text(header + wrappers + body + main, encoding="utf-8")
    binary = directory / "reference.exe"
    subprocess.run([compiler, "-std=c++11", "-O2", str(program), "-o", str(binary)], check=True)
    return binary, hashlib.sha256(upstream.read_bytes()).hexdigest()


def inputs():
    random_source = random.Random(0)
    trim = struct.pack("<HHbhhbbHHbbhbb", 27500, 32000, 1, 19000, 16000, 5, -2, 12000, 300, -3, 1, 200, -1, 1)
    rows = []
    for _ in range(256):
        pressure = random_source.randrange(1_000_000, 9_000_000)
        temperature_c = random_source.uniform(2, 65)
        temperature = int(27500 * 256 + temperature_c * 2**30 / 32000)
        rows.append(f"{trim.hex()} {pressure} {temperature}\n")
    return "".join(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rust", type=Path, required=True)
    parser.add_argument("--compiler", default="g++")
    parser.add_argument("--output", type=Path, default=ROOT / "target/bmp388-host")
    args = parser.parse_args()
    reference_binary, sha = reference(args.output, args.compiler)
    data = inputs()
    left = subprocess.check_output([str(reference_binary)], input=data, text=True).splitlines()
    right = subprocess.check_output([str(args.rust)], input=data, text=True).splitlines()
    if left != right or len(left) != 256:
        first = next((i for i, (a,b) in enumerate(zip(left,right)) if a != b), None)
        raise SystemExit(f"BMP388 compensation parity failed at row {first}")
    report = {"schema":1,"px4_ref":REF,"source":SOURCE,"source_sha256":sha,"synthetic":True,
              "seed":0,"cases":256,"tolerance":TOLERANCE,"passed":True,"hardware_validation":"not exercised by this tool",
              "input_sha256":hashlib.sha256(data.encode()).hexdigest()}
    (args.output / "report.json").write_text(json.dumps(report,indent=2)+"\n",encoding="utf-8")
    print(json.dumps(report))


if __name__ == "__main__": main()
