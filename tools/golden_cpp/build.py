#!/usr/bin/env python3
"""Build a profiled host-only reference from fixed PX4 EKF sources, not HEAD."""

import argparse
import hashlib
import io
import json
import os
import re
import subprocess
import sys
import tarfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
import gen_params

REF = gen_params.DEFAULT_REF
SOURCE_PATHS = (
    "LICENSE", "cmake/px4_add_common_flags.cmake", "src/modules/ekf2/EKF", "src/lib/matrix", "src/lib/mathlib",
    "src/lib/geo", "src/lib/lat_lon_alt", "src/lib/atmosphere", "src/lib/world_magnetic_model",
    "platforms/common/include", "msg/EstimatorAidSource1d.msg",
    "msg/EstimatorAidSource2d.msg", "msg/EstimatorAidSource3d.msg",
)
CORE_PARAMS = (
    "EKF2_PREDICT_US", "EKF2_IMU_CTRL", "EKF2_VEL_LIM", "EKF2_HGT_REF",
    "EKF2_DELAY_MAX", "EKF2_GYR_NOISE", "EKF2_ACC_NOISE", "EKF2_GYR_B_NOISE",
    "EKF2_ACC_B_NOISE", "EKF2_GBIAS_INIT", "EKF2_ABIAS_INIT", "EKF2_ANGERR_INIT",
    "EKF2_NOAID_NOISE", "EKF2_HDG_GATE", "EKF2_HEAD_NOISE", "EKF2_ABL_LIM",
    "EKF2_ABL_ACCLIM", "EKF2_ABL_GYRLIM", "EKF2_ABL_TAU", "EKF2_GYR_B_LIM",
    "EKF2_NOAID_TOUT",
)
BARO_MAG_PARAMS = (
    "EKF2_BARO_CTRL", "EKF2_BARO_DELAY", "EKF2_BARO_NOISE", "EKF2_BARO_GATE",
    "EKF2_GND_EFF_DZ", "EKF2_GND_MAX_HGT", "EKF2_MAG_DELAY", "EKF2_MAG_E_NOISE",
    "EKF2_MAG_B_NOISE", "EKF2_MAG_NOISE", "EKF2_MAG_DECL", "EKF2_MAG_GATE",
    "EKF2_DECL_TYPE", "EKF2_MAG_TYPE", "EKF2_MAG_ACCLIM", "EKF2_SYNT_MAG_Z",
    "EKF2_MAG_CHECK", "EKF2_MAG_CHK_STR", "EKF2_MAG_CHK_INC",
)
GNSS_RANGE_PARAMS = (
    "EKF2_GPS_CTRL", "EKF2_GPS_MODE", "EKF2_GPS_DELAY", "EKF2_GPS_V_NOISE", "EKF2_GPS_P_NOISE",
    "EKF2_GPS_V_GATE", "EKF2_GPS_P_GATE", "EKF2_GPS_CHECK", "EKF2_REQ_EPH", "EKF2_REQ_EPV",
    "EKF2_REQ_SACC", "EKF2_REQ_NSATS", "EKF2_REQ_PDOP", "EKF2_REQ_HDRIFT", "EKF2_REQ_VDRIFT",
    "EKF2_REQ_FIX", "EKF2_GSF_TAS", "EKF2_RNG_CTRL", "EKF2_RNG_DELAY", "EKF2_RNG_NOISE",
    "EKF2_RNG_GATE", "EKF2_RNG_PITCH", "EKF2_RNG_SFE", "EKF2_RNG_A_HMAX", "EKF2_RNG_A_VMAX",
    "EKF2_RNG_QLTY_T", "EKF2_RNG_K_GATE", "EKF2_RNG_FOG", "EKF2_TERR_NOISE", "EKF2_TERR_GRAD", "EKF2_MIN_RNG",
)
PROFILES = {"core_imu": CORE_PARAMS, "baro_mag": CORE_PARAMS + BARO_MAG_PARAMS,
            "gnss_range": CORE_PARAMS + BARO_MAG_PARAMS + GNSS_RANGE_PARAMS}
FIELD_TYPES = {
    "uint64": "uint64_t", "uint32": "uint32_t", "uint8": "uint8_t",
    "float32": "float", "float64": "double", "bool": "bool",
}


def aid_header(document, name):
    """Only payload fields are needed: no uORB metadata, transport or runtime."""
    lines = [
        f"// Generated from PX4 {REF} msg/{name}.msg (BSD-3-Clause).",
        "// Full source copyright and license are in the adjacent PX4 snapshot.",
        "#pragma once", "#include <cstdint>",
        f"struct {re.sub(r'(?<!^)(?=[A-Z])', '_', name).lower()}_s {{",
    ]
    for line in document.splitlines():
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        match = re.fullmatch(r"(\w+)(\[\d+\])?\s+(\w+)", line)
        if not match or match[1] not in FIELD_TYPES:
            raise ValueError(f"unsupported aid field: {line}")
        lines.append(f"    {FIELD_TYPES[match[1]]} {match[3]}{match[2] or ''}{{}};")
    return "\n".join([*lines, "};", ""])


def configure_header(definitions, profile="core_imu", fixture="nominal"):
    catalog = {definition.name: definition for definition in definitions}
    lines = ["#pragma once", f'inline constexpr char REFERENCE_PX4_REF[] = "{REF}";',
             "inline void configure_reference(Ekf &ekf) {",
             "    auto &params = *ekf.getParamHandle();"]
    applied = {}
    for name in PROFILES[profile]:
        definition = catalog[name]
        value = definition.default
        literal = gen_params.float_literal(value) + "f" if definition.kind == "Float" else str(value)
        lines.append(f"    params.{name.lower()} = {literal};")
        applied[name] = value
    for name, setter in [("EKF2_TAU_POS", "set_pos_correction_tc"),
                         ("EKF2_TAU_VEL", "set_vel_correction_tc")]:
        value = catalog[name].default
        lines.append(f"    ekf.output_predictor().{setter}({gen_params.float_literal(value)}f);")
        applied[name] = value
    if profile == "gnss_range":
        value = catalog["EKF2_REQ_GPS_H"].default
        lines.append(f"    ekf.set_min_required_gps_health_time({round(value * 1_000_000)}u);")
        lines.append("    ekf.set_rangefinder_limits(0.1f, 30.0f); // Synthetic fixture limits, not hardware specifications.")
        applied["EKF2_REQ_GPS_H"] = value
        if fixture == "lever_arm":
            for group, values in {"GPS": [0.2, 0.1, -0.05], "IMU": [0.03, -0.02, 0.01], "RNG": [0.1, -0.05, -0.1]}.items():
                for axis, value in enumerate(values):
                    name = f"EKF2_{group}_POS_{'XYZ'[axis]}"
                    lines.append(f"    params.{group.lower()}_pos_body({axis}) = {gen_params.float_literal(value)}f;")
                    applied[name] = value
        if fixture in ("gsf_yaw", "baro_reset"):
            overrides = {"EKF2_MAG_TYPE": 5} if fixture == "gsf_yaw" else {"EKF2_HGT_REF": 0, "EKF2_GPS_CTRL": 0, "EKF2_RNG_CTRL": 0}
            for name, value in overrides.items():
                lines.append(f"    params.{name.lower()} = {value};")
                applied[name] = value
        lines.append("    ekf.output_predictor().set_imu_offset(params.imu_pos_body);")
    lines.extend(["    ekf.updateParameters();", "}", ""])
    return "\n".join(lines), applied


def prepare(px4_root, output, profile="core_imu", fixture="nominal"):
    # Never switch features inside an existing reference cache. That would
    # silently replace the executable/manifest used by another stage.
    manifest_path = output / "source_manifest.json"
    if manifest_path.exists() and json.loads(manifest_path.read_text(encoding="utf-8"))["profile"] != profile:
        raise ValueError("reference profile differs; select a separate output directory")
    if manifest_path.exists() and json.loads(manifest_path.read_text(encoding="utf-8")).get("fixture", "nominal") != fixture:
        raise ValueError("reference fixture differs; select a separate output directory")
    archive = subprocess.check_output(
        ["git", "-C", str(px4_root), "archive", REF, *SOURCE_PATHS]
    )
    source = output / "px4"
    source.mkdir(parents=True, exist_ok=True)
    with tarfile.open(fileobj=io.BytesIO(archive)) as bundle:
        hashes = {member.name: hashlib.sha256(bundle.extractfile(member).read()).hexdigest()
                  for member in bundle.getmembers() if member.isfile()}
        bundle.extractall(source, filter="data")

    include = output / "host_include"
    topics = include / "uORB/topics"
    topics.mkdir(parents=True, exist_ok=True)
    (include / "px4_boardconfig.h").write_text("#pragma once\n", encoding="utf-8")
    for dimension in (1, 2, 3):
        name = f"EstimatorAidSource{dimension}d"
        document = (source / f"msg/{name}.msg").read_text(encoding="utf-8")
        (topics / f"estimator_aid_source{dimension}d.h").write_text(
            aid_header(document, name), encoding="utf-8"
        )
    definitions = gen_params.collect(gen_params.load_documents(px4_root, REF))
    for path in gen_params.yaml_paths(px4_root, REF):
        hashes[path] = hashlib.sha256(gen_params.git_show(px4_root, REF, path).encode("utf-8")).hexdigest()
    header, applied = configure_header(definitions, profile, fixture)
    (include / "reference_params.h").write_text(header, encoding="utf-8")
    manifest = {"schema": 1, "px4_ref": REF, "profile": profile,
                "fixture": fixture,
                "error_state_count": 24, "nominal_state_count": 25,
                "applied_parameters": applied, "source_sha256": hashes,
                "tools_sha256": {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                                 for path in [Path(__file__), Path(__file__).with_name("main.cpp"),
                                               Path(__file__).with_name("CMakeLists.txt"), ROOT / "tools/gen_params.py"]}}
    if profile == "gnss_range":
        manifest["synthetic_rangefinder_limits_m"] = [0.1, 30.0]
    (output / "source_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return source, include


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(os.environ.get("PX4_ROOT", "/home/dev/PX4-Autopilot")))
    parser.add_argument("--profile", choices=PROFILES, default="core_imu")
    parser.add_argument("--fixture", choices=("nominal", "lever_arm", "gsf_yaw", "baro_reset"), default="nominal")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--prepare-only", action="store_true")
    args = parser.parse_args()
    try:
        defaults = {"core_imu": "target/ekf2-host", "baro_mag": "target/ekf2-host-baro-mag",
                    "gnss_range": "target/ekf2-host-gnss-range"}
        if args.fixture != "nominal" and args.profile != "gnss_range":
            raise ValueError("nondefault fixture requires the five-sensor profile")
        default_output = defaults[args.profile] + (f"-{args.fixture.replace('_', '-')}" if args.fixture != "nominal" else "")
        output = (args.output or ROOT / default_output).resolve()
        source, include = prepare(args.root, output, args.profile, args.fixture)
        if not args.prepare_only:
            subprocess.run([
                "cmake", "-S", str(Path(__file__).parent), "-B", str(output / "build"),
                "-G", "Ninja", f"-DPX4_SOURCE_ROOT={source}", f"-DHOST_INCLUDE={include}",
                "-DCMAKE_BUILD_TYPE=Release", f"-DREFERENCE_PROFILE={args.profile}",
            ], check=True)
            subprocess.run(["cmake", "--build", str(output / "build"), "--parallel", "2"], check=True)
            manifest_path = output / "source_manifest.json"
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            manifest["compiler"] = subprocess.check_output(["c++", "--version"], text=True).splitlines()[0]
            manifest["executable_sha256"] = hashlib.sha256((output / "build/golden_cpp").read_bytes()).hexdigest()
            if args.profile in ("baro_mag", "gnss_range"):
                manifest["aids_executable_sha256"] = hashlib.sha256((output / "build/golden_cpp_aids").read_bytes()).hexdigest()
            if args.profile == "gnss_range":
                manifest["wmm_probe_sha256"] = hashlib.sha256((output / "build/wmm_probe").read_bytes()).hexdigest()
                manifest["output_executable_sha256"] = hashlib.sha256((output / "build/golden_cpp_output").read_bytes()).hexdigest()
                manifest["resets_executable_sha256"] = hashlib.sha256((output / "build/golden_cpp_resets").read_bytes()).hexdigest()
            manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
        print(f"golden_cpp: fixed PX4 {REF}; profile={args.profile}; output={output}")
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as error:
        print(f"golden_cpp: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
