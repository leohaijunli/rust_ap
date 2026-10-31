#!/usr/bin/env python3
"""Generate host-only DroneCAN v0 reference vectors from pinned upstreams.

No CAN interface is opened. Reference roundtrips validate fixture generation,
not this project's future Rust implementation or transfer-session handling.
"""

import argparse
import copy
import hashlib
import json
from pathlib import Path
import subprocess
import sys


PINS = {
    "pydronecan": "092d09cb9e6b115e262efe4cb42f7b65e6054f26",
    "DSDL": "b4653c7abc3c47cb31b16efa24ea755232774756",
}


def check_source(path, expected):
    actual = subprocess.check_output(
        ["git", "-C", str(path), "rev-parse", "HEAD"], text=True
    ).strip()
    if actual != expected:
        raise SystemExit(f"{path}: expected {expected}, found {actual}")
    subprocess.run(["git", "-C", str(path), "diff", "--exit-code", "HEAD"], check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pydronecan", type=Path, required=True)
    parser.add_argument("--dsdl", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    for key, path in (("pydronecan", args.pydronecan), ("DSDL", args.dsdl)):
        check_source(path, PINS[key])
    sys.path.insert(0, str(args.pydronecan.resolve()))
    import dronecan
    from dronecan.transport import Transfer, TransferError

    # Discard ambient/vendor definitions loaded by the package at import.
    dronecan.DATATYPES.clear()
    dronecan.TYPENAMES.clear()
    dronecan.load_dsdl(str(args.dsdl.resolve() / "uavcan"), exclude_dist=True)
    protocol = dronecan.uavcan.protocol
    esc = dronecan.uavcan.equipment.esc
    safety = dronecan.uavcan.equipment.safety
    gnss = dronecan.uavcan.equipment.gnss
    vectors = []
    transfer_frames = {}

    def add(label, message, *, service=None, tid=0, priority=16):
        tx = Transfer(
            payload=message,
            source_node_id=42,
            dest_node_id=73 if service else None,
            service_not_message=service is not None,
            request_not_response=service == "request",
            transfer_id=tid,
            transfer_priority=priority,
            canfd=False,
        )
        frames = tx.to_frames()
        rx = Transfer()
        rx.from_frames(frames)
        repacked = Transfer(payload=rx.payload).payload
        if bytes(repacked) != bytes(tx.payload):
            raise AssertionError(f"reference roundtrip mismatch: {label}")
        vectors.append({
            "label": label,
            "type": dronecan.get_dronecan_data_type(message).full_name,
            "service": service,
            "transfer_id": tid,
            "payload_hex": bytes(tx.payload).hex(),
            "payload_bytes": len(tx.payload),
            "frame_count": len(frames),
            "frames": [{"extended_id": f"{f.message_id:08x}",
                        "data_hex": bytes(f.bytes).hex()} for f in frames],
        })
        transfer_frames[label] = frames

    for count in (0, 1, 4, 5, 8, 20):
        add(f"raw_zero_{count}", esc.RawCommand(cmd=[0] * count), priority=8)
    add("raw_signed_edges", esc.RawCommand(cmd=[-8192, -1, 0, 8191]), tid=31, priority=8)
    for tid in (30, 31, 0, 1):
        add(f"raw_tid_{tid}", esc.RawCommand(cmd=[1, 2, 3, 4]), tid=tid, priority=8)
    add("arming_disarmed", safety.ArmingStatus(status=0), priority=8)
    add("arming_fully_armed", safety.ArmingStatus(status=255), priority=8)
    # Valid wire value, but not permission to arm: AM32 accepts only 255.
    add("arming_reserved_status", safety.ArmingStatus(status=1), priority=8)
    add("esc_status", esc.Status(error_count=123, voltage=24.5, current=1.25,
                                temperature=300.0, rpm=12000, power_rating_pct=50, esc_index=3))
    add("esc_unknown_negative_rpm", esc.Status(voltage=float("nan"), current=float("nan"),
                                              temperature=float("nan"), rpm=-131072, esc_index=31))
    add("node_status", protocol.NodeStatus(uptime_sec=123456, health=0, mode=0,
                                          sub_mode=0, vendor_specific_status_code=0x1234))
    add("get_node_info_request", protocol.GetNodeInfo.Request(), service="request")
    info = protocol.GetNodeInfo.Response(name="org.rust-ap.fmuv6x")
    info.status.uptime_sec = 123456
    info.software_version.major = 1
    info.hardware_version.unique_id = list(range(16))
    add("get_node_info_response", info, service="response")
    add("get_param_index", protocol.param.GetSet.Request(index=7), service="request")
    param = protocol.param.GetSet.Request(name="CAN_NODE_ID")
    param.value.integer_value = 42
    add("set_param_integer", param, service="request")
    param = protocol.param.GetSet.Request(name="CAN_NODE_ID")
    param.value.real_value = 42.0
    add("set_param_real", param, service="request")
    reply = protocol.param.GetSet.Response(name="CAN_NODE_ID")
    reply.value.real_value = 42.0
    reply.default_value.real_value = 42.0
    reply.min_value.real_value = 1.0
    reply.max_value.real_value = 127.0
    add("get_param_response", reply, service="response")
    add("save_request", protocol.param.ExecuteOpcode.Request(opcode=0), service="request")
    add("save_response", protocol.param.ExecuteOpcode.Response(ok=True), service="response")
    add("restart_request", protocol.RestartNode.Request(magic_number=0xACCE551B1E), service="request")
    add("restart_response", protocol.RestartNode.Response(ok=True), service="response")

    fix = gnss.Fix2(longitude_deg_1e8=-12212345678, latitude_deg_1e8=4712345678,
                    height_ellipsoid_mm=123456, height_msl_mm=100000,
                    ned_velocity=[1.0, -2.0, 0.5], sats_used=20, status=3,
                    gnss_time_standard=2, num_leap_seconds=27, pdop=1.25)
    fix.gnss_timestamp.usec = 1700000000123456
    for count in (0, 1, 6, 21, 36):
        fix.covariance = [0.25] * count
        add(f"fix2_covariance_{count}", fix)
    fix.mode = 2
    fix.sub_mode = 0
    add("fix2_rtk_float", fix)
    fix.sub_mode = 1
    add("fix2_rtk_fixed", fix)
    ecef = gnss.ECEFPositionVelocity(velocity_xyz=[1.0, 2.0, 3.0],
                                    position_xyz_mm=[1000, -2000, 3000],
                                    covariance=[0.5] * 36)
    fix.ecef_position_velocity = [ecef]
    add("fix2_max_arrays", fix)
    add("fix2_no_fix", gnss.Fix2(status=0, gnss_time_standard=0, pdop=float("nan")))
    add("gnss_auxiliary", gnss.Auxiliary(gdop=1.0, pdop=1.25, hdop=0.75, vdop=1.5,
                                       tdop=float("nan"), ndop=float("nan"), edop=float("nan"),
                                       sats_visible=28, sats_used=20))
    for count in (0, 1, 128):
        add(f"rtcm_{count}", gnss.RTCMStream(protocol_id=3, data=list(range(count))), priority=24)

    negatives = []
    for label, base, operation in (
        ("bad_crc", "raw_zero_8", "crc"),
        ("bad_initial_toggle", "raw_zero_4", "toggle"),
        ("missing_last_frame", "raw_zero_8", "drop_last"),
    ):
        frames = copy.deepcopy(transfer_frames[base])
        if operation == "crc":
            frames[0].bytes[0] ^= 1
        elif operation == "toggle":
            frames[0].bytes[-1] ^= 0x20
        else:
            frames.pop()
        try:
            Transfer().from_frames(frames)
        except TransferError:
            pass
        else:
            raise AssertionError(f"reference accepted {label}")
        negatives.append({"label": label, "expected": "reject",
                          "frames": [{"extended_id": f"{f.message_id:08x}",
                                      "data_hex": bytes(f.bytes).hex()} for f in frames]})

    types = {}
    for name in sorted({v["type"] for v in vectors}):
        dtype = dronecan.TYPENAMES[name]
        limits = ({mode: getattr(dtype, f"get_max_bitlen_{mode}")()
                   for mode in ("request", "response")}
                  if dtype.kind == dtype.KIND_SERVICE else {"message": dtype.get_max_bitlen()})
        types[name] = {"id": dtype.default_dtid,
                       "signature": f"{dtype.get_data_type_signature():016x}",
                       "max_bits_without_tao": limits}
    specs = {str(p.relative_to(args.dsdl)): hashlib.sha256(p.read_bytes()).hexdigest()
             for p in sorted((args.dsdl / "uavcan").rglob("*.uavcan"))}
    result = {
        "pins": PINS, "pydronecan_version": dronecan.__version__,
        "scope": "reference fixture generation only; Rust/session/HIL acceptance pending",
        "classic_can": True, "tail_array_optimization": True,
        "types": types, "vectors": vectors, "negative_vectors": negatives,
        "dsdl_sha256": specs,
    }
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(f"Generated {len(vectors)} reference vectors and {len(negatives)} rejections; "
          "Rust interoperability remains pending.")


if __name__ == "__main__":
    main()
