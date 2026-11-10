#!/usr/bin/env python3
"""Generate QGC JSON from the compiled single runtime registry, never a second table."""

import argparse
import csv
import io
import json
import subprocess
import zlib
from pathlib import Path

from gen_params import DEFAULT_REF

ROOT = Path(__file__).resolve().parents[1]
DIRECTORY = ROOT / "src/core/metadata"
HELP_FILE = ROOT / "tools/ekf2_parameter_help.json"

CONDITIONS = {
    "conditional:baro": "已接入气压配置；融合需有效气压输入及对应开关，部分项还影响初始化/状态。",
    "conditional:mag": "已接入磁场配置；融合需有效磁场输入和模式，部分噪声项也参与协方差预测。",
    "conditional:gps": "已接入 GNSS 配置；融合需有效输入并满足质量/时间门，不代表当前正在融合。",
    "conditional:range": "已接入测距/地形配置；路径受观测/高度参考/状态控制，部分项也参与预测。",
    "conditional:ground": "已接入地效配置；仅在相应地效状态、地形和创新条件满足时使用。",
    "conditional:height": "已接入高度参考配置；对应观测源需真实可用，不表示全部 PX4 辅助模块已实现。",
}

RESTRICTIONS = {
    "EKF2_EN": "0 选择 Mahony；1 允许按 EST_ALGO 选择 EKF2，仍需 EST_EN=1；不解锁。",
    "EKF2_LOG_VERBOSE": "当前未读取此开关；估计日志由 LOG_EN_EST/LOG_EST_HZ 控制。",
    "EKF2_GPS_CTRL": "当前仅支持 0..7；双天线航向位 8 未实现并被配置校验拒绝。",
    "EKF2_GPS_MODE": "当前仅接受 0；其他 PX4 重置模式未接入。",
    "EKF2_MAG_TYPE": "当前配置仅接受 0/1/5/6；没有真实磁场输入时不能据此认为正在融合。",
    "EKF2_HGT_REF": "当前允许配置值 0..3；外部视觉模块未接入，不能把其参考源视为可用。",
    "EKF2_DELAY_MAX": "应覆盖已启用辅助源的最大延迟；实际还校验预测周期与延迟缓冲容量。",
    "EKF2_PREDICT_US": "单位 us，不是姿态发布频率；组合须通过延迟缓冲容量校验。",
}

LOG_SOURCES = {
    "IMU": "IMU 加速度/角速度",
    "INA": "INA219 独立支路电压/电流/功率",
    "CAN": "ESC 状态/GNSS/CAN 统计",
    "HEALTH": "运行健康/时序",
    "EVENT": "事件/参数变化/节点状态",
    "EST": "估计输出/状态",
    "BARO": "气压/温度",
    "MAG": "地磁 XYZ（Gauss）",
}

LINK_STREAMS = {
    "MAV_RATE_HB": ("心跳/系统状态/UTC 遥测速率", "正常联机不建议设 0；0 关闭心跳组，可能影响 QGC 连接及升级预检。"),
    "MAV_RATE_STATUS": ("周期状态文本速率", "0 只关闭周期文本，状态变化事件仍可发送。"),
    "MAV_RATE_SENSORS": ("IMU/气压/地磁遥测速率", "0 停发该组，不改变实际采样或 SD 记录。"),
    "MAV_RATE_EST": ("估计器遥测速率", "含姿态/健康及有效位置，0 停发但不停止估计。"),
    "MAV_RATE_GPS": ("GNSS 遥测速率", "无数据时 NO_GPS 最多 1 Hz，0 停发但不停止 GNSS 接收。"),
    "MAV_RATE_POWER": ("支路电源遥测速率", "调高不会提高 INA219 采样率，可能重复最新值；0 停发。"),
    "MAV_RATE_HEALTH": ("EKF/传感器健康汇总速率", "DEBUG_FLOAT_ARRAY/AP_HEALTH；实测频率窗口固定 1 s，调高只重复缓存；0 停发，不改变采样、估计或 INA219 电流消息。"),
}


def runtime_help(name):
    if name == "MAV_SYS_ID":
        return "MAVLink 系统 ID", "当前固定为 1 且只读，不是可修改的系统地址。"
    if name == "LINK_LED_HZ":
        return "状态/以太网指示灯闪烁频率", "单位 Hz；只影响指示灯，不是安全灯、采样或控制频率。"
    if name == "EST_EN":
        return "数据估计启用", "0 停止数据估计；1 启用；不解锁，也不授予导航资格。"
    if name == "EST_ALGO":
        return "请求的估计算法", "0=Mahony；1=EKF2，还受 EKF2_EN/构建类型影响。实际选择以 !ekf2 为准。"
    if name.startswith("CAL_MAG"):
        slot, separator, field = name[7:].partition("_")
        if separator and slot in "0123" and len(slot) == 1:
            labels = {
                "ID": ("磁传感器校准绑定 ID", "0 未绑定；填写 !mag 的真实 device_id。校准槽编号不是板载/外接编号；重复非零 ID 拒绝。"),
                "PRIO": ("磁传感器配置优先级", "-1 默认（内部 50、外部 75）；0 禁用，1..100 越高越优先，仍受健康选择约束。失败后的有效优先级降为 1，改变配置优先级或重启可重新参与选择。"),
                "ROT": ("外接磁传感器安装方向", "支持 PX4 rotation 0..40、100 自定义，-1 未配置按 0；41..99 拒绝。内部磁源固定跟随本项目板/机体 rotation=0，不独立旋转；非零 ROLL/PITCH/YAW 使用自定义方向。"),
            }
            if field in labels:
                short, detail = labels[field]
            elif field in ("XOFF", "YOFF", "ZOFF"):
                short, detail = "磁场硬铁偏移 " + field[0], "单位 Gauss，在软铁矩阵和安装旋转之前减去；不是 EKF 学到的磁偏置。"
            elif field in ("XSCALE", "YSCALE", "ZSCALE", "XODIAG", "YODIAG", "ZODIAG"):
                short, detail = "磁场软铁矩阵 " + field, "无量纲；SCALE 为对角项，X/Y/ZODIAG 为 XY/XZ/YZ 对称项。组合必须正定，非法组合拒绝。"
            elif field in ("ROLL", "PITCH", "YAW"):
                short, detail = "外接磁源自定义角 " + field, "单位 deg，ZYX 旋转；内部磁源忽略此角，以保持与 IMU 的机体方向一致。"
            else:
                raise ValueError(f"Missing compass description: {name}")
            return f"校准槽 {slot}：{short}", detail + "此处只应用参数，不执行校准拟合；绑定不等于实物/飞行资格通过。"
    if name in LINK_STREAMS:
        short, detail = LINK_STREAMS[name]
        return short, detail + "作用于两链路默认值；511 的链路覆盖优先。"
    if name.startswith("LOG_EN_"):
        source = LOG_SOURCES[name.removeprefix("LOG_EN_")]
        detail = "0 关闭、1 允许该组 SD 记录；不是 logger 启停，也不停止传感器/算法。"
        if name == "LOG_EN_EVENT":
            detail += "关闭会丢失事件/参数变化/节点状态记录；通常保持 1。"
        return f"{source}日志开关", detail
    if name.startswith("LOG_") and name.endswith("_HZ"):
        source = LOG_SOURCES[name[4:-3]]
        return f"{source}日志速率上限", "单位 Hz；须先启动 logger；不提高数据源采样率，高频会增加 SD/队列压力。"
    raise ValueError(f"Missing audited runtime description: {name}")


def value_guide(entry, reference):
    unit = entry.get("units", "")
    default = f"{entry['default']:.9g}" + (f" {unit}" if unit else "")
    parts = [f"默认 {default}。"]
    kind = reference.get("type")
    options = reference.get("options", {})
    if kind == "boolean":
        parts.append("0=关闭；1=开启。")
    elif kind in ("enum", "bitmask"):
        values = []
        for key, label in sorted(options.items(), key=lambda item: int(item[0])):
            value = 1 << int(key) if kind == "bitmask" else int(key)
            values.append(f"{value}={label}")
        if kind == "bitmask":
            parts.append("按位相加，0=全部关闭：" + "；".join(values) + "。")
        else:
            parts.append("目录取值：" + "；".join(values) + "。")
    else:
        low, high = entry["min"], entry["max"]
        lower = f"≥{low:.9g}" if low > -2**31 else ""
        upper = f"≤{high:.9g}" if high < 2**31 - 1 else ""
        bounds = "、".join(value for value in (lower, upper) if value)
        parts.append(f"目录值域 {bounds}。" if bounds else "目录无有效数值上下限；仍须通过运行时校验。")
        if entry["type"] == "Int32":
            parts.append("必须设置整数。")
    return "".join(parts)


def description(entry, usage, access, help_entries):
    name = entry["name"]
    if entry["group"] == "EKF2":
        if name not in help_entries:
            raise ValueError(f"Missing fixed-source parameter help: {name}")
        reference = help_entries[name]
        if usage == "unmapped":
            label = "未接入"
            detail = "用户值可读写/保存，但当前运行时不读取；算法可能使用固定默认值。"
        elif usage == "mapped":
            label = "已接入配置"
            detail = "当前运行时读取此配置；EKF2 启用且重配置成功后由相关路径使用。"
        elif usage in CONDITIONS:
            label, detail = "条件生效", CONDITIONS[usage]
        else:
            raise ValueError(f"Unknown EKF integration status for {name}: {usage}")
        short = f"{reference['shortDesc']} [{label}]"
        detail += value_guide(entry, reference)
        detail += RESTRICTIONS.get(name, "")
        detail += "修改可能重新初始化估计器；回显/保存不等于算法已应用或观测正在融合。"
    else:
        if usage != "runtime":
            raise ValueError(f"Unexpected runtime integration status for {name}: {usage}")
        short, detail = runtime_help(name)
        short += " [只读]" if access == "read-only" else " [已接入]"
        detail += value_guide(entry, {})
    if access == "read-only":
        detail += "本固件只读。"
    elif access == "next-cycle":
        if entry["group"] != "EKF2":
            detail += "下一处理周期/重配置边界生效。"
        detail += "持久保存仅在未解锁且输出关闭时执行。"
    else:
        raise ValueError(f"Unknown parameter access for {name}: {access}")
    return short, detail


def generated(executable, help_file=HELP_FILE):
    reference = json.loads(Path(help_file).read_text(encoding="utf-8"))
    if reference.get("ref") != DEFAULT_REF:
        raise ValueError("EKF help does not match the pinned PX4 source")
    help_entries = reference["parameters"]
    rows = csv.reader(io.StringIO(subprocess.check_output([str(executable)], text=True, encoding="utf-8")))
    entries = []
    for row in rows:
        if len(row) != 9:
            raise ValueError("Rebuild parameter_catalog: metadata requires 9 exported columns")
        name, group, kind, default, minimum, maximum, units, usage, access = row
        entry = {"name": name, "type": kind, "group": group,
                 "min": float(minimum), "max": float(maximum), "default": float(default)}
        if units:
            entry["units"] = units
        # Integer values stay JSON integers; scientific Float limits retain their
        # full f32 range rather than being saturated to i32 by formatting.
        for key in ("min", "max", "default"):
            if kind == "Int32" or abs(entry[key]) < 2**31 and entry[key].is_integer():
                entry[key] = int(entry[key])
        entry["shortDesc"], entry["longDesc"] = description(entry, usage, access, help_entries)
        entries.append(entry)
    actual = {entry["name"] for entry in entries if entry["group"] == "EKF2"}
    if actual != set(help_entries):
        raise ValueError("EKF help/catalog coverage differs")
    parameters = '{\n  "version": 1,\n  "parameters": [\n' + ",\n".join(
        "    " + json.dumps(entry, ensure_ascii=False) for entry in entries) + "\n  ]\n}\n"
    crc = zlib.crc32(parameters.encode())
    general = json.dumps({"version": 1, "metadataTypes": [{"type": 1, "uri": "mftp:///component_metadata/parameters.json", "fileCrc": crc}]}, indent=2) + "\n"
    return {"parameters.json": parameters, "general.json": general}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rust", type=Path, required=True)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    for name, text in generated(args.rust).items():
        path = DIRECTORY / name
        if args.check:
            if not path.exists() or path.read_text(encoding="utf-8") != text:
                raise SystemExit(f"QGC metadata differs: {name}")
        else:
            DIRECTORY.mkdir(parents=True, exist_ok=True)
            with path.open("w", encoding="utf-8", newline="\n") as output:
                output.write(text)


if __name__ == "__main__":
    main()
