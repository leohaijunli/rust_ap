# 全参数使用状态与设置说明审核

日期：2026-11-07。工程师要求修正 EKF2 `unmapped` 提示，并审核其余参数、补简要设置说明。本轮没有设参、保存、烧录或改动 EKF 数学/调度。

## 结果

- 165 项全覆盖：1 项只读、22 项已接入、24 项已接入配置、54 项条件生效、64 项未接入。78 个 EKF2 配置键的是否读取直接来自 `Params::maps()`，观测/状态依赖仅作说明，不充当实时资格。
- PARAM_VALUE 的值/类型、165 项顺序、注册表默认/min/max/group、FRAM 布局和设值/重配置行为不变。旧 JSON 与新 JSON 的上述字段逐项比较一致。
- 说明包含作用、单位、默认/目录值域、整数/布尔/枚举/位含义，以及拒绝值、回显/保存/实际生效的区别；特殊支持范围单独说明，未悄悄收窄目录。
- 23 项非 EKF 参数按状态灯、日志组、启用/选择与共享遥测调度逐项核对。日志不是启停命令，遥测速率不是采样率；关闭心跳与关闭事件记录的影响已说明。
- 固定 PX4 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` 的 20 个 YAML 文件仅提取短说明和选项为 `tools/ekf2_parameter_help.json`；不重复数值表，CI 用同一固定源码检查 stale。
- 参数 JSON 与 general JSON 同源重生成，CRC 链随新文案更新。中文 UTF-8/LF 精确字节保持；新固件和 QGC 界面仍需后续授权核对。

[设置指南](../../parameter-settings.md)；[能力矩阵](../../qgc-compatibility.md)。

## 必要检查

| 检查 | 结果 |
| --- | --- |
| host `parameter_catalog` 示例测试 | 2 passed，读取状态与未接入优先级 |
| `test_gen_parameter_metadata.py` | 4 passed，文案/数值/CRC、缺失/非法标记、stale 不覆盖 |
| `test_gen_params.py` | 10 passed，固定帮助不复制数值、帮助 stale 检查及原有解析 |
| `component_metadata::` 核心用例 | 4 passed；最后生成后再次复核 CRC/数值/类型/分组通过 |
| 元数据生成器 `--check`、fmt、host 示例 Clippy | 通过 |
| 普通 H753 Clippy、release/打包/布局 | 通过；source=`a17a2d35bab6f05a`，raw/padded=`1124928/1124928 B`，static RAM=`254224 B`，最大入口栈帧 `11496 B`、0 超限 |

普通 features=`[]`，board ID 53、入口 `0x08020000`、分区 `1966080 B` 保持。文案相对上一软件包增加 54592 B raw，RAM 与最大入口栈帧不变；不作为新板载资源/时序证明。

包：`target/thumbv7em-none-eabihf/release/v6x-rust-led.px4`；SHA-256=`14abc5bf3a5f4740af80c0d5f5fa1ae965f447dc184db478a69cb020aac4d0f7`。未回装，实际 QGC/缓存重新获取仍待工程师。

一次 stdout 展示受 Windows cp1252 编码限制而失败；UTF-8 文件生成及数值比较已成功，改用 `python -X utf8` 展示后核对通过，不改历史失败结果。

本地来源 hash/结果/产物：`target/qgc-parameters/20261107-metadata/`。局部 code-simplifier 审视已完成；不重复无关 EKF 回放或硬件长测。尚未放行 QGC GUI、M1 或完整 E7。
