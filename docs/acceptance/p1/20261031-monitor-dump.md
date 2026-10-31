# P1：monitor 的 Dump 识别与解码修复

日期：2026-10-31。**host修复与离线验证完成；工程师后续已成功采集实际Dump，定位到P1启动时钟断言。** SCUEN最小修复及普通P1复验见[诊断记录](20261031-startup-assertion.md)。

后续再次报错的核对与修复见[重枚举/诊断证据](20261031-monitor-rediscovery.md)：CLI增加默认10秒发现等待、只读`ports`入口和持久诊断日志；当前59项Python通过。本页52项结果保留为首轮修复证据。

## 问题与修复

用户重复执行 `bash tools/firmware.sh monitor 10`，得到
`RuntimeError: Expected one Rust CDC device, found []`。原脚本只匹配
`1209:0001 / V6X-RUST-001`；已枚举的诊断镜像使用 `V6X-RUST-DUMP`。

`tools/check_mavlink.py` 现在先识别 USB 身份，再选择传输路径：

| 入口/状态 | 行为 |
| --- | --- |
| `--monitor` + 普通 Rust | 保留 MAVLink STATUSTEXT 请求及遥测显示；无消息时非零退出 |
| `--monitor` + Dump | 仅接收文本；保存原始字节后复用 `decode_fault_dump.py` 的 CRC、源码映射与寄存器解码 |
| 默认完整检查 + Dump | 明确报没有 MAVLink，并提示 `--monitor 10`；不会报告协议检查通过 |
| bootloader / 无设备 / 多设备 | 明确错误、列出已发现身份并非零退出；多设备用 `--port` 选择 |
| 显式端口 | 支持 tty 与解析到同一设备的 by-id 链接；显式普通串口适配器继续走 MAVLink，拒绝 bootloader 与未枚举路径 |
| `find_port()` | 保持仅发现普通 Rust，兼容 `p1_capture.py` 的文本请求 |

Dump 默认保存到 `target/fault-dumps/<UTC 时间戳>.txt`，遵循 `CARGO_TARGET_DIR`；
`--raw` 可指定新文件。文件以独占创建模式打开，串口断开时保留已读字节。
没有报告/只有未结束报告时返回 1；参数错误返回 2；完整报告即使 `valid=NO`
也返回 0，表示采集完成。该返回码不表示 P1 启动或故障记录有效。

## 解码器修正

- `PLLnDIVR.N` 为倍频数减一编码，解码补 `+1`；fixture 使用真实寄存器值
  `PLL1=0000023b`、`PLL3=0003022f`，对应 N=60、48。
- 线格式的 `usbsel` 实际承载整个 `RCC_D2CCIP2R`，取 `[21:20]`。
- 系统时钟状态取 `RCC_CFGR.SWS[5:3]`，另显示 `SW[2:0]` 请求值。
- 跨报告边界采集优先选第一个完整报告；越界 BKP 字按无效记录处理。

固定 PAC 19.0.0 与本地固定 PX4 的交叉核对见
[硬件台账](../../fmuv6x-hardware.md#dump-寄存器解码核对2026-10-31)。
`entry` 是 Dump 入口、bootloader 跳转后的状态；`p0` 是 Dump HAL 初始化后的状态。
它们不直接代表旧 P1 故障发生时的时钟。

## 验证

```sh
TMPDIR=/tmp/opencode .venv/bin/python -m unittest discover -s tools -p 'test_*.py' -v
.venv/bin/python tools/check_mavlink.py --help
git diff --check
```

结果：**52 tests / OK**（新增 22 项），帮助入口与 diff 检查通过。
新增回归覆盖普通/Dump/bootloader/未知设备、by-id、多设备、非法时长、
普通协议检查身份透传、无遥测、分片采集、CRC/无效字、空/截断报告、断开保存和文件保护。
测试中的枚举与串口均为 mock；未运行真实 `monitor` 或固件刷写。

仅调用新 `select_port(allow_dump=True)` 的现场只读发现结果：

```text
Read-only discovery: mode=dump /dev/ttyACM0 (V6X Rust Dump, 1209:0001, serial=V6X-RUST-DUMP)
```

## 工程师采集操作

关闭 QGC/其他串口读取器后，直接执行原命令：

```sh
/bin/bash /home/dev/rust_ap/tools/firmware.sh monitor 10
```

保存输出中的 `Raw dump:` 文件与解码结果；已有文件可离线转 JSON：

```sh
.venv/bin/python tools/decode_fault_dump.py target/fault-dumps/<实际文件名>.txt --json
```

当前 TODO：
- [x] 监看分流、解码修正、host 回归与操作文档。
- [x] 工程师采集6456 B实际Dump，有效PANIC绑定`afaa140f499ff958 / src/platform.rs:91`；SCUEN断言已修复并通过离线回归。
- [ ] P1 普通固件启动、QGC 与恢复路径复验。
