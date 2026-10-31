# P1：普通 Rust CDC / MAVLink 与 QGC 连接恢复

日期：2026-10-31。**用户确认 QGC 已连接；已有本机日志同时证明普通 Rust CDC 枚举及10秒MAVLink监看成功。启动/连接回归的基本功能已恢复。** 设备操作由工程师执行，agent核对并归档已有文件。

## 1. 用户确认与已有日志

用户在收到SCUEN最小修复及Dump→bootloader→普通P1交接步骤后回复：

> ok it is connected with qgc

| UTC时间 | 已记录结果 | 原始诊断JSON |
| --- | --- | --- |
| 16:13:22 | `ports`仍列出`V6X-RUST-DUMP`，`outcome=listed` | [Dump枚举](20261031-qgc-retest-dump-ports.json) |
| 16:14:38 | `ports`列出`PX4 BL FMU v6X.x`，VID:PID=`3185:0035`，`outcome=listed` | [原bootloader枚举](20261031-qgc-retest-bootloader-ports.json) |
| 16:15:35–16:15:45 | `V6X Rust CDC / V6X-RUST-001`，VID:PID=`1209:0001`，`selected_mode=mavlink`，`monitor=10`，`outcome=ok` | [普通MAVLink监看](20261031-qgc-retest-mavlink-monitor.json) |

三份JSON均记录同一`tools/check_mavlink.py` SHA-256：`ed1d16ce4c8ca8fedf3b6a72b35c914308c5f0f7887bdf370f77c2a360756d47`，已与当前脚本核对。归档按原始字节复制：

| 文件 | 大小 | SHA-256 |
| --- | ---: | --- |
| `20261031-qgc-retest-dump-ports.json` | 6508 B | `b148443667d94c66f89d1e7ef64d0c08d7d8f3a4d0e6e98eca1bcf3a9c2c1424` |
| `20261031-qgc-retest-bootloader-ports.json` | 6507 B | `1013a6f2b34e23b79815b35a63a99102a83ccbd2f84226ee4f7077ab091a8baa` |
| `20261031-qgc-retest-mavlink-monitor.json` | 6573 B | `9e0f5b272db5766b4a63e391354ff90e7de5edf7ed8e3d74fbc153dde33c784e` |

`monitor()`成功表示该窗口接收到MAVLink消息。该JSON未保存逐条消息或实际固件source；其`expected_version=0.2.0`是命令行预期值，monitor模式不执行版本/source断言。QGC连接本身以本次用户报告为证据。

## 2. 构建身份与后续核验

当轮本机目标普通P1为 **`source=4cba95aa54cd899a`**，features=[]；`.px4` SHA-256=`51fdd8a4189f519c014d3e78554f8eb422813656332a4f4b751626cebcb9739a`。本轮已核验17个源码输入、四个产物及[原构建报告](20261031-startup-after.build.json)一致。修复及离线证据见[启动断言记录](20261031-startup-assertion.md)。该轮板上精确source未读回绑定。

该轮交接由工程师关闭QGC后，在项目目录执行完整检查；后续新包身份与步骤以[备份域修复](20261031-backup-domain.md#5-工程师待验与关闭条件)为准：

```sh
.venv/bin/python tools/check_mavlink.py --source-id 4cba95aa54cd899a
```

保存终端输出和自动生成的诊断JSON。该入口检查实际`AUTOPILOT_VERSION`中的source、版本、心跳、只读参数、协议响应等；成功后可补齐精确身份及协议回归证据。

## 3. 当前验收状态

- [x] 原PX4 bootloader枚举 → 普通Rust CDC枚举 → MAVLink监看成功。
- [x] 用户确认QGC基本连接恢复。
- [ ] 最新普通包source读回与完整MAVLink检查，当前目标见[备份域修复](20261031-backup-domain.md)。
- [ ] P1物理时基、调度、CPU/栈、故障及恢复路径，按[上板操作单](bring-up.md)继续。
- [x] 后续备份域软件修复及三版离线回放，见[证据](20261031-backup-domain.md)。
- [ ] 板上备份域保持、P1-01/H-05恢复资料与P1-04完整验收。
