# P4b-a DroneCAN 零输出软件验收

日期：2026-10-31。**CAN-A01～03 软件门通过；CAN-A04 实物验收待 H-02/03，P4b-a 尚未关闭。**

## 已通过

| 检查 | 结果 |
| --- | --- |
| v0 传输与 DSDL | 经典 CAN 扩展帧、tail/TID、CRC、多帧重组、NodeStatus/GetNodeInfo、GetSet/ExecuteOpcode、ESC Status、四槽零 RawCommand 与 DISARMED 已实现 |
| 有界与失败路径 | 12 重组槽、64 会话及消息保留容量；CRC/toggle/超时/重复/乱序/容量/错误地址安全拒绝 |
| FDCAN1 | PD0/PD1、HSE 16 MHz、1 Mbit/s、87.5% 采样点；FIFO 模式，FDCAN IRQ P2，I/O 与周期帧运行于 TIM6_DAC/P4 中断执行器 |
| TX 边界 | HAL 的优先级模式 abort 有无界忙循环，因此使用 FIFO 且关闭自动重传；每帧以 `TXBRP/TXBTO` 检查实际完成，旧帧不会无限占住周期安全流量 |
| 节点服务 | 静态本节点 ID 42；GetNodeInfo、统一参数 GetSet、实际 FRAM SAVE 结果已接入。RestartNode 因当前 HAL 无有界取消能力如实返回 `ok=false` |
| 软件构建 | fmt、普通/注入/Dump target Clippy、host Clippy、33 项 Rust 测试通过；自动 bootloader helper 9 项、端口回归 6 项及 Bash 语法通过 |
| release/打包 | Linux Rust 1.99.1；source=`a69dea23f29c032a`；raw/padded=`195344/195360 B`；static RAM=`38648 B`、空余 `419080 B`；分区/向量/ELF/bin/PX4 一致 |

普通 `.px4` SHA-256：`4e0da792467af07ac0939ff67f03e69239622614b68fd741067c17af681ca26b`。本轮没有烧录或操作实体 CAN。

## 待工程师实物验收

- H-02：四个 ESC 的固件、静态 node ID、`ESC_INDEX`、Status 频率及位置映射。当前映射明确为未配置，Status 只记 candidate，不会自动成为可信电机槽。
- H-03：CAN1 插座/收发器、针序、供电和 120 Ω 终端；核对 1 Mbit/s 寄存器及抓包。
- CAN-A04：确认四槽零 RawCommand、ArmingStatus=0、Status 映射与年龄；断线一次后失效并恢复。无 ACK/总线故障时 USB、采样替身及 CAN I/O 必须继续运行。

本节点 ID 42 是当前软件固定值，上板前须与实物网络核对不冲突。若 ESC 仍为默认 DNA/node 0，先由工程师确认能否用厂商工具配置静态 ID；只有设备确实必需时才增加 DNA。

## 自动烧录入口

`tools/firmware.sh upload` 在构建后会调用 `tools/enter_bootloader.py`：正常 Rust CDC 自动发送完整 `!bootloader` 行并等待原 PX4 bootloader 重枚举；已在 bootloader 时幂等继续，官方 PX4 应用仍交给 uploader 的 MAVLink 重启。VMware 需让 `1209:0001` 和 `3185:0035` 两个身份都自动连接到同一虚拟机；物理拔插只保留为独立恢复手段。

本轮在隔离 Python 环境/WSL 中通过新增 9 项和既有端口 6 项测试；上传 helper 仅完成 host 验证。
