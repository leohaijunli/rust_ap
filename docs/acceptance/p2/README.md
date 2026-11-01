# P2 MAVLink 双链路软件验收

日期：2026-11-01。**MAV-01～03 软件开发通过；实物双链路/重连/版本显示待 H-03 与工程师接线，P2 尚未关闭。**

## 已通过

| 检查 | 结果 |
| --- | --- |
| 共享核心 | `mavlink_core.rs`：消息构建（heartbeat/status/system/autopilot_version/parameter/ack/warning）、`FrameParser`、入站 `handle` 均传输无关，USB 与 TELEM1 共用 |
| USB MAVLink | 原 `mavlink_usb.rs` 改为只含 CDC 传输、会话重启与诊断行控制台；消息/参数/命令逻辑委托共享核心，行为与 P1/P3a 一致 |
| TELEM1 | UART7（RX PF6、TX PE8），115200 8N1，DMA1_CH0/CH1，Async 传输；独立序列号、解析器、发送队列与 1 s 心跳节拍 |
| 独立运行 | 双链路各自维护 MAVLink 序列、RX 解析、发送队列与 stream rate；互不等待，慢/缺接收端不阻塞采样或另一链路 |
| 版本显示 | AUTOPILOT_VERSION 由共享核心构建，两端连接时与请求时均发送；`check_mavlink.py` 对飞行版本/source/UID 断言保持通过 |
| 软件检查 | fmt、普通/注入/Dump target Clippy、host Clippy `-D warnings`、42 项 Rust host 测试、17 项编译时钟谓词全部通过 |
| release/打包 | Linux Rust 1.99.1；source=`3b03df195f9c7cdf`；raw/padded=`341584/341600 B`；static RAM=`221252 B`、空余 `236476 B`；布局/向量/ELF/bin/PX4 一致 |

普通 `.px4` SHA-256：`816518587e8a191d38536e15ae67fe682cfc0fab45635364f6a73ba3b472169f`；`.bin` SHA-256：`4590317713bddfbf700a0b7f5e3aa830aae782e395a27fdc784b18c7040a4bd9`。本轮没有烧录、访问 SD 卡或改变当前板载 P3a 固件。

## 待工程师实物验收

- H-03：TELEM1 UART7 线序/电平、USB-UART 适配器、波特率与流控。当前软件为 115200 8N1 无硬件流控，实物接线确认后如启用 RTS/CTS 再适配。
- MAV-02：两个独立客户端（QGC + 脚本）同时运行，USB 与 TELEM1 各连接一次、双链路同时工作并各自重连一次；同一参数/数据源在两链路读到一致当前值。
- MAV-03：QGC Inspector 的 AUTOPILOT_VERSION 在连接与重连后均显示；P1 的重连未显示问题由本次共享核心在每次会话重启重发初始突发并在请求时响应，实物 HIL 复核。

## 范围与后续

TELEM1 目前无诊断行控制台；MAVLink 请求处理、参数读写、心跳/状态/时间与命令响应与 USB 一致。真实 IMU/AHRS/ESC/GNSS 遥测随对应数据源阶段接入（TEL-01）。本结果仅允许继续软件开发，不放行依赖 P2 的硬件阶段。