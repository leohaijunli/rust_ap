# Pixhawk 6X Rust 飞控 TODO

更新：2026-11-02。**P0、P1、P3a、RTC-01/TIME-01 完成；P4b-a、PL-a、P2、P4 软件门通过，PL-a/PL-b 有限 HIL 通过；实物集中待 H-02/03/04 与工程师操作，GNSS 授时和 M1 未开始。**
规则与必要测试：[plan.md](plan.md) · 证据：[P0](docs/acceptance/p0/README.md) / [P1](docs/acceptance/p1/README.md) / [P3a](docs/acceptance/p3a/README.md) / [P4b-a](docs/acceptance/p4b-a/README.md) / [PL-a](docs/acceptance/pl-a/README.md) / [P2](docs/acceptance/p2/README.md) / [P4](docs/acceptance/p4/README.md) / [PL-b](docs/acceptance/pl-b/README.md)

## 执行规则

- 只做[必要测试](plan.md#4-仅保留必要测试)，复用已有通过结果；相关实现变化或新失败才补测。
- `[x]` 有证据；阶段通过后放行依赖阶段。工程师负责实体操作，agent 负责实现和记录。
- 保持纯 Rust/Embassy、原 PX4 bootloader、`0x08020000` 入口和 QGC 恢复能力。
- 完成本次工作后提交推送；文档/计划修改只做内容与差异检查。
- 编写时用 `clean-code`，收尾时用 `code-simplifier`；[项目技能](docs/ai-coding-skills.md)已安装。

## 当前优先工作 — MAVCAN

工程师要求先完成飞控 MAVCAN passthrough，再开发 GPS；范围与必要检查见 [CAN-BRD](plan.md#can-brd-当前范围)。

- [x] 固定并核对 VimDrones `7bfe9963` / pydronecan `092d09cb` 协议：经典 DroneCAN 用 `CAN_FRAME`（386）、`MAV_CMD_CAN_FORWARD`（32000）握手，ID bit31=扩展、bit30=RTR。
- [ ] **进行中**：修正 CAN2/FDCAN2（PB12/PB13）桥接，补全总线/目标/长度校验、转发租约、过滤、固定队列、超时取消及 USB 会话清理。
- [ ] MAVCAN 核心/协议测试、fmt/Clippy、普通固件构建与资源核对。
- [ ] 烧录后核对 CAN2 双向互通、关闭/超时恢复和 DroneCAN GUI 节点可见；SLCAN codec 保留，当前不接入。
- [ ] 局部简化审视、更新真实验收结果并提交推送。

## 已确认范围

- [x] **D-A3 / D-RUST / D-EMBASSY**：当前工程 + Rust/Embassy + 按需复用固定 Cybflight 算法。
- [x] **D-M1 / D-QGC-M1 / D-PARAM**：IMU、AHRS、DroneCAN、ULog、USB/TELEM1、QGC 数据和标准参数读设。
- [x] **D-IMU / D-ESC / D-GNSS**：单 SPI2 ICM-42688-P；4 个 Vimdrones Nano / Quad X；H-RTK DroneCAN SKU12059。
- [x] **D-MESSAGES**：PX4 标准数据消息/ULog 字段 + MAVLink `common.xml` 标准消息与命令；[映射](docs/message-contracts.md)。
- [x] **D-INA219 / H-11 接线配置**：SEN0291，独立 I2C3 插座，拨码 A0=0/A1=1 即 0x44（固件自动探测 4 个地址）、10 mΩ、±8 A，测量独立支路；工程师确认已连接。
- [x] **D-PUBLIC / D-HOLSATUS / D-DEFER**：允许 GPL 来源并记录许可；Holsatus 构建修复暂缓；远期细节待 M1。
- [x] **P0-01～10 / H-01**：软件基线、来源、硬件资料与 A3/M1 方案已归档。

## P1 — 保持 QGC 恢复能力的 bring-up

依赖 P0；[当前结果](docs/acceptance/p1/20261031-hil.md)。

- [x] **P1-02/03**：构建/打包、source/协议、平台和备份域通过；[软件证据](docs/acceptance/p1/20261031-backup-domain.md)。
- [x] **P1-04 已测**：两组 65 s 时序/CPU/栈、两类重启及 panic/HardFault 保持/清除通过。
- [x] **独立 QGC 恢复**：USB 重插、Stable 上传/校验和 PX4 v1.17.0 运行已留档。
- [x] **P1-01 / H-05 / P1-04 收口**：21:28 UTC 从 PX4 直接回装普通包，CRC/source、disarmed、fault_injection=false、故障 valid=false 全部通过；[最终回装](docs/acceptance/p1/20261031-hil.md#最终普通回装与p1结论)。

P1 最终普通回装为 `a946a529360eac83`；最近实测板载为普通 Rust `81d6c1221048f08a`（raw/padded=`370064/370080 B`、static RAM=`231704 B`），经原 PX4 bootloader 上传/校验；心跳 `autopilot=PX4`，disarmed、输出关闭，`LINK_LED_HZ=2.0` 的 P3a 保存值有效。RTC LSE 走时、写后回读、`SYSTEM_TIME`、日志 UTC 及短时旧 IMU ULog 已通过。Inspector 版本显示及主机重启后 CDC 零数据问题归 P2；FMU 灯目视项不适用。项目开始前原配置仍无备份，P3a 已备份写入前当前 FRAM。

## M1 前置阶段

- [x] **P3a / P3A-01～04**：FM25V02A 全量备份、统一参数 API、双槽原子持久化、一次保存/掉电读回通过；[证据](docs/acceptance/p3a/README.md)。
- [x] **MSG-01 标准消息软件检查**：`sensor_accel` / `sensor_gyro`、`actuator_armed` 与固定 PX4 定义对齐；fmt/Clippy、4 项受影响核心用例、pyulog 与普通打包/布局通过，软件 source=`c2f3128aecc8a70b`。[记录](docs/acceptance/messages/README.md)；实物核对并入 P4 下一次必验记录。
- [ ] **下一步 P4b-a / CAN-A01～04**：CAN-A01～03 软件门已通过；待 H-02/03 完成 ESC 映射/Status、零命令与一次断线恢复实测。[证据](docs/acceptance/p4b-a/README.md)；DNA 仅设备必需时加入。
- [ ] **PL-a / LOG-A01～05**：LOG-A01～04 及 LoggerControl/MAVLink Console 软件门已通过；实物正常 start/stop、MAVLink 下载和 pyulog 解析已通过；restart 先补录已有通过结果的直接证据，H-04 仍待受控断电恢复、缺卡/拔卡和时延边界。[证据](docs/acceptance/pl-a/README.md)；P4b-a 未关闭，不放行依赖阶段。
- [ ] **P4b-g / GNSS-01～03**：GPS 已接 CAN1；MAVCAN 收口后接入 H-RTK Fix2/Auxiliary、标准 `GPS_RAW_INT` 和 `sensor_gps` 日志并实测。CAN2 桥不自动转发 CAN1。阶段依赖 P4b-a、PL-a、H-10。
- [ ] **P2 / MAV-01～03**：USB/TELEM1 双链路、QGC 连接/重连及版本显示；MAV-01 软件门已通过，待 H-03 与工程师接线做双链路/重连实测。[证据](docs/acceptance/p2/README.md)。依赖 P1、P3a。
- [ ] **P3 / PARAM-01～02**：软件/工具路径已由 P3a/P2 就绪；待 P2 闭合与 H-03 接线后做双链路一致性、合法/非法值回显及保存重启读回。依赖 P3a、P2。
- [ ] **P4 / SENS-01～04**：软件门、实物 WHO_AM_I/SPI/短时 8 kHz 计数、USB `HIGHRES_IMU` 与旧 `imu_icm42688` 短时 1 kHz ULog 已通过；待 DRDY 仪器测量、60 s 标准 `sensor_accel` / `sensor_gyro` 日志、六面方向、校准与故障恢复。[证据](docs/acceptance/p4/README.md)。依赖 P1、P3a、PL-a。
- [ ] **P4p / PWR-01～04**：INA219 单条 V/A/W、Windows Inspector 同显和短时 ULog 已通过。日志参数默认值读取 bug 已修复并验证参数回显；实际停录/变速、保存后掉电恢复仍待验。全量日志曾出现 dropout，队列满原因需进一步定位。参考负载精度、支路供电和断线恢复待工程师配合。[记录](docs/acceptance/ina219/README.md)。
- [ ] **P6a / EST-01～02**：Mahony AHRS、姿态日志/QGC 方向；相对 yaw。依赖 P4、P2、PL-a。
- [ ] **P7 / TEL-01～02**：随真实数据源补遥测与速率控制。依赖相应数据源、P2。
- [ ] **M1-01～02**：一次 5 分钟数据链联合运行，核对数据、日志和资源；零命令/DISARMED。依赖以上阶段。

## M1 后续

- [ ] **PL-b / LOG-B01～02**：软件与有限 HIL 通过：QGC 协议列出 2 个日志，分块下载 306176 B 后 pyulog 无 corruption；MAVLink Console start/sync/stop 通过，restart 的直接证据待补录；心跳改 PX4 兼容后 QGC 下载应命名为 `.ulg`。待 QGC GUI Refresh/Download、断链与速率边界收口。[证据](docs/acceptance/pl-b/README.md)。依赖 M1、PL-a、P2。
- [x] **RTC-01 / TIME-01**：source=`81d6c1221048f08a` 实测 LSE `ticking=true counting=true`，30 s 增加 31 s，`!time set 1700000000` 回读 `ok=true`，`SYSTEM_TIME` 与 `LOG_ENTRY.time_utc=1700000004` 非零。根因是清 `RSF` 未解锁 WPR，且 `DR` 后再次读 `SSR` 会重新锁住影子寄存器；见[时间基准](docs/time-sync.md)。
- [ ] **TIME-02**：H-RTK GNSS 授时待 P4b-g；3D fix 且样本新鲜才采纳，GNSS > 地面站 `SYSTEM_TIME` > 继承 RTC，偏差 ≥5 s 才写；LSI 后备额外周期校准。无 GPS 驱动前，时间来自继承 RTC 或入站 `SYSTEM_TIME`。
- [ ] **P6a-e / EKF-01～03**：固定 PX4 v1.17.0 数学核心的纯 Rust 移植；不含 uORB/NuttX/FFI。先 IMU 预测与数值对拍，再接 GNSS aiding、健康/复位和 Mahony 回退。依赖 M1、P6a、P4b-g、P2、PL-b。
- [ ] **P4b-r / RTK-01～02**：RTCM 转发、RTK 与中断恢复。依赖 M1、P2、P4b-g、H-10。
- [ ] **P5a / OUT-A01～02**：单一仲裁、限时授权/撤销、看门狗及零输出门控。依赖 M1、P4b-a、PL-a、H-06。
- [ ] **P4b-b / CAN-B01～03**：拆桨电机映射/旋向与故障停转；恢复不自动解锁。依赖 P5a、P4b-a、PL-a。
- [ ] **P5b / OUT-B01～02**：RC、预检/解锁、仲裁与故障策略。依赖 M1、P6a、P4b-b、P2/P3、H-07。
- [ ] **P6 / CTRL-01～03**：控制/分配、实时性和台架/系留。依赖 P6a、P5b、PL-b。

## Backlog

- [ ] **P6b→P6c→P6d→P6e→P6f→P6g**：模式→GNSS 融合/导航→路径→影子回退→INDI/NMPC/轨迹→任务协议；依赖详见计划。
- [ ] **P3b / P8 / BACK-01～03**：元数据、按需外设/冗余和发布自动化。

## 待补硬件输入

| 编号 | 输入 / 阻塞阶段 |
| --- | --- |
| H-02/03 | ESC 固件/映射，CAN 适配器/接线/供电/终端，以及 TELEM1 UART7 适配器/线序/电平/流控；P4b-a/b、P2/P3 |
| H-04 | 当前卡的型号/布局、记录时延与允许掉电窗口；受控断电及缺卡/拔卡故障；PL-a |
| H-06/07 | 台架限幅/停转，RC/UART/电源和预检策略；P5a/b |
| H-08/09 | GNSS 坐标/精度，动力学/辨识/求解预算；P6c/d/f |
| H-10 | H-RTK 固件/节点/消息率及后续 RTCM 来源；P4b-g/r |
| H-11 | INA219 0x44 通信、单条 `DEBUG_VECT` V/A/W 在 Windows Inspector 已确认、ULog `rust_current_sensor` 已实测；待支路供电/接线确认、参考负载精度和一次断线恢复；P4p |
