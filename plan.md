# Pixhawk 6X Rust 飞控实施计划

更新：2026-11-02。路线：**A3（当前工程 + Rust/Embassy + 按需复用 Cybflight 算法）**；首目标：**M1 数据链**。
状态看 [TODO.md](TODO.md)，结果看 [P0](docs/acceptance/p0/README.md) / [P1](docs/acceptance/p1/README.md) / [P3a](docs/acceptance/p3a/README.md) / [P4b-a](docs/acceptance/p4b-a/README.md) / [PL-a](docs/acceptance/pl-a/README.md) / [P2](docs/acceptance/p2/README.md) / [P4](docs/acceptance/p4/README.md) / [PL-b](docs/acceptance/pl-b/README.md)。

## 1. 当前进度与执行原则

- P0、P1、P3a、RTC-01/TIME-01 已完成。P4b-a、PL-a、P2、P4 软件门已通过，PL-a/P4/PL-b 正常路径有限 HIL 通过；P3 软件/工具验收路径可复用 P3a/P2。CAN-A04/LOG-A05/MAV-02/03、完整 P4/PL-b 仍待 H-02/03/04、QGC GUI 与工程师操作，GNSS 授时和 M1 未开始。
- P1 的独立 QGC 往返和 21:28 UTC 普通回装结果保留。**最近实测板载为普通 Rust `81d6c1221048f08a`：RTC LSE 走时/写后回读、`SYSTEM_TIME`、日志 UTC 和短时自定义 IMU ULog 通过；心跳 `autopilot=PX4`，QGC `.ulg` 文件名/日期显示仍待 GUI 确认。**
- 标准消息软件 source=`c2f3128aecc8a70b` 已通过必要检查/普通打包，见[记录](docs/acceptance/messages/README.md)；新 ULog schema 的实物核对并入下一次 P4 必验记录。
- 工程师要求：**只保留必要测试，复用已完成结果，本次完成后提交推送。** P1 依赖已放行，后续阶段仍按各自必验项验收。
- 本计划的精简验收取代早期操作单中的重复次数、长时压测和截图齐套要求。历史测试结果及缺失记录仍按实保存。
- 只实现当前阶段所需功能；清晰命名、小而直接的函数、复用现有工具。遵循 KISS/YAGNI，避免提前抽象、重复实现及无用注释。
- 已安装项目技能：编写时用 `clean-code`，收尾时用 `code-simplifier` 做一次局部审视；[适配说明](docs/ai-coding-skills.md)。
- 每个问题修复后只运行相关检查；已有检查通过且没有相关代码变化时直接继续交付。文档修改只检查内容、链接及差异。

## 2. 固定工程边界

1. 唯一目标为工程师的 Pixhawk 6X Rev 3 / STM32H753。引脚、时钟、电源和总线事实以固定 PX4 源码及实测为准，记录到[硬件台账](docs/fmuv6x-hardware.md)。
2. 板上应用保持**纯 Rust**；使用 `embassy-stm32`、`embassy-executor`、`embassy-time`，不以 FFI 引入 C/C++ 运行库。host 参考工具可用其他语言。
3. 保留原 PX4 bootloader、前 128 KiB、应用入口 `0x08020000`、board ID=53 和 QGC 刷回官方 PX4 的能力。上传只写应用区。
4. 所有固件默认 disarmed、输出关闭。M1 仅发送零 RawCommand / DISARMED；非零电机测试在 P5a 之后，由工程师拆桨、限流并按时限执行。
5. 只启用 SPI2 **ICM-42688-P**；不实现另外两颗 IMU、其他板型或运行时器件变体。
6. ESC 为 4 个 Vimdrones Nano / Quad X；GNSS 为 Holybro H-RTK DroneCAN SKU12059。采用 **DroneCAN/UAVCAN v0**，不是 Cyphal/v1。
7. 上游仓库、工具链、DSDL 和生成器固定版本。允许 GPL 等来源复用；保留适用版权/许可证并记录[来源](docs/provenance.md)。Holsatus 的既有构建问题暂缓，按需取用固定 Cybflight 算法。
8. 接线、供电、目视/QGC 和台架操作由工程师配合；人员称“工程师”。未知硬件事实和缺失备份如实记录。
9. 后续 EKF2 只移植固定 PX4 v1.17.0 的估计数学核心；不移植 uORB、NuttX work queue、`EKF2.cpp` 模块封装或 PX4 shell，不以 C++ FFI 接入。Rust 版本保持 no_std、固定容量与独立来源/许可记录。
10. 数据边界和 ULog 优先使用固定 PX4 v1.17.0 标准消息的字段、单位、时间与坐标语义；MAVLink 使用 `common.xml` 标准消息/命令。纯 Rust 类型与显式编码实现这些契约，标准映射见[消息约定](docs/message-contracts.md)。

## 3. M1 范围与实现边界

| 模块 | 首版范围 |
| --- | --- |
| IMU | ICM-42688-P 的 ID、DRDY/FIFO/DMA；8 kHz ODR、8 包批处理、200 Hz ULog（`LOG_IMU_HZ` 默认）、时间戳、SI 单位、方向及基础校准 |
| 支路电流 | 工程师新增 INA219/SEN0291：独立 I2C3 插座，0x44（A0=0/A1=1）并自动探测、10 mΩ、±8 A；10 Hz 采样，先用单条标准 `DEBUG_VECT` 同时显示 V/A/W |
| AHRS | 统一 `AttitudeEstimator` API 下先接 Mahony gyro/accel 姿态、角速度与有效性；yaw 为相对航向。EKF2 不作为 M1 前置，位置融合归后续 P6a-e/P6c |
| DroneCAN | 节点发现/服务、ESC 映射与 Status、零命令、H-RTK Fix2/Auxiliary；优先静态 ID，仅设备必需时增加 DNA |
| 日志 | 纯 Rust ULog、有界缓冲、SDMMC2/FAT；IMU 用标准 `sensor_accel` / `sensor_gyro`，输出状态用 `actuator_armed`；M1 取卡解析，在线下载归 PL-b |
| MAVLink | USB CDC + TELEM1/UART7；真实心跳、状态、IMU/姿态、ESC/GNSS；心跳 `autopilot=PX4` 以便 QGC 以 `.ulg` 保存日志，不声明 FTP 能力 |
| 参数 | 唯一静态注册表，标准 PARAM_* 读设、实际值回显、范围校验与持久化 |
| QGC | 上游 QGC 连接、查看真实数据、参数读设；不依赖 PX4 专用校准界面 |

实现原则：

- 算法核心保持 no_std、host 可测；HAL/引脚/启动集中在平台层，驱动只面向实际器件。按需要拆模块，不提前建立空框架。
- IMU/AHRS 高优先级执行；日志、遥测与存储低优先级。数据用独立订阅或快照分发，日志不能消费掉估计器的样本。
- 通道、重组、队列和重试有固定上限；慢 SD/链路不能阻塞采样。记录丢样、超时和队列高水位。
- 明确采样/接收/写入时间、坐标系、单位、有效性和数据年龄；未知数据按协议表示，不发布虚构姿态或融合位置。
- DMA 缓冲按控制器可访问 RAM 放置；D-cache 使用明确的 MPU 或 clean/invalidate 策略。不能把全部 RAM 当作连续 DMA 内存。
- FRAM 先只读识别并备份原内容。参数采用版本/CRC/原子提交，持久写入只在 disarmed 且输出关闭时执行；内部 flash 方案须明确升级擦除与导出/恢复。
- DroneCAN RawCommand 数组按 `esc_index`，不是 node ID 排列；撤销输出时清除待发非零帧，恢复不自动解锁。
- 控制、参数和各链路使用同一真源与单一输出仲裁。细节见[架构](docs/architecture.md)、[DroneCAN](docs/dronecan-notes.md)、[日志预算](docs/logging-budget.md)、[日志控制](docs/logging-control.md)。
- 估计器不直接访问外设或链路：输入为带 source/sequence/sample time/valid/age 的 DTO，输出为 `Watch<EstimatorState>` 最新快照；Mahony/EKF2 通过同一接口选择，配置只在显式 reset/apply 边界生效。
- 标准消息必须完整匹配字段语义，不能只改 topic 名；附加有效性/相对 yaw/链路诊断用独立元数据或 `rust_*` / `dronecan_*` topic。真实数据源接入时再新增消息定义与发布路径。

## 4. 仅保留必要测试

**原则：一次有效结果覆盖一个行为；正常路径加关键失败路径。相同问题不在 host、脚本、GUI 和多轮刷写中重复验收。**

| 变更 / 阶段 | 必要检查 |
| --- | --- |
| 文档 / 计划 | 内容、引用和 `git diff --check`；复用已有软件结果 |
| Rust / 工具实现 | 相关 fmt/Clippy、受影响核心测试、实际使用配置的 release 构建/打包；涉及链接/打包时核对分区和向量 |
| P1 bring-up | 普通版启动/source/心跳/disarmed、平台与备份域、一次 60 s 时序采集、正常重启及受控故障恢复、独立 QGC 往返；本轮已测项直接复用 |
| P3a 参数存储 | 参数合法性和 CRC/中断提交的 host 用例；一次实际保存/掉电重启，读回有效状态；核对升级处理策略 |
| P2 / P3 协议 | USB/TELEM1 各连接一次，双链路同时工作并重连一次；全量读表、按名/索引读取、合法与非法设值各一例、保存后重启读回 |
| P4 / P6a | host 单位/FIFO/坐标/陈旧数据用例；实物 ID、8 kHz ODR/1 kHz 批次与 200 Hz 日志、六面方向和静态采样；USB/TELEM1 `HIGHRES_IMU`；QGC 转动确认姿态 |
| P6a-e / EKF2 | 固定 PX4 输入的 Rust/PX4 数值回放；状态/协方差有限、对称及有界；延迟缓冲、陈旧/跳变/拒绝/复位与 Mahony 回退；实际 GNSS 融合另验 |
| P4b-a / P4b-g | 关键单/多帧及 CRC/超时向量；实物节点/ESC 索引、零输出、GNSS 字段与定位；一次断线后状态失效/恢复 |
| PL-a | 一份日志经独立解析器读出；一次受控写入中断电并检查可恢复记录；一次缺卡或 I/O 故障，采样保持运行 |
| CAN-BRD / MAVCAN | host 校验经典帧 ID/标志/DLC、转发会话/过滤；普通构建与资源核对；板载 CMD_CAN_FORWARD 握手、CAN2 节点双向收发和一次关闭/超时恢复，复用同次 USB 心跳确认 |
| M1 | **一次 5 分钟额定联合运行**：IMU→AHRS、CAN、SD、USB/TELEM1、QGC 同时工作；复用该记录核对速率、丢样、队列、CPU/栈及日志 |
| P5 / P6 非零输出 | 必须覆盖授权/撤销、过期命令、丢 RC/ESC、总线失效及停转；实际控制需台架/系留和实时性验证 |

- 默认取消固定 100 次参数回环、20 次随机断电、每阶段 30 分钟压测、全 feature 重建和重复 QGC 往返；出现新失败或相关实现变化时才增加针对性验证。
- 同一协议或数据正确性已有脚本证据时，QGC 只核对连接和用户可见功能，不要求逐条截图。P1 的 AUTOPILOT_VERSION Inspector 显示问题转入 P2 跟进。
- 测量阈值在对应测试前按实际配置确定；已有结果不因简化流程而改写。P1 的 1 kHz、±0.1% 时基、≤10 µs 内部 release 延迟、CPU <50% 和栈余量 ≥32 KiB 原判据保留。
- 5 分钟运行用于 M1 功能集成，不作为耐久性或飞行验收。关键停转、持久化及恢复检查仍是相应阶段必验项。
- 每项保留一条结论、对应固件/source 和必要原始输出即可。失败记录保留，完整 GUI 文本、重复截图和额外报告不是阶段门。

## 5. P1 收口

| 状态 | 结果 / 下一步 |
| --- | --- |
| 完成 | SCUEN/备份域修复、12 Rust / 64 Python、三版构建/布局与既有 ELF 回放，见[软件记录](docs/acceptance/p1/20261031-backup-domain.md) |
| 完成 | 普通 source=`a946a529360eac83` 的协议、平台/备份域、空载/USB 两组 65 s、两类重启及 panic/HardFault 保持/清除，见[HIL](docs/acceptance/p1/20261031-hil.md) |
| 完成 | 本次 QGC 独立 USB 入口、Stable 上传/校验、PX4 v1.17.0 运行；早一轮 P0/普通回装结果也已保存 |
| 完成 | 21:28 UTC 从 PX4 直接回装普通包一次；CRC=`B9FB636E`、source、3 个 disarmed 心跳、fault_injection=false、故障 valid=false，见[最终回装](docs/acceptance/p1/20261031-hil.md#最终普通回装与p1结论) |

保存的普通包 SHA-256：`7652fa90836be3182becf8f888a052aebc982ecbf67e6ba303dc66719db8fff5`，应用 CRC=`B9FB636E`。这是 P1 收口的历史包；P3a 随后已通过，当前下一步为 CAN-A04 与各阶段剩余硬件门。

本次 QGC 使用的包 SHA-256：`3206a5f445d85be730a4f6158d8fdd414b9cfab27fa84bbc05c6a6752deac284`；控制台上传/校验与独立版本采集已足够，不额外要求刷写前 PX4 CRC 读回或再绕经 P0。
FMU 灯目视项因本板无可观察灯不适用；ETH 观察、Inspector 版本截图和完整升级 UI 文本作为补充记录。原 PX4 参数/校准/FRAM 无备份，保留此事实。

## 6. 阶段与依赖

每阶段只按第 4 节必要检查验收，完成后才进入依赖它的硬件工作。

| 阶段 / 编号 | 实现目标 | 依赖 |
| --- | --- | --- |
| P0 / P0-01～10 | 固定来源、硬件资料、A3/M1 基线；已完成 | — |
| P1 / P1-01～04 | 可恢复 bring-up 与周期基线；已完成 | P0 |
| P3a / P3A-01～04 | FRAM 识别/备份、参数注册表、持久化；已完成 | P1 |
| P4b-a / CAN-A01～04 | DroneCAN 节点、ESC 状态和零命令；软件门通过，实物待验 | P1、P3a、H-02/03 |
| PL-a / LOG-A01～05 | ULog、缓冲、SDMMC2/FAT、LoggerControl/MAVLink Console；软件门/正常路径有限 HIL 通过，故障门待验 | P3a、P4b-a、H-04 |
| P4b-g / GNSS-01～03 | H-RTK 接收、有效性和日志 | P4b-a、PL-a、H-10 |
| P2 / MAV-01～03 | USB/TELEM1 双链路、消息/命令和重连；软件门通过，实物待验 | P1、P3a |
| P3 / PARAM-01～02 | QGC 标准参数读设；软件/工具路径就绪，双链路实物待验 | P3a、P2 |
| P4 / SENS-01～04 | 单 IMU 驱动、校准及采样；软件门通过，实物待验 | P1、P3a、PL-a |
| P4p / PWR-01～02 | SEN0291/INA219 I2C3 支路电流、有效性和标准遥测；本轮新增 | P1、P2、H-11 |
| P6a / EST-01～02 | Mahony AHRS、姿态遥测与日志 | P4、P2、PL-a |
| P7 / TEL-01～02 | 随已有数据源补遥测及速率控制 | 对应数据源、P2 |
| M1 / M1-01～02 | 一次完整数据链联合验收 | 上述 M1 阶段 |

### P3a 范围与结果

1. **P3A-01**：SPI5 只读 RDID/RDSR/READ；确认实物型号后备份全容量，并逐块读回校验。备份是当前内容，不补称为项目开始前的原始备份。
2. **P3A-02**：把现有 `MAV_SYS_ID`、`LINK_LED_HZ` 迁入唯一注册表，明确范围、只读/生效时机及实际值回显。
3. **P3A-03**：根据实物与备份确定 FRAM 布局，实现版本、CRC、双槽原子提交；当前值、未保存状态与持久成功分开。
4. **P3A-04**：参数合法性、CRC/逐字节中断提交 host 用例；一次实际保存、工程师掉电重启后读回，记录升级/不兼容数据处理。必验通过后才放行依赖阶段。

| 状态 | 结果 / 下一步 |
| --- | --- |
| 完成 | 实物识别 `7F…C2 22 08`、SR=0x00、32 KiB；128 块全量备份并复读一致，`0x7800–0x7FFF` 全 0，见 [P3a 记录](docs/acceptance/p3a/README.md) |
| 完成 | 统一注册表与 API、双槽 CRC/原子提交、MAVLink 读设/保存和 `!params` 已实现；17 项 host 测试与普通配置 Clippy 通过 |
| 完成 | 23:57 UTC 一次保存与工程师完全掉电重启读回通过；槽 A/生成号 1、`LINK_LED_HZ=2.0`、dirty=false；FRAM 其余内容保持。P3a 关闭，放行依赖阶段的 P3a 前置条件 |

### P4b-a 当前结果

1. **CAN-A01**：有界无堆 v0 传输、固定 DSDL 编解码、CRC/TID/重组及失败路径已通过 host 测试。
2. **CAN-A02**：FDCAN1/HSE 1 Mbit/s、P2 IRQ + P4 I/O 执行器、FIFO 与单次发送完成检查已构建；关闭自动重传以避开 HAL 无界 abort。
3. **CAN-A03**：NodeStatus/GetNodeInfo、参数 GetSet/保存、节点目录、四槽零 RawCommand 与 DISARMED 已接入；RestartNode 当前安全返回失败。
4. **CAN-A04 待验**：H-02/03 提供后，由工程师核对四个 ESC 身份/index、零输出、Status 和一次断线失效/恢复。当前不自动信任 candidate，不关闭 P4b-a；PL-a 仅先行软件开发。

### PL-a 当前结果

1. **LOG-A01/02**：固定 ULog schema、pyulog 1.3.0 独立解析、136 KiB 有界 MPSC 队列、事件保留和 dropout 已通过软件测试。
2. **LOG-A03/04**：SDMMC2/PLL1Q、PC13 显式启动、固定 FAT 栈、MBR/superfloppy、新文件与 32 KiB 对齐写入已构建。
3. **LOG-A05 待验**：实物正常 start/stop、MAVLink 下载和 pyulog 解析已通过；H-04 仍需受控断电、缺卡/拔卡和时延边界。一次旧日志断电后的 corruption/dropout 保留为失败记录，不能按通过收口。
4. **控制 API**：start/stop/sync/status/apply_config 共用单一状态机；QGC MAVLink Console 与标准 logging commands 已接入。正常 stop 后 Disabled 与解析已验证；restart 的通过标记尚缺直接证据引用，先补录已有结果。

详情和精确产物见 [PL-a 记录](docs/acceptance/pl-a/README.md)。

### P2 当前结果

1. **MAV-01**：`src/comms/mavlink_core.rs` 提供传输无关的消息构建、帧解析与入站处理；USB（`mavlink_usb.rs`）与 TELEM1（`mavlink_uart7.rs`，UART7 115200 8N1）共用，各自维护序列号、解析器、发送队列与节拍。
2. **MAV-02 待验**：H-03 与工程师接线后做双链路同时工作、各重连一次、同一参数/数据源两链路一致。
3. **MAV-03**：AUTOPILOT_VERSION 由共享核心构建，两端连接与请求时均发送；重连未显示问题由每次会话重启重发初始突发并在请求时响应，实物 HIL 复核。

详情和精确产物见 [P2 记录](docs/acceptance/p2/README.md)。

### P3 / P4 当前结果

1. **P3 软件/工具就绪**：P3a 已实现统一参数注册表、合法性校验、原子保存和掉电读回；P2 共享 MAVLink 核心已提供 `PARAM_REQUEST_LIST/READ/SET/VALUE` 与存储命令。`tools/check_mavlink.py`、`tools/p3a_parameters.py` 已覆盖全表、名称/索引、合法/非法设值及保存读回，不重复实现协议。
2. **P3 实物待验**：P2 闭合且 H-03 接线后，一条链路设值、另一条链路读回同一当前值，再复用既有保存/掉电读回路径；通过后关闭 PARAM-01/02。
3. **P4 软件门通过**：`src/core/icm42688.rs` 与 `src/core/config.rs` 提供寄存器策略、8 kHz 高分辨率 FIFO 解码、SI 单位、YAW270/FLU→FRD、陈旧判断与配置 API；`src/drivers/imu.rs` 接 SPI2 DMA1_CH2/3、PF4/PH5、PA10 EXTI、复位/回读/FIFO 恢复和周期寄存器核对。
4. **P4 fan-out**：AHRS 使用按需启用的 16 槽有界队列，满时丢弃积压并从最新样本重同步；ULog 按 `LOG_IMU_HZ`（默认 200 Hz，每 40 样本各记录一个）记录标准 `sensor_accel` / `sensor_gyro`（FRD/SI、`samples=1`，不是 8 样本平均）；USB/TELEM1 各 20 Hz `HIGHRES_IMU`，慢消费者不反压采样。标准消息软件检查见[记录](docs/acceptance/messages/README.md)，新 schema 随下一次 P4 必验记录核对实物。
5. **P4 有限 HIL**：普通包已由原 bootloader 上传/校验；WHO_AM_I=`0x47`、SPI=24 MHz、短时约 8.08 kHz、错误计数 0，USB `HIGHRES_IMU` 合理。旧 `imu_icm42688` 有 2686 条/2.652 s、约 1012.5 Hz、无 corruption；仍待 DRDY 仪器测量、60 s 日志、六面 YAW270、校准与故障恢复，不关闭 SENS-01～04。详情见 [P4 记录](docs/acceptance/p4/README.md)。

### EKF2 后续移植约束

1. **范围**：M1 仍用 Mahony 建立可运行姿态基线。新增 `P6a-e / EKF-01～03` 作为 M1 后续阶段，移植 PX4 v1.17.0 中 EKF 的 state、predict、covariance、aiding、innovation gate 与 health/reset 逻辑；不移植 uORB 和模块调度层。
2. **Rust 接口**：在 `src/core/estimator/` 定义 `AttitudeEstimator` / `NavigationEstimator`、`EstimatorInput` 和 `EstimatorState`。IMU 通过现有 16 槽有界队列输入；GNSS/后续传感器用带时间与有效性的快照；输出通过 `Watch` 独立给控制、ULog 和 MAVLink。
3. **首个融合子集**：EKF-01 先做 IMU 预测、roll/pitch 与相对 yaw；EKF-02 在 P4b-g 后加入 GNSS 位置/速度和固定容量延迟缓冲。磁力计、气压计、空速不以虚构数据占位，只有实际硬件阶段才加入。
4. **配置/切换**：`v6x_rust_led::config::Algorithm` 后续扩展 `Ekf2`；参数映射必须有范围校验和显式 reset/apply。EKF 无效、NaN、协方差异常、输入陈旧或 deadline miss 时撤销有效性并回退到已验证 Mahony，不无缝伪造连续状态。
5. **数值验证**：用同一固定输入分别运行 PX4 v1.17.0 参考核心和 Rust 版本，按状态、创新、方差及拒绝/复位事件对拍；容差在测试前固定。未通过 replay、边界失败路径和实际 GNSS HIL 前，不称为“等价 EKF2”。
6. **许可/来源**：逐文件记录 PX4/ECL 原路径、commit、BSD-3-Clause 版权与 Rust 修改点；估计数学输入保留独立 DTO，标准 PX4 消息用于数据边界，不链接 PX4 C++。

### INA219 支路电流（工程师新增）

- **PWR-01**：参考固定 LibDriver `266a1ee32a5f70e4e5d47591ce28c4e1c387857c`，纯 Rust/Embassy I2C3（PA8/PH8、100 kHz、DMA1_CH4/5、P6）；PG4 低使能 5 V 外设电源。SEN0291 拨码 A0=0/A1=1 对应 0x44；启动及每次重试只读探测 0x40/0x41/0x44/0x45 并采纳首个应答地址，10 mΩ、±8 A，采用 ±80 mV PGA、12-bit 单次 shunt+bus 转换、CAL=16776（FS0 固定零）、有界超时和回读/重试。
- **PWR-02**：工程师要求使用现有 QGC Inspector、三个量放在一条消息，先验证标准 `DEBUG_VECT`（ID 252）：`name=INA219`、`time_usec` 为采样启动相对时间，`x=电压 V`、`y=电流 A`、`z=功率 W`。USB/TELEM1 共用最新快照，各 10 Hz；W 为同次采样的 V×I，保留负电流符号。失败或年龄 >300 ms 停发，请求 ID 252 回同一消息；`!ina219` 可诊断。
- **PWR-03 日志**：同次有效测量直接进入现有有界 ULog 队列，主题 `rust_current_sensor`，字段为 `timestamp`、`timestamp_sample`、`voltage_v`、`current_a`、`power_w`，10 Hz；记录不依赖 MAVLink/QGC，采样失败不重复写旧值。日志仍显式 start/stop。
- **PWR-04 日志开关与速率**：新增 `LOG_EN_IMU/INA/CAN/HEALTH/EVENT`（0/1）与 `LOG_IMU_HZ`（200）、`LOG_INA_HZ`（10）、`LOG_CAN_HZ`（400）、`LOG_HEALTH_HZ`（10）参数，NextCycle 生效并持久化；`log_config` 按数据源组限速/启用，事件源只启用。新数据源按来源归组或新增一组参数。
- **必要检查**：host 标定/单位/符号、CNVR/OVF、范围/陈旧用例；普通配置 fmt/Clippy、release/布局；新 ULog 主题经 pyulog 独立解析，核对单位、符号和采样时间。刷写后做一次已连接传感器读取、MAVLink 字段/更新率及短时 ULog 主题核对；精度收口用一组已知负载/参考电流，一次断线失效/恢复。复用当前板和已通过结果，见[记录](docs/acceptance/ina219/README.md)。

### 时间基准与 GNSS 授时

1. **范围**：RTC 只服务日志 UTC、QGC 命名和 `SYSTEM_TIME` 遥测；控制与估计仍用 TIM2 单调时间，不依赖墙上时钟。
   RTC-01/TIME-01 已在 `81d6c1221048f08a` 通过 LSE 走时、写后回读、遥测/日志 UTC；TIME-02 GNSS 授时仍待 P4b-g，见[实测记录](docs/time-sync.md#2026-11-02-实测结论)。
2. **驱动约束**：PAC 直写 RTC/BDCR，不置 `BDRST`、不覆盖已是可信日期的日历；清 `RSF` 前解锁 WPR，日历快照固定为 `SSR → TR → DR`；同时验证 SSR 边沿与日历秒递增。必要时 `LSEDRV=HIGH` 重启 LSE，最后回退 LSI 并关闭失效的 LSE，避免下次启动等待 `LSERDY`。
3. **精度**：LSE 晶振约 ±20 ppm；LSI 约 ±5%，只作后备，必须由 GNSS 周期校准（偏差 ≥1 s 或每 60 s）。
4. **GNSS 授时（P4b-g）**：解析 H-RTK 的 `time_utc_usec`，仅 fix ≥3 且样本新鲜时采纳，按 PX4 的 5 s 阈值写 RTC；优先级 GNSS > 地面站 `SYSTEM_TIME` > 继承值。
5. **不做**：不回填已关闭日志的日期/名称，不修改 ULog 内部 boot 相对时间戳。

详情见 [时间基准方案](docs/time-sync.md)。

## 7. M1 后续

| 阶段 | 目标与必要验收 | 依赖 |
| --- | --- | --- |
| PL-b / LOG-B01～02 | 列表/下载/取消与删除限制；软件和协议 HIL 已通过，QGC GUI/异常链路待验 | M1、PL-a、P2 |
| P4b-r / RTK-01～02 | GPS_RTCM_DATA→RTCMStream；关键重组用例、实际 RTK 与改正中断/恢复 | M1、P2、P4b-g、H-10 |
| P6a-e / EKF-01～03 | 纯 Rust EKF2 核心：IMU 预测→GNSS aiding→健康/回退与遥测；固定 PX4 数值对拍 | M1、P6a、P4b-g、P2、PL-b |
| P5a / OUT-A01～02 | 单一仲裁、限时/限幅授权、看门狗与故障锁定；先验零输出门控 | M1、P4b-a、PL-a、H-06 |
| P4b-b / CAN-B01～03 | 逐电机映射/旋向、命令超时与断线停转；恢复不自动解锁 | P5a、P4b-a、PL-a |
| P5b / OUT-B01～02 | 直连 RC、预检/解锁、来源仲裁及丢控行为 | M1、P6a、P4b-b、P2/P3、H-07 |
| P6 / CTRL-01～03 | 速率/姿态控制和分配；回放、台架/系留、实际控制联合负载 | P6a、P5b、PL-b |
| P6b / MODE-01 | 模式/控制器槽位与无扰切换 | P6 |
| P6c / NAV-01 | GNSS 融合、位置/速度控制和失效降级 | P6b、P4b-g、H-08；RTK 指标另依赖 P4b-r |
| P6d / PATH-01 | 路径跟踪和任务入口 | P6c |
| P6e / SHADOW-01 | 候选控制器影子运行、受限启用及故障回退 | P6d |
| P6f / INDI-01、NMPC-01、TRAJ-01 | 辨识、INDI/NMPC/轨迹；有界求解与过期/NaN 回退 | P6e、H-09 |
| P6g / MISSION-01～02 | MAVLink 任务上传/执行/完成，复用参数和日志协议 | P6f、P2/P3、PL-b |
| P3b / P8 / BACK-01～03 | 元数据、按需外设/冗余、发布自动化 | 实际需求确认后 |
| CAN-BRD / 新增 | 飞控 MAVCAN passthrough：USB MAVLink ↔ CAN2/FDCAN2（PB12/PB13、1 Mbit/s 经典 CAN）；按 VimDrones/pydronecan 核对 CAN_FRAME、CAN_FORWARD 和过滤协议，详见下文 | 当前优先收口，CAN2 节点/接线由工程师配合 |

### CAN-BRD 当前范围

- 固定参考 VimDrones `7bfe9963bbbe42309f3b1d0851f6a138552ce7f6` 与 pydronecan `092d09cb9e6b115e262efe4cb42f7b65e6054f26`；纯 Rust/Embassy 复用 USB MAVLink 收发循环。SLCAN 保留未接入状态。
- `MAV_CMD_CAN_FORWARD`（32000）参数 1=2 开 CAN2、0 关闭；单客户端租约 5 s，pydronecan 每秒续约。`CAN_FRAME`（386）中的 bus=1，ID bit31=扩展、bit30=RTR；经典帧 0–8 B，非法 ID/长度和 CANFD_FRAME 拒绝而不截断。
- `CAN_FILTER_MODIFY`（388）bus=2，按 ID 的第 2/3 字节过滤；replace/add/remove、最多 32 个 ID。收发使用固定容量队列，关闭/USB 重置/租约过期清理旧会话；TX 有界等待与硬件取消，disarmed 且输出关闭才允许桥接。
- GPS 当前接 CAN1；CAN2 桥只能查看 CAN2 物理总线。GUI 自行解码透传的 DroneCAN；飞控侧 Fix2/Auxiliary、GPS_RAW_INT 与 sensor_gps 日志仍按后续 P4b-g 验收。
- 修复已验证：source=`a2b91d35630180da` 经典 `CAN_FRAME` 从 CAN2→USB 转发通过（`usb_frames=10`，匿名 Allocation 请求），握手/关闭/会话生效。[记录](docs/acceptance/mavcan/README.md)。**GUI 节点可见性待确认**：节点无静态 ID，需 GUI allocator 分配；出向 GetNodeInfo 待分配后验证。

远期接口和指标在 M1 硬件结果后细化。控制与估计算法不直接驱动外设；输出必须经同一仲裁。同步求解不能仅靠 async timeout 声称有时限；保留 Mahony 可运行基线、独立过期检测和 EKF2 明确回退。

## 8. 待补硬件输入

| 编号 | 输入 | 归属 |
| --- | --- | --- |
| H-02 | ESC 固件/参数、node ID/esc_index、电机位置/旋向 | P4b-a/b |
| H-03 | CAN 适配器/端口/供电/终端，以及 TELEM1 UART7 适配器、线序、电平、波特率/流控 | CAN 上板、P2/P3 |
| H-04 | 当前 SD 卡型号/布局、记录时延/sync/允许丢失窗口，以及断电和缺卡/拔卡故障 | PL-a |
| H-06 | 台架限幅/时限、撤销及停转判据 | P5a |
| H-07 | RC/UART、电源监测、预检与故障策略 | P5b |
| H-08 | GNSS 坐标/杆臂、精度与降级指标 | P6c/d |
| H-09 | 质量/惯量、动力模型、辨识和求解预算 | P6f |
| H-10 | H-RTK SKU/固件/节点/消息率；后补 RTCM 来源 | P4b-g/r |
| H-11 | INA219/SEN0291 独立 I2C 插座、0x44（A0=0/A1=1）、10 mΩ、±8 A、独立支路已确认且已连接；超时处理与总线恢复修改后 0x44 通信和三条命名值 10 Hz 已实测，原 SCL 低电平未复现；单条调试消息、参考负载精度和一次断线恢复待验 | P4p |

## 9. 记录与交付

- `TODO.md` 只列状态、下一步和依赖；`docs/acceptance/` 保存结果与原始证据引用，已测结果可跨任务复用。
- 硬件资料：[台账](docs/fmuv6x-hardware.md)；来源/许可：[porting-notes](docs/porting-notes.md)、[provenance](docs/provenance.md)；设计：[architecture](docs/architecture.md)、[消息约定](docs/message-contracts.md)、[dronecan-notes](docs/dronecan-notes.md)、[logging-budget](docs/logging-budget.md)、[logging-control](docs/logging-control.md)、[time-sync](docs/time-sync.md)。
- 早期[恢复操作单](docs/recovery.md)、[P1 操作单](docs/acceptance/p1/bring-up.md)、[M1 操作单](docs/m1-verification.md)保留为参考；当前必验范围以本计划为准。
- 2026-11-01 损坏文档的原始副本已保存到 HIL 证据中的 `recovery/push-prep-restart-damage/`；当轮源码和工具 hash 与当时通过的软件基线一致。
