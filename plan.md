# Pixhawk 6X Rust 飞控实施计划

更新：2026-10-31。路线：**A3（当前工程 + Rust/Embassy + 按需复用 Cybflight 算法）**；首目标：**M1 数据链**。
状态看 [TODO.md](TODO.md)，结果看 [P0](docs/acceptance/p0/README.md) / [P1](docs/acceptance/p1/README.md) / [P3a](docs/acceptance/p3a/README.md)。

## 1. 当前进度与执行原则

- P0、P1、P3a 已完成。P1 构建、普通版运行、时序、重启及故障保持/清除已有通过证据。
- P1 的独立 QGC 往返和 21:28 UTC 普通回装结果保留。**当前板上为 P3a 普通 Rust `1d3d08e3dd64f3ac`；FRAM 备份、参数 API、保存与掉电读回通过，下一步 P4b-a。**
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

## 3. M1 范围与实现边界

| 模块 | 首版范围 |
| --- | --- |
| IMU | ICM-42688-P 的 ID、DRDY/FIFO/DMA、1 kHz 采样起点、时间戳、SI 单位、方向及基础校准 |
| AHRS | Mahony gyro/accel 姿态、角速度与有效性；yaw 为相对航向，位置融合归 P6c |
| DroneCAN | 节点发现/服务、ESC 映射与 Status、零命令、H-RTK Fix2/Auxiliary；优先静态 ID，仅设备必需时增加 DNA |
| 日志 | 纯 Rust ULog、有界缓冲、SDMMC2/FAT、参数/数据/事件记录；M1 取卡解析，在线下载归 PL-b |
| MAVLink | USB CDC + TELEM1/UART7；真实心跳、状态、IMU/姿态、ESC/GNSS；Generic 身份 |
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
- 控制、参数和各链路使用同一真源与单一输出仲裁。细节见[架构](docs/architecture.md)、[DroneCAN](docs/dronecan-notes.md)、[日志预算](docs/logging-budget.md)。

## 4. 仅保留必要测试

**原则：一次有效结果覆盖一个行为；正常路径加关键失败路径。相同问题不在 host、脚本、GUI 和多轮刷写中重复验收。**

| 变更 / 阶段 | 必要检查 |
| --- | --- |
| 文档 / 计划 | 内容、引用和 `git diff --check`；复用已有软件结果 |
| Rust / 工具实现 | 相关 fmt/Clippy、受影响核心测试、实际使用配置的 release 构建/打包；涉及链接/打包时核对分区和向量 |
| P1 bring-up | 普通版启动/source/心跳/disarmed、平台与备份域、一次 60 s 时序采集、正常重启及受控故障恢复、独立 QGC 往返；本轮已测项直接复用 |
| P3a 参数存储 | 参数合法性和 CRC/中断提交的 host 用例；一次实际保存/掉电重启，读回有效状态；核对升级处理策略 |
| P2 / P3 协议 | USB/TELEM1 各连接一次，双链路同时工作并重连一次；全量读表、按名/索引读取、合法与非法设值各一例、保存后重启读回 |
| P4 / P6a | 实物 ID、六面方向与静态采样；QGC 转动确认姿态；关键单位/坐标和陈旧数据处理用例 |
| P4b-a / P4b-g | 关键单/多帧及 CRC/超时向量；实物节点/ESC 索引、零输出、GNSS 字段与定位；一次断线后状态失效/恢复 |
| PL-a | 一份日志经独立解析器读出；一次受控写入中断电并检查可恢复记录；一次缺卡或 I/O 故障，采样保持运行 |
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

保存的普通包 SHA-256：`7652fa90836be3182becf8f888a052aebc982ecbf67e6ba303dc66719db8fff5`，应用 CRC=`B9FB636E`。P1 必验项全部通过；下一阶段为 P3a 的 FRAM 只读识别/备份、参数表与持久化。

本次 QGC 使用的包 SHA-256：`3206a5f445d85be730a4f6158d8fdd414b9cfab27fa84bbc05c6a6752deac284`；控制台上传/校验与独立版本采集已足够，不额外要求刷写前 PX4 CRC 读回或再绕经 P0。
FMU 灯目视项因本板无可观察灯不适用；ETH 观察、Inspector 版本截图和完整升级 UI 文本作为补充记录。原 PX4 参数/校准/FRAM 无备份，保留此事实。

## 6. 阶段与依赖

每阶段只按第 4 节必要检查验收，完成后才进入依赖它的硬件工作。

| 阶段 / 编号 | 实现目标 | 依赖 |
| --- | --- | --- |
| P0 / P0-01～10 | 固定来源、硬件资料、A3/M1 基线；已完成 | — |
| P1 / P1-01～04 | 可恢复 bring-up 与周期基线；已完成 | P0 |
| P3a / P3A-01～04 | FRAM 识别/备份、参数注册表、持久化；已完成 | P1 |
| P4b-a / CAN-A01～04 | DroneCAN 节点、ESC 状态和零命令 | P1、P3a、H-02/03 |
| PL-a / LOG-A01～05 | ULog、缓冲、SDMMC2/FAT | P3a、P4b-a、H-04 |
| P4b-g / GNSS-01～03 | H-RTK 接收、有效性和日志 | P4b-a、PL-a、H-10 |
| P2 / MAV-01～03 | USB/TELEM1 双链路、消息/命令和重连 | P1、P3a |
| P3 / PARAM-01～02 | QGC 标准参数读设 | P3a、P2 |
| P4 / SENS-01～04 | 单 IMU 驱动、校准及采样 | P1、P3a、PL-a |
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

## 7. M1 后续

| 阶段 | 目标与必要验收 | 依赖 |
| --- | --- | --- |
| PL-b / LOG-B01～02 | 关闭日志的列表/下载/取消/删除；一次导出字节校验和删除限制检查 | M1、PL-a、P2 |
| P4b-r / RTK-01～02 | GPS_RTCM_DATA→RTCMStream；关键重组用例、实际 RTK 与改正中断/恢复 | M1、P2、P4b-g、H-10 |
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

远期接口和指标在 M1 硬件结果后细化。控制算法不直接驱动外设；输出必须经同一仲裁。同步求解不能仅靠 async timeout 声称有时限；保留可运行基线与独立的过期检测。

## 8. 待补硬件输入

| 编号 | 输入 | 归属 |
| --- | --- | --- |
| H-02 | ESC 固件/参数、node ID/esc_index、电机位置/旋向 | P4b-a/b |
| H-03 | CAN 适配器、端口/接线、供电和终端 | CAN 上板 |
| H-04 | SD 卡、记录/sync/允许丢失窗口 | PL-a |
| H-06 | 台架限幅/时限、撤销及停转判据 | P5a |
| H-07 | RC/UART、电源监测、预检与故障策略 | P5b |
| H-08 | GNSS 坐标/杆臂、精度与降级指标 | P6c/d |
| H-09 | 质量/惯量、动力模型、辨识和求解预算 | P6f |
| H-10 | H-RTK SKU/固件/节点/消息率；后补 RTCM 来源 | P4b-g/r |

## 9. 记录与交付

- `TODO.md` 只列状态、下一步和依赖；`docs/acceptance/` 保存结果与原始证据引用，已测结果可跨任务复用。
- 硬件资料：[台账](docs/fmuv6x-hardware.md)；来源/许可：[porting-notes](docs/porting-notes.md)、[provenance](docs/provenance.md)；设计：[architecture](docs/architecture.md)、[dronecan-notes](docs/dronecan-notes.md)、[logging-budget](docs/logging-budget.md)。
- 早期[恢复操作单](docs/recovery.md)、[P1 操作单](docs/acceptance/p1/bring-up.md)、[M1 操作单](docs/m1-verification.md)保留为参考；当前必验范围以本计划为准。
- 本轮损坏文档的原始副本已保存到 HIL 证据中的 `recovery/push-prep-restart-damage/`；源码和工具 hash 与已通过的软件基线一致。
