# 基础评估与 A3 选择

日期：2026-10-31。**用户已明确选择 A3 混合**，并要求先验证 [M1 数据链](m1-verification.md)，其他细节等硬件测试后再定。本记录保留 A1/A2/A3 的比较依据；不是飞行/硬件验收。

## 1. 与首目标的匹配

| 项 | A1：Holsatus 为底 | A2：Cybflight 应用为底 | A3：当前工程 + 按需复用 core |
| --- | --- | --- | --- |
| 软件基线 | 四项依赖解析失败；按 D-HOLSATUS 跳过修复 | host 与 H743 构建通过 | 当前固件回归通过；core 独立 M7 git 依赖已编译 |
| AHRS | 有姿态/ESKF/校准任务，common 与运行时耦合 | Mahony/ESKF 已有纯算法与应用任务 | 直接适配 Mahony，保留其 host 回归 |
| ICM-42688-P | 新旧驱动 trait 共存，需核对并补本板接线 | 有 IMU 驱动/BSP，仍需 Rev 3 SPI2 适配 | 独立 Embassy SPI2/DRDY/DMA，按实测接入 |
| MAVLink/QGC/参数 | 已有 mavio/handler，mav-param 依赖阻塞 | 原生通信并非本项目的 MAVLink/QGC 路径 | 沿用 rust-mavlink，新增公共路由、读/设与 TELEM1 |
| 参数内部接口 | 配置/持久化与 common 任务耦合 | derive 注册表、YAML、存储逻辑可参考/复用 | 单一注册表；显式 MAVLink 别名/REAL32 校验，介质经 HIL 决定 |
| ULog / SD | 有 logger/FAT 任务，构建未验证 | MCAP+CBOR 与 FAT/SDMMC，非 ULog | 最小纯 Rust ULog；评估复用块设备/FAT 适配 |
| DroneCAN ESC/GNSS | 本轮未验证可直接满足最小消息集的栈 | 本轮未验证可直接满足需求的栈 | 独立纯 Rust v0 层，三者均需新增/适配 |
| Embassy/H753 | git Embassy 与器件平台需统一 | HAL 版本接近，但时基/BSP/任务需调整 | 保留 HAL 0.5.0、1 MHz 时基，新增高优先级采样 |
| QGC 恢复 | 新接原 bootloader、应用地址与 USB | 需改上游 0x08000000 布局及重启/打包 | 最大程度沿用已知良好 0x08020000/USB/打包入口 |
| 主要迁移成本 | 先恢复依赖，再剥离 common 与平台 | 迁移应用编排、通信、日志及板级布局 | 新写任务接线/协议层，但 M1 改动范围较清楚 |

**选择理由**：M1 的主要工作是高频设备采集、AHRS、DroneCAN、ULog 与 QGC/TELEM1。当前工程已有可回归的 USB/打包路径，Cybflight core 有独立 host/M7 证据；A3 可以保留这两部分的优势，避免在首次硬件验证前迁移整套应用调度。

A2 是有效对照，GPL 来源也未被排除。A1 保留“构建未验证”结论；授权跳过失败不代表通过。完整命令、数字与模块位置见 [porting-notes](porting-notes.md)。

## 2. M1 的实际适配工作

1. `Mahony<f32>` 输入前校验 NaN/Inf、加速度范数、时间连续性；只用 gyro/accel 时 yaw 为相对值。上游 gain setter 只检查符号，不能代替参数层有限值/范围校验。
2. 将唯一 SPI2 ICM-42688-P 的传感器轴映射到 FLU；将姿态/角速率转换到 MAVLink FRD/NED 约定。使用矩阵/基向量测试，避免只改 Euler 符号。
3. 日志采用独立订阅；USB、TELEM1 参数服务共用一个注册表。Cybflight `param_set_f32` 的整数转换会截断/饱和、bool 使用阈值，M1 入口须先做严格校验。
4. CAN、SPI/UART DMA、SDMMC2 都经 `embassy-stm32`；HAL 内部同步等待和缓存可达性仍须测量。使用异步接口本身不能证明采样稳定。
5. QGC 消息检查与读/设参数属于 M1；Generic 下参数页/Inspector 的实际体验由上游 QGC HIL 留档。

## 3. 已有远期调研，只作存档

以下是固定 Cybflight 源码的接口事实，按 D-DEFER 在硬件测试后再展开设计：

- `IndiController::step/step_alpha` 的 `IndiOutput` 已返回 `[0,1]` 电机命令和 `nan_failsafe`，包含 WLS 分配/推力线性化（`crates/cybflight_core/src/indi/controller.rs:192-199`）；不能当作只返回力矩再做第二次分配。`indi_off` 是无积分的比例 NDI 退化路径。
- 两种 SQP 模型 N=20：10 状态 `QuadModel` 输入总推力 N + 体轴角速度；13 状态 `FullQuadModel` 输入四个电机推力 N。后者在应用 `control/outer_loop.rs:1528-1563` 经分配矩阵转为 `(T_d, τ_d)` 给 INDI α 内环；模型输入与应用发布输出不同。
- `indi/rpm_tracker.rs` 接受 DShot `RpmInput::Erpm(u32)` 并按帧累计有效/无效次数。AM32 Status 已换算机械 RPM，50 Hz 状态也不能按 1 kHz 控制周期累计“缺帧”。
- core 有 ESKF、轨迹数学与参数逻辑；应用有任务编排、全局通道/监督和互斥控制 features，core 没有通用 `FlightMode` 插件接口。
- `plan_online` 同时门控工作区和 task future；不能只用运行时/const 标志推断大块 RAM 会被移除。

M7 ABI 探针只测 `size_of/align_of`：INDI 1852 B、ESKF 1068 B、Simple SQP 35000 B、Full SQP 54788 B、BFGS 工作区 55788 B、MincoSnap 85616 B，均 align 4。**不含临时栈和任务 future，也不是 M1 的内存需求。** 原始数据见 [core-abi.json](acceptance/p0/core-abi.json)。

## 4. 维护与来源

A3 初期直接依赖固定 git rev，保留锁文件和上游回归；若需修改，再建立可追踪 fork/vendor、保留来源头与修改说明。顶层 Apache 声明与 INDI/Betaflight 来源注释同时登记，见 [provenance](provenance.md)。最终飞控应用保持纯 Rust，host 参考工具与保留的 PX4 bootloader 属于独立边界。

目前完成的是选型和基线。上板恢复、器件识别、高频采样、SD、CAN、QGC/TELEM1 与参数实际行为仍按 [M1 验收单](m1-verification.md) 执行。
