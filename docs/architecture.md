# A3：M1 数据链架构

日期：2026-11-02。基础 **A3 已由用户选定**。本轮只规定 IMU/AHRS、DroneCAN、ULog、USB/TELEM1、QGroundControl 消息与参数读/设所需边界；控制/INDI/NMPC/任务细节待更多硬件测试。标准消息见[字段约定](message-contracts.md)，观测结果回填 [M1 验收单](m1-verification.md)。

## 1. 纯 Rust + Embassy

| 层 | 职责 / 实现边界 |
| --- | --- |
| FMUv6X 平台 | `embassy-stm32` 管理时钟、电源、GPIO、EXTI、SPI DMA、UART DMA、FDCAN、SDMMC2、USB；板级映射统一维护 |
| 设备 | 仅 SPI2 ICM-42688-P；寄存器协议使用 embedded-hal/embedded-hal-async 边界，保留 host mock 能力 |
| 调参/SDK 层 | `v6x_rust_led::config`：校验 IMU ODR/日志率/FSR/旋转与算法选择，输出驱动与 AHRS 可用的配置；无 I/O、host 可测，参数注册表映射待 P3/P4 实物结果后接入 |
| AHRS | 固定 `cybflight-core::mahony::Mahony<f32>` 的适配器；无 I/O，显式输入时间步长/数据/参数 |
| DroneCAN | 有界 no_std Rust v0 编解码、重组和节点服务；由平台转换为经典 CAN 帧 |
| 参数 | 一个静态注册表提供类型/范围/默认值/生效时机；USB、MAVLink、DroneCAN 与持久层共用 |
| 日志 | 纯 Rust ULog、PX4 标准测量/输出字段与独立诊断 topic；有界非阻塞入口、独立存储任务；LoggerControl 统一 start/stop/sync/status/config |
| 通信 | 复用 rust-mavlink；USB CDC 和 TELEM1 共用消息/参数/`SERIAL_CONTROL` shell 逻辑，各自有队列与速率预算 |

任务和时间采用 `embassy-executor` / `embassy-time`，tick 固定 **1 MHz**。板上应用无 C/C++ 运行库或 FFI；原 PX4 bootloader 为保留的恢复边界。`src/core` 放 no_std/host 核心，`src/drivers` 放硬件所有者，`src/comms` 放链路，`src/storage` 放 FRAM/参数/日志；启动、平台、诊断和故障入口留在 `src/` 根目录。

## 2. 采样与 fan-out

```text
ICM-42688 DRDY → 时间戳 → SPI DMA / FIFO → 单位与轴向转换 → AHRS
                                      ├→ 独立 ULog 入口
                                      └→ 最新 IMU 遥测快照
AHRS → 带有效性/时间的姿态快照 ────────────→ ULog、USB、TELEM1
FDCAN → 有界帧队列 → DroneCAN → ESC/GNSS 快照 → ULog、USB、TELEM1
USB / TELEM1 / DroneCAN 参数请求 → 单一注册表 → 实际值回显 / 变更事件
```

- IMU 使用 **8 kHz gyro/accel ODR** 与 20 B 高分辨率 FIFO 包；水位为 8 包，约 1 kHz 唤醒并批量解码。AHRS 可消费全部 8 kHz，ULog 抽取后各写 1 kHz `sensor_accel` / `sensor_gyro`（单样本），MAVLink 读取最新快照 20 Hz；不能用低速链路反压采样。
- 唯一 DRDY 为 PA10/EXTI15_10、CS PH5、供电 PF4。EXTI P0 handler 先保存单调时钟低 32 位，再唤醒 TIM7/P3 的 IMU task；任务按当前时间重建跨回绕时间戳，仍须用外部边沿测量真实 IRQ 延迟。
- FIFO 按包序号/器件时间还原每样本时间，检测缺包、溢出和重复。AHRS 使用测得的 dt，不能只因任务被唤醒就假定得到一个新样本。
- AHRS 输入有界队列初值 16 个样本；满时显式记录丢样/重同步，不追赶陈旧样本造成长时间占用。日志不与 AHRS 争抢同一个消费型 Channel。
- AHRS 输入先校验有限值、加速度范数与 dt；超期/异常保持可诊断的无效状态。内部误差恢复和参数更改通过明确的 reset/apply 边界完成。
- 遥测读最新快照，慢 USB 主机或 UART 发送不能阻塞 SPI/AHRS；每个消费者看到的样本序号、年龄和丢弃统计可观测。

### 调参/SDK 层（`config`）

为了让设置、调参和换算法不经改驱动/改 HAL，lib 已提供单一 `v6x_rust_led::config`。它是 no_std、无 I/O、host 可测的纯逻辑层：**校验配置并输出驱动与算法可直接使用的值**，不自己持存储或外设。当前运行配置为 8 kHz ODR、1 kHz ULog、±16 g、±2000 dps、YAW270 和 Mahony；运行时参数映射待保存格式与实物结果确认后加入。

- 职责边界：只做"参数值 → 驱动/算法配置"的映射与校验。SPI/DRDY/EXTI 等平台细节不在本层；寄存器协议仍在设备层（embedded-hal 边界）。
- 接口形态：`Config::imu()` 返回 `ImuConfig { odr_hz, log_rate_hz, accel_fsr_g, gyro_fsr_dps, rotation, stale_after_us }`；`Config::algorithm()` 返回 `Algorithm::Mahony`，供后续 AHRS 适配器读取。
- 生效时机：与参数注册表 `Access` 一致——`ReadOnly`（如 `MAV_SYS_ID`）只读；`NextCycle`（如 IMU/AHRS 调参项）在显式 `apply` 边界生效，避免运行中直接改系数造成不连续。
- 范围校验：当前 API 只接受已实现的 1/8 kHz ODR、整除 ODR 的日志率、±16 g 和 ±2000 dps；非法配置不能跨过 `Config::new`。接入参数注册表后继续复用其 min/max/有限值校验。
- 算法可替换：`Algorithm` 枚举把"当前算法选择"显式化，AHRS 只依赖该选择而非硬编码调用；新算法经配置层接入，不动驱动与遥测。
- host 可测：config 无 HAL 依赖，可单测默认值、参数映射、越界回退与 `Algorithm` 选择，作为 P4"关键单位/坐标和陈旧数据处理用例"的 host 部分。
- KISS/YAGNI：本层只暴露 P4 当前真实需要的字段，不为 AHRS/控制/任务预留空表；后续阶段按实际需求扩展。

## 3. 坐标、时间与有效性

| 数据 | 契约 |
| --- | --- |
| IMU | 角速度 rad/s，加速度为比力 m/s²，温度 °C；源 ID 固定为该 ICM-42688-P |
| 内部机体系 | FLU：前/左/上；驱动到机体的安装变换由 PX4 YAW270 配置与六面/转动实测共同验证 |
| AHRS 导航系 | Cybflight 的 ENU/z-up 数学约定；M1 未观测绝对航向，水平轴仍是初始化的局部参考 |
| 四元数 | 对外结构显式 `[w,x,y,z]`、body→world；与 nalgebra 使用命名分量转换，不转储内存 |
| MAVLink | 按消息约定转换至 FRD/NED；IMU 静止时比力的符号、roll/pitch/yaw 方向须实测 |
| PX4 标准消息 / ULog | 同样使用 FRD/NED；`timestamp` / `timestamp_sample` 与完整固定 `.msg` 字段一致，内部 DTO 元数据单列 |
| GNSS | 原始 WGS84 经纬度、MSL/椭球高度、NED 速度及质量；M1 尚无融合位置/速度 |
| 时间 | 单调 `u64` 微秒；sample、receive、publish/write 时刻分开。GNSS 源时间和时间标准单列，未同步不替代本地时钟 |
| 有效性 | 有效/陈旧/无定位/未同步/相对 yaw 均显式；NaN/未知值不转成伪造的零测量 |

向量转换：`v_FRD = diag(1,-1,-1) × v_FLU`，`v_NED = [[0,1,0],[1,0,0],[0,0,-1]] × v_ENU`。旋转矩阵应同时转换机体和世界两端，再构造四元数；不能只交换四元数数组或改 Euler 符号。M1 的相对 yaw 不用作真北航向或 GNSS 航向。

M1 快照至少携带源、序号、本地微秒时间、有效标志和数据年龄。具体线格式大小受 [日志预算](logging-budget.md) 约束；GNSS 源数据与 AHRS 姿态分别发布，不提前制造 `GLOBAL_POSITION_INT` 融合结果。

## 4. Embassy 资源与调度初值

下表为 M1 分配提案；P1 时基/USB、P4b-a FDCAN、PL-a SDMMC2、P2 UART7 与 P4 IMU 已接入软件。PX4 来源见台账，不能将 PX4 DMAMAP 宏当作 Embassy 已分配的 stream。NVIC 数值越小越优先。

| 路径 | 资源 / 初始优先级 | 要求 |
| --- | --- | --- |
| DRDY | EXTI15_10，P0 | 尽早时间戳/唤醒；共享向量统一处理 |
| SPI2 DMA | DMA1_CH2/CH3（STREAM2/3），P1 | Embassy trait 编译核对；24 MHz 请求值，实际配置由 `!imu` 输出，实物待验；不与 UART 共用 |
| 时间驱动 | P1 已显式 TIM2，P1 | tick 1 MHz，PSC=239、ARR=65535；HAL 半溢出/溢出 IRQ 扩展时间 |
| FDCAN1 | IT0/IT1，P2；TIM6_DAC I/O executor，P4 | P4b-a 已实现：IRQ 唤醒 P4 任务，转入 64 帧有界队列并独立调度周期安全帧 |
| IMU/AHRS | TIM7 软件 pend 的 InterruptExecutor，P3 | IMU task 与既有周期诊断同 executor；AHRS 尚未接入。16 槽队列只在 AHRS 首次等待后启用，满时清积压、计数并从最新样本重同步 |
| CAN 协议 | 线程 executor | P4 只做 I/O/周期帧；重组、节点与参数服务走有界跨执行器通道，不持有高优先级锁 |
| USB / UART7 | OTG_FS；UART7 DMA1_CH0/CH1，P6 | USB/TELEM1 双链路互不等待；各自 20 Hz 读取同一 IMU 最新快照 |
| SD | SDMMC2/P7、内部 IDMA；线程存储任务 | PL-a 已构建：PC13 默认低、显式启动、32 KiB staging、FAT 单写者；同步 HAL 等待仍待 H-04 量化 |
| 日志/参数/低速遥测 | 线程 executor | 有界 poll；任何等待都不持有高优先级路径需要的锁 |

Embassy 在同一个 executor 内是协作调度；同步忙循环会阻塞该 executor。高优先级采样使用独立 InterruptExecutor，仍须限制 PRIMASK 临界区。HAL 的 SD 命令/取消与 CAN abort 忙等须测量/有界化，不能靠外层 async timeout 假装抢占。

### RAM / cache 初值

| 区域 | M1 提案 | 验证 |
| --- | --- | --- |
| AXI SRAM 512 KiB | 普通/事件日志 136 KiB；任务 futures ≤64 KiB；AHRS/快照 ≤8 KiB；参数/文件系统/通信 ≤32 KiB；其他 ≤16 KiB；共享 CPU 栈 64 KiB | 合计上限 320 KiB，保留 ≥192 KiB；以 map/实际 ABI 核对 |
| D2 SRAM 288 KiB | DMA pool 64 KiB（SD staging 32 KiB、SPI/UART 与余量）；CAN 软件结构 ≤32 KiB | 当前 CAN 队列和 SD staging 均在 AXI；FDCAN 使用专用消息 RAM，D2/MPU 仍待后续验证 |
| DTCM / SRAM4 / backup | M1 暂无必需分配 | 不把 DMA 数组默认移入 DTCM；RTC bootloader 签名按原约定处理 |
| flash | 应用仍为 0x08020000、1920 KiB 上限；M1 镜像目标 ≤1536 KiB | 含向量、代码、只读数据和 `.data` load image，打包器同步检查 |

DMA pool 采用经验证的 MPU non-cacheable 区或完整 clean/invalidate 规则，地址/长度按 32 B cache line 对齐，不共享含其他可变数据的 cache line。当前 D-cache 关闭；最近实测包 static RAM 为 231704 B，最新软件布局以 build.json 为准。M1 初始余量目标为 192 KiB，实物 SPI DMA/IDMA 可达性待验。

Embassy task future 是静态存储，不是每任务独立线程栈；中断嵌套和函数临时量都计入共享 CPU 栈。64 KiB 栈初值要求实测至少剩余 32 KiB，并通过水位/保护区检查；不能从 `.bss` 大小推断栈安全。P1 的 AXI 实际布局为447 KiB static allocation、独立1 KiB HardFault栈、64 KiB共享栈；当前 static 使用/空余以 [P1报告](acceptance/p1/README.md)为准，并非表中M1资源已分配。

### P1 实际测量边界

`src/platform.rs` 配置 HSE/PLL1 480 MHz、HCLK240/PCLK120 MHz、LDO/Scale1+ODEN、I-cache开/D-cache关，assert与文本诊断检查PLL/分频/供电/flash等待；这些是配置核对，不证明物理频率。线程用 Embassy raw executor 与 measured idle，TIM7/P3跑周期替身，USB/P6通信。

替身使用绝对release，不追赶过期任务；分别记录 body deadline miss 与统计/快照越界跳过。body/IRQ cycle打点不覆盖整个ISR入栈/出栈及全部bookkeeping；报告观测分布，不称WCET，不替代DRDY外部打点。CPU为wall减PRIMASK下WFE bracket，包含MMIO/DSB等被减开销，tick误差上界和硬件开销分开；不能仅凭exit bookkeeping计数宣布全局屏蔽≤2 μs。

共享栈用 cortex-m-rt 0.7.7 paint-stack 与可抢占扫描/有限SP采样，无法证明所有瞬态峰值。HardFault裸入口先切独立应急栈再保存CRC故障、请求原bootloader；panic等路径仍先用共享栈，备份域断电/嵌套故障不保证恢复。详见 [P1操作单及测量限制](acceptance/p1/bring-up.md)、[静态故障审查](acceptance/p1/fault-review.md)。

## 5. 首轮稳定性测量目标

| 指标 | 初始目标 / 记录方式 |
| --- | --- |
| 板上单调时基 | 对比 host/仪器至少 60 s，漂移目标 ≤0.1%；记录观测窗口和测量误差 |
| IMU ODR | 8 kHz 采集、8 包 FIFO 批、约 1 kHz 任务唤醒；分别记录采集/批次/1 kHz 日志率，偏差目标 ≤1% |
| DRDY 打点抖动 | <10 us，需外部边沿/输入捕获对照；纯任务时间戳只能给内部延迟证据 |
| DRDY→样本发布 | 观测最大 ≤500 us；正常段零 FIFO 溢出/丢样/倒退时间戳 |
| AHRS | 每新样本更新；单次 IMU/AHRS 快路径 ≤300 us，快照年龄 ≤2 ms；正常段无 NaN/无效四元数 |
| 共享临界区 / logger producer | 每次全局屏蔽中断 ≤2 us；`try_record` ≤5 us；原目标不变，P1 masked-WFE等待未满足此项证明，进入采样前需调整idle并按原阈值实测，pending→服务延迟另记 |
| CAN | M1 预算 33.028%；60% 持续压力下正常段无 RX 溢出；零命令年龄/间隔目标 ≤5 ms |
| 日志 | 标准消息 M1 提案 164322 B/s，含余量 205403 B/s；固定槽缓冲按记录数预算，M1 5 分钟正常段零 dropout；见[日志预算](logging-budget.md) |
| CPU / 栈 | 联合负载 CPU <50%；共享栈余量 ≥32 KiB，记录所有队列高水位 |

用 DWT cycles/微秒时钟记录分布、最大值、样本数、deadline miss；USB 输出低频汇总，原始统计进入 ULog。报告“观测最大值”，不称为已经证明的 WCET。过载/断线注入单列时间段，验证计数和恢复；不可混进正常统计以掩盖丢样。

这些目标用于找到硬件瓶颈；初次结果若不达标，先修复/调整再验收。非零电机停转阈值、闭环稳定性与求解时间等依据后续实测再规定。

## 6. QGC、MAVLink 与参数

**上游 QGroundControl 连接、Inspector 消息检查和参数页读/设是 M1 必验项。** 历史 q-MAG/MagGCS 连接不能替代。心跳 `autopilot` 为 `MAV_AUTOPILOT_PX4`：QGC 只在 PX4 身份下把机载日志保存为 `.ulg`，其余身份保存为 `.bin`（内容同为 ULog）。不声明 `MAV_PROTOCOL_CAPABILITY_FTP`，日志仍走 `LOG_REQUEST_LIST/DATA/END` 消息通道；能力位只声明已支持的功能。

| 消息 | 首轮默认速率 / 条件 |
| --- | --- |
| HEARTBEAT、SYS_STATUS、SYSTEM_TIME | 1 Hz；健康位与电池/UTC 未知值真实填写 |
| ATTITUDE | 20 Hz；有效 AHRS；按请求可调至经预算允许的速率 |
| ATTITUDE_QUATERNION | 按需请求/设置速率；与 ATTITUDE 的总额度一起限制 |
| HIGHRES_IMU | 20 Hz；只标记实际更新的 gyro/accel/temperature 字段 |
| ESC_STATUS / ESC_INFO | 10 Hz / 1 Hz；四个真实 ESC 状态，正确处理 K→消息单位及未知值 |
| GPS_RAW_INT | 按收到的 GNSS，目标 5 Hz；GPS 无定位也如实表示 |
| AUTOPILOT_VERSION、PARAM_VALUE、STATUSTEXT | 请求/变化触发；队列/发送速率受限 |
| SERIAL_CONTROL（SHELL） | QGC MAVLink Console；有界行缓冲与分片响应，logger/诊断命令，不提供任意内存或 OS shell |

USB 与 TELEM1 分别维护 MAVLink 序列号、RX 解析器、发送队列和 stream rate。首轮 TELEM1 候选 115200、8N1：线速上限 11520 B/s，发送总额先限 70%（8064 B/s），常规遥测目标 ≤6000 B/s，余量供参数回复；USB 常规限 32000 B/s。超过请求预算返回明确结果，不积压旧遥测。更高波特率根据实际接线/USB-UART 设备测试后采用。

MAVLink Console 使用 `SERIAL_CONTROL_DEV_SHELL`，只解析固定命令（`help/status/platform/metrics/fault/fram/params/dronecan/imu/logger`）；输入上限 32 B，输出按 70 B MAVLink payload 分片。`logger start/stop/sync/config` 和标准 `MAV_CMD_LOGGING_START/STOP` 共用 `src/core/logger.rs` 状态契约与 `src/storage/sd_logger.rs` 单一所有者，不允许控制面绕过 flush/断电顺序。

参数契约：

1. 名称 ≤16 ASCII 且唯一；REAL32 对外，整数/枚举必须精确可逆；拒绝 NaN/Inf、越界、非整数枚举和只读写入。
2. `PARAM_REQUEST_LIST` 限速发送完整表，`param_count/index` 稳定；`PARAM_REQUEST_READ` 支持名称/索引。
3. `PARAM_SET` 校验后应用并回显 **实际值**；失败回显保留值并有原因。两条链路访问同一表，重复设置幂等，节点/总线配置按明确的重启/生效时机处理。
4. 当前值、待保存值与保存成功分开；持久化只有 disarmed 且输出关闭时允许。P3a 已只读识别/备份 FRAM 并确定双槽布局；内部 flash 会受原 bootloader 应用擦除影响。
5. QGC 参数页加独立 pymavlink 脚本检查全表、按名/索引访问、合法/非法设置、双链路一致性、掉电后的实际保存结果。QGC 元数据和专有校准向导不是 M1 的参数协议前置。

### P3a 参数 API 与存储契约

参数只有一份静态注册表 `src/core/parameters.rs`：每条包含名称、类型、默认值、范围、只读/生效时机。`MAV_SYS_ID` 为只读整数，`LINK_LED_HZ` 为 0.1–10 Hz 实数、下一个灯周期生效。

| 层 | 位置 | 职责 |
| --- | --- | --- |
| 注册表与 RAM 状态 | `src/core/parameters.rs`（no_std、host 可测） | `Id` 类型键、名称/索引查找、范围与整数校验、`Parameters::{get,set,snapshot,restore,is_dirty}` |
| 存储格式与提交 | `src/core/parameter_store.rs`（无 HAL） | 双槽记录编码/解码、schema 标签、CRC、生成号选择、写入计划 |
| 设备服务 | `src/storage/parameter_manager.rs` | 唯一持有 FRAM；`get/set/is_dirty/status` 与异步 `load/save`，保存后按“已校验快照”确认 |
| 传输 | `src/comms/mavlink_core.rs`、`src/comms/usb_console.rs` | MAVLink 读设/保存与 `!params`、`!param-save`、`!param-load` |

**新增参数**：在 `parameter_table!` 加一条，功能侧用 `Id::新名称` 读取即可；MAVLink 全表/按名/按索引、FRAM 记录和诊断输出自动跟随。名称 ≤16 个 ASCII 大写/数字/下划线且唯一，默认值必须在范围内；只读项写 `ReadOnly`。

功能侧共用主任务创建的 `ParameterManager`（当前为单执行器服务）；`set` 返回实际是否改变，错误不改变 RAM，只有 `save` 成功才确认存储快照：

```rust
let hz = parameters.get(Id::LinkLedHz);
let changed = parameters.set(Id::LinkLedHz, 2.0); // Result<bool, parameters::Error>
let dirty = parameters.is_dirty();
let saved = parameters.save(output_state).await; // Result<Saved, manager::Error>
let loaded = parameters.load().await; // 从 FRAM 读回；无兼容记录则默认值
```

**FRAM 布局**（实物 FM25V02A，32 KiB，实测 `7F…C2 22 08`、SR=0x00）：

| 地址 | 内容 |
| --- | --- |
| `0x0000–0x77FF` | 现有 PX4 参数/校准数据；Rust 不写，写入前已全量备份 |
| `0x7800–0x7BFF` | Rust 参数槽 A（1024 B） |
| `0x7C00–0x7FFF` | Rust 参数槽 B（1024 B） |

记录（小端）：`0` 提交字节 `0xA5`、`1..5` 魔数 `RPAR`、`5` 保留 0、`6` 格式版本、`8` 注册表 schema 标签、`12` 生成号、`16` 参数个数、`18` 负载字节数、`20` 起按注册表顺序的 f32、末尾 IEEE CRC-32（覆盖 `1..负载末`）。

- **提交**：先把目标槽首字节写 0 使其失效 → 写其余字节并逐块读回 → 最后只写首字节 `0xA5` → 整槽读回并解码。任一时刻掉电，最多丢失本次新值，另一槽保持上一个有效版本。
- **加载**：解码两槽，按生成号（串行回绕比较）取最新有效记录；无有效记录则用默认值。启动和 `!param-load` 都不写 FRAM。
- **保存许可**：`OutputState{armed, outputs_enabled}` 任一为真即拒绝。P3a 无解锁/输出通路，固定为 false；P5a 接入同一仲裁真源后替换该常量。
- **升级策略**：格式版本或 schema 标签不匹配的槽被拒绝；有另一兼容槽则加载它，均无效才用默认值。`!params` 显示槽状态与原因，不做静默迁移；确需迁移时提升格式版本并单独实现。应用重刷和 PX4 bootloader 上传只擦内部 flash，FRAM 保留；回到 PX4 后该器件仍被其当作参数 MTD，后续 PX4 保存可能覆盖本区，恢复依据是写入前的全量备份。

## 7. 启动与实现顺序

先完成 [QGC 恢复往返](recovery.md) 和 P1 时基/诊断，再按 [plan](../plan.md) 接参数基础、CAN 零输出、SD、IMU/AHRS、TELEM1/QGC。可并行准备独立的 host 工作，硬件阶段依验收门推进。

M1 上电为 disarmed，RawCommand 四槽全零、ArmingStatus=0；设备重连不改变这一约束。AHRS/日志/通信异常可诊断，QGC 官方 PX4 恢复入口始终按 P1 验证的路径保留。
