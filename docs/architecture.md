# A3：M1 数据链架构

日期：2026-10-31。基础 **A3 已由用户选定**。本轮只规定 IMU/AHRS、DroneCAN、ULog、USB/TELEM1、QGroundControl 消息与参数读/设所需边界；控制/INDI/NMPC/任务细节待更多硬件测试。所有数字是首轮测试提案，观测结果回填 [M1 验收单](m1-verification.md)。

## 1. 纯 Rust + Embassy

| 层 | 职责 / 实现边界 |
| --- | --- |
| FMUv6X 平台 | `embassy-stm32` 管理时钟、电源、GPIO、EXTI、SPI DMA、UART DMA、FDCAN、SDMMC2、USB；板级映射统一维护 |
| 设备 | 仅 SPI2 ICM-42688-P；寄存器协议使用 embedded-hal/embedded-hal-async 边界，保留 host mock 能力 |
| AHRS | 固定 `cybflight-core::mahony::Mahony<f32>` 的适配器；无 I/O，显式输入时间步长/数据/参数 |
| DroneCAN | 有界 no_std Rust v0 编解码、重组和节点服务；由平台转换为经典 CAN 帧 |
| 参数 | 一个静态注册表提供类型/范围/默认值/生效时机；USB、MAVLink、DroneCAN 与持久层共用 |
| 日志 | 纯 Rust ULog 编码、有界非阻塞入口、独立存储任务 |
| 通信 | 复用 rust-mavlink；USB CDC 和 TELEM1 共用消息/参数逻辑，各自有队列与速率预算 |

任务和时间采用 `embassy-executor` / `embassy-time`，tick 固定 **1 MHz**。板上应用无 C/C++ 运行库或 FFI；原 PX4 bootloader 为保留的恢复边界。具体 crate 拆分随实现需要进行，不为目录重排延迟硬件验证。

## 2. 采样与 fan-out

```text
ICM-42688 DRDY → 时间戳 → SPI DMA / FIFO → 单位与轴向转换 → AHRS
                                      ├→ 独立 ULog 入口
                                      └→ 最新 IMU 遥测快照
AHRS → 带有效性/时间的姿态快照 ────────────→ ULog、USB、TELEM1
FDCAN → 有界帧队列 → DroneCAN → ESC/GNSS 快照 → ULog、USB、TELEM1
USB / TELEM1 / DroneCAN 参数请求 → 单一注册表 → 实际值回显 / 变更事件
```

- IMU 先以 **1 kHz ODR/处理率**为目标；SPI 起始时钟与上电/复位等待按器件 datasheet 和 SENS-01 实测确定。唯一 DRDY 为 PA10/EXTI15_10，CS PH5、供电 PF4，详见 [硬件台账](fmuv6x-hardware.md)。
- 时间戳尽量在 DRDY 入口取得；若 HAL 默认仅唤醒任务，BSP 需补时间戳 hook，分别记录 IRQ/任务/DMA 完成时刻，不能把任务开始时间当作已测 DRDY 边沿。
- FIFO 按包序号/器件时间还原每样本时间，检测缺包、溢出和重复。AHRS 使用测得的 dt，不能只因任务被唤醒就假定得到一个新样本。
- AHRS 输入有界队列初值 16 个样本；满时显式记录丢样/重同步，不追赶陈旧样本造成长时间占用。日志不与 AHRS 争抢同一个消费型 Channel。
- AHRS 输入先校验有限值、加速度范数与 dt；超期/异常保持可诊断的无效状态。内部误差恢复和参数更改通过明确的 reset/apply 边界完成。
- 遥测读最新快照，慢 USB 主机或 UART 发送不能阻塞 SPI/AHRS；每个消费者看到的样本序号、年龄和丢弃统计可观测。

## 3. 坐标、时间与有效性

| 数据 | 契约 |
| --- | --- |
| IMU | 角速度 rad/s，加速度为比力 m/s²，温度 °C；源 ID 固定为该 ICM-42688-P |
| 内部机体系 | FLU：前/左/上；驱动到机体的安装变换由 PX4 YAW270 配置与六面/转动实测共同验证 |
| AHRS 导航系 | Cybflight 的 ENU/z-up 数学约定；M1 未观测绝对航向，水平轴仍是初始化的局部参考 |
| 四元数 | 对外结构显式 `[w,x,y,z]`、body→world；与 nalgebra 使用命名分量转换，不转储内存 |
| MAVLink | 按消息约定转换至 FRD/NED；IMU 静止时比力的符号、roll/pitch/yaw 方向须实测 |
| GNSS | 原始 WGS84 经纬度、MSL/椭球高度、NED 速度及质量；M1 尚无融合位置/速度 |
| 时间 | 单调 `u64` 微秒；sample、receive、publish/write 时刻分开。GNSS 源时间和时间标准单列，未同步不替代本地时钟 |
| 有效性 | 有效/陈旧/无定位/未同步/相对 yaw 均显式；NaN/未知值不转成伪造的零测量 |

向量转换：`v_FRD = diag(1,-1,-1) × v_FLU`，`v_NED = [[0,1,0],[1,0,0],[0,0,-1]] × v_ENU`。旋转矩阵应同时转换机体和世界两端，再构造四元数；不能只交换四元数数组或改 Euler 符号。M1 的相对 yaw 不用作真北航向或 GNSS 航向。

M1 快照至少携带源、序号、本地微秒时间、有效标志和数据年龄。具体线格式大小受 [日志预算](logging-budget.md) 约束；GNSS 源数据与 AHRS 姿态分别发布，不提前制造 `GLOBAL_POSITION_INT` 融合结果。

## 4. Embassy 资源与调度初值

下表为 M1 分配提案；P1 已实现 TIM2/P1、TIM7/P3 和 USB/P6，其余外设仍待接入。寄存器物理回读/HIL 未完成；PX4 来源见台账，不能将 PX4 DMAMAP 宏当作 Embassy 已分配的 stream。NVIC 数值越小越优先。

| 路径 | 资源 / 初始优先级 | 要求 |
| --- | --- | --- |
| DRDY | EXTI15_10，P0 | 尽早时间戳/唤醒；共享向量统一处理 |
| SPI2 DMA | DMA1 两个独占 stream，P1 | 具体 DMAMUX/stream 在 BSP 编译时核对；不与 UART 共用 |
| 时间驱动 | P1 已显式 TIM2，P1 | tick 1 MHz，PSC=239、ARR=65535；HAL 半溢出/溢出 IRQ 扩展时间 |
| FDCAN1 | IT0/IT1，P2 | 只取帧/打点/计数，及时排空实际 3 帧 FIFO |
| IMU/AHRS | TIM7 软件 pend 的 InterruptExecutor，P3 | P1 当前仅固定32次整数替身，非 IMU/AHRS；外设 TIM7 不运行 |
| CAN 协议 | 拟 TIM6_DAC InterruptExecutor，P4 | 分片/服务处理有界，避免 SD 忙等待挤压 RX |
| USB / UART7 | OTG_FS、UART7 与独占 DMA，P6 | USB/TELEM1 双链路互不等待 |
| SD | SDMMC2 与其实际 DMA/IDMA，P7 | DMA 路径须按 HAL/芯片核对；FAT/写卡任务在低优先级 |
| 日志/参数/低速遥测 | 线程 executor | 有界 poll；任何等待都不持有高优先级路径需要的锁 |

Embassy 在同一个 executor 内是协作调度；同步忙循环会阻塞该 executor。高优先级采样使用独立 InterruptExecutor，仍须限制 PRIMASK 临界区。HAL 的 SD 命令/取消与 CAN abort 忙等须测量/有界化，不能靠外层 async timeout 假装抢占。

### RAM / cache 初值

| 区域 | M1 提案 | 验证 |
| --- | --- | --- |
| AXI SRAM 512 KiB | 普通/事件日志 136 KiB；任务 futures ≤64 KiB；AHRS/快照 ≤8 KiB；参数/文件系统/通信 ≤32 KiB；其他 ≤16 KiB；共享 CPU 栈 64 KiB | 合计上限 320 KiB，保留 ≥192 KiB；以 map/实际 ABI 核对 |
| D2 SRAM 288 KiB | DMA pool 64 KiB（SD staging 32 KiB、SPI/UART 与余量）；CAN 软件结构 ≤32 KiB | 具体 DMA 可达性先核对；剩余容量不自动视为可通用分配 |
| DTCM / SRAM4 / backup | M1 暂无必需分配 | 不把 DMA 数组默认移入 DTCM；RTC bootloader 签名按原约定处理 |
| flash | 应用仍为 0x08020000、1920 KiB 上限；M1 镜像目标 ≤1536 KiB | 含向量、代码、只读数据和 `.data` load image，打包器同步检查 |

DMA pool 采用经验证的 MPU non-cacheable 区或完整 clean/invalidate 规则，地址/长度按 32 B cache line 对齐，不共享含其他可变数据的 cache line。当前 `memory.x` 只使用 AXI，表中 D2 布局尚未实现；P1/对应外设阶段必须检查 map 和实际 DMA 路径。

Embassy task future 是静态存储，不是每任务独立线程栈；中断嵌套和函数临时量都计入共享 CPU 栈。64 KiB 栈初值要求实测至少剩余 32 KiB，并通过水位/保护区检查；不能从 `.bss` 大小推断栈安全。P1 的 AXI 实际布局为447 KiB static allocation、独立1 KiB HardFault栈、64 KiB共享栈；当前 static 使用/空余以 [P1报告](acceptance/p1/README.md)为准，并非表中M1资源已分配。

### P1 实际测量边界

`src/platform.rs` 配置 HSE/PLL1 480 MHz、HCLK240/PCLK120 MHz、LDO/Scale1+ODEN、I-cache开/D-cache关，assert与文本诊断检查PLL/分频/供电/flash等待；这些是配置核对，不证明物理频率。线程用 Embassy raw executor 与 measured idle，TIM7/P3跑周期替身，USB/P6通信。

替身使用绝对release，不追赶过期任务；分别记录 body deadline miss 与统计/快照越界跳过。body/IRQ cycle打点不覆盖整个ISR入栈/出栈及全部bookkeeping；报告观测分布，不称WCET，不替代DRDY外部打点。CPU为wall减PRIMASK下WFE bracket，包含MMIO/DSB等被减开销，tick误差上界和硬件开销分开；不能仅凭exit bookkeeping计数宣布全局屏蔽≤2 μs。

共享栈用 cortex-m-rt 0.7.7 paint-stack 与可抢占扫描/有限SP采样，无法证明所有瞬态峰值。HardFault裸入口先切独立应急栈再保存CRC故障、请求原bootloader；panic等路径仍先用共享栈，备份域断电/嵌套故障不保证恢复。详见 [P1操作单及测量限制](acceptance/p1/bring-up.md)、[静态故障审查](acceptance/p1/fault-review.md)。

## 5. 首轮稳定性测量目标

| 指标 | 初始目标 / 记录方式 |
| --- | --- |
| 板上单调时基 | 对比 host/仪器至少 60 s，漂移目标 ≤0.1%；记录观测窗口和测量误差 |
| IMU ODR | 1 kHz 起步；记录 1 s/全程样本率，偏差目标 ≤1%，实际器件时钟误差单列 |
| DRDY 打点抖动 | <10 us，需外部边沿/输入捕获对照；纯任务时间戳只能给内部延迟证据 |
| DRDY→样本发布 | 观测最大 ≤500 us；正常段零 FIFO 溢出/丢样/倒退时间戳 |
| AHRS | 每新样本更新；单次 IMU/AHRS 快路径 ≤300 us，快照年龄 ≤2 ms；正常段无 NaN/无效四元数 |
| 共享临界区 / logger producer | 每次全局屏蔽中断 ≤2 us；`try_record` ≤5 us；原目标不变，P1 masked-WFE等待未满足此项证明，进入采样前需调整idle并按原阈值实测，pending→服务延迟另记 |
| CAN | M1 预算 33.028%；60% 持续压力下正常段无 RX 溢出；零命令年龄/间隔目标 ≤5 ms |
| 日志 | 111062 B/s，含余量 138828 B/s；30 分钟正常段零 dropout，SD 指标见日志文档 |
| CPU / 栈 | 联合负载 CPU <50%；共享栈余量 ≥32 KiB，记录所有队列高水位 |

用 DWT cycles/微秒时钟记录分布、最大值、样本数、deadline miss；USB 输出低频汇总，原始统计进入 ULog。报告“观测最大值”，不称为已经证明的 WCET。过载/断线注入单列时间段，验证计数和恢复；不可混进正常统计以掩盖丢样。

这些目标用于找到硬件瓶颈；初次结果若不达标，先修复/调整再验收。非零电机停转阈值、闭环稳定性与求解时间等依据后续实测再规定。

## 6. QGC、MAVLink 与参数

**上游 QGroundControl 连接、Inspector 消息检查和参数页读/设是 M1 必验项。** 历史 q-MAG/MagGCS 连接不能替代。身份为 Generic，能力位只声明已支持的功能。

| 消息 | 首轮默认速率 / 条件 |
| --- | --- |
| HEARTBEAT、SYS_STATUS、SYSTEM_TIME | 1 Hz；健康位与电池/UTC 未知值真实填写 |
| ATTITUDE | 20 Hz；有效 AHRS；按请求可调至经预算允许的速率 |
| ATTITUDE_QUATERNION | 按需请求/设置速率；与 ATTITUDE 的总额度一起限制 |
| HIGHRES_IMU | 20 Hz；只标记实际更新的 gyro/accel/temperature 字段 |
| ESC_STATUS / ESC_INFO | 10 Hz / 1 Hz；四个真实 ESC 状态，正确处理 K→消息单位及未知值 |
| GPS_RAW_INT | 按收到的 GNSS，目标 5 Hz；GPS 无定位也如实表示 |
| AUTOPILOT_VERSION、PARAM_VALUE、STATUSTEXT | 请求/变化触发；队列/发送速率受限 |

USB 与 TELEM1 分别维护 MAVLink 序列号、RX 解析器、发送队列和 stream rate。首轮 TELEM1 候选 115200、8N1：线速上限 11520 B/s，发送总额先限 70%（8064 B/s），常规遥测目标 ≤6000 B/s，余量供参数回复；USB 常规限 32000 B/s。超过请求预算返回明确结果，不积压旧遥测。更高波特率根据实际接线/USB-UART 设备测试后采用。

参数契约：

1. 名称 ≤16 ASCII 且唯一；REAL32 对外，整数/枚举必须精确可逆；拒绝 NaN/Inf、越界、非整数枚举和只读写入。
2. `PARAM_REQUEST_LIST` 限速发送完整表，`param_count/index` 稳定；`PARAM_REQUEST_READ` 支持名称/索引。
3. `PARAM_SET` 校验后应用并回显 **实际值**；失败回显保留值并有原因。两条链路访问同一表，重复设置幂等，节点/总线配置按明确的重启/生效时机处理。
4. 当前值、待保存值与保存成功分开；持久化只有 disarmed 且输出关闭时允许。P3a 先只读识别/备份 FRAM 再决定布局；内部 flash 会受原 bootloader 应用擦除影响。
5. QGC 参数页加独立 pymavlink 脚本检查全表、按名/索引访问、合法/非法设置、双链路一致性、掉电后的实际保存结果。QGC 元数据和专有校准向导不是 M1 的参数协议前置。

## 7. 启动与实现顺序

先完成 [QGC 恢复往返](recovery.md) 和 P1 时基/诊断，再按 [plan](../plan.md) 接参数基础、CAN 零输出、SD、IMU/AHRS、TELEM1/QGC。可并行准备独立的 host 工作，硬件阶段依验收门推进。

M1 上电为 disarmed，RawCommand 四槽全零、ArmingStatus=0；设备重连不改变这一约束。AHRS/日志/通信异常可诊断，QGC 官方 PX4 恢复入口始终按 P1 验证的路径保留。
