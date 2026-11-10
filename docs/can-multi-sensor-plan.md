# CAN1/CAN2 多传感器：现状评估与实施方案

日期：2026-11-10。源码基线 `f2600f8`；板载版本与实测状态见 [TODO](../TODO.md)。**当前软件尚不能满足完整需求。本轮工程师选择先评估与方案，以下新增参数、消息和阶段均待实现。**

## 1. 已确认的需求

| 类型 | 当前启用的板载源 | 外接上限 | 每类总容量 |
| --- | --- | ---: | ---: |
| IMU（陀螺仪＋加速度计成对） | SPI2 ICM-42688-P，1 组 | 1 组 | 2 组 |
| 气压计 | BMP388，1 个 | 1 个 | 2 个 |
| 地磁 | BMM150，1 个 | 4 个模块 | 5 个 |

- 外接均为 **DroneCAN/UAVCAN v0**。每个模块可能在 **CAN1，也可能在 CAN2**；允许混合分布，上限按两条总线**合计**，不是每端口各一份。
- 两条总线都须具备原生传感器接收；每类独立配置首选源、自动选择健康备源。记录全部实例，并能看出配置首选、实际选中和 EKF 实际接纳的源。
- 板载 IMU 仍指当前启用的这一组；板上另外两颗 IMU 不属于本轮扩展。

## 2. 当前软件能做到什么

| 项目 | 已核实的现状 | 与需求的差距 |
| --- | --- | --- |
| IMU | 单 ICM FIFO → `SharedImu` → EKF/Mahony；估计状态的 IMU device ID 固定 | 无 CAN RawIMU 解码、第二实例、主源参数或输入源切换 |
| 气压 | BMP388 直接提交 Baro 量测；独立时间与失效判断 | 无 CAN 气压接入及选择层，不能直接把第二路量测混入同一偏置历史 |
| 地磁 | 四源选择器、`CAL_MAG0_*`～`CAL_MAG3_*`、健康/优先级/失败降级与磁历史重置；真实驱动仅 BMM150 | **四源包含板载，最多只剩三个外接位置**；CAN 解码仍缺失 |
| CAN1 | 原生接收 NodeStatus、ESC、GNSS Fix2/Auxiliary 和既有服务 | DSDL 白名单没有 RawIMU、磁场、气压/温度 |
| CAN2 | MAVCAN 双向透传，帧可到 DroneCAN GUI | 没有原生传感器解码；GUI 可见不等于进入估计器 |
| ULog | accel/gyro/baro 各单实例；mag 原始实例 0..3，另有选中磁源 | 缺第二 IMU/baro、第五 mag，以及跨类型的选择/失效记录 |
| MAVLink | `HIGHRES_IMU` 来自板载 IMU/baro＋选中 mag；`AP_HEALTH` 固定统计板载传感器、CAN1 GNSS、INA219 | 无完整逐实例状态/读数；不能用该汇总判断四个外接磁模块各自是否正常 |

依据：[IMU](../src/drivers/imu.rs)、[估计器入口](../src/drivers/estimator.rs)、[气压](../src/drivers/barometer.rs)、[磁选择](../src/core/magnetometer.rs)、[CAN1](../src/drivers/dronecan_can.rs)、[CAN2](../src/drivers/mavcan.rs)、[DSDL](../src/core/dronecan/dsdl.rs)、[参数](../src/core/parameters.rs)、[ULog](../src/core/ulog.rs)、[健康采样](../src/comms/telemetry.rs)。现有磁机制的软件/实物边界见[地磁说明](magnetometer.md)。

## 3. 接收、身份与时间

### 双 CAN 接收

```text
CAN1 硬件 RX ─→ CAN1 独立重组/节点状态 ─┐
CAN2 硬件 RX ─→ CAN2 独立重组/节点状态 ─┼→ 传感器适配 → 各类源表/校准/选择 → 主 EKF
             └→ MAVCAN 独立有界队列 ───→ USB GUI
板载 IMU/BMP388/BMM150 ────────────────┘              └→ ULog / 遥测快照
```

- 每个 FDCAN 保持一个 RX/TX 硬件拥有者。CAN2 原生接收常驻，MAVCAN 的 USB 会话、过滤、队列满或关闭不得停止原生接收。
- 每总线独立 `RxTransport` 与节点/温度缓存。当前重组键只有 source node＋消息/服务；把两条总线灌进同一个实例会混淆 CRC、toggle 和 transfer-ID。
- CAN2 原生发送与桥接共用有界仲裁；现有 `cancel_pending_tx()` 会取消该控制器全部待发帧，扩展时须按发送归属取消，不能让关闭 GUI 取消原生服务。不在 CAN1/CAN2 之间自动转发帧。
- 分别统计每总线 CRC/toggle/TID 缺口、重组超时、FIFO/软件队列丢帧和不支持类型；不支持类型不等于已识别传感器故障。

### 完整源身份

源键采用 **`传感器类型＋CAN 总线号＋node ID＋消息内 sensor_id`**；无 sensor_id 的类型使用约定的单实例值。板载源使用原有设备身份。CAN1/node 10 与 CAN2/node 10 必须是不同源，同一总线的节点 ID 必须唯一。

- `device_id` 不是硬件序列号；换 CAN 口或 node ID 后身份变化，原校准不能按“第几个到包”自动转绑。GetNodeInfo UID 可帮助工程师识别模块。
- 原始实例号、校准槽号、首选源与实际选中源分别表示，不能互相代替。运行期源表固定容量，板载位置有保留；超容量明确计数/报告，不覆盖在用源。
- 现有 ID 参数内部用 float32 存值、限制在 24 bit。保留这部分 ID 的总线/节点/类型语义；Mag2 的 sensor_id 用额外整数绑定字段保存，不能拼成任意 uint32 再转 float32 丢低位。
- 同节点同时发两种磁场消息时优先使用 Mag2；旧格式作为兼容路径，不能重复注册/计数为两颗磁传感器。无可靠身份对应时明确报告歧义。

### 标准消息契约

已核对项目固定 DSDL `b4653c7abc3c47cb31b16efa24ea755232774756`：

| 消息 ID / 类型 | 数据语义 | 适配要求 |
| --- | --- | --- |
| 1001 `ahrs.MagneticFieldStrength` | 三轴 Gauss；无 sensor_id/时间戳 | 按接收时间标记，兼容旧格式 |
| 1002 `ahrs.MagneticFieldStrength2` | sensor_id＋三轴 Gauss | 保存完整身份；不得忽略 sensor_id |
| 1028 `air_data.StaticPressure` | Pa、Pa² 方差 | 压力到包才产生新气压样本 |
| 1029 `air_data.StaticTemperature` | K、K² 方差 | 转为 °C；只关联同总线/节点的新鲜温度，缺失为未知 |
| 1003 `ahrs.RawIMU` | 总线共享时基的 timestamp；积分区间 s；最新 rad/s、m/s² 及积分 rad、m/s | 优先使用有效积分及其区间，不能再乘到包间隔；`integration_interval<=0` 表示仅瞬时样本有效 |

每份 DTO 保留源采样时间、本地采样时间/映射方式、接收时间、序列、身份、有效性与错误。**RawIMU 默认采用帧内 timestamp 和 integration_interval**；积分 dt 的计算不要求源端计数数值与 TIM8 相同，跨传感器融合时再核对共享时基或建立偏移/漂移映射。无可靠映射时可先接收监测，进入主 IMU 候选前完成映射与延迟验证。无时间戳的气压/磁场消息明确使用接收时间估计。

### 用系统 tick 差统一外接 IMU 时间，是否可行

工程师提出这一思路并指出 CAN 报文中有时间。RawIMU 确实带有上述时间字段，评估需区分**时间戳 t**与**积分时长 dt**，以及 tick 的采集位置：

| tick 取自哪里 | 结论 |
| --- | --- |
| 相邻 RawIMU 报文内部的采样 timestamp | **可行**：有效、连续的源时间差可用于瞬时样本积分；已有积分量仍配套使用 `integration_interval`，timestamp 差用于连续性核对 |
| 外接模块本地、实际积分窗口的起止边界 | **可行**：单调 tick 差除以真实计时频率作为 `integration_interval`，核对回绕与积分窗口覆盖 |
| 飞控两次 CAN 收包或任务处理时刻 | 可记录接收间隔，**不能直接替代已积分数据的 dt**，也不是采样时间戳 |
| 飞控收到的仅瞬时 gyro/accel、无积分量 | 可作接收时间近似积分方案，但须验证采样/传输抖动、缺帧和运动场景误差，合格后才作为主 IMU |

原因是 `Δ接收时间 = Δ采样时间 + Δ传输延迟`。例如源积分窗口均为 4 ms，交替延迟可能让到包间隔变成 6/2 ms；用这两个值解释原 4 ms 的积分量就会失真。丢包后也不能把最后一份积分扩展成整个缺失区间。

建议**本地时间戳统一为飞控 TIM8 单调时间**（有映射则用采样时刻，否则标为接收时间估计），**积分量配套使用源端实际 `integration_interval`**。有可信帧内时间时，无需采用收包间隔近似积分 dt；TIM8 负责接收超时和本地时间映射。累计积分时长与 TIM8 的长期差用于检查时钟频差。板载 FIFO 用本机批次时差的已有机制不能直接证明 CAN 收包时差等价；项目单调时钟实际用 TIM8，不要求改成 Cortex-M SysTick。

## 4. 主源参数与选择策略（拟定）

建议使用**稳定的配置槽**指定首选源，避免发现顺序变化；以下名字是本项目拟新增参数，不是当前可用参数或照搬的 PX4 参数：

| 参数 | 建议取值与含义 |
| --- | --- |
| `SENS_IMU_PRIM` | 0 自动；1/2 优先使用 `CAL_IMU0/1` 绑定的 IMU 对 |
| `SENS_BARO_PRIM` | 0 自动；1/2 优先使用 `CAL_BARO0/1` 绑定的气压计 |
| `SENS_MAG_PRIM` | 0 自动；1..5 优先使用 `CAL_MAG0..4` 绑定的磁源 |
| `CAL_IMUn_ID/PRIO/ROT`，n=0..1 | 设备绑定、优先级/禁用、安装旋转；gyro/accel 校准各自归属该源 |
| `CAL_BAROn_ID/PRIO/OFF`，n=0..1 | 设备绑定、优先级/禁用、压力偏移 Pa |
| `CAL_MAG4_*` | 第五组磁校准，复用现有 offset/软铁矩阵/旋转语义 |
| `CAL_MAGn_SID`，n=0..4 | Mag2 的 sensor_id 绑定；板载/旧格式采用约定值 0 |

首选参数默认 0；非零引用未绑定槽应拒绝。槽可以预先绑定暂时离线的设备，离线不会修改已保存的首选参数。PRIO=0 禁用该源参与选择，但其接收/原始监测仍可见；`-1` 采用默认优先级。磁源复用内部 50/外部 75，IMU/气压自动模式建议板载优先，外接通过首选或 PRIO 提升。

选择顺序为：**启用且数据合格 → 未处于失败降级的健康首选源 → 健康备源的有效优先级**。同级保留当前源，启动同级采用确定顺序。首选失效后切备源，继续报告“首选不可用/正在回退”；所有源失效则撤销有效性。失败源恢复采样后继续保留降级，沿用已有磁机制，通过改变优先级/重新绑定或重启释放降级，避免反复抢回。

健康包括数据有限/范围、数据年龄、序列和错误状态；IMU 另有时间/速率准入。两颗传感器不一致时不能仅凭差值判定哪颗正确：先记录同时间尺度、同坐标下的差异，保留估计器创新检查。首版不平均混用不同源，也不以多个同总线传感器宣称独立总线冗余。

参数复用唯一注册表、QGC 元数据与现有 disarmed/输出关闭的应用和持久化流程。新增项追加在现有 **228 项**之后；第五磁槽不能直接延长当前 `CalMag0Id + n*15` 的索引，因为后面已有 `MAV_RATE_HEALTH`。FRAM 双 1024 B 槽仍最多存 166 个非默认键，实施前核算完整校准＋既有设置，超限明确拒绝，另行设计有迁移依据的扩容才可改变布局。

## 5. 切换如何进入估计器

每类只输出一路选中数据，现有主 EKF2/Mahony 运行时消费这些数据；备用源持续接收和监测。

| 类型 | 必须处理的切换边界 |
| --- | --- |
| 地磁 | 复用已有延迟消费时重置磁偏置/历史；把完整身份和校准代次纳入重置键，通道满后仍保留重置意图 |
| 气压 | 增加选择层；清旧源待处理/延迟量测，按固定 PX4 参考处理气压偏置、参考高度及复位计数，验证不同零偏不会造成未报告高度跳变 |
| IMU | gyro/accel 成对切换；清旧源队列、积分/coning/时间基准，显式处理 gyro/accel 偏置与输出 reset；不足以连续恢复时撤销输出并重新初始化，不能拼接两颗 IMU 的积分 |

身份/选择代次随数据走到消费端；ULog `estimator_status` 的 device ID 必须对应被接纳的数据，不能仍硬编码 ICM，也不能仅用选择器“当前 ID”标记尚未消费的新源。Mahony 算法回退与 IMU 硬件备源切换是两件事，两者要分别报告。

**IMU 是主要实现难点**：当前 `ATTITUDE_FRESH_US=5000`，1 kHz 姿态发布不能把低速外接数据变成 1 kHz 新采样。外接主 IMU 必须在这一时间预算内满足采样/传输延迟和抖动要求；否则显示“接收健康、主源不合格”及原因。若工程师确需更低速主 IMU，需要另定预测/时延方案，不能直接放宽陈旧门。

## 6. ULog 与 MAVLink 监测

### ULog

- 完整标准原始 topic：`sensor_accel` / `sensor_gyro` 各实例 0..1，`sensor_baro` 0..1，`sensor_mag` 0..4。每条携带真实 device ID、独立时间和单位，实例内身份稳定。
- 选中磁源继续用 `vehicle_magnetometer`；选中 IMU 积分、选中气压及完整源键通过适配后的标准 topic 或必要的 `rust_*` 补充记录。只有完整满足固定 PX4 字段语义时才采用 `vehicle_imu` / `vehicle_air_data`；抽样日志不得称为全速可回放输入。
- 拟新增 `rust_sensor_status`：类型/实例、总线/node/sensor_id、校准槽、接收/有效更新计数、实际 Hz、年龄、启用/健康/准入、配置/有效优先级。`rust_sensor_selection` 记录首选与实际源、选择代次、切换原因/次数和相关复位；掉线时仍要有状态记录。
- `LOG_EN_IMU/BARO/MAG` 和各组 Hz 复用；**每实例独立限速**，选中流另有名额。地磁为五路原始＋一路选中，共六个限速位置；IMU/baro 也不能共用一个时间戳而互相挤掉。
- 样本失效不写伪零或重复旧样本；状态变化复用事件保留路径，周期状态走有界普通队列。SD 队列满只产生可见丢失，不消费/阻塞估计器数据。

### MAVLink

| 用途 | 方案 |
| --- | --- |
| 主数据流 | `HIGHRES_IMU` 使用实际选中 IMU/mag/baro；`SCALED_PRESSURE` 使用选中气压。各字段独立新鲜度/更新位，不假定同次采样 |
| 机体状态 | `SYS_STATUS` 按每类实际可用选中源汇总；`ESTIMATOR_STATUS` 报真实融合状态，不能用“接收健康”代替“已融合” |
| 现有汇总 | `DEBUG_FLOAT_ARRAY/AP_HEALTH` v1 保留既有索引含义，尤其板载 Hz 不悄悄改成外接 Hz；[现有协议](mavlink-health.md) |
| 逐实例诊断 | 拟新增标准 `DEBUG_FLOAT_ARRAY` 的 `SENS_I0/1`、`SENS_B0/1`、`SENS_M0..4`，独立 array_id；带 schema、完整源键、采样/接收年龄、Hz、读数、状态/优先级/错误、首选/选中/回退与切换代次 |
| 切换提示 | 有界 `STATUSTEXT` 提示类型、旧/新源及原因；完整历史在 ULog |

拟新增 `MAV_RATE_SDIAG` 控制逐实例诊断，0 停发；候选默认每实例 1 Hz，最终由带宽预算确定。USB/TELEM1 共用源快照和一次计数差分，各自调度，不能把遥测发包率当传感器更新率。首窗口/计数重置为未知，无新样本为 0 Hz。

**QGC 显示与调度需一起处理：**

- 已查 QGC 5.0.3 `_receiveMessage()`：按 sysid、compid、msgid 查找消息，**不会按 name/array_id 分行**。上述多实例 #350 与 AP_HEALTH 会在 Inspector 同一行交替；直接画该行会混源。
- 因此方案包含只读 `sensors` Console 总表，以及按完整源键解复用的监测工具（支持 MAVLink 连接或保存的 tlog），逐行显示 2/2/5 源和丢失/回退。工具复用现有 pymavlink 依赖；与 QGC 同时使用时走独立转发链路，不能争抢同一 USB 串口。新命令/工具本轮均未实现。
- #350 只有一个消息 ID，现有流表也要求组件内 ID 唯一。应实现一个 #350 复用调度器，`SET/GET_MESSAGE_INTERVAL` 按该链路的**总发包间隔**处理，不能给每个实例重复分配同一速率；`REQUEST_MESSAGE(350)` 仍返回 AP_HEALTH 保持既有约定，周期诊断由参数开启。覆盖降速时公平轮转并显示真实诊断刷新率。
- 当前 #350 最小间隔 500 ms 是单 AP_HEALTH 的限制；新增实例后须结合总预算调整，不能绕过限制额外偷发。保留 INA219 `DEBUG_VECT` 的独立含义。

## 7. 资源预算与必要验证

以下仅为设计估算，不能代替新组合实测：

- RawIMU 无 covariance 时经典 CAN 有效载荷 47 B，含传输 CRC 需 7 帧；36 元素 covariance 时 119 B、18 帧。以每帧约 160 bit 的保守预算、200 Hz 估算，占**所在一路** 1 Mbit/s 的约 22.4%/57.6%，还需叠加磁场/GNSS/ESC/服务。不能照搬板载 1 kHz 批次率到 CAN。
- 新增原始 ULog 在外接 IMU 200 Hz、baro 10 Hz、4 mag 各 20 Hz 时，按现有 payload＋5 B data record 头约增加 **23.57 kB/s**；另计状态/选择/积分 topic。按实际源率取上限，必要时单独配置外接 IMU 日志降采样。
- 9 路诊断各 1 Hz＋AP_HEALTH 1 Hz，按 #350 最长 264 B 无签名 MAVLink2 帧约 **2.64 kB/s**，占 TELEM1 115200/8N1 理论 11.52 kB/s 的约 23%；仍要计入既有流、参数与命令。
- 第二套 CAN 重组、2/2/5 源表、通道与快照须给出真实静态 RAM/入口帧增量，避免按最大 MAVLink union 给每路重复排队。

| 阶段 | 内容与通过条件 |
| --- | --- |
| MS0：冻结契约 | 核实外接模块型号/固件、实际发布类型/sensor_id/速率/时间戳/安装坐标；完成完整参数占用、CAN/SD/TELEM1/RAM 预算，确定外接 IMU 主源准入 |
| MS1：双 CAN 原生入口 | 分总线重组/状态，CAN2 原生与桥接并存；固定 DSDL 向量正常/坏长度/CRC/TID/超时通过。覆盖两条总线同 node ID、跨总线交错、多帧丢失，以及关闭桥接不影响原生 |
| MS2：五磁源闭环 | 板载＋4 外接满容量、校准/SID/首选/禁用/降级；覆盖同节点不同 SID、双格式去重，第五源可用、第六源明确拒绝，断线/恢复及满通道的重置意图不丢；原始/选中日志及逐源遥测可独立解析 |
| MS3：双气压闭环 | 独立温度/压力缓存，首选/失效/偏置切换，零偏不同的两源不会静默混合高度历史；日志/遥测与实际接纳源一致 |
| MS4：双 IMU 闭环 | 时间映射/积分/安装校准，gyro/accel 配对切换、序列缺口、偏置/reset、陈旧/不合格源拒绝；覆盖接收抖动不改源积分 dt、丢包不拉伸积分、时钟频差，校验积分不跨源、无旧输出冒充连续健康 |
| MS5：组合验收 | 复用前阶段 host 结果，做真实端口分布与一次切换/故障窗口，核对全部 ULog/MAVLink 身份和刷新率、队列/CPU/栈、IMU/EKF 时限；不是由 host 结果放行实物门 |

每个实现阶段按 [plan.md §4](../plan.md#4-仅保留必要测试) 做受影响 host、fmt/Clippy、元数据、普通 H753 release/打包/布局；协议/日志用固定 pydronecan、pyulog/pymavlink 独立核对。修改输入/中断调度后，在授权硬件窗口补一次真实 FPU 抢占/资源回归，复用已有无关数值结果，不重跑全量数学对拍。板载验证、M1/E7 及输出许可仍按原阶段依赖；保持纯 Rust/Embassy、原 bootloader、`0x08020000` 与 QGC 恢复能力。

**下一步：**工程师确认本方案的首选/恢复和监测方式，并提供 MS0 的模块输出资料；随后从 MS1→MS2 开始实现，气压和 IMU 接续。当前仅完成源码/协议评估与计划。

## 8. 固定参考

- [项目 DSDL/pydronecan 固定版本](../tools/p0/reference_vectors.py)。DSDL：[RawIMU](https://github.com/dronecan/DSDL/blob/b4653c7abc3c47cb31b16efa24ea755232774756/uavcan/equipment/ahrs/1003.RawIMU.uavcan)、[MagneticFieldStrength](https://github.com/dronecan/DSDL/blob/b4653c7abc3c47cb31b16efa24ea755232774756/uavcan/equipment/ahrs/1001.MagneticFieldStrength.uavcan)、[MagneticFieldStrength2](https://github.com/dronecan/DSDL/blob/b4653c7abc3c47cb31b16efa24ea755232774756/uavcan/equipment/ahrs/1002.MagneticFieldStrength2.uavcan)、[StaticPressure](https://github.com/dronecan/DSDL/blob/b4653c7abc3c47cb31b16efa24ea755232774756/uavcan/equipment/air_data/1028.StaticPressure.uavcan)、[StaticTemperature](https://github.com/dronecan/DSDL/blob/b4653c7abc3c47cb31b16efa24ea755232774756/uavcan/equipment/air_data/1029.StaticTemperature.uavcan)。
- [固定 PX4 v1.17.0 磁接收桥](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/drivers/uavcan/sensors/mag.cpp)按 node 查 channel，未使用 Mag2 sensor_id；本需求须显式补足总线/SID 区分，不能直接照抄节点键。
- 标准日志定义：[VehicleImu](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/msg/VehicleImu.msg)、[VehicleAirData](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/msg/VehicleAirData.msg)。
- [QGC 5.0.3 Inspector 接收/消息键](https://github.com/mavlink/qgroundcontrol/blob/v5.0.3/src/AnalyzeView/MAVLinkInspectorController.cc)、[当前 MAVLink 流表与间隔规则](../src/core/telemetry.rs)、[当前 IMU 时基](imu-timebase.md)。
