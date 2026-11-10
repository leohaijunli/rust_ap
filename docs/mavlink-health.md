# EKF／传感器健康汇总

更新：2026-11-10。标准 MAVLink 2 `DEBUG_FLOAT_ARRAY`（ID **350**），system/component=1/1，`name=AP_HEALTH`、`array_id=1`。QGC 的 MAVLink Inspector 可直接解码，无需自定义方言；数组索引按下表读取。

## 消息固定字段

| 字段 | 含义 |
| --- | --- |
| `time_usec` | 汇总采样的启动相对时间，µs；不是 UTC |
| `name` | 整组数组的名称，固定为 `AP_HEALTH` |
| `array_id` | 本汇总的数组标识，固定为 1 |
| `data` | 58 个 float32；状态、位图、计数也以浮点值发送，按整数意义读取 |

## 速率与边界

- 默认 **1 Hz**，独立参数 `MAV_RATE_HEALTH`（0 停发，最高 2 Hz）；支持 REQUEST_MESSAGE(512)、SET/GET_MESSAGE_INTERVAL(511/510)。350 的正间隔最小 500000 µs，避免完整数组挤占 TELEM1；提高发送频率只重复最近汇总，不提高采样频率。
- Router 每约 1 s 用 **成功采样／接收计数增量 ÷ TIM8 实际窗口秒数**计算 Hz，两链路共用同一份缓存。它不是 ODR 配置、日志速率或 MAVLink 发送速率；窗口长度也随消息发送。
- EKF Hz 是 `Status.updates` 的延迟核心更新计数，不是 1 kHz 姿态发布。该计数经现有 50 ms 快照读取，窗口边界会有少量抖动。停用／不可用时 Hz 为 NaN，不能把 Mahony 健康当作 EKF2 健康。
- 首个窗口、计数重置、EKF epoch 切换或时间异常给 NaN；正常 u32 回绕可计算。没有新样本时实际 Hz 为 0，健康位另行撤销。没有可信时间戳、未使用字段也用 NaN，**不是伪零数据**。
- `time_usec` 是汇总采样的启动相对时间；缓存超过 2 s 不发。各健康位来自现有身份／错误／新鲜度规则，GNSS 接收健康不等于定位、已融合或 RTK。
- 不覆盖 INA219 的 `DEBUG_VECT`（ID **250**、`name=INA219`、x/y/z=V/A/W），也不改其速率、BATTERY_STATUS 或 ULog。旧文档中的 252 是编号笔误，原固件已使用正确的标准类型。
- 不涉及设参、校准、节点分配、日志启停、蜂鸣或解锁；数据健康不授予飞行／非零输出资格。

## `data[]` v1 索引

传感器顺序固定为 **gyro、accel、BMP388、板载 BMM150、CAN1 GNSS、INA219**；gyro/accel 共用成功 FIFO 样本计数。BMM150 的采样健康与选中／校准后的磁源健康分开，不把工厂 trim 当作机体安装校准。

下列显示名是文档／解码器标签，**不是新增的 MAVLink 字段名**；线上仍为 `data[索引]`。

| 索引 | 建议显示名 | 物理量／诊断含义 | 单位／值 |
| --- | --- | --- | --- |
| 0 | `schema` | 数据布局版本 | 当前 1 |
| 1 | `window_s` | 实测计数窗口长度 | s；首帧 NaN |
| 2 | `estimator_state` | 估计器实际状态 | 状态码，见下文 |
| 3 | `ekf_healthy` | 选中 EKF2、有新鲜姿态、Running、无运行时／数值／核心故障 | 0/1；不是飞行资格 |
| 4 | `ekf_update_hz` | 延迟 EKF 核心实际更新率，不是姿态发布频率 | Hz |
| 5 | `sensor_present_mask` | 已识别传感器位图 | 位图 |
| 6 | `sensor_healthy_mask` | 传感器健康位图，只含已识别且启用的源 | 位图 |
| 7 | `sensor_enabled_mask` | 已识别传感器的启用位图 | 位图 |
| 8 | `rate_valid_mask` | 更新率可计算的位图；NaN 不置位，实测 0 Hz 可置位 | 位图 |
| 9 | `gyro_hz` | 陀螺仪实际更新率 | Hz |
| 10 | `accel_hz` | 加速度计实际更新率 | Hz |
| 11 | `baro_hz` | BMP388 气压计实际更新率 | Hz |
| 12 | `mag_hz` | 板载 BMM150 地磁实际更新率 | Hz |
| 13 | `gnss_hz` | CAN1 GNSS Fix2 实际接收率 | Hz |
| 14 | `ina219_hz` | INA219 成功测量的实际更新率 | Hz |
| 15 | `gyro_age_ms` | 陀螺仪样本年龄 | ms |
| 16 | `accel_age_ms` | 加速度计样本年龄 | ms |
| 17 | `baro_age_ms` | BMP388 样本年龄 | ms |
| 18 | `mag_age_ms` | BMM150 样本年龄 | ms |
| 19 | `gnss_age_ms` | GNSS 接收样本年龄 | ms |
| 20 | `ina219_age_ms` | INA219 测量年龄 | ms |
| 21 | `ekf_step_max_us` | 当前 EKF epoch 的累计 step 耗时峰值，不是单个汇总窗口峰值 | µs |
| 22 | `ekf_deadline_misses` | 当前 EKF epoch 的累计超期次数 | 次 |
| 23 | `solution_status_flags` | 原估计快照的解算有效性标志；无快照 NaN | 位图 |
| 24 | `filter_fault_flags` | 原估计快照的 EKF 核心故障标志；无快照 NaN | 位图 |
| 25 | `gnss_fix_type` | 新鲜 GNSS 的标准 GPS_FIX_TYPE；不可用 NaN | 状态码 |
| 26 | `selected_mag_healthy` | 选中／校准后的磁源健康，不等于磁融合成功率 | 0/1 |
| 27 | `flight_qualified` | 飞行资格；数据健康不授予此许可 | 当前固定 0 |
| 28 | `cpu_load_pct` | 最近有效 CPU 窗口负载；不可用 NaN | % |
| 29..57 | 保留 | 未使用，不携带物理量 | NaN |

### 状态码与位图

`data[2]`：**0=停用，1=未激活／无新鲜姿态，2=EKF2，3=主动 Mahony，4=Mahony 回退**。

传感器位图与更新率有效位图使用不同的 bit 编号：

| 数据源 | `data[5/6/7]` 的 bit | `data[8]` 的 bit | 实际 Hz 索引 | 年龄索引 |
| --- | --- | --- | --- | --- |
| EKF | 不在传感器位图中 | 0 | 4 | — |
| 陀螺仪 | 0 | 1 | 9 | 15 |
| 加速度计 | 1 | 2 | 10 | 16 |
| BMP388 气压计 | 2 | 3 | 11 | 17 |
| BMM150 地磁 | 3 | 4 | 12 | 18 |
| GNSS | 4 | 5 | 13 | 19 |
| INA219 | 5 | 6 | 14 | 20 |

例如 `data[6]=15` 表示陀螺、加计、气压、BMM150 健康，GNSS/INA219 未标健康。`data[14]=0` 表示窗口内没有成功 INA219 采样，**不是电流为零**；未知／无法计算用 NaN。年龄没有可信时间戳时也为 NaN。

## 能否直接显示物理量名

标准消息只有一个 `data` 数组，元素没有独立字段名；`name=AP_HEALTH` 只能给整组数据命名。修改 Rust 变量名不会改变 QGC Inspector 的 `data[9]` 等显示。

- 保持标准消息：在额外的解码／监控界面按上表显示物理量名；现有 QGC Inspector 不会自动获得这些标签。
- 要求单条消息在 QGC Inspector 直接显示 `gyro_hz`、`baro_hz` 等具名字段：需要自定义 MAVLink 消息，并让飞控和 QGC 支持同一方言。

当前仅补充说明，未改变标准消息、字段布局或固件。

[本轮软件／烧录记录](acceptance/p2/20261110-health-position-flash.md)：板载 `77431b8dde7cae71` 已收到约 1 Hz 汇总。收到此消息不替代完整 E7/M1 或工程师对 QGC 界面的实物确认。
