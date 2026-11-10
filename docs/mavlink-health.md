# EKF／传感器健康汇总

更新：2026-11-10。标准 MAVLink 2 `DEBUG_FLOAT_ARRAY`（ID **350**），system/component=1/1，`name=AP_HEALTH`、`array_id=1`。QGC 的 MAVLink Inspector 可直接解码，无需自定义方言；数组索引按下表读取。

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

| 索引 | 含义 |
| --- | --- |
| 0 | schema=1 |
| 1 | 实测计数窗口，s；首帧 NaN |
| 2 | 估计器状态：0 停用、1 未激活／无新鲜姿态、2 EKF2、3 主动 Mahony、4 Mahony 回退 |
| 3 | EKF2 健康，0/1：选中 EKF2、有新鲜姿态、Running、无运行时／数值／核心故障 |
| 4 | 延迟 EKF 核心实际更新率，Hz |
| 5 / 6 / 7 | 传感器 present / healthy / enabled 位图，bit 0..5 对应上述顺序；位图健康只包含已识别且启用的源 |
| 8 | Hz 有效位图：bit 0=EKF，bit 1..6=上述六个传感器；NaN 不置位，实测 0 Hz 可置位 |
| 9 / 10 | gyro / accel 实际 Hz |
| 11 / 12 / 13 / 14 | BMP388 / BMM150 / GNSS Fix2 / INA219 实际 Hz |
| 15..20 | 对应六个传感器的样本年龄，ms；无可信时间戳为 NaN |
| 21 / 22 | 当前 EKF epoch 的累计 step 峰值（µs）／deadline_misses（计数） |
| 23 / 24 | 原估计快照的 solution_status_flags / filter_fault_flags；无快照 NaN |
| 25 | 新鲜 GNSS 的标准 fix_type；不可用 NaN |
| 26 | 选中磁源健康，0/1；不等于磁融合成功率 |
| 27 | flight_qualified=0，保持未资格化 |
| 28 | 最近有效 CPU 窗口负载，%；不可用 NaN |
| 29..57 | 保留，NaN |

例如 `data[6]=31` 表示 bit 0..4 健康、INA219 不健康；并不表示电流为零。`data[14]=0` 表示该窗口没有成功 INA219 采样，原电流消息仍按自己的新鲜度规则停发。

软件／烧录结果另记到 `docs/acceptance/`；收到此消息不替代完整 E7/M1 或工程师对 QGC 界面的实物确认。
