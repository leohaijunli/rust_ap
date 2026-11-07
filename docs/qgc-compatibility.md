# QGC 最小兼容矩阵

更新：2026-11-07。仅 Pixhawk 6X Rev 3 / 四旋翼；保持 data-only、disarmed、输出关闭。本矩阵描述源码能力，不等同于完整 PX4 或可飞资格。

协议依据：固定 PX4 v1.17.0（`d6f12ad1c4f70ad3230afd7d86e971421e02fef4`）、锁定的 `mavlink 0.17.1` 所附 `common.xml`。QGC 5.0.3 已确认；本轮合并包的类型/说明界面由工程师确认，健康位和元数据由只读协议核对覆盖。[记录](acceptance/p2/20261107-qgc-flash.md)。完整双链路/实物门仍待验。

## 1. 身份与健康位

- HEARTBEAT：`MAV_TYPE_QUADROTOR`、`MAV_AUTOPILOT_PX4`、CUSTOM_MODE_ENABLED；`custom_mode` 仍为 PX4 MANUAL/sub_mode=0 的兼容上报，不代表已有手动控制器。
- AUTOPILOT_VERSION 仅声明 MAVLink2 与 PARAM_ENCODE_BYTEWISE，不新增任务、Offboard、通用 FTP 等能力位。
- 参数已补[设置说明与使用状态](parameter-settings.md)：全 165 项有说明，78 个 EKF2 配置键已读取、64 项未接入；原数值/范围不变，目录选项不保证全部已实现。
- SYS_STATUS 的普通/扩展 present、enabled、health 六个掩码显式从空值开始；生成代码的默认掩码会置首个标志，不能使用它表示“未知/没有”。
- `health ⊆ enabled ⊆ present`。物理设备身份未确认时不标存在；已确认身份但样本缺失/陈旧/失效时不标健康。AHRS 的 present 只表示内建软件能力。

| 状态位 | present / enabled 来源 | healthy 判据 |
| --- | --- | --- |
| 3D_GYRO / 3D_ACCEL | IMU 最近读到 WHO_AM_I=0x47；固定采样任务已运行 | 初始化成功、样本时间不超前且年龄≤2 ms、温度有限；陀螺/加计各自三轴有限 |
| ABSOLUTE_PRESSURE | BMP388 最近读到 chip_id=0x50；固定采样任务已运行 | 无当前驱动错误、样本年龄≤200 ms，压力有限且>0，温度有限 |
| GPS | 收到 CAN1 接受的 Fix2 所生成的 sensor_gps | 接收时间不超前且年龄≤1 s、fix_type 是已知接收状态、system_error=0；NO_FIX 也可表示接收通路正常，不代表有位置解 |
| AHRS | 软件存在；enabled 取实际估计启用请求 | 当前 epoch 的新鲜输出、有效四元数所派生的姿态与有限角速度；有效 Mahony 回退也可健康 |

磁力计、第二 IMU、RC、控制环、电机输出、动力电池、日志和预检通过位本轮不置位。INA219 是独立支路，不能用它伪报动力电池健康；SYS_STATUS 电压/电流/电量继续保留未知值。GPS/AHRS 健康不授予导航、解锁或控制资格。

实现：[纯核心映射](../src/core/mavlink_vehicle.rs)、[共享 MAVLink 核心](../src/comms/mavlink_core.rs)。USB 与 TELEM1 共用这些构建函数和已有流调度，不新增状态缓存或参数。

## 2. 能力矩阵

| 功能 | 软件行为 | 剩余边界 / 验收 |
| --- | --- | --- |
| 身份 / SYS_STATUS | 四旋翼 + PX4 身份；真实传感器状态；默认心跳组 1 Hz | 合并普通包烧录、类型界面与健康位协议核对通过；不代表可飞 |
| IMU / baro / attitude | 标准流；无效或陈旧不冒充新样本 | 方向、校准、实物失效恢复按原 P4/P6a/BARO 门 |
| GPS / 位置 | NO_GPS 兜底；无效 LOCAL/GLOBAL_POSITION 不发送 | 真实 GPS、授时与导航资格待原依赖门 |
| 电源 | INA219 DEBUG_VECT / BATTERY_STATUS 仅实测支路数据，电量未知 | 实物连接、参考负载和断线恢复待验；不是动力电池保护 |
| PARAM_* / FRAM | 165 项同源注册表、类型/范围校验、延迟保存及旧存档兼容 | 当前版本双链路设读、保存/重启等原实物门待验 |
| REQUEST_MESSAGE / 510 / 511 | 支持已登记消息与每链路间隔；无新鲜数据 TEMPORARILY_REJECTED，未知请求 UNSUPPORTED | 双链路覆盖隔离/重连待验，不重写已实现的调度 |
| 组件元数据 | 397→COMPONENT_METADATA；限定文件的只读 MAVFTP | CRC/字节/165 项与工程师参数界面确认通过；不声明通用 FTP 能力 |
| 日志 / Console | 既有 LOG_* 列表/下载与 logger 控制、诊断查询 | QGC GUI、SD 停顿/故障等原门未收口；不自动启动日志 |
| ARM/DISARM / DO_SET_MODE | COMMAND_LONG/INT 返回 UNSUPPORTED，不动作 | 完整控制入口归 P5；SET_MODE 消息当前不执行 |
| 校准 / 电机测试命令 | 未实现，命令返回 UNSUPPORTED，不动作 | 不以界面可见为台架或首飞授权 |
| RC_CHANNELS / SERVO_OUTPUT_RAW | 无已接入真实来源，不发送 | RC/PWM/CAN 语义不能用占位数据混淆 |
| 任务 / Offboard | 未实现，不执行任务或控制目标 | 稳定控制/导航与安全门之后单独实施 |
| MAVCAN | 既有 CAN2 桥接、过滤/租约与输出关闭门 | 历史板验复用；不是 CAN1 ESC/GNSS 实物门通过 |
| MAVLink 签名 | 消耗标准签名尾并校验 CRC | 未实现签名认证，不宣称安全认证通信 |

## 3. 本轮必要核对

- host：四旋翼/未解锁身份、能力位、缺失/失效/禁用掩码、IMU/baro/GPS 年龄与非法值，以及 MAVLink2 编码。
- 普通配置 fmt/Clippy、release 打包、向量/分区及入口栈帧检查；没有相关算法变化，不重跑 EKF 黄金回放或全特性压测。
- 经工程师明确授权烧录后，一次 QGC 连接核对四旋翼类型与健康位；仍拒绝 ARM/模式动作，不改参数、不清历史故障。实际版本和结果再记录，不预写 GUI 通过。
