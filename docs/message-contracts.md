# PX4 / MAVLink 标准消息约定

更新：2026-11-02。工程师要求使用标准消息；范围与阶段门见 [plan.md](../plan.md)，软件结果见[记录](acceptance/messages/README.md)。

## 数据映射

PX4 定义固定为 v1.17.0 commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`。纯 Rust 类型位于 `src/core/px4_messages.rs`，ULog 按字段显式小端编码；Embassy 队列/快照负责分发。

| 数据源 | PX4 / ULog 消息 | MAVLink 标准消息 | 当前状态 |
| --- | --- | --- | --- |
| ICM-42688-P | `sensor_accel`、`sensor_gyro` | `HIGHRES_IMU` | 标准 ULog 已接入，实物随 P4 验证；MAVLink 已有有限 HIL |
| INA219 独立支路 | `rust_current_sensor`：记录/采样时间、V/A/W | `DEBUG_VECT`：`name=INA219`，`x=V`、`y=A`、`z=W` | SEN0291 I2C3，日志/遥测各 10 Hz；W=V×I，失败停记录/停发，见 P4p |
| 输出状态 | `actuator_armed` | `HEARTBEAT.base_mode` | 同一 `OUTPUT_STATE`；当前 disarmed、无预解锁/预检许可 |
| Mahony / EKF2 姿态、角速度 | `vehicle_attitude`、`vehicle_angular_velocity` | `ATTITUDE`、`ATTITUDE_QUATERNION` | P6a 接入真实估计输出；相对 yaw 单列健康元数据 |
| 四个 ESC | `esc_status` / `EscReport[8]` | `ESC_STATUS`、`ESC_INFO` | P4b-a 映射可信后由 P7 汇总；原始 DroneCAN Status 保留独立诊断名 |
| H-RTK GNSS | `sensor_gps`（PX4 同源别名 `vehicle_gps_position`） | `GPS_RAW_INT` | P4b-g 接真实 Fix2/Auxiliary；尚无融合位置 |
| 参数 / 日志管理 | ULog `P` 参数记录 + 持久状态诊断 | `PARAM_*`、`LOG_*`、`COMMAND_ACK`、`SERIAL_CONTROL` | 已接入标准协议，双链路/GUI 与异常门按既有阶段验收 |
| UTC / 系统状态 | RTC 与运行状态快照 | `SYSTEM_TIME`、`SYS_STATUS`、`AUTOPILOT_VERSION`、`STATUSTEXT` | 真实字段，未知值遵守协议 |

MAVLink 使用固定 rust-mavlink 0.17.1 的 `mavlink::common` 生成类型；USB/TELEM1 共用 `src/comms/mavlink_core.rs`。消息 ID、字段、CRC 和命令枚举均由该标准 dialect 提供。

## 字段语义

- `timestamp` 是本地发布时间，`timestamp_sample` 是本地采样时间，均为单调微秒；数据年龄由两者或读取时刻计算。UTC 单独走标准时间字段。
- INA219 调试消息 ID 252 使用标准 `DEBUG_VECT` 三个浮点槽：`x=bus_voltage_v`、`y=current_a`、`z=power_w`；`time_usec` 为转换完成的启动相对微秒。三个量来自同一快照，W=V×I，负电流/功率符号保留。请求 ID 252 返回同一消息；无新鲜数据时 Temporarily Rejected。
- 每个常流量数据源由 `LOG_EN_*`（0/1）+ `LOG_*_HZ`（速率）参数控制记录：IMU（`LOG_IMU_HZ` 默认 200）、支路电流（`LOG_INA_HZ` 默认 10）、DroneCAN（`LOG_CAN_HZ` 默认 400）、健康/统计（`LOG_HEALTH_HZ` 默认 10）；事件类（`actuator_armed`/`rust_parameter_change`/`dronecan_node_status`）只由 `LOG_EN_EVENT` 控制、不设速率。参数 NextCycle 生效，持久化走 P3a 存储。详情见 [logging-control](logging-control.md)。
- `rust_current_sensor` 直接记录同次解码测量：`timestamp` 为入队时刻，`timestamp_sample` 为转换完成时刻，`voltage_v/current_a/power_w` 为 SI 值；ULog payload 28 B、DATA 33 B，10 Hz。记录只在 Logger Ready 时接收，不依赖 MAVLink 连接。
- 标准 IMU 是 FRD：加速度 m/s²、角速率 rad/s、温度 °C。内部算法仍可用 FLU/ENU DTO，边界必须显式转换；姿态四元数为 Hamilton `[w,x,y,z]`、FRD→NED。
- 当前 IMU 每 40 个原始样本抽取 1 个，各标准 topic 200 Hz；`samples=1`。每条 payload 44 B、ULog DATA 49 B，两条合计 19.6 kB/s；没有声称 PX4 FIFO 平均算法等价。
- IMU `device_id=0x260012`：PX4 DeviceId 的 ICM42688P 类型 `0x26`、SPI bus 2、同型实例地址 0。`HIGHRES_IMU.id=0` 是 MAVLink 传感器实例号。
- `error_count` 为 SPI/FIFO 传输失败、解码失败和 DRDY 超时累计值的饱和和；初始化 FIFO flush 不算错误。`clip_counter` 只计所记录的一个样本，按 PX4 单样本发布的 99.9% 满量程阈值判定。
- 只有真实解码样本写标准 IMU；陈旧遥测停发。ESC 温度 K→°C、GNSS deg/mm→标准单位以及未知精度/航向须在各数据源阶段落实；不把未融合 GNSS 当作 `GLOBAL_POSITION_INT`。

## 扩展与来源

标准 topic 完整匹配固定 `.msg` 字段；附加诊断不混入其 schema。`dronecan_raw_command` / `dronecan_esc_status` / `dronecan_node_status` 记录原始协议，`rust_*` 记录本项目运行与持久状态；`esc_status` 名称留给真实 PX4 汇总结构。

新增数据源时再加入其标准定义、Rust 类型和实际发布路径。当前日志注册 10 个已接入 topic，新增 `rust_current_sensor`；旧 9-topic 软件和 `imu_icm42688` HIL 保留历史语义。

固定输入：`tools/px4_msgs/{SensorAccel,SensorGyro,ActuatorArmed}.msg` 来自上述 PX4 commit 的 `msg/`；随附上游 BSD-3-Clause [LICENSE](../tools/px4_msgs/LICENSE)。`tools/check_ulog.py` 用 pyulog 独立比较标准字段并读回 FRD 数值；来源见[台账](provenance.md)。
