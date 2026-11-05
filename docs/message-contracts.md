# PX4 / MAVLink 标准消息约定

更新：2026-11-04。工程师要求使用标准消息；范围与阶段门见 [plan.md](../plan.md)，软件结果见[记录](acceptance/messages/README.md) / [E6-H](acceptance/ekf2/20261104-e6-host.md)。

## 数据映射

PX4 定义固定为 v1.17.0 commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`。纯 Rust 类型位于 `src/core/px4_messages.rs`，ULog 按字段显式小端编码；Embassy 队列/快照负责分发。

| 数据源 | PX4 / ULog 消息 | MAVLink 标准消息 | 当前状态 |
| --- | --- | --- | --- |
| ICM-42688-P | `sensor_accel`、`sensor_gyro` | `HIGHRES_IMU` | 标准 ULog 已接入，实物随 P4 验证；MAVLink 已有有限 HIL |
| INA219 独立支路 | `rust_current_sensor`：记录/采样时间、V/A/W | `DEBUG_VECT`：`name=INA219`，`x=V`、`y=A`、`z=W` | SEN0291 I2C3，日志/遥测各 10 Hz；W=V×I，失败停记录/停发，见 P4p |
| 输出状态 | `actuator_armed` | `HEARTBEAT.base_mode` | 同一 `OUTPUT_STATE`；当前 disarmed、无预解锁/预检许可 |
| Mahony / EKF2 姿态、角速度 | `vehicle_attitude`、`vehicle_angular_velocity` | `ATTITUDE`、`ATTITUDE_QUATERNION` | EKF2 四元数/角速度 host 适配通过；实际任务仍待 P6a/E7 |
| EKF2 局部位置、速度 | `vehicle_local_position` | `LOCAL_POSITION_NED` | E6-H host 通过；任一位置/速度轴无效或非有限时 MAVLink 停发，日志保留无效位 |
| EKF2 全球位置 | `vehicle_global_position`（仅有效时） | `GLOBAL_POSITION_INT` | E6-H host 通过；须全球原点、对准与有效解，无 GPS 禁发 |
| 独立无 GPS / 健康诊断 | 独立 multi_id=1 的局部/姿态/`estimator_status` / `estimator_states` / aid topic | `ODOMETRY`、`ESTIMATOR_STATUS` | host 通过；水平无有效约束时方差无界，不当作第二套 GPS |
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
- EKF2 输出固定三组：姿态（`vehicle_attitude` / `ATTITUDE_QUATERNION`）、局部位置（`vehicle_local_position` / `LOCAL_POSITION_NED`）、全球位置（`vehicle_global_position` / `GLOBAL_POSITION_INT`，仅有效时）。姿态四元数 Hamilton `[w,x,y,z]`、FRD→NED；局部/全球位置为 NED，单位 m 与 m/s；三者共用同一 `EstimatorState` 快照，不各自重算或插值。日志与 MAVLink 复用同一字段映射，允许的差异只有消息自身的时间/字段宽度。

## 扩展与来源

### E6-H estimator 边界

- 类型与映射：`src/core/estimator/standard_output.rs`；独立 schema/编码：`standard_ulog.rs`；MAVLink：`src/core/mavlink_estimator.rs`。现有共享 `mavlink_core::encode` 复用同一 v2 serializer，**尚未启用 EKF 板载流**。
- 姿态/局部/全球使用最新 IMU `timestamp_sample`；states/status 使用延迟 EKF sample，`timestamp` 为发布微秒。MAVLink boot_ms 除 1000；`ESTIMATOR_STATUS.time_usec` 保留发布微秒，ODOMETRY 保留采样微秒。
- `[w,x,y,z]` 为 FRD→NED；位置/速度 m、m/s。`GLOBAL_POSITION_INT` 经纬度×1e7、高度×1000、速度×100 后按 PX4 截断，航向包角后 deg×100；没有有效 home 时 relative_alt 按 PX4 使用 AMSL。普通 repr_offset_q 为全 0。
- `eph/epv/evh/evv` 使用误差 P 索引 6+7 / 8 / 3+4 / 5；heading_var=P[2]，tilt_var=P[0]+P[1]。body 旋转方差按 `Rᵀ P_att R`；状态 25 与协方差 24 不混用索引。
- 无 GPS 的 xy/v_xy 始终无效且无全球参考。ODOMETRY frame/child_frame 均 LOCAL_NED、estimator_type=AUTOPILOT、quality=0；pose 对角下标 0/6/11/15/18/20、velocity 对角 0/6/11，其余保持 NaN。无效轴方差 `+INFINITY`，复位计数为 quat/velNE/velD/posNE/posD 之和（回绕）。
- 复位 delta/counter 原样映射；全球 altitude/terrain delta 按固定 `EKF2.cpp` 使用 `-delta_z`，不擅自改为 HAGL delta。heading_good_for_control 保留 yaw/mag alignment 与严格 >1 s 门；故障或 >100 ms 陈旧撤销有效输出。
- **未知量不填伪值**：angular xyz_derivative、ellipsoid altitude、模块 time_slip、status LPF test ratio / mag 检查量、aid observation/variance/filtered 值为 NaN；device_id 和未导出的 time_last_fuse 为 0。aid 的原始 innovation/variance/ratio 与 fused/rejected 保留核心值，estimator_instance 为实际 0/1。未实现的 LPF 不复制原始 ratio，pre_flt_fail_innov_* 保留未知初始 false，不能据此称预检通过。
- 12 个 estimator topic 的 main id=11–22、独立实例 id=23–34；不并入固件 TOPICS。完整 format 使用独立 2048 B 调用方缓冲；runtime Record 仍 800 B，队列记录上限仍 152 B。local/status/states payload 为 192/129/213 B，E7 必须先定日志预算和接入方式。

9 份消息的运行字段摘录位于 `tools/px4_msgs/{Vehicle*,Estimator*}.msg`；常量/注释不进入 payload。独立解析器比较完整字段顺序/类型/数组，整数时间/标志逐项相等，f64 经纬度绝对容差 1e-10°。固定上游源码核对与 hash 见 [E6-H 记录](acceptance/ekf2/20261104-e6-host.md)；CI 同时核对固定 checkout，不能仅通过换 topic 名称。

### E7 / BARO 当前接入状态

上述 E6-H 独立注册表的历史边界保持。E7 代码新增 `sensor_baro`（id=35，32 B payload）和 `rust_estimator_runtime`（id=36，44 B）；基础 13 个 topic 与 estimator id=11–34 不冲突。队列改为 512 槽、每记录最多 218 B，120 KiB 预算，64 槽事件保留；默认每实例 estimator=5 Hz、baro=10 Hz，真实日志/吞吐待验。

`SensorBaro.msg` 完整 6 字段保留压力 Pa、温度 °C、采样/发布时间与 error_count；设备 ID=0x677721（BMP388/I2C4/0x77）。HIGHRES_IMU 的 abs_pressure 为 hPa、pressure_alt 为标准气压高度、温度仍是 IMU；`SCALED_PRESSURE` 单独携带气压计温度 centi-°C，差压未知为 NaN。BMP388 首读已通过，标准流/日志实物校验尚待，见 [BARO 记录](acceptance/barometer/20261104-bringup.md)。

E7 task 尚未在板上启动；软件中的相对 yaw 独立 ODOMETRY 用 LOCAL_FRD，主/独立导航有效位保持假。设备 ID 来自真实驱动，`rust_estimator_runtime` 明示选择源/故障/epoch/未资格化；不能把 host NED 例程的假设直接当真北实物证据。

标准 topic 完整匹配固定 `.msg` 字段；附加诊断不混入其 schema。`dronecan_raw_command` / `dronecan_esc_status` / `dronecan_node_status` 记录原始协议，`rust_*` 记录本项目运行与持久状态；`esc_status` 名称留给真实 PX4 汇总结构。

新增数据源时再加入其标准定义、Rust 类型和实际发布路径。当前日志注册 10 个已接入 topic，新增 `rust_current_sensor`；旧 9-topic 软件和 `imu_icm42688` HIL 保留历史语义。

固定输入：`tools/px4_msgs/{SensorAccel,SensorGyro,ActuatorArmed}.msg` 来自上述 PX4 commit 的 `msg/`；随附上游 BSD-3-Clause [LICENSE](../tools/px4_msgs/LICENSE)。`tools/check_ulog.py` 用 pyulog 独立比较标准字段并读回 FRD 数值；来源见[台账](provenance.md)。
