# 板载地磁驱动与 PX4 磁源区分

本轮：工程师要求接入板载地磁到滤波器。只实现本板 Rev 3 的 **BMM150**，不扩大为外接驱动/冗余投票或 QGC 校准工程；硬件验收尚未完成。

## 本项目数据链

`I²C4 → BMM150 trim/ADC 补偿 → FRD/Gauss 快照 → Measurement::Mag → 主 EKF2`

- 依据固定 PX4 v1.17.0：I²C4 PF14/PF15、地址 `0x10`、PG8 共享电源、驱动 rotation=0、20 Hz 高精度模式。身份、自检、配置回读和周期配置检查；超时/溢出/无数据撤销有效性，500 ms 后重试，不切断 BMP388 电源。
- BMM150 与 BMP388 在同一线程执行器上共享 `RefCell<I2c>`；每次有界阻塞事务结束即释放借用，不跨 `await`，不把总线引用送入中断执行器。保留现有 I²C4 10 ms 超时路径，不使用无法访问 AXI 的 BDMA。
- `device_id=0x431021`（十进制 4395041），独立采样时间/序号/错误计数；单位为 Gauss，不是 Tesla 或 µT。BMM150 无温度通道，`sensor_mag.temperature=NaN`，不借用气压计温度。
- 一次有效采样通过现有有界 aid 通道送滤波器，独立于 logger 开关。`EKF2_MAG_TYPE`、磁场质量/创新门与估计器启用状态仍控制实际融合，不强制修改参数。
- 日志：标准 `sensor_mag` / `vehicle_magnetometer`，`LOG_EN_MAG=1`、`LOG_MAG_HZ=20` 默认，仅为记录上限。两条记录最多约 1.66 kB/s，最大记录不超过既有 218 B 槽位；原 topic ID 不重排。
- 两链路共享 `HIGHRES_IMU` 磁场 XYZ（新鲜时置磁场更新位）；失效为 NaN/清位。`SYS_STATUS` 按实际 ID/样本报告磁传感器健康。`!mag` 查询原始源；`!ekf2` 的 `EKF2_MAG` 查询 device ID、融合/拒绝状态。

**边界**：只应用芯片工厂 trim，尚未做机体安装后的硬/软铁校准；`vehicle_magnetometer.calibration_count=0`，板坐标与机体坐标暂取相同方向。不能以驱动读数、健康位或合成融合证明绝对航向/飞行资格。

## PX4 如何区分板载与外接

以下依据同一固定版本，不是本项目已实现的全部功能：

| 机制 | PX4 行为 |
| --- | --- |
| 设备身份 | `device_id` 编码芯片类型、总线类型、总线号和地址；同型号在不同总线仍是不同 ID。不是芯片唯一序列号，换接口可能改变 ID |
| 内/外部属性 | 从板级总线描述判断。FMUv6X I²C1/2/3 是外部，I²C4 内部；DroneCAN 磁源视为外部。固定版本的 `SensorMag.msg` 没有 `is_external` 字段 |
| 校准归属 | `CAL_MAGn_ID` 按 ID 匹配校准槽；`n` 不等于固定的“板载/外接编号”。每颗磁传感器有各自 offset/scale |
| 方向 | 板载跟随板安装方向；外接使用自身 `CAL_MAGn_ROT` 或自定义安装角。不能把不同方向的原始 XYZ 直接混在一起 |
| 单磁源选择 | 先校准到机体 FRD，再按 `CAL_MAGn_PRIO` 与健康投票选择；默认内部 50、外部 75，0 禁用。外接失效/陈旧不应覆盖健康板载；存在多磁源发布模式 |
| EKF 输入 | 通常订阅已选择的 `vehicle_magnetometer`，不直接识别 I²C 芯片。device ID 或校准计数改变时重置磁偏置学习；`EKF2_MAG_TYPE` 是融合模式，不是传感器选择参数 |

当前 Rust 版本只发送板载 BMM150；没有伪造 `CAL_MAG*` 多源选择已完成。后续接外部磁源时，需独立 ID/安装校准、健康选择和切换时偏置/延迟缓冲重置，不按“最后收到的数据”覆盖。

## 来源与许可

固定 commit：`d6f12ad1c4f70ad3230afd7d86e971421e02fef4`。补偿/寄存器移植遵循 BSD-3-Clause，版权保留在 Rust 模块与生成的 host 参考中；许可证见 `tools/px4_msgs/LICENSE`。

- [Rev 3 启动定义](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/boards/px4/fmu-v6x/init/rc.board_sensors)、[总线描述](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/boards/px4/fmu-v6x/src/i2c.cpp)
- [BMM150 驱动/补偿](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/drivers/magnetometer/bosch/bmm150/BMM150.cpp)、[寄存器](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/drivers/magnetometer/bosch/bmm150/Bosch_BMM150_registers.hpp)
- [校准与优先级](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/lib/sensor_calibration/Magnetometer.hpp)、[健康选择](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/sensors/vehicle_magnetometer/VehicleMagnetometer.cpp)、[EKF 磁源切换](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/ekf2/EKF2.cpp)
