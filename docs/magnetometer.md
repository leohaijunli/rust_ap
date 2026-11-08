# 板载地磁驱动与 PX4 磁源区分

更新：2026-11-08。板载 **BMM150** 驱动之后，已按工程师要求实现四磁源身份/校准/优先级与健康选择、切换重置。当前真实驱动仍只有 BMM150；外接驱动、校准拟合及硬件/航向资格尚未完成。

## 本项目数据链

`真实驱动 → sensor_mag（各源）→ ID 绑定校准 → 健康/优先级选择 → vehicle_magnetometer → 主 EKF2`

- 依据固定 PX4 v1.17.0：I²C4 PF14/PF15、地址 `0x10`、PG8 共享电源、驱动 rotation=0、20 Hz 高精度模式。身份、自检、配置回读和周期配置检查；超时/溢出/无数据撤销有效性，500 ms 后重试，不切断 BMP388 电源。
- BMM150 与 BMP388 在同一线程执行器上共享 `RefCell<I2c>`；每次有界阻塞事务结束即释放借用，不跨 `await`，不把总线引用送入中断执行器。保留现有 I²C4 10 ms 超时路径，不使用无法访问 AXI 的 BDMA。
- `device_id=0x431021`（十进制 4395041），独立采样时间/序号/错误计数；单位为 Gauss，不是 Tesla 或 µT。BMM150 无温度通道，`sensor_mag.temperature=NaN`，不借用气压计温度。
- 各源经现有有界入口发布，只有选中的机体 FRD 数据送现有 aid 通道，独立于 logger 开关。`EKF2_MAG_TYPE`、磁场质量/创新门与估计器启用状态仍控制实际融合，不强制修改参数。
- 日志：`sensor_mag` 原始实例 0..3 和单一 `vehicle_magnetometer`；`LOG_EN_MAG` / `LOG_MAG_HZ` 控制五条独立限速流，避免一个源挤掉另一个。单 BMM150 最多约 1.66 kB/s，四原始源+选中源最多约 4.36 kB/s；记录仍适配 218 B 槽，旧 ID 不重排。
- 两链路共享 `HIGHRES_IMU` 的**选中磁源** XYZ；失效为 NaN/清位。`SYS_STATUS` 的磁健康依赖可用选中数据。`!mag` 增加 `MAG_SELECT` / `MAG_SOURCE`，可区分原始实例、设备 ID、校准绑定、配置/有效优先级和健康；`!ekf2` 查询实际融合/拒绝状态。

**边界**：工厂 trim 与机体校准分开。未绑定校准时采用零偏移/单位软铁矩阵，不自动写 `CAL_MAG*`。绑定参数并不证明完成实物校准；不能以驱动读数、健康位或合成融合证明绝对航向/飞行资格。

## PX4 如何区分板载与外接

下列关键机制已在单主 EKF 的 Rust 数据链实现；不是整个 PX4 sensors/EKF2 模块移植：

| 机制 | PX4 行为 |
| --- | --- |
| 设备身份 | `device_id` 编码芯片类型、总线类型、总线号和地址；同型号在不同总线仍是不同 ID。不是芯片唯一序列号，换接口可能改变 ID |
| 内/外部属性 | 从板级总线描述判断。FMUv6X I²C1/2/3 是外部，I²C4 内部；DroneCAN 磁源视为外部。固定版本的 `SensorMag.msg` 没有 `is_external` 字段 |
| 校准归属 | `CAL_MAGn_ID` 按 ID 匹配校准槽；`n` 不等于固定的“板载/外接编号”。每颗磁传感器有各自 offset/scale |
| 方向 | 板载跟随板安装方向；外接使用自身 `CAL_MAGn_ROT` 或自定义安装角。不能把不同方向的原始 XYZ 直接混在一起 |
| 单磁源选择 | 先校准到机体 FRD，再按 `CAL_MAGn_PRIO` 与健康投票选择；默认内部 50、外部 75，0 禁用。外接失效/陈旧不应覆盖健康板载；存在多磁源发布模式 |
| EKF 输入 | 通常订阅已选择的 `vehicle_magnetometer`，不直接识别 I²C 芯片。device ID 或校准计数改变时重置磁偏置学习；`EKF2_MAG_TYPE` 是融合模式，不是传感器选择参数 |

## 当前参数与选择规则

- 四组 `CAL_MAG0_*`～`CAL_MAG3_*`：`ID`、`PRIO`、`X/Y/ZOFF`、`X/Y/ZSCALE`、`X/Y/ZODIAG`、`ROT`、`ROLL/PITCH/YAW`。在原 167 项后追加，新目录 **227 项**，ID 匹配校准槽，不按原始实例编号匹配。
- 校准公式 `body = R * S * (raw - offset)`：OFF 为 Gauss；S 为对称正定软铁矩阵，ODIAG 的 X/Y/Z 对应 XY/XZ/YZ。重复非零 ID、非法 ID/旋转或非正定组合拒绝，不更改原值。
- `PRIO=-1` 使用内部 50/外部 75；0 禁用。未绑定源仍以默认优先级参与，但标为未绑定校准。**解绑 ID 不等于禁用源**；禁用要保持实际 ID 并置 PRIO=0。
- 外接 `ROT` 支持固定 PX4 0..40、100 自定义；41..99 拒绝。非零 ROLL/PITCH/YAW 自动采用自定义 ZYX 旋转。内部跟随项目当前与 IMU 一致的板/机体 rotation=0，不单独旋转；尚未扩展 `SENS_BOARD_ROT` 的全传感器安装配置。
- 健康参考固定 DataValidator：300 ms 超时、100 次错误密度窗口、累计错误 >10000、相等值阈值 1000（按轴累积）；常规置信度阈值 0.9、优先级比较的 1% 置信度差规则。额外拒绝非有限/零磁场、未来/重复/过期采样；修改校准不清除驱动故障。
- 外接失效后回退健康源；失败源有效优先级降到 1，避免反复抢回。新鲜数据可恢复健康，但需改变配置优先级/重新绑定或重启来释放降级。日志/诊断同时保留配置和有效优先级。
- 设备/校准或选择代次变化，以 reset 元数据进入 EKF。通道满或 runtime 拒绝时不丢重置意图；清除的仅是旧磁 pending/延迟样本，磁偏置、协方差和 LPF 在新的延迟样本被消费时重置，不重启 IMU/气压或整个滤波器。
- 真实驱动通过 `crate::magnetometer::publish(SensorMag)` / `invalidate(device_id)` 接入。没有报告的外接设备不注册为可用。原始日志实例按实物报告注册顺序分配，与校准槽编号无关。

## 保存与验收边界

仍用原 FRAM 双 1024 B 槽，最多保存 **166 个非默认键**；超容量明确返回 Capacity，不写部分参数、不移动存档布局。没有自动校准拟合、自动存入设备 ID、功率/温度磁补偿或多 EKF 发布模式。外接实物切换与安装校准需另验；软件结果见[区分机制记录](acceptance/magnetometer/20261108-selection.md)。

## 来源与许可

固定 commit：`d6f12ad1c4f70ad3230afd7d86e971421e02fef4`。补偿/寄存器移植遵循 BSD-3-Clause，版权保留在 Rust 模块与生成的 host 参考中；许可证见 `tools/px4_msgs/LICENSE`。

- [Rev 3 启动定义](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/boards/px4/fmu-v6x/init/rc.board_sensors)、[总线描述](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/boards/px4/fmu-v6x/src/i2c.cpp)
- [BMM150 驱动/补偿](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/drivers/magnetometer/bosch/bmm150/BMM150.cpp)、[寄存器](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/drivers/magnetometer/bosch/bmm150/Bosch_BMM150_registers.hpp)
- [校准与优先级](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/lib/sensor_calibration/Magnetometer.hpp)、[健康选择](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/sensors/vehicle_magnetometer/VehicleMagnetometer.cpp)、[EKF 磁源切换](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/ekf2/EKF2.cpp)
