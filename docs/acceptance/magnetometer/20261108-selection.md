# 磁源区分机制：软件交付

日期：2026-11-08。工程师追加实现 PX4 磁源区分机制。全程未烧录、未写实物参数/存档、未清故障、未解锁；**不代表外接磁源或板载磁融合实物验收**。

## 实现

- `src/core/magnetometer.rs`：固定四源/四校准槽，按 DeviceId 的芯片/总线/地址区分，板级描述决定内外；CAL_MAGn_ID 匹配校准而非原始实例编号。默认内部 50/外部 75，0 禁用，结合置信度选择；故障后有效优先级降到 1。
- 校准：正定软铁矩阵和 Gauss 偏移，外接支持固定 PX4 0..40 旋转及自定义 Euler；内部保持本项目与 IMU 一致的 rotation=0。重复绑定/非法矩阵/旋转拒绝。新增 60 项，原 167 项顺序不变，目录 227 项；原 FRAM 双 1024 B 槽不移动，超 166 个非默认键返回 Capacity。
- `src/drivers/bmm150.rs` 保留器件驱动；`src/drivers/magnetometer.rs` 单独拥有源选择。真实驱动走同一原始报告/失效入口，不把未接入外接设备标为可用。修改校准不能清除驱动故障或复活过期样本。
- runtime 只接选中的 `vehicle_magnetometer`；设备/校准/选择代次变化的重置意图在满通道/拒绝后保留。只清磁 pending 和延迟缓冲，磁 LPF/偏置/协方差在新延迟样本消费时重置，不重置 IMU、气压或整个 EKF。
- 标准原始 `sensor_mag` 实例 0..3、选中 `vehicle_magnetometer` 与 `HIGHRES_IMU`，五条独立日志限速流；`!mag` 查询配置/有效优先级、内外、绑定、选中源、故障/切换计数。

机制、参数使用和来源见[地磁说明](../../magnetometer.md)。基于固定 PX4 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` 的 DeviceId、板级 I²C/SPI 描述、Magnetometer 校准、DataValidator/Group 和 controlMagFusion；BSD-3-Clause 版权/许可证沿用模块与 `tools/px4_msgs/LICENSE`。

## 必要结果

| 检查 | 实际结果 |
| --- | --- |
| host 核心测试 | **209 passed**；含 ID/校准/优先级/失效/禁用/陈旧/回退降级、四源容量、重置保留与延迟消费、非法配置保持原值、存档容量 |
| pyulog 双源合成日志 | 2 个原始实例、4 条选中记录；外接旋转/偏移、校准计数变化、回退 ID 与独立采样时间正确，`file_corruption=false` |
| fmt / Clippy | fmt、host lib/受影响示例及普通 H753 `-D warnings` 通过 |
| 元数据 | 4 项生成器用例与目录 `--check` 通过，227 项完整覆盖；旧条目保持 |
| 普通包 | source=`2c58e1faa86f4f72`；raw/padded 1,192,792/1,192,800 B，RAM 374,336 B（余量 83,392 B），最大入口帧 11,496 B、oversized=0；board ID 53 / `0x08020000` 保持 |

本地证据：`target/magnetometer-selection/{host-tests.log,selection.ulg,report.json}`；普通包/资源/栈报告在 `target/thumbv7em-none-eabihf/release/`。板载 BMM150 补偿数学没有修改，复用前轮 256 组固定对拍，不重做同一验证。

初次检查有一处测试常量名错误及格式/Clippy 嵌套条件问题，修正后通过；局部审视又修正了“配置清除驱动失效”和“采样过期但发布时间新”的边界，并新增回归用例。没有据此虚构硬件结果。

## 边界与下一步

- 实现单磁源出口，不移植 uORB/NuttX、全传感器安装配置、多个 EKF、自动校准拟合、自动写 ID 或功率/温度补偿；未声称完整 PX4 sensors 等价性。
- 参数限于有限范围与正定矩阵；比 PX4 直接加载任意校准更严格。保留错误密度/置信度核心判据，未增加 RMS/不一致性诊断。绑定只表明有参数，不等于实物校准完成。
- 外接驱动仍未接入，双源与切换结果是明确标注的合成测试。另获回装授权后先核对板载 BMM150/气压共存与实际磁融合，再由工程师提供外接型号/接口/方向做真实失效切换；M1/E7/飞行资格不提前放行。
