# 最小 QGC 身份与健康位补齐

日期：2026-11-07。工程师要求推进下一步；本轮仅实现数据展示兼容，不接入控制/解锁，不自动烧录。

## 实现与边界

- 新增纯核心 `mavlink_vehicle`，HEARTBEAT 改为 Quadrotor；PX4 身份、MANUAL 兼容编码、STANDBY 与现有输出状态来源保持。
- SYS_STATUS 映射 IMU gyro/accel、BMP388、GPS 接收通路和 AHRS 的 present/enabled/health；复用驱动身份/最新快照与估计输出，不新增缓存或参数。
- 六个普通/扩展掩码显式初始化为空，健康位不得超出已存在且启用的来源。磁力计/第二 IMU/控制/电机/RC/动力电池/预检通过位不虚构。
- IMU/baro/GPS 读取快照后再取参考时刻，避免抢占期间的新样本被旧查询时刻误判为“未来”。采样、寄存器、IRQ 与估计调度未变。
- GPS 接收健康不等于 fix/导航有效，AHRS 健康不等于可飞；无效定位继续停发。动力电池数值保留未知，INA219 支路不能替代动力电池保护。
- ARM/DISARM、模式与电机命令处理路径、固定输出关闭状态和原 bootloader/应用入口均未修改。
- [能力矩阵与精确定义](../../qgc-compatibility.md)。

## 必要软件检查

| 检查 | 结果 |
| --- | --- |
| `cargo test --locked --target x86_64-pc-windows-msvc --lib mavlink_vehicle::` | 7 passed；身份/能力、缺失/禁用/错误、年龄边界/时间超前/非有限值、MAVLink2 编码 |
| `cargo fmt --all -- --check` | 通过 |
| host `cargo clippy --locked --target x86_64-pc-windows-msvc --lib -- -D warnings` | 通过 |
| 普通 H753 `cargo clippy --locked --target thumbv7em-none-eabihf --bin v6x-rust-led -- -D warnings` | 通过；局部审视后重新核对驱动查询包装 |
| 普通 release/打包/布局/入口栈帧 | 通过；source=`fdf7fcd1ff9d10c7`，raw/padded=`1070336/1070336 B`，static RAM=`254224 B`，最大入口栈帧 `11496 B`、0 超限 |

普通配置 features=`[]`，board ID 53、应用入口 `0x08020000`、应用分区 `1966080 B`。相较上次回装包，raw 增加 1184 B，静态 RAM 与最大入口栈帧不变；这不是板载栈/实时性证明。

包：`target/thumbv7em-none-eabihf/release/v6x-rust-led.px4`；SHA-256=`f18e9365c7f9c28327f45a1b7a7fad6d433fdab5b604c0e039e89573cf62568c`。

首次测试 fixture 误用不存在的 `SensorGps::default`，改用既有 `process_fix2` 构造明确未知字段，未给生产 DTO 添加零默认值。随后 2 个测试暴露生成库默认掩码为首个标志（值 1）的真实问题，修复六个掩码初始化后 7 项全过；失败记录不改写为首次通过。

一次局部 `code-simplifier` 审视完成，仅精简名称限定和核对查询时序。复用不受影响的 180 项核心/EKF 对拍及历史板验，不重跑全特性或长时采集。

本地记录：`target/qgc-compat/20261107-minimal/`。新包未回装，QGC 版本/GUI 与实物核对待工程师；本次不放行 M1 或完整 E7。
