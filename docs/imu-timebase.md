# IMU 时间基准：对齐 PX4

更新：2026-11-06。工程师确认采用 PX4 的积分时基方案，并切换 TIM8、统一 FIFO 时间戳换算。

## 当前实现

`ICM-42688-P FIFO → 整批队列 → 梯形均值 → TIM8 dt 积分 → Runtime → EKF2 / Mahony`

- 默认 8 kHz、每批 8 个样本。估计输入队列改为 2 个完整批次，仍容纳 16 个原始样本；原始样本的遥测、200 Hz 日志分发照常工作。
- 批次时间来自新鲜 DRDY 中断捕获的 `Instant::now()`，即 TIM8。中断缺失/陈旧时使用读取开始时间，并回扣留在 FIFO 中的样本周期。
- **新鲜 DRDY 已对应本批末样本，不能再因读取时多出一个样本而扣 125 µs。** 已修正旧代码这处重复回推。
- 批内按 PX4 的等间隔梯形均值汇总，`mean = (0.5 × (上一批末值 + 本批末值) + 本批其余值之和) / N`。批内回推的 125 µs 时间戳不参与积分。
- 相邻批次的 `dt = (本批 TIM8 时间 − 上批 TIM8 时间) × 1e-6`；再按 `VehicleIMU / Integrator` 做梯形积分。每批输出一个积分，陀螺保留 `IntegratorConing` 的跨批圆锥修正；EKF 内部降采样/延迟缓冲继续处理这些 `ImuDelta`。
- FIFO 时间戳采用与 PX4 相同的 `16.f × (32.f / 30.f)` 比例，约 **17.0667 µs/tick**。这用于传感器时间诊断，实际积分依赖 MCU 批间隔。
- 首批只建立时间与数值历史，不用启动时间或传感器 tick 编造 `dt`。无历史的首批均值左端使用首个真实样本。
- 非有限值、陀螺饱和、序号断裂、时间倒退或不在 `2..=5000 µs` 的批间隔使前端失效并清空积分历史。下一批重新建立基准，不跨丢失区间补积分。加速度 clipping 保留为积分标志。
- 每个读取包都占用序号，包括解码失败的包；FIFO/传输失败与重初始化也制造序号断点。损坏批次整体不交给估计器，满队列丢弃积压批次，丢弃计数仍按原始样本数报告。

这里采用固定约 1 kHz 的单批积分路径；PX4 `VehicleIMU` 可依据 `IMU_INTEG_RATE` 聚合多批。本项目的首批和丢样重建规则是显式的有界输入策略，不声明整个 PX4 模块逐位等价。

### TIM8 平台配置

- 使用锁定的 `embassy-stm32 0.5.0` 原生 `time-driver-tim8`，由 HAL 占用 TIM8 并实现 `embassy_time`。
- APB2 定时器时钟 240 MHz，`PSC=239`、`ARR=65535`，计数分辨率 1 µs；`TIM8_CC` 和 `TIM8_UP_TIM13` 两个中断均设 P1。
- Embassy 使用 CC1 半回绕、CC2 闹钟；PX4 的 HRT 使用 CC3。保留 Embassy 的时间驱动实现和回绕竞争处理，MCU 定时器及积分时间语义与 PX4 对齐。
- 空闲计时改读 TIM8 CNT 的 16 位字段，平台断言、`!platform` 的 TIMER 行与编译后时钟判据回放地址同步。原 bootloader、`0x08020000` 入口和 QGC 恢复契约保留。

### TIM2 原生 32 位驱动评估（已记录，暂不执行）

工程师询问后评估，2026-11-06 结论：

- **精度无可测提升**：分辨率同为 1 µs（APB1/APB2 定时器时钟均 240 MHz、PSC=239）；积分 `dt` 已是 64 位微秒且板载 `integration_error_ppm=0`，dt 精度瓶颈在 DRDY 捕获延迟/读取回退，与计数器宽度无关。
- **性能收益不可见**：时间维护中断从 ≈30.5 次/秒（16 位半溢出）降到 ≈0.00003 次/秒（2³² µs ≈ 71.6 min），每秒仅省几十 µs（<0.01% CPU）。
- **真实收益是鲁棒性**：消除 16 位 `calc_now` 奇偶竞态类（要求 ISR 延迟 <32.768 ms，当前余量约 1000×）；32 位闹钟编程简化；空闲 WFE 不再依赖半溢出唤醒。
- **成本**：自写 embassy time driver（TIM2 空闲可用）+ 重写 `diagnostics::idle()`/CPU 忙闲统计（失去周期性唤醒后按闹钟实际唤醒计账）+ 平台断言/诊断/时钟判据回放同步 + 一轮 P1 时基复验与重新烧录。
- **决定**：维持 TIM8。如需收口该评估的三项可测口径（时间维护中断计数、idle 统计连续性、`integration_error_ppm` 保持 0），再按独立任务排期；见 `TODO.md` 可选项。

## PX4 的完整链路

固定版本为 v1.17.0 / `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`：

| 层 | 源码事实 |
| --- | --- |
| [FMUv6X board_config.h](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/boards/px4/fmu-v6x/src/board_config.h#L314-L319) | `HRT_TIMER=8`、channel 3；本项目原为 TIM2，本轮已切到 TIM8。 |
| [hrt.c](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/platforms/nuttx/src/px4/stm/stm32_common/hrt/hrt.c#L440-L444) | 计数器预分频到 1 MHz。分辨率不等于振荡器的实际精度。 |
| [ICM42688P.cpp](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/drivers/imu/invensense/icm42688p/ICM42688P.cpp) | `DataReady` 捕获 `hrt_absolute_time()`；`RunImpl` 优先 DRDY，缺失时用读取时间。`ProcessGyro/Accel` 还计算 FIFO 消息的 `dt`。 |
| [ICM42688P.hpp](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/drivers/imu/invensense/icm42688p/ICM42688P.hpp#L73-L77) | 无外部时钟时 FIFO 换算为 `tick × 16 × 32/30`；`TMST_CONFIG=0x1d` 与本项目相同。 |
| [rc.board_sensors](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/boards/px4/fmu-v6x/init/rc.board_sensors#L100-L106) | 此板启动 ICM42688P 时未指定外部 CLKIN 频率。 |
| [PX4Gyroscope.cpp](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/lib/drivers/gyroscope/PX4Gyroscope.cpp) / [PX4Accelerometer.cpp](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/lib/drivers/accelerometer/PX4Accelerometer.cpp) | `updateFIFO` 发布等间隔梯形均值和批次 `timestamp_sample`；生成 `sensor_gyro/accel` 时没有把 FIFO `dt` 传入积分器。 |
| [VehicleIMU.cpp](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/sensors/vehicle_imu/VehicleIMU.cpp) / [Integrator.hpp](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/sensors/Integrator.hpp) | `UpdateGyro/Accel` 用相邻 `timestamp_sample` 之差积分，发布 `vehicle_imu.delta_angle_dt / delta_velocity_dt`。 |
| [EKF2.cpp](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/ekf2/EKF2.cpp#L753-L760) | 接收这些积分时间，并比较样本经过时间与累计积分时间，输出 `estimator_status.time_slip`（秒）。 |

因此，先前“PX4 也直接用 FIFO tick 积分”的说明不准确；只看驱动中的 `gyro.dt` 会漏掉后续 `VehicleIMU` 链路。

## `-92887 ppm` 的调查结论

这是旧固件 `f5756b75d6ed78bf` 的历史读数，计算的是 `Σ(tick × 16)` 与 TIM2 样本时间跨度的差，并非已隔离测出的晶振频偏。

1. **已确认换算缺项**：TDK [DS-000347 v1.6 §12.7](https://product.tdk.com/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/ds-000347-icm-42688-p-v1.6.pdf) 的补充说明要求无 RTC 输入时乘 `32/30`。其 1 kHz 示例指出 FIFO 值在 58/59 间变化；按公式精确计算，平均应为 58.59375 ticks，32 个间隔合计 `1875 × 16 × 32/30 = 32000 µs`。PX4 使用同一系数。
2. **缺项不能解释全部差异**：`(1 − 92887/1e6) × 32/30 − 1 = −0.0324128`，仍约 **−32413 ppm（−3.24%）**。这只是对历史值的离线换算，不是 PX4 实测值，也不是修改后的板载结果。
3. **已确认输入路径差异**：旧版使用 tick 积分、DRDY 多余回扣、FIFO/解码丢失未必体现在序号中。新实现使用 TIM8 批间隔，并让丢失显式触发重新建立基准。
4. **尚不能归因的部分**：缺少同一窗口的 FIFO tick 分布、实际样本数、FIFO reset/reject 与完整批次时间序列，不能断言剩余 3.24% 就是量化、丢样或 MCU 时钟偏差。恒定批次时间偏移通常只影响窗口端点，不能单独解释持续 9.29% 比例差。

## 新诊断与“PX4 差多少”

**2026-11-06 板载实测**（`eb0d589438e0a6dc`）：`integration_error_ppm=0`（积分 `dt` 与 TIM8 完全一致）；PX4 `32/30` 换算后 `sensor_dt_us=968158` vs `tim8_dt_us=1000443` → `error_ppm=-32270`；`raw_sensor_dt_us=907648` 与历史 −92887 ppm 同口径。即缺 `32/30` 换算解释了大部分历史偏差，残余 −3.23% 为传感器时间戳自身偏差，不影响积分，成因未定。实际样本 8104/窗口（≈8.10 kHz）。

`!ekf2` 使用 `EKF2_RUNTIME schema=3`，`EKF2_CLOCK` 在连续约 1 秒窗口输出：

- `sensor_samples`：本窗口真实输入样本数，可计算实际 ODR 和平均 tick。
- `raw_sensor_dt_us`：旧 `tick × 16` 口径，保留用于追溯历史读数。
- `sensor_dt_us / error_ppm`：采用 PX4 的 `tick × 16 × 32/30` 换算后与 TIM8 比较。累计 tick 在窗口末换算并四舍五入为整数微秒，避免逐样本截断累积误差。
- `integrated_dt_us / integration_error_ppm`：实际发出的积分 `dt` 累计与 `tim8_dt_us` 比较。采集工具的 100 ppm 一致性检查改看这一项；传感器时间偏差仍完整输出。

**PX4 没有一个适用于所有板卡的固定 IMU/MCU ppm 差值。** 对连续、无缺失的积分区间，基于相邻 hrt 时间差的总和应等于该区间 hrt 时间跨度，仅有浮点/微秒舍入等误差；这不证明 IMU 振荡器与 MCU 振荡器频率相同。PX4 的实际积分偏移可从 `estimator_status.time_slip` 看，实际采样率可从 `vehicle_imu_status.gyro_raw_rate_hz / accel_raw_rate_hz` 看；原始 FIFO 时钟与 hrt 的差需另外记录 FIFO `dt/samples` 才能量化。

本轮环境只枚举到系统 COM1 与 Intel AMT COM3，没有飞控端口，未取得新的 PX4 或 Rust 板载测量。软件结果见[本轮记录](acceptance/ekf2/20261106-single-estimator-timebase.md)；下一步由工程师连接飞控后，复用一次连续采集核对配置、样本/丢失计数、积分时间、发布时序和共享栈。
