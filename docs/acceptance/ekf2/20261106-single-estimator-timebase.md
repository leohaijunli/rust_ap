# 单实例估计器与 TIM8 积分

日期：2026-11-06。范围：工程师要求移除独立无 GNSS 第二实例，并采用 PX4 的 MCU 批次时间积分方案。

## 实现

- 删除 `NoGnssEkf`、第二份 Runtime/快照/欧拉角/配置与耗时诊断、component 197 的 ODOMETRY/ESTIMATOR_STATUS、ULog instance 1；保留主 EKF2 与 Mahony 选择/回退。
- 标准日志固定 `multi_id=0`，aid 的标准 `estimator_instance` 字段固定为 0；`rust_estimator_runtime` 删除第二实例故障字节，payload 为 51 B。
- 更新 host 标准输出示例、解析器与 CI 为单 GNSS 主实例；板载采集/配置工具对应单实例与诊断 schema 3。
- FIFO 整批输入、梯形均值、TIM8 批间隔积分及圆锥修正；首批建立历史，断序/失效后重建。时间戳使用 PX4 的 `16 × 32/30` 换算，保留原始诊断。来源、公式与调查限制见[时基说明](../../imu-timebase.md)。
- 时钟驱动切换为 Embassy 原生 `time-driver-tim8`；同步 TIM8_CC / TIM8_UP_TIM13 优先级、16 位空闲计时、平台断言/诊断和时钟判据回放地址。
- 修复新鲜 DRDY 的重复回扣，以及 FIFO/解码损失未显式断序的问题。

## 验证

- `cargo test --locked --target x86_64-pc-windows-msvc --lib`：**180 passed**。包含常量输入/读出抖动下 1 秒积分、FIFO 端点与圆锥修正、首批/断序/非法值/时间倒退/序号回绕/加速度 clipping，以及 DRDY 留样回推。
- 原有 10 秒多轴旋转 8 kHz 原始参考 / 约 1 kHz 前端 A/B 已改用 MCU 物理时间，按既定最大误差门限 **0.5° / 0.1 m/s / 0.1 m** 通过；不将旧版的具体误差数值写成本轮结果。
- `python -m unittest discover -s tools -p test_check_ekf2_standard.py`：**7 passed**。
- 初次收尾检查发现遗留的 ULog instance 1 消息 ID 断言及 component 197 目标断言，已同步为当前协议。Windows CRLF 曾改变嵌入 JSON 的 CRC；按既有生成器恢复 LF，并用 `.gitattributes` 固定该目录的精确字节，相关元数据用例通过。
- 新增换算用例曾把手册近似示例的 58.5 ticks 当成精确 1 ms，实际公式结果为 998.4 µs；已修正为 32 个 1 ms 间隔合计 1875 ticks，并按微秒输出契约检查。换算实现保持 PX4 的原系数。
- 上述首次检查通过后，工程师追加 TIM8 与 PX4 时间戳换算要求。复验与收尾实际结果：`cargo fmt --all -- --check` 通过；H753 Clippy（default / estimator-mahony / fault-injection / fault-dump）与 host `--example ekf2_standard` Clippy 全部通过；`tools/check_ekf2_standard.py` 单实例解析通过（26000 输入、1300 发布、165 字段）；`tools/build_firmware.ps1` 打包 source=`eb0d589438e0a6dc`，raw/padded=`1069744/1069760 B`，static RAM=`254256 B`，最大入口栈帧=`11496 B`（0 超限）。

## 板载验证（2026-11-06，工程师授权烧录）

固件 `eb0d589438e0a6dc`，board 53 / protocol v5，原 bootloader 上传 24 s（erase/program/verify 完成）；运行于 COM26。

- `!platform`：TIM8 `psc=239 arr=65535 cr1=1`，`registers_ok=true`，VTOR=`08020000`，备份域保留，fault_injection=false。
- 65 s 采集（`tools/ekf2_runtime_capture.py`，样本 `target/ekf2-runtime/20261106-tim8-run1`）：主 EKF2 约 5 s Mahony 对齐后选 **Filter**，`mode=Running`、`fault=None`、`config_errors=0`、`recoveries=0`。
- **计算性能**：`max_update_us=556`（历史旧包 1117 µs；预算 2000 µs），`main_step` max/p99=`561/448 µs`，`deadline_misses=0`；934 updates / 9334 samples / 207 measurements 稳定 10 ms 节奏。
- **1 kHz 发布**：`missed=0`，`max_late_us=67`，`max_age_us=1232`，repeated=420（≈0.08%）。
- **时钟/积分**：`integrated_dt_us=1000443` 与 `tim8_dt_us` 相同，`integration_error_ppm=0`；PX4 `32/30` 换算后 `sensor_dt_us=968158`、`error_ppm=-32270`，`raw_sensor_dt_us=907648`（旧 ×16 口径 ≈ 旧 −92887 ppm 同源）；`sensor_samples=8104`/窗口（实际 ODR ≈ 8.10 kHz）。积分 `dt` 锁定 TIM8，传感器时间残差 −3.23% 不进入积分，成因未定。
- **共享栈（工程师明确本轮不作门）**：基线 36688 B untouched，Filter 稳态观测最低 32144 B；仅记录。采集工具的栈门提前中止，估计器随后以 `!ekf2 stop` 停止。
- `!fault`：旧 `f509ec6b85b6a4fa` panic 记录原样保留，未被清除或改写。

- **USB 流核对（2026-11-06）**：15 s 采样，HIGHRES_IMU/SCALED_PRESSURE ≈22 Hz（参数 20 + 调度）、ATTITUDE/ATTITUDE_QUATERNION/ESTIMATOR_STATUS ≈9.67 Hz（参数 10）、心跳组 ≈1.13 Hz、GPS_RAW_INT 1.13 Hz 为 NO_GPS 兜底帧；`LOCAL_POSITION_NED`/`GLOBAL_POSITION_INT` 未发系导航未资格化按"无效不发"抑制（SD 侧仍记完整诊断快照），链路语义分工正确。USB 与 SD 成对证据齐备。

## 结论与依赖

- 单实例 EKF2 + TIM8 批积分在板上运行健康：0 期限超时、1 kHz 发布 0 丢失、积分 0 ppm；计算余量较旧包约 3.6×。
- **dropout 第 1 层板验**：`raw_command` 移除后（source=`8d1ae82d5f5d13b3`），65 s 采集 dropout 41→4 次（3 次 ≥65.5 ms 卡级停顿，u16 饱和）、产生 24.3 kB/s、无损坏；小突发型丢失清零，剩余归第 3 层卡选型（H-04）。缓解选项与第 2 层核查见[备选文档](../../logging-dropout-options.md)。
- **顺带修复历史 bug**：`record_can_stats` 误用 `Group::Health` 限流器，`rust_runtime_health` 被 10 Hz can_stats 饿死（三轮数据吻合：2 Hz / 0 / 仅前 0.9 s）。改回 `Group::Can` 后板验（source=`ad99705b817a8c7f`）：health 与 can_stats 并存（116/116 行 ≈10 Hz），12 s 会话 dropout=0、无损坏。
- 残余 −32270～−32444 ppm 传感器时间偏差仅影响诊断口径，不进入积分；归因需保留 FIFO tick 分布等更多证据，[时基说明](../../imu-timebase.md)继续保留该限制。
- 共享栈判据按工程师确认放宽（2026-11-06）：untouched ≥16 KiB（原 ≥32 KiB）。依据 PX4 v1.17.0 默认 WQ 栈预算（INS0/EKF2 6000 B，SPI/hp/uavcan/lp/tty/manager 合计 ≈15.3 KB，等效路径 ≈21.3 KB 拆在 ~7 个独立任务栈）；本项目同样内容压在单个 64 KiB 共享栈，实测 touched 33392 B 与 PX4 同量级略高（含 IRQ 嵌套）。新余量 ≈ 实测的 2×，也是 PX4 最大单 WQ 栈的 2.7×。对比表见[下一步计划](../../next-steps.md)。
- **按新判据的完整 65 s 采集通过**（source=`ad99705b817a8c7f`，`target/ekf2-runtime/20261106-tim8-run3-16kib-gate`）：`passed=true, issues=[]`；`Filter/Running`、fault=None、6784 updates、`deadline_misses=0`、`max_update_us=560`、`integration_error_ppm=0`（66 窗口）、1 kHz 发布 `missed=0`、栈最低 30728 B >16 KiB、CPU 28.7%<50%、PERIOD `deadline_misses=0`（P1 空载 10 µs release 门不再并入 E7 联合负载判定，1 kHz 合同由 `EKF2_PUBLISH missed=0` 与 `PERIOD deadline_misses=0` 承担，`p1_capture.analyze` 已参数化）。
- GNSS 实物融合、日志/双链路、M1/E7 资格仍按原依赖门，不放行。后续执行顺序见[下一步计划](../../next-steps.md)。
