# 启动 yaw 收敛：Rust 与固定 PX4 参考的合成输入对照

日期：2026-11-09。范围：工程师报告上电后 yaw 收敛慢，要求比较本项目与 PX4 原生固件的地磁数据、yaw 收敛时间与精度，并解释原因。工程师当前只有飞控本体、无外接设备，因此本轮只做宿主软件对照，不刷写、不换固件、不设参/保存、不启停日志、不启用输出。

**边界**：输入为合成 IMU/磁数据，没有真实航向基准，也不是同板 PX4 原生固件实测。所有“误差”只相对于合成场景的已知 60° 航向，不能用于宣称绝对航向精度、安装校准或飞行资格。

## 方法

- Rust 侧使用仓库现有宿主例程 `examples/ekf2_input.rs`（`--output`）；PX4 侧使用固定参考 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` 的 `tools/golden_cpp` 同源构建，二者读同一份 CSV。
- 场景 A“水平静止”：机体水平、真实磁航向 60°、EKF 初始 yaw=0；IMU 250 Hz、baro/mag 20 Hz、60 s、无噪声。
- 场景 B“陀螺偏置+噪声”：在 A 上增加固定陀螺偏置 z=0.3 °/s、x=y=0.1 °/s；陀螺噪声 0.002 rad/s、加速度计 0.03 m/s²、磁 0.003 G（seed 7）。
- PX4 四个变体：
  1. 默认：不喂系统标志，等价于当前仓库 golden 用法；
  2. 地面标志：按 IMU 频率喂 `at_rest=true, in_air=false`；
  3. 重力融合：同一固定源码树加 `CONFIG_EKF2_GRAVITY_FUSION` 重新编译；
  4. 重力融合+地面标志。
- 系统标志只存在于实验副本 `target/startup-20261109/cpp/harness.cpp`；PX4 自己的 `setSystemFlagData` 仍做半拍延迟和限速。仓库 `tools/golden_cpp`、既有 CMake/manifest/baseline 均未修改。
- 收敛时间定义：yaw 误差首次进入 ±1° 并保持至少 1 s；30–60 s 窗口统计稳态误差。

## 结果

| 场景 | 输入 | tilt_align | mag/yaw_align | 30–60 s 均值误差 | 窗口最大误差 | 末值误差 |
| --- | --- | --- | --- | --- | --- | --- |
| A | Rust | 7.032 s | 8.104 s | 0.000° | 0.000° | 0.000° |
| A | PX4 默认 | 7.032 s | 8.104 s | 0.000° | 0.000° | 0.000° |
| A | PX4 地面标志 | 0.576 s | 1.648 s | 0.000° | 0.000° | 0.000° |
| B | Rust | 7.032 s | 8.104 s | +10.204° | 14.077° | +14.077° |
| B | PX4 默认 | 7.032 s | 8.104 s | +10.204° | 14.077° | +14.077° |
| B | PX4 地面标志 | 0.576 s | 1.648 s | +0.092° | 0.187° | +0.100° |

- 场景 A 的 Rust 与 PX4 默认：15000 行中 `control_status` 不一致 0 行，yaw 最大差 **6×10⁻⁹°**，四元数分量最大差 **5.382×10⁻⁹**。场景 B 分别为 0 行、**3.6×10⁻⁴°**、**2.504×10⁻⁶**；差异来自同一输入的浮点实现顺序，量级可忽略。
- 地面标志使 tilt 对齐从 7.032 s 降到 0.576 s，yaw/mag 锁定从 8.104 s 降到 1.648 s；两组场景缩小时刻完全相同。
- 重力融合不改变收敛时间；场景 B 中反而使窗口漂移由 7.684° 变为 8.865°、末值误差由 +14.077° 变为 +13.979°。重力融合不是启动慢的原因，也不是修复项。

## 原因分析

1. Rust 估计器没有 `vehicle_land_detected`/`at_rest` 输入，也没有 ZeroVelocityUpdate（ZVU）、ZeroGyroUpdate（ZGU）。`control` 初值和 `previous_control` 都固定带 `IN_AIR`（`src/core/estimator/imu_ekf.rs:28、326、345`），第 1241–1242 行注释也明确“没有 at_rest 系统标志来启用 zero gyro/velocity”。
2. PX4 地面通路：系统标志经 `control.cpp:52–67` 调 `set_vehicle_at_rest`，随后每拍执行 ZVU（`control.cpp:157`）和受 `EKF2_IMU_CTRL` 控制的 ZGU（`control.cpp:159–160`）。
3. tilt 对齐完成条件是 tilt 方差降到 3° 等效方差以下（`control.cpp:71–76`）。启动阶段尚未 tilt 对齐时，ZVU 的继续条件成立（`ZeroVelocityUpdate.cpp:54–64`），零速度观测快速收紧水平速度/姿态协方差，因此 tilt 方差在 0.576 s 就达标，而纯惯导路径需要 7.032 s。
4. 磁融合启动还要求 `tilt_align`、`mag_counter>3` 和至少 1 s 磁场健康时间（`mag_control.cpp:161–172、559`；`ekf.h:622`）。两组场景在 tilt 对齐后都多约 1.07 s，说明这 1 s 健康窗口与 at_rest 无关；8.104 s 与 1.648 s 的差异由 tilt 对齐主导，不是地磁融合算法本身慢。
5. ZGU 在静止时每 200 ms 把陀螺读数直接作为零角速率观测融合到陀螺偏置（`ZeroGyroUpdate.cpp:49–77`）。场景 B 的 0.3 °/s z 偏置因此被持续估掉；没有这条通路时偏置在 60 s 内积分出约 14° 的 yaw 漂移。
6. 因此本轮对照把差异定位为“缺少地面/静止系统标志通路”，而不是磁量测、磁检查门限或重力融合；Rust 与 PX4 在相同输入、相同控制位下核心数值一致。

## 结论与限制

- 当前固件在地面启动时走的是“无 at_rest”的慢路径，与 PX4 默认（不喂系统标志）一致；原生 PX4 之所以快，是 `vehicle_land_detected` 提供了 at_rest 并启用 ZVU/ZGU。
- 本场景中若接入保守的静止判定和 ZVU/ZGU，预期可把 yaw 锁定从约 8.1 s 缩短到约 1.6 s，并显著降低陀螺偏置引起的启动漂移；但这是合成输入外推，尚无板上真实航向或原生固件同板实测，不能当作已实现或已验收。
- 本轮未修改任何固件、参数、磁检查门限或融合计数器，也未把合成结论写成绝对精度。

## 复现与产物

- Rust 例程：`cargo build --release --example ekf2_input --target x86_64-pc-windows-msvc`，运行 `ekf2_input.exe imu.csv rust.csv aids.csv --output`。
- 生成、汇总、对拍脚本和两场景 CSV 在 `target/startup-20261109/`（`generate.py`、`metrics.py`、`compare.py`、`cpp/harness.cpp`、`cpp_gravity/` 等）。该目录被 `.gitignore` 忽略，仅作本轮实验档案，不入库；仓库既有 golden 参考与基线未改。
- PX4 变体在 WSL 中用固定源码树重新编译；地面标志 CSV 由同一 IMU 时间戳逐行生成，`timestamp_us,at_rest,in_air`。
- 本轮只新增文档；未运行会改变固件或板载状态的命令，未刷写、未采集、未设参。

## 后续实现：保守静止检测 + ZVU/ZGU（同日）

按工程师确认实现“保守静止检测 + ZeroVelocityUpdate/ZeroGyroUpdate”，纯软件，不刷写。要点：

- 新增 `src/core/estimator/at_rest.rs`：陀螺模长 ≤ 0.05 rad/s、比力模长与重力差 ≤ 1.0 m/s²、外部许可保持连续 1 s 才置静止；任一条件破坏立即释放。外部许可 = `!armed && !outputs_enabled`（`src/drivers/estimator.rs`，当前产品常量下恒为真，保留门控以备未来接入）。
- `ImuEkf` 新增 `VEHICLE_AT_REST`（1<<26，对齐 PX4 位）并在每个延迟历元更新；`control_fusion` 在零创新航向更新后按 PX4 顺序执行 ZVU（200 ms 间隔、两历元均静止、tilt 对齐后观测方差 0.2²，否则 0.001²）与 ZGU（受 `EKF2_IMU_CTRL & 1` 控制、累计 ≥0.2 s、观测方差 `EKF2_GYR_NOISE²`），均复用 `fuse_direct` 并置 `constrain_variances`。静止时加速度计偏置视为可观测（对齐 `updateIMUBiasInhibit`）。
- 宿主例程新增 `--at-rest` 许可开关；静止检测使用延迟 EKF 同源样本，检测置位比 PX4 的实时 land detector 晚约 1.2 s（1 s 保持 + 延迟路径），这是保守设计的选择。
- 单元测试：at_rest 检测 4 个、`imu_ekf` 集成 3 个（许可/运动控制位、ZGU 只在许可时学偏置、ZVU 消除水平速度）；`cargo test --target x86_64-pc-windows-msvc --lib` 225 通过 0 失败。host/H753 fmt、Clippy `-D warnings`、release example 与固件构建（`build_firmware.ps1`）全部通过，栈预算 oversized=0。

### 实现后宿主对照（同一场景 CSV 复用，Rust 带 `--at-rest` 重跑）

| 场景 | 输入 | tilt_align | mag/yaw_align | 1° 内收敛 | 30–60 s 均值 | 窗口最大 | 末值 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| A | Rust（上轮，无 at_rest） | 7.032 s | 8.104 s | 8.104 s | 0.000° | 0.000° | 0.000° |
| A | Rust（本轮 at_rest） | 1.384 s | 2.448 s | 2.448 s | 0.000° | 0.000° | 0.000° |
| A | PX4 地面标志（参考） | 0.576 s | 1.648 s | 1.648 s | 0.000° | 0.000° | 0.000° |
| B | Rust（上轮，无 at_rest） | 7.032 s | 8.104 s | 8.104 s | +10.204° | 14.077° | +14.077° |
| B | Rust（本轮 at_rest） | 1.384 s | 2.448 s | 2.448 s | +0.138° | 0.222° | +0.130° |
| B | PX4 地面标志（参考） | 0.576 s | 1.648 s | 1.648 s | +0.092° | 0.187° | +0.100° |

- at_rest 位首次置位于 1.16 s（1 s 保持 + 延迟路径），其后 14711/15000 行保持静止；上轮 dump 无该位。
- 场景 B 末值误差 +14.077° → +0.130°（-98.7%），与 PX4 地面标志参考同量级（+0.100°）；窗口均值 +0.138° vs +0.092°，差异来自静止判定起始更晚（ZGU 学习偏置晚约 0.8 s）。
- 收敛仍比 PX4 地面标志慢 0.8 s（2.448 vs 1.648 s）：PX4 实验变体由外部直接喂 `at_rest=true`，Rust 的保守检测需要自身确认静止。这是设计取舍，不是缺陷。
- 本轮 `compare.py` 逐样本 `control_status` 不再可比：Rust 与 PX4 的位布局本就不同（Rust 自有 1<<35/1<<37 等位），且 at_rest 引入后两侧控制序列按各自时序推进；有效对照是时间线与误差指标。
- 上轮“Rust 与 PX4 默认在相同控制位下核心数值一致”的对拍结论不受本轮影响：默认路径（无 `--at-rest`）代码行为未变，既有 218 项测试与本次新增 7 项均通过。

### 边界

- 仍是合成输入对照：真实 IMU 噪声、振动、磁干扰与安装误差未覆盖，不能当作绝对航向精度、磁校准或飞行资格。
- 板上验证待工程师授权刷写后进行（如 60 s 静止日志核对 at_rest 位与 ZGU 后陀螺偏置），本轮未刷写、未设参、未采集。
