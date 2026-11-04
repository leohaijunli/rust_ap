# EKF2 → Rust `no_std` 移植方案（v2，工程落地版）

日期：2026-11-04。基线固定为 PX4 v1.17.0 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`；不以工作区里的 v1.18 checkout 作为生成输入。范围仍受 [plan.md](../plan.md) 的 M1 数据链优先顺序约束。

## 1. 结论与执行边界

- EKF2 最终替代 M1 的 Mahony 姿态基线，但**不改变当前 M1 数据链验收顺序**。E0 与 E1-H～E6-H 可在 host 依软件门先行；实物资格和 E7 上板仍按 `P6a-e / EKF-01～03` 的依赖放行。
- 保留固定 PX4 的 **24 维误差状态/协方差**与 **25 个名义状态（含地形）**，不用 16 状态裁剪版；四元数是 4 个名义分量，但姿态误差只有 3 维。
- 传感器范围按 **IMU、GNSS、气压、磁力计、雷达高度计** 五类设计。当前固件只有 SPI2 ICM-42688-P；BMP388/BMM150 驱动、GNSS 实物数据和雷达高度计硬件仍是前置条件，不能用虚构数据占位。
- 算法可替换：固件只依赖 `NavFilter`；EKF2 内部再分 `Core` / `Gate` / `Supervisor` / `AidSource`。
- 参数从固定 PX4 YAML 生成，保留全部 `EKF2_*` 名称；不新建第二套运行时参数系统，最终接入现有 `src/core/parameters.rs` 与 FRAM 双槽存储。
- 无 GPS 解算作为独立实例，输出必须携带有效性和随时间增长的水平位置 σ；不得被下游当作第二套 GPS 位置。

## 2. 相对 v1 的改动

| 项 | v1 | v2 |
| --- | --- | --- |
| 状态维度 | 裁到 16 | 对齐固定 PX4：24 维误差状态、25 个名义状态，含地形 |
| 传感器 | IMU/GPS/气压/磁力计 | 加入雷达高度计与地形估计 |
| 结构 | 单一滤波器 | `NavFilter` / `AidSource` / `Core` 分层 |
| 参数 | 未涉及 | 从 PX4 YAML 生成，名称/类型/默认值/范围对齐 |
| 输出 | 单解 | 主解算（含 GPS）+ 独立无 GPS 解算 |

24×24 f32 协方差为 2304 B。固定基线 `EKF/python/ekf_derivation/generated/state.h` 定义名义状态 25 个分量，但 `State::size=24` 是误差维度；不能把旧版四元数占 4 维的索引用于协方差。

| 量 | 名义状态索引 | 误差状态/协方差索引 |
| --- | --- | --- |
| 姿态 | 四元数 0–3 | 旋转误差 0–2 |
| 速度 / 位置 | 4–6 / 7–9 | 3–5 / 6–8 |
| 陀螺 / 加计零偏 | 10–12 / 13–15 | 9–11 / 12–14 |
| 地磁 / 磁零偏 | 16–18 / 19–21 | 15–17 / 18–20 |
| 风 / 地形 | 22–23 / 24 | 21–22 / 23 |

## 3. 与当前工程的对齐

v2 的目标 crate 划分（`nav-types` / `nav-core` / `nav-params` / `nav-ekf2` / `nav-simple` / `nav-sim` / `nav-replay` / `nav-fw`）作为后续目标，不在 E0 一次性建空 workspace。当前工程是单 crate `v6x-rust-led`，E0 先落在：

```
src/core/estimator/
├─ mod.rs               NavFilter / AidSource / 输入输出 DTO
├─ params_generated.rs  由 PX4 v1.17.0 YAML 生成
├─ covariance_generated.rs 固定 PX4 符号表达式转译，CI --check
├─ imu_ekf.rs            E1-H 无外部 aid 的延迟核心
├─ imu_downsampler.rs    定长积分降采样 / IMU 缓冲
├─ math.rs              四元数 / WGS84 累积
└─ (后续) core.rs, gate.rs, supervisor.rs, aid_sources/
```

只有 host/board 边界确实需要独立编译或独立依赖时，才拆 crate；避免提前建立空框架。

### 3.1 参数与存储

- 生成的 `EKF2_PARAMS` 先作为唯一 EKF2 参数元数据表；`support` 标记 `Active` / `Ignored` / `Planned`。
- 运行时接入现有 `parameters::Definition` / `Snapshot` / FRAM 双槽协议，不引入 `sequential-storage` 或第二套参数存储。
- 全部 142 个 `EKF2_*` 参数保留；未实现参数被修改时记录 warning，不静默忽略。
- 扩展参数使用 `NAVD_*`，不占用 `EKF2_*` 命名空间。
- 现有 FRAM 记录为 `24 + COUNT * 4` 字节；加入 142 个参数后仍在 1024 B 槽内。schema tag 变化会使旧存档按既有策略失效并回退默认值。

### 3.2 与 Mahony 的切换

- `v6x_rust_led::config::Algorithm` 后续增加 `Ekf2`；Mahony 保持可运行回退。
- EKF 无效、NaN、协方差异常、输入陈旧或 deadline miss 时撤销有效性并回退 Mahony；不无缝伪造连续状态。
- 估计器输入为带 source/sequence/sample time/valid/age 的 DTO，输出为 `Watch<EstimatorState>` 最新快照；不直接访问外设或链路。

## 4. 分层接口

第一层是固件唯一依赖的滤波器接口：

```rust
pub trait NavFilter {
    type Params;

    fn new(params: &Self::Params, t0: Timestamp) -> Self;
    fn push_imu(&mut self, sample: &ImuDelta);
    fn push_measurement(&mut self, measurement: Measurement);
    fn step(&mut self, now: Timestamp) -> StepReport;
    fn output(&self) -> NavOutput;
    fn health(&self) -> FilterHealth;
}
```

第二层是 EKF2 内部的 aid source。名义状态与误差状态分别使用固定别名，不使用 const 泛型：

```rust
pub const STATE_COUNT: usize = 24;
pub const NOMINAL_STATE_COUNT: usize = 25;
pub type StateVector = [f32; NOMINAL_STATE_COUNT];
pub type ErrorStateVector = [f32; STATE_COUNT];
pub type Covariance = [[f32; STATE_COUNT]; STATE_COUNT];

pub trait AidSource<P> {
    fn id(&self) -> AidId;
    fn delay_us(&self, params: &P) -> u32;
    fn enabled(&self, params: &P, status: &ControlStatus) -> bool;
    fn scalar_updates(&mut self, state: &StateVector, covariance: &Covariance, out: &mut ScalarBatch);
    fn on_reject(&mut self, status: &mut ControlStatus);
}
```

`ScalarBatch` 是 4 槽定长批（3D 测量最多 3 个标量更新），满时 `push` 返回 `false` 并丢弃，不做堆分配。

第三层是 `Core`（状态、协方差、预测）、`Gate`（NIS 门限）、`Supervisor`（对准、重置、aid 状态机）。换 UKF/InEKF 时只替换 Core 与更新公式；aid source 的创新量/方差计算尽量复用。

`no_std` 无分配器，固件持有 `Estimator<F: NavFilter>` 的静态实例，换算法只改类型别名，不使用 `Box<dyn NavFilter>`。

## 5. 五类传感器

| 传感器 | PX4 对应输入 | 测量模型 | 当前前置条件 |
| --- | --- | --- | --- |
| IMU | `vehicle_imu`（多实例）/ `sensor_combined`（单实例） | 预测输入；积 delta angle/velocity（含 coning/sculling），降到约 100 Hz；原始 IMU 供输出预测器 | 已有 8 kHz ICM-42688-P；需补积分/降采样 |
| GNSS | `vehicle_gps_position` | 位置 3 + 速度 3，标量序贯；`EKF2_GPS_DELAY` 对齐；局部 NED 原点 | P4b-g 实物数据与 H-10 |
| 气压 | `vehicle_air_data` | 高度标量；处理 `EKF2_PCOEF_*` / `EKF2_BARO_*` | BMP388 驱动未实现 |
| 雷达高度计 | `distance_sensor` | 离地高度 = 斜距·cos(倾角)；地形估计或高度基准 | 硬件/安装/量程待补（H-12） |
| 磁力计 | `vehicle_magnetometer` | 3D 融合（地磁与磁零偏状态）或仅偏航 | BMM150 驱动未实现；无 GPS 时磁偏角需参数 |

地形是固定基线的误差状态 23 / 名义状态 24，范围传感器可更新高度与地形的相关状态；不另建独立 1 状态 KF 冒充原版 EKF2。`EKF2_RNG_*` 的条件式使用、门限、噪声、安装位置偏置和质量时间参数随 E3 对拍落实。

必须处理：最小/最大量程、信号质量、倾角超限、安装位置杠杆臂补偿、数据丢失与恢复。

## 6. 参数生成

`tools/gen_params.py` 读取固定 PX4 v1.17.0 的 `src/modules/ekf2/module.yaml` 与 `params_*.yaml`，合并后生成 `src/core/estimator/params_generated.rs`。生成输入按 git ref 读取，避免误用工作区 v1.18 文件。

生成表字段：

```rust
pub struct ParamDef {
    pub name: &'static str,
    pub kind: ParamKind,
    pub default: ParamValue,
    pub min: Option<f32>,
    pub max: Option<f32>,
    pub unit: &'static str,
    pub support: Support,
}
```

- 名称必须 ≤16 字符并符合 MAVLink `param_id` 约束。
- `ParamKind`：`Int32` / `Float` / `Boolean` / `Enum` / `Bitmask`。
- `Support`：`Active` / `Ignored` / `Planned`；E0 默认 `Planned`，实现某 aid 后再提升。
- `.github/workflows/p1.yml` 拉取同一固定 PX4 commit 的 EKF2 目录，再用 `--check` 比较生成文件与 YAML；名称/类型/默认值/范围漂移即失败。host 依赖 `PyYAML==6.0.3` 已固定在 `tools/requirements.txt`。
- 运行时生成强类型 `EkfParams` 快照；参数变更只在安全点生效，不在融合中途改门限。

## 7. 无 GPS 解算

无水平外部观测时，水平速度和位置不可观测，只能航位推算：残余加计零偏 0.05 m/s² 时，60 s 水平位置误差约 90 m。因此输出定位是：

- 姿态、高度、垂直速度可靠；
- 水平位置是带增长协方差的航位推算；
- 必须报告 `validity` 与 `horiz_pos_sigma`，下游据此降级。

独立性：独立任务/缓冲/协方差，只共享只读传感器样本；类型上不含 `Gnss` 变体；编译期检查依赖树。注意这是数据/算法独立，不是故障域独立；`panic = abort` 时两个实例会一起停止。

初始化：静止时加计求倾角、磁力计求偏航；水平原点默认起飞点；磁偏角无 GPS 时由 `NAVD_*` 参数给定，否则标记磁北参考。从主解算拷贝初值作为显式开关，默认关闭。

输出走独立 `ODOMETRY` 流；`estimator_type` 与 frame 字段按 MAVLink `common.xml` 确认，不复用主解算的全局位置消息。

后续水平约束按性价比：零速更新 → 多旋翼拖曳模型（`EKF2_DRAG_CTRL` / `EKF2_BCOEF_*` / `EKF2_MCOEF`）→ 磁异常匹配 `AidSource`。

## 8. 里程碑（避免与项目 M1 重名）

| 阶段 | 内容 | 退出标准 |
| --- | --- | --- |
| **E0 基础设施** | 参数生成、ULog 回放框架、C++ 黄金数据程序、合成仿真器 | 参数表 CI 通过；同一输入可同时喂 C++ 与 Rust |
| **E1 仅预测** | IMU 积分/降采样、状态/协方差预测 | 对拍四元数偏差 ~1e-5，协方差对角相对偏差 <1e-4，持续 10 s |
| **E2 气压 + 磁力计 + 对准** | 倾斜/偏航对准、零偏收敛 | 静止数据姿态/零偏曲线与 PX4 同形，NIS 合理 |
| **E3 GPS + 雷达** | 序贯更新、门限、地形估计、杠杆臂 | 真实日志姿态 RMS <0.5°、速度 RMS <0.1 m/s |
| **E4 延迟视界 + 输出预测器** | 缓冲对齐、低延迟输出 | 抖动/延迟下输出无台阶，位置连续 |
| **E5 状态机与重置** | 超限、超时、aid 恢复、位置/速度/高度/偏航重置 | 故障注入通过，重置后协方差同步修正 |
| **E6 无 GPS 解算** | 独立实例、输出、MAVLink 流 | 漂移与理论量级一致，σ 能包住误差 |
| **E7 上板** | Embassy 任务、DWT 计时、参数桥 | 见第 9 节 L5，必要范围以 `plan.md` 为准 |

工程师已确认拆分软件门与实物资格：E1-H～E6-H 按上述顺序在 host 先行，前一软件门通过才推进后一门；实物资格和 E7 上板仍依赖 M1、P6a、P4b-g、P2、PL-b。合成输入只用于软件对拍，不冒充真实传感器结果。

E1-H 范围为纯 IMU、无外部 aid 的 `core_imu`：降采样、原版延迟缓冲、预测、协方差约束与零创新航向/伪位置/伪高度等内部融合。首次 Rust 对拍前固定判据：符号对齐后的四元数差范数 ≤`1e-5`；全部状态绝对/相对容差 `1e-5`/`1e-5`；完整协方差绝对/相对容差 `1e-7`/`1e-4`，持续 10 s；时间与状态标志必须一致。PSD 用归一化 LDL 检查，仅容许 `1e-5` 的 f32 舍入枢轴误差；对角相对误差报告以 `1e-7` 为分母下限。另验非法/陈旧输入、clipping 与数值失效。C++ 基线重建的既定容差不变。

表中真实日志、硬件和实时性指标属于实物资格；E2-H～E6-H 先完成相应的固定 C++ 合成输入对拍和关键失败路径，不用合成数据冒充真实日志。

## 9. 验证

### L0 单元与性质测试

- 四元数/旋转归一化，与参考实现对比。
- 有限差分校验 H/F 雅可比；容差按相对误差。
- 协方差对称、对角非负、特征值非负；随机序列数万步不发散。
- 每个 aid source 的创新量/方差公式单独对照 C++ 同名函数。

### L1 C++ 黄金数据逐步对拍

`tools/golden_cpp` 链接固定 PX4 EKF 库，读 CSV 输入，每步输出状态与协方差。输入同时喂 C++ 与 Rust，逐步比对状态、协方差对角、创新量与创新方差。前几秒应几乎逐位一致；融合后看差异是否有界，不要求位级一致。

真实日志回放需在飞行时开启全速率传感器日志（`SDLOG_PROFILE` 的 EKF2 回放位，具体位以 v1.17.0 为准）。

### L2 真实日志回放

对照 PX4 `estimator_*` / `estimator_aid_src_*`，统计姿态/速度/位置/零偏 RMS 与最大偏差、创新量相关系数、fused 一致率、状态机转移时间。日志集覆盖静止、悬停、机动、GPS 丢失、气压突变、雷达越界。

### L3 合成数据、蒙特卡洛与故障注入

`nav-sim` 生成真值轨迹，叠加 IMU 白噪声/随机游走零偏、气压噪声/漂移/下洗、GPS 噪声/延迟/更新率、雷达量程/野值、磁硬铁/软铁。用 NEES/NIS 做 χ² 区间检验；无故障时门限拒绝率接近 0。

故障注入：GPS 跳变/丢失/恢复、气压阶跃、磁干扰、雷达野值/失效、IMU 饱和、时间戳抖动、乱序、丢包。验证不发散、状态机符合预期、能恢复。

### L4 无 GPS 专项

离线切 GPS（30/60/120 s），以 GPS 轨迹为真值画水平误差，检查 3σ 覆盖率；GPS 正常时姿态/高度与主解算接近；运行期向 GPS 流注入大幅错误，无 GPS 解算不受影响；零速更新/拖曳模型逐项对比漂移改善。

### L5 MCU 验证

DWT 测预测/标量更新/整轮最坏时间，初始目标整轮 <1 ms；热数据放 DTCM，DMA 缓冲不能放 DTCM；启动涂色检查栈余量；板上回放与主机回放一致；连续运行数小时检查时间戳溢出/协方差发散；QGC 参数完整往返。

### CI

1. `cargo test` + `proptest`（L0）
2. 参数表漂移检查
3. 小型黄金数据对拍（L1）
4. 固定种子蒙特卡洛小样本（L3）
5. 无 GPS 解算依赖树不含 GNSS
6. 夜间大回放集 + 完整蒙特卡洛

## 10. 主要风险

- aid source 的超时、恢复、重置计数器是隐式状态机，文档少，只能读源码并用 L1/L2 对拍。
- 偏航/位置/速度重置必须同步修正协方差，容易漏。
- 雷达高度计的倾角与安装位置误差会直接变成高度偏差。
- 无 GPS 解算的水平位置可能被误用，必须靠 validity 与 σ 约束下游。
- f32 协方差可能丧失正定性，需要原版稳定更新与约束；全球位置按固定 `LatLonAlt` 用 f64 累积经纬度、f32 累积高度，不以名义状态的零 xy 代替真实位置累积。
- 当前缺少 BMP388/BMM150 驱动和雷达硬件；五传感器范围不能靠软件假设补齐。

## 11. 来源与许可

- PX4 v1.17.0 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`，EKF2 数学核心与参数 YAML，BSD-3-Clause。
- 逐文件记录原路径、commit、版权与 Rust 修改点，纳入 [provenance.md](provenance.md)。
- 不链接 PX4 C++，不引入 uORB/NuttX/FFI；C++ 黄金程序只在 host 工具中使用。

## 12. E0 当前状态（2026-11-04）

| 项 | 结果 |
| --- | --- |
| 参数生成 | `tools/gen_params.py` 从 `d6f12ad…` 的 20 个 YAML 生成 142 条 `EKF2_*`，全部 `Support::Planned`；`--check` 与仓库表一致，版权与来源已登记 |
| 参数契约 | `src/core/estimator/mod.rs` 的 host 测试校验数量、名称长度/字符集/唯一性、默认值有限且在范围内、类型与默认值变体一致 |
| 接口骨架 | 固定源码核对后纠正为 25 名义 / 24 误差维度；`ImuDelta` 补独立积分时长与逐轴 clipping，H 使用 `ErrorStateVector`；未接入固件 |
| 生成器测试 | `tools/test_gen_params.py` 8 项通过，含 `--check` 漂移拒绝/不改写；固定输入 `--check`、fmt 和 CI 配置核对通过 |
| host 参考 | GCC 15.2 编译固定 PX4 `core_imu`，保留原版缓冲、预测及伪观测/约束；从 YAML 应用 23 个相关默认值，不是独立纯预测 oracle |
| 输入与黄金数据 | 10 s / 400 Hz，先静止 1 s 再 5°/s 偏航；C++ 与 Rust DTO 均消费 4000 条，输入摘要 `f89e278bed7eae06` 一致；1001 步状态/完整协方差通过有限、非负对角与对称检查，最大对称误差 0、最大方差 102.850784 |
| 数据存储 | `tests/data/ekf2/imu_yaw/{imu,truth,golden}.csv.gz` 与 `manifest.json` 已生成，纳入仓库；原始黄金 CSV 2533266 B → 434688 B，记录固定 ref、源/工具/二进制 hash、参数、种子与容差；CI 只校验，不自动覆盖 |
| 本轮软件检查 | estimator 6 项、Rust intake 2 项、Python 12 项通过（含实际 ULog 文件、单实例 f32 换算、基线不可覆盖/篡改拒绝）；host lib/example Clippy 通过，97 项未受影响 Rust 用例复用。旧 DroneCAN `--tests` Clippy 告警保持，不扩改 |
| 普通构建/打包 | 原生 Windows release 与 ELF/bin/PX4 分区/向量检查通过；source=`3de5f0f08f2056b4`，raw/padded=`412720/412736 B`，static RAM=`268680 B`；未刷写 |
| CI | `7d0c486` 的 C++ 构建、Rust intake 与黄金数据回归[通过](https://github.com/leohaijunli/rust_ap/actions/runs/41780879699) |
| 后续状态 | E1-H 纯 IMU host 核心开始实现；完整五传感器、Rust 数值资格与板上融合尚未完成，实物资格门不变 |

历史普通构建 `f79daad8874002c4` 的布局检查保留。Bash 构建曾因 Windows checkout 的 CRLF 失败，采用等价原生命令通过，未改脚本。本轮 C++ 快照最初遗漏 WMM 头文件，随后发现缺少 PX4 的 `-fno-strict-aliasing`；均修正为固定源码/编译约定，没有改上游数学代码。

原版延迟缓冲填充时，最早时间戳会重复；本次保留 19 次初始重复，检查最新输入递增、延迟时间不倒退且不超过输入，禁止暖机后的重复。不伪造时间戳来让检查通过。

**“同一输入可喂 C++ 与 Rust”的最小 IMU 通路已建立，但只证明输入契约与 host 参考可用，不证明 Rust EKF 数值等价。** `vehicle_imu` 必须带积分字段；单实例 `sensor_combined` 按固定 `EKF2.cpp:682–686` 的 f32 rate×dt 转换，并要求对应 `sensor_selection`。当前 200 Hz 抽取的 `sensor_accel/gyro` 不能还原成等价回放输入；设备/校准切换、gyro clipping 或无效加计时间戳会显式拒绝。

### 重现与扩展

Ubuntu/WSL 使用已有工具环境（`tools/requirements.txt`），编译缓存和临时输出在忽略的 `target/ekf2-host/`：

```sh
python3 tools/golden_cpp/build.py --root /home/dev/PX4-Autopilot
python3 tools/check_ekf2_host.py
python3 tools/ekf2_replay.py ulog recording.ulg target/ekf2-host/imu.csv --instance 0
python3 tools/ekf2_replay.py ulog recording.ulg target/ekf2-host/imu.csv --source sensor_combined
```

`--record-baseline` 只允许首次创建，已有黄金数据拒绝覆盖；新场景使用新目录并审查后提交。C++ host 回归容差在首次对拍前固定为状态绝对 `1e-6`、协方差绝对 `1e-7`、相对 `1e-5`，不替代 E1 Rust 数值资格门。

## 13. E1-H 纯 IMU 软件门（2026-11-04）

**本地通过，实物资格未验。** `ImuEkf` 已实现已积分 `ImuDelta` 的旋转/速度复合降采样、固定缓冲、倾角初始化、状态与协方差预测、bias inhibit、Joseph 更新和无外部 aid 内部状态机。协方差的 144 个中间项 / 300 个上三角结果从固定 C++ 表达式严格生成；不手写近似雅可比，不改黄金结果。

- `imu_yaw` 与新增 `imu_noisy_clipped` 各 4000 输入 / 1001 更新 / 10 s，通过全状态、完整协方差、标志、对称及 PSD 对拍；本地 f32 位差均为 0。这里只覆盖这两个场景，不泛称完整 EKF2 等价。
- estimator 12 项、intake 2 项、Python 18 项及 host Clippy 通过；普通 H753 release/打包/布局通过，未刷写。具体产物与命令见 [E1-H 记录](acceptance/ekf2/20261104-e1-host.md)。
- 新增黄金数据与真值保存在 `tests/data/ekf2/imu_noisy_clipped/`，噪声/短时 clipping 明确为合成输入；旧 `imu_yaw` 不改写。CI 校验两场景与生成漂移，不记录/覆盖基线。
- `ImuEkfParams` 默认值与范围复用唯一生成表，配置改变须重新构造/显式 reset；超出 64 槽容量的配置拒绝。非法/非有限输入、>100 ms IMU 中断与数值异常锁定失效，显式 reset 才恢复。
- 当前只提供延迟数学核心，相对 yaw 不代表绝对航向，伪高度/位置不代表有效导航。没有输出预测器、完整 `NavFilter` 适配、8 kHz 板载输入桥、传感器融合或 Mahony 板载回退；这些仍按后续阶段实现/验收。下一软件门为 E2-H，不等待 M1 实物验收来编写 host 代码。

## 14. E2-H 本地软件门（2026-11-04）

首次 Rust 对拍前沿用 E1-H 的固定全状态/协方差及四元数容差；不因 E2 差异放宽。新的 C++ 参考场景先独立记录，现有基线不覆盖。状态/协方差通过后仍须补 NIS 与关键失败路径，不能只看最终对准标志关闭 E2-H。

NIS 首次对拍前固定绝对 `1e-6`、相对 `1e-4`，观测时间、fused/rejected 和控制/故障标志逐项相等。使用独立 NIS CSV，不改已记录的状态/协方差基线格式。

- 固定 C++ `baro_mag` profile 已编译，独立缓存在 `target/ekf2-host-baro-mag/`；启用原版 barometer/magnetometer 源文件与固定 WMM 表，不下载更新地磁表。切换现有缓存的 profile 会被拒绝。
- 从同一 YAML 应用额外 19 个气压/磁参数；合成观测以 50 Hz 压力高度与 body-FRD 磁场输入，CLI 为 `golden_cpp IMU.csv OUTPUT.csv BARO_MAG.csv`。输入显式标注合成，不是实际 BMP388/BMM150 数据。
- Rust 气压/磁力计序贯更新、对准、磁协方差/偏航停止状态机与垂直 dead-reckoning 门已通过正常、离群值和对准后 1 s 中断三类状态/协方差/NIS 对拍；不逐位一致，不放宽容差。
- 17 项核心、2 项 intake、22+3 项 Python、host Clippy 与普通 H753 release/打包/布局通过。新增中断场景为 12 s（9–10 s 中断），旧 5–6 s 场景保留但不能证明磁停止；完整边界、命令与历史失败见 [E2-H 记录](acceptance/ekf2/20261104-e2-host.md)。本次远程 CI 待核对，下一步 E3-H～E6-H，不等待工程师逐阶段确认。
