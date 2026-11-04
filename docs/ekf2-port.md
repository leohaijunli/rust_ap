# EKF2 → Rust `no_std` 移植方案（v2，工程落地版）

日期：2026-11-04。基线固定为 PX4 v1.17.0 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`；不以工作区里的 v1.18 checkout 作为生成输入。范围仍受 [plan.md](../plan.md) 的 M1 数据链优先顺序约束。

## 1. 结论与执行边界

- EKF2 最终替代 M1 的 Mahony 姿态基线，但**不改变当前 M1 数据链验收顺序**。E0（host-only 基础设施）可先行；E1 及之后的板上融合仍按 `P6a-e / EKF-01～03` 的依赖放行。
- 保留 PX4 的 **24 状态布局**，不用 16 状态裁剪版；未启用状态用激活掩码和零方差关闭。
- 传感器范围按 **IMU、GNSS、气压、磁力计、雷达高度计** 五类设计。当前固件只有 SPI2 ICM-42688-P；BMP388/BMM150 驱动、GNSS 实物数据和雷达高度计硬件仍是前置条件，不能用虚构数据占位。
- 算法可替换：固件只依赖 `NavFilter`；EKF2 内部再分 `Core` / `Gate` / `Supervisor` / `AidSource`。
- 参数从固定 PX4 YAML 生成，保留全部 `EKF2_*` 名称；不新建第二套运行时参数系统，最终接入现有 `src/core/parameters.rs` 与 FRAM 双槽存储。
- 无 GPS 解算作为独立实例，输出必须携带有效性和随时间增长的水平位置 σ；不得被下游当作第二套 GPS 位置。

## 2. 相对 v1 的改动

| 项 | v1 | v2 |
| --- | --- | --- |
| 状态维度 | 裁到 16 | 保留 PX4 24 状态布局，激活掩码关闭未用状态 |
| 传感器 | IMU/GPS/气压/磁力计 | 加入雷达高度计与地形估计 |
| 结构 | 单一滤波器 | `NavFilter` / `AidSource` / `Core` 分层 |
| 参数 | 未涉及 | 从 PX4 YAML 生成，名称/类型/默认值/范围对齐 |
| 输出 | 单解 | 主解算（含 GPS）+ 独立无 GPS 解算 |

24×24 f32 协方差约 2.3 kB，H7 内存不是约束。保留 PX4 索引顺序（四元数 0-3、速度 4-6、位置 7-9、陀螺零偏 10-12、加计零偏 13-15、地磁 16-18、磁零偏 19-21、风 22-23）可直接对照 PX4 `estimator_states`，减少对拍时的索引映射。

## 3. 与当前工程的对齐

v2 的目标 crate 划分（`nav-types` / `nav-core` / `nav-params` / `nav-ekf2` / `nav-simple` / `nav-sim` / `nav-replay` / `nav-fw`）作为后续目标，不在 E0 一次性建空 workspace。当前工程是单 crate `v6x-rust-led`，E0 先落在：

```
src/core/estimator/
├─ mod.rs               NavFilter / AidSource / 输入输出 DTO
├─ params_generated.rs  由 PX4 v1.17.0 YAML 生成
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

第二层是 EKF2 内部的 aid source。状态维度固定为 PX4 的 24，用 `StateVector` / `Covariance` 别名而不是 const 泛型，避免为不存在的其他维度付出复杂度：

```rust
pub const STATE_COUNT: usize = 24;
pub type StateVector = [f32; STATE_COUNT];
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
| IMU | `sensor_combined` | 预测输入；积 delta angle/velocity（含 coning/sculling），降到约 100 Hz；原始 IMU 供输出预测器 | 已有 8 kHz ICM-42688-P；需补积分/降采样 |
| GNSS | `vehicle_gps_position` | 位置 3 + 速度 3，标量序贯；`EKF2_GPS_DELAY` 对齐；局部 NED 原点 | P4b-g 实物数据与 H-10 |
| 气压 | `vehicle_air_data` | 高度标量；处理 `EKF2_PCOEF_*` / `EKF2_BARO_*` | BMP388 驱动未实现 |
| 雷达高度计 | `distance_sensor` | 离地高度 = 斜距·cos(倾角)；地形估计或高度基准 | 硬件/安装/量程待补（H-12） |
| 磁力计 | `vehicle_magnetometer` | 3D 融合（地磁与磁零偏状态）或仅偏航 | BMM150 驱动未实现；无 GPS 时磁偏角需参数 |

雷达高度计优先采用**单状态地形估计**：独立 1 状态 KF 估计地面海拔，高度计只更新它，再由气压高度与离地高度反推地形。这样能复用 PX4 `EKF2_RNG_*` 的条件式使用、门限、噪声、安装位置偏置和质量时间参数。作为高度基准只适用于平坦地面，作为后续简化选项。

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

E0 可在当前 M1 数据链工作之外先行；E1 起仍按 `P6a-e / EKF-01～03` 依赖放行，不跳过 M1、P6a、P4b-g、P2、PL-b。

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
- f32 协方差可能丧失正定性，需要对称化与钳位；经纬度换算用 f64，且只在输出边界。
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
| 接口骨架 | `NavFilter`、`AidSource`、`Measurement`/`AidId`、`ControlStatus`、`ScalarBatch` 已落地；`src/lib.rs` 注册 `estimator` |
| 生成器测试 | `tools/test_gen_params.py` 8 项通过，含 `--check` 漂移拒绝/不改写；固定输入 `--check`、fmt 和 CI 配置核对通过 |
| 复用软件结果 | host 102 项、host lib/普通固件 Clippy 通过；额外 `--tests` Clippy 的 3 处既有 `chunks_exact_to_as_chunks` 告警保留，未改无关 DroneCAN 测试 |
| 普通构建/打包 | 原生 Windows release 与 ELF/bin/PX4 分区/向量检查通过；source=`f79daad8874002c4`，raw/padded=`412720/412736 B`，static RAM=`268680 B`；未刷写 |
| CI | 固定 PX4 checkout + `--check` 已接入；本轮核对配置与相同输入的本地检查，不预写远程结论 |
| 尚未开始 | `Core`/`Gate`/`Supervisor`、IMU 降采样、`nav-sim`/`golden_cpp`、回放与对拍 |

首次调用 `tools/firmware.sh build` 因 Windows checkout 的 CRLF 行尾失败；本轮改用等价的原生构建/打包命令通过，未改动该脚本。构建报告位于本地 `target/thumbv7em-none-eabihf/release/v6x-rust-led.build.json`。

E0 尚未满足“同一输入可同时喂 C++ 与 Rust”的退出条件；黄金数据、合成仿真器与 ULog 回放随 E1 排期，必须先提供参考输出再验 Rust 预测器。当前 DTO/trait 仅为未接入固件的骨架；IMU 积分时长、序贯更新等契约在对拍前细化。E1 起的放行条件不变。
