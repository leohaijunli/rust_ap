# 下一步计划（2026-11-09 状态同步）

本文档整合 `plan.md`（范围/边界/必要测试）与 `TODO.md`（状态/依赖）为唯一的前向执行计划，作为日常工作参考。历史证据与已通过结果保留在原两份文档及 `docs/acceptance/` 的链接中，不在此复述；新结果仍记入 `docs/acceptance/` 并回链本表的对应条目。

新产品路线的[评估与下一步建议](p0-p3-review-and-next-plan.md)已完成。工程师随后授权推进，最小 QGC 类型/健康位与[能力矩阵](qgc-compatibility.md)的软件实现、7 项用例/fmt/Clippy 及普通包构建/布局已通过，source=`fdf7fcd1ff9d10c7`；没有自动回装，QGC 版本/GUI/实物门待验。现行阶段门与非零输出授权不因此变更。[本轮记录](acceptance/p2/20261107-qgc-minimal.md)。

随后完成全 165 项[参数说明与审核](parameter-settings.md)：原数值/范围/存储及算法不变，逐项区分配置读取、条件作用与未接入；相关用例/fmt/Clippy、CRC 与普通包构建/布局通过，source=`a17a2d35bab6f05a`。未自动设参或烧录，新文案的 QGC 显示待回装/重新获取元数据。[记录](acceptance/p3a/20261107-parameter-help.md)。

工程师随后授权回装，新包 `a17a2d35bab6f05a` 已经原 bootloader 上传/CRC 校验；165 项值/类型与历史故障保持，元数据 CRC/字节相同，QGC 5.0.3 类型/说明界面由工程师确认通过。[回装记录](acceptance/p2/20261107-qgc-flash.md)。下一步回到 H-04 输入/实物门，不自动进行断电/拔卡。

2026-11-08 增量：当时板载为合并包 `81fd0d06601f9e2f`，工程师确认日志文件删除正常；只读复测保持 Running/Filter、无 Numerics/漏期，yaw 根因调查按要求停止。[复测记录](acceptance/magnetometer/20261108-bringup.md#yaw-只读复测与调查停止) / [日志记录](acceptance/pl-b/20261108-qgc-log-service.md)。

2026-11-09 工程师单独授权回装，**当前板载 source=`ecc872b86f6a2e98`（提交 `df8dcb8`，首轮正向保守 at_rest + ZVU/ZGU）**：原 bootloader 上传/CRC 和启动保持核对通过，主 EKF Running/Filter、无 Numerics/漏期；60 s 静止日志核对待工程师安排。[回装记录](acceptance/ekf2/20261109-startup-yaw-compare.md#板上回装2026-11-09)。

**后续仍以软件为主**：工程师只有飞控本体，没有外接设备。H-02/03/10 等外设门暂停；本次仅授权上述回装及启动查询，不自主追加采集、刷写或日志/输出动作。不推进 burst、非零输出或门后优化，硬件未验项不放行。

2026-11-09 追加：按工程师要求完成启动 yaw 的宿主合成对照。Rust 与固定 PX4 参考在相同输入下 `control_status` 完全一致、yaw 最大差 6×10⁻⁹°；PX4 地面标志通路把 tilt/yaw 锁定从 7.03/8.10 s 缩短到 0.58/1.65 s，并在陀螺偏置场景把末值误差从 +14.08° 降到 +0.10°。差异定位为 Rust 未接入 land-detected/at_rest 与 ZVU/ZGU，重力融合无关；未改代码/参数/门限，合成结论不代表绝对精度。[记录](acceptance/ekf2/20261109-startup-yaw-compare.md)。

2026-11-09 最新增量：最近已记录板载回装为 `d21b326c3a753d15`，高度/静止数据链及后续反向计时的软件结果见 [TODO.md](../TODO.md)。工程师已确认固定基站＋CAN Rover，要求[独立 RTK/地面站步骤](rtk-can-validation.md)；crate 已改为 `v6x-rust-ap`，新包 `136329bb61f63100` 仅构建、未烧录。[改名记录](acceptance/p1/20261109-crate-rename.md)。本文件较早“当前/只有本体”描述保留为对应批次背景；后续以 plan.md 的范围和 TODO.md 的最新状态为准，未授权实体操作与阶段门不自动放行。

## 1. 已留档基线（2026-11-07，最新增量见上文）

| 领域 | 状态 |
| --- | --- |
| 估计器 | 当前 source=`a17a2d35bab6f05a` 已回装至 COM4；只读快照 `Filter/Running`、fault=None、max_update=560 µs、发布 missed=0、积分 0 ppm。host/65 s 门与 PX4 同板对比复用[历史记录](acceptance/ekf2/20261106-single-estimator-timebase.md)，不冒充新完整时序验收。[回装记录](acceptance/p2/20261107-qgc-flash.md) |
| 标准输出 | host fixture 165 字段与历史 pyulog/pymavlink、USB/SD 成对核对已通过；本次未启动新日志。Flight Review/PlotJuggler 实际打开仍不能由 pyulog 解析替代 |
| 日志/dropout | 第 1 层完成：`dronecan_raw_command` 移除，产生 37.6→24.3 kB/s，dropout 41→4 次；剩余 3 次 ≥65.5 ms（u16 饱和，实际更长）为卡级停顿。第 2 层核查完毕：固定 `embedded-fatfs` 无预分配接口，写入器已 32 KiB 攒批，软件侧到顶 |
| 已修复并板验 | `record_can_stats` 误用 `Group::Health` 限流器（历史 bug）导致 `rust_runtime_health` 被 10 Hz can_stats 饿死；改回 `Group::Can` 后板验：health 与 can_stats 并存（≈10 Hz），12 s 会话 dropout=0。source=`ad99705b817a8c7f` |
| 协议/参数 | 165 项参数、P0–P3 软件+部分板载通过、MAVCAN（CAN2）收口、P4b-g 软件刷入（NO_GPS 帧已验） |
| QGC 最小兼容 | 当前合并包已回装：Quadrotor/disarmed、真实健康掩码 `0x0020000B`、165 项新说明和元数据 CRC/字节核对通过；工程师确认 QGC 5.0.3 类型/说明界面正常，完整双链路与实物门仍待验 |
| 当前实物状态 | IMU/BMP388 有效；CAN1/GNSS 无数据、INA219 无应答、PX4IO Timeout。参数存档兼容载入但 dirty=true/pending_save=false，未保存；旧 panic 保留；M1/完整 E7 不放行 |

## 2. 未收口问题（按优先级）

### P1 — E7 输出门（本板可完成，无硬件依赖）

1. ~~USB MAVLink 流核对~~ ✅ 已完成（2026-11-06）：各流速率符合 `MAV_RATE_*` 参数；LOCAL/GLOBAL_POSITION 按"无效不发"抑制（导航未资格化），SD 侧诊断快照照记。E7「USB+SD」成对证据齐备，[记录](acceptance/ekf2/20261106-single-estimator-timebase.md)。
2. ~~can_stats 限流修复的板载验证~~ ✅ 已完成（2026-11-06）：health 与 can_stats 并存 ≈10 Hz，12 s 会话 dropout=0。
3. ~~共享栈~~ ✅ 已决策并放宽（2026-11-06）：untouched ≥16 KiB（原 ≥32 KiB），依据 PX4 v1.17.0 WQ 栈对比，见 §5 决策点。

### P2 — 硬件输入门（等工程师/实物，按依赖排序）

以下外设硬件项当前暂停；恢复后再沿用原顺序，不以软件检查补称通过。

| 顺序 | 门 | 输入 | 内容 |
| --- | --- | --- | --- |
| 1 | ~~H-04~~ ✅ 通过（2026-11-07）：分布 10 次 ≤5 ms、事件 0；断电/缺卡/拔卡实物闭环；边界见[收口记录](../acceptance/pl-a/20261107-h04-closure.md)。 |
| 2 | H-02 | 工程师 | ESC 接 CAN1 并供电（零输出）→ `!dronecan` 只读枚举身份/index → P4b-a CAN-A04 零输出 Status 与断线恢复实测 |
| 3 | H-03 | 工程师 | TELEM1 适配 → P2 双链路 + P3 参数双链路一致/保存重启 |
| 4 | H-10 | 工程师 | GPS 实物 → P4b-g GNSS-03 + TIME-02 授时 |

### P3 — 当前可做的软件项（硬件门暂停）

- 传感器时间残差 −3.24%（PX4 `32/30` 换算后）：不进入积分（`integration_error_ppm=0`），归因需保留 FIFO tick 分布与批次时间序列证据；仅在需要传感器时间诊断精度时再投入。
- 气压日志新鲜度门控与 6 项测试告警修复的软件检查已通过，合并普通包 `49b3552f38510053` 随后单独授权回装；不重复 host/PX4 对拍，不由启动快照放行真实日志异常输入或完整 E7。[软件记录](acceptance/pl-a/20261108-baro-log-freshness.md) / [回装记录](acceptance/p1/20261109-log-guard-flash.md)。
- 启动 yaw 慢：宿主合成对照已完成，Rust 与 PX4 默认核心数值一致，差异在缺少 land-detected/at_rest 与 ZVU/ZGU。已按工程师确认移植 PX4 反向计时机制实现 at_rest + ZVU/ZGU（纯软件）：at_rest 首拍置位，1° 内收敛 8.104→1.648 s（与 PX4 地面标志参考逐项一致），偏置场景末值 +14.077°→+0.051°（参考 +0.100°）；host 225 项、Clippy/构建通过，固件 `197fb10f367d6ae2` 未回装。不改磁检查门限，合成结果不宣称绝对精度。[记录](acceptance/ekf2/20261109-startup-yaw-compare.md#机制修正改为-px4-反向计时同日工程师确认)。

### P4 — 优化类（门后才做）

- EKF2 阶段 4 / M4（H 稀疏化、小角度、协方差/内存、FPU 路径）：前置为行为/实时性门通过。
- 结构化兜底（双优先级队列、裸块环形日志）：仅在 1–3 层 + 卡选型后 dropout 仍超 H-04 窗口时评估，见[备选文档](logging-dropout-options.md)。
- TIM2 原生 32 位驱动：评估结论"无可测收益，暂不执行"，见[时基说明](imu-timebase.md)。

## 3. 执行顺序

```
已完成：气压日志门控＋测试告警修复 → 必要检查/普通构建 → 单独授权回装/启动核对
后续：记录/提交推送，仍以软件为主；不自主追加实体动作

外设具备后恢复：H-02 / H-03 / H-10 依次到位
   ↓
CAN-A04 → P2/P3 双链路 → GNSS-03 + TIME-02 → P4 SENS 收口
   ↓
M1 联合负载（5 min）：全链 + dropout vs H-04 窗口 + 栈/CPU 复核
   ↓
M4 / 结构化优化（仅门后）
```

## 4. 必要测试原则（沿用 plan.md §4）

- 一次有效结果覆盖一个行为；正常路径 + 关键失败路径。
- 复用未受改动影响的结果；相关实现变化或新失败才补测。
- 文档/计划修改只做内容、链接与 `git diff --check`。
- 板载单次采集（`!ekf2` + `!metrics` + 65 s 日志）作为改动后的默认复核口径。

## 5. 决策点（需工程师输入）

| 决策 | 背景 | 选项 |
| --- | --- | --- |
| 共享栈 32 KiB 门 | ✅ 已决策（2026-11-06）：放宽为 untouched ≥16 KiB。依据：PX4 v1.17.0 Kconfig 默认 WQ 栈 —— INS0（EKF2）6000 B、SPI 2392、hp_default 2800、uavcan 3624、lp_default 3500、tty 1728、manager 1280，等效活跃路径合计 ≈21.3 KB（拆在 ~7 个独立任务栈）；本项目同样内容压在单个 64 KiB 共享栈，实测 touched 33392 B 同量级略高（含 IRQ 嵌套）。16 KiB 余量 ≈ 实测的 2×，也是 PX4 最大单 WQ 栈的 2.7×。采集工具两处门已同步（`ekf2_runtime_capture.py`、`p1_capture.analyze` 参数化）；**按新判据的 65 s 完整采集 passed=true, issues=[]**（栈最低 30728 B、CPU 28.7%、deadline_misses=0、发布 missed=0、积分 0 ppm）。 |
| dropout 验收 | 已定（2026-11-07）：正常日志单次 ≤10 ms、事件丢失 0；32 GB 普通卡基线 3×5 ms、正常丢 8 条（0.026%） | 按窗口判定 5 min 分布与断电/拔卡；选型标注偏差在 H-04 收口注明 |
| `LOG_CAN_HZ` | 已砍 raw_command，400 Hz 只剩 ESC/Status | 维持 400 或按事件记录 |
| 传感器时间残差 | −3.24%，不影响积分 | 追查 or 保留诊断口径不动 |

## 6. 与旧文档的关系

- `plan.md`：保留为范围、工程边界与历史验收的原始记录；新条目不再追加到其 §8b 之后，阶段状态以本文档 §1/§2 为准。
- `TODO.md`：保留为证据索引与历史状态；当前优先级以本文档 §2/§3 为准。
- 新验收结果写入 `docs/acceptance/<domain>/<date>-<topic>.md`，并在本文档 §1/§2 对应条目回链。
