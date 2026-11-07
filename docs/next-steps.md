# 下一步计划（2026-11-07 状态同步）

本文档整合 `plan.md`（范围/边界/必要测试）与 `TODO.md`（状态/依赖）为唯一的前向执行计划，作为日常工作参考。历史证据与已通过结果保留在原两份文档及 `docs/acceptance/` 的链接中，不在此复述；新结果仍记入 `docs/acceptance/` 并回链本表的对应条目。

新产品路线的[评估与下一步建议](p0-p3-review-and-next-plan.md)已完成。工程师随后授权推进，最小 QGC 类型/健康位与[能力矩阵](qgc-compatibility.md)的软件实现、7 项用例/fmt/Clippy 及普通包构建/布局已通过，source=`fdf7fcd1ff9d10c7`；没有自动回装，QGC 版本/GUI/实物门待验。现行阶段门与非零输出授权不因此变更。[本轮记录](acceptance/p2/20261107-qgc-minimal.md)。

随后完成全 165 项[参数说明与审核](parameter-settings.md)：原数值/范围/存储及算法不变，逐项区分配置读取、条件作用与未接入；相关用例/fmt/Clippy、CRC 与普通包构建/布局通过，source=`a17a2d35bab6f05a`。未自动设参或烧录，新文案的 QGC 显示待回装/重新获取元数据。[记录](acceptance/p3a/20261107-parameter-help.md)。

工程师随后授权回装，新包 `a17a2d35bab6f05a` 已经原 bootloader 上传/CRC 校验；165 项值/类型与历史故障保持，元数据 CRC/字节相同，QGC 5.0.3 类型/说明界面由工程师确认通过。[回装记录](acceptance/p2/20261107-qgc-flash.md)。下一步回到 H-04 输入/实物门，不自动进行断电/拔卡。

## 1. 当前基线（2026-11-07 回装，复用历史验收）

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

| 顺序 | 门 | 输入 | 内容 |
| --- | --- | --- | --- |
| 1 | H-04 | 工程师 | 三项实物测试闭环（断电/缺卡/拔卡均过）；只剩裁决：152 ms 长尾处置（换可靠卡 / 队列 512→1024 槽扩容复测 / 接受离群）与写失败自动重启是否实现 |
| 2 | H-02 | 工程师 | ESC 身份/index/旋向 → P4b-a CAN-A04 实物（零输出、Status、断线恢复） |
| 3 | H-03 | 工程师 | TELEM1 适配 → P2 双链路 + P3 参数双链路一致/保存重启 |
| 4 | H-10 | 工程师 | GPS 实物 → P4b-g GNSS-03 + TIME-02 授时 |

### P3 — 软件遗留（低优先，不阻塞上述门）

- 传感器时间残差 −3.24%（PX4 `32/30` 换算后）：不进入积分（`integration_error_ppm=0`），归因需保留 FIFO tick 分布与批次时间序列证据；仅在需要传感器时间诊断精度时再投入。
- `record_baro` 限流槽位在新鲜度检查之前消费（陈旧样本消耗槽位）；当前无实际丢失（下一样本间隔 > 限流周期），与 can_stats 修复同批顺手处理即可。

### P4 — 优化类（门后才做）

- EKF2 阶段 4 / M4（H 稀疏化、小角度、协方差/内存、FPU 路径）：前置为行为/实时性门通过。
- 结构化兜底（双优先级队列、裸块环形日志）：仅在 1–3 层 + 卡选型后 dropout 仍超 H-04 窗口时评估，见[备选文档](logging-dropout-options.md)。
- TIM2 原生 32 位驱动：评估结论"无可测收益，暂不执行"，见[时基说明](imu-timebase.md)。

## 3. 执行顺序

```
本轮收尾（can_stats 修复板验 → commit/push）
   ↓
E7 软件门收口（USB 流核对 + 记录归档）          ← 本板即可
   ↓
H-04 卡选型 + dropout 复验                      ← 等工程师输入
   ↓
H-02 / H-03 / H-10 依次到位
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
