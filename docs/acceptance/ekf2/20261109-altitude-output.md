# ALTITUDE 输出与垂直有效性修复

日期：2026-11-09。工程师要求在含 `df8dcb8` / `186709b` 的固件上验证静止检测，随后增加 altitude 日志与 MAVLink 消息，并查清 baro 融合时高度仍为 NaN 的原因。仅高度数据发布，不放行 E7/M1、水平/全球导航或非零输出。

## 根因与固定 PX4 语义

- 初版 `bc7f29fe3a24ce1f`：USB 10 s 收到 97 条 `ALTITUDE`，只有 `altitude_monotonic` 有值；SD `target/e7-log/01.ulg` 为 576000 B、20.58 s、103 条 `vehicle_altitude`，其余字段为 NaN。103 个 baro aid 快照 `fused=true`，`baro_hgt` 位有效，但发布的 `z_valid/v_z_valid` 全为 false。
- **不是核心的 `vertical_position_dead_reckoning` 未清除。** `src/core/estimator/product.rs` 原 E7 资格护栏把四轴有效位强制清零，并清除 `solution_status_flags & 0x7e`。新增回归在同一 baro-only 输入下先确认核心垂直位置有效、原始快照 `z_valid/v_z_valid=true`，然后失败在产品快照的有效位断言；修后通过。
- `vertical_position_aiding()` 检查当前支持的 baro/range/GNSS 高度活动位（9/10/11），不要求 GNSS 水平定位。活动时保存 `last_vertical_position_aiding=last_height_fuse` 并清除垂直位置 dead-reckoning；停止后按 `EKF2_NOAID_TOUT` 超时。`timed_out()` 为 `last==0 || last+timeout<delayed_us`，等于边界时尚未超时。
- baro 正常融合与高度重置均更新 `last_height_fuse`；输入中断 400 ms 停止 baro。`height_reset_required()` 的连续坏垂直加速度 500 ms / 高度融合超时 5 s 是恢复判据，不是活动 aid 分支额外的有效性门。新增测试覆盖实际延迟融合、停止后的宽限期、严格超时边界，并确认 fake height 不延长真实 aid 的有效期。无 GNSS 时垂直**速度** dead-reckoning 可仍为 true；发布层按 PX4 使用位置/速度有效性的 OR，同时映射 `z_valid/v_z_valid`。
- 以上与固定 PX4 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` 的 `EKF/estimator_interface.cpp`、`EKF/ekf_helper.cpp:updateVerticalDeadReckoningStatus`、`EKF2.cpp:1588–1590` 一致，未修改 EKF 数学、aid 门限或融合顺序。

## 实现与输出契约

- 产品快照仅在运行时选择主 Filter 且姿态新鲜时保留核心垂直有效性；停用、Mahony 回退、陈旧时撤销高度及对应 solution 位。水平有效位、`heading_good_for_control` 与全球位置发布仍关闭，`LOCAL_POSITION_NED/GLOBAL_POSITION_INT` 不因高度有值而放行。
- `standard_output::altitude()` 为日志和双链路 MAVLink 的共享转换；未知值保持 NaN，不用零或气压冒充无效 EKF 局部高度。字段按固定 `mavlink/streams/ALTITUDE.hpp`：

| 字段 | 来源，单位均为 m |
| --- | --- |
| `altitude_monotonic` | 新鲜 BMP388 压力高度 |
| `altitude_amsl` | `z_valid` 时，全球原点有效则 `-z+ref_alt`，否则气压高度；无效 `ref_alt` 不回填气压 |
| `altitude_local` | 有效局部高度 `-z`（NED 向上取反） |
| `altitude_relative` | 当前无 home 数据源，按 PX4 无 home 分支使用 `-z` |
| `altitude_terrain` / `bottom_clearance` | 仅 `dist_bottom_valid` 时为 `-z-dist_bottom` / `dist_bottom` |

- MAVLink `ALTITUDE`（#141、component 1）复用 `MAV_RATE_EST`，默认 10 Hz；支持现有按需请求与逐链路流间隔。`time_usec` 为发布时的启动相对微秒时间。
- ULog 扩展 topic `vehicle_altitude`（本项目定义，不冒充 PX4 uORB 消息）：ID 43，`uint64_t timestamp` 加六个 `float`，payload 32 B；追加在 M1 registry，标准 estimator registry 不变。服从 `LOG_EN_EST/LOG_EST_HZ`（默认 5 Hz），共用有界队列；未增加日志自动启动、参数或新状态源。

## 软件检查

- 初版 host 全库 226 项通过；追加修复只复验相关项：`estimator::` 72 项（含 MAVLink）、`ulog::` 7 项、`telemetry::` 3 项通过。
- 新增产品回归先失败，修后通过；新增 ULog 测试曾触发 `chunks_exact_to_as_chunks` Clippy，改用定长 `as_chunks::<4>()` 后该用例及 host Clippy 通过。fmt、host 库/测试与普通 H753 Clippy `-D warnings` 通过。
- 一次局部 `code-simplifier` 审视：保留共享转换、现有队列/调度和提前返回，固定 schema 编码错误不再静默丢弃；未扩展算法或架构。
- 初版固件与报告保存在 `target/altitude-20261109/trial-bc7f29fe3a24ce1f/`；修复版在 `target/altitude-20261109/d21b326c3a753d15/`。旧 `00.ulg/01.ulg` 保留，不覆盖历史结果。

## 板载验证

- 普通构建/打包/布局与入口帧通过：source=`d21b326c3a753d15`，构建基线 `git_head=186709b39bcdaf6af9974166dfbf617f71e9165d`，另含本轮功能改动；应用 raw/padded=1220592/1220608 B、静态 RAM 374464 B（余 83264 B）、入口帧最大 11528 B、oversized=0。`.px4` 777466 B，SHA-256 `5a80009b1667ee7bf725500649dfca1e026f2264dbb20bed579c285be4db7c74`。
- 原 PX4 bootloader COM12 识别 board 53/protocol v5，应用区上传/CRC 100%（25 s）；回到 COM26，`!platform` 核对新 source、`git=186709b…`、`fault_injection=false`。VMware 产品字符串仍是通用 USB Serial Device，不按缓存字符串判定 bootloader；由 uploader 的协议握手确认原 bootloader。
- **USB**：15.08 s 收到 **145 条 ALTITUDE（9.62 Hz）**，前四个高度字段 **145/145 有限**；local=relative，AMSL=monotonic；terrain/bottom 全为 NaN。独立 `pymavlink.common` 解析全部帧，时间递增、system/component=1/1，145 条 `ESTIMATOR_STATUS` 的垂直位 `0x24` 有效、水平位关闭。局部/全球位置帧均为 0，15 个心跳均 disarmed。
- **同次 SD**：`target/e7-log/02.ulg`，416768 B（下载 38.9 s），`pyulog` 无损坏、dropout=0；`vehicle_altitude` **73 条 / 14.52 s（4.96 Hz）**，符合日志参数 `LOG_EN_EST=1/LOG_EST_HZ=5`。逐字段 schema、32 B payload 与 local 时间戳一致；`z_valid/v_z_valid` **73/73 为真**，`altitude_local=-z`、relative=local、AMSL=monotonic 精确相等，前四字段均有限、terrain/bottom 均 NaN。baro aid 73 个快照 fused、0 rejected；at_rest 73/73 置位。
- SD 高度示例：monotonic/AMSL 首末 −10.231/−9.835 m，local/relative 首末 0.061/0.081 m。无 GNSS 全球原点时 AMSL 按 PX4 回退气压高度，不是经过独立参考确认的真海拔；负气压高度并非无效值。
- 收尾 `Filter/Running`、fault/numerics=None、`deadline_misses=0`、step 最大 1488 µs/2000 µs、发布 `missed=0`、TIM8 积分误差 0 ppm，`nav_qualified=false`。logger 已停止（status/stage=0/0，errors=0，normal/event dropped=0）；未设参/保存、删除旧日志或启用输出。

证据：新包目录中的 `platform.txt`、`ekf2-{before,after}.txt`、`logger-{before,after}.txt`、`mavlink.bin`、`usb-summary.json`、`sd-summary.json`；上传输出 `target/altitude-20261109/upload-d21b326c3a753d15.txt`。`02.ulg` SHA-256：`3a7c950822159f8d9e4d3e56eb4aff1f0a4276d4d234a8e86c9333053952c0d1`。实验脚本和二进制仅保留在忽略的 `target/`，不入源码库。

限制：本次只验真实高度数据链，不是高度精度、实物 aid 断线/恢复、TELEM1、完整 E7/M1 或飞行资格；这些阶段门继续保留。
