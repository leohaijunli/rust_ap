# Pixhawk 6X Rust 飞控 TODO

更新：2026-11-09。**板载已为 `32269dec1d8137b4` /代码 `8cddb79`：CAN 执行器优先级/丢帧诊断修复回装通过，CAN1 3 s CRC 0→0、FIFO lost=false，真实抢占增量 1744，EKF 峰值 1677 µs/2000 µs、无漏期/数值故障、CPU 35.3%、栈余 32584 B。原始 GPS 5 Hz、姿态/高度/健康 10 Hz；水平/全球位置发布护栏保持，CAN2 GUI 原场景尚未再次完整复核。蜂鸣暂停、RGB Nack/此前 PX4IO Timeout，IO/输出门及 M1/E7/RTK 未验依赖不放行。**
规则与必要测试：[plan.md](plan.md) · 证据：[P0](docs/acceptance/p0/README.md) / [P1](docs/acceptance/p1/README.md) / [P3a](docs/acceptance/p3a/README.md) / [P4b-a](docs/acceptance/p4b-a/README.md) / [P4b-g](docs/acceptance/p4b-g/README.md) / [PL-a](docs/acceptance/pl-a/README.md) / [P2](docs/acceptance/p2/README.md) / [P4](docs/acceptance/p4/README.md) / [PL-b](docs/acceptance/pl-b/README.md) / [MAVCAN](docs/acceptance/mavcan/README.md)

工程师随后授权源码拆分位置数据有效性与飞行资格；本轮仅软件检查/构建/推送，板载仍为上方 `32269dec1d8137b4`，不把新源码的发布策略写成已回装。

## 执行规则

- 只做[必要测试](plan.md#4-仅保留必要测试)，复用已有通过结果；相关实现变化或新失败才补测。
- `[x]` 有证据；阶段通过后放行依赖阶段。工程师负责实体操作，agent 负责实现和记录。
- 保持纯 Rust/Embassy、原 PX4 bootloader、`0x08020000` 入口和 QGC 恢复能力。
- 工程师已确认基站/CAN Rover 连接方案；H-10 的固件/节点/数据与 RTK 实测仍待验，H-02/03 等未验项保持。除明确授权外，不自主设参、刷写、采集或执行实体操作。
- 完成本次工作后提交推送；文档/计划修改只做内容与差异检查。
- 编写时用 `clean-code`，收尾时用 `code-simplifier`；[项目技能](docs/ai-coding-skills.md)已安装。

## 位置数据发布与飞行资格拆分 — 2026-11-09

- [x] 按核心位置/速度/全球原点有效性与运行时新鲜度发布数据-only位置；停用/陈旧/Mahony 回退撤销，航向控制资格/ARM/非零输出门不变。新诊断单独报告数据有效位与 `flight_qualified=false`。[记录](docs/acceptance/ekf2/20261109-position-data-validity.md)。
- [x] 新合成 GNSS 回归先在旧产品发布门失败，再通过；字段与原始核心位置一致，可构建 LOCAL/GLOBAL MAVLink，陈旧/回退/停用撤销。不是板上或绝对精度验收。
- [x] 产品 3 项/MAVLink 映射 2 项 host、fmt、host/普通 H753 Clippy 与局部审视通过；未改核心数学，不重复全量对拍。
- [ ] 完成普通 release/布局和提交推送；本轮不回装，新的实物位置消息仍待另行授权核对，飞行/RTK/控制资格不自动放行。

## CAN2 GUI 转发回归 — 2026-11-09

- [x] 与原 GUI 通过版本比对：CAN 帧/总线/引脚/租约代码除 crate 改名外一致；Rust 6 项 MAVCAN、Python 5 项协议检查通过。授权 3 s 实测却发现接收 2985/USB 21/丢弃 2821、队列满 64，保留失败，不把 host 通过当作硬件通过。[记录](docs/acceptance/mavcan/20261109-can2-router-regression.md)。
- [x] 最小调度修复已实现：CAN 到帧及 USB 可写唤醒，每轮最多 4 帧；控制/正常遥测优先，100 ms 帧寿命/租约/过滤和原输出门不变。fmt/H753 Clippy 与一次局部审视通过。
- [x] `f24b138` 已提交推送；普通包 `620736ef4e52c4d2` 构建/布局与原 bootloader 上传/CRC（25 s）、启动核对通过。3.016 s RX/USB=2996/2896、软件丢弃 0、队列末值 1，GUI 节点 125/身份已确认。
- [x] 接收修复前（`620736ef4e52c4d2`）CAN1/GPS 只读：GPS_RAW_INT 15/3.016 s（DGPS/29 星），姿态/高度/健康各 29 条，位置类 0；当次 CRC 54→55。已确认水平/全球发布护栏原因，未解除。[证据](docs/acceptance/mavcan/20261109-can2-router-regression.md)。
- [x] 工程师选择先修 CRC：CAN 执行器 P4→P3、FIFO/实际抢占诊断，fmt/H753 Clippy/局部审视通过；`8cddb79` 已推送，普通包 `32269dec1d8137b4` 构建/布局、原 bootloader 回装/CRC（25 s）通过。
- [x] CAN1 3 s 有界复核：GPS 15/估计流各 30 条，CRC 0→0、FIFO lost=false，实际抢占增量 1744；FPU/Filter/数值/漏期、1677 µs预算、35.3% CPU/32584 B 栈通过。Logger Disabled/蜂鸣 off/CCR=0，模块参数/节点 ID 与输出门未改。
- [ ] 保留旧 GUI CRC/toggle 和 RawIMU 缺帧失败；当前 GPS 在 CAN1，CAN2 GUI 原场景尚未再确认全部异常消除，未知类型/服务错误不冒称全零。水平/全球位置发布护栏未解除，不用吞吐恢复或软件队列丢弃 0 冒充端到端无损/飞行资格。

## 当前纯软件工作 — 2026-11-08

- [x] 气压日志新鲜度前置：陈旧/未来时间样本不消耗限流名额；原补偿、200 ms 新鲜度、参数/ULog 字段与 EKF 输入不变。旧顺序回归失败已保留，修后 4 项相关 host、fmt、普通 H753/host 库 Clippy 通过。[记录](docs/acceptance/pl-a/20261108-baro-log-freshness.md)。
- [x] 气压修复初始候选 `2a2b272fcd059412` 构建/打包/入口帧检查通过，未回装；此包不含随后追加的 Clippy 清理。[记录](docs/acceptance/pl-a/20261108-baro-log-freshness.md)。
- [x] 工程师追加要求清理 6 项测试 Clippy 告警：定长数组分块、精确 `0.7071f` 位值、条目顺序与编译期断言已修复；fmt、host 库/测试 Clippy `-D warnings` 与全套 218 项 host 通过，未改生产门限或算法。[记录](docs/acceptance/pl-a/20261108-baro-log-freshness.md)。
- [x] 合并普通包 `49b3552f38510053` 的 H753 Clippy、构建/打包/入口帧检查通过；RAM 与最大入口帧不变。软件结果与后续回装分别记录。[软件记录](docs/acceptance/pl-a/20261108-baro-log-freshness.md)。
- [x] 2026-11-09 单独授权回装 `49b3552f38510053`：原 bootloader board 53/v5 上传/CRC（25 s）、COM4/source/3 个 disarmed 心跳与平台/FPU 核对通过；227 项参数/历史故障保持，EKF Running/Filter、step 峰值 1223 µs、漏期 0。无日志/输出动作，不扩展为完整 E7。[回装记录](docs/acceptance/p1/20261109-log-guard-flash.md)。

## 启动 yaw 合成对照 — 2026-11-09

- [x] 工程师追加要求比较本项目与 PX4 原生的地磁/yaw 收敛。用同一合成静止场景（水平、磁航向 60°、IMU 250 Hz、aid 20 Hz、60 s）跑 Rust 宿主 EKF 与固定 PX4 参考的四个变体；Rust 与 PX4 默认 `control_status` 0 差异、yaw 最大差 6×10⁻⁹°，核心数值一致。[记录](docs/acceptance/ekf2/20261109-startup-yaw-compare.md)。
- [x] 地面标志（`at_rest=true,in_air=false`）使 tilt 对齐 7.032→0.576 s、yaw/mag 锁定 8.104→1.648 s；带陀螺偏置场景末值误差 +14.08°→+0.10°。重力融合不改变收敛时间，也不修复偏置漂移。差异定位为 Rust 缺少 land-detected/at_rest 与 ZVU/ZGU 通路；未改代码、参数或磁检查门限。
- [x] 工程师同意实现保守静止检测 + ZVU/ZGU（纯软件）：新增 `at_rest.rs`（陀螺 ≤0.05 rad/s、比力差 ≤1 m/s²、连续 1 s、许可=disarmed 且无输出），`ImuEkf` 置 `VEHICLE_AT_REST` 并按 PX4 顺序在航向更新后执行 ZVU/ZGU，宿主例程加 `--at-rest`。host 225 项（新增 7）、fmt/Clippy、release 与固件构建（source=`76dd5804fdfb9bf2`，栈 oversized=0）通过。[实现与对照记录](docs/acceptance/ekf2/20261109-startup-yaw-compare.md#后续实现保守静止检测--zvuzgu同日)。
- [x] 实现后同场景重跑：1° 内收敛 8.104→2.448 s（PX4 地面标志参考 1.648 s）；偏置场景末值误差 +14.077°→+0.130°（参考 +0.100°），漂移被抑制。比 PX4 慢 0.8 s 是保守静止判定的 1 s 保持窗口，属设计取舍。仍是合成输入结论，不用于绝对精度或飞行放行；未刷写、未设参。
- [x] 2026-11-09 工程师授权回装最新提交 `df8dcb8`（source=`ecc872b86f6a2e98`）：原 PX4 bootloader 上传/CRC 25 s；COM26（`1209:0001 / V6X-RUST-001`）启动，`!platform` 返回 `git=df8dcb84…`、TIM8 `psc=239 arr=65535`、备份域保留；`!ekf2` 为 `Filter/Running`、fault=None、updates 6312、`max_update_us=1467`（预算 2000）、`deadline_misses=0`、`integration_error_ppm=0`、发布 `missed=0`。本次未设参/保存、启停/删除日志或启用输出。[回装记录](docs/acceptance/ekf2/20261109-startup-yaw-compare.md#板上回装2026-11-09)。
- [x] 工程师关闭 QGC 后授权静止日志核对（板上 `df8dcb8` 正向保守机制）：`00.ulg` 68.6 s，`VEHICLE_AT_REST` 在 341/341 快照置位，gyro z 偏置 −0.938 °/s 与原始均值 −0.936 °/s 相符；yaw 全窗变化 0.21°。未覆盖 0→1/实物扰动，不作绝对航向或飞行资格。[记录](docs/acceptance/ekf2/20261109-startup-yaw-compare.md#后续授权静止日志与-yaw-核对同日)。
- [x] 工程师质询"为何不取 PX4 设置"后查证固定 commit 源码：PX4 `at_rest` 来自 land detector 的反向计时（默认静止、角速度 >3°/s 或振动超限重置、disarmed 即 landed）。经工程师选择改为等价移植（比力模长差 >1 m/s² 作为振动指标简化替代）；首轮正向保守实现（1° 内收敛 2.448 s，慢 0.8 s）结果保留在验收记录中作对照。host 225 项、fmt/Clippy、release 与固件构建（source=`197fb10f367d6ae2`，栈 oversized=0）通过。[实现与对照记录](docs/acceptance/ekf2/20261109-startup-yaw-compare.md#机制修正改为-px4-反向计时同日工程师确认)。
- [x] 反向计时后同场景重跑：at_rest 首拍 8 ms 置位，tilt/yaw 对齐 0.576/1.648 s 与 PX4 地面标志参考逐项一致；偏置场景末值误差 +14.077°→+0.051°（参考 +0.100°），窗口均值/最大均略优。仍是合成输入结论，不用于绝对精度或飞行放行；未刷写、未设参。
- [x] 反向计时实现随后随 USL 诊断版 `d884e6ba1adf41b7` 授权回装；原包 `197fb10f367d6ae2` 未单独回装。本轮未专项复验静止判定，不借用 `00.ulg` 正向机制的实测作为新机制验收。

## ALTITUDE 与垂直有效性 — 2026-11-09

- [x] 共享 PX4 ALTITUDE 映射、MAVLink #141/component 1 与 ULog `vehicle_altitude`（ID 43、32 B）已实现；复用 `MAV_RATE_EST` 与 `LOG_EN_EST/LOG_EST_HZ`，未知保持 NaN。[记录](docs/acceptance/ekf2/20261109-altitude-output.md)。
- [x] 根因确认：baro 已融合时核心垂直 dead-reckoning 正常清除，但 `product::snapshot` 的 E7 资格护栏强制清零高度有效位。仅修正高度发布门，保留回退/陈旧/停用与水平/全球/控制护栏；新增回归先失败后通过。相关 host 72/7/3 项、fmt、host/H753 Clippy 通过，核心融合逻辑未改。
- [x] 普通包 `d21b326c3a753d15` 构建/布局/入口帧与原 bootloader 上传/CRC 25 s 通过；RAM 374464 B，入口帧 11528 B、0 超限。USB 145 条/15.08 s ALTITUDE 前四字段全有限；同次 SD `02.ulg` 73 条、`z_valid/v_z_valid` 全有效、逐字段关系/独立解析通过，无损坏/dropout。
- [x] 收尾 Filter/Running、fault/numerics=None、step 峰值 1488 µs/2000 µs、漏期/发布 missed=0、积分 0 ppm，logger 已停止；未设参/保存、删除日志或启用输出。[板载证据](docs/acceptance/ekf2/20261109-altitude-output.md#板载验证)。
- [ ] TELEM1、独立高度精度与实物 aid 失效/恢复、完整 E7/M1 仍按原阶段门待验，不由高度数据链放行。

## H-RTK 步骤与 crate 改名 — 2026-11-09

- [x] 固定基站＋QGC＋单 CAN Rover 的[独立步骤](docs/rtk-can-validation.md)已细化：节点/普通定位、基站 Survey-In、RTCM 软件、地面站出站证据、Fixed/日志与中断恢复；本轮只成文，未执行或改写阶段依赖。
- [x] Package/bin 改为 `v6x-rust-ap`，Rust 导入与 Windows/Linux/CI 产物路径同步；229 项 host、相关 Clippy/工具与普通包通过，source=`136329bb61f63100`，未回装，旧记录与 USB 身份保持。[改名记录](docs/acceptance/p1/20261109-crate-rename.md)。
- [ ] 下一步先确认基站型号/固件、Rover CAN1 节点与真实 Fix2；RTCM 转发与专用诊断、必要的节点分配、RM3100 接收待另行授权实施，正式 RTK 验收仍依赖 M1/P2/P4b-g/H-10。

## CUAV USL 日志控制 — 2026-11-09

- [x] PF5 按键从 IO-ready 错误耦合中解开，按 disarmed/输出关闭接受；短按启停、长按仅事件，原 IO/安全 LED/PWM 护栏保持。9 项按键/试音及 2 项 RGB host、fmt、相关 Clippy/构建通过。[记录](docs/acceptance/pl-a/20261109-usl-logger-switch.md)。
- [x] 授权回装试灯包 `2b7dd895b8cee3d3` 和长试音诊断版 `d884e6ba1adf41b7`：应用区上传/CRC、source与时序核对通过；RGB I2C1/0x39 协议响应，按键新日志 55296 B独立下载/解析无损坏/dropout，收尾 Logger Disabled、beeper off/CCR=0。
- [x] 按工程师要求完成局部清理：RGB 仅保留实测路径、共享写入/错误处理，console 与试音常量简化；相关 host/Clippy、fmt/diff 和普通构建/布局/入口帧通过。新包 `094b08072d1267ac` 先独立保存，软件检查与板载失败分别记录。
- [x] 工程师追加 push/flash：代码 `736d4aa` 先推送，再经原 board 53/v5 bootloader 回装 `094b08072d1267ac`（25 s 上传/CRC）；source/3 条 disarmed 心跳、VTOR/FPU/时钟与备份域核对通过。Logger Disabled、RGB schema=2/errors=0、蜂鸣 off/tests=0/CCR=0；未试音/设参/日志或非零输出操作，不作完整实物验收。
- [ ] 蜂鸣短/长试音均未听到，工程师已暂停；提供的 pin7 表与 Holybro标准GPS1资料冲突，后续只在可靠两端线序/供电/波形核实后另行授权继续，不盲驱 PD10/不解除 safety。
- [ ] RGB 实际灯色/完整 H-12、PX4IO Timeout及原阶段依赖仍待验；协议响应和累计按键计数不冒充完整物理验收。

## 板载地磁 — 2026-11-07

- [x] BMM150/I²C4 驱动与主 EKF2 磁事件入口、标准地磁日志/遥测/诊断已接入；共用 BMP388 总线而不切断共享电源。host 196 项、256 组固定 PX4 补偿对拍、pyulog、Clippy/元数据与普通包通过，source=`cd136580311b4298`（RAM +292 B、入口帧不变）。[软件记录](docs/acceptance/magnetometer/20261107-bmm150.md)。
- [x] 2026-11-08 磁源机制：四源 DeviceId/内外属性、CAL_MAG0～3 绑定与矩阵/旋转、优先级健康选择、失效回退/降级、只重置磁历史；209 项、双源 pyulog、Clippy 与普通包通过，source=`2c58e1faa86f4f72`。[记录](docs/acceptance/magnetometer/20261108-selection.md)。
- [x] 已授权回装并实测 BMM150/BMP388 共存、227 项参数保持、短日志/磁遥测；驱动错误 0。[板载记录](docs/acceptance/magnetometer/20261108-bringup.md)。
- [x] 失败项/对称性阶段诊断：host 210 项、fmt/Clippy 与普通包通过；授权回装 `2de8a1b91a3d467d`、`95b1b57207604874`，分别观察到不对称、负方差及非有限协方差，未清故障或保存参数。[续测记录](docs/acceptance/magnetometer/20261108-bringup.md#失败项诊断续测)。
- [x] 授权只读诊断回装 `3e8d1725a4f91020`：FPCCR=0 / ASPEN=false、Nearest、无 FZ/DN；开机更新 823 次后负方差。确认上下文保存配置缺陷，不归因于高 CPU。[记录](docs/acceptance/magnetometer/20261108-bringup.md#fpu-只读核对与修复准备)。
- [x] FPU 修复对照：`da2bd6595cb3de1c` 启动设置 ASPEN=1/LSPEN=0；两种 Joseph 更新各 7 组 ARM 镜像指令同寄存器且无中间覆盖，host 210 项/Clippy/普通构建通过。授权 60.5 s 对照，updates 7684→13712、磁融合有效、fault/numerics=None；CPU 32.2–32.4%、step 峰值 1403 µs/2000 µs、漏期 0、栈余 25,592 B。[修复记录](docs/acceptance/magnetometer/20261108-bringup.md#fpu-修复板上对照通过)。
- [x] 同类遗漏软件修复：普通/备用入口共用 `src/fpu_context.rs`，保留 P0 时钟和旧故障；偏角融合补固定 PX4 的 `S<R` 拒绝/故障位及恢复。212 项 host、1001 点固定参考状态/协方差/NIS 对拍、fmt/相关 Clippy 与两配置构建通过；普通 `3de26249db0a54d7`、dump `9243a9b7e1a154e9` 为日志修改前的软件验证包，本轮未回装。[软件记录](docs/acceptance/magnetometer/20261108-bringup.md#同类遗漏软件修复)。
- [x] 防回归原则已写入 `.opencode/skills/rust-embedded/SKILL.md`：统一入口、ABI、真实浮点抢占验证、镜像单次计算及坏方差保护；仍受必要测试/工程师授权约束，不把离线通过当作硬件证明。
- [x] 合并普通包 `81fd0d06601f9e2f` 经授权原 bootloader 回装；只读检查 ASPEN=true、Running/Filter、fault/numerics=None、step 峰值 1302 µs、漏期 0。备用 dump 新初始化未上板，自动化抢占测试未新增；不扩展为完整 E7。
- [x] 工程师报告 yaw 恢复稳定，要求停止根因调查并重测；最新只读窗口 Running/Filter、fault/numerics=None、漏期 0，未改算法/参数。磁融合布尔快照不当作成功率；不放行安装校准或飞行资格。[复测记录](docs/acceptance/magnetometer/20261108-bringup.md#yaw-只读复测与调查停止)。
- [ ] 机体安装校准、外接磁驱动与真实切换、完整 E7/M1 仍待验；IMU/TIM8 时基差独立跟进，不由此次 Numerics 修复自动收口。

## 2026-11-07 状态检查与回装

- [x] 工程师授权回装 `a17a2d35bab6f05a`：原 bootloader 上传/CRC 校验、Quadrotor/disarmed/平台与备份域通过；165 项值/类型/顺序及历史故障保持，未设参/保存/启动日志。[回装记录](docs/acceptance/p2/20261107-qgc-flash.md)。
- [x] 元数据两文件只读下载/CRC/精确字节核对通过；工程师确认 QGC 5.0.3 类型、示例标签及设置说明正常。本轮显示子项收口，不等于完整 P2/P3/M1 验收。
- [x] H-04 基线（32 GB 普通卡）：65 s 零输出日志 1,654,784 B、25.31 kB/s，3 次各 5 ms dropout、正常丢失 8 条（0.026%）、事件 0，pyulog 可解析。[基线记录](docs/acceptance/pl-a/20261107-h04-baseline.md)。
- [x] H-04 300 s 分布：7,528,960 B、25.06 kB/s，正常丢失 32 条（0.0225%）、事件 0；发现 dropout 时长在时间戳交错时饱和为 65535 ms（6/12），已改 `saturating_sub` 并加 host 用例，188 项通过；新包 source=`f1825489b4f35661`。[记录](docs/acceptance/pl-a/20261107-h04-300s.md)。
- [x] H-04 回装与 300 s 复测：原 bootloader 回装 `f1825489b4f35661`（VTOR/时钟/备份域/165 参数/历史故障保持）；300 s 分布 17 次 dropout 全部 ≤6 ms（合计 46 ms）、正常丢失 34 条（0.0239%）、事件 0，按 ≤10 ms/事件 0 窗口通过。[记录](docs/acceptance/pl-a/20261107-h04-300s-r2.md)。
- [x] H-04 受控断电：写日志中断电后日志 id 24（17,832,448 B、712.85 s）完整可解析、`file_corruption=false`；VTOR/备份域/165 参数/历史故障保持。会话内出现 1 次 152 ms 停顿（48 次 ≤6 ms、事件 0），超出 ≤10 ms 窗口，待工程师裁决换卡或放宽。[记录](docs/acceptance/pl-a/20261107-h04-power-loss.md)。
- [x] H-04 缺卡启动：无卡时 heartbeat 在线、`logger start` 进入 Error(stage=2) 不挂死、列表请求回 "Log SD mount failed"、无新 fault；插回卡后旧 25 条日志完好、`logger start/stop` 恢复正常（验证短会话 id 25 不计入统计）。卡面确认为 **A1 HC1**（此前"无 A1/A2 标注"记录修正；A1 保证随机读 IOPS 而非无长写停顿，152 ms 事件仍按窗口裁决）。[记录](docs/acceptance/pl-a/20261107-h04-nocard.md)。
- [x] H-04 写日志中拔卡：写入中拔卡 ≥5 s → logger 进 Error(stage=12)、errors+1、无 panic/重启、`SYSTEM_TIME` 连续；断电恢复后 28 条旧日志完好（id 27 大小与计数器逐字节一致）、头部 2 MiB 解析无损坏、新日志可启停、参数/故障不变。首次 <1 s 拔插被完全吸收（0 写失败）。[记录](docs/acceptance/pl-a/20261107-h04-card-removal.md)。
- [x] **H-04 收口（2026-11-07 通过）**：最终分布 300 s r3 = 10 次 ≤5 ms（合计 20 ms）、正常丢失 0.0141%、事件 0，按 ≤10 ms/事件 0 窗口通过；断电 ×3、缺卡、拔卡 Error 路径全部板载实测通过。边界如实移交：自动重启板载目击待实验台 USB 固定后补验；152 ms 长尾已由 1024 槽扩容缓解（A1 卡未达 PX4 可靠卡档）。[收口记录](docs/acceptance/pl-a/20261107-h04-closure.md)。
- [ ] 下一步 **H-02（ESC）**：工程师把 ESC 接上 CAN1 并供电（桨叶已拆、零输出），主机用 `!dronecan` 只读枚举节点身份/index，然后按 P4b-a CAN-A04 做零输出 Status 与断线恢复实测。

- [x] 工程师授权修正参数说明并审核其余参数：165 项均有使用状态/设置含义，数值/类型/默认/范围/分组逐项不变；固定 PX4 描述与运行时读取分开，去除整组 `unmapped` 误导。[设置指南](docs/parameter-settings.md) / [本轮记录](docs/acceptance/p3a/20261107-parameter-help.md)。
- [x] 参数说明普通包与最后元数据 CRC/布局核对通过，source=`a17a2d35bab6f05a`；RAM/最大入口栈帧不变。未设参/保存/烧录。
- [x] GitHub 凭据问题已解决：仓库配置 `credential.username=leohaijunli`，此前积压提交已推送；后续沿用正常非交互推送。

- [x] 工程师授权推进最小 QGC 兼容：Quadrotor 身份、真实 IMU/baro/GPS/AHRS 健康位与[能力矩阵](docs/qgc-compatibility.md)已实现；7 项新用例、fmt、host/普通 H753 Clippy 通过，解锁/输出门未改。[本轮记录](docs/acceptance/p2/20261107-qgc-minimal.md)。
- [x] 新普通包构建/布局通过，source=`fdf7fcd1ff9d10c7`；static RAM=`254224 B`、最大入口栈帧=`11496 B` 不变。未自动烧录。
- [x] QGC 版本/回装授权及类型/参数说明用户可见核对已完成；健康位由同次协议读取覆盖，不启用非零输出。

- [x] 从干净的 `aaba83b` 当前源码构建普通包，原 board 53/protocol v5 bootloader 上传与 CRC 校验、COM4 身份/3 个 disarmed 心跳/平台与备份域核对通过；没有修改固件代码或 IO 应用。[本次记录](docs/acceptance/p1/20261107-current-firmware.md)。
- [x] 只读快照：主 `Filter/Running`、fault=None、max_update=559 µs、期限超时=0、积分 0 ppm；继承参数启用状态，未设参/保存、未清旧 panic、未启动新日志。
- [x] 阅读 P0–P3 产品方案并完成[评估与下一步建议](docs/p0-p3-review-and-next-plan.md)；仅文档建议，未更改现行阶段门或实施控制/非零输出。
- [ ] 硬件主线按 H-04/02/03/10 与当前传感器/IO 待验项推进，再验 M1；QGC 板载/GUI 子项已收口。详见[下一步计划](docs/next-steps.md)。

## 单实例 EKF 与 IMU 时基 — 2026-11-06 记录

### 2026-11-06 工程师追加要求

- [x] 删除独立无 GNSS 第二实例及 component 197 / ULog instance 1，主 EKF2/Mahony 保留；示例、解析器和 CI 同步单实例。
- [x] 改用 PX4 的 FIFO 梯形均值与 MCU 批间隔积分；按追加要求切换 TIM8，FIFO 时间戳采用 `16 × 32/30` 换算。修复 DRDY 重复回扣和丢失未断序，首批/故障后重建基准。[调查与实现](docs/imu-timebase.md)。
- [x] 核心 180 项、标准输出解析器 7 项用例通过，原始 8 kHz / 批次积分 A/B 满足既定误差门限。[本轮记录](docs/acceptance/ekf2/20261106-single-estimator-timebase.md)。
- [x] 完成 TIM8/时间戳换算的受影响项复验、Clippy、标准输出 fixture 和普通固件打包/布局：fmt/Clippy（4 组特性 + host 示例）通过，标准输出 7 项通过，打包 source=`eb0d589438e0a6dc`、最大入口栈帧 11496 B。[本轮记录](docs/acceptance/ekf2/20261106-single-estimator-timebase.md)。
- [x] 板载烧录与 65 s 采集（工程师授权）：TIM8 运行、主 EKF2 选 Filter 且 fault=None、`max_update_us=556`、`deadline_misses=0`、1 kHz 发布 `missed=0`、`integration_error_ppm=0`；PX4 换算后传感器时间残差 `-32270 ppm`（不进入积分）。共享栈最低观测 32144 B，按工程师指示仅记录不作门。旧 panic 记录保留，`!ekf2 stop` 已收尾。
- [x] 2026-11-08 状态核查确认：本轮改动已随后续提交推送（凭据阻塞已解决，仓库与 origin/main 同步），此项收口。
- [x] 共享栈判据放宽（工程师确认，2026-11-06）：untouched ≥16 KiB（原 ≥32 KiB）。依据 PX4 v1.17.0 Kconfig 默认 WQ 栈：INS0（EKF2）6000 B、SPI 2392、hp_default 2800、uavcan 3624、lp_default 3500、tty 1728、manager 1280，等效路径合计 ≈21.3 KB 拆在 ~7 个独立任务栈；本项目单共享栈实测 touched 33392 B 同量级。`p1_capture.analyze`/`ekf2_runtime_capture` 两处门已参数化同步，[决策记录](docs/next-steps.md)。
- [x] 按新判据完整 65 s 采集 **passed=true, issues=[]**：`Filter/Running`、`deadline_misses=0`、`max_update_us=560`、`integration_error_ppm=0`、发布 `missed=0`、栈最低 30728 B、CPU 28.7%。P1 空载 10 µs release 门不再并入 E7 负载判定。见[本轮记录](docs/acceptance/ekf2/20261106-single-estimator-timebase.md)。TIM2 32 位驱动评估见 [时基说明](docs/imu-timebase.md)，暂不执行；传感器 −3.23% 时间残差归因保留 FIFO tick 分布证据后再投入。

### 2026-11-06 dropout 缓解（工程师授权，按 1–3 层执行）

- [x] 缓解选项成文：[dropout 缓解与极端优化备选](docs/logging-dropout-options.md)；第 4 层（双优先级队列、裸块环形日志）明确"仅 1–3 层与 H-04 之后仍超标才评估"。
- [x] 第 1 层：`dronecan_raw_command` 主题从 ULog 注册表与 SD 写入移除（`TOPIC_COUNT 14→13`、ID 重排连续、按名匹配保持旧日志可解析）；DroneCAN 总线零命令发送不变。host 180 项、H753 Clippy、打包 source=`8d1ae82d5f5d13b3`（static RAM 254224 B、最大入口栈帧 11496 B）通过。
- [x] 第 2 层核查：固定 `embedded-fatfs 518528c` 无 `set_len`/预分配接口，写入器已是 32 KiB 攒批 + 512 对齐整块写；软件侧到顶，剩余停顿归第 3 层卡选型。不改代码，不引入 fork。
- [ ] 第 3 层：H-04 卡选型判据（A1/A2 或工业 pSLC + 写延迟分布实测）仍待工程师输入；当前 32 GB 普通卡不满足选型标注，65 s 基线已记录。
- [x] 重刷验证：source=`ad99705b817a8c7f`，12 s 采集 `rust_runtime_health`（≈10 Hz）与 `rust_can_stats` 并存、dropout=0、无损坏；EKF2 正常。结果补录到[本轮记录](docs/acceptance/ekf2/20261106-single-estimator-timebase.md)。
- [x] 顺带修复历史 bug：`record_can_stats` 误用 `Group::Health` 限流器，`rust_runtime_health` 被 10 Hz can_stats 饿死（run1 2 Hz / run2 0 / run3 仅前 0.9 s 的三轮数据全部吻合）；改回 `Group::Can` 后板验通过。
- [x] E7 USB 流核对：各流速率符合参数，LOCAL/GLOBAL_POSITION 按"无效不发"抑制；USB+SD 成对证据齐备。E7 软件门（不含实物依赖）收口，见[本轮记录](docs/acceptance/ekf2/20261106-single-estimator-timebase.md)与[下一步计划](docs/next-steps.md)。

### MAVLink 协议补齐与参数/存储重构（2026-11-05 工程师授权，软件完成/板载待验）

需求与计划见 [plan.md §8b](plan.md#8b-mavlink-协议补齐与参数存储重构2026-11-05-整合需求)。

- [x] **P0：PARAM_REQUEST_LIST 流式 + INT32 类型**：全量参数改为游标逐发（`next_param_value`，两链路 transmit 每 tick 一个），回包按 `Kind` 发 `INT32`/`REAL32`，`PARAM_SET` 同时收两者。**板载已验证：一次 `PARAM_REQUEST_LIST` 收齐 159 个参数、0 次补拉（`single_list_stall_retries=0`），source=`b623b18f05e5f694`**。初版共享游标被 USB/UART7 两链路争抢致丢参，已改为每链路独立游标。host 167 测试全过。
- [x] **P1 实现/核心检查**：500 ms 静默窗口、100 ms 保存轮询；单次校验，估计器拒绝后回滚；归属判据由 EKF 生成器产出。防抖窗口直接用例通过。
- [x] **P2 实现/核心检查**：FRAM v2 按名字 CRC-16 存非默认值；旧 11/159 项 v1 前缀记录可迁移；注册表宏生成 Id/typed getter/hash，分组由编译产物生成 QGC JSON，长度/碰撞及 stale 校验通过。新增 6 项速率参数后共 **165 项**。
- [x] **P3 实现/核心检查**：唯一 Router 任务；双链路独立游标/FTP/流间隔与会话；流表、511/510、统一 REQUEST_MESSAGE 和非阻塞 `telemetry::publish`。补 BATTERY_STATUS 与有效姿态/位置；ARM/模式仅拒绝应答。状态文本区分 off、EKF2、主动 Mahony、Mahony 回退和无新鲜姿态。
- [x] **本轮软件检查**：host **180 passed**；普通/fault-injection/fault-dump 与 host Clippy（`-D warnings`）通过；EKF 生成器与 QGC 元数据 `--check` 通过；已做一次局部 `code-simplifier` 审视。
- [x] **普通包/布局**：`tools/build_firmware.ps1` 通过；source=`f5756b75d6ed78bf`，raw/padded=`1105864/1105888 B`，static RAM=`276656 B`，最大入口栈帧=`11560 B`（0 超限，不是完整调用链/板载水位证明）；board ID 53、入口 `0x08020000` 保持。包：`target/thumbv7em-none-eabihf/release/v6x-rust-led.px4`；SHA-256=`096638626b96e0b097fdb04189fcdf12592e5947d1fa0d3b25330ec6e2622b21`。
- [x] **本地提交**：实现与验证记录已提交 `3a1bc08`。
- [ ] **推送阻塞**：首次 `git push` 120 s 超时；非交互重试因 Git Credential Manager 无可用凭据失败（`unable to get password from user`）。已确认远端仍为 `c12d03b`；工程师完成 GitHub 凭据授权后执行 `git push`。
- [x] **2026-11-06 刷写/USB 最小核对**：工程师授权后经原 board 53/protocol v5 bootloader（COM3）上传/CRC 校验成功，25 s；回到 COM4，普通版 source=`f5756b75d6ed78bf`、disarmed、VTOR=`08020000`，一次读表收齐 165 项/0 补拉。`LINK_LED_HZ=4` 与 `EST_EN/EST_ALGO/EKF2_EN=1/1/1` 保留，主实例实际 `Filter/Running`。两槽仍是 v1（11 项/gen 3、159 项/gen 4），RAM 已兼容载入，dirty=true/pending=false；未保存、未清故障、未另行启用估计器。证据：`target/flash-verify/20261106T143035.356888Z/`。
- [ ] **板载门未过**：共享栈余量 `32560 B <32768 B`（guard=true）；独立实例 `Mahony/Fatal`、`Filter(Numerics)`；IMU/TIM2 诊断偏差 `-92887 ppm`，根因未定。旧 source=`f509ec6b85b6a4fa` 的 panic 记录保留。先收口这些 E7/资源问题；H-03 具备后补双链路/重连、按名/索引、合法/非法设值、延迟保存及重启读回、间隔不串线与 QGC 恢复。不放行 M1/E7。

边界（工程师确认）：ARM/DISARM 与 DO_SET_MODE 只应答不动作（归 P5a）；RC/SERVO 无硬件不发。

### EKF 重构 — 方案 A1（2026-11-05 工程师授权，实施中）

四阶段内容与验收：[EKF 运行时重构计划](docs/ekf2-runtime-refactor.md)。

**当前：阶段 3 板载门未过，阶段 4 未开始。最近板载包 `f5756b75d6ed78bf` 的主实例 Running/config_errors=0，旧独立实例 Numerics/Fatal、共享栈不足 32 KiB、原始 IMU/TIM2 偏差的记录保留。当前源码已取消第二实例并使用 TIM8 批次积分，等待新版本板载核对。`335f484ffd467691` 是此前未回装的软件产物，不补写其验收。** [此前证据](docs/acceptance/ekf2/20261105-runtime-refactor.md)

- [ ] **阶段 0 / M0：护栏**：参数、延迟配置、单次超时继续运行的软件用例通过；DWT 分路径 max/p99 已接，实测数字待新固件采集。
- [x] **阶段 1 / M1 历史软件门**：A1 双实例、三种 `NavFilter`、产品别名、原地初始化及旧入口适配曾通过；当前按新范围收敛为单产品 Runtime、EKF2/Mahony 和统一 `ImuDelta` 入口。
- [ ] **阶段 2 / M2：1 kHz 与事件量测**：TIM7/P3 轻量发布、TIM5/P4 独占融合及有界事件入口完成，回放/输出软件检查通过；板上发布时序及 mag/range 实物融合待验。
- [ ] **阶段 3 / M3：共享预处理与恢复**：9 项 Runtime 专项通过；共享 8→1 kHz 多轴旋转 A/B 的最大差为姿态约 0.288°、速度 0.00582 m/s、位置 0.00363 m。原地初始化已消除 23/24 KiB 嵌套构造帧；IRQ 扣除、板载 CPU/栈及时基仍待验。
- [x] **阶段 3 板载配置激活**（2026-11-05）：首次普通包 EKF 未激活（`config_errors=1`）。加入 `config_stage` 阶段码与主机复现/板载启停循环定位到**配置后的健康检测瞬态故障**（参数每次读稳定、阶段码随周期随机）。间隔 1 ms 自动重配（最多 3 次）后 8/8 周期 `config_errors=0` 全绿；间隔 10 ms 出现 2 次真实失败，证明与重试时序耦合。当前以 1 ms 重试为防御，`config_retries` 计数可见。**已知问题：根因未定，疑为参数多源头并发访问（`ParameterManager.values` 用 `NoopRawMutex` 无锁 `RefCell`，被 `!ekf2`/`PARAM_SET`/DroneCAN/`main` 共享），待另开分支核查。** 本轮普通包 `282121108641b649`，8/8 全绿证据在 `target/ekf2-config/282121108-run1`。
- [ ] **阶段 4 / M4：稳定后优化**：H 稀疏化、小角度、协方差/内存及 FPU 指令路径；在前述行为/实时性门通过后实施。

2026-11-05 历史软件检查（本轮结果见上方）：
- [x] 45 项原有 estimator host 用例、9 项 Runtime 专项、1 项无 GNSS 编译失败用例通过。
- [x] 新分步入口 16 个固定 PX4 场景：29214 行状态/P、16000 行输出及对应创新/复位检查通过；旧入口 GNSS/range 回放通过。
- [x] GNSS / no-GPS 标准输出各 26000 输入、1300 发布、165 字段，经 pyulog/pymavlink 检查通过。
- [x] ULog 注册表长度修复后 7 项相关用例通过；host/H753 Clippy 与构建工具检查通过。
- [x] 新 H753 包：raw/padded=901928/901952 B，static RAM=285568 B；入口/分区/打包通过，最大入口栈帧 11208 B。完整共享栈水位须上板确认。
- [ ] 完成最终调用链/FPU、板载资源与时基核对。`f5756b75d6ed78bf` 已回装至 COM4；本次只读诊断不代替 1 kHz/CPU/栈完整验收，Git 推送仍受凭据阻塞。

MAVCAN 已完成收口；P4b-g 软件已刷入，下一步在 H-10 与依赖门具备后验证 GPS 实物数据。

**2026-11-04 历史板载：`34ef7adba931a677`（诊断包）。E6-H/PX4IO 合并已推送 `1c34dcd`；栈整改包 `f9d3004c0cf4691e` 实测启动余量 44524 B（门通过），但 EKF 更新 1117 µs 超 1 ms 期限后自动回退 Mahony。性能整改后（TIM5 独立执行器 + opt-level=3）发现 reconfigure 窗口内存损坏：参数被写为 230/290 等垃圾值，EKF2 无法激活；损坏定位到 `enable_gnss` 窗口，软件越界写（非硬件），EKF `update()` 路径 14 万样本无损坏。**[分析链](docs/acceptance/ekf2/20261104-e7-bringup.md)。日志/欧拉角输出已验证（858 KiB 完整下载，pyulog 无损坏）。PX4IO Timeout/实体按键禁用仍未解决，不称 E7 完成。

- [x] **E6-H 标准输出/日志 host 门**：独立无 GPS 输入类型、状态隔离、标准姿态/角速度/局部/全球/诊断 schema、MAVLink 编码与 pyulog/pymavlink 独立解析通过；无 GPS 只发独立 `ODOMETRY` / 健康状态，水平无界协方差明确，全球位置禁发。未启用固件 estimator 日志注册表或输出任务。[证据](docs/acceptance/ekf2/20261104-e6-host.md)。
- [ ] **E7 真实输出/日志**：165 项统一参数（142 个 EKF2）、`!ekf2 start/stop`、气压输入、回退与双链路/日志分发已接入；旧包 `f5756b75d6ed78bf` 主实例板载 Running。新单实例/TIM8 积分版须核对启停、周期、共享栈、真实输出及日志；此前失败保留为历史结果。完整资格仍依赖 M1、P6a、P4b-g、P2、PL-b。

### MAVCAN（历史已完成，本次回归待复核）

- [x] 固定并核对 VimDrones `7bfe9963` / pydronecan `092d09cb` 协议：经典 DroneCAN 用 `CAN_FRAME`（386）、`MAV_CMD_CAN_FORWARD`（32000）握手，ID bit31=扩展、bit30=RTR。
- [x] 修正 CAN2/FDCAN2（PB12/PB13）桥接，补全总线/目标/长度校验、转发租约、过滤、固定队列、超时取消及 USB 会话清理。[证据](docs/acceptance/mavcan/README.md)
- [x] MAVCAN 核心/协议测试、fmt/Clippy、普通固件构建与资源核对（source=`a2b91d35630180da`）。
- [x] **烧录核对 CAN2→USB 经典 `CAN_FRAME` 转发通过**（`usb_frames=10`，匿名 Allocation 请求），**DroneCAN GUI 节点可见性与数据通过**：工程师修正 GUI bus number（CAN2=bus 2）后，GPS 节点在 GUI 中可见且数据正常显示；Allocation 分配与出向 GetNodeInfo 已验证，`tx_frames` 递增确认双向收发。MAVCAN（CAN2）已收口。SLCAN 保留，未接入。

### P4b-g（当前）

- [ ] **P4b-g / GNSS-01～03**：软件实现完成并已刷入板载（source=`588ab77941985319`，raw/padded=412624/412640 B，static RAM=268680 B）。Fix2(1063)/Auxiliary(1061) 纯 Rust 解码与 P0 参考向量对拍（4 项 host 测试）；`process_fix2` 按固定 PX4 `gnss.cpp` process_fixx 翻译（covariance 1/6 有效、Auxiliary 2 s DOP、UTC 换算、未知字段不虚构）；标准 `sensor_gps` ULog（完整 37 字段，147 B）+ `GPS_RAW_INT`（USB/TELEM1 5 Hz，按 PX4 GPS_RAW_INT.hpp 换算，断链 1 s 后 1 Hz NO_GPS 帧）；`!dronecan` 增 GNSS 诊断行。日志队列扩容（RUNTIME_RECORD_LEN=152、QUEUE_BYTES=168 KiB、MAX_RECORD_LEN=800）支持完整标准字段。**板载已验：NO_GPS 断链帧 1 Hz 逐字段符合 PX4 语义，发送链路连通**；[证据](docs/acceptance/p4b-g/README.md)。待验：GPS 连接后正常帧速率与数值、sensor_gps 日志 pyulog 解析、一次断线失效/恢复、TIME-02 授时；dropout 复验并入联合负载。依赖 P4b-a、PL-a、H-10。

## 已确认范围

- [x] **D-A3 / D-RUST / D-EMBASSY**：当前工程 + Rust/Embassy + 按需复用固定 Cybflight 算法。
- [x] **D-M1 / D-QGC-M1 / D-PARAM**：IMU、AHRS、DroneCAN、ULog、USB/TELEM1、QGC 数据和标准参数读设。
- [x] **D-IMU / D-ESC / D-GNSS**：单 SPI2 ICM-42688-P；4 个 Vimdrones Nano / Quad X；H-RTK DroneCAN SKU12059。
- [x] **D-MESSAGES**：PX4 标准数据消息/ULog 字段 + MAVLink `common.xml` 标准消息与命令；[映射](docs/message-contracts.md)。
- [x] **D-INA219 / H-11 接线配置**：SEN0291，独立 I2C3 插座，拨码 A0=0/A1=1 即 0x44（固件自动探测 4 个地址）、10 mΩ、±8 A，测量独立支路；工程师确认已连接。
- [x] **D-USL**：工程师确认 CUAV USL 已接 GPS1，要求复用全部已核实可控的灯；按键用于日志、长按仅保留事件，不伪造 safety/arming。IO 安全灯与可能的 I²C RGB 灯分开识别，不按三个 GPIO 猜测。
- [x] **D-PUBLIC / D-HOLSATUS / D-DEFER**：允许 GPL 来源并记录许可；Holsatus 构建修复暂缓；远期细节待 M1。
- [x] **P0-01～10 / H-01**：软件基线、来源、硬件资料与 A3/M1 方案已归档。

## P1 — 保持 QGC 恢复能力的 bring-up

依赖 P0；[当前结果](docs/acceptance/p1/20261031-hil.md)。

- [x] **P1-02/03**：构建/打包、source/协议、平台和备份域通过；[软件证据](docs/acceptance/p1/20261031-backup-domain.md)。
- [x] **P1-04 已测**：两组 65 s 时序/CPU/栈、两类重启及 panic/HardFault 保持/清除通过。
- [x] **独立 QGC 恢复**：USB 重插、Stable 上传/校验和 PX4 v1.17.0 运行已留档。
- [x] **P1-01 / H-05 / P1-04 收口**：21:28 UTC 从 PX4 直接回装普通包，CRC/source、disarmed、fault_injection=false、故障 valid=false 全部通过；[最终回装](docs/acceptance/p1/20261031-hil.md#最终普通回装与p1结论)。

P1 最终普通回装为 `a946a529360eac83`；P1 阶段最近实测板载为普通 Rust `81d6c1221048f08a`（raw/padded=`370064/370080 B`、static RAM=`231704 B`），经原 PX4 bootloader 上传/校验；心跳 `autopilot=PX4`，disarmed、输出关闭，`LINK_LED_HZ=2.0` 的 P3a 保存值有效。RTC LSE 走时、写后回读、`SYSTEM_TIME`、日志 UTC 及短时旧 IMU ULog 已通过。Inspector 版本显示及主机重启后 CDC 零数据问题归 P2；FMU 灯目视项不适用。项目开始前原配置仍无备份，P3a 已备份写入前当前 FRAM。

## M1 前置阶段

- [x] **P3a / P3A-01～04**：FM25V02A 全量备份、统一参数 API、双槽原子持久化、一次保存/掉电读回通过；[证据](docs/acceptance/p3a/README.md)。2026-11-03 复测：设值→`!param-save`→重启两次（slot 0/gen 1、slot 1/gen 2）均从 FRAM 读回。旧存档因新增 `LOG_*` 参数的 schema 变更被拒绝并回退默认值（设计如此，不静默迁移）。
- [x] **参数自动保存历史验证**：原版 `PARAM_SET` 在 disarmed 且输出关闭时直接写 FRAM；实测一次设值即 `slot=0 generation=3 dirty=false`，`!param-save`/`PREFLIGHT_STORAGE` 可用。本轮已改为延迟提交，实物验证待上方 §8b 板载门。
- [x] **参数分组元数据**：`MAV_CMD_REQUEST_MESSAGE`(397) → `COMPONENT_METADATA` + 只读 MAVFTP（`component_metadata/{general,parameters}.json`），9 个 `LOG_*` 归入 `SDLog`；板载下载与 CRC 校验通过（source=`c88f30f929842543`，raw/padded=`398000/398016 B`，static RAM=`236872 B`，栈未触达 36972 B）。[证据](docs/acceptance/p3a/README.md#参数分组元数据2026-11-03)。QGC 参数页 “SDLog” 分组呈列待工程师目视确认。
- [x] **MSG-01 标准消息软件检查**：`sensor_accel` / `sensor_gyro`、`actuator_armed` 与固定 PX4 定义对齐；fmt/Clippy、4 项受影响核心用例、pyulog 与普通打包/布局通过，软件 source=`c2f3128aecc8a70b`。[记录](docs/acceptance/messages/README.md)；实物核对并入 P4 下一次必验记录。
- [ ] **下一步 P4b-a / CAN-A01～04**：CAN-A01～03 软件门已通过；待 H-02/03 完成 ESC 映射/Status、零命令与一次断线恢复实测。[证据](docs/acceptance/p4b-a/README.md)；DNA 仅设备必需时加入。
- [ ] **PL-a / LOG-A01～06**：LOG-A01～04 及 LoggerControl/MAVLink Console 软件门已通过；实物正常 start/stop、下载/解析已通过。LOG-A06 按键/蜂鸣软件及 USL I²C1/0x39 RGB 驱动已接入，按键有新日志、RGB 有协议响应；实际灯色待验，蜂鸣无声且已暂停。IO 安全灯不覆盖，旧 `473ffd4797e692f3` 包撤回；H-12 IO/完整物理项与 restart 直接证据仍待验。[USL 记录](docs/acceptance/pl-a/20261109-usl-logger-switch.md)，不放行依赖阶段。
- [x] **IO-H0 软件**：固定 PX4 v5 报文/CRC、USART6/DMA、零输出配置与回读、按钮锁存 ACK、`!px4io` 诊断；12 项相关 host 用例及隔离构建通过。[记录](docs/acceptance/px4io/20261104-host.md)。后来 H7 回装实测 Timeout，**IO 实物未通过**，见[回装记录](docs/acceptance/px4io/20261104-merged-flash.md)。
- [ ] **IO-R0～R5（本次仅计划）**：Rust 模块拆分/协议共享/迁移与必要验收已记录；先 H-12/13 型号/灯归属/恢复，再独立 M3 启动与 LED/gesture，后续 PWM/RC、更新/恢复按 [重构计划](docs/px4io.md) 逐门放行。不开始 IO 刷写、不新增 M1 前置。
- [ ] **P4b-g / GNSS-01～03**：GPS 已接 CAN1；软件已实现并刷入，待 H-10 与依赖门具备后验证 H-RTK Fix2/Auxiliary 实物数据、标准 `GPS_RAW_INT` 和 `sensor_gps` 日志。CAN2 桥不自动转发 CAN1。阶段依赖 P4b-a、PL-a、H-10。
- [ ] **P2 / MAV-01～03**：USB/TELEM1 双链路、QGC 连接/重连及版本显示；MAV-01 软件门已通过，待 H-03 与工程师接线做双链路/重连实测。[证据](docs/acceptance/p2/README.md)。依赖 P1、P3a。
- [ ] **P3 / PARAM-01～02**：软件/工具路径已由 P3a/P2 就绪；待 P2 闭合与 H-03 接线后做双链路一致性、合法/非法值回显及保存重启读回。依赖 P3a、P2。
- [ ] **P4 / SENS-01～04**：软件门、实物 WHO_AM_I/SPI/短时 8 kHz 计数、USB `HIGHRES_IMU` 与旧 `imu_icm42688` 短时 1 kHz ULog 已通过；标准 `sensor_accel` / `sensor_gyro` 速率确认为 200 Hz（`LOG_IMU_HZ` 默认，按样本时间戳抽取，不受批次抖动影响），文档与日志预算已同步；抽取实现修正已包含在当前 `588ab77941985319` 固件（此前修正 build source=`023c2163007a9ddf`，raw/padded=`397952 B`，static RAM=`236872 B`，未单独刷写），随下一次 P4 必验核对；待 DRDY 仪器测量、60 s 标准 `sensor_accel` / `sensor_gyro` 日志、六面方向、校准与故障恢复。[证据](docs/acceptance/p4/README.md)。依赖 P1、P3a、PL-a。
- [ ] **P4p / PWR-01～04**：INA219 单条 V/A/W、Windows Inspector 同显和短时 ULog 已通过。日志参数默认值读取 bug 已修复并验证参数回显；实际停录/变速、保存后掉电恢复仍待验。曾见 dropout 已定位为 IMU 1 kHz 日志时代的 SD 稳态亏空（产生 ≈98 kB/s > 隐含吞吐 ≈45 kB/s，见[定量分析](docs/logging-budget.md#31-dropout-定量分析2026-11-03)）；200 Hz 现状余量 ≈2.1×，复验并入联合负载（[analyze_dropout.py](tools/analyze_dropout.py)）。参考负载精度、支路供电和断线恢复待工程师配合。[记录](docs/acceptance/ina219/README.md)。
- [ ] **P6a / EST-01～02**：Mahony AHRS、姿态日志/QGC 方向；相对 yaw。依赖 P4、P2、PL-a。
- [ ] **P4b-baro / BARO-01～04**：BMP388/I2C4/0x77 驱动、补偿/CRC、标准发布及单独的 baro 量测事件已实现；256 用例对拍与真实 ID/CRC/温压首读通过。待栈整改后的 MAVLink 字段/速率、sensor_baro 独立解析和一次失效恢复，不把首读当精度验收。[记录](docs/acceptance/barometer/20261104-bringup.md)，依赖 P4、PL-a、P2、H-14。
- [ ] **P7 / TEL-01～02**：随真实数据源补遥测与速率控制。依赖相应数据源、P2。
- [ ] **M1-01～02**：一次 5 分钟数据链联合运行，核对数据、日志和资源；零命令/DISARMED。依赖以上阶段。

## M1 后续

- [ ] **PL-b / LOG-B01～02**：修复包 `81fd0d06601f9e2f` 已回装；90 字节分包、事件唤醒及受保护根目录 `.ULG` 删除的软件/构建和只读下载核对通过。**工程师确认实际日志删除正常**，不重复删除；GUI 下载速率、完整文件解析与链路门仍待验，不宣称 GUI 提速倍数。[记录](docs/acceptance/pl-b/20261108-qgc-log-service.md)。依赖 M1、PL-a、P2。
- [ ] 更快日志路径待工程师选定范围：仅保留 SD 页缓存/USB 合帧建议，均未实现/测量；工程师已要求不推进 MAVFTP burst，现有元数据 FTP 保持不变。
- [x] **RTC-01 / TIME-01**：source=`81d6c1221048f08a` 实测 LSE `ticking=true counting=true`，30 s 增加 31 s，`!time set 1700000000` 回读 `ok=true`，`SYSTEM_TIME` 与 `LOG_ENTRY.time_utc=1700000004` 非零。根因是清 `RSF` 未解锁 WPR，且 `DR` 后再次读 `SSR` 会重新锁住影子寄存器；见[时间基准](docs/time-sync.md)。
- [ ] **TIME-02**：H-RTK GNSS 授时待 P4b-g；3D fix 且样本新鲜才采纳，GNSS > 地面站 `SYSTEM_TIME` > 继承 RTC，偏差 ≥5 s 才写；LSI 后备额外周期校准。无 GPS 驱动前，时间来自继承 RTC 或入站 `SYSTEM_TIME`。
- [ ] **P6a-e / EKF-01～03**：E1-H～E6-H 本地软件门已通过，[E6-H 证据](docs/acceptance/ekf2/20261104-e6-host.md)；实物/E7 仍依赖 M1、P6a、P4b-g、P2、PL-b。[方案](docs/ekf2-port.md)
- [ ] **P4b-r / RTK-01～02**：RTCM 转发、RTK 与中断恢复。依赖 M1、P2、P4b-g、H-10。
- [ ] **P5a / OUT-A01～02**：单一仲裁、限时授权/撤销、看门狗及零输出门控。依赖 M1、P4b-a、PL-a、H-06。
- [ ] **P4b-b / CAN-B01～03**：拆桨电机映射/旋向与故障停转；恢复不自动解锁。依赖 P5a、P4b-a、PL-a。
- [ ] **P5b / OUT-B01～02**：RC、预检/解锁、仲裁与故障策略。依赖 M1、P6a、P4b-b、P2/P3、H-07。
- [ ] **P6 / CTRL-01～03**：控制/分配、实时性和台架/系留。依赖 P6a、P5b、PL-b。

## Backlog

- [ ] **P6b→P6c→P6d→P6e→P6f→P6g**：模式→GNSS 融合/导航→路径→影子回退→INDI/NMPC/轨迹→任务协议；依赖详见计划。
- [ ] **P3b / P8 / BACK-01～03**：元数据、按需外设/冗余和发布自动化。

## 待补硬件输入

| 编号 | 输入 / 阻塞阶段 |
| --- | --- |
| H-02/03 | ESC 固件/映射，CAN 适配器/接线/供电/终端，以及 TELEM1 UART7 适配器/线序/电平/流控；P4b-a/b、P2/P3 |
| H-04 | 当前卡的型号/布局、记录时延与允许掉电窗口；受控断电及缺卡/拔卡故障；PL-a |
| H-06/07 | 台架限幅/停转，RC/UART/电源和预检策略；P5a/b |
| H-08/09 | GNSS 坐标/精度，动力学/辨识/求解预算；P6c/d/f |
| H-10 | H-RTK 固件/节点/消息率及后续 RTCM 来源；P4b-g/r |
| H-11 | INA219 0x44 通信、单条 `DEBUG_VECT` V/A/W 在 Windows Inspector 已确认、ULog `rust_current_sensor` 已实测；待支路供电/接线确认、参考负载精度和一次断线恢复；P4p |
| H-12 | CUAV USL 接 GPS1：IO 协议/CRC/零 PWM、PF5 电平、短按 start/stop、长按无动作、蜂鸣与可解析日志；识别 I²C RGB/安全灯/左右小灯归属；LOG-A06、IO-H0/R0/R2 |
| H-13 | IO 实物 MCU、原应用/bootloader 备份条件、SWD/电源和不依赖新应用的恢复；IO-R0/R1/R4/R5 |
| H-14 | 已识别 BMP388/I2C4/0x77、PG8，trim CRC 与温压首读通过；待标准数据/日志、参考精度及一次失效恢复；BARO-01～04 / E7 |
