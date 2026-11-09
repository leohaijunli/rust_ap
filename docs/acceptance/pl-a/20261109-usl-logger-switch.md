# CUAV USL：日志按键、RGB 与蜂鸣器

日期：2026-11-09。工程师选择将 GPS1 CUAV 按键用于**日志启停**，授权应用区回装及实体核对，并要求启用 USL RGB、蜂鸣器。不是飞行安全许可，不解锁或允许非零输出；原 PX4 bootloader/IO 固件与 IO-owned PD10 安全灯不改。

## 实现与约定

- 原 PF5 按键任务已运行，但 `px4io::ready()` 未就绪时每拍重置消抖，导致动作禁用。改为 disarmed 且输出关闭才接受按键；IO 故障仍独立可见，不更改 PWM/安全护栏。
- 短按释放：按 Logger 状态请求 start/stop；40 ms 消抖，启动时按住不触发，撤销许可后须重新释放/按下；长按 ≥1.5 s 只记录保留事件，不切换日志、不解锁。
- `LOGGER_SWITCH schema=3` 明确 `role=logger`，区分 `enabled/io_ready/raw_high/pressed`，记录短/长按、请求与拒绝计数。
- USL RGB 清理后固定为实测 **I²C1 PB8/PB9、0x39、NCP5623-compatible**，不再探测第二总线/TCA/其他地址。每笔阻塞事务限 10 ms，失败撤销 ready、保留错误并间隔 1 s 重试；协议响应不是独立芯片身份，颜色仍须工程师目视确认。初次多协议探测记录保留在下文。

| RGB | 本次含义 |
| --- | --- |
| 熄灭 | Logger Disabled；模块/驱动未就绪也可能不亮，须看诊断 |
| 绿常亮 | Logger Ready，正在记录；不证明所有数据源有效 |
| 黄慢闪 2 Hz | 启动、停止或日志管理 |
| 红快闪 5 Hz | Logger Error，例如挂载/写入/同步失败 |
| 红→绿→蓝各 1 s | 开机或 `rgb test` 通道测试，不表示安全/RTK 状态 |

- PF9/TIM14 输出 4 kHz 蜂鸣：日志启动成功一短声、停止成功两短声、拒绝/错误三短声。软件布尔或 PWM 配置不作为实听证明。
- 工程师报告短提示音**未听到**，实物项保留未通过；新增独立 `buzzer test`，仅在 Logger Disabled、disarmed/输出关闭且无待处理日志动作时接受，延迟 0.5 s 后三次 0.7 s 试音，间隔 0.4 s，自动结束，不操作日志。
- 当前命令：QGC console `rgb` / `rgb test` / `buzzer` / `buzzer test`；原始 USB 对应 `!` 前缀。`buzzer` 读回 PF9 模式/AF、TIM14 使能/通道/PSC/ARR/CCR，仍不能代替实际电压/波形或声音。

## 软件检查

- 9 项 logger_switch host（含许可撤销与独立试音边界）、2 项 RGB 编码/状态 host 通过；fmt、host/普通 H753 Clippy 通过。PWM 读回接口先编译失败，按 PAC 实际 `cce(0)` / PSC 原始值修正后通过，未改变控制寄存器。
- 首次试灯固件 source=`2b7dd895b8cee3d3`：构建/分区/入口帧通过，raw/padded=1229176/1229184 B，静态 RAM 374712 B（余 83016 B），入口帧最大 11528 B、oversized=0。基线含 crate 改名和已提交的 PX4 反向计时；本轮不改估计数学。
- 工程师曾确认串口空闲，上传后 COM26 被占用，首次启动查询未完成；不终止未知程序。工程师释放串口后再核对，不能把访问失败当作固件功能失败或通过。

## 清理与软件交付

- 工程师要求 clean code 并推送后，仅做软件清理：RGB 驱动 256→142 行，去掉未用兼容分支，共享颜色写入/错误处理；`USL_RGB schema=2` 去掉 `candidate_mask`。NCP 字节顺序、亮度及日志灯态不变，旧 schema=1 捕获不改写。
- console 测试回包沿用 `.and_then` 传播格式错误；独立蜂鸣测试时序改为命名常量，仍是 0.5 s 延迟、三次 0.7 s/间隔 0.4 s。一次局部 `code-simplifier` 审视完成，不扩展接口或改估计数学/输出护栏。
- 清理后实际运行 RGB 2 项 host、host 库/测试与普通 H753 Clippy，均通过；按键/蜂鸣 9 项沿用本轮清理前通过结果，时序仅等值常量命名变化。未重跑全库数学测试，不把过滤掉的用例写成通过。
- 最终普通 release source=`094b08072d1267ac`（构建时 git 基线 `8a586f1…`，含待提交 USL 清理）：构建/打包/分区/入口帧通过，board ID=53、应用入口 `0x08020000`；raw/padded=1227200/1227200 B、静态 RAM 374680 B（余 83048 B）、入口帧最大 11528 B、oversized=0。相比已回装 `d884…`，Flash 减少 3960 B、静态 RAM 减少 40 B；入口帧不是完整调用链或板载栈水位证明。
- fmt/diff 检查通过；109 项构建源码 SHA-256 与当前文件、4 个产物 SHA-256 与报告逐项一致。ELF/bin/map/px4、构建报告、入口帧与原始构建输出保存于 `target/usl-20261109/094b08072d1267ac/`；px4 SHA-256=`ca3e3dec79d55cf5a50cad527d9f74a43a154c7e623651d0de39f91a82222af3`。**此时仅构建、未回装；随后独立授权回装见下文**。
- 全文件链接检查发现基线已有多处历史文档缺失（如 `docs/acceptance/pl-a/README.md`）；本轮不恢复或删除未知历史文件，仅检查本次新增引用，移除修改行中的失效引用。此项不写成全仓链接通过。

## 首次回装与只读证据

- 原 PX4 bootloader 应用区上传/CRC 100%（25 s）。启动 `source=2b7dd895b8cee3d3/git=8a586f1…/fault_injection=false`。
- `USL_RGB`：**I2C1 PB8/PB9、地址 0x39、NCP5623-compatible**，ready=true、probe=1、mask=01、errors=0、一次启动测试已发送；查询时 Logger 停止，RGB 请求为 [0,0,0]。这是总线/协议与请求证据，非三色目视验收。
- 工程师已自行多次试按；查询快照 presses=23、short=18、long=5、start/stop 各 6、rejected=6，`enabled=true/io_ready=false/raw_high=false`。不把这些累积计数当作每次动作或声音均通过。
- Logger 当时 Disabled，file_index=2、bytes=342016、flushes=13、errors=0；正常丢失 6、事件 0。旧日志/卡状态不覆盖，不将累计丢失清零冒充无 dropout。
- EKF Running/Filter、fault/numerics=None、deadline_misses=0、max_update=1493 µs/2000 µs、发布 missed=0、TIM8 积分误差 0 ppm。PX4IO 仍 `ready=false/Timeout`，本轮不称其问题收口。
- 原包/报告/上传及只读输出：`target/usl-20261109/2b7dd895b8cee3d3/`。先前改名软件包保存于 `target/crate-rename-20261109/136329bb61f63100/`，不改写旧结果。

## 待完成的实体核对

1. RGB 的实际三色顺序/日志灯态仍须工程师目视确认；协议响应不替代颜色验收。
2. 按键已有真实启停/日志证据，但明确的较长正常窗口、长按保留行为及 H-12 其余物理项不由累计计数代替。
3. 蜂鸣器短/长试音均未听到，工程师已选择**暂停蜂鸣验证**。不再触发试音、扫频或直接驱动 pin 7；后续先核对可靠线序、供电及实际输出波形，再由工程师单独决定是否继续。

## 独立长试音与最终收尾

- 长试音版 source=`d884e6ba1adf41b7` 构建/原 bootloader 上传/CRC 100%（25 s）通过。raw/padded=1231160/1231168 B，静态 RAM 374720 B（余 83008 B）、入口帧 11528 B、oversized=0；新包保存在 `target/usl-20261109/d884e6ba1adf41b7/`。
- 工程师确认准备好后执行一次 `!buzzer test`：accepted=true、tests 0→1；PF9 模式 ALTERNATE/AF9，TIM14 timer/channel enabled，PSC=0、ARR=59999，三段期间 CCR 0→30000（50%），结束回零。日志始终 Disabled，未用测试命令启停日志。
- 工程师明确报告**三次 0.7 s 长试音仍完全无声**。这是失败/未通过，不能因 GPIO/定时器配置看似正确而改写为蜂鸣可用；也未将占空比拉到 100% 或假装有音量参数。`buzzer-test.json` 保存此次响应与原始字节。
- 固定 FMUv6X `board_config.h:298–306` 明确 FMU PF9/TIM14 是 tone alarm，IO 协议没有音调输出命令；PX4IO owns safety LED 不等于 owns buzzer。工程师提供的“飞控 GPS1 pin 7=BUZZER”表与 [Holybro 标准 GPS1 资料](https://docs.holybro.com/autopilot/pixhawk-baseboards/pixhawk-baseboard-v1-ports)冲突：官方 pin 7=safety LED、8=3.3 V、9=BUZZER−。具体底板/USL 两端路由未核实，不据此更换 GPIO、调换线或在原 IO 未隔离时驱动 PD10。
- 收尾只读：Logger Disabled、file_index=6、bytes=55296、errors=0、normal/event dropped=0；按键 short=6、start/stop各3、rejected=0。RGB仍为 I2C1/0x39、ready=true、errors=0、请求 OFF；buzzer on=false、test_pending=false、CCR=0。
- 工程师自行短按产生的最近日志 id6 下载到独立 `logs/06.ulg`（55296 B，5.4 s），source与新固件一致，pyulog 无损坏/dropout；243 个 accel/gyro、7 个高度/估计快照，证明按键会话产生真实可解析日志，不作为 60 s 或 M1 验收。SHA-256 `82fc2d52946ee68b565c505829f53aeec42c65cbc62920a581addcca467ddcea`，`log-summary.json`保存 topic 计数；未覆盖原 `e7-log` 或删除旧 SD 文件。
- EKF收尾 Running/Filter、fault/numerics=None、deadline_misses/发布 missed=0、max_update=1476 µs/2000 µs、积分0 ppm；PX4IO仍Timeout，未更新其固件。`final-readonly.json`保存收尾查询，未执行输出或参数变更。

## 后续授权回装与只读核对

- 工程师随后追加要求 push/flash。代码提交 `736d4aa` 已先推送 `origin/main`，再回装原样保存的合格普通包 `094b08072d1267ac`；不恢复试音/扫频/pin 7 试驱。构建时的 `git=8a586f1…` 标签保留，新 source 对应提交 `736d4aa` 的源码内容，不是重刷旧源码。
- 109 项工作树源码 SHA-256 仍与构建报告逐项一致。Git 归档的原始字节比较初次发现 51 项不同，核实为 `core.autocrlf=true` 下的 CRLF/LF 行尾差异；仅在比较时归一化后，109 项源码内容全部与 `736d4aa` 一致。未改源码/固件，`commit-source-compare.json` 保留差异与结果，不宣称 Git blob 原始 SHA-256 全同。
- 回装前 COM26 只读确认旧 source=`d884…`、Logger Disabled、buzzer off/CCR=0，串口已释放；`pre-flash-readonly.json` 保存原始字节。经固定哈希上传器与原 board 53/protocol v5 bootloader（COM12），应用区上传/CRC 100%（25 s）；`upload.txt` 保存完整输出，未修改 bootloader 或 IO 固件。
- 首次主机只读采集在导入 `check_mavlink` 时因 Python 3.10 缺少 `tomllib` 失败，尚未打开串口；沿用现有 `tomli` 兼容导入后通过，未因此重建/重刷固件或安装依赖。
- 回到 COM26（`1209:0001 / V6X-RUST-001`），3 条 Quadrotor/PX4 心跳均 disarmed；平台 source=`094b08072d1267ac`、fault_injection=false、VTOR=`08020000`，时钟寄存器匹配、TIM8 PSC=239/ARR=65535，FPU ASPEN=true/LSPEN=false，备份域 config/bkp_preserved=true。
- Logger Disabled/stage=0、bytes/flushes/errors=0，按键 start/stop/rejected=0、pending=none；RGB schema=2、I²C1/0x39、ready=true、probes=1、transfers=20、errors=0，启动试灯结束（tests=1）、请求 OFF。仅确认总线/软件状态，仍非实际三色验收。
- Buzzer on=false/test_pending=false、tests=0、CCR=0；本轮没有试音命令。EKF Running/Filter、fault/numerics=None、updates=15030、max_update=1477 µs/2000 µs、deadline_misses/发布 missed=0、积分误差 0 ppm。PX4IO 仍 ready=false/Timeout，不称其问题收口。
- `post-flash-readonly.json` 保存心跳、6 项查询及原始字节；未设参/保存、启停/删除日志或发送非零输出。这是普通回装/启动核对，不是完整 H-12、E7/M1 或蜂鸣实物验收。

## 边界与来源

- H-12 剩余物理灯/声、PX4IO 通信、完整 E7/M1 和非零输出仍按 [plan.md](../../../plan.md) 的门，不因输入/指示灯可用自动放行。清理构建本身未回装；工程师随后单独追加 push/flash 授权，允许应用区回装及启动只读核对，不恢复蜂鸣测试。
- 固定 PX4 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`：`boards/px4/fmu-v6x/{src/board_config.h,nuttx-config/include/board.h}` 的 PF5/PD10/I²C 引脚；`src/drivers/lights/{rgbled,rgbled_ncp5623c}` 的兼容探测与编码。保留 BSD-3-Clause 来源，板上仅 Rust，不导入 C++。
