# P1 上板操作单（工程师执行）

日期：2026-10-31。**本文件保留早期完整流程；现行必验范围以[精简计划](../../../plan.md#4-仅保留必要测试)为准。** 已有身份/平台/测量、重启与故障结果直接复用，本次QGC刷回PX4也已完成，只待最终普通回装。实际结果见[HIL](20261031-hil.md)，软件产物见[备份域证据](20261031-backup-domain.md)。
软件构建与产物身份见 [软件证据索引](README.md)。P1-01/H-05 的原恢复镜像、上传/CRC、供电/重连与参数资料状态必须先补齐；未完成时不要上传新包或注入故障。

## 1. 安全与记录

- 唯一 Pixhawk 6X Rev 3；拆桨、断开电机动力，P1 不连接/测试 ESC 输出。不要更换 bootloader、改 option bytes 或刷坏镜像。
- 按 [恢复操作单](../../recovery.md)保存官方 PX4 镜像/hash 与独立于 Rust 命令的入口；SWD 可用性不是默认前提。
- 保留 `.p0-baseline/20261031/v6x-rust-led.px4`，SHA-256：`d74b3b80e7c35bf4354ea568c09631f2fad362570caff414516ee0fe48a3c02d`。
- 记录日期/执行者、板号/Rev、主机/OS、USB 归属、所有供电与外设、QGC/bootloader 版本；协议 revision 与实际 bootloader 构建版本分开，不可取得的标“未知”。参数/校准备份缺失如实注明。
- 新建 `target/p1-hil-<日期>/` 保存原始日志；采集输出必须用不存在的子目录。QGC、IDE monitor、MAVProxy 与采集工具不可同时读同一串口。

开始前复制 [恢复模板](recovery-template.md)及本文第 7 节，记录**事前采用的阈值**。失败/重试不删除，不在测完后放宽阈值代替修复。

## 2. 普通版上传与基本回归

仅在 P1-01 通过后执行以下设备命令。软件构建不等于允许跳过该前置。

```sh
bash tools/firmware.sh build
sha256sum target/thumbv7em-none-eabihf/release/v6x-rust-led.px4
# 此命令会重新构建普通版、请求 bootloader、上传应用、校验 CRC 并复位：
bash tools/firmware.sh upload
```

保存完整 uploader 输出、board ID=53、应用 CRC 校验、镜像路径/hash、`v6x-rust-led.build.json`。**upload 默认不会上传注入版；不要把新 build 当保存的 P0 恢复包。**

检查启动：

| 检查 | 预期（实际值另填） |
| --- | --- |
| USB | `1209:0001` / `V6X-RUST-001`；重连重新找端口，不固定 tty 编号 |
| PHY | PWR 常亮，LINK 约 1 Hz；`!status` 给出 PHY ID/LED 配置 |
| 状态 LED | PE5 蓝灯启动期间亮、初始化完成后灭；PE4 绿灯 1 Hz；PE3 红灯随保留故障亮，不能把 PHY 的灯误认成状态灯 |
| `!platform` | schema=2、version=0.2.0、source 与**本次包** build.json 一致、fault_injection=false |
| `BACKUP` | 保存 bdcr_before/after、csr_before/after；config_preserved=true、bkp_preserved=true，BDCR配置掩码`0x0001833d`内一致；这是HAL边界保持检查，RTC走时另验 |
| 时钟/核心 | `CLOCK registers_ok=true`；CPU/HCLK/PCLK 配置 480/240/120 MHz，USB 48 MHz；VTOR=`08020000`，I-cache 开、D-cache 关、DWT 计数开启 |
| 定时器/IRQ | TIM2 PSC=239、ARR=65535、计数开启；逻辑优先级 TIM2/P1、TIM7/P3、USB/P6 |

本板适用性补充：工程师于本轮确认只可见I/O/ETH灯，没有可观察的FMU LED。因此PE3/PE4/PE5目视项记为不适用，不能用I/O B/E或ETH闪烁代替，也不记为FMU灯实测通过；故障保持/清除仍须由原始字段与CRC验证。详见[实物反馈](20261031-hil.md#led适用范围)。

可用 pyserial 自带终端发送以 LF 结尾的文本命令（CDC 同时含二进制 MAVLink，屏幕可能夹杂乱码；原始证据以采集日志为准）：

```sh
.venv/bin/python -m serial.tools.miniterm --eol LF --raw /实际Rust端口 115200
```

终端退出后独立验证 MAVLink；把下列占位符换为本次 build.json 的 16 位 source_id：

```sh
.venv/bin/python tools/check_mavlink.py --source-id <普通版source_id>
```

它检查 disarmed 心跳、版本、只读参数、v1/v2 分片、CRC 拒收和超时恢复；不会 reboot。当前两项只读参数不是 M1 的完整参数读设功能。

## 3. 空载与 USB 压力

关闭其他读串口程序，先空载再压力，每次独立运行并保存结果：

```sh
.venv/bin/python tools/p1_capture.py --seconds 65 --output target/p1-hil-idle
.venv/bin/python tools/p1_capture.py --seconds 65 --pressure-hz 100 --output target/p1-hil-usb
```

工具只发送 `!platform`、`!fault`、`!metrics-reset`、`!metrics` 与可选 PING；**不会刷写、重启、清故障或注入故障**。reset 仅重置统计 epoch，不清栈历史水位/保留故障；先等待约 2 s 获得新 CPU 窗口。保存 `wire.jsonl`（原始 TX/RX hex、主机单调时间）和 `summary.json`（平台、故障、全部快照及结论）。

| 指标 | 本轮判据 / 范围 |
| --- | --- |
| 时基 | 板上观测 ≥60 s；漂移范围完整落在 ±0.1% 内。host 时间戳以请求发送至响应接收为保守边界；范围跨阈值为“不确定”，不是通过，需更长窗口/较低 RTT 或独立仪器 |
| 周期替身 | 1 kHz ±1%；`skipped/deadline_misses/scheduling_overruns/scheduling_skipped/time_errors` 全程为 0，epoch 不变 |
| 内部 release 延迟 | `late_max_us ≤10`，保存全程直方图、间隔 min/max；这是任务开始相对预定 release 的延迟，**不是 DRDY 外部边沿抖动** |
| DWT | invalid_cycles=0；保存 body/IRQ 样本数、sum/max，cycle÷480 得配置频率下的 μs；物理时钟未验收前转换值仍有时钟误差 |
| CPU | 所采集 1 s 窗口均 valid、窗口/快照新鲜；最大 busy_permille + idle tick 量化上界 + 1 permille 整数舍入界 <500。硬件 idle bracket 开销另测，不能只凭此宣布精确整机 CPU |
| 共享栈 | guard_ok=true、最少 untouched_bytes ≥32768，快照年龄 <2 s；保存 min_msp/touched_bytes |
| USB 压力 | 目标 100 PING/s，记录实际发送/应答速率；有效 MAVLink 帧 >0、CRC 错帧=0、应答观察率 ≥95%。这只是 USB 压力，不是 CAN/SD/IMU 联合负载 |

`summary.json` 的 `software_measurement_gate=pass` 还要求schema-2备份域证据有效；缺失或保持失败均不能通过，旧固件需先按流程升级。该门仅为这些 USB 观测的筛选结果；**仍需工程师完成基本检查、恢复和测量限制确认**。它不能证明未采集的每个 1 s CPU 窗口或全程 WCET；快照约每秒生成，host 每秒请求可能遗漏个别窗口。

## 4. 测量限制（验收记录必须保留）

- TIM2 虽为 32-bit 外设，Embassy 0.5.0 时基驱动使用 16-bit ARR、半溢出/溢出中断扩展单调时间，PSC=239。检查寄存器只是核对配置，时基仍要独立比对。
- TIM7 **外设不运行**，仅作为 P3 软件 pend 的 InterruptExecutor 向量。替身为固定 32 次整数运算；不能推算未来 IMU、AHRS、控制性能。
- body cycles 包含两次 `Instant` 读取附近的计数开销，可能被 P1 TIM2 抢占；不含后续统计/快照和 timer arm。IRQ cycles 从 TIM7 内部打点到 executor poll 返回，包含栈采样及更高优先级抢占；**不含硬件入栈/出栈、函数完整尾部及 IRQ 统计更新**。均为观测值，不是 WCET。
- CPU 使用 wall time 减 PRIMASK 下 TIM2/WFE bracket；SEVONPEND 使新 pending IRQ 唤醒后再开中断，ISR 时间留在 busy。被减掉的 bracket 同时含 MMIO/DSB、WFE 唤醒开销；`idle_calls` μs 是 tick 量化误差上界，**不涵盖上述硬件开销**。
- `exit_bookkeeping_max_cycles` 只测 WFE 后的部分 bookkeeping，未覆盖完整全局屏蔽区。PRIMASK 在等待时可持续到下次事件；不能声称已证明“每次全局屏蔽 ≤2 μs”。后续采样阶段需仪器或硬件时间戳核实**pending→重新允许服务**延迟及临界区，必要时调整 idle 实现。
- 64 KiB 栈由 cortex-m-rt 0.7.7 paint-stack 初始化；水位扫描可被抢占，不长时间关中断。相同 paint 值的写入/未写的局部空间可能未被识别，SP 只在有限位置采样，不能证明所有瞬态峰值或阻止任意越界。任务 futures 属 static RAM，不是每任务独立栈。
- HardFault 的独立 1 KiB 栈不计入正常 watermark；静态反汇编结论见 [故障入口审查](fault-review.md)。panic/DefaultHandler/stack guard 路径仍先使用共享栈；锁死、嵌套故障或损坏硬件不保证软件恢复。独立 QGC 入口必须保留。

## 5. 正常 reboot 与故障保持

### 启动故障转储（P1-02 诊断，不替代 P1-04）

新 P1 无 CDC/LED 时，先由工程师明确选择刷入 `bash tools/firmware.sh build-dump` 产生的诊断镜像（`--features fault-dump`），不要反复盲刷普通 P1。镜像用 P0 时钟树和共享备份域wrapper启动，输出 BKP16..31/RSR/RCC 及 schema-2 `V6X_DUMP_BACKUP`，比较全部BKP并保留检查失败结果，支持 `!bootloader`。当前行为见[备份域修复](20261031-backup-domain.md)，原诊断流程见[历史记录](20261031-fault-dump-image.md)。保存原始串口文本、解码输出与完整 uploader 日志后，再按故障记录定位。

### 普通 reboot

1. 普通版发送 `!bootloader`；保存 ACK/掉线、原 bootloader 枚举与重装**相同普通包**的 CRC 日志。
2. USB 重连后再次核对 source/版本、LED/MAVLink。再单独测试 MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN、param1=3，并记录同样的结果。
3. 不把软件 reboot 成功代替独立恢复。当前 P1 无电机输出；后续引入输出时，reboot/保存需 disarmed 且输出关闭。

### 故障注入（只由工程师明确执行）

仅在普通版上传、测量和恢复已正常、P1-01 已通过时进行。注入版构建：`bash tools/firmware.sh build-injection`。确认 build.json 的 features 含 `fault-injection`，用固定 uploader **显式选择** `target/p1-fault-injection/thumbv7em-none-eabihf/release/v6x-rust-led.px4`，不要用普通 upload action 代替。保存注入版 source/hash/CRC。

1. 注入版 `!platform` 确认 fault_injection=true；先 `!fault` 保存旧记录。仅在已保存后发送 `!fault-clear`，确认 valid=false。
2. 发送一次 `!panic`。预期 ACK 后请求原 bootloader；不以串口退出单独判定成功。**不要断开所有供电**，通过原 bootloader 显式回装普通包。
3. 普通版 `!fault` 应 valid=true、kind=1、source 为**注入包** source、detail 为 panic 行号、file_hash 非零；红灯亮。保存记录/正常软件复位后再次读回，字段与 CRC 保持一致。
4. 保存后 `!fault-clear`，预期 valid=false、红灯随后灭。再重新上传注入版，核对身份后发送一次 `!hardfault`，按相同流程回装普通版。
5. HardFault 应 kind=2、source 为注入包、detail=1、PC/LR/SP/xPSR/EXC_RETURN 可对照注入包 ELF；`udf` 触发的 fault 状态需与反汇编一致。记录无法取得帧/未进入 bootloader等失败，不自动重试注入。
6. 故障记录位于 RTC BKP16..31，BKP0 保留 bootloader 请求。每次回装均保存 `BACKUP`，并比较故障字段/CRC。测试的是**保持备份域供电的复位与回装**；不保证彻底断电后仍保留，也不保证经历官方 PX4（不同 BKP 所有者）仍保留。当前wrapper通过匹配配置避免BDRST，HAL后只比较、不补写BKP；该结果不代替RTC calendar走时实测。
7. 全部结束后回装普通版、核对 fault_injection=false 与 CRC；保存并明确清除测试记录。未经上板验证不声明任意栈损坏可恢复。

## 6. QGC 往返与退出条件

1. 关闭所有串口占用者，用**上游 QGC 5.1.5**连接新普通版，Inspector 查看 HEARTBEAT/SYS_STATUS/SYSTEM_TIME/AUTOPILOT_VERSION，保存截图/日志、source 与 disarmed 状态。
2. 不发送 Rust reboot，重复 P1-01 中已记录的独立断电/重连或复位流程；QGC 识别原 bootloader，上传本板官方 PX4、校验并启动。保存实际镜像/hash及完整 QGC 记录。
3. 通过原 bootloader 回装保存的 P0 已知良好包并做 0.1.0 回归（`tools/check_mavlink.py --expected-version 0.1.0`）。需要继续 P1 时，再回装验收普通包并核对其身份。
4. 故障回填、两种入口、普通版两类 reboot、空载/压力及 QGC 往返均有记录才关闭 P1-04。失败或资料缺失保持待验，不放行依赖 P1 的上板阶段。

## 7. 实际结果回填模板

复制为独立日期文件，链接原始资料；禁止直接把下面的“待验”改成没有证据的通过。

| 项 | 实际值 / 原始证据 | 结论 |
| --- | --- | --- |
| 执行者/日期/板号/主机/USB归属/供电 | 待填 | 待验 |
| P1-01 恢复记录链接、参数/校准资料状态 | 待填 | 待验 |
| 普通/注入包 source/hash、上传 board ID/CRC | 待填 | 待验 |
| LED、USB身份、平台原始寄存器、MAVLink检查 | 待填 | 待验 |
| 备份域BDCR/CSR、全部BKP保持标记、回装后的故障CRC | 待填；保存schema-2原始报告 | 待验 |
| 空载 window/drift bounds/周期/latency/CPU/栈 | 待填；链接 wire.jsonl、summary.json | 待验 |
| 压力实际PING速率/应答率、同上统计 | 待填；独立输出目录 | 待验 |
| CPU硬件开销、pending服务延迟、未覆盖范围 | 待填；未实测如实注明，不宣称≤2 μs | 待验 |
| 普通文本/MAVLink reboot 与回装 | 待填 | 待验 |
| panic/HardFault字段/CRC/复位保持/清除 | 待填；保留注入包ELF与source | 待验 |
| 新版 QGC/Inspector、独立入口/PX4/回装P0 | 待填；镜像hash、日志、截图 | 待验 |
| 最终安装包/故障记录状态、失败与重试 | 待填 | 待验 |

最终工程师结论/签名：待填。软件证据与 HIL 分开；不将 P1 周期替身验收写成 M1 传感器或控制链已通过。
