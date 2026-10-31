# P0 软件基线与 G0-M1 记录

日期：2026-10-31。**A3 混合与首目标 M1 已由用户确定。** 本目录归档软件基线、选型依据、初始测试提案和工程师待办；当前固件仍是 USB/MAVLink/LED demo，完整实现阶段与本板新 HIL 尚未验收。

后续进展：用户随后报告了 PX4 恢复、QGC 连接与 Inspector 心跳/其他消息成功，见 [P1 用户记录](../p1/20261031-user-recovery-report.md)。下文软件结果和最终核验计数保留 P0 收口时口径；H-05/P1-01 的最新缺口以该记录和 TODO 为准。

## 1. 本轮结论

| 项 | 状态 |
| --- | --- |
| P0 软件资料 | 基线、固定来源、实际构建/host 结果与 M1 最小文档齐备；核验完成，P0-01～10 按软件交付范围关闭 |
| G0-M1 选择 | 用户已选 A3：当前可恢复工程 + Embassy 设备层 + 按需复用固定 Cybflight 纯算法，见 TODO 的 D-A3/H-01 |
| M1 范围 | 单 IMU/AHRS、DroneCAN 服务/ESC 状态与零输出/H-RTK GNSS、SD ULog、USB/TELEM1、上游 QGC 消息检查与参数读/设 |
| 数值预算 | 首轮测试提案；在对应 HIL 前记录采用值，以实测修订。A3 选择不等于这些指标已经通过 |
| 后续细节 | 控制/INDI/NMPC/任务/非零输出、日志下载与长期策略按 D-DEFER 在更多硬件测试后确定 |
| 上板前置 | H-05/P1-01：工程师完成原 bootloader 的独立入口及 QGC 官方 PX4 → 已知良好 Rust 往返恢复 |

P0 的关闭只覆盖软件调研与资料交付。后续阶段按 [TODO](../../../TODO.md) 的依赖分别验收；离线准备可按阶段说明推进，首次新功能刷写须已有恢复通过记录。

## 2. 文档与任务证据

| 任务 | 交付与主要原始证据 |
| --- | --- |
| P0-01 | [移植/基线记录](../../porting-notes.md)、[入口清单](baseline-entry.json)、[持久备份清单](baseline-preservation.json)、[重建产物](built-artifacts.json) |
| P0-02 | [来源/许可](../../provenance.md)、[上游入口](upstreams-entry.json)、[工具版本](environment-pinned.txt)、[依赖清单](dependency-inventory.json)、[锁文件](locks/) |
| P0-03 | [当前工程](current-results.json)、[Holsatus](holsatus-results.json)、[Cybflight](cybflight-results.json)、[host 计数](test-counts.json) |
| P0-04 | [A1/A2/A3 评估与选择](../../base-evaluation.md)、[M7 core ABI](core-abi.json) |
| P0-05 | [硬件台账](../../fmuv6x-hardware.md)：固定 PX4 文件行、HAL 能力与待测归属 |
| P0-06 | [QGC 恢复操作单](../../recovery.md)、[工程师回填模板](../p1/recovery-template.md) |
| P0-07 | [DroneCAN 记录](../../dronecan-notes.md)、[探针结果](probe-results.json)、[参考向量](dronecan-vectors.json)、[协议来源](protocol-upstreams.json)、[生成器来源](generator-upstreams.json) |
| P0-08 | [ULog/SD 初始预算](../../logging-budget.md)、[M1 预算](budget-m1.json) |
| P0-09 | [A3/Embassy 架构](../../architecture.md)、[M1 联合验收单](../../m1-verification.md) |
| P0-10 | 本索引、已记录的用户选择、下述遗留项与最终核验结果 |

各 `*-results.json` 保存命令、cwd、退出码与耗时，同名 `.txt` 保存原始输出。host 耗时不代表 H753 执行时间；源码配置、厂商规格与本板实测分别留证。

## 3. 实际软件结果

| 检查 | 结果 | 证据 |
| --- | --- | --- |
| 当前固件 check/build/section-size/依赖检查 | 四项通过；check 含 fmt、Clippy、4 项打包测试 | [current-results.json](current-results.json)、[current-check.txt](current-check.txt) |
| Holsatus common host、std-test-device、RP2040、H743 | 四项 exit=101；同一 `mav-param/serde` 解析错误，构建未验证；用户要求跳过修复/重跑 | [holsatus-results.json](holsatus-results.json) |
| Cybflight core | 446 passed / 12 ignored，其中 Mahony 12 项通过 | [core 输出](cybflight-core-test.txt) |
| Cybflight drivers / vehicle-YAML / sim snapshot | 26 / 51 / 1 passed | [test-counts.json](test-counts.json) |
| Cybflight M7 core check / H743 release build | 通过；上游镜像链接 0x08000000，仅作编译证据 | [cybflight-results.json](cybflight-results.json)、[镜像 sections](cybflight-h743-size.txt) |
| A3 独立 git core 依赖 | Rust 1.99.1、M7 ABI 探针编译通过；尺寸不含临时栈/任务 future | [core-abi-m7.txt](core-abi-m7.txt)、[core-abi.json](core-abi.json) |
| umi DroneCAN | std 8 tests passed；无 alloc M7 编译失败；异常探针确认四项缺陷 | [probe-results.json](probe-results.json)、[异常输出](dronecan-malformed-probe.txt) |
| Python 参考生成 | 40 个样本 + 3 个拒绝样本；Rust/会话/HIL 对拍待 CAN-A01 | [reference-generation.txt](reference-generation.txt)、[向量](dronecan-vectors.json) |

原始成功结果继续作为本轮基线；仅在新增修改或核验问题涉及它们时重跑。Holsatus 的授权跳过和 umi 诊断进程 exit=0 都不表示候选已合格。

## 4. 预算口径

总线假设为 1 Mbit/s；CAN 总计包含 **10 个百分点**错误/重试预留，并非实测重传。

| 文件 / profile | CAN 含预留 | ULog B/s | 加 25% B/s |
| --- | ---: | ---: | ---: |
| [budget-m1.json](budget-m1.json)，默认 `m1` | 33.028% | 111062 | 138828 |
| [budget-default.json](budget-default.json)，历史 `exploration` | 39.428% | 142793 | 178492 |
| [budget-eight-esc.json](budget-eight-esc.json)，历史 8 ESC/8 槽 | 58.892% | 158193 | 197742 |
| [budget-sparse-esc.json](budget-sparse-esc.json)，历史 8 ESC/20 槽 | 78.092% | 177393 | 221742 |

M1 为四 ESC 连续槽、单 IMU、AHRS，不启用 RTCM 额度。用户询问的“另加 3×float32、200 Hz”会增加 6.4 个百分点和约 5000 B/s ULog；得到的 39.428% 恰与历史探索相同，但两者流量组成不同。该消息尚未指定含义/DSDL，未加入默认范围；详见 [容量说明](../../dronecan-notes.md)。

## 5. 基线保护与复现

入口 HEAD 为 `5daeca30fcd116203164f44128239e7dd8a884ef`，入口已有用户改动。25 个源文件 hash、当时状态和三个固件产物见 [baseline-entry.json](baseline-entry.json)。本机持久副本在 `.p0-baseline/20261031/`：源码归档、入口 diff、ELF、BIN、PX4 共五份，受 gitignore 保护。

已知良好 `.px4` SHA-256：

```text
d74b3b80e7c35bf4354ea568c09631f2fad362570caff414516ee0fe48a3c02d
```

恢复使用保存的包。归档恢复到独立目录；公开资料中的 hash 不能代替本机二进制副本。当前工程与上游成功基线的复现命令见 [porting-notes](../../porting-notes.md)；两个探针、向量和四种预算命令见 [tools/p0/README.md](../../../tools/p0/README.md)。

## 6. 移交到对应阶段

| 待办 | 归属 / 当前缺口 |
| --- | --- |
| 实际恢复往返 | H-05、P1-01/04：QGC/bootloader 版本、官方 PX4 镜像/hash、可用参数/校准资料、独立入口与工程师结果 |
| 时钟/采样/缓存 | P1、SENS-01～04：本板 VOS/时基、ICM ID/DRDY/轴向、DMA/cache 与实测抖动/丢样 |
| 参数与 SD | P3a、H-04 的首轮卡/测试项、LOG-A01～05：FRAM 原内容备份/介质决定、SD 卡与持久化/停顿结果 |
| CAN 实现与设备 | CAN-A01～04、H-02/H-03：合格纯 Rust v0 层、实际口/适配器/拓扑/终端、ESC 固件/node ID/ESC_INDEX/速率 |
| H-RTK | H-10 的 GNSS 项、GNSS-01～03：SKU/序列号/固件/身份、Fix2/Auxiliary 与额外广播；改正源归 P4b-r |
| QGC 与联合运行 | MAV/PARAM/EST、M1-01/02：TELEM1 接线、上游 QGC 双链路消息/参数、真实 SD 日志与 30 分钟结果 |

与 M1 无关的日志下载/长期策略、非零电机/控制/规划细节在对应后续阶段落实。任务勾选和最终核验状态以本索引与 [TODO](../../../TODO.md) 同步记录为准。

## 7. 最终软件核验

- 四份预算可逐字节重现，默认 profile 确认为 `m1`；七种传输长度与固定参考向量一致。
- 40+3 向量可逐字节重现；两个 Rust 探针 fmt 通过，诊断探针修正了一处换行格式。
- 五份持久备份、归档内 25 个源文件、四个已构建产物的大小/hash 匹配；当前主固件源码及构建输入与已通过的软件基线一致。
- 12 个仓库的 pin/origin、锁文件与许可 hash 核对通过；294 条依赖记录中的 426 个许可文件 hash 匹配本地来源。
- 文档相对链接/标题、源码引用行范围、任务 ID 与阶段依赖已检查；17 个阶段节点无环。关键引脚/启动边界和 HAL 限制另经源码阅读复核，自动行范围检查不代替语义判断。
- 详细检查口径与结果见 [closeout-checks.json](closeout-checks.json)。以上为纯软件核验，P1/M1 的实际恢复、器件和 QGC 验收仍按模板回填。
