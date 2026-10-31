# P1 证据索引

日期：2026-10-31。**P1已完成。软件、普通版运行/测量、重启、故障保持/清除及独立QGC往返通过；21:28 UTC最终普通回装确认成功，当前板上为普通Rust `a946a529360eac83`。**
必验范围以[计划](../../../plan.md#4-仅保留必要测试)为准；下一阶段为P3a。

## 当前证据

- [板上记录](20261031-hil.md)：精确包、时序、故障与恢复结果、QGC实际包及最终普通回装结论。
- [证据封包](20261031-hil-evidence.md)：本轮原始记录与最终回装增量包，均有SHA-256清单及逐成员校验。
- [备份域软件记录](20261031-backup-domain.md)：共享wrapper、schema-2、H753 LSE编码及回放结果。
- [工程师报告](20261031-user-recovery-report.md)：历史确认、缺失备份、授权与最新精简决定。
- [HardFault审查](fault-review.md)：裸入口、帧边界、独立栈、调用链和限制。
- [早期操作单](bring-up.md)、[恢复模板](recovery-template.md)：作为参考，次数/材料要求按当前精简计划执行。

## 验收固件

| 产物 | source ID | raw / padded flash | static RAM | 构建报告 |
| --- | --- | ---: | ---: | --- |
| 普通0.2.0 | `a946a529360eac83` | 135284 / 135296 B | 17692 B | [normal.build.json](20261031-backup-domain/normal.build.json) |
| fault-dump | `167032c8e25576ed` | 35740 / 35744 B | 11020 B | [dump.build.json](20261031-backup-domain/dump.build.json) |
| fault-injection | `0de073823b066c6e` | 135748 / 135776 B | 17692 B | [injection.build.json](20261031-backup-domain/injection.build.json) |

普通包SHA-256=`7652fa90836be3182becf8f888a052aebc982ecbf67e6ba303dc66719db8fff5`。fmt、三feature target Clippy、host Clippy、12 Rust / 64 Python、三版构建/打包/布局通过；三版备份域ELF回放各132项，普通版时钟谓词14项通过。

构建基于`5daeca30fcd116203164f44128239e7dd8a884ef`及当时工作树，不能只靠该HEAD重建验收包。完整ELF、源码快照、反汇编与113项SHA清单在`target/p1-backup-domain-20261031/`；仓库保存[报告/日志/回放JSON](20261031-backup-domain/)。历史封包保持原样。上述为本机结果，远程CI结果另计。

## 历史定位与基线

| 记录 | 范围 |
| --- | --- |
| [初始软件输出](software-check.log)、[注入构建](software-injection.log) | 最初P1软件基线，非当前包身份 |
| [启动回归](20261031-startup-regression.md)、[工程师上传日志](20261031-user-upload.log) | 早期上传/校验成功但无CDC；该包hash未知 |
| [QGC主机锁问题](20261031-qgc-lock.md) | `/tmp`配额导致锁失败，工程师清理后恢复 |
| [PX4上传入口](20261031-upload-discovery.md) | 默认端口匹配修复与host回归 |
| [Dump监看](20261031-monitor-dump.md)、[重枚举](20261031-monitor-rediscovery.md) | 诊断端口发现、原始保存及错误报告 |
| [SCUEN定位/修复](20261031-startup-assertion.md) | 有效PANIC与实测PWR_CR3，前后机器码回放 |
| [Dump原始数据](20261031-startup-dump.txt)、[monitor JSON](20261031-startup-monitor.json) | 真实启动诊断 |
| [修复后检查](20261031-startup-fix-software.log)、[代码核对](20261031-startup-code-review.log) | 当轮12 Rust / 59 Python及构建/回放 |
| [Rust/QGC连接复验](20261031-qgc-retest.md) | 16:15普通CDC恢复，后续精确source验收见当前HIL |

历史普通source依次包括`6852872943daf5a9`、失败包`afaa140f499ff958`、SCUEN修复包`4cba95aa54cd899a`；故障采集Dump为`4f8bbf296699e6f2`。这些包与当前备份域修复包分开绑定。
历史完整归档在`target/p1-software-20261031/`及`target/p1-startup-20261031/`；保存的P0包在`.p0-baseline/20261031/`，也随本轮HIL镜像封包保存。

## 结论边界

当前板上数据支持P1整数周期替身、寄存器/备份域、CPU/栈有限观测及正常/受控故障恢复；不声明M1数据链、WCET、≤2 µs全局屏蔽、RTC走时或任意栈损坏恢复。
FMU灯目视项不适用，原参数/校准/FRAM无备份。完整QGC升级UI文本缺失，Inspector版本消息未出现，事实均保留；版本显示归P2。
