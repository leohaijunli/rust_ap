# 备份域修复原始证据

结论与工程师待验项见[验收说明](../20261031-backup-domain.md)。本目录记录软件验证；回放使用合成MMIO及RTC/BKP哨兵。

| 内容 | 文件 |
| --- | --- |
| 三版构建身份、资源与逐文件hash | [普通](normal.build.json)、[Dump](dump.build.json)、[注入](injection.build.json) |
| fmt/Clippy、12 Rust、64 Python、构建/打包 | [软件检查](software-check.log)、[注入构建](software-injection.log) |
| 每版132项完整回放 | [普通JSON](normal-replay.json) / [日志](normal-replay.log)、[Dump JSON](dump-replay.json) / [日志](dump-replay.log)、[注入JSON](injection-replay.json) / [日志](injection-replay.log) |
| 普通版14项时钟谓词回放 | [日志](clock-replay.log) |
| 修复中发现的LSE中档编码缺陷 | [失败结果](drive-mapping-failure.json)、[对应构建身份](pre-drive-fix.build.json) |
| 开工前快照与当前源码差异 | [diff](source-review.diff)、[输入范围与hash](source-review-inputs.json) |
| 精确ELF的HardFault调用链 | [普通](fault-chain-normal.txt)、[注入](fault-chain-injection.txt) |
| 固定HAL/PAC/PX4/NuttX与生成mux | [输入hash](upstream-inputs.json)、[mux原文片段](clock-mux-excerpt.txt) |
| 工具hash刷新，固件产物保持 | [回放工具更新](report-refresh-final.log)、[工程师称呼更新](report-refresh-engineer.log) |

`final-check.log`记录最终身份、证据与文档一致性核验。进入本目录执行`sha256sum -c SHA256SUMS`校验文件。

完整ELF/bin/PX4/map、源码快照和生成mux在本机`target/p1-backup-domain-20261031/`，附顶层`SHA256SUMS`；交接固件时一并保存该目录。构建时报告在`at-build-reports/`，回放工具更新后的中间报告在`after-replay-tool-reports/`。
