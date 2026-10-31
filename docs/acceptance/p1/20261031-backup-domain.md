# P1 备份域互操作修复与软件验收

日期：2026-10-31。执行者：agent（源码、构建、host回放、已有文件核对）。**软件修复通过；工程师上板验收待完成，P1未关闭。** 本轮设备操作记录为空；原PX4 bootloader、应用入口`0x08020000`和P0恢复包保持原约定。

## 1. 问题与修复

PX4留下的RTC/LSE配置和BKP字会跨部分复位保留。固定`embassy-stm32 0.5.0`在低速配置不匹配时复位备份域，后续生成的`ClockMux`还会再次写RTCSEL。因此，只指定RTC源或在HAL后补写BKP不足以保持原RTC状态。

新增`src/backup_domain.rs`作为普通、注入及Dump共用的初始化边界：

- 读取继承的BDCR/CSR及全部32个BKP字，同时匹配`LsConfig`与`ClockMux.rtcsel`。
- 保留LSE启用、振荡/旁路、驱动位值及RTC源；RTC选择LSI时启用LSI，其余情况保留原LSI状态。
- HAL后比较BDCR配置掩码`0x0001833d`和全部BKP字，删除原来的BKP补写逻辑。LSERDY/LSECSSD为实时状态，不纳入配置相等条件。
- 普通/注入版要求`preserved()`为真；Dump输出实际保持结果。原故障格式、BKP16..31分配及BKP0 bootloader请求继续使用。

### 回放发现的中档编码缺陷

最初实现按原始数字`1→MediumLow / 2→MediumHigh`转换。实际H753选择`rcc_h7rm0433.rs`，其PAC枚举是`MEDIUM_HIGH=1 / MEDIUM_LOW=2`；HAL按名称转换回PAC，导致两档互换并触发BDRST。通用`rcc_h7.rs`的编码不能代替实际变体。

已归档的中间普通source=`8f9dd4c92a1d2768`，在合成LSE振荡场景中：

```text
BDCR 0x0000810b -> 0x00008113
BDRST writes = 1
BKP sentinels lost; RTC-area sentinels lost; preserved = false
```

[失败JSON](20261031-backup-domain/drive-mapping-failure.json)及[对应构建报告](20261031-backup-domain/pre-drive-fix.build.json)保留该缺陷证据。修复改为匹配实际`Lsedrv`变体，再交给HAL转换；四种原始编码均保持。NuttX还区分芯片修订的驱动档位，故此处枚举名不作为本板物理强度结论。来源详见[硬件台账](../../fmuv6x-hardware.md#rtclse备份域互操作2026-10-31)。

### 支持与拒绝边界

| 继承状态 | 行为 |
| --- | --- |
| RTC关闭且RTCEN=0，或RTC选择LSE/LSI且RTCEN=1 | 匹配配置；LSE被选为RTC源时须LSEON=1 |
| LSE停用且无旁路、LSE振荡或LSE旁路 | 保留兼容组合及四档原始驱动位值；CSS配置保持，实时状态另行记录 |
| HSE作为RTC源、BDRST=1、RTCSEL/RTCEN不一致 | HAL前拒绝；固定HAL不能无损表示或实现 |
| LSE停止却设置旁路或选择LSE RTC | HAL前拒绝 |

拒绝回放止于panic入口，证明其未进入HAL/触发备份域写入。普通/注入版随后仍会按既有故障策略写BKP16..31和BKP0；该回放不宣称拒绝后的故障处理保持全部BKP。Dump的panic停机且不写故障记录。

## 2. schema-2诊断与采集

`P1_PLATFORM schema=2`增加`BACKUP`行；`V6X_DUMP schema=2`增加`V6X_DUMP_BACKUP`行：

| 字段 | 含义 |
| --- | --- |
| `bdcr_before/after`、`csr_before/after` | HAL边界的原始寄存器快照，8位十六进制 |
| `config_preserved` | BDCR配置掩码内前后一致 |
| `bkp_preserved` | 全部32个BKP字前后一致 |

`tools/backup_domain.py`供Dump解码和`p1_capture.py`共用：独立核对原始BDCR掩码、BDRST及固件标记。缺字段、畸形值或保持失败不能被解读为通过；旧报告可继续解码故障，但保持状态是未知。采集门要求有效的schema-2保持证据。新增5项host回归覆盖混合流、旧报告、原始位域/标记不一致与错误输入。

这些字段是启动瞬间的保持检查，不代表RTC日历走时、振荡器质量或全程BKP监测。

## 3. 真实ELF回放

工具：`tools/replay_backup_domain.py`，Unicorn **2.1.4 / Cortex-M7**。加载真实ELF/bin及运行地址的`.data`、零化RAM，从编译出的`main`执行到共享wrapper返回，再调用实际`preserved()`。调用方、完整HAL及生成mux均执行，不猜Rust配置结构的ABI布局。

模型提供即刻就绪的时钟/电压MMIO、RTCSEL写锁、BDRST清空RTC/BKP、APB读门与DBP写门；正常启动触发一次等值BDCR mux写，不写RTC/BKP。RTC区域放20个合成字，BKP放32个不同哨兵，含bootloader标记及CRC有效的合成故障。

| 每版场景 | 数量 | 验证结果 |
| --- | ---: | --- |
| RTC关闭/LSE/LSI × LSE模式/四档驱动/LSI继承，另含CSS及实时状态 | 76 | 返回、配置/BKP/RTC区域哨兵保持；无BDRST或配置改变写入 |
| 不支持的继承状态 | 12 | HAL前拒绝；到panic入口前无备份域写入 |
| 分别扰动BKP0..31 | 32 | 全部被真实保持谓词检测，无补写 |
| 分别扰动9个BDCR配置位 | 9 | 全部检测为不保持 |
| 分别扰动LSERDY/LSECSSD | 2 | 实时状态变化不误判为配置变化 |
| HAL入口后制造驱动不匹配 | 1 | 真实HAL执行BDRST；即使最终BDCR恢复原值，BKP丢失仍被检测 |
| **合计** | **132/版，396项** | **普通、Dump、注入均通过** |

完整JSON及日志：[普通](20261031-backup-domain/normal-replay.json) / [Dump](20261031-backup-domain/dump-replay.json) / [注入](20261031-backup-domain/injection-replay.json)。每项记录BDCR、保持结果、BDRST次数和相关MMIO写入PC。模型和HAL实现并非同一份逻辑；中档编码缺陷即由实际机器码写入暴露。

Dump的WFI-only panic handler被LTO内联到`core::panicking`，初次脚本找不到`rust_begin_unwind`。回放工具现对实际panic入口加停止hook，三版重新通过；未因此改动固件。该初次工具错误日志保存在本机归档。

**模型限制**：非整板仿真；未执行启动汇编/真实bootloader、USB枚举/任务调度或fault handler；未验证供电写一次机制、振荡器启动耗时、中断竞态、RTC日历推进或物理时基。合成RTC区域包含占位字，不能当真实寄存器取值。上述回放不证明本板过去发生过BDRST。

## 4. 产物与检查

Rust 1.99.1，`thumbv7em-none-eabihf`，锁定Cargo.lock，release LTO/opt-level=s。Git HEAD=`5daeca30fcd116203164f44128239e7dd8a884ef`加未提交工作树；18个源码/构建输入和features决定source ID。完整源文件/工具/产物SHA-256在构建报告中，source ID不是密码学签名。

| 镜像 | source | raw / padded flash | static RAM | `.px4` SHA-256 |
| --- | --- | ---: | ---: | --- |
| 普通 | `a946a529360eac83` | 135284 / 135296 B | 17692 B | `7652fa90836be3182becf8f888a052aebc982ecbf67e6ba303dc66719db8fff5` |
| Dump | `167032c8e25576ed` | 35740 / 35744 B | 11020 B | `76ee4e2efe206423bb2fb526fdb966fe2d32389e73f9f07d71d8f3912b2d0ad6` |
| 注入 | `0de073823b066c6e` | 135748 / 135776 B | 17692 B | `013adbb372538c7025033402db72a87453725ef586878ae537187e7cad78a157` |

三版均为board ID=53，应用分区1966080 B、共享栈65536 B、故障栈1024 B；static RAM不是运行期栈水位。

| 检查 | 结果 / 证据 |
| --- | --- |
| fmt、三feature target Clippy、host库Clippy | 通过；warnings作为错误 |
| Rust host核心测试 | 12通过 |
| Python工具回归 | 64通过 |
| 普通/Dump构建、打包与ELF/bin/向量/布局检查 | [software-check.log](20261031-backup-domain/software-check.log)通过 |
| 注入构建、打包及相同检查 | [software-injection.log](20261031-backup-domain/software-injection.log)通过 |
| 真实备份域机器码回放 | 三版各132项通过，见上表 |
| 新普通版时钟谓词 | [14项通过](20261031-backup-domain/clock-replay.log)；仍用历史实测CR3和合成P1时钟 |
| 普通/注入HardFault调用链 | 重核最深软件栈132 B；[审查](fault-review.md)及两份反汇编已更新 |

回放工具修正及工程师称呼更新后刷新构建报告中的工具hash；ELF/bin/PX4/map的hash不变，原构建时及中间报告亦保存在归档。P0恢复包SHA-256仍为`d74b3b80e7c35bf4354ea568c09631f2fad362570caff414516ee0fe48a3c02d`。CI未远程执行。

### 离线复现

```sh
TMPDIR=/tmp/opencode bash tools/firmware.sh check
TMPDIR=/tmp/opencode bash tools/firmware.sh build-injection
export PYTHONPATH=/tmp/opencode/v6x-platform-replay-deps
.venv/bin/python tools/replay_backup_domain.py target/thumbv7em-none-eabihf/release
.venv/bin/python tools/replay_backup_domain.py target/p1-fault-dump/thumbv7em-none-eabihf/release
.venv/bin/python tools/replay_backup_domain.py target/p1-fault-injection/thumbv7em-none-eabihf/release
.venv/bin/python tools/replay_platform_check.py target/thumbv7em-none-eabihf/release --dump docs/acceptance/p1/20261031-startup-dump.txt
```

可选Unicorn的安装来源见[provenance](../../provenance.md)。`--json <新文件>`可保存完整逐场景结果；脚本仅在全部通过后写JSON，失败须同时保留终端日志。

### 归档

仓库内[证据目录](20261031-backup-domain/README.md)，含构建报告、原始检查输出、三版完整回放JSON/日志、缺陷JSON、[源码diff](20261031-backup-domain/source-review.diff)、故障链及输入hash。diff覆盖开工前快照可对照的34个固件/host输入，其中11个变化；CI文件不在旧快照范围内，其当前hash见构建报告。

上游清单固定17个引用文件、两个crate归档及三版ELF调试信息所指的生成mux。普通构建目录另有旧TIM12缓存；本轮依据ELF中的OUT_DIR选择TIM2版本，三版生成文件hash相同，见[清单](20261031-backup-domain/upstream-inputs.json)与[mux片段](20261031-backup-domain/clock-mux-excerpt.txt)。

本机完整归档：`target/p1-backup-domain-20261031/`（git忽略），含`normal/`、`dump/`、`injection/`的ELF/bin/PX4/map/sections/disassembly；`pre-drive-fix/`保存中档缺陷版本及匹配源码，`before-sources.tar.gz`保存开工前快照，`sources.tar.gz`保存本轮源码/工具/文档。目录内`SHA256SUMS`用于校验；交接二进制时一并复制。

## 5. 工程师待验与关闭条件

按[上板操作单](bring-up.md)及[恢复模板](recovery-template.md)执行，记录实际执行者、日期、供电、包路径/hash和完整输出：

1. 补齐P1-01/H-05官方PX4镜像/版本/hash、独立bootloader入口与往返日志。原资料暂不可得、原参数/校准/FRAM无备份的状态继续如实保留。
2. 核对新普通版`!platform`的source和schema-2 `BACKUP`；关闭QGC后运行`tools/check_mavlink.py --source-id a946a529360eac83`，保存完整协议结果。重建后如source改变，以本次build.json为准。
3. 完成LED、完整寄存器、≥60 s空载/USB压力时基、1 kHz替身、CPU/栈验收；保留`wire.jsonl`和`summary.json`。
4. 完成正常文本/MAVLink重启、持续备份域供电的panic/HardFault记录保持及QGC→官方PX4→Rust恢复，保存每次schema-2报告、故障CRC和镜像身份。

已有普通QGC基本连接报告不等于本轮新包验收。上述必验项完整通过并回填后，才关闭P1、提交并推送仓库；依赖P1的后续阶段按TODO放行。
