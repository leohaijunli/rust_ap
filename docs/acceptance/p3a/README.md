# P3a 参数 API 与 FRAM 验收

日期：2026-10-31（UTC）。**P3A-01～04 必验通过，P3a 已关闭。**
当前板上普通 Rust source=`1d3d08e3dd64f3ac`，disarmed、fault_injection=false；`MAV_SYS_ID=1`、`LINK_LED_HZ=2.0` 已保存，启动读回 dirty=false。

## 结果与精确产物

| 检查 | 结果 / 原始目录（相对 `target/p3a-20261031/`） |
| --- | --- |
| 只读识别 | FM25V02A、32768 B，ID=`7f7f7f7f7f7fc22208`、SR=`00`、MODE3；SPI5 配置读回 937500 Hz；`fram-backup/` |
| 全量备份 | 22:00 UTC，128 块各 256 B，全量复读一致；坏 MAVLink 帧 0；`fram-backup/fram.bin` |
| 软件检查 | 已通过 Rust fmt、普通 release Clippy、17 项 host 测试；复用此前 64 项 Python 结果，验收脚本修正后另补 3 项针对性测试通过 |
| 构建/安装 | 普通配置 raw/padded=167228/167232 B，static RAM=18456 B；分区、向量和 ELF 检查通过；上传预期/实报 CRC=`7199EF25`；`parameters-image/`、`install-parameters-reconnected/` |
| 参数与保存 | 23:57:04–09 UTC，全表与诊断一致，合法设 2 Hz 回显；0/11/NaN 和只读修改均拒绝并报告原因；唯一一次 storage save ACK=accepted；`parameters-save/` |
| 保存独立校验 | 槽 A=`0x7800`、生成号 1、格式 1、schema=`9badaa1a`、CRC=`a8242281`、值 `[1.0, 2.0]`；槽 B 未提交；有效记录外所有字节与备份一致 |
| 掉电读回 | 工程师确认断开全部电源至少 5 秒再接 USB；23:57:40–42 UTC 只读验收，自动加载 Stored/valid=true/dirty=false；槽/生成号/值正确，整片哈希未变；`parameters-power-cycle/` |

- 普通 `.px4` SHA-256：`c19ec4b0023dddc0247cc4867fe9ce42351982d8a66c7383899f5bec994a3bd2`。
- 只读 source=`aa50a4dfe7893747`，上传 CRC=`444A9085`；先只读备份，后安装参数包。
- 写入前备份 SHA-256：`600969004d68715044b55828cf5288c7af02d1bfb0c97e437f6480d3e5a45097`，CRC32=`a2cb3d91`。
- 保存/掉电后 FRAM SHA-256：`7ce0d75fc4200a2ddabda1a8790fbacd66443ab5a26fe94fdafb84e67c797c29`。
- 本次验收工具 SHA-256：`d0f3f59961da07cf8e1c1098c6f40a153923acb1d42c54dca1d787b4f4110a57`。

## API、边界与升级

`src/core/parameters.rs` 的唯一注册表生成 `Id`、名称/索引查找及表顺序；新增参数只需登记并接入功能侧读取。共享 `ParameterManager` 提供 `get/set/is_dirty/status` 和异步 `load/save`；核心快照与校验不依赖 HAL，MAVLink 和控制侧共用 RAM 真源。使用和格式见[架构](../../architecture.md#p3a-参数-api-与存储契约)。

- Rust 只写末尾 2 KiB：槽 A=`0x7800–0x7BFF`、槽 B=`0x7C00–0x7FFF`。本次仅槽 A 的 32 字节记录发生变化。
- 先使目标槽失效，再写/读回正文，最后提交单字节并最终读回。逐字节中断、CRC 损坏及生成号回绕在 host 验证；实物只做一次保存后完全掉电，不声称做了真实写入中断电或耐久性验证。
- 无兼容有效槽时使用默认值，状态明确显示原因；不自动迁移、不在启动/load 写 FRAM。加载失败保留当前 RAM 并报告 Unavailable；`set` 失败不改当前值。
- 保存要求 disarmed 且输出关闭。P3a 无解锁/输出通路，使用固定关闭状态；后续 P5a 必须接入输出仲裁真源。
- 原 PX4 bootloader、128 KiB 保留区与 `0x08020000` 入口保持；P1 的独立 QGC 往返结果复用，不重复刷回。应用刷写不擦 FRAM；后续 PX4 参数保存可能覆盖 Rust 区。
- 备份包含 **P3a 写入前的当前 PX4 参数/校准记录**，不是项目开始前的原始配置备份；其末个非零字节在 `0x6C57`，Rust 区当时全 0。
- 本板无可观察 FMU 灯；2 Hz 已验 RAM/存储与功能读取路径，不声称完成灯频率仪器测量。TELEM1、双链路和 QGC 参数页归 P2/P3。

## 中断与局部审视

主机重启后两次出现 CDC 正常枚举但零数据：`install-read-only/`、`install-parameters/` 均在 prepare 的心跳检查停止，未发 reboot、未刷写。工程师 USB 拔插后各安装成功；无数据现象根因未确认，不能仅凭描述符判定应用健康，后续归 P2。

已按 `code-simplifier` 做一次局部审视：去掉验收脚本未用的生成号选择/长度包装，修正批量回包游标、警告作用域与空白槽处理，合并存储读回校验；固件源码 hash 与已通过构建一致，不重复构建/刷写。

## 原始证据

[封包](20261031-evidence.tar.gz) / [SHA-256 清单](20261031-evidence.sha256) 保存两版精确镜像/ELF/map/源码、全量备份、上传与 TX/RX、失败记录、保存/掉电读回结果、当次工具和参考规格；排除 Python 缓存。清单对封包 42 个成员逐项 SHA-256 校验。

构建报告的固件基线 HEAD=`ceb095690b8c3f22fd8d485658e769a68cee095f` 加当时工作树；本次提交前固件来源 hash 全部一致。验收工具在构建后修正，以保存/verify 的 `script_sha256` 为准，旧工具快照仍随精确构建源码保留。软件检查终端通过结果复用，不把后补说明冒充原始终端输出。
