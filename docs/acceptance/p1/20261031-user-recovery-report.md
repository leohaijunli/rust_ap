# P1：恢复与QGC检查的工程师报告

日期：2026-10-31。执行者：工程师；记录者：agent。实际数据见[板上记录](20261031-hil.md)。历史缺失资料不由新演练追溯补造。

## 早期报告

> i have verified that the current version can flash px4 back and connect with qgroundcontrol, in mavlink inspector, i could see the heartbeat and other messages

安装本机QGC 5.1.5后，工程师补充：

> now it is connected with qgc

- 当时连接对象明确为Rust固件；恢复入口为重插USB或复位，没有先发Rust软件重启命令。
- 当时“当前版本”未绑定镜像hash；具体消息清单、早期主机/供电/完整过程尚缺。
- 随后工程师要求“start p1”。保存的P0包SHA-256=`d74b3b80e7c35bf4354ea568c09631f2fad362570caff414516ee0fe48a3c02d`；本轮实际回装通过，不据此认定更早演练使用了同一个文件。

## 资料缺失与启动故障

- 工程师确认旧恢复镜像、日志和记录“资料暂时不可得”；原PX4参数、校准和FRAM“没有备份”。不把没有备份等同于板上内容已丢失。
- 早期新Rust上传/校验成功但无CDC；QGC刷回PX4又遇主机锁失败。agent确认`/tmp`配额问题后，工程师反馈：

> you are right, i deleted /tmp folder, then i flashed px4 back

主机锁问题按此结果关闭，当时尚不能单凭该反馈认定整轮P1通过。SCUEN与备份域后续修复及精确包结果见[当前HIL](20261031-hil.md)，原始报告和失败副本保留在[封包](20261031-hil-evidence.md)。

## 本轮授权与实际结果

- 工程师完成前三步普通上传/协议/测量后，明确授权agent执行第4步重启、回装与受控故障；实体操作和QGC由工程师配合。
- 独立入口确认：仅USB供电、断开再插入、未先发Rust重启命令。
- 早一轮PX4采集为v1.17.0、17个心跳和705帧，随后P0/普通回装通过。新普通版协议/平台/备份域、两组65 s测量、两类重启及panic/HardFault保持/清除通过。
- LED最终反馈“i dont have fmu led”；仅可观察I/O/ETH，FMU PE3/PE4/PE5目视项不适用。
- 本次新Inspector截图仅确认HEARTBEAT、SYS_STATUS、SYSTEM_TIME；AUTOPILOT_VERSION重连后仍未出现。早期“四条均可见”不覆盖该缺口。
- 本次QGC实际使用Stable包，上传/校验和独立USB枚举已记录。工程师确认“日志页已关闭，QGC已关”，完整升级UI文本未保存。
- 20:58 UTC独立采得PX4 v1.17.0、board_version=53、19个心跳。当前板上为官方PX4，本次最终普通回装尚未执行。

## 最新执行决定

工程师要求“push to repo”“similify the verification”“only keep the essential tests”，并要求安装clean-code/code-simplifier应用到当前计划。
按[精简计划](../../../plan.md)，已有结果直接复用，当前进度先提交推送；P1只补直接回装普通包并确认身份/运行/无测试故障。额外P0中转、PX4 CRC、齐套UI日志/截图和ETH观察不再设为必验；Inspector版本显示归P2。
