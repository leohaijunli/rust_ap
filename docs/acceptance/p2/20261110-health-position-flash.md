# 位置发布＋AP_HEALTH 合并包

日期：2026-11-10。工程师授权“烧录新固件、更新文档”，随后选择标准单条 `DEBUG_FLOAT_ARRAY` 汇总 EKF／传感器实际 Hz 与健康；只写原 PX4 bootloader 的应用区，不设参/保存、不启停/删除日志、不试音/分配节点或启用非零输出。

## 更新内容

- 包含既有 `48d2d62` 的位置数据有效性／飞行资格拆分：仅健康、新鲜 Filter 的真实有效解算可发布 LOCAL/GLOBAL；停用、陈旧或 Mahony 回退撤销，不用 GPS_RAW_INT 冒充融合位置。
- 新增 `DEBUG_FLOAT_ARRAY` #350、`name=AP_HEALTH`、`array_id=1`；每约 1 s 用成功计数增量和 TIM8 实际窗口求 Hz，两链路共用缓存。EKF Hz 是延迟核心更新，gyro/accel 是成功 FIFO 原始样本，另含 BMP388/BMM150/GNSS Fix2/INA219、健康掩码、年龄及故障/CPU 诊断。[字段与边界](../../mavlink-health.md)。
- 注册表末尾追加 `MAV_RATE_HEALTH`（默认 1 Hz、0 停发、最多 2 Hz），350 的 511 正间隔至少 500000 µs；REQUEST_MESSAGE/510/511 复用原 Router。原 227 项顺序/值/范围和 FRAM 槽不改。
- INA219 保持 `DEBUG_VECT` #250/name=INA219/V/A/W、BATTERY_STATUS 及 ULog；旧文档的 252 为编号笔误，生产类型原本正确，本轮不迁移电流消息。
- 不改估计数学、GNSS/CRC 门、IRQ/FPU 模式、IO/bootloader、控制或飞行资格。首窗/重置/异常时基为 NaN，缺样实际 Hz 为 0 与不健康分开。

## 必要软件检查：通过

- Rust 19 项：健康 4、参数注册/防抖 3、存档兼容/原子性 8、调度/间隔 3、支路电源映射 1。覆盖计数窗口/回绕/重置/epoch、陈旧/停用/回退、不可用/禁用源和标准编码；350 禁发/限速不影响原 DEBUG_VECT 10 Hz 默认。
- Python 元数据 4 项、生成及 `--check` 通过；仅追加健康参数及元数据 CRC，原目录条目不变。
- fmt、host 库/测试与普通 H753 Clippy（`-D warnings`）通过；一次局部 `code-simplifier` 审视完成，主动 Mahony 不发布为 EKF Hz，单 Router 采样且不消费传感器队列。
- 初次测试构造误用了 `MAVLinkV2MessageRaw::new` 参数，按锁定 API 改为 new＋serialize 后通过；新增独立流断言沿用旧文档 252 编号而失败，核对 common.xml/pymavlink 后改用 `DEBUG_VECT_DATA::ID`，通过。没有修改生产 INA219 类型或放宽检查来迁就测试。
- 未改核心数学，复用 `aa795b9` 已通过 CI 的 PX4 对拍；本地不重复全库回放/全特性构建。原始检查输出在 `target/flash-position-20261110/host-checks.log`、`h753-clippy.log`。

## 烧录前与交付

- 首次烧前只读快照：COM4/source=`32269dec1d8137b4`、3 条 disarmed 心跳、227 项参数；Logger Disabled、pending_save=false、历史故障保持。Filter/Running、无故障/数值失效/漏期，峰值 1573 µs/2000 µs、CPU 33.2%、栈余 30980 B。证据 `target/flash-position-20261110/before-20261110T054057485597Z.json`。
- 本轮最初仅位置版本从 `aa795b9` 构建 source=`e8ac3f78e928873c`、普通包/布局通过但**未回装**；与历史 `f2d8ad0593f361b1` 身份差异来自 Windows 工作副本 CRLF，109 项源码逐项与 Git blob 比较仅换行不同。该中间包保存在 `target/flash-position-20261110/e8ac3f78e928873c/`，追加健康后不刷此包。
- 合并普通包的构建/source/SHA-256 与本轮应用区烧录、只读保持及消息核对结果待追加；未完成前板载仍为 `32269dec1d8137b4`。

本记录只覆盖本轮软件与有界回装范围；QGC 实际界面、TELEM1、外设失效/恢复、完整 M1/E7/RTK 与飞行资格仍按原门验收。
