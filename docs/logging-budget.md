# M1 ULog 与 SD 初始预算

日期：2026-11-02。用户首目标中的 `ulogger` 按 **纯 Rust ULog 写入器**落实设计。软件门与实物正常路径有限 HIL 已通过，LOG-A05 故障门待 H-04；结果见 [PL-a 记录](acceptance/pl-a/README.md)。

## 1. 格式与存储选择

| 方案 | 本轮结论 |
| --- | --- |
| 最小 ULog | 已实现；固定 pyulog 1.3.0，完整/尾部截断 fixture 解析通过 |
| Cybflight MCAP + CBOR | 已有编码与 recorder 可参考，但不直接满足 ULog 要求；避免首轮双格式成本 |
| SDMMC2 + FAT | 已构建；默认断电，LoggerControl/MAVLink Console 显式 start/stop/sync；实际卡/HAL/时延待验证 |
| 仅主机串口记录 | 用于早期诊断，不能算板载 ULog 已完成 |

ULog 使用显式小端编码：16 B 文件头、格式/订阅/版本信息、参数快照、DATA 和 dropout/event 记录。DATA 开销为 3 B 消息头 + 2 B msg_id，即 **payload + 5 B**。字段类型/顺序/长度须定义清楚，不能转储 Rust struct 或其 padding。格式规范参考 [ULog file format](https://docs.px4.io/main/en/dev_log/ulog_file_format.html)；LOG-A01 固定实际解析器版本，验证完整/截断/损坏样本。

文件包含本工程/消息定义 commit、板信息、配置、单位/坐标约定、完整参数快照、参数变化与易失/已保存状态。标准测量消息遵循 [PX4 字段约定](message-contracts.md)，原始协议和本项目诊断使用独立前缀。

## 2. M1 topic 预算

以下为 M1 payload 上限，不是 Rust `size_of`；未接入的数据源在各阶段复核。标准 IMU/输出状态已有编码，其余标准消息随真实数据源接入。单位为 B/s，Hz 为所有实例合计。

| Topic | payload 最大 B | Hz | 含 ULog DATA 开销 B/s |
| --- | ---: | ---: | ---: |
| `sensor_accel`（8 kHz 采集每 40 样本记录 1 个，按样本时间戳抽取） | 44 | 200 | 9800 |
| `sensor_gyro`（同一采样时刻） | 44 | 200 | 9800 |
| `vehicle_attitude` | 49 | 250 | 13500 |
| `vehicle_angular_velocity` | 40 | 250 | 11250 |
| `dronecan_raw_command`（四槽全零） | 40 | 400 | 18000 |
| `actuator_armed`（disarmed） | 15 | 10 | 200 |
| `esc_status`（汇总四个 ESC；352 B 为上限） | 352 | 50 | 17850 |
| `sensor_gps` | 256 | 5 | 1305 |
| `dronecan_gnss_auxiliary` | 64 | 1 | 69 |
| `dronecan_node_status` | 20 | 16 | 400 |
| `rust_can_stats` | 80 | 10 | 850 |
| `rust_runtime_health` | 64 | 10 | 690 |
| `events_allowance` | 96 | 10 | 1010 |
| `rust_parameter_change` | 48 | 20 | 1060 |
| `rust_mavlink_stats`（USB/TELEM1） | 64 | 2 | 138 |
| **总计** | | | **85922** |
| **加 25% 格式/事件/余量** | | | **107403** |

M1 5 分钟原始预算约 **26 MB**，加余量约 32 MB；卡需有足够空闲空间。上表按 `LOG_IMU_HZ` 默认 200 Hz 计；把日志参数调高后必须用下面的复算命令重新核预算。启动格式/参数快照突发预算另限 32 KiB，开始正常记录前确认头部完成。参数变更事件超过 20 次/s 时限速或合并，并明确统计；不得无限生成日志事件。

本次新增 `rust_current_sensor`：payload 28 B、DATA 33 B，10 Hz 增加 330 B/s；
在上表预算上合计 86252 B/s，加 25% 为 107815 B/s。

`sensor_gps` 的 256 B 上限用于完整标准字段；Fix2 原始协方差/ECEF 若需独立记录，必须另算可选诊断预算。当前 `dronecan_esc_status` 每条 37 B、四节点合计最高 200 Hz，标准 ESC 汇总接入后该原始诊断默认按需启用，避免重复常驻流量。M1 不记录另外两颗 IMU。

复算命令：

```sh
python3 tools/p0/budgets.py --profile m1
```

上面的命令复算当前标准消息预算；[P0 budget-m1.json](acceptance/p0/budget-m1.json) 保留历史自定义 schema 的 111062 B/s 结果。旧的全系统探索 profile 为 142793 B/s，保留在 `budget-default.json`。

## 3. 缓冲、调度与 SD 能力边界

- 当前为单一 **136 KiB** MPSC 固定槽队列，普通记录准入上限严格保留至少 **8 KiB** 事件容量；运行时最大记录 124 B。生产者只做有界 `try_record`，不调用 SD/FAT。
- 当前已接入的 10 个 topic 均小于 124 B；后续标准 `esc_status` 汇总超过此上限，接入前须调整固定记录/队列布局并重新核验 RAM、编码及准入，不能直接放入现有队列。
- 单一 LoggerControl 状态机提供 start/stop/sync/status/apply_config；stop 先切 Stopping 阻止新记录，再清空队列、对齐 staging、flush file/fs、释放 FAT 所有权并拉低 PC13。错误态也允许重新 start，新会话始终创建新文件。
- sync 周期运行时范围 100–10000 ms，默认 1000 ms；修改只影响后续同步，不改变 ULog schema 或数据采样率。USB 原始控制台、MAVLink Console 和标准 logging commands 共用该状态机。
- 普通记录入口目标 ≤5 us；关全局中断区每次 ≤2 us。大块编码/复制/SD 等待均在临界区外；错误计数更新不会递归产生日志。
- 写入 staging **32 KiB**，32 B 对齐；定义区与每次 sync 用兼容未知消息补齐到 512 B，保持块地址/长度对齐。当前位于 AXI、D-cache 关闭，IDMA 可达性仍须实测。
- 初始流量上限 **256000 B/s**。当前是固定槽而非紧凑字节环：1024 槽中普通记录最多 960 槽，每条 IMU 样本占两槽。M1 提案共 1434 条/s，保守按全部记录计算约 **0.67 s** 无消费窗口；不能用 128 KiB/字节率声称更长缓冲，后续改变槽布局须重算。
- 当前 M1 验收复用 5 分钟联合运行并要求正常段零 dropout；卡资格仍可另做 30 分钟观察。持续有效吞吐目标 ≥512000 B/s，32 KiB 写入 p99 ≤50 ms、观测最大 ≤250 ms。
- 记录环高水位 75% 报警并停用可选 trace；满时丢记录、计数与记录缺口，采样任务继续。事件区也有上限，重复错误合并计数；磁盘写失败保留 USB/遥测诊断。

Cybflight `crates/drivers/src/blackbox_storage.rs` 的 `BlockStore` 是异步 512 B 单块接口；实际应用 `blackbox/sdmmc_block.rs` 另有 CMD25 合并和 32 KiB static 缓冲。其上游注释中的 64 KiB/8 kHz 启动问题、吞吐与停顿记录均不是本板实测。

FAT 已固定为 `embedded-fatfs` commit `518528cc111fcf65c48abbdeb80735a38eada112`，配套 `block-device-adapters`、`embedded-partitions` 与根 `[patch.crates-io] block-device-driver`，支持 MBR FAT 与 superfloppy。文件名固定 8.3 且只创建新文件，不启用 LFN/alloc/std。

HAL `embassy-stm32 0.5.0/src/sdmmc/mod.rs:1043-1142` 存在同步命令/取消等待。为避免默认无卡影响，PC13 保持低且存储任务等待显式 start；启动后的异常卡仍可能阻塞线程执行器，导致 stop/sync 不能及时执行，必须在 H-04 量化。普通 async timeout 不能抢占该忙循环。

## 4. 首轮策略与留后事项

| 决策 | M1 起点 | 硬件结果后的细化 |
| --- | --- | --- |
| D1 格式 | ULog 单格式、独立解析 | 工具兼容性、扩展 topic |
| D2 介质 | 本板 SDMMC2、指定卡 | 卡型号/容量/文件系统/最大停顿依据 |
| D3 访问 | 首轮取 SD 卡解析 | PL-b 至少一种在线下载；USB/MAVLink 选择、分帧/取消/吞吐 |
| D4 记录 | API 显式 start/stop/sync；正常联合负载零 dropout；sync 默认 1 s、可配 100–10000 ms | 轮转/满卡保留、掉电损失、飞行解锁相关策略 |

首轮只创建新文件，满卡报告并停止记录，避免自动删除未知旧文件。生产记录从 `LogReady` 后统计；初始化期间的未记录区间明确标记。采样/接收时间写入 topic，SD 写入时间单列；逐 topic 单调，跨 topic 不强求文件顺序等于测量时间顺序。

完整 PL-a 仍按 LOG-A05 做断电/损坏/移除验证。首轮可提出“最多丢 2 s 完整记录前缀”作为待测值，不能把 1 s sync 当作物理持久化保证；FAT 元数据与卡内缓存必须另测。此时不冻结长期轮转、飞行日志故障动作或在线下载细节。

原始 CAN trace 默认关闭；启用时按独立字节上限限速并重算预算。IMU 内部采集/给 AHRS 的速率为 8 kHz，ULog 每 40 个原始样本记录 1 个（**200 Hz**），不能把 8 kHz 全量写卡而仍宣称本表预算。真实 AHRS/IMU/ESC/GNSS 日志在 M1 验收；控制/任务 topic 随后续硬件测试扩展。
