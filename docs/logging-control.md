# 日志控制：启动、状态与故障定位

更新时间：2026-11-02。实现见 `src/storage/sd_logger.rs`、`src/core/logger.rs`、`src/comms/usb_console.rs`。

## 1. 启动方式

| 入口 | 命令 | 说明 |
| --- | --- | --- |
| QGC 控制台 | `!logger start` 或 `logger start` | 两种前缀都接受；旧写法 `!logger-start` 仍可用 |
| QGC 控制台 | `!logger` / `!logger status` | 查看状态机快照 |
| MAVLink | `MAV_CMD_LOGGING_START` / `MAV_CMD_LOGGING_STOP` | 走同一状态机并回 `COMMAND_ACK` |
| 停止 | `!logger stop` / `logger stop` / `!logger-stop` | 仅 `status=2` 时接受 |
| 落盘 | `!logger sync` / `logger sync` / `!logger-sync` | 立即 flush 文件与文件系统 |
| 周期 | `!logger config <100..10000>` / `!logger-config <…>` | 运行期改同步周期，单位 ms |

库内部命令解析位于 `logger::parse_console`，设备端 `usb_console::CommandLine` 只在收到整行
（`\n` 或 `\r`）时派发一次。

## 2. 期望回显

```
LOGGER_CONTROL operation=start accepted=true error=None
```

`accepted=true` 只表示**请求已入队**；真正开始记录要等 SD 上电、初始化、挂载和建文件完成。
1 s 后查询：

```
LOGGER schema=2 status=2 stage=... file_index=0 bytes_written=... flushes=... errors=0 sync_period_ms=1000 ...
```

`status` 取值：

| 值 | 名称 | 含义 |
| --- | --- | --- |
| 0 | Disabled | 未记录，可接受 start |
| 1 | Initializing | start 已接受，正在建会话 |
| 2 | Ready | **正在记录**，可接受 stop/sync |
| 3 | Stopping | 正在 drain/flush/断电 |
| 4 | Error | 会话失败，可再次 start；看 `stage` 和 `errors` |
| 5 | Managing | 列表/下载会话，start 会被拒绝 |

`stage` 是故障定位用的阶段号：`1x` 为建会话阶段，`2x` 为管理（列表/下载）阶段，具体值随实现演进，
`errors` 递增表示失败次数。

## 3. 常见“start 不生效”的原因

| 现象 | 原因 | 处理 |
| --- | --- | --- |
| 命令无任何回显 | 行未被解析（拼写/前缀不匹配） | 用上表写法；2026-11-02 前的固件只认 `!logger-start`，见第 4 节 |
| `accepted=false error=Some(InvalidState)` | 已在记录（2）、正在停止（3）或管理（5） | 先 `!logger` 看 `status`，必要时等 1 s 重试 |
| `accepted=false error=Some(QueueFull)` | 控制队列满 | 重试一次；连续出现说明记录任务被卡住 |
| `accepted=true` 但 `status=4` | SD 上电/初始化/挂载/建文件失败：缺卡、接触不良、FAT 异常 | 看 `stage`、`errors`；换卡或修复文件系统后重试 |
| 记录正常但 QGC 列表看不到 | 记录态禁止 `LOG_REQUEST_LIST`/下载 | `!logger stop` 关闭文件后再 Refresh |
| 日志有文件但日期是 `UnknownDate` | RTC 未授时，`LOG_ENTRY.time_utc=0` | 见 [时间基准](time-sync.md)；与能否记录无关 |

## 4. 本次排查记录（2026-11-02）

**现象**：工程师在 QGC MAVLink Console 输入 `!logger start`，设备**完全没有回显**，
`!logger` 仍是 `status=0`，看起来“启动不了”。

**定位**：设备端日志控制解析器只接受 `!logger-start` 和 `logger start` 两种写法，
`!logger start` 落入“未匹配”分支，`CommandLine::push` 返回 `None`，既不执行也不回错误。
同一目录下的 `!logger stop` / `!logger sync` / `!logger config …` 有同样问题。
`!logger` 本身可用，所以状态查询正常，容易误判为状态机问题。

**修复**：`src/core/logger.rs::parse_console` 现在同时接受
`!logger-start`、`!logger start`、`logger start`（stop/sync/status/config 同理），
并在 host 测试 `console_commands_and_state_transitions_are_explicit` 中覆盖四种写法。

**遗留改进**：未匹配的 `!` 命令目前仍是静默丢弃。需要时再把 `CommandLine` 的 `None` 升级为
`ERR unknown-command` 回显，避免同类拼写问题再次表现为“设备没反应”。

## 5. ULog 中的 IMU 字段

本项目当前不复制 PX4 的 `sensor_accel` / `sensor_gyro` topic 名；六轴原始数据在
`imu_icm42688` 中：

- 角速率：`gyro_rad_s[0]`、`gyro_rad_s[1]`、`gyro_rad_s[2]`；
- 加速度：`accel_m_s2[0]`、`accel_m_s2[1]`、`accel_m_s2[2]`；
- 另有 `sample_timestamp`、`sequence`、`age_us` 和 `temperature_c`。

因此按 PX4 标准 topic 名筛选会误以为没有 IMU 数据。2026-11-02 从 source
`81d6c1221048f08a` 板上下载的最新日志经 pyulog 1.3.0 解析：2686 条 IMU 记录、
2.652 s、约 1012.5 Hz，上述六个字段均有有限实数，`file_corruption=false`。

## 6. 验证命令序列

```
!logger
!logger start
（等 2 s，写入若干条日志）
!logger
!logger stop
（等 2 s）
!logger
```

判据：start 后 `status=2` 且 `bytes_written` 递增、`errors=0`；stop 后回到 `status=0`，
`flushes` 增加；文件出现在卡上且可被 pyulog 解析。
