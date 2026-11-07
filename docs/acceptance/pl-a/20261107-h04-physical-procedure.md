# H-04 实物步骤：受控断电、缺卡启动、写日志中拔卡

日期：2026-11-07。适用：source=`a17a2d35bab6f05a`、32 GB 普通卡、COM4、USB 供电、disarmed。

前置条件（每步都保持）：桨叶已拆、动力电池断开或未接、不设参/不保存、不解锁、零输出。电源与 SD 卡只由工程师动手；主机侧只做只读 MAVLink 查询与日志下载。

## 1. 受控断电（写日志中）

1. 工程师告知开始；主机启动 `logger start`，确认 `bytes_written` 持续增长、`status=2`。
2. 至少写满 20 s 后，工程师直接断开 USB（若接了电池一并断开）。
3. 等 5 s，重新上电（USB 插回）。
4. 主机执行只读验证：端口重新枚举、heartbeat 为 disarmed、source 不变、`!fault`、日志列表、下载被中断的那条日志、pyulog 解析。

判定要点：

- 文件存在且能被 pyulog 解析，`file_corruption=false`；已 sync 部分完整，最多丢失最后一个 sync 周期（1000 ms）。
- 重启后不进入 fault、参数保持不变、新日志仍可启动。
- 若文件尾部被截断但可解析，记录实际保留时长；这是断电场景的预期行为。

## 2. 缺卡启动

1. 断电，取出 SD 卡，再上电。
2. 主机验证：heartbeat 正常、`!fault`、`logger status` 显示不可用状态及 stage，无 panic/重启循环；QGC 仍能看到机体。
3. 尝试 `logger start`，应被拒绝或进入 Error，不得卡死；记录返回文本。
4. 断电，插回卡，再上电；主机验证旧日志仍可列出、`logger start` 成功、`logger stop` 回到 Disabled。

## 3. 写日志中拔卡

1. 主机启动 `logger start` 并用 `tools/h04_watch.py` 记录 heartbeat 与 logger 状态。
2. 写满 20 s 后，工程师在 USB 保持连接的情况下取出 SD 卡。
3. 主机继续观察 ≥15 s：heartbeat 不得中断，不得重启；logger 应进入 Error 并记录错误计数。
4. 断电，插回卡，再上电；主机验证卡上既有日志仍可列出/下载、新日志可写。

判定要点：

- 不允许 panic、重启循环或 QGC 掉线。
- 被中断的那个文件允许不完整；卡本身必须仍可用（既有日志可读、新日志可写）。若整卡不可用，记为失败并保留卡状态。

## 证据与回传

主机把每次结果写入 `target/h04-logging/` 下对应目录的 `verify.json` / `watch.json` 与下载的 `.ulg`。工程师只需回传：做了哪一步、断电/拔卡时刻、面板 LED 或蜂鸣器变化、QGC 是否掉线。最终判定写入 H-04 收口记录，不预先承诺通过。
