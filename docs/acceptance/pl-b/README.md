# PL-b MAVLink 日志管理有限验收

日期：2026-11-01。**LOG-B01/02 软件与协议级有限 HIL 通过；QGC GUI、断链压力和 PL-a 异常卡门仍待验，PL-b 未关闭。**

## 实现

- SD/FAT 仍由 `src/storage/sd_logger.rs` 单任务持有；记录态拒绝在线列表/下载。
- Disabled/Error 状态的 `LOG_REQUEST_LIST` 建立 10 s 只读管理会话，响应 `LOG_ENTRY`。
- `LOG_REQUEST_DATA` 按 90 B `LOG_DATA` 分块读取；`LOG_REQUEST_END` 卸载并断电。
- USB/TELEM1 请求与响应按原链路路由，跨执行器通道固定容量；新会话清理旧响应，断链发送有 2 s timeout。
- `LOG_ERASE` 明确拒绝，避免 QGC 一键删除未知日志。
- LoggerControl 提供 start/stop/sync/status/apply_config；QGC MAVLink Console 与标准 logging commands 共用状态机。

## 有限 HIL

板载 source=`496484e26b3502fe` 完成下列 1～7 项；`.ulg` 身份修正后构建 `021161d4c09a4b91` 已刷写，board 53/Rev V，输出关闭：

1. MAVLink Console `logger config 500` 读回成功并恢复 1000 ms；Disabled 状态的标准 `MAV_CMD_LOGGING_STOP` 返回 Temporarily Rejected。
2. `logger start` 立即返回 accepted；首次轮询即 Ready。记录约 4 s 后 `logger sync`、`logger stop`，约 300 ms 回到 Disabled；file index 1、306176 B、flushes 7、errors/dropout 0。
3. `LOG_REQUEST_LIST` 初次返回 1 个旧日志；新记录停止后返回 2 个，ID 1 大小 306176 B。
4. ID 1 首 270 B 返回 3 个连续 `LOG_DATA`，ULog magic 正确。
5. 以 4500 B 窗口逐 offset 下载完整 306176 B，所有 offset 齐全；pyulog：file_corruption=false、`imu_icm42688` 4157 样本、dropout 0。
6. 单次请求整个文件时主机接收曾漏中间包并导致 corruption；分窗/重试通过。因此客户端必须按 offset 检查并重试，不能只看最终长度。
7. QGC 实机下载得到 `log_0_UnknownDate.bin`（1660928 B，即旧固件断电结束的 ID 0）。文件头为 `ULog\x01\x125`，内容仍是 ULog；带 offset 校验重下该文件传输缺失 0 包，pyulog 仍报 corruption 与 2 个 dropout，与 LOG-A05 记录的断电未收尾一致。正常 `logger stop` 的 ID 1 无 corruption。
8. `.bin` 是 QGC 命名规则：仅当心跳 `autopilot=MAV_AUTOPILOT_PX4` 时保存为 `.ulg`（无 `SYS_LOGGER` 参数），其他身份一律 `.bin`；`UnknownDate` 来自 `LOG_ENTRY.time_utc=0`（固件无墙上时钟）。因此心跳身份改为 PX4 兼容，仍未声明 FTP 能力，日志继续走消息通道。

新构建 `021161d4c09a4b91` 刷写后复验：`HEARTBEAT.autopilot=12 (MAV_AUTOPILOT_PX4)`；`tools/check_mavlink.py` 全通过（含 20 Hz `HIGHRES_IMU` 与 MAVLink Console）；`LOG_REQUEST_LIST` 仍返回 2 个 `LOG_ENTRY`（1660928/306176 B），`LOG_REQUEST_END` 正常释放。

最终构建 source=`021161d4c09a4b91`，raw/padded=`361856/361856 B`，static RAM=`231672 B`、空余 `226056 B`；`.px4` SHA-256=`319b90ccc2effa9c51a220ba909bac5a6d4373e7e14f5993f31bfe747f4f4d07`，`.bin` SHA-256=`5eda35dfda5662810098c8f3977d8cb23060c15993c51489d9149ac0f3985518`。

## 待验

- 工程师在 QGC **Analyze Tools → Log Download** 实际点击 Refresh/Download，确认下载文件名为 `.ulg`、GUI 进度/取消/重试正常，并能被 pyulog 直接打开。
- 下载中 USB/UART 断链、另一链路并发请求、10 s idle 回收和随后 logger restart。
- PL-a 缺卡/慢卡/拔卡/受控断电；HAL 同步忙等仍可能延迟 console，不能由 async timeout 宣称完全可抢占。
