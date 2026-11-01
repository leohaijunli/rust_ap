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

板载 source=`496484e26b3502fe`，board 53/Rev V，输出关闭：

1. MAVLink Console `logger config 500` 读回成功并恢复 1000 ms；Disabled 状态的标准 `MAV_CMD_LOGGING_STOP` 返回 Temporarily Rejected。
2. `logger start` 立即返回 accepted；首次轮询即 Ready。记录约 4 s 后 `logger sync`、`logger stop`，约 300 ms 回到 Disabled；file index 1、306176 B、flushes 7、errors/dropout 0。
3. `LOG_REQUEST_LIST` 初次返回 1 个旧日志；新记录停止后返回 2 个，ID 1 大小 306176 B。
4. ID 1 首 270 B 返回 3 个连续 `LOG_DATA`，ULog magic 正确。
5. 以 4500 B 窗口逐 offset 下载完整 306176 B，所有 offset 齐全；pyulog：file_corruption=false、`imu_icm42688` 4157 样本、dropout 0。
6. 单次请求整个文件时主机接收曾漏中间包并导致 corruption；分窗/重试通过。因此客户端必须按 offset 检查并重试，不能只看最终长度。

最终未刷写构建 source=`ad6b017000ad127e`，raw/padded=`361864/361888 B`，static RAM=`231672 B`、空余 `226056 B`；`.px4` SHA-256=`5622749dbdc6dc0e763f482008640a457d68781378445318209259fe6f3004de`。

## 待验

- 工程师在 QGC **Analyze Tools → Log Download** 实际点击 Refresh/Download，确认 GUI 进度、取消和重试。
- 下载中 USB/UART 断链、另一链路并发请求、10 s idle 回收和随后 logger restart。
- PL-a 缺卡/慢卡/拔卡/受控断电；HAL 同步忙等仍可能延迟 console，不能由 async timeout 宣称完全可抢占。
