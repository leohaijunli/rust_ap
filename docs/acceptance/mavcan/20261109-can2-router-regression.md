# CAN2 GUI：USB Router 转发回归

日期：2026-11-09。工程师报告 GPS 接 CAN2、DroneCAN GUI 经飞控 USB/MAVLink 桥接，原先正常、近期不可见，要求检查代码。授权关闭 GUI/QGC 后做一次 3 s 诊断，随后选择最小修复、提交推送、应用区回装与同样 3 s 复核。

## 代码与失败证据

- 原 GUI 通过版本为 `d499d02` 记录的 `a2b91d35630180da`。当前 `src/core/mavcan.rs` 与 `src/drivers/mavcan.rs` 除 crate 改名外一致：CAN2/FDCAN2 PB12/PB13、1 Mbit/s、经典 `CAN_FRAME`/扩展 ID、5 s 租约和过滤均未回退。近期 USL/RGB 不直接修改 CAN2 引脚或这些协议逻辑。
- 固定 pydronecan `092d09cb` 的 MAVCAN 驱动连接时自动发送 `MAV_CMD_CAN_FORWARD`，随后约 1 Hz 续租；GUI bus 2 对应命令 param1=2、`CAN_FRAME.bus=1`。这不是工程师每次手动设置的参数。
- 与已通过版本相比，`3a1bc08` 的单 Router 改造把 CAN 发送改为每轮取一帧、约 5 ms 轮询。接收队列 64、帧寿命 100 ms 不变；近 1000 帧/s 时队列满后的排队约 320 ms，后续帧会持续过期。不能只因 CAN 模块未修改就排除 USB 调度回归。
- 本轮 Rust 6 项 MAVCAN、Python 5 项协议检查通过，但旧固件实物仍失败；没有重跑全库数学测试，也没有用这些 host 结果覆盖实际失败。

### 旧固件 source=`094b08072d1267ac`

- COM26、disarmed；CAN2 ready/registers_ok=true。自动握手 ACCEPTED、续租后 active=true/lease_ms=4976、filters=0，未新增 RX/协议拒绝错误。
- 3.000 s 捕获前后：`rx_frames +2985`、`usb_frames +21`、`rx_dropped +2821`，`rx_queue=64`。全部 21 个 USB 帧在捕获开始约 31 ms 内收到，之后 CAN 接收持续但转发停止，符合满队列/过期失效模式。
- 捕获来自节点 125：type 1003×16、1001×1、1028×2、1029×2；没有 NodeStatus/匿名分配请求。未取得节点身份，不把 125 直接断定为 GPS，也不根据大量丢帧后的缺失断定模块未发 GPS。
- 收尾 CAN_FORWARD bus=0 ACCEPTED、active=false、队列清空，串口释放；未分配节点、改 GPS 参数、启停日志、试音或输出非零命令。
- 原始帧、ACK、前/后/关闭计数：`target/mavcan-20261109/094b08072d1267ac/20261109T214806.624738Z/capture.json`；有界采集程序保存在 `target/mavcan-20261109/inspect_bridge.py`。

## 最小修复与验收边界

- `src/comms/router.rs` 增加非消费式 CAN 队列 readiness 唤醒；USB TX 满时等待可写，避免空转。`select` 取消等待时不提前取走 CAN 帧。
- 控制回复与正常遥测仍先处理，随后每轮最多转发 4 帧；逐帧沿用既有会话/过滤/100 ms 有效性检查。周期兜底计时从本轮工作结束后开始，避免已过期的轮起始 deadline 旁路 TX 背压。
- 不扩大队列、延长帧寿命、提高电压/输出或新增永久参数；CAN1 原生 GPS、CAN2 MAVCAN 桥、节点分配与 RTCM 的边界不变。
- fmt、普通 H753 Clippy 与一次局部 `code-simplifier` 审视通过；只新增实际需要的等待/批次边界，取消安全与 TX 背压已核对。纯核心/协议未改，复用上方 6/5 项通过结果；有界采集程序的 source 参数/语法检查通过。
- 新包的 release/布局、原 bootloader 回装及 3 s 实测待结果补录。实际 GUI 节点显示/身份由工程师确认，不自动放行 RTK、E7/M1、非零输出或此前未验物理项。
