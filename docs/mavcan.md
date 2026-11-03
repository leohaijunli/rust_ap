# MAVCAN：USB ↔ CAN2

当前范围与必验项见 [plan.md](../plan.md#can-brd-当前范围)，结果见[验收记录](acceptance/mavcan/README.md)。

## 接口约定

| 项目 | 约定 |
| --- | --- |
| 物理总线 | Pixhawk 6X CAN2，FDCAN2 RX=PB12 / TX=PB13；HSE 16 MHz，经典 CAN 1 Mbit/s，采样点 87.5% |
| USB | 原 Rust CDC 的 MAVLink 2；与遥测共用序列号和发送入口 |
| 开关 | `COMMAND_LONG` / `MAV_CMD_CAN_FORWARD`（32000），param1=2 开启、0 关闭；返回定向 `COMMAND_ACK` |
| 帧 | `CAN_FRAME`（386），bus=1（零起始）；ID bit31=扩展、bit30=RTR，DLC=0～8，保留原始字节 |
| 过滤 | `CAN_FILTER_MODIFY`（388），bus=2（一起始）；按 CAN ID 的第 2/3 字节，replace/add/remove，最多 32 个 ID；空表接收全部 |
| 会话 | 单一 MAVLink system/component，5 s 租约；pydronecan 每秒续约；关闭、USB reset/失连、过期清队列并取消待发帧 |
| 边界 | disarmed 且输出关闭；拒绝错误目标、总线、ID/DLC、过期帧和 CANFD_FRAME；RX/TX 队列分别 64/32 帧，积压年龄上限 100 ms |

CAN2 保持正常接收/ACK，透传由会话控制。USB 出队再次核对会话和过滤；遥测节拍与命令回复优先于 CAN 流量。CAN TX 仅一帧在途，5 ms 内检查 `TXBRP/TXBTO`，超时写 `TXBCR` 请求取消。

固定 Embassy 0.5.0 在 DLC 非零时会清 RTR。因此 CAN2 TX 使用 HAL 初始化与所有权、PAC 写 Classic TX element 后再置 `TXBAR`。启动核对 64 B element、3 槽 FIFO，以及 HAL 的独立消息 RAM `0x0C00–0x0F4F`；CAN1 使用独立区域。

## 工程师复测

本次工程师已确认测试节点接 **CAN2**，DroneCAN GUI 使用 **bus 2**。加载普通包后，先让验证脚本独占飞控 USB：

```bash
.venv/bin/python tools/check_mavcan.py --check-timeout
```

脚本打印固件 source、握手 ACK、CAN_FRAME 样例、NodeStatus 节点、GetNodeInfo 响应帧、租约失效/重开及 USB 心跳。默认用未占用的节点 ID 127 发 GetNodeInfo；可用 `--local-node-id` 指定空闲 ID、`--node-id` 指定目标、`--source-id` 核对包版本、`--port` 指定设备（Windows 可用 `--port COMx`）。省略 `--check-timeout` 时检查显式关闭/重开。

随后在 DroneCAN GUI 连接该 Rust CDC、MAVCAN、bus 2，核对节点及名称可见。GUI 节点名称用于确认完整 GetNodeInfo 解码；脚本的响应帧证明 CAN2 双向收发。

`!mavcan`（也可通过 MAVLink Console 发送 `mavcan`）提供诊断：

- `ready=false` / `registers_ok=false`：CAN2 初始化、时钟、模式或消息 RAM 配置异常。
- `active=false`：检查 CAN_FORWARD ACK 与续约；`owner_sys/owner_comp/lease_ms` 显示当前客户端。
- 活跃会话下 `rx_frames` 不增加：检查 CAN2 节点供电、线序、终端和 1 Mbit/s。
- `rx_frames` 增加而 `usb_frames` 不增加：结合 `rx_filtered/rx_dropped`、过滤表和会话定位转发。
- `tx_received` 增加而 `tx_frames` 不增加：结合 `tx_errors/tx_timeouts` 与 `MAVCAN_HW` 的 `psr/ecr/txbrp` 定位 CAN ACK/发送失败。

## 固定参考

- [VimDrones MAVCAN_Bridge](https://github.com/VimDrones/MAVCAN_Bridge/tree/7bfe9963bbbe42309f3b1d0851f6a138552ce7f6)：核对 `Core/Src/main.c` 的 CAN_FRAME、扩展/RTR 标志和经典 CAN 行为。
- [pydronecan](https://github.com/dronecan/pydronecan/blob/092d09cb9e6b115e262efe4cb42f7b65e6054f26/dronecan/driver/mavcan.py)：核对一起始握手/过滤总线、零起始帧总线、客户端身份和 1 Hz 续约。
- MAVLink `common.xml` 使用锁定的 rust-mavlink 0.17.1 生成定义；[来源台账](provenance.md)。
