# MAVCAN：USB ↔ CAN2 桥接

日期：2026-11-03。**修复已验证：经典 CAN 帧现经 `CAN_FRAME`（386）正确从 CAN2 转发到 USB；动态节点 ID 分配请求已可见。GUI 节点可见性仍待工程师确认。** 范围与必验见 [CAN-BRD](../../../plan.md#can-brd-当前范围)，接口与复测见 [mavcan.md](../../mavcan.md)。

## 原问题（工程师报告：DroneCAN GUI 无消息）

旧版桥接（`eb81d45`）有三个确定缺陷：

1. **经典帧被发成 `CANFD_FRAME` 且丢失扩展 ID 标志**：`frame_to_mavlink` 对经典帧构造 `CANFD_FRAME_DATA`，未在 `id` 上置 bit31；pydronecan 收到后按 `CAN_FRAME` 过滤，因此无消息。
2. **入站 `CAN_FRAME` 完全未处理**：`handle_message` 只接受 `CANFD_FRAME`，GUI 下发的帧全被丢弃，CAN2 出向不可用。
3. **HAL 0.5.0 在 DLC 非零时清 RTR**：`peripheral.rs` 的 `put_tx_header` 把 `rtr` 写成 `len==0 && rtr`；直接构造 RTR 帧会被当成数据帧发出。

## 修复

- `src/drivers/mavcan.rs`：接收路径 `from_hal` 正确保留 `header.rtr()`/`header.fdcan()` 与扩展 ID；`CAN_FRAME`/`CAN_FILTER_MODIFY`/`CAN_FORWARD` 入站全部处理。出向用 PAC 直接写 STM32 H753 Classic TX element（2 header 字 + 8 data 字，`TX_ELEMENT_WORDS=18`）后再置 `TXBAR`，规避 HAL 的 RTR 丢失与无界 abort；`queue_frame` 在会话锁内执行，防止关闭/reset 竞态。
- `src/core/mavcan.rs`：经典帧 ID/标志/DLC、5 s 租约、单一 MAVLink 客户端、过滤（bus=2，按 ID 第 2/3 字节，最多 32 个）、会话 epoch/年龄上限 100 ms。
- `src/comms/mavlink_usb.rs`/`usb_console.rs`：`UsbSessionEvents::restart()` 与 reset 路径调用 `crate::mavcan::stop()` 清理会话/队列/待发帧；`!mavcan` 诊断。
- `tools/check_mavcan.py` + `tools/test_check_mavcan.py`：握手、诊断、`CAN_FRAME` 校验、GetNodeInfo 请求、租约关闭/超时/重开。

## 软件检查

| 项目 | 结果 |
| --- | --- |
| Rust 核心 | 6 项 host 测试通过（含 `fdcan_mailbox_preserves_remote_dlc_and_classic_frame_format`） |
| Python 协议 | 5 项通过（经典帧经 MAVLink2 往返、CANFD/非法 ID/DLC/总线/目标拒绝、GetNodeInfo 向量） |
| fmt / Clippy / 构建 | fmt、host+release Clippy、普通 release/打包通过；raw/padded=`393944/393952 B`，static RAM=`236872 B`，空余 `220856 B` |
| 固定参考 | VimDrones `7bfe9963` / pydronecan `092d09cb`；HAL/PAC 保留纯 Rust |

source=`a2b91d35630180da`。

## 实物验证（2026-11-03，工程师确认节点接 CAN2、GUI bus 2）

经原 PX4 bootloader 上传/校验（`PX4_PORTS=/dev/ttyACM0`，progress 100%），运行 source=`a2b91d35630180da`：

1. 启动：`MAVCAN ready=true`、`MAVCAN_HW registers_ok=true can1_registers_ok=true`、`cccr=00000040`（正常模式、未 bus-off、`ecr=0`、`nbtp=02000c01`）。
2. `CAN_FORWARD bus=2` 返回 ACCEPTED，会话 `active=true owner_sys=250 owner_comp=189`（MAVCAN 客户端身份）。
3. **CAN2→USB 转发通过**：8 s 捕获 `CAN2 frames=10`、`usb_frames=10`，帧 `id=983cd500 dlc=8 data=014c0034001850d1..`，即匿名节点的 **Dynamic Node ID Allocation 请求**。这是修复前"GUI 无消息"的直接证据修复。
4. 关闭 `CAN_FORWARD bus=0` 后会话失效，后续无帧转发；租约/重开逻辑就绪（脚本未跑到超时分支，因无静态 NodeStatus 节点）。

## 待工程师确认

- **DroneCAN GUI 节点可见性未通过**：GUI 打开、连接 Rust CDC（MAVCAN、bus 2）后仍未显示节点。原因：CAN2 上目前只看到匿名 Allocation 请求（`983cd500`），GPS/ESC 尚无静态 node ID，需要 GUI 的 Dynamic Node ID Allocation server 分配 ID 后节点才会广播 NodeStatus 并显示名称。
- USB→CAN2 出向（GetNodeInfo / Allocation 响应）尚未端到端验证：`tx_received=0`，因为还没给节点分配 ID 后发起请求。GUI 分配 ID 的过程即会测试出向路径。
- 若节点期望静态 ID，需按 H-02/H-10 核对 GPS/ESC 的 node ID 配置；`!mavcan` 可在会话内核对 `rx_frames`/`usb_frames`/`tx_*` 与 `psr/ecr`。

## 记录与下一步

- 本次提交推送包含全部 MAVCAN 修复与验证工具；GUI 节点可见性与出向 GetNodeInfo 为未关闭项，随 GUI/节点 ID 确认后收口。
- GPS MAVLink（P4b-g，`GPS_RAW_INT`/`sensor_gps`）仍待本阶段关闭与 H-10 后接入，未在本轮实现。