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
- 3 s 复核预定判据：相同近 1000 帧/s 输入下 USB/RX 增量比 ≥95%，转发持续至窗口末段、末次队列不满 64，RX/协议拒绝无新增错误；原 100 ms 帧寿命保持。另核对 NodeStatus/GUI 身份与启动的 disarmed、EKF 2000 µs 预算/无漏期，不把流量恢复直接写成 GUI/RTK 验收。
- 实际 GUI 节点显示/身份由工程师确认，不自动放行 RTK、E7/M1、非零输出或此前未验物理项。

## 回装与吞吐复核：恢复，但完整性未收口

- 修复提交 `f24b138` 已推送；从该提交构建普通包 source=`620736ef4e52c4d2`，embedded git 与提交一致。构建/分区/向量/入口帧通过：raw/padded=1227360/1227360 B，静态 RAM 374680 B（余 83048 B）、入口帧最大 11528 B、oversized=0；比旧包 Flash +160 B、RAM/入口帧不变。
- 109 项工作树源码 SHA-256 与报告、4 个产物与报告一致；ELF/bin/map/px4/报告/构建输出保存在 `target/mavcan-20261109/620736ef4e52c4d2/`。px4 SHA-256=`0c0f3293c49077edde54f5d86021b6a76b5b47b9ba79521c9138818a8f7ee7f6`。
- 回装前旧包 Logger Disabled、buzzer off/CCR=0；RGB 此时已 ready=false/Nack（不是新修复才出现），未扩大排查。原 board 53/protocol v5 bootloader 应用区上传/CRC 100%（25 s），不改 bootloader 或 IO；`pre-flash-readonly.json` / `upload.txt` 保存证据。
- 新包同样 3.016 s 捕获：RX 增量 2996、USB 增量 2896（96.66%，计数快照非严格同窗，不作为端到端丢帧率）、软件丢弃/拒绝/RX 错误增量 0，队列末值 1；关闭 ACK 成功、active=false、队列归零。实际 USB 捕获 2869 帧，约 951.26 帧/s，转发持续至窗口末段，满足此前吞吐判据。
- NodeStatus 节点 125；捕获 type 1003×2088、1028×246、1029×245、1001×77、1061×45、1063×150、20003×15、341×3。记录：`620736ef4e52c4d2/20261109T220932.792400Z/capture.json`。这些是 CAN 帧数，不是传感器更新次数。
- 启动只读：source/git/default、3 条 disarmed 心跳、VTOR/FPU/时钟/备份域保持；EKF Running/Filter、fault/numerics=None、max_update=1472 µs/2000 µs、漏期/发布 missed=0，栈余 31560 B/guard=true；收尾 CPU 32.3%（不是转发活动窗口 CPU 测量）。Logger Disabled、蜂鸣 tests=0/CCR=0。RGB Nack、PX4IO Timeout 保持待验；USB tx_errors 为累计 1873，不清零或当成本次转发错误增量。`post-flash-readonly.json` 保存原始结果。
- 工程师确认 GUI 已看见节点，但随后出现持续 CRC/toggle 异常。日志确认节点 125 身份为 `org.ardupilot.HolybroG4_GPS`、软件 4.8/vcs `6284e70e`、硬件 4.29；这证明身份/请求返回已工作，不代表数据完整性通过。GUI 日志节选：`target/mavcan-20261109/gui-spin-errors-20261109.txt`。
- 离线重组同一已保存捕获：RawIMU 297 个完成传输中有 1 个 toggle 异常（payload 40 B，其余 296 个为 47 B），另 1 个传输被后续 start 中断；初始 4 个 orphan 帧按加入实时流边界单独记录。Fix2/Auxiliary 各 15 个完整传输按固定签名 CRC 全部通过，不能覆盖后续 GUI 的错误；RawIMU 本次只检查尾标/长度，不宣称其 CRC 全过。`transfer-check.json` 保存结果。
- 既有 `MAVCAN_HW rxf0s=02000000/02020200` 的 bit25 是 H753 `Rx FIFO Message Lost`，确有硬件 FIFO 丢帧标志。锁定 HAL 的非缓冲 RX IRQ 仅唤醒任务，FIFO 只有 3 槽，而 CAN 任务与 EKF 同为 P4；这是接收不及时的高可信风险。软件 `rx_dropped=0` 不覆盖这层损失，优先级/接收路径变更尚未执行，须另行授权并复核浮点抢占/资源。

## 后续 CAN1 原生 GPS / 位置输出只读核对

- 工程师自行把 GPS 移到 CAN1，并澄清已有有效 `GPS_RAW_INT`，缺的是滤波器位置类消息；另行授权一次约 3 s 只读核对，不设参/分配节点/刷写或启停日志。
- source 仍为 `620736ef4e52c4d2`；节点 125 在线、新鲜 GNSS、fix_type=4（DGPS）/29 颗卫星。`MAV_RATE_GPS=5`、`MAV_RATE_EST=10`，#24/#33 配置间隔 200000/100000 µs。
- 3.016 s：GPS_RAW_INT 15 条（约 4.97 Hz，全部 fix_type=4），ATTITUDE/QUATERNION/ALTITUDE/ESTIMATOR_STATUS 各 29 条（约 9.6 Hz），LOCAL_POSITION_NED/GLOBAL_POSITION_INT 各 0；EKF Running/Filter、fault/numerics=None、峰值 1870 µs/2000 µs、无漏期。
- 原因是 `src/core/estimator/product.rs` 明确把 `xy_valid/v_xy_valid=false`、`global_position=None`：当前 E7 水平/全球发布护栏仍在，流间隔有效也不会产生这些位置消息。原始 GPS 到达/Filter Running 不独立证明 GNSS 已融合；未解除发布或输出护栏。
- 同窗 CAN1 CRC 错误 54→55，队列丢弃为 0；大部分 codec/rejected 增量包含不支持的传感器类型，不将其全部写成 CRC 损坏。历史累计总线/超时错误保留，不冒充本窗口错误。证据：`target/mavcan-20261109/620736ef4e52c4d2/can1-20261109T225231.315427Z/capture.json`。

## CRC 接收时序修复：工程师选择先修 CRC

- 工程师明确授权 CAN 接收优先级/丢帧诊断、普通回装及有界完整性/浮点抢占/资源复核；未授权放开水平/全球发布，本轮保持该门。
- `TIM6_DAC` CAN 执行器 P4→P3，仍低于 FDCAN P2，能及时抢占 EKF P4 排空 3 槽 FIFO；不改帧数据、CRC、过滤、队列或 100 ms 寿命，也不分配节点/设参。
- `CAN_RX_FIFO` 读回实际深度/占用/Message Lost，CAN1/2 共用同一诊断实现；`CAN_SCHED` 记录 CAN IRQ 进入时 TIM5 活动的实际抢占次数，不把仅提高优先级等同于已发生抢占。
- fmt/H753 Clippy 与一次局部审视通过；锁定 Cortex-M NVIC API、非缓冲 HAL/优先级和原 FPU 保存模式不改，纯核心/协议测试结果复用。普通包/回装/实测尚待补录。
- 预定有界复核：真实 CAN1 输入下 CRC 增量 0、FIFO lost=false，CAN/EKF 优先级读回 48/64且实际抢占增量 >0；FPU ASPEN=true/LSPEN=false、Filter Running/numerics=None、漏期 0、峰值 ≤2000 µs、栈余 ≥16 KiB/guard=true、CPU <50%。保持日志停止、蜂鸣 off/CCR=0，不写位置/RTK或飞行放行结论。
