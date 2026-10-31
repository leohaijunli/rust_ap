# DroneCAN：M1 接入与测试起点

日期：2026-10-31。范围为 **DroneCAN / UAVCAN v0、经典 CAN、29 位扩展 ID、最多 8 B 数据**。M1 包含节点服务、4 个 Vimdrones Nano 的状态/零命令、Holybro H-RTK GNSS；后续 RTCM 与非零电机细节按硬件结果再定。

## 1. 实现与参考来源

固定版本/许可见 [provenance](provenance.md)。官方 DSDL 使用 `b4653c7abc3c47cb31b16efa24ea755232774756`，pydronecan 使用 `092d09cb9e6b115e262efe4cb42f7b65e6054f26`（1.0.27）。Cyphal/UAVCAN v1 栈不能直接替代 v0。

| 候选 | 已得到的证据 | 结论 |
| --- | --- | --- |
| umi-eng/dronecan-rust | std 单元测试 8 passed；无 alloc M7 check 失败；异常探针发现四项缺陷 | 不能直接作为合格协议栈 |
| David-OConnor/dronecan | 含可选 stm32-hal2/fdcan 依赖；本轮未找到许可文件、未做构建基线 | 留参考，不作为已验证实现 |
| pydronecan | 生成并回读固定 DSDL 向量 | host 参考实现，不进板上应用 |
| dronecan_dsdlc | 已固定版本/许可，生成 C | 仅可用于 host 对拍；本轮未运行 |

umi `src/transfer.rs:53` 在无 alloc 时仍引用 `ManagedSlice::Owned`；`:33,129-131` 明示未校验 CRC；`:107-108` 对短多帧首帧可能越界。探针结果：

```text
short_start_panics=true
distinct_crcs_both_accepted=true
invalid_initial_toggle_accepted=true
service_node_zero_accepted=true
```

见 [probe-results.json](acceptance/p0/probe-results.json) 和 [dronecan-malformed-probe.txt](acceptance/p0/dronecan-malformed-probe.txt)。exit=0 仅表示诊断完成。修复无堆编译并不能消除协议缺陷；CAN-A01 仍须实现/修复有界纯 Rust v0 层并独立验收。

## 2. M1 最小消息集

| 类型 | ID | 用途 |
| --- | ---: | --- |
| `uavcan.protocol.NodeStatus` | 341 | 本节点 1 Hz，监视外部节点 |
| `uavcan.protocol.GetNodeInfo` | 1，服务 | 身份/版本发现 |
| `uavcan.protocol.param.GetSet` / `ExecuteOpcode` | 11 / 10，服务 | 共用参数注册表；保存状态如实返回 |
| `uavcan.protocol.RestartNode` | 5，服务 | 受状态限制的重启 |
| `uavcan.equipment.esc.RawCommand` | 1030 | M1 固定四槽全零 |
| `uavcan.equipment.esc.Status` | 1034 | 四个 ESC 的状态 |
| `uavcan.equipment.safety.ArmingStatus` | 1100 | M1 固定 DISARMED=0 |
| `uavcan.equipment.gnss.Fix2` / `Auxiliary` | 1063 / 1061 | H-RTK GNSS 数据 |

签名、非 TAO 最大位长、实际 TAO payload 与 CAN 帧全部保存在 [dronecan-vectors.json](acceptance/p0/dronecan-vectors.json)。RTCMStream（1062）的向量也已生成，功能归 P4b-r。旧 Fix 仅在实物固件需要时加入，避免双重使用同一测量。

### 参考向量复现

在项目根目录，先取得上述两个固定、干净的仓库：

```sh
.venv/bin/python tools/p0/reference_vectors.py \
  --pydronecan /tmp/opencode/rust-ap-p0-20261031/pydronecan \
  --dsdl /tmp/opencode/rust-ap-p0-20261031/DSDL \
  --output /tmp/opencode/dronecan-vectors-reproduced.json
```

当前为 **40 个正常参考样本 + 3 个拒绝样本**：RawCommand 长度/符号边界、TID 数值回绕、服务请求/响应、ArmingStatus 0/255/保留值、Status NaN、Fix2 协方差/RTK/最大数组、Auxiliary 和 RTCM；拒绝 CRC 错误、错误初始 toggle、缺尾帧。

这是 Python 参考生成器自身的回读，不是 Rust 对拍、会话去重或 HIL 通过。CAN-A01 另测 TID 31→0、重复/超时/乱序、源/目标节点、服务关联、帧间混入、缺帧、超长、容量耗尽与安全拒绝。ArmingStatus 的保留值 1 是可编码的字节，不是有效解锁许可。

## 3. 接收/发送边界与初始容量

HAL 使用 `embassy-stm32` FDCAN；拟先 CAN1（PD0/PD1），实际端口、电气/供电与终端由 H-03 确认。1 Mbit/s 为设备参考值；若使用 HSE 16 MHz，合法候选为 prescaler=1、TSEG1=13、TSEG2=2、SJW=2（采样点 87.5%），须通过 HAL 配置、寄存器和总线实测。AM32 的 90% 采样点是 ESC 端配置，不要求两端寄存器相同。

以下是首轮可测初值，不是已实现分配：

| 资源 | 初值 / 行为 |
| --- | --- |
| HAL 硬件消息 RAM | 0.5.0 固定每 RX FIFO 3 帧、TX 3 帧；不能拿软件队列容量冒充硬件深度 |
| 软件 RX | ESC/状态快速队列 64 帧，GNSS/服务队列 128 帧；IRQ 只取帧/时间戳/计数 |
| 重组 | 12 槽，每槽最多 512 B payload + 有界元数据；ESC/GNSS 保留槽不被服务洪泛挤占 |
| 会话 / 节点 | 64 个有界会话、16 个节点；4 个 ESC 身份槽 + 单 GNSS 源显式配置 |
| 服务响应 | 最多 8 个 pending 响应；限速，满时计数并拒绝新增，不占满周期输出 |
| 周期 TX | RawCommand 与 ArmingStatus 各一个最新值；M1 始终零/0，丢弃过期发送而不追赶积压 |
| CAN 软件 RAM | 上述结构整体预算 ≤32 KiB，实际 `size_of`/map 验证 |

512 B 槽不仅覆盖 Fix2 最大 221 B，也覆盖 GetNodeInfo/GetSet 的约 377/371 B 最大响应。TID 按源/类型/请求或响应/目的节点建会话；多帧使用 DSDL 签名参与 CRC。初值：会话重同步 timeout 2 s，GNSS 整笔重组 100 ms、服务整笔 1 s、帧间 100 ms；兼容性须与固定独立实现对拍。

FDCAN IRQ 起始延迟目标 ≤25 us，单次取帧处理目标 ≤20 us；协议层每次 poll 有界并优先排空 ESC 队列。软件队列溢出丢弃受影响传输、记录计数；不得把半笔传输发布为有效数据。HAL `src/can/fd/peripheral.rs:170-195` 的 abort 有忙循环，CAN-A02 须测量或改为有界平台适配。

M1 新鲜度初值：ESC Status 年龄 >100 ms 为 stale，NodeStatus >3 s 为 offline，GNSS Fix2 >600 ms 为 stale。标志更新目标 ≤10 ms；这些是数据显示/记录阈值，非飞行故障动作。总线错误/重连不改变零输出约束。

## 4. 带宽预算是什么意思

1 Mbit/s 的 CAN 总线每秒可传约 1,000,000 bit。预算按扩展经典帧计算，包含仲裁/CRC/ACK/帧间隔、DroneCAN tail、多帧 transfer CRC 和保守位填充：

```text
单帧位数上界 = 67 + 8 × DLC + floor((53 + 8 × DLC) / 4)
单帧 payload ≤7 B；多帧先加 2 B transfer CRC，再按每帧 7 B 分片。
```

满 8 B 帧按 160 bit 计；错误帧/重发不在这个公式内，另外预留 10 个百分点。

| M1 流量假设 | 帧数/transfer | bit/s |
| --- | ---: | ---: |
| 四槽 RawCommand，400 Hz，全零 | 1 | 64000 |
| ArmingStatus，10 Hz，DISARMED | 1 | 1000 |
| 4 ESC × Status 50 Hz | 3 | 86000 |
| Fix2 最大数组，5 Hz | 32 | 25550 |
| Auxiliary，1 Hz | 3 | 450 |
| NodeStatus 总计 8 Hz | 1 | 1280 |
| 服务流量，最多 200 满帧/s | 按帧限速 | 32000 |
| 未使用 GNSS 广播余量 | 按实测核对 | 20000 |
| 合计 | | **230280（23.028%）** |
| 加错误/重试预留 100000 bit/s | | **33.028%** |

因此“含重试余量 33.0%”指预计带宽额度，还有约 67% 未分配；不表示已测到 10% 重传，也不代表一定满足延迟。NodeStatus 的 8 Hz 含一个额外 GNSS 节点余量与 GUI 节点，不增加第二台 rover 的需求。

初始持续压力测试包络为 **60% 总线负载**；加到 80%/拥塞时验证计数、有界丢弃和采样稳定。外部节点流量无法靠本节点限速控制，必须抓包测实际优先级、突发和错误。400 Hz 零命令的正常发送间隔目标 ≤5 ms、生成到发出年龄 ≤5 ms，使用 TX 完成/外部抓包时间测量。

复算：`python3 tools/p0/budgets.py --profile m1`，结果见 [budget-m1.json](acceptance/p0/budget-m1.json)。历史完整探索为 39.428%（含 RTCM 额度），8 ESC 连续/稀疏槽探索为 58.892%/78.092%；这些是已保存的敏感性计算，不是本轮实现范围。

### 用户追加的容量询问：3 个 float32 × 200 Hz

若另发一条固定长度、仅含三个 float32 的 DroneCAN 消息：payload=12 B，加 2 B transfer CRC 后恰好两个满帧，`2 × 160 × 200 = 64000 bit/s`，增加 **6.4 个百分点**；M1 总预算变为 **39.428%**，仍在 60% 初始测试包络内。若增加时间戳/其他字段，按实际 DSDL 重新计算。此处为容量评估，数据含义/消息类型尚未指定，未计入 M1 默认配置。

同时写 ULog 时，三个 float32 + 8 B 时间戳 + 5 B DATA 开销为 **5000 B/s**。若经 MAVLink 以 200 Hz 送入 QGC，须按所选消息和 TELEM1 波特率另算，不能从 CAN 余量直接推出串口余量。

## 5. 实物配置回填

| 电机编号 | Quad X 实际位置 | 源 node ID | ESC_INDEX（拟 0–3） | 旋向/极数 | AM32 target/版本/参数快照 |
| --- | --- | --- | --- | --- | --- |
| M1 | 待核对 | 待填 | 0，待验证 | 待填 | 待填 |
| M2 | 待核对 | 待填 | 1，待验证 | 待填 | 待填 |
| M3 | 待核对 | 待填 | 2，待验证 | 待填 | 待填 |
| M4 | 待核对 | 待填 | 3，待验证 | 待填 | 待填 |

node ID 与数组下标独立，检测重复/越界/身份不符。RawCommand 是 `int14[<=20]`，协议范围 -8192..8191；M1 全零。优先 GUI 预设静态 node ID；若实物只能 DNA，再落实 CAN-DNA 前置。

固定 AM32 参考：`VIMDRONES_NANO_L431_CAN`，默认 `CAN_NODE=0`、`ESC_INDEX=0`、`TELEM_RATE=25`、`REQUIRE_ARMING=1`、`REQUIRE_ZERO_THROTTLE=1`。预算 50 Hz 必须配置并实测。`Src/DroneCAN/DroneCAN.c:683-768` 仅 status=255 置 armed，含零油门/滤波门控；`Src/DroneCAN/DroneCAN.c:1339-1363` 在最后 RawCommand 后 >250 ms 清零，不是机械停转时间，也未证明 ArmingStatus 有独立 timeout。后续非零测试沿用单一输出仲裁，细节归 P5a/P4b-b。

Status 温度为 K，RPM=`e_rpm*200/MOTOR_POLES`（机械 RPM），error_count 是 desync 次数、power_rating_pct 在该实现为 0；保留 NaN/未知语义。来源与厂商终端电阻说明见 [硬件台账](fmuv6x-hardware.md)。

H-RTK 回填：SKU12059/序列号、固件、node ID/唯一身份、Fix2/Auxiliary 速率、额外广播、供电/拓扑/终端。保留经纬度 1e-8 度、两种高度 mm、NED 速度 m/s、协方差、GNSS 时间标准与本地接收时间；未知值/无定位/stale 明确标记。MAVLink `GPS_RAW_INT` 使用真实原始 GNSS；收到 GNSS 不代表已完成导航融合。RTCM/RTK 和模块内其他传感器另按后续阶段处理。
