# H-RTK CAN Rover：接入、差分转发与地面站验证步骤

日期：2026-11-09。工程师已确认：固定基站连接地面站，单个 **H-RTK ZED-F9P Rover（IP66、RM3100）** 连接飞控 CAN。CAN 4-pin 版本对应 Holybro SKU12059；开始前核对实物标签与固件版本，不能把同名 UART 版本当作 CAN 版本。

**本文是操作与开发计划，不是已通过的验收记录。本轮只编写文档及另行要求的 crate 改名，不执行 RTK 开发、设参、基站配置、刷写或实体测试。**

## 1. 范围、现状与分工

- 工程师：接线/供电、基站与 QGC 配置、室外静置、授权的断线/恢复；软件侧：实现、host 检查、协议采集/解析及记录。
- 全程 disarmed、输出关闭，不运行电机测试、任务上传或飞行模式切换。保留纯 Rust/Embassy、原 PX4 bootloader、`0x08020000` 入口、参数和旧故障/日志。
- 依赖以 [plan.md](../plan.md) 为准：GNSS-03 依赖 P4b-a、PL-a、H-10；P4b-r 正式 RTK 实物验收依赖 M1、P2、P4b-g、H-10。经单独授权的前置数据链核对不等于放行这些阶段。
- 本流程只做固定基站＋单 Rover 定位；不是机体双天线 moving-baseline 航向。RM3100 接入另做一步，不作为差分转发的前置；不扩展模块内 IMU/气压计。

```text
基站 USB/串口 ──RTCM──> 地面站 QGC
                           │ GPS_RTCM_DATA（MAVLink #233）
                           ▼
                     飞控 Rust 接收/重组【待开发】
                           │ RTCMStream（DroneCAN #1062）
                           ▼ CAN1
                     ZED-F9P Rover：内部 RTK 解算
                           │ Fix2（1063）/Auxiliary（1061）
                           ▼
              sensor_gps ──> GNSS/EKF、GPS_RAW_INT、SD 日志【已有】

模块 RM3100 ──CAN 磁场消息──> 磁源选择/校准/EKF【接收入口待开发】
```

| 项目 | 当前状态 |
| --- | --- |
| CAN1、DroneCAN v0、1 Mbit/s；Fix2/Auxiliary 接收 | 已实现，实际 Rover 接收仍须验证 |
| RTK Float/Fixed 标志、MSL/椭球高度、NED 速度、日志/遥测和 EKF 输入 | 已实现；标志解码不等于实物 RTK 或融合通过 |
| 飞控侧动态节点 ID 分配器 | 未实现，优先配置稳定且不冲突的静态 ID |
| 地面站 RTCM 到 CAN Rover 的转发及专用诊断 | 未实现；当前连接基站并不会自动打通此链路 |
| 外接 RM3100 磁数据接收 | 未实现；现有多源选择/校准框架可复用 |

当前 crate 为 `v6x-rust-ap`，Rust 导入名 `v6x_rust_ap`，新产物为 `v6x-rust-ap.{bin,px4,map,build.json}`（ELF 不带扩展名）。历史 `v6x-rust-led` 记录保留。此前 `d21b326c3a753d15` 的[高度数据链结果](acceptance/ekf2/20261109-altitude-output.md)只作基线，不作为 RTK 成功证据。

## 2. S0：准备与记录输入

**工程师操作：**
1. 记录基站型号/固件、Rover SKU/固件、QGC 版本、飞控 `git/source`；基站型号目前尚未提供，不能默认它与 Rover 相同。
2. 区分基站串口和飞控 MAVLink 口。第一轮用 **USB 连接飞控**；实际外场若走 TELEM1/数传电台，提供后另验该链路，不用 USB 结果代替。
3. Rover 接 **CAN1**，核对厂商线序、供电、1 Mbit/s 与总线终端。CAN2 当前仅用于 MAVCAN 透传，不能替代 CAN1 GNSS 接收；不要随意把 CAN1/CAN2 短接。
4. 基站与 Rover 天线置于开阔天空，基站牢固固定；记录天线位置、基站坐标来源和 Rover 天线相对 IMU 的杆臂。供电/接线变更前断电，操作由工程师完成。
5. 约定本次观测窗口、固定解等待预算和用途：本流程验数据链，厘米级绝对位置/海拔精度须另有可靠参考与事先确定的判据。

**通过条件：**型号、端口、链路、坐标/高度基准和授权范围明确。型号不兼容或端口身份不明确时，停止后续配置。

## 3. S1：先验证 Rover 普通定位，不依赖差分

**工程师操作与软件核对：**
1. 暂不启用地面站差分发送。QGC 只连接飞控，检查真实心跳、disarmed、固件身份。
2. 在 QGC **Analyze Tools → MAVLink Console** 输入 `platform`、`dronecan`、`ekf2`、`logger status`（不带 `!`）。原始 USB 文本控制对应 `!platform` 等；不要让第二个串口程序与 QGC 同时独占同一 COM 口。
3. `dronecan` 应看到 Rover 的 `CAN_NODE ... online=true`，以及 `GNSS ... fresh=true`、递增时间和真实位置。记录 `rx_bus_errors/rx_codec_errors/tx_timeouts` 的增量；总 `rx_frames>0` 只能证明有 CAN 流量。
4. 若模块尚未分配节点 ID，先用支持 DroneCAN 动态分配的工具给它临时 ID，才能访问参数服务；已有有效 ID 则跳过分配。随后读取其实际节点 ID 参数，配置不冲突的静态 ID 并按授权保存/重启验证。FMU 的 `MAV_SYS_ID` 不是 Rover 节点 ID，QGC 的 FMU 参数列表也不等于模块参数列表。
5. 没有配置工具且模块必须动态分配时，另行实现最小分配器，不能假定当前 Rust 固件会分配。现有 GUI/CAN2 透传如用于临时模块配置，须单独授权并最终恢复 CAN1，不能当作 CAN1 原生接收已通过。
6. 在 QGC **Analyze Tools → MAVLink Inspector** 选择飞控 system/component **1/1** 的 `GPS_RAW_INT`：确认 `fix_type` 至少 3、卫星数/时间/经纬度有效，频率符合 `MAV_RATE_GPS`（现值 5 Hz，不代表 Rover 内部采样率）。
7. 核对 `h_acc/v_acc` 为毫米精度信息；`eph/epv` 是 DOP×100，不能当作米。`alt` 为 MSL 毫米高度，`alt_ellipsoid` 为椭球毫米高度；`0` 精度或诊断 `-1` 可能是未知，不是零误差。
8. 软件侧确认真实 Fix2 的协方差格式：当前仅长度 1/6 可形成有效精度。其他格式需核对固定参考与模块输出，不能填零让 EKF 接受。若只发送旧 Fix 而无 Fix2，按协议兼容问题处理。

**只读参数基线（来自此前 `02.ulg`，执行时重新核对，勿盲改）：**

| 参数 | 已记录值 | 含义/限制 |
| --- | ---: | --- |
| `EKF2_GPS_CTRL` | 7 | 水平位置、垂直位置、速度；双天线航向位 8 未实现 |
| `EKF2_HGT_REF` | 1 | GNSS 高度参考，必须有真实可融合高度；缺失时可回退 |
| `EKF2_REQ_FIX` | 3 | 先允许普通 3D；不要为验证 RTK 强制改成只接受 6 |
| `EKF2_REQ_GPS_H` | 10 s | 健康保持时间；有输入不代表立即开始融合 |
| `EKF2_GPS_P_NOISE` | 0.5 m | 位置噪声下限；RTK Fixed 不代表 EKF 自动采用厘米级权重 |
| `EKF2_GPS_POS_X/Y/Z`、`EKF2_GPS_DELAY` | 现场核对 | 按实测安装/延迟设置，变更可能重初始化，须先备份并另获授权 |

**通过条件：**Rover 身份/节点确定，Fix2 新鲜、普通定位与有效精度进入 `sensor_gps/GPS_RAW_INT`；质量门通过后在 GNSS aid 日志确认融合。`ekf2` 的 `Filter/Running` 可能仅靠 IMU/baro/mag，不能单独证明 GNSS 已融合。

**失败分流：**无节点先查 CAN/ID；有节点无 Fix2 查模块消息；有 Fix2 无融合查格式、精度、时间和质量拒绝原因，不放宽门限或改成伪有效。

## 4. S2：验证地面站识别基站与 Survey-In

界面以 **QGC 5.0.3 / Stable_V5.0** 为参考；其他版本先记录差异，不套用 master 新版菜单。本节自动配置以 QGC 支持的基站为前提，未知型号先核对兼容性。

**工程师操作：**
1. 关闭占用基站串口的 u-center/厂商工具。QGC 的 **Application Settings → General → AutoConnect** 启用 **RTK GPS**；不要把基站误配成仅提供地面站位置的 **NMEA GPS Device**。
2. 在 **General → RTK GPS** 选择一种基站位置来源：
   - **Perform Survey-In**：记录 `Survey-in accuracy` 与 `Minimum observation duration`。初次功能验证可采用现场确认的默认值；需要调整时先约定数值，不把 Survey-In 内部精度当作外部绝对精度证明。
   - **Use Specified Base Position**：仅填写可靠、未移动的天线坐标及真实精度。页面 **Base Position Alt (WGS84)** 是 WGS84 椭球高，不直接填 AMSL 海拔；不知道基准时不要填零或猜测。
3. 接入基站，确认 QGC 的基站状态为 connected，卫星数有效；Survey-In 期间 duration 增长、accuracy 更新。
4. Survey-In 完成须同时满足时间与精度条件，状态变为 **valid=true、active=false**。固定坐标模式按其有效状态核对，不要求重新跑 Survey-In。
5. 仅在基站天线位置确实不变时，用 **Save Current Base Position** 保存并复用；搬动天线后旧坐标失效，重新测定。
6. 保持飞控在线；QGC 5.0.3 将差分送往每个在线 vehicle 的 **primary link**。记录实际主链路，不假设 USB/TELEM1 同时都会收到。

**通过条件：**QGC 能识别正确基站，基站位置有效并具备 RTCM 输出条件。**基站 valid 只证明基站就绪，不证明 Rover Fixed，也不证明 RTCM 已到飞控。**

**失败分流：**无 connected 查 USB 身份/串口占用/接收机支持；Survey-In 不结束查天空与门限，不通过调大“宣称精度”或旧坐标绕过。QGC 已有效但 Rover 无变化时进入 S3/S4，不改磁场参数。

## 5. S3：开发最小 RTCM 转发及可验证诊断

**软件工作；当前未实现，实施须另行授权：**
1. 在现有 `mavlink_core/router` 接收 `GPS_RTCM_DATA`，复用现有 MAVLink CRC/解析和链路生命周期，不新增另一套串口拥有者或 NTRIP 客户端。
2. 按固定 MAVLink 定义处理长度、分片标志、fragment ID 与 5-bit sequence；保证缓存/时限有界，覆盖重复、缺片、乱序、新序列、31→0 回绕及链接断开。不同链路/来源不混片，切换时清理旧缓存；UART7 无物理断开通知时按新鲜度超时清理，不能宣称立即识别拔线。
3. 用真实 QGC 5.0.3 包作为兼容参考，核对 180/360/540/720 B 边界。该版本对 180 的整数倍不补零长度尾片；不要仅按“收到短尾片”判断完成。超出片组容量也须显式处理；不能用超时拼接、截断或放松 CRC 隐藏兼容问题。
4. 转为固定 DroneCAN `RTCMStream`（1062、RTCM3 协议），复用 CAN1 的 DSDL/CRC/transfer-ID/发送队列；根据固定类型容量分包，不改 RTCM 字节内容。若采用 RTCM3 字节流恢复边界，长度与 CRC24Q 检查同样有界。
5. 实测基站字节率后确定队列/发送预算。队列满、发送超时、陈旧改正明确丢弃并计数，不能阻塞 IMU/EKF、无限重试或重播上次会话的数据。
6. 增加专用诊断，至少区分：MAVLink 接收帧/有效字节、完整片组或 RTCM 帧、坏长度/缺片/超时/重复、CAN RTCM 发送完成/丢弃/失败、最近接收年龄和来源链路。总 CAN `tx_frames` 不足以证明发送了 RTCM。
7. 诊断输出复用现有 console/必要日志。可新增 `rtcm status`（原始 USB 对应 `!rtcm`），但**这是拟定命令，当前固件不能执行**；命名与字段在实施时确定并同步本操作单。

**必要软件检查：**
- 正常未分片/多片、上述边界与关键失败用例；用同一输入核对 QGC 出站有效字节和 CAN 解包后的字节一致。
- 固定参考核对 DSDL 编码/CRC与分包，断开/重连不继承旧片组；模拟队列满和发送失败不阻塞。
- 相关 fmt/Clippy、host 用例，普通 H753 release/打包/分区与入口帧检查。未改 EKF 数学则复用已有黄金结果，不重新启动整套数学对拍或长压测。
- 经单独授权再用原 bootloader 回装，核对 `git/source`、disarmed、参数/故障保持；不因软件通过直接放行实物阶段。

构建入口仍是 `tools/build_firmware.ps1` / `bash tools/firmware.sh build`；产物改名不改变上传协议或 USB 身份。不要照搬官方 PX4 的 `UAVCAN_ENABLE` 或 ArduPilot 的 `GPS_TYPE` 到本项目的 FMU 参数。

## 6. S4：证明地面站出站改正到达飞控和 CAN

**仅在 S3 已完成并授权回装后执行；可并入 S5 的同一次采集：**
1. 重开 QGC，确认飞控身份/主链路与基站有效，记录时间 `T0` 和改正专用计数器基值。
2. 让基站持续输出 RTCM，观察完整接收字节、CAN RTCM 发送完成计数增加，最近接收年龄保持新鲜；记录来源链路、字节率和错误增量。
3. **MAVLink Inspector 只显示 QGC 收到的 traffic**：看不到 `GPS_RTCM_DATA` 不等于没发送；看到某个回程字段也不能替代出站证明。优先用飞控的改正专用接收计数证明“地面站已发、飞控已收”。
4. 需要逐字节定位时，用既有可用的链路抓包/只读诊断记录 QGC→飞控的 #233，并与 CAN1062 解包记录对齐。不要让新程序独占 QGC 正使用的 USB COM 口；不默认 `.tlog` 一定包含出站帧。
5. CAN 发送完成只能证明总线发送，不能单独证明 Rover 消费了改正；最终结合 S5 Fixed 状态和 S6 受控中断/恢复确认。

**通过条件：**来源正确、新鲜改正持续进入飞控并按原字节送到 CAN1；完整正常窗口内无片组/队列/CAN 错误增量。若有失败保留原始包，先定位失败层，不直接调低 GNSS 质量门。

## 7. S5：地面站、Rover、EKF 与日志的闭环核对

**工程师操作与软件核对：**
1. 基站保持固定且有效，Rover 室外静置。等待真实 `fix_type` 到 6，记录首次 3D/Float/Fixed 时间；不强制每次都经过 5，不设统一的“必在 N 秒固定”保证。
2. QGC 的基站状态与飞控 system/component 1/1 的 `GPS_RAW_INT` 分别记录；确认 `fix_type=6` 是 Rover 报告的新鲜解，不是基站图标、地面站位置图标或旧缓存。
3. Fixed 后取**一次 60 s 正常窗口**，同时记录改正诊断、GPS_RAW_INT（状态/坐标/精度/频率）、EKF 状态和授权启动的 SD 日志；后续异常测试续用该会话，不重复开多份正常日志。
4. QGC General 的 **Data Persistence** 不禁用保存，**Telemetry Logs from Vehicle** 启用 **Save logs even if vehicle was not armed**；记下保存目录。`.tlog` 用于遥测证据，不替代基站配置记录或 SD 数据。
5. 解析 `sensor_gps` 与 `estimator_aid_src_gnss_pos/vel/hgt`，区分接收、质量通过和实际融合；抽样 `fused` 布尔不当作逐量测成功率。核对模块 MSL/椭球高度、时间、杆臂补偿与复位计数，不把未知精度视作厘米精度。
6. Inspector 核对 `ALTITUDE`：有效全球原点时 AMSL 来源为 `-z+ref_alt`；monotonic 仍是气压高度，可以是负数。局部/relative 与全球高度不同，时间/滤波延迟对齐后再比较原始 GNSS 海拔。
7. **水平/全球导航资格护栏仍关闭**，LOCAL_POSITION_NED/GLOBAL_POSITION_INT 或地图轨迹缺失不是 RTK 转发失败，不为显示地图临时解除护栏。基站绝对坐标误差会传递到 Rover；Fixed 不等于绝对海拔厘米级验收。
8. 核对 Running/Filter、fault/numerics=None、更新预算 2000 µs 内、deadline_misses/发布 missed 无新增、积分误差 0 ppm；SD 可解析，dropout 按已有 H-04 窗口判定，不由短窗口重做 H-04 或 M1。

**通过条件：**该窗口中新鲜 Rover Fixed 持续、改正链路无错误增量、真实 GNSS aid 融合可证且无实时性/输出回归；同一证据能对齐地面站、CAN 接收与 SD。只收到了 Fixed 但无法证明融合时，分别记录定位链路通过/融合待查。

## 8. S6：改正中断、Rover 断线与恢复

每种失败路径只做一次。断线、重连和日志动作须在授权范围内；暂停方式按当前 QGC/基站工具实际能力确定，**基站天线不得移动**。

| 动作 | 地面站观察 | 飞控/Rover 必须核对 |
| --- | --- | --- |
| 停止基站 RTCM 或断开基站 USB，飞控/Rover 保持在线 | 基站 disconnected 或改正停止，vehicle 心跳继续 | RTCM 接收停止、age 增长、旧缓存不重播；Rover Fixed 可能按内部策略延迟降级，记录真实 6→5/3 时刻，不要求 1 s 内降级，也不由 FMU伪造 fix_type |
| 恢复基站改正 | 基站重新有效，主链路不变 | 新会话改正恢复；Rover重新固定、GNSS aid恢复，无旧片组混入，无自动解锁 |
| 断开 Rover CAN/供电，飞控与基站保持在线 | 飞控在线，GPS 状态失效 | 约 1 s 数据陈旧后 GPS_RAW_INT 转 NO_GPS；GNSS aid 按现有停止/宽限期撤销，不能继续融合缓存。ALTITUDE 可继续由 baro 有效支持，不要求全字段立刻 NaN |
| 恢复 Rover | 节点重新在线 | ID、数据年龄、质量门与新输入恢复；需要动态分配时重新获得有效 ID。基站未动则不因 Rover 重启而重做 Survey-In，不把旧节点状态当作已恢复 |
| 最后断开/重连地面站主链路一次 | 基站接收与 vehicle 链路状态分别记录 | 断开时清理 RTK 会话；重连后只用新改正。若正式使用 TELEM1/数传，此项在该实物链路再做一次 |

改正停止到失去 Fixed 的时间由接收机固件/缓存决定，测试前约定观察预算；预算内未降级时报告保持策略和改正 age，不能仅凭仍显示 6 就判改正仍新鲜。改正中断与 GNSS 接收中断是不同故障，不混为一个立即失效门。

## 9. S7：RM3100 独立接入（可后做）

1. 核实 Rover 实际发布的 DroneCAN 磁场类型/单位/频率/方向与 sensor ID，再开发接收桥；不假定“GPS 已收到”就代表外接磁已接入。
2. 将 CAN 磁场送现有磁源选择/校准/日志通路，稳定绑定真实 device ID；在 `mag`（原始 USB `!mag`）看到外接源、健康与选中状态，区分板载 BMM150。
3. 按实物安装核对三轴/旋转并进行授权校准；当前固件的 `CAL_MAGn_*` 只应用参数，不等于 QGC 自动校准拟合已经实现。
4. 一次转动/静止窗口核对选中外接源及磁融合，再撤去该源一次确认失效与回退；不接入其额外 IMU/气压计，不由磁数据验收放行航向绝对精度或飞行资格。

## 10. 记录、收尾与放行

**收尾：**停止本次授权启动的 logger，确认 Disabled、记录错误/丢失；断开诊断会话，保存 QGC 日志、SD日志、基站配置和必要原始包。保持 disarmed/输出关闭，不清故障、不删除旧日志，不擅自保存新的参数或刷写模块。

一份记录覆盖正常与异常，模板如下；没做的项填写“未执行”，不要预勾通过：

```text
日期/工程师授权范围：
基站型号/固件/COM；坐标来源、WGS84高度、精度、Survey-In配置与结果：
Rover SKU/固件/node ID/CAN口/位率；QGC版本：
飞控crate/git/source；USB或TELEM1主链路；参数基值/实际变更：
正常窗口起止、RTCM RX/TX字节与错误增量、GPS 3/5/6时间线：
sensor_gps/GNSS aid/ALTITUDE核对；EKF时序/SD结果：
基站中断/恢复、Rover中断/恢复、主链路重连的时刻与结果：
QGC/SD/原始包路径与SHA-256；未执行项/问题/下一步：
```

| 门 | 当前状态 | 放行依据 |
| --- | --- | --- |
| S0～S2 基础接收/地面站准备 | 未实测 | 设备与坐标明确、Rover新鲜定位、QGC基站有效 |
| S3 软件 | 未实施 | 相关 host/工具/普通构建检查，原始字节与失败门正确 |
| S4～S6 RTK 数据链 | 未实测 | 同次地面站/CAN/SD闭环、Fixed与中断恢复、实时性保持 |
| S7 外接磁 | 未实施/未实测 | 独立来源/方向/校准、融合与失效回退 |
| M1/P4b-r/完整 E7/非零输出 | 不因此放行 | 各自原阶段依赖及必验项，见 plan.md |

## 11. 参考与实施位置

- [Holybro 规格与 CAN/UART 型号区分](https://docs.holybro.com/gps-and-rtk-system/zed-f9p-h-rtk-series/specification)
- [QGC Stable_V5.0：General / RTK GPS / 日志设置](https://docs.qgroundcontrol.com/Stable_V5.0/en/qgc-user-guide/settings_view/general.html)
- [QGC Stable_V5.0：MAVLink Inspector 的接收视角](https://docs.qgroundcontrol.com/Stable_V5.0/en/qgc-user-guide/analyze_view/mavlink_inspector.html)
- [QGC v5.0.3 RTCMMavlink 发包与 primary link](https://github.com/mavlink/qgroundcontrol/blob/v5.0.3/src/GPS/RTCMMavlink.cc)
- 本项目：[MAVLink 接收](../src/comms/mavlink_core.rs)、[CAN/诊断](../src/drivers/dronecan_can.rs)、[DSDL](../src/core/dronecan/dsdl.rs)、[GNSS 转换](../src/core/gps.rs)、[参数实际映射](../src/core/estimator/runtime_params.rs)、[状态索引](../TODO.md)。
