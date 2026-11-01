# M1：首个集成目标与验收单

日期：2026-11-01。**A3 已选定；P4b-a、PL-a、P2、P4 软件门通过，本板集中 HIL 尚未开始。** M1 是一次真实设备数据链验证，随后依据硬件结果细化其他功能。

## 1. 用户确定的目标

1. `embassy-stm32` 驱动唯一 **SPI2 ICM-42688-P**，验证 8 kHz ODR、8 包 FIFO 批处理和 1 kHz 日志抽取。
2. 纯 Rust AHRS，优先固定 Cybflight Mahony 的 gyro/accel 模式；姿态方向/新鲜度可验证，yaw 明确为相对值。
3. DroneCAN 节点服务、4 个 Vimdrones Nano 的状态和全零命令、H-RTK Rover DroneCAN GNSS。
4. 纯 Rust **ULog 板载 SD 记录**，包含 IMU/AHRS、CAN/ESC/GNSS、参数与系统事件。
5. **USB CDC 与 TELEM1** 提供对应 MAVLink 消息。
6. **上游 QGroundControl 实际连接、Inspector 消息检查、参数页读取/设置**；标准 PARAM_REQUEST_LIST/READ/SET/VALUE，全表/索引/范围/回显/双链路一致性。

全程使用原 PX4 bootloader/应用边界，先完成 [QGC 官方 PX4 往返恢复](recovery.md)。M1 保持 disarmed、四槽零 RawCommand、ArmingStatus=0。

## 2. 实施与证据顺序

| 顺序 | 工作 | 对应 TODO |
| --- | --- | --- |
| 1 | P0 软件资料收口、A3 选择、已知良好回归包 | P0-01～10、D-A3 |
| 2 | QGC 恢复、Embassy 时基/USB/诊断与周期测量 | P1-01～04 |
| 3 | 公共参数注册表、介质只读核实/备份与持久状态 | P3A-01～04 |
| 4 | Rust DroneCAN 对拍、FDCAN/节点/ESC 状态、零输出 | CAN-A01～04；按实物需要 CAN-DNA |
| 5 | ULog/FAT/SD、有界记录与主机解析 | LOG-A01～05 |
| 6 | 单 IMU 与 H-RTK、USB/TELEM1 协议可按独立前置并行 | SENS-01～04、GNSS-01～03、MAV-01～03 |
| 7 | AHRS、QGC 参数读/设、已有数据源遥测 | EST-01～02、PARAM-01～02、TEL-01 相关部分 |
| 8 | 5 分钟联合运行、异常输入/重连与结果回填 | M1-01～02 |

主固件已有 P1/P3a、P4b-a/PL-a 软件实现；`tools/p0` 仍只是调研探针，不能替代对应 HIL。其余阶段先软件检查再由工程师集中实测，按实际结果更新 TODO。

## 3. 首轮硬件输入

| 输入 | 现场需确认 |
| --- | --- |
| 恢复 | QGC 版本、官方 PX4 镜像/hash、原 bootloader 独立入口、可用参数/校准备份 |
| IMU | ICM-42688-P ID、PA10 DRDY、SPI2/PH5、PF4 上电、实际方向 |
| CAN | 实际 CAN 口/适配器、供电/针序/拓扑、终端、四 ESC 和 rover node ID |
| ESC / GNSS | 固件/参数快照，ESC_INDEX 与 Status 频率，H-RTK SKU12059/消息速率 |
| SD | 卡型号/容量/文件系统、空闲空间、DMA/写延迟结果 |
| TELEM1 | UART7 线序/电平、适配器、波特率、流控；QGC 所在主机 |

其余控制/飞行模型、任务、RTCM/下载的细节按用户要求留到更多硬件测试后。资料缺失明确关联对应阶段，不填推测的实物值。

## 4. 联合测试记录

由工程师准备原始日志目录并记录：本工程/上游版本、固件 hash、配置、设备版本、各接口接线、采用的初始阈值、执行者。默认值见 [architecture](architecture.md)、[CAN](dronecan-notes.md)、[ULog](logging-budget.md)，实际采用值必须在测试前填写。

| 验证 | 操作与通过证据 |
| --- | --- |
| IMU | 记录至少 5 分钟静止和六面/转动数据；分别核对 8 kHz 样本、约 1 kHz FIFO 批次与 1 kHz ULog，DRDY/读出延迟、FIFO/丢样、SI 单位和 YAW270 方向符合阈值 |
| AHRS | 转动板子并比较 QGC 与 ULog；静止倾角/收敛/四元数范数正确，过期/异常数据显式无效 |
| DroneCAN | 独立 GUI/pydronecan 对拍节点/ESC/GNSS；确认只有零命令/DISARMED，测负载/时间/丢帧 |
| QGC USB | 实际上游 QGC 建立连接；Inspector 查看心跳/状态、`HIGHRES_IMU`、姿态、ESC、GPS 的值和更新率 |
| QGC TELEM1 | 通过实际串口链路重复检查；重新连接能恢复，源数据和单位与 USB 一致 |
| 参数页与脚本 | 完整读表、名称/索引/count；对选定非危险参数做受控读/设/回显，非法值拒绝；另一链路读到相同当前值，记录实际保存/重启行为 |
| 双链路负载 | 两个独立客户端同时运行；QGC 可每次验证一种连接，另一条由脚本监测，避免同一串口被抢占 |
| ULog | 复用 5 分钟联合记录，取 SD 卡用独立解析器核对 schema、时间/单位、参数、topic 频率/dropout；记录 SD 时延/高水位 |
| 故障与恢复 | 分段注入 CAN/GNSS 断线、USB/UART 重连/慢消费者、慢卡/缺卡；采样不被通信/存储阻塞，错误计数与日志区间可对应 |
| 资源 | 原始时序分布、CPU、栈/队列高水位和 deadline miss；正常段达到初始预算，异常段单列 |

`tools/check_mavlink.py` 当前检查 demo 的两项只读参数和协议基线；MAV/PARAM 阶段须扩展消息与读/设检查，再作为 M1 工具。Python 向量回读、离线构建或 QGC 截图单独都不能证明整条链已验收。

## 5. 回填结论

```text
执行日期 / 执行工程师：待填
固件 / 配置 / 器件 / QGC 版本：待填
采用的阈值：待填
原始数据 / 抓包 / ULog / 校验值：待填
正常 5 分钟结果：待填
QGC USB / TELEM1 消息与参数结果：待填
故障注入 / 恢复 / 未通过项：待填
M1 是否通过：待填
依据硬件结果调整的下一阶段事项：待填
```

此文件是目标和操作单。M1-02 在真实结果回填后才关闭；非零电机、控制闭环和后续功能采用各自的验收记录。
