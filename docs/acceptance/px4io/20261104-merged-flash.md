# 普通合并包回装：E6-H / PX4IO / logger-switch

2026-11-04，工程师明确要求合并全部修改、推送并烧录 H7。**上传/校验与 H7 启动回读通过；PX4IO 通信未通过，EKF2 未在板上启用。** 没有更新 IO MCU、改 bootloader 或执行非零电机动作。

## 包与上传

- 普通、fault_injection=false，source=`c6fe90f898b68f8f`；raw/padded=`426488/426496 B`，static RAM=`269856 B`、余量 `187872 B`，共享栈 64 KiB / 故障栈 1 KiB；入口 `0x08020000`、board ID 53，ELF/bin/PX4 一致性通过。
- 包 SHA-256：`3e3adef360dfa33da1701c9e96fa23a60a5bb60c29777a7afebf3f0f5285a334`。固定原 PX4 uploader SHA-256：`f05929e1035e693005677e088e4e7085503b2e8ab3d7c8785f7bd84cd48880f8`。
- 17:47:05 UTC 前置回读：COM26、1209:0001 / V6X-RUST-001，旧 `588ab77941985319`，心跳 base_mode=0，logger 未运行。只发送原 `!bootloader` 应用恢复命令。
- 原 bootloader 在 COM12：board=53、rev=0、protocol=5；serial=`004900223132510339313731`，chip=`0x20036450` / STM32H7 V，应用 flash=1966080 B。没有使用 `--force` / `--force-erase`。
- uploader 顺序完成应用擦除、写入、verifying、done/100%，返回成功，耗时 19 s；未打印具体 CRC 数字，不补造。原始输出：[upload.txt](20261104-merged-upload.txt)。

## 17:50:59 UTC 回读

- COM26 回到 Rust CDC；`AUTOPILOT_VERSION.flight_custom_version=[143,143,182,152,248,144,254,198]` 对应 source；版本 0.2.0。
- 3 条 sys=1/comp=1、autopilot=PX4 心跳，均 base_mode=0；`!platform` source 正确、fault_injection=false、VTOR=08020000、时钟寄存器匹配、backup config/bkp preserved=true；`!fault valid=false`。
- 编译时 git 仍为父提交 `788eff9`（先验证/烧录、后提交）；source_id 是本次实际源码内容身份，不把 git 字段误当旧镜像。
- **PX4IO：ready=false，protocol/hardware/CRC 未取得，last_error=Timeout**。读诊断时 transactions=789、retries=526、timeouts=789、crc_errors=0，最长事务 13601 µs；不能证明 IO 上电、协议或 PWM=0，也不能通过增加超时伪称通信正确。
- `LOGGER_SWITCH.enabled=false`、presses/requests=0、buzzer=false，logger 未启动；PD10 安全灯不由 H7 覆盖。按键/蜂鸣/日志实物功能仍待通信与 H-12，不关闭 LOG-A06。
- 只读短时指标：P1 替身 deadline_misses=0、time_errors=0、CPU busy=154‰，栈 untouched=37572 B、guard=true；不是 EKF WCET、长时负载或完整 M1 验收。

原始前后回读：[preflight.json](20261104-merged-preflight.json) / [postflight.json](20261104-merged-postflight.json)。软件验收与来源沿用[PX4IO host 记录](20261104-host.md)，E6-H 结果见[host 记录](../ekf2/20261104-e6-host.md)。

## 下一步

1. 核对 IO MCU 供电/复位与 USART6 实物响应，定位 Timeout；取得真实版本/状态/PWM 后再验证按钮、蜂鸣与日志。不自动复位/更新 IO，不绕过 ready 门。
2. 工程师提出启动 EKF2，但当前固件没有板载任务或启动命令。需先明确受限真实 IMU 姿态实验或正式 E7 输入桥/任务/参数/健康/回退范围；不把 host 合成观测上板当真实导航，不放行原实物依赖门。
