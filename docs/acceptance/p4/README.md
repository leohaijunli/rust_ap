# P4 ICM-42688-P 软件验收

日期：2026-11-01。**SENS-01 的身份/基础链路及 USB `HIGHRES_IMU` 已做有限 HIL；六面、长时、ULog、校准与故障恢复尚未通过，P4 未关闭。**

## 已通过

| 检查 | 结果 |
| --- | --- |
| PX4 固定参考 | 对照 PX4 v1.17.0 commit `d6f12ad…` 的 `icm42688p` 寄存器头、状态机与 FIFO 流程；Rust 独立实现，差异见下文 |
| config SDK | `src/core/config.rs` 与 `src/core/icm42688.rs`：no_std、无 I/O、host 可测；默认 8 kHz ODR、1 kHz ULog、±16 g、±2000 dps、YAW270、2 ms stale、Mahony 选择 |
| FIFO/单位 | 20 B high-resolution packet；header/无效值/20 位符号扩展、132.48 LSB/°C、accel m/s²、gyro rad/s、传感器时间与 u32 IRQ 时间跨回绕用例通过 |
| 坐标 | 传感器数据按 YAW270 转为内部 FLU；MAVLink 再由 FLU 转 FRD。host 基向量用例固定方向，仍须六面实测 |
| Embassy 驱动 | SPI2 PI1/PI3/PI2、DMA1_CH2/CH3、PH5 CS、PF4 power、PA10/EXTI10；24 MHz 请求；EXTI15_10=P0、DMA=P1、TIM7 executor=P3 |
| 初始化/恢复 | 软复位、WHO_AM_I=`0x47`/RESET_DONE、低噪声启动等待、三 bank 配置/回读、FIFO flush；周期轮询寄存器，连续失败后重新初始化 |
| 高频采样 | gyro/accel 8 kHz；FIFO 水位 8×20 B，约 1 kHz 唤醒。允许 PX4 同型流程中的额外 1 包并保留到下一批，过量/满 FIFO 显式复位 |
| fan-out | AHRS 16 槽跨 executor 队列按需启用；满时清积压、计数并放入最新样本。ULog 独立抽取 1 kHz；遥测只读最新快照，不消费 AHRS 队列 |
| MAVLink | USB 与 TELEM1 各 20 Hz `HIGHRES_IMU`，只标记 accel XYZ、gyro XYZ、temperature；过期样本不发送；请求消息 ID 105 可读取 |
| 目录 | `src/core`、`src/drivers`、`src/comms`、`src/storage` 按职责整理；`#[path]` 保持已有 crate 模块 API |
| 软件检查 | fmt；普通/注入/Dump target Clippy；host Clippy（`-D warnings`）；52 项 Rust host 测试；77 项 Python 回归；17 项编译时钟谓词全部通过 |
| release/打包 | Linux Rust 1.99.1；source=`ad6b017000ad127e`；raw/padded=`361864/361888 B`；static RAM=`231672 B`、空余 `226056 B`；布局/向量/ELF/bin/PX4 一致 |

最终构建 `.px4` SHA-256：`5622749dbdc6dc0e763f482008640a457d68781378445318209259fe6f3004de`；`.bin` SHA-256：`e76701d101ccf91ec035612c2d9d1e4f0b8cb562f6d4ac828a26ffa4f8312858`。板上有限 HIL 使用前一精确 source=`496484e26b3502fe`；最终 `ad6b…` 增加断链响应清理，尚未重复刷写。

## 有限 HIL（2026-11-01）

| 检查 | 结果 |
| --- | --- |
| 上传/身份 | 正常应用 `COM4`/`1209:0001` 发送 `!bootloader`，重枚举 `COM3`/`3185:0035`；固定 PX4 v1.17 `px_uploader.py` 识别 board 53、协议 5、STM32H7[4|5]x Rev V、1966080 B，擦除/写入/CRC Verify/重启通过 |
| 应用 | 当前板载 `P1_PLATFORM`：source=`496484e26b3502fe`、fault_injection=false、clock registers_ok=true |
| IMU 身份 | `!imu`：initialized=true、WHO_AM_I=`47`、odr_hz=8000、log_rate_hz=1000、实际 spi_hz=24000000 |
| 短时计数 | 两次 `!imu` 间样本 `1146992→1195640`，host 窗口 6.019719 s，观测 8081.441 Hz；串口命令窗口/器件时钟误差未剥离，不据此关闭速率项 |
| 健康 | 第二次快照 transfer_errors=0、decode_errors=0、fifo_resets=1（初始化 flush）、ahrs_dropped=0、latest_age_us=950 |
| MAVLink | `tools/check_mavlink.py` 全部通过；`HIGHRES_IMU` 静止快照 accel=`(0.141,-0.362,-9.944) m/s²`、gyro=`(-0.0016,0.0035,-0.0172) rad/s`、temperature=`34.54°C` |

本轮只证明该板短时可识别和产生合理静止数据；host 计数窗口不是外部仪器 ODR/DRDY 测量，单一姿态不能确认 YAW270/六面方向，也没有完成校准。

## 与 PX4 参考的边界

- 沿用经固定源码核对的 20 B high-resolution FIFO、8 kHz、±16 g/±2000 dps、reset/power/config/FIFO/check/recovery 顺序及 AAF 配置。
- 不复用 PX4 C++ work queue、uORB、设备类或传感器发布器；外设、DMA、EXTI 与执行器均使用 Embassy。
- PX4 分别发布 accel/gyro FIFO；本项目形成带同一 sample timestamp/sequence 的六轴 `Sample`，分别 fan-out 到 AHRS、ULog 和最新快照。
- PX4 输出 FRD；本项目内部契约先保持 FLU，只有 MAVLink 边界转 FRD。

## 待工程师实物验收

1. SENS-01 剩余：用逻辑分析/示波确认 PF4/PH5/PA10、实际 SPI mode/频率和 DRDY 边沿；软件身份/初始化已通过。
2. SENS-02：静止至少 60 s，分别核对 8 kHz 样本、约 1 kHz FIFO 批次与 1 kHz ULog；记录 DRDY→发布延迟、FIFO reset、transfer/decode error 和队列丢样。
3. SENS-03：六面与逐轴转动，确认 YAW270、内部 FLU 和 MAVLink FRD 的符号/单位；静止比力约 1 g，gyro 约零，温度合理。
4. SENS-04：QGC/脚本同时读取 USB/TELEM1 `HIGHRES_IMU`；一次受控传感器电源或 DRDY 故障后确认无伪造零值、陈旧数据停止发送、驱动可恢复且 CAN/USB 快路径继续。

上述实物项和 PL-a 依赖关闭前，不放行 P6a，不把本软件构建称为传感器可用性证据。
