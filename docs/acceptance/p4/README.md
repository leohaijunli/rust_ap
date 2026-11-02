# P4 ICM-42688-P 软件验收

日期：2026-11-02。**SENS-01 的身份/基础链路、USB `HIGHRES_IMU` 及短时实物 ULog 已做有限 HIL；六面、60 s 长时、校准与故障恢复尚未通过，P4 未关闭。**

新软件的标准 `sensor_accel` / `sensor_gyro` 字段与检查见[消息记录](../messages/README.md)；以下固件/hash 和旧 `imu_icm42688` HIL 保留历史绑定，新 schema 的实物核对并入下一次 P4 必验记录。

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
| release/打包 | Linux Rust 1.99.1；source=`021161d4c09a4b91`；raw/padded=`361856/361856 B`；static RAM=`231672 B`、空余 `226056 B`；布局/向量/ELF/bin/PX4 一致 |

最终构建 `.px4` SHA-256：`319b90ccc2effa9c51a220ba909bac5a6d4373e7e14f5993f31bfe747f4f4d07`；`.bin` SHA-256：`5eda35dfda5662810098c8f3977d8cb23060c15993c51489d9149ac0f3985518`。该包已由原 PX4 bootloader 上传/CRC 校验并运行；心跳 `autopilot` 改为 PX4 兼容，仅用于 QGC 把机载日志保存为 `.ulg`，仍不声明 FTP 能力，日志继续走 `LOG_REQUEST_*` 消息通道。

## 有限 HIL（2026-11-01）

| 检查 | 结果 |
| --- | --- |
| 上传/身份 | 正常应用 `COM4`/`1209:0001` 发送 `!bootloader`，重枚举 `COM3`/`3185:0035`；固定 PX4 v1.17 `px_uploader.py` 识别 board 53、协议 5、STM32H7[4|5]x Rev V、1966080 B，擦除/写入/CRC Verify/重启通过 |
| 应用 | 当前板载 `P1_PLATFORM`：source=`021161d4c09a4b91`、fault_injection=false、clock registers_ok=true；本节 IMU 数据采集自前一 source=`496484e26b3502fe`，配置未变 |
| IMU 身份 | `!imu`：initialized=true、WHO_AM_I=`47`、odr_hz=8000、log_rate_hz=1000、实际 spi_hz=24000000 |
| 短时计数 | 两次 `!imu` 间样本 `1146992→1195640`，host 窗口 6.019719 s，观测 8081.441 Hz；串口命令窗口/器件时钟误差未剥离，不据此关闭速率项 |
| 健康 | 第二次快照 transfer_errors=0、decode_errors=0、fifo_resets=1（初始化 flush）、ahrs_dropped=0、latest_age_us=950 |
| MAVLink | `tools/check_mavlink.py` 全部通过；`HIGHRES_IMU` 静止快照 accel=`(0.141,-0.362,-9.944) m/s²`、gyro=`(-0.0016,0.0035,-0.0172) rad/s`、temperature=`34.54°C` |
| 短时 ULog（2026-11-02） | source=`81d6c1221048f08a` 的板上日志由 MAVLink 下载并经 pyulog 1.3.0 解析；`imu_icm42688` 有 2686 条/2.652 s（约 1012.5 Hz），`gyro_rad_s[0..2]`、`accel_m_s2[0..2]` 均存在且为有限实数，文件无 corruption |

本轮只证明该板短时可识别、产生合理静止数据并写入自定义 `imu_icm42688` topic；host 计数窗口不是外部仪器 ODR/DRDY 测量，2.652 s 日志不能替代 60 s 稳态检查，单一姿态也不能确认 YAW270/六面方向或校准。

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
