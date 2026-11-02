# P4p INA219 / SEN0291 支路电流

日期：2026-11-02。0x44 通信及上一版三条命名值 10 Hz 已实测通过；本次按工程师要求先改为单条 `DEBUG_VECT` 同显 V/A/W，待构建/烧录核对。精度与断线恢复待参考负载。阶段门见 [plan.md](../../../plan.md#ina219-支路电流工程师新增)。

## 接线与实现

- SEN0291 拨码 **A0=0、A1=1 → 0x44**（DFRobot 官方四地址：0x40/0x41/0x44/0x45）。固件启动及未初始化重试时只读探测这四个地址并采纳首个应答地址；不做跨地址写。
- 线序陷阱：模块 **P3 通孔座**为 1=VCC、2=SCL、3=SDA、4=GND，与 Pixhawk I2C 口一致；**P2 Gravity 座**为 1=GND、2=SDA、3=SCL、4=VCC，与飞控整序相反。直插 Gravity 线会把 5 V 接到模块 GND、把飞控 GND 接到模块 VCC，模块不上电并可能把 SCL/SDA 拉低。
- Holybro v1/v2/mini 独立 4 针 I2C 均为 SCL3/SDA3；固定 PX4 映射 I2C3/PA8/PH8。100 kHz、DMA1_CH4/5、EV/ER=P6；PG4 低使能外设 5 V。
- 参考 [LibDriver](https://github.com/libdriver/ina219/tree/266a1ee32a5f70e4e5d47591ce28c4e1c387857c) 1.0.7：地址枚举使用 8 位写地址，本项目 Embassy 使用 7 位地址；其返回 mV/mA，本项目统一 V/A。
- ±80 mV PGA 与 10 mΩ 对应 ±8 A；`Current_LSB=8/32768 A`、CAL=16776。TI SBOS448G 说明 FS0 固定为零，公式截断值 16777 清零最低位后才写入/比对；舍入误差远小于 ADC 一步。12-bit 单次 shunt+bus，等待 CNVR，OVF/饱和/超范围拒绝；每次转换前后核对配置/标定，I2C 事务由 Embassy 内部 20 ms 超时约束，转换等待用 10 ms 截止时间，失败后 500 ms 重试。
- 出错后释放总线：清 PE、把 PA8/PH8 切为开漏输出，记录空闲电平，必要时补 9 个 SCL 时钟并产生 STOP，再恢复 AF4 与上拉；`recoveries`/`idle_*` 可在 `ina219` 中查看。
- 共享最新快照向 USB/TELEM1 各发布单条 10 Hz 标准 `DEBUG_VECT`（ID 252）：`name=INA219`、`time_usec=采样启动相对微秒`、`x=电压 V`、`y=电流 A`、`z=功率 W`。W 为同次 V×I、负电流符号保留，年龄 >300 ms 或失败时停发。请求 ID 252 回同一消息，无新鲜数据时 ACK 为 Temporarily Rejected。
- 工程师要求使用现有 Windows QGC Inspector；标准消息可直接解码，选中 `DEBUG_VECT` 后可同时查看/绘制 x/y/z。QGC 5.1.5 源码已支持按 `name` 区分 `NAMED_VALUE_FLOAT`，旧版 Inspector 可能合并；单条调试消息避免依赖此功能。
- 时间戳取本地检测到转换完成时刻，随后读取同次冻结寄存器；转换等待在轮询内部按 10 ms 截止时间退出，不再从外部取消 DMA 事务。
- `!ina219` / MAVLink Console `ina219` 显示地址、探测结果、总线电平、配置、计数、恢复次数与真实值/有效性。`last_error`：1=I2C，2=事务超时，3=配置/标定，4=转换超时，5=测量状态/范围；`i2c_error`：1=NACK，2=Bus，3=Arbitration，4=Timeout，5=Crc，6=Overrun，7=ZeroLength。

## 软件检查

| 项目 | 结果 |
| --- | --- |
| 核心 | 6 项 host 用例通过：标定/单位、负电流/功率、CNVR/OVF、范围和陈旧/未来时间 |
| fmt / Clippy / 普通构建 / 打包 | 普通/host Clippy 与 release/布局通过；raw/padded=`382344/382368 B`，static RAM=`233472 B`，余量 `224256 B` |
| Ubuntu 上传工具 | `bash tools/setup_ubuntu.sh` 成功，固定 uploader SHA-256 通过，help 可用；bash 语法与 6 项端口回归通过 |

核心输出：`test result: ok. 6 passed; 0 failed; 0 ignored; 0 measured; 59 filtered out`。

局部简化审视已完成：固定单一模块配置、共享采样周期/快照/标准消息构建；探测只读、自动采纳地址，`join3` 保持任务接线直接。

## 实物历史与本次待验

- 早期 `source=52ddee6405ec0d05` 四地址超时、SCL/SDA 低；`4891cadcea53b5d2` 加上拉后短暂获得 5 个样本，随后 SCL 持续低、SDA 高，重试未恢复。外层超时取消 DMA 是可能原因，尚未单独复现证实。
- 上一版 `source=7874cc5479bfe119` 经原 bootloader 烧录/CRC 校验（应用入口 0x08020000）。读取 `ina219`：`address=44 initialized=true valid=true config=299b calibration=16776 samples=63 errors=0 last_error=0 i2c_error=0 voltage_v=22.648`；后续 `samples` 164→255、`errors=0`，`scl_high=true sda_high=true recoveries=0`，SCL 持续低电平未复现；自动恢复分支未触发验证。
- 同期 USB `NAMED_VALUE_FLOAT INA219_A/INA219_V/INA219_W` 每 100 ms 一组，V≈22.65 V、I接近 0；负载条件和参考电流未提供。
- 工程师反馈 Windows QGC 已能看到消息，显示的单个值在 -0.012～0 波动，要求 V/A/W 同显。

## 本次单条消息 + ULog（2026-11-02）

- 烧录 `source=b2e70b0ecc7ab844`（经原 bootloader 上传/校验，应用入口 0x08020000，SHA-256 见 `v6x-rust-led.build.json`）。修改为单条 `DEBUG_VECT` 且把同次测量加入 ULog 主题 `rust_current_sensor`；工程师在 Windows QGC Inspector 选中 `DEBUG_VECT`（`name=INA219`）已确认 **x/y/z 三值同显**。
- `pymavlink` 实测约 4.1 s：41 个唯一 `timestamp_sample`，约 **9.98 Hz**；`voltage_v≈0.924`、`current_a` 在 -0.0029～-0.0020、`power_w=V×I`。请求 ID 252 返回 ACCEPTED。实测总线约 0.92 V，需工程师确认支路供电和接线。
- 前一份较长短日志下载并 pyulog 校验：`rust_current_sensor` **43 条、约 9.975 Hz**、中位周期 100044 µs、V/A/W 与采样时间一致，`file_corruption=false`、**无 dropout**；`ver_sw=b2e70b0ecc7ab844`。日志 `target/ina219-vaw-b2e70b0ecc7ab844-clean.ulg`（SHA-256 `4019c10e…7a464a`）。
- 另一次全量记录 `target/ina219-vaw-b2e70b0ecc7ab844-verified.ulg`：5.3 s 内 `rust_current_sensor` 19 条，但普通队列丢 **5685 条**、4 个 dropout（总 2.3 s、最大 1303 ms），`high_water_bytes=130020`。丢样发生在 IMU 1 kHz + CAN 400 Hz + 遥测共存的正常负载下，普通队列接近满（136 KiB 槽队列），是 **SD 写入吞吐落后**的既有日志吞吐问题（见 [logging-budget.md](../logging-budget.md) 缓冲边界），非 INA219 路径本身；干净日志证明 INA219 记录正确。该丢样并入 PL-a/LOG 负载待验项，不据此宣称 INA219 日志阶段已通过。
- 剩余：精度收口需工程师接入已知负载/参考电流；断线失效/恢复需一次模块拔插并核对停发/恢复；INA219 日志在联合负载下的零 dropout 归 M1/PL-a 联合运行核对。
- 后续软件（source=`15a3094727d65d4a`）：IMU ULog 降至 200 Hz，并新增日志开关/速率参数 `LOG_EN_IMU/INA/CAN/HEALTH/EVENT` 与 `LOG_IMU_HZ/LOG_INA_HZ/LOG_CAN_HZ/LOG_HEALTH_HZ`；DEBUG_VECT 单条 V/A/W 与 `rust_current_sensor` 不变。该版已通过必要检查、尚未烧录，待工程师安排上板核对。
