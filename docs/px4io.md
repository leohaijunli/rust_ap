# PX4IO：Rust 重构与移植计划

日期：2026-11-04。范围和必要测试以 [plan.md](../plan.md) 为准。

## 当前状态与边界

- 已实现的是 **H7 上的 Rust/Embassy PX4IO 驱动**，不是 IO MCU 的 Rust 固件。原 IO 应用与 bootloader 没有刷写；2026-11-04 H7 合并包已回装，但 PX4IO 实物通信 Timeout、ready=false，实体 logger 按键按设计禁用。[回装记录](acceptance/px4io/20261104-merged-flash.md)。
- 固定参考为 PX4 `v1.17.0` / `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`。
- 官方 IO-v2 配置选择 STM32F100C8 / Cortex-M3，HSE 与 CPU 24 MHz；flash 64 KiB、RAM 8 KiB，应用 `0x08001000` / 60 KiB，前 4 KiB 为 bootloader。来源：`boards/px4/io-v2/nuttx-config/{nsh/defconfig:16,include/board.h:57-65,scripts/script.ld:36-46}`。这些是源码预期，实际 IO 型号/固件仍需工程师确认。
- USART6 PC6/PC7、1.5 Mbit/s 是 H7→IO 链路；IO 侧 USART2 收发。H7 的原 PX4 bootloader、`0x08020000` 入口、QGC 恢复与 FRAM 参数区不改变。
- IO 管理 MAIN PWM、RC、安全按钮/灯及失联输出；H7 管理 CAN、EKF、SD、MAVLink 与本板 PF9/TIM14 蜂鸣器。**DroneCAN ESC 不经过 IO，IO 不能替代 CAN 停转策略。**

### 已完成的 H7 接口

`src/core/px4io.rs` 提供 v5 报文、CRC-8、回复长度/地址/寄存器数检查；`src/drivers/px4io.rs` 独占 USART6、DMA2_CH2/CH3，IRQ P5，先启动 RX 再发送 TX。

- 单次事务预算 10 ms，最多 3 次尝试，串口 break 后间隔 1 ms；错误后间隔 500 ms 重新探测。SD/HAL 同步等待可延迟线程，预算不等于已证明可抢占的硬件期限；迟到回复仍拒绝。
- 仅在 protocol=5、hardware=2 和容量合法时写入；公开写操作只允许 arming=0、safety_off=0、features=0、disarmed PWM=0 与按钮 ACK=0。
- 回读 setup、状态及 PWM。通信失效、陈旧、INIT_OK 缺失或输出配置不符时，实体日志按键停用；不自动解锁，不写 direct/nonzero PWM，不复位或更新 IO。
- `!px4io` 上报版本、固件 CRC、状态、PWM 与错误/重试计数。IO 的 `OUTPUTS_ARMED` 是定时器使能状态，不能直接等同于飞行解锁；与实际脉宽分别报告。

## GPS1 灯与按键

工程师选择复用 **全部已核实可控的灯**，不更换 GPS1 接线。

| 通路 | 当前认识 / 实施边界 |
| --- | --- |
| PF5 按钮 | 高有效、上拉按固定 PX4 配置；40 ms 消抖，短按释放后切换 logger start/stop，1.5 s 长按仅保留事件；上电或重连时按住不动作 |
| IO 安全灯 / FMU PD10 | 官方 IO 固件驱动安全灯，H7 PD10 保持浮空输入。v5 没有任意灯型命令；不能伪造 safety_off/armed 来显示日志状态 |
| GPS1 I²C1（PB8/PB9） | PX4 会尝试独立 RGB 灯驱动；中间 FC Status 大灯可能走此通路。需确认地址/型号，ACK 不等于器件识别；现 Rust 尚未增加探测命令 |
| 左右小灯 | 未确定是安全灯、电源/活动灯还是其他连接，不能承诺独立控制或改色 |
| PF9 / TIM14 | MCU 侧推挽方波，0% PWM 为低电平静音；4 kHz / 50% 音调，1/2/3 声分别反馈启动/停止/失败，实际可闻性待验 |

PX4 来源：FMUv6X `board_config.h:300-350`、`SafetyButton.cpp`、IO `safety_button.cpp` / `registers.c`、`ToneAlarmInterfacePWM.cpp`。`ROMFS/px4fmu_common/init.d/rcS` 同时启动外部 `rgbled`、`rgbled_ncp5623c`、`rgbled_lp5562`、`rgbled_is31fl3195`，因此不能仅凭一根安全灯信号判断整个模块一定是单色。

### 不改 IO 也可先推进的工作

先识别 GPS1 的 I²C 灯及可见发光点；若大灯为 I²C RGB，可在 H7 独立显示 logger 状态，无需 IO 重写。保持 IO 安全灯原语义。识别前不向猜测的 LED 地址写配置；Embassy H7 的空长度 `I2c::write` 不能直接作为地址探测，需核对可用 API 或实现有界的专用探测。

## Rust 重构目标（本次只计划，不开始刷写）

1. 把 IO 的协议、状态机、按钮/灯策略从寄存器访问和中断中分离，形成 host 可测试的 no_std 核心。
2. 先实现可恢复、输出关闭的最小 IO 应用，再加入实际需要的 PWM/RC；不为了“完整移植”提前搬入所有协议或调参分支。
3. H7/IO 保持独立编译与内存布局，共享真实需要的协议契约；默认 disarmed、零脉宽、不自动恢复解锁。
4. LED 显示与输出权限彻底解耦；GPIO 单色灯只控制灯型，多色由已识别的 I²C RGB 驱动负责。

### 目标结构

```text
src/core/px4io.rs              现有 H7 端协议；共享提取前保持原位置
src/drivers/px4io.rs           现有 H7 USART6 所有者、超时、诊断

io-firmware/
  Cargo.toml / Cargo.lock      独立 Cortex-M3/STM32F100 配置
  memory.x / build.rs          IO 应用入口、flash/RAM 边界
  src/main.rs                 Embassy 启动与资源装配
  src/board.rs                固定引脚、时钟、IRQ、定时器/DMA 所有权
  src/core/registers.rs        寄存器空间、合法值与 capability 契约
  src/core/output_gate.rs      disarmed/failsafe/陈旧输入/恢复策略
  src/core/button_led.rs       消抖、gesture、LED 覆盖租约
  src/core/rc.rs               所选 RC 协议解码、有效性与新鲜度
  src/drivers/fmu_uart.rs      IO 侧 USART2/DMA，仅收发有界报文
  src/drivers/pwm.rs           定时器整组更新及零脉宽
  src/drivers/rc.rs            RC 捕获/串口硬件适配
  src/drivers/watchdog.rs      看门狗与复位原因

crates/px4io-protocol/         IO-R2 才按两个真实消费者提取
  src/lib.rs                  无 HAL、无堆；常量/CRC/编解码/版本
tools/                        两 ELF 构建、IO 校验与统一发布清单
```

上述是目标职责，不在 IO-R1 创建空文件或通用驱动框架。`registers` 不访问 HAL，`output_gate` 不播放灯效，ISR 只收集数据/清中断/唤醒；重活交 Embassy 任务。只有 IO 端真正开始使用时才提取共享 crate，不提前重组主工程。

### 对照 PX4 的迁移清单

| PX4 固定参考 | Rust 职责 | 保持 / 明确差异 |
| --- | --- | --- |
| `protocol.h`、`px4io_serial.cpp` | 共享报文核心与两端 UART 适配 | 页/字段/小端/CRC/回复码；先收后发、固定容量、错误分类；扩展有明确版本与 capability |
| `registers.c` | `core/registers.rs` | 可读/可写范围、ACK、零脉宽与状态位语义；未知写操作拒绝，标准兼容差异记录，不静默接受 |
| `safety_button.cpp` | `core/button_led.rs` | 保留标准 ≥1 s 锁存事件兼容；独立短/长按事件按序号确认；覆盖灯型有租约，不复用 arming 位 |
| `mixer.cpp` | `core/output_gate.rs` | 输出关闭、disarmed/failsafe/陈旧输入策略；禁止未经授权的 in-air reset 自动恢复解锁 |
| IO 板级 / PWM / RC / HRT | `board` 与专用 drivers | 核对实际 timer、通道、IRQ、DMA、计数器位宽/回绕；不照搬 H7 HAL 或假定 async 等于有界 |
| `px4io_uploader.cpp`、`rcS` | H7 侧显式更新流程与打包工具 | 保留旧 IO bootloader；双 MCU 两份镜像、一个交付包；校验后才宣称更新成功，不自动擦除/升级 |

## 分阶段实施

**估算**：有调试器、明确接线及工程师配合时，最小通信/按钮/LED 固件约 1–2 周；覆盖 PWM、所需 RC、故障与更新恢复约 4–6 个工程周。不是日历交付承诺；更多 RC 变体和未发现的 HAL/恢复问题增加工期。

建议单独 `io-firmware/` crate / lock / linker，显式目标 `thumbv7m-none-eabi`，不得让 IO target 覆盖主工程的 H7 配置。保持纯 Rust/no_std/Embassy、无堆与有界队列；F100 HAL 缺口才使用注明不变量的 PAC。首轮预算目标为 flash ≤60 KiB、静态 RAM ≤6 KiB，剩余 ≥2 KiB 供共享栈；按真实 ELF 和水位验证调整，不把 host 编译当上板通过。

| 阶段 | 估算工程日 | 实现与阶段门 |
| --- | --- | --- |
| **IO-R0 硬件与恢复** | 1–2 | 确认实物 MCU、原 IO 版本/CRC、SWD/电源、USART/LED/RC/PWM 接线；可读则备份 IO flash，不做解锁读保护或整片擦除。必须有不依赖新 IO 应用的恢复路径；先判定 I²C 灯是否已能满足需求 |
| **IO-R1 最小 Rust 启动** | 2–3 | 独立 M3 工程、时钟/向量/中断/DMA、旧 bootloader 应用入口；所有 PWM 引脚保持关闭。构建/分区检查后，工程师拆桨验证启动与恢复，不接电机输出 |
| **IO-R2 协议与用户界面** | 2–3 | 保留 v5 标准寄存器/CRC/错误契约；扩展单独 capability/version 与 LED/gesture 页，具体编号在此阶段固定。IO 唯一驱动安全灯，覆盖模式带有界租约，过期恢复官方灯型；新短/长按事件有序号及显式 ACK，保持原 ≥1 s safety 事件兼容，不重复触发 logger |
| **IO-R3 输出与 RC** | 4–7 | 8 路 PWM/OneShot、整组更新、零脉宽、disarmed/failsafe、实际所需 RC 协议与新鲜度；watchdog、UART/DMA 错误和失联处理。恢复不自动解锁；非零验证仍依赖 P5a、H-06/07 明确授权，未需 RC 变体留后续 |
| **IO-R4 双 MCU 更新包** | 2–4 | H7 携带 IO `.bin` 和版本/CRC/大小清单，由 H7 经旧 IO bootloader 单独更新；只允许 disarmed/输出关闭下显式更新，失败保持可恢复。最终一个 `.px4` 交付，但不是把两个 MCU 镜像直接拼接写入 H7 |
| **IO-R5 联合故障验收** | 3–5 | 一次联合记录覆盖 H7/IO 分别复位、通信中断/坏包、陈旧 RC、零输出与恢复、LED 租约超时；一次受控 IO 更新中断后恢复；官方 PX4 可回装并恢复官方 IO。复用既有有效结果，不默认随机断电/重复长时压测 |

IO-R0→R1→R2 先形成可用按钮/灯原型；之后再放行输出与更新工作。**IO-R2 的 LED 覆盖只能改变显示，不能修改实际 safety/arming 或输出权限。** 单色 IO 灯不会因软件重写变成 RGB，多色仍由已核实的 I²C 灯实现。

### 必要验收与放行

- **协议 / 核心**：固定 PX4 请求/回复逐字节对拍；坏 CRC、截断、越界页/数量、重复 ACK、gesture 回绕/抖动、LED 租约过期、控制/RC 陈旧、零输出与恢复。只测试受影响边界，不追求默认全量覆盖率。
- **构建 / 资源**：各实际 target 的 fmt/Clippy、release、入口/向量/分区和两 MCU artifact/source/hash 清单；IO 不能超过 60 KiB flash / 8 KiB RAM，真实栈余量另测。
- **IO-R2 原型上板**：工程师确认按住启动/重连不动作，短/长按无重复，灯覆盖不改变 safety/arming，租约失效恢复原灯型，PWM 始终无脉冲。
- **IO-R3 输出上板**：先拆桨零输出与过期/坏帧，再按 P5a 限时授权验证实际 PWM/RC 和故障停转；不能以 MCU 没有 panic 代替示波器/逻辑分析仪输出证据。
- **更新 / 恢复**：先验证恢复通路，再做一次受控 IO 更新中断；IO/H7 各复位及官方 PX4 回装后可恢复。H7 原 P1 结果未受改动部分直接复用。
- 每阶段分别记录“host / 构建 / 刷写 / 功能 / 故障恢复”；上一门未通过不放行下一项硬件工作。只写计划不触发 IO 构建或刷写。

IO 重写不是当前 M1 的必需条件；不能因 UI 原型通过就关闭原 CAN/SD/RC/非零输出门。阶段软件/刷写/实物验收分别记录到 `docs/acceptance/px4io/`。

当前 H7 软件结果：[2026-11-04 记录](acceptance/px4io/20261104-host.md)。
