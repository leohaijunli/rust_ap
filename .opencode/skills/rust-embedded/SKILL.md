---
name: rust-embedded
license: Apache-2.0
description: 为本项目编写、维护和审查纯 Rust/no_std/Embassy 嵌入式代码。涉及 STM32H753 启动、HAL/PAC、外设驱动、中断、DMA、共享状态、取消/超时、交叉编译或硬件验证时使用；不把 host 编译通过称为板上验收。
---

# Rust Embedded — Pixhawk 6X 项目适配

改编自 [full-stack-skills/rust-skills](https://github.com/full-stack-skills/rust-skills/blob/b8a6af74af28ee86b8cbbbc3075cde0b31a5155e/skills/rust-embedded/SKILL.md)，
固定 commit `b8a6af74af28ee86b8cbbbc3075cde0b31a5155e`。
Copyright 2026 PartMe.AI；许可保存在 [LICENSE](LICENSE)。
**本文件于 2026-11-04 修改**：精简并转为中文项目约束，保留三份上游参考；
不导入其他技能、示例 crate 或安装/刷写脚本。

## 优先级与边界

- 先读 [plan.md](../../../plan.md)、[TODO.md](../../../TODO.md) 与本次修改范围；
  它们的阶段门、必要测试与已验事实优先于本技能及通用参考。
- 板上只用 Rust/Embassy，目标为 Pixhawk 6X Rev 3 / STM32H753、
  `thumbv7em-none-eabihf`。工具链、HAL/PAC、executor/time 版本以锁文件为准。
- 保留原 PX4 bootloader、前 128 KiB、`0x08020000` 应用入口、board ID 53
  和 QGC 恢复能力。不得套用其他开发板的 linker/runner 或整片擦除命令。
- 不换 RTIC/Tokio，不新增堆、panic handler、defmt/RTT 或第二套参数存储。
  C/C++ 参考程序只允许用于 host，不通过 FFI 引入板上应用。
- 默认 disarmed、输出关闭；实体操作由工程师执行，刷写/复位/非零输出
  必须符合明确授权和当前阶段。未过硬件门不能放行依赖阶段。

## 工作流

### 1. 区分任务与核对事实

区分纯核心/DTO、驱动协议、板级接线/启动和 host 工具。
纯逻辑可以先做 host 验证，不能为其虚构硬件通过结果。

从现有 `Cargo.toml`、`Cargo.lock`、`rust-toolchain.toml`、
`.cargo/config.toml`、`memory.x`、`build.rs` 与平台代码核对实际配置。
引脚、电压、时钟、IRQ、总线模式/地址和 DMA 区域须有固定源码、
厂商资料或实测依据，不能按其他 STM32 型号猜测。

确认 target、启动或内存布局时，按需读
[target-and-runtime.md](references/target-and-runtime.md) 和
[硬件台账](../../../docs/fmuv6x-hardware.md)。

### 2. 保持已有分层和唯一真源

- 核心保持 `no_std`、HAL 无关、host 可测；板级层管理实际引脚、时钟和外设。
- 仅在真实复用或边界需要时抽取 trait/crate；不把单板单器件扩成空泛框架。
- 复用既有参数、队列、日志与遥测实现；保持单位、坐标、采样/接收时间、
  source、序列、有效性和数据年龄的显式语义。
- 错误不得用 `.ok()` 静默吞掉；区分超时、坏帧、CRC、设备不可用和恢复失败。

### 3. 检查并发、DMA 和 unsafe 不变量

- ISR 有界、非阻塞；按器件规定清中断，重活交给现有执行器任务。
- 采样/估计优先于 SD、日志和遥测；慢消费者不得反压或消费掉估计器样本。
- 临界区尽量短，明确嵌套、共享所有权和优先级反转；不用裸 `static mut`
  绕过所有权，也不能凭 host 原子支持推断 MCU 支持。
- DMA 完成/取消前保持缓冲所有权；核对控制器可访问的 RAM、对齐、MPU
  与 cache 维护，不把 DTCM 当作通用 DMA 区域。
- async timeout 不证明同步计算或底层事务已经停止；核对 cancel/drop、
  硬件取消、重试上限、状态失效和从最新样本重同步。
- 仅在 HAL 无法表达实际需求时使用 PAC/MMIO；`SAFETY` 注释说明地址、
  生命周期、别名、并发和缓存假设。host/Miri 结果不代替硬件证明。

驱动、中断或 Embassy 任务修改时，按需读
[hal-and-concurrency.md](references/hal-and-concurrency.md)。

### 4. 保留故障和可恢复性

沿用当前 panic/HardFault、备份域、看门狗、超时诊断与输出仲裁。
不要因新增驱动或日志覆盖恢复入口、旧故障或原始失败证据。
日志必须有界，不能用大量输出掩盖实时性问题；不发布未知或虚构的状态。

### 5. 只做必要验证并交付

按 [plan.md 第 4 节](../../../plan.md#4-仅保留必要测试) 选择相关 fmt/Clippy、
受影响 host 用例、实际配置 release 构建/打包和必要布局检查。
复用未受改动影响的结果，不默认重跑 all-targets/all-features 或新增长时压测。

涉及硬件时再按当前阶段验证通信、时序、失效/恢复和工程师可见功能，
参考 [hardware-validation.md](references/hardware-validation.md)；其矩阵不是额外阶段门。
分别记录 host/构建、刷写、功能和硬件验收，未实测项保持待验。

编写时仍使用 `clean-code`；收尾用 `code-simplifier` 做一次局部审视。
简短记录结果、来源/许可、剩余项与依赖，不预写通过结论。
