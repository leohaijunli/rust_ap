# A1 泛型 Runtime 重构与新代码验证

日期：2026-11-05。方案见 [四阶段计划](../../ekf2-runtime-refactor.md)。
当前为软件验证，板载 1 kHz、资源/时基与 mag/range 实物资格待验。

## 已完成的软件结果

- 45 项原有 estimator host 用例通过；随后 8 项 Runtime 专项通过：三种算法、量测等待/拒绝、参数与延迟、连续超时恢复、丢样重同步、无 GNSS 隔离、Fatal 重配置、产品输出复位/陈旧撤销。
- 新 `--split` 入口的 16 场景通过，复用固定 PX4 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` 黄金数据；涵盖状态/完整 P、创新、复位及输出连续性。未覆盖或放宽黄金文件与既定容差。
- 旧入口 `gnss_range_stationary` 的状态/P/NIS 兼容检查通过。
- GNSS / no-GPS 两种标准输出各 26000 输入、1300 发布、165 字段，经 pyulog/pymavlink 独立核对通过；合成输入，不作为实物融合证据。
- host lib/两个回放 example 的 Clippy 通过；普通 H753 Clippy、`estimator-mahony` H753 check 通过。构建工具两项 host 检查通过。

回放二进制 SHA-256：`2af52033f7cb463fe33b07e8d677c14615336578b5ad2d53683b3768c6c6ecb3`。
详细结果暂存于 `target/ekf2-refactor/report.json`、`target/ekf2-refactor-legacy/report.json` 和 `target/ekf2-refactor-standard/{gnss,no-gps}/report.json`；交付时保存汇总证据。

## 首个 release：构建通过，调用链栈未通过

source=`536766f00f812a82`；ELF SHA-256=`a3a4b3e1003c5361a59f159ae738165c3535b6631f7aa88f83d733a5c3bcc7a9`。
raw/padded=`901112/901120 B`，static RAM=`286432 B`；应用入口 `0x08020000`、board ID 53、分区/打包检查通过。

单函数入口检查报告最大 24560 B、oversized=0，但调用链不满足共享栈余量：

| 函数 | 入口报告字节 |
| --- | ---: |
| `Runtime<GnssEkf>::replace_filter` | 23396 |
| `<GnssEkf as NavFilter>::new` | 24560 |
| `Runtime<GnssEkf>::recover` | 23416 |
| estimator worker poll | 8408 |

ARM 反汇编在 `0x08076378` 确认 `replace_filter` 调用 `0x080ce540` 的 trait `new`，然后从临时栈复制 `0x5b50` 字节到实例。**仅两层入口已至少 47956 B**，尚未计 worker、构造子函数、被抢占线程或 IRQ，不能以“单帧小于 32 KiB”放行。

整改：保持 `NavFilter` 七个方法签名，产品静态 Runtime 使用原地初始化函数；重配置/自动恢复不再经整对象按值构造。整改后的 release 与调用链待新结果；该中间包未刷写。

## 本轮失败与修复

1. H753 首次 check：参数管理器引用旧 `runtime::Params`；迁到产品参数入口后 Clippy 通过。
2. 新用例首次作为 integration target 启动，Cargo 同时构建 host 固件 bin，触发 MCU-only HAL/重复 panic 错误；移入现有 lib 测试入口。
3. Runtime 用例首轮 2 通过/4 失败：固定 3–6 s 后就假定默认倾角对准完成，实际 `fault=None`、tilt 未对准。监督测试改为显式已知初始姿态先验 0.03 rad；产品默认值与 PX4 默认先验回放不改。修正后新增 Fatal/产品复位用例合计 8 项通过。
4. Python 3.10 直接运行构建工具测试缺少 `tomllib`；复用已安装 `tomli` 兼容导入后 2 项通过。
5. ULog 注册表检查发现新增两项运行时计数后，`rust_estimator_runtime` 的长度仍为 44 B；字段定义实际为 52 B。已同步长度，复验待结果。

## 板载配置故障诊断（2026-11-05）

首次普通包 `335f484ffd467691` 板载 65 s 显示 EKF 未激活（`config_errors=1`、`last_config_fault=1`、`config_stage=1`）。参数 dump 核对 159 项全过，排除参数越界。加入 `config_stage` 阶段码（IDLE=0…HEALTH=8）与主机复现、板载 start/stop 循环：

- 阶段码随周期随机为 4/7/2/4 等，且读到的参数每次一致 —— **非参数值驱动，是执行期间的状态问题**。
- 定位到配置后的健康检测阶段（`health()`），该处读到的 fault 是**瞬态**：间隔 1 ms 重配一次（最多 3 次）可消解，8/8 周期 `config_errors=0` 全绿。
- 间隔 10 ms 时 8 周期出现 2 次真实失败（主实例 IMU_INIT、独立实例 VALIDATE），证明故障与重试时序耦合、非纯随机。
- 结论：启动早期存在一次性瞬态健康故障，1 ms 自动重试是当前防御；**根因（疑为参数多源头并发访问：`ParameterManager.values` 用 `NoopRawMutex` 无锁 `RefCell`，被 `!ekf2`、`PARAM_SET`、DroneCAN、`main` 共享）留待分支排查**。

诊断固件 source=`282121108641b649`（1 ms 间隔，8/8 全绿，`config_errors=0`）。探针证据：`target/ekf2-config/{6c9956e0,6d31753e,9513e9c8,f509ec6b,e26e26fa,282121108}-run*`。

## 复现命令

```powershell
cargo test --locked --target x86_64-pc-windows-msvc --lib estimator::runtime_tests
cargo clippy --locked --target x86_64-pc-windows-msvc --lib --example ekf2_input --example ekf2_standard -- -D warnings
cargo check --locked --target thumbv7em-none-eabihf --features estimator-mahony --bin v6x-rust-led
py -3 tools/check_ekf2_refactor.py --rust target/x86_64-pc-windows-msvc/release/examples/ekf2_input.exe
py -3 tools/check_ekf2_standard.py --case no-gps --rust target/x86_64-pc-windows-msvc/release/examples/ekf2_standard.exe --output target/ekf2-refactor-standard/no-gps
py -3 tools/check_ekf2_standard.py --case gnss --rust target/x86_64-pc-windows-msvc/release/examples/ekf2_standard.exe --output target/ekf2-refactor-standard/gnss
& .\tools\build_firmware.ps1
```

## 剩余项

- 原地初始化后的固件调用链与栈检查、最终差异/lint、证据整理和提交推送。
- 新固件板载启停/恢复、1 kHz release/年龄、CPU/栈、IMU 时基及分实例计时。DWT 原始时间包含抢占，完整 IRQ 扣除尚未实现，不称为纯算法 WCET。
- mag/range 的实际驱动、设备与标定仍待硬件条件；原 E7/M1 依赖不由本轮 host 结果放行。
- 阶段 4 算法/硬件优化尚未开始。当前保留已有 Joseph 更新与数学黄金结果。
