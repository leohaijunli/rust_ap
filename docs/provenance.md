# 来源与复用台账

日期：2026-11-02。用户允许公开发布与 GPL 来源复用；最终板上飞控应用为纯 Rust + Embassy。下面区分实际工程依赖、评估候选和 host 参考，固定版本不等于已接入固件。

## 1. 固定版本与许可证据

| 来源 / URL | 固定 commit 或版本 | 查到的许可 / 当前用途 |
| --- | --- | --- |
| [Holsatus](https://github.com/holsatus/holsatus-flight) | `45dd8c479e0c73b36046e33fb90bbf5f43e1cbdc` | Apache-2.0；评估、未取用应用源码 |
| [Cybflight](https://github.com/cybird-robotics/cybflight) | `806af5648c336e7af81abefea31052db783387f5` | 顶层 Apache-2.0 + NOTICE；A3 算法来源，P0 core 探针 git 依赖 |
| [Embassy](https://github.com/embassy-rs/embassy) | stm32 0.5.0、executor 0.9.1、time 0.5.1、USB 0.5.1 | MIT OR Apache-2.0；当前固件 Rust HAL/runtime |
| [stm32-metapac](https://github.com/embassy-rs/stm32-data) | 19.0.0，Cargo.lock固定crate校验值 | MIT OR Apache-2.0；H753实际使用`rcc_h7rm0433.rs`/`rtc_v2_h7.rs`；备份域地址/枚举参考 |
| [rust-mavlink](https://github.com/mavlink/rust-mavlink) | 0.17.1 | MIT/Apache-2.0；当前 MAVLink |
| cortex-m / cortex-m-rt | 0.7.7 / **lock解析0.7.7**（manifest兼容要求0.7.5） | MIT OR Apache-2.0；异常/复位、set-vtor/paint-stack，实际paint起点为 `_stack_end` |
| static_cell | 2.1.1 | MIT OR Apache-2.0；P1静态raw executor初始化，Cargo.lock已同步 |
| [PX4 板级参考](https://github.com/PX4/PX4-Autopilot) | `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` | BSD-3-Clause 顶层；板级/bootloader 源码参考 |
| PX4 uploader 来源树 | `a2be9197b57b2d065e4c47ca7fc5b7565ee07282` | host 上传工具，使用其文件许可；与板级参考分开固定 |
| PX4 标准消息 | 同一 v1.17.0 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` | BSD-3-Clause；`msg/{SensorAccel,SensorGyro,ActuatorArmed}.msg` 字段与 Rust 数据边界/ULog，副本及 LICENSE 在 `tools/px4_msgs/` |
| [NuttX](https://github.com/PX4/NuttX) | `fb2fadf6f599c1406f052db013efd00a2518e72c` | Apache-2.0 + NOTICE/文件级来源；时钟/IRQ 配置参考 |
| PX4 uploader工作树内NuttX | `9304289d9f2b304c6835c9d39c131648c5f804c2` | 本轮启动断言/PWR/RCC/内存映射及RTC/LSE修订差异参考；保留其文件级来源，不与v1.17子模块混用 |
| [ST CMSIS H7](https://github.com/STMicroelectronics/cmsis-device-h7) | v1.10.4，`faccfec37f82f7a1319c21638111b0f7335de7fe` | Apache-2.0（`LICENSE.md`）；`Include/stm32h753xx.h` Copyright (c) 2019 STMicroelectronics；仅核对SCUEN定义，未拷入/链接C头文件 |
| [Unicorn](https://github.com/unicorn-engine/unicorn) | 2.1.4，`8028ec436f2d9376525352dd38ed9ed6b9f6be10` | 可选host CPU回放依赖；核心`COPYING`为GPLv2，Python发行metadata标BSD、随包说明同时注明核心GPLv2；没有引入板上依赖或修改上游 |
| [DroneCAN DSDL](https://github.com/dronecan/DSDL) | `b4653c7abc3c47cb31b16efa24ea755232774756` | MIT；本轮固定协议定义 |
| [umi-eng/dronecan-rust](https://github.com/umi-eng/dronecan-rust) | `0f5ffc906aa4527e57b8db343a7f133874eece27` | MPL-2.0；host 缺陷探针依赖，未作为合格板上栈采用 |
| [David-OConnor/dronecan](https://github.com/David-OConnor/dronecan) | `06ee0c35d204b46dcac5152ef299b4e507a8b9b2` | 本轮未找到许可证文件；仅调研，取用前须查明授权 |
| [pydronecan](https://github.com/dronecan/pydronecan) | `092d09cb9e6b115e262efe4cb42f7b65e6054f26`，1.0.27 | MIT；host 独立参考向量 |
| [dronecan_dsdlc](https://github.com/dronecan/dronecan_dsdlc) | `431170fa4bfe2212b516b8f33bdc796267907f1c` | LGPLv3 工具、MIT 生成输出例外；C host 对拍候选，尚未运行 |
| PX4 内置 pydronecan / DSDL | `19fdf2e5b383243ccdb1094edae0603cf11469e8` / `993be80a62ec957c01fb41115b83663959a49f46` | 旧 MIT 参考；不与本轮生成输入混用 |
| [AM32](https://github.com/am32-firmware/AM32) | `86cf1e55e56c7e7e622d01ba6855905504f15105` | GPLv3；外接 ESC 行为参考，未构建/刷写 ESC |
| [embedded-fatfs](https://github.com/MabezDev/embedded-fatfs) | `518528cc111fcf65c48abbdeb80735a38eada112` | MIT；PL-a 板上 FAT、块适配与 MBR/superfloppy，lock 与根 patch 固定 |
| [pyulog](https://github.com/PX4/pyulog) | 1.3.0 | BSD-3-Clause；host 独立解析完整/截断 ULog，不进入板上应用 |
| [LibDriver INA219](https://github.com/libdriver/ina219) | `266a1ee32a5f70e4e5d47591ce28c4e1c387857c`（1.0.7） | MIT；寄存器/大小端、shunt/bus/current 缩放、标定与单次转换时序参考；[许可](licenses/libdriver-ina219.txt) |

许可文件 hash、origin、入口工作树和子模块清单分别见：

- [upstreams-entry.json](acceptance/p0/upstreams-entry.json)、[upstream-licenses.json](acceptance/p0/upstream-licenses.json)
- [protocol-upstreams.json](acceptance/p0/protocol-upstreams.json)、[generator-upstreams.json](acceptance/p0/generator-upstreams.json)、[esc-upstream.json](acceptance/p0/esc-upstream.json)
- [dependency-inventory.json](acceptance/p0/dependency-inventory.json)：当前工程/Cybflight 解析包及逐文件许可 hash；Cargo.lock 副本在 [locks](acceptance/p0/locks/)

顶层 LICENSE 不是覆盖所有来源注释的结论。Cybflight 的 `indi/controller.rs`、`indi/linearization.rs` 标注移植自 indiflight；`indi/rpm_notch.rs` 标注来自 Betaflight。曾查到 indiflight commit `15b762626a35be4be4c9af92d138072ed36911f3` 的 GPLv3 声明，但尚未证明它就是原移植版本。A3 不因 GPL 排除复用；实际取用这些部分时继续确认原始来源/适用许可并保留说明。M1 先使用 Mahony，不要求提前冻结远期 INDI 的实现方式。

## 2. 本轮新增内容

| 路径 | 来源 / 修改性质 |
| --- | --- |
| `tools/p0/core-abi/` | 本项目编译探针，依赖固定 Cybflight；只导出类型大小/对齐 |
| `tools/p0/dronecan-probe/` | 本项目 host 诊断，调用固定 umi 栈验证异常输入行为 |
| `tools/p0/reference_vectors.py` | 本项目生成脚本，调用固定 MIT pydronecan/DSDL；输出注明生成 pins 与 DSDL hash |
| `tools/p0/budgets.py` | 本项目预算计算；M1 与历史探索 profile 均为提案 |
| `docs/acceptance/p0/` | 原始命令、输出、清单与生成数据；不是上板验收结果 |
| 本组文档 / plan / TODO | 依据固定源码、用户决定与厂商资料整理；厂商规格与本板实测单列 |
| `src/platform.rs`、`src/main.rs` | 本项目P1平台/调度实现；对照固定PX4板级与Embassy HAL时钟/供电API，非拷入C/C++驱动 |
| `src/platform.rs` 启动断言修复 | 根据有效PANIC、实测CR3与固定PX4/PAC/ST定义删除错误SCUEN条件；保留供电/电压就绪检查，见[实际证据](acceptance/p1/20261031-startup-assertion.md) |
| `tools/replay_platform_check.py` | 本项目host回放脚本，复用本项目ELF读取/故障解码工具并调用Unicorn；实际执行ELF中的谓词，CR3取实测、其余寄存器为预期模型；未复制模拟器实现 |
| `src/backup_domain.rs`、`src/platform.rs`、`src/fault_dump.rs` | 本项目共享初始化边界；对照固定HAL与H753实际PAC匹配RTC/LSE及生成mux，比较BDCR/BKP并提供schema-2诊断；未修改HAL/PAC或拷入C代码 |
| `tools/replay_backup_domain.py` | 本项目Unicorn host工具；执行真实startup/wrapper/HAL及保持谓词，合成MMIO和RTC/BKP哨兵用于发现复位/驱动编码回归；不作硬件或RTC走时证据 |
| `tools/backup_domain.py`、转储解码/采集门及测试 | 本项目schema-2保持证据解码；旧报告保持状态为未知，缺失/失败不能通过采集门；新增5项host回归 |
| `src/fault.rs`、`src/core/fault_record.rs` | 本项目BKP线格式/IEEE CRC32/故障路径与host测试；裸入口是Rust内联架构汇编，帧布局参考固定NuttX `arm_exception.S:118-137`，未链接NuttX处理器 |
| `src/diagnostics.rs`、`src/core/metrics.rs` | 本项目绝对周期统计、masked-WFE CPU/共享栈测量；参考Embassy执行器唤醒与TIM2时基API、cortex-m-rt paint-stack约定 |
| `build.rs`、`tools/build_report.py`、`tools/p1_capture.py`及测试 | 本项目source/feature ID、ELF/布局/资源/hash检查、USB混合流/时基不确定性分析；host使用已有pyserial3.5/pymavlink2.4.49，不引入板上C/C++协议运行库 |
| `.github/workflows/p1.yml`、`docs/acceptance/p1/` | 本项目软件检查及工程师授权的HIL/恢复证据；P1必验项通过，远程CI状态未查询 |
| `src/storage/fram.rs`、`src/core/parameters.rs`、`src/core/parameter_store.rs`、`src/storage/parameter_manager.rs`、`tools/fram_backup.py`、`tools/p3a_parameters.py` | 本项目 P3a 实现：按 FM25V02A 厂商规格（`001-90865 Rev. *I`）和固定 PX4 `ramtron.c` 的 8 位/MODE3/两字节地址约定独立编写，未拷入驱动代码 |
| `src/core/dronecan/`、`src/drivers/dronecan_can.rs` | 本项目 P4b-a 独立实现；协议字段/签名对照固定 MIT DSDL、P0 pydronecan 向量及 MIT libcanard 状态机，未复制候选 Rust 栈；板级适配使用现有 Embassy MIT/Apache-2.0 API |
| `src/core/ulog.rs`、`src/core/log_queue.rs`、`src/storage/sd_logger.rs`、`examples/ulog_fixture.rs`、`tools/check_ulog.py` | 本项目 PL-a 实现；ULog 线格式对照固定 PX4 v1.17 文档，FAT 使用上述固定 MIT 栈，pyulog 只作 host 互操作检查 |
| `src/core/logger.rs`、`src/comms/usb_console.rs`、`src/comms/mavlink_core.rs` | 本项目 LoggerControl 与 MAVLink Console；使用 common.xml `SERIAL_CONTROL`/`MAV_CMD_LOGGING_START/STOP`，通过固定 rust-mavlink 生成类型接入，不实现任意 shell 或引入 OS/FFI |
| `src/core/icm42688.rs`、`src/core/config.rs`、`src/drivers/imu.rs` | 本项目 P4 纯 Rust/Embassy 实现；寄存器地址、20 B high-resolution FIFO、复位/配置/水位/周期回读流程对照固定 PX4 `d6f12ad…` 的 `src/drivers/imu/invensense/icm42688p/{InvenSense_ICM42688P_registers.hpp,ICM42688P.cpp,ICM42688P.hpp}`（BSD-3-Clause）。本项目采用 8 kHz ODR、YAW270/FLU、独立 fan-out 与 Embassy API，未链接或拷入 C++ 驱动 |
| `src/core/px4_messages.rs`、`src/core/ulog.rs`、`tools/px4_msgs/`、fixture/检查工具 | 标准字段适配上述固定 `.msg`；本项目显式编码和 FRD 转换，保留 BSD-3-Clause 许可。DeviceId 对照同 commit 的 `Device.hpp`、`nuttx/SPI.cpp`、`drv_sensor.h`、FMUv6X `spi.cpp` 与 `spi_hw_description.h`；裁剪判据参考 `PX4Accelerometer.cpp` / `PX4Gyroscope.cpp` 的单样本 99.9% 满量程规则，未移植 FIFO 平均或 uORB 运行时 |
| `tools/enter_bootloader.py`、`tools/test_enter_bootloader.py`、`tools/firmware.sh` | 本项目 host 上传入口；复用应用既有 `!bootloader` 协议和原 PX4 uploader，不修改或替换 bootloader |
| `tools/setup_ubuntu.sh`、`.run/Setup Ubuntu.run.xml` | 本项目 Ubuntu 工具入口；下载上述固定 uploader 的 `Tools/px4_uploader.py` 到忽略的 `.venv/`，SHA-256=`f05929e1035e693005677e088e4e7085503b2e8ab3d7c8785f7bd84cd48880f8`；保留上游脚本许可，未修改上游代码 |
| `src/core/ina219.rs`、`src/drivers/ina219.rs` 与遥测/诊断接线 | 本项目 SEN0291 纯 Rust 实现，参考固定 LibDriver `src/driver_ina219.{c,h}` / example 与 TI SBOS448G（23–24 页 CNVR/OVF/FS0）；采用工程师确认的 10 mΩ/8 A、±80 mV、CAL=16776（FS0 清零）、单次转换和同次 V×I。工程师提供的根目录 `ina219/` C++ 文件为参考，不链接 uORB/NuttX/C++ |
| `.opencode/skills/`、`AGENTS.md` | 工程师要求安装并应用的 clean-code / code-simplifier 项目适配；固定来源、改动与许可见[技能说明](ai-coding-skills.md) |

主工程没有拷入候选协议实现或引入 C/C++ FFI；P4b-a 为依据固定公开协议和参考向量编写的纯 Rust 实现。Python、参考 C 生成器只在 host 使用；原 PX4 bootloader 按恢复要求保留，外接 ESC/H-RTK 的厂商固件不属于本项目飞控应用。

启动回放的Unicorn通过PyPI安装到 `/tmp/opencode/v6x-platform-replay-deps/`，使用时显式设置 `PYTHONPATH`；普通固件构建/检查不依赖它。版本固定为 `unicorn==2.1.4`；PyPI公布的Linux x86-64 wheel为 `unicorn-2.1.4-cp37-abi3-manylinux_2_17_x86_64.manylinux2014_x86_64.whl`，16436886 B，SHA-256=`9d6e6dea140560de4ebd8446661f7ef84a357d428c14a3ef09dacd306ec8c239`。该hash是发行元数据，不是已归档wheel的本地校验结果；本项目不打包分发该wheel。固定版本来源：[PyPI元数据](https://pypi.org/pypi/unicorn/2.1.4/json)、[核心许可](https://github.com/unicorn-engine/unicorn/blob/8028ec436f2d9376525352dd38ed9ed6b9f6be10/COPYING)。

本轮备份域回放复用同一Unicorn安装；固定HAL/PAC、PX4/NuttX文件与生成mux的本地SHA-256见[输入清单](acceptance/p1/20261031-backup-domain/upstream-inputs.json)。H753选用PAC的中档枚举与通用H7不同，适配通过variant往返保持原位值；没有据枚举名重新设置本板驱动强度。

## 3. 后续取用规则

直接依赖写完整 `rev` 并提交锁文件；vendor/fork 时记录原 URL、commit、原路径、复制范围、版权头/NOTICE 与修改点，随源码携带对应许可。算法参考实现、代码移植和独立实现应如实登记。当前仓库自身的公开发布 LICENSE 尚未选择，应在发布时与实际取用范围一起落实；本轮不代为重新许可上游代码。

Holsatus 缺失 lockfile、David-OConnor 许可未查明和 INDI 原始移植版本不明都保持有名的待处理项；按用户要求将与 M1 无关的进一步调查留到实际取用阶段。
