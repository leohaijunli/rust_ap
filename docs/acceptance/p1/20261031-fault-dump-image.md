# P1：启动故障转储镜像与采集

日期：2026-10-31。**本文件记录诊断镜像的设计和历史产物；工程师后续已成功采集实际故障记录。** 新P1的PANIC已定位到时钟断言，SCUEN最小修复通过离线回归，见[原始证据与修复](20261031-startup-assertion.md)。agent没有刷写、打开实际串口、复位或注入故障。

**后续状态**：用户通过新版monitor取得6456 B、4份完整报告及有效PANIC，Dump source=`4f8bbf296699e6f2`。SCUEN修复后的普通P1 source=`4cba95aa54cd899a`已恢复基本连接；该轮重建Dump为`15432408177fb110`。最新普通/Dump身份、共享备份域保持行为及工程师待验项见[备份域修复](20261031-backup-domain.md)，本页保留历史设计与采集身份。

## 目的与边界

诊断镜像（`--features fault-dump`）只做三件事：

1. 在任何 HAL/时钟初始化之前，读取 `RCC_RSR`、`RTC_BKP0`、`RTC_BKP16..31` 及入口 RCC/PWR/FLASH/SYSCFG 寄存器，保存在 RAM。
2. 使用**已知可用的 P0 时钟树**（HSI 64 MHz 核、HSE 16 MHz、PLL3Q 48 MHz USB）启动，不加载 P1 的 PLL1 480 MHz 路径，也不执行 `platform::init()` 的芯片版本/时钟断言。
3. 通过 USB CDC（`1209:0001`，product=`V6X Rust Dump`，serial=`V6X-RUST-DUMP`）循环输出转储文本；红色 LED 以 1 Hz 心跳常亮/闪烁，绿色/蓝色关闭，加热输出关闭。

安全边界：

- 不写故障记录，不覆盖 `BKP16..31`；只把入口时捕获的 32 个 BKP 字原样恢复，使 app 故障复位后的 bootloader 请求语义保持原样。
- 不修改 bootloader、option bytes、应用边界或 USB 身份文件；不访问 FRAM/SD/CAN/IMU。
- 捕获后立即恢复备份域内容；即使 HAL 的 backup-domain reset 清除了 BKP，也会在初始化后写回。
- 支持文本命令 `!bootloader`：仅写 `BKP0=0xb007b007` 并系统复位，用于回到 PX4 bootloader。该命令不是 MAVLink，不影响正常 P1 应用。
- 以Dump专用USB身份运行，默认`upload`/`upload-dump`不匹配该身份；先发送`!bootloader`并确认原BL枚举，再运行上传入口。断电本身不保证BL持续停留。

## 历史产物（实际采集使用，source=4f8bbf296699e6f2）

| 项 | 值 |
| --- | --- |
| 构建命令 | `bash tools/firmware.sh build-dump` |
| 目标目录 | `target/p1-fault-dump/` |
| source id | `4f8bbf296699e6f2` |
| features | `fault-dump` |
| raw / padded | 34788 / 34816 B |
| static RAM | 10996 B |
| `.px4` 归档 | `target/p1-startup-20261031/dump-4f8bbf296699e6f2/v6x-rust-led.px4` |
| `.px4` SHA-256 | `90d074c2876ba1be7e41a750255b6eae39fa7bb20dd8c931c0da70f7bb402095` |
| `.bin` SHA-256 | `a91493b7a109025f95322e617ca9f153d0043b24b5798e4207c687daf8dba882` |
| `.elf` SHA-256 | `d07213499f0a547708a25d06a9de2c286fb401fd421950c9a55314cc87c86fd1` |

离线检查：dump feature Clippy `-D warnings`、`cargo fmt --check`、打包/布局/全部向量检查、12 项 Rust host 测试与 30 项 host Python 测试通过。**这些不等于上板验证。**

> 新增 `src/fault_record.rs` 的 `file_hash` fixture 测试会改变 `src/` 内容，因此 source id 与早先记录的 `6a09281b26c6f06b` 不同；**早先的 dump 产物不得复用**。每次 `upload-dump` 都会按当前树重建，记录产物 hash 时以 uploader 日志打印的 `source=` 为准。

## 当前已运行 Dump：直接采集

前提：拆桨/台架安全，关闭 QGC 和任何占用 `/dev/ttyACM*` 的程序。

```sh
bash tools/firmware.sh monitor 10
```

脚本自动识别 Dump、保存原始字节至 `target/fault-dumps/<时间戳>.txt` 并打印故障解码。
`valid=NO` 表示没有有效的保留记录；可能是未写入、被清除、损坏或备份供电丢失，不能单独确定故障发生点。
本次已得到有效`afaa140f499ff958`记录。源码显示PX4/P1的备份域互操作存在清空记录的路径，但更早记录是否曾丢失仍未实测证实。

## 历史诊断方案：普通 P1 → Dump

以下为**修复前**双刷诊断顺序，保留用于理解故障记录来源。当前`upload`已重建修复后的普通包，不再按下面的失败预期判定；从Dump返回普通包的流程见[工程师复验步骤](20261031-startup-assertion.md#5-工程师复验从当前-dump-返回普通-p1)，当前身份以[备份域修复](20261031-backup-domain.md)为准。

1. 确认板子处于 PX4 bootloader（`3185:0035`，product=`PX4 BL FMU v6X.x`）或官方 PX4 应用。
2. 当时重建并刷入失败普通 P1 `afaa140f499ff958`（工程师执行）：

   ```bash
   bash tools/firmware.sh upload
   ```

   预期：握手/擦除/写入/校验成功，`Rebooting to application`；随后仍无 Rust CDC/LED，并因 `BKP0` 请求停在 PX4 bootloader。**记录此时还在 BKP16..31；不要启动官方 PX4。**
3. 立刻刷入诊断镜像：

   ```bash
   bash tools/firmware.sh upload-dump
   ```

   预期：BL 写入 34816 B 镜像并校验成功；这不是 P1 修复，也不是官方 PX4 恢复。
4. 刷写后若出现 `usb-Rust_FMUv6X_V6X_Rust_Dump_V6X-RUST-DUMP-if00`，读取并解码：

   ```bash
   bash tools/firmware.sh monitor 10
   ```

   解码器会用 `file_hash` 直接映射到当前 `src/*.rs`（`file=src/platform.rs` 之类）；若报“no local src matches”，说明要用与记录 `source` 对应的源码归档，而不是当前工作树。
5. 保存两次 uploader 完整日志、原始串口文本、解码输出与内核枚举；`upload` 打印的 `source=` 用于绑定记录。**不要**把 dump 输出当作 P1 已修复。
6. 若第 2 步后板子完全无 USB（应用挂死而非复位，BL 未出现）：断电重启，并在约 5 s 的 BL 窗口内执行第 3 步（`BOOTLOADER_DELAY=5000`，`platforms/nuttx/src/bootloader/stm/stm32_common/main.c:643`）；不要反复盲刷。

## 判读树

- `retained fault: valid=YES, kind=PANIC`：`detail` 是 `src/*.rs` 行号，`file_hash` 是 FNV-1a 路径哈希，解码器会打印 `file=`。用同source的源码定位；本次已确认为失败包`src/platform.rs:91`的`assert!(clock_registers_ok())`，位于USB/LED初始化之前。
- `kind=HARD_FAULT`：`cfsr/hfsr` 与 `pc/lr/sp` 来自故障现场；按对应 `source` 的 ELF/源码定位。
- `kind=UNEXPECTED_EXCEPTION`：`detail` 是 IRQ 号。
- `kind=STACK_GUARD`：共享栈越界；`sp` 是检测点。
- `valid=NO` 且 BKP16..31 全零：说明这次失败没有写记录（挂死、复位前未进入故障路径，或记录又被备份域复位清掉）；结合 `BKP0`、`RCC_RSR` 与“第 2 步后是否还枚举到 BL”判断。
- `entry RCC` 为 bootloader 跳转后的 Dump 入口，`P0 RCC` 为 Dump 的 HAL 初始化后；不能据二者差异直接还原旧 P1 故障前的 PLL 配置。

## 恢复

- Dump内`!bootloader`写入BKP0请求后复位；原BL检测到请求会清除签名并`timeout=0`停留（PX4 `platforms/nuttx/src/bootloader/stm/stm32_common/main.c:671-685`）。若使用独立复位/重插入口，按本板已确认流程操作并实际核对BL枚举，不以断电保证持续停留。
- 需要回到已知良好 Rust 时，由工程师显式选择 `.p0-baseline/20261031/v6x-rust-led.px4`（SHA-256=`d74b3b80e7c35bf4354ea568c09631f2fad362570caff414516ee0fe48a3c02d`），不要用默认 `firmware.sh upload`。
- 本诊断镜像不是官方 PX4 恢复包，不能替代 P1-01/H-05 的资料补齐。

关联：[启动回归主记录](20261031-startup-regression.md)、[上传入口修复](20261031-upload-discovery.md)、[软件索引](README.md)。
