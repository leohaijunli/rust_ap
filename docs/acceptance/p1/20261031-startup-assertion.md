# P1：启动时钟断言定位与 SCUEN 最小修复

日期：2026-10-31。**实际PANIC已定位到USB启动前的时钟断言；错误的SCUEN要求已修复并通过离线回归。后续普通Rust CDC/MAVLink监看成功，用户确认QGC基本连接恢复。** 已有实测见[连接复验](20261031-qgc-retest.md)，最新固件身份与完整P1待验项见[备份域修复](20261031-backup-domain.md)。工程师执行设备操作；agent执行源码、机器码、构建与host回放及已有日志核对。

## 1. 实际故障与身份绑定

用户执行 `bash tools/firmware.sh monitor 10`，关键字段如下（完整原始文件见下表）：

```text
Device: /dev/ttyACM0 (V6X Rust Dump, 1209:0001, serial=V6X-RUST-DUMP)
Captured 6456 bytes
V6X fault dump: version=0.2.0 source=4f8bbf296699e6f2 complete=yes
retained fault: valid=YES kind=1 (PANIC)
  source=0xafaa140f499ff958 detail=91 file_hash=0x60652e12 file=src/platform.rs
  sp=0x2407fdf0 cfsr=0x00000000 hfsr=0x00000000
entry power: cr3=0x05000042 d3cr=0x0000e000 pwrcr=0x00000081 flash_acr=0x00000022
```

原始文件按字节复制入本目录，保留 CRLF：

| 证据 | 大小 | SHA-256 |
| --- | ---: | --- |
| [实际 Dump](20261031-startup-dump.txt) | 6456 B | `856ffbaf3cfb1049c852a9c33b62b2dec18b58071f9f21cae05b2ab143a17dda` |
| [monitor 诊断 JSON](20261031-startup-monitor.json) | 6651 B | `77e071070ea3cbd5d95e6b74437e00ba0bfe7badec1738efa80b0a99b0843d58` |

- 采集时间为 `20261031T145903.692446Z`，JSON 的 `outcome=ok`。文本含同一启动快照的 **4 份完整报告**，不是四次独立故障。
- 故障记录的存储/计算 CRC 均为 `0xd209f8cf`。PANIC 格式不记录异常 PC/LR，二者为零符合该记录格式。
- 修改前已核验失败 build report 中的 **17 个源码输入、4 个构建产物 SHA-256**。匹配源码的 `src/platform.rs:91` 是 `assert!(clock_registers_ok())`；不能只靠路径哈希推断源码版本。
- 该断言在 `embassy_stm32::init()` 和 BKP 恢复之后，在 `application_main()` 创建 executor、启动 USB/任务之前。panic 路径写故障记录、请求原 PX4 bootloader 后复位，符合本次观察。
- 早先 uploader 日志中的 **134208 B** 包路径/hash 仍未知；本次绑定的是后来重建的 `afaa140f499ff958`，raw/padded 为 **134212/134240 B**。

失败源码、ELF/bin/px4/map、构建报告和完整反汇编在本机：
`target/p1-startup-20261031/pre-fix-afaa140f499ff958/`。
采集所用的旧 Dump 产物在 `target/p1-startup-20261031/dump-4f8bbf296699e6f2/`。
对应构建报告：[失败普通包](20261031-startup-before.build.json)、[实际使用的 Dump](20261031-startup-observed-dump.build.json)。

### 15:30 UTC 复采核对

用户随后提供`target/monitor-logs/20261031T153059.855122Z.json`。该JSON为`outcome=ok`、`selected_mode=dump`，USB身份仍为`V6X-RUST-DUMP`；原始报告头部source仍是旧Dump **`4f8bbf296699e6f2`**。

| 证据 | 大小 | SHA-256 |
| --- | ---: | --- |
| [复采原始Dump](20261031-startup-repeat-dump.txt) | 6456 B | `856ffbaf3cfb1049c852a9c33b62b2dec18b58071f9f21cae05b2ab143a17dda` |
| [复采诊断JSON](20261031-startup-repeat-monitor.json) | 6651 B | `2be2dc5a6fe9d14b1e9a9bdccad39797b53ea2de2afa65153b18b5febbcc0e93` |

4份报告均完整、故障CRC有效；原始6456 B与14:59采集**逐字节相同**，保留的PANIC仍为`afaa140f499ff958 / src/platform.rs:91 / CRC=d209f8cf`。记录格式没有故障序号或时间戳，因此不能仅据内容相同推断期间是否发生过重启。这次采集确认运行的是旧文本诊断镜像，尚未提供修复后普通`4cba95aa54cd899a`的运行结果。下一步按第5节切回普通P1并复验；Dump没有MAVLink。

## 2. 确认的错误检查

Dump 入口和 P0 HAL 初始化后均读到 **`PWR_CR3=0x05000042`**：

| 位 | 实测值 | 含义 |
| --- | ---: | --- |
| LDOEN，bit 1 | 1 | LDO 启用 |
| BYPASS，bit 0 | 0 | 未旁路稳压器 |
| SCUEN，bit 2 | 0 | 供电配置更新控制位；不是电压就绪标志 |

固定来源及 `文件:行` 见[硬件台账](../../fmuv6x-hardware.md#启动断言实测与供电位核对2026-10-31)：PX4/NuttX 与 PAC 均把 SCUEN 定义为 **Supply configuration update enable**；ST CMSIS 的 H753 定义一致。HAL 请求写 SCUEN/LDO，并注明 CR3 低字节在 POR 后的 write-once 限制，以及它无法知道 bootloader 已做过哪些写入。**请求置位不等于能保证该位读回为 1**。本次没有实测证明 SCUEN 自动清零，也未确定本板读零的全部硬件机制。

最小固件改动：删除 `&& supply.scuen()`，补充原因注释。17 个固件构建输入中仅 `src/platform.rs` 改变。仍检查 LDOEN、BYPASS、VOS/VOSRDY/ACTVOSRDY/ODEN、PLL/总线/USB、flash 和 TIM2；本次固件修改未调整 HAL 供电写入或备份域策略。

两个 ELF 中 `clock_registers_ok()` 均位于 `0x0802681e`：

| 地址 | 失败普通包 | 修复普通包 |
| --- | --- | --- |
| `0x080268fc` | `and r1, lr, #7` | `and r1, lr, #3` |
| `0x08026900` | `cmp r1, #6` | `cmp r1, #2` |

`lr` 在该处保存 CR3。旧条件要求 `(CR3 & 7) == 6`，实测低三位为 2，必然拒绝；新条件为 `(CR3 & 3) == 2`，只要求 LDOEN=1、BYPASS=0。源码差异及完整函数反汇编见[机器码审查](20261031-startup-code-review.log)。

这确认了一个足以拒绝本板实测供电状态的启动检查缺陷。**Dump 入口经过 bootloader，不能还原全部故障前 P1 寄存器**；本次断言命中不证明实际 P1 的其他每个检查都曾通过。

## 3. 离线验证

命令：`TMPDIR=/tmp/opencode bash tools/firmware.sh check`。
完整输出：[startup-fix-software.log](20261031-startup-fix-software.log)。脚本启用 `set -euo pipefail`，完成到最后的 Dump 构建报告。

| 检查 | 结果 |
| --- | --- |
| Rust fmt | 通过 |
| 普通 / fault-injection / fault-dump target Clippy；host 库 Clippy | 均通过，`-D warnings` |
| Rust host 测试 | 12 passed |
| Python 工具测试 | 59 passed |
| 普通与 Dump H753 release 构建、打包、ELF/bin/向量/布局/source 校验 | 通过 |
| 修复前、修复后真实 ELF 的 Cortex-M7 回放 | 各 14 项通过 |

回放工具为 `tools/replay_platform_check.py`，使用 **Unicorn 2.1.4** 执行实际已编译函数。ELF/bin 先与构建报告核验 SHA-256；CR3 从真实 Dump 读取，其余寄存器由脚本构造预期 P1 配置。只执行这一谓词，不执行 HAL、不模拟外设动态行为或整板启动，也不测硬件时序。

| 回放条件 | 旧 ELF | 修复 ELF |
| --- | --- | --- |
| 实测 CR3=`0x05000042`，其余为预期 P1 配置 | false | true |
| 同一配置，仅把 SCUEN 置 1 | true | true |
| 12 项错误配置 | 全部 false | 全部 false |

12 项为 LDO 关闭、旁路启用、电压未就绪、实际电压未就绪、overdrive 关闭、错误 SYSCLK、PLL1/PLL3 未锁定、错误 USB 时钟、flash 延迟不足、错误 TIM2 分频/周期。错误用例以 SCUEN=1 为底，确保旧谓词不会全在同一个 SCUEN 条件提前退出。

原始输出：[修复前](20261031-clock-replay-before.log)、[修复后](20261031-clock-replay-after.log)。复现（host-only，可选依赖；不属于普通构建依赖）：

```sh
.venv/bin/python -m pip install --disable-pip-version-check --no-cache-dir \
  --target /tmp/opencode/v6x-platform-replay-deps unicorn==2.1.4
PYTHONPATH=/tmp/opencode/v6x-platform-replay-deps .venv/bin/python \
  tools/replay_platform_check.py \
  target/p1-startup-20261031/pre-fix-afaa140f499ff958/release \
  --dump docs/acceptance/p1/20261031-startup-dump.txt --legacy-scuen
PYTHONPATH=/tmp/opencode/v6x-platform-replay-deps .venv/bin/python \
  tools/replay_platform_check.py target/thumbv7em-none-eabihf/release \
  --dump docs/acceptance/p1/20261031-startup-dump.txt
```

Unicorn 固定 commit、wheel 来源及许可见[来源台账](../../provenance.md)。旧 ELF 的回放依赖本机归档；`target/` 被 Git 忽略，交接时须另行保存。

## 4. 本轮SCUEN修复产物（历史）

Git HEAD=`5daeca30fcd116203164f44128239e7dd8a884ef`，包含未提交工作树；Rust 1.99.1 / LLVM 23.1.1、`thumbv7em-none-eabihf`。源码身份及逐文件 SHA 以构建报告为准。

| 产物 | source ID | raw / padded flash | static RAM | `.px4` SHA-256 |
| --- | --- | ---: | ---: | --- |
| 普通 P1，features=[] | `4cba95aa54cd899a` | 134212 / 134240 B | 17796 B | `51fdd8a4189f519c014d3e78554f8eb422813656332a4f4b751626cebcb9739a` |
| 重建 Dump | `15432408177fb110` | 34788 / 34816 B | 10996 B | `4803dd65b8ac3e1bca1a7af79786b603c97b203af0d8a00d33b0eb868d40cb9e` |

- 该轮普通包：`target/p1-startup-20261031/post-fix-4cba95aa54cd899a/release/v6x-rust-led.px4`；[构建报告](20261031-startup-after.build.json)。
- 该轮重建 Dump：`target/p1-startup-20261031/dump-15432408177fb110/v6x-rust-led.px4`；[构建报告](20261031-startup-rebuilt-dump.build.json)。其 source ID 覆盖整个 `src/`，所以本次也变化；板上采集用的仍是旧 Dump `4f8bbf296699e6f2`。
- 修复源码/工具/产物快照：`target/p1-startup-20261031/post-fix-4cba95aa54cd899a/`；重建 Dump 另存 `target/p1-startup-20261031/dump-15432408177fb110/`。
- 应用起点仍为 `0x08020000`、board ID=53；P0 恢复包 SHA-256 重核为 `d74b3b80e7c35bf4354ea568c09631f2fad362570caff414516ee0fe48a3c02d`。

收尾时纠正了回放工具的一行来源注释（计时器头文件为`stm32_tim.h`），Python AST相同；已刷新普通/Dump构建报告与`SHA256SUMS`并同步归档。固件source及ELF/bin/px4/map的hash均保持上述身份。最终归档、文档链接与差异核验见[一致性检查](20261031-startup-final-check.log)。

## 5. 工程师复验：从当前 Dump 返回普通 P1

以下保留从Dump返回普通P1的交接流程。后续用户已确认QGC连接，已有原BL及普通CDC/MAVLink成功日志；当前新包精确source、schema-2及完整验收按[备份域交接](20261031-backup-domain.md#5-工程师待验与关闭条件)执行。下面协议检查的source已同步为最新普通包。

按[上板操作单](bring-up.md)在拆桨、默认锁定/输出关闭的条件下执行，关闭 QGC、monitor 和其他串口读取器。以下会发送重启命令并上传应用，由工程师执行。

1. 交接起点为 `V6X-RUST-DUMP`。向 **Dump** 的串口发送完整一行 `!bootloader`：

   ```sh
   .venv/bin/python - <<'PY'
   import serial
   import time

   port = '/dev/serial/by-id/usb-Rust_FMUv6X_V6X_Rust_Dump_V6X-RUST-DUMP-if00'
   with serial.Serial(port, 115200, timeout=1, write_timeout=2) as device:
       device.write(b'\n!bootloader\n')
       time.sleep(0.2)
   PY
   bash tools/firmware.sh ports
   ```

   Dump 命令立即复位，没有 `OK` 回显。等待重新枚举，`ports`显示`PX4 BL FMU v6X.x / 3185:0035`后再执行下一步；若暂时为空，稍后重查。默认uploader不匹配Dump，需先完成此步。

2. 上传普通 P1，保存完整 uploader 输出及打印的 source/hash：

   ```sh
   bash tools/firmware.sh upload
   bash tools/firmware.sh monitor 10
   .venv/bin/python tools/check_mavlink.py --source-id a946a529360eac83
   ```

   预期上传/CRC 成功，枚举 `V6X Rust CDC / V6X-RUST-001`，收到心跳及正确版本/source，完整 MAVLink 检查通过。如果构建 source 改变，先按新报告核对实际源码；不能沿用本页身份。

3. 退出脚本后，用 QGC 的串口连接选择普通 Rust CDC、115200；检查 system 1 的 HEARTBEAT、SYS_STATUS、SYSTEM_TIME 和 AUTOPILOT_VERSION。随后按操作单采集 `!platform`/时基/调度/CPU/栈并验证恢复路径。

`!fault` 可能仍显示保留的旧 `source=afaa140f499ff958`，红色故障灯也可能亮；它不等于新包再次 panic。先保存记录并区分 source 与当前 `!platform` 身份。若新包仍回 bootloader，保存上传输出、monitor 诊断 JSON 和新故障记录，再按实际记录定位。

## 6. 尚待完成

- [x] 实际采集、CRC/源码身份绑定、最小修复、离线构建与机器码回放。
- [x] 普通Rust CDC/MAVLink监看成功，用户确认QGC基本连接恢复。
- [ ] 板上精确source与完整协议、物理480 MHz/1 MHz时基、调度和恢复路径复验。
- [x] 后续备份域软件修复：同时匹配 `LsConfig` 与 `ClockMux.rtcsel`，schema-2诊断及三版各132项回放通过，见[证据](20261031-backup-domain.md)；板上保持待验。本次有效PANIC未实测证明更早记录曾被清空。
- [ ] P1-01/H-05 完整恢复资料；P1-04 签名验收。此前官方恢复报告和本轮离线通过不替代这些记录。
