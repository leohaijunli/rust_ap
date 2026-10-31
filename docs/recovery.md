# QGroundControl 恢复操作单

日期：2026-10-31。**首次新功能固件刷写前，由工程师在这块 Pixhawk 6X 上完成：已知良好 Rust → QGC 官方 PX4 → 已知良好 Rust。** 至少一次 bootloader 入口不使用 Rust 软件重启。

**当前结果**：P1已关闭。普通版协议/平台/备份域、测量、重启与故障保持/清除，以及QGC独立USB入口刷回PX4 v1.17.0均通过。21:28 UTC直接回装保存普通包，CRC=`B9FB636E`、source=`a946a529360eac83`、disarmed、fault_injection=false、故障valid=false；当前板上为普通Rust，见[HIL](acceptance/p1/20261031-hil.md)。下文完整流程作为参考，必验范围以[计划](../plan.md#5-p1-收口)为准。

**资料状态**：原恢复资料暂时不可得，原PX4参数/校准/FRAM没有备份；不把缺失备份等同于板上内容已丢失。本轮另存现场QGC缓存包和从GitHub下载、核验发布digest的官方v1.17.0包，两个hash不同，身份及使用状态见[板上记录](acceptance/p1/20261031-hil.md#qgc恢复留档)。旧记录和新演练不混用。

**历史故障与本轮授权**：新P1曾上传成功但无CDC；SCUEN与备份域修复后，本轮普通/注入板测通过。QGC早期主机锁失败由工程师清理`/tmp`后恢复，见[历史结果](acceptance/p1/20261031-qgc-lock.md)。工程师随后明确授权本轮第4步重启、回装和受控故障；实体操作及QGC观察由工程师配合，失败与补核全部保留。

**PX4→Rust入口**：默认上传模式已增加官方Auterion应用身份，见[入口修复](acceptance/p1/20261031-upload-discovery.md)。本轮实际通过固定uploader回装P0/普通包并核验CRC。PX4应用与BL共用`3185:0035`，应按产品字符串区分；已进入BL时显式选`*PX4_BL*`，避免上游uploader首个过期路径跳过后续端口。`firmware.sh upload`构建并上传当前普通版，不能用来回装保存的P0。

## 1. 固定边界

| 项 | 来源 / 状态 |
| --- | --- |
| bootloader 保留 | flash sector 0，前 128 KiB；保留已安装的原 PX4 bootloader |
| 应用 | `0x08020000`，1920 KiB，board ID 53 |
| 擦除 | 所审 PX4 源码擦应用 sector 1–15；应用中预留参数区不能保证跨上传保留 |
| bootloader USB | 本轮实测`3185:0035`、`PX4 BL FMU v6X.x`；协议v5，实际bootloader构建版本未知 |
| 当前 Rust USB | `1209:0001`、`V6X Rust CDC`；重新枚举可能改变 tty 编号 |
| 软件便利入口 | `!bootloader` 或 MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN、param1=3；RTC BKP0=`0xb007b007` |
| 独立入口 | 仅USB断开/重插；本次原BL枚举、QGC上传/校验及PX4启动已有记录 |

板级事实及 PX4 `文件:行` 见 [硬件台账](fmuv6x-hardware.md)。`BOOTLOADER_DELAY=5000` 是源码常量；VBUS/启动分支、已安装版本与 USB 枚举会影响实际入口，不能承诺每次上电都有固定 5 秒上传窗口。板头文件的 PWM 强制入口只是注释示例。

## 2. 演练前资料

按 [回填模板](acceptance/p1/recovery-template.md)记录；已有测试可补入[工程师实际报告](acceptance/p1/20261031-user-recovery-report.md)：

1. 上游 QGroundControl 版本、下载来源、操作系统；它必须是实际 QGC，不是 q-MAG/MagGCS。
2. QGC 为 **PX4 FMUv6X** 选择的官方发布镜像；保存实际文件、版本、来源 URL、SHA-256。若工程师选择自定义文件，仍须是匹配该板的官方 PX4 `.px4` 包，核对 board ID=53。
3. 已知良好 Rust 包：`.p0-baseline/20261031/v6x-rust-led.px4`，SHA-256 为 `d74b3b80e7c35bf4354ea568c09631f2fad362570caff414516ee0fe48a3c02d`。不要用未来新构建覆盖这份回归包。
4. 当前可取得的 PX4 参数导出、校准记录、版本与机型配置。没有原备份就明确标“缺失”，不能把恢复后的默认参数冒充原参数。
5. 本板 USB/bootloader 身份、协议版本/实际软件版本（能读到的分别记录）。协议 revision 不唯一确定 bootloader 构建版本。
6. 连接方式与供电：拆桨，电机动力断开；记录哪些外围/电源连接可能使主板持续供电。板号与实物型号保持一致。

本地文件检查（不访问设备）：

```sh
sha256sum .p0-baseline/20261031/v6x-rust-led.px4
sha256sum /实际路径/px4_fmu-v6x_default.px4
```

本次QGC实际选择Stable包，SHA-256=`3206a5f445d85be730a4f6158d8fdd414b9cfab27fa84bbc05c6a6752deac284`；预下载GitHub包未用于本次刷写。镜像与原始输出见[封包](acceptance/p1/20261031-hil-evidence.md)。

## 3. 独立于 Rust 应用的入口

1. 关闭串口监看、MAVProxy、IDE 上传器等占用者。在选定主机打开 QGC 的 Firmware/固件升级页面，按该版本的连接提示操作。
2. 从已知良好 Rust 出发，**本次不发送** `!bootloader` 或 MAVLink reboot。按本板可用方法彻底断开主板电源，再由 USB 重新连接；如采用实物复位按钮，记录真实按钮/供电状态。
3. 观察 QGC 是否发现原 PX4 bootloader。保存设备枚举、QGC 检测日志和板型信息，并记录从上电到识别的实际时序。
4. 虚拟机情况下明确 USB 所属主机，Rust 与 bootloader 的两个身份分别处理。QGC 在 Windows 时，设备应交给 Windows；Linux uploader 操作时再交给 Linux，避免两个程序同时持有接口。
5. 若 QGC 无法发现，记录失败与现场状态，继续诊断入口条件；该项保持未通过。不要故意刷入损坏镜像来测试恢复，也不要用更换 bootloader/option bytes 代替这个入口。

成功发现并完成下一节刷写，才证明该独立入口可用；仅重新出现一个 CDC 端口不算通过。

## 4. QGC 官方 PX4 → 回到已知良好 Rust

### A. QGC 刷官方 PX4

1. 在 QGC 固件页选择为该板提供的官方 PX4 发布版本，核对 FMUv6X 身份。保存 QGC 实际使用的镜像与 SHA-256；菜单名称以现场版本为准。
2. 完成上传/校验/重启，保存完整日志及失败重试记录。
3. 通过 QGC 确认 PX4 正常启动、心跳/版本/板型正确。若有对应参数/校准备份，按官方流程恢复并核对；记录缺失项。这是固件恢复演练，不等于飞行配置验收。

### B. 回装已知良好 Rust

1. 关闭 QGC 的串口占用；通过 PX4 支持的 `reboot -b` 或 MAVLink bootloader reboot 进入原 bootloader，或重复刚验证的独立入口。
2. 使用固定 uploader 和**保存的** Rust 包。例如本机 Linux：

```sh
.venv/bin/python target/p1-hil-20261031-104403/recovery/px4_uploader.py \
  --port '/dev/serial/by-id/*PX4_BL*' \
  --debug --noninteractive-json \
  .p0-baseline/20261031/v6x-rust-led.px4
```

3. 固定来源为 PX4 工作树 `a2be9197b57b2d065e4c47ca7fc5b7565ee07282` 的 `Tools/px4_uploader.py`。它与 v1.17 的 `Tools/px_uploader.py` 参数不同。现场核对实际 by-id 路径，保存 board ID、擦写与 CRC 结果。
4. 重启后重新找 Rust USB 端口，检查 PHY LED、`!status` 与 MAVLink 心跳/版本。回装保存的 0.1.0 时运行 `.venv/bin/python tools/check_mavlink.py --expected-version 0.1.0`；它核对历史 `RUSTFMU1` 签名，不误用新 P1 构建身份。新 0.2.0 使用默认版本并显式传入该包报告的 `--source-id`。
5. 在这个已知良好 Rust 上再验证正常软件 bootloader 重启和回装，作为便利入口记录；它与第 3 节的独立入口分别给出结论。

## 5. 参数/存储与后续回归

- USB uploader 不等于整片 flash/bootloader 读出工具；可选 SWD 备份按实际条件增加，不替代 QGC 恢复。
- 原 PX4 bootloader 与 EEPROM 身份/校准区保持原约定。P3a 对 SPI5 FRAM 先只读识别/原内容备份，再确定本项目布局；当前没有执行该操作。
- 内部 flash 参数可能被应用上传擦除。恢复资料包含升级前导出/升级后导入，而不是修改 bootloader 来保留数据。
- 启动、时钟、链接布局、打包、USB、参数存储变更后，回归相关恢复步骤。首次新功能上板必须已有本操作单的通过记录。

## 6. 通过条件

- 工程师实际完成独立入口、QGC 官方 PX4 上传并启动、回装已知良好 Rust、正常软件重启回归。
- 记录镜像 hash、QGC/bootloader 身份、参数资料状态、操作时序、上传/校验原始结果与执行者。
- 按当前精简计划核对必验项，本轮已全部通过。完整UI文本、额外CRC读回和重复P0中转不作为通过条件，缺失事实仍记录。

完成后将模板复制为实际日期记录，更新 [TODO](../TODO.md) 和硬件台账。设备功能实现与 [M1 的 QGC 消息/参数验收](m1-verification.md) 是另一组结果。
