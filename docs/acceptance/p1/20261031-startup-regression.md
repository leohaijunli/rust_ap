# P1：新固件启动 / bootloader 连接问题（基本连接已恢复）

日期：2026-10-31。**有效PANIC绑定`afaa140f499ff958 / src/platform.rs:91`；SCUEN最小修复及目标普通`4cba95aa54cd899a`离线回归通过。后续普通CDC/MAVLink监看成功，用户确认QGC基本连接恢复。** 见[连接复验](20261031-qgc-retest.md)及[断言修复](20261031-startup-assertion.md)。板上精确source、完整P1运行/恢复验收与更早134208 B包身份仍待补，P1-01/04未关闭。

## 用户报告

- 上传新固件后，QGC报错：**“no connection to bootloader”**。报错属于Firmware上传页还是普通连接、发生在擦除/写入之前还是之后，待确认。
- 用户确认：**“the previous version in this repo works”**。旧版为重要对照，但尚未绑定到保存的0.1.0包/hash，也未确认当前是否已回装旧版。
- 用户随后确认当前显示：**“it shows as BL FMU”**。这与agent观察的原PX4 bootloader产品身份一致；不是Rust CDC枚举，也未提供新的协议握手/上传/CRC结果。
- 随后执行并提供原始输出：

```text
/bin/bash /home/dev/rust_ap/tools/firmware.sh monitor 10
Traceback (most recent call last):
  File "/home/dev/rust_ap/tools/check_mavlink.py", line 195, in <module>
    main()
    ~~~~^^
  File "/home/dev/rust_ap/tools/check_mavlink.py", line 170, in main
    port = args.port or find_port()
                        ~~~~~~~~~^^
  File "/home/dev/rust_ap/tools/check_mavlink.py", line 25, in find_port
    raise RuntimeError(f"Expected one Rust CDC device, found {ports}")
RuntimeError: Expected one Rust CDC device, found []
```

此命令在端口发现阶段退出：没有找到`1209:0001`、serial=`V6X-RUST-001`，未执行后面的串口打开/STATUSTEXT请求。**仅说明没有发现普通Rust身份；原脚本也漏掉Dump身份，不能据此断言所有Rust USB都不可见。** 初始bootloader现场见下文；后续Dump枚举下重复出现的同一报错已由[监看入口修复](20261031-monitor-dump.md)解决。

## 用户补充：上传成功与QGC恢复失败

用户提供完整[uploader原始输出](20261031-user-upload.log)；没有提供本次upload命令或包路径/hash：

- board=`53,0`、protocol v5，134208 B；芯片=`0x20036450`、Revision V，应用flash容量=1966080 B。
- Erasing/Programming/Verifying达到100%，`Rebooting to application`，`Uploaded in 17s`、exit code 0。固定uploader的v5校验走GET_CRC，日志支持校验成功；预期/实际CRC数值没有在此INFO日志打印。
- `Serial: 004900223132510339313731`是芯片serial，不是bootloader USB描述符serial=`0`。
- 大小与归档普通P1的padded 134208 B一致，不等于精确文件身份已确认。仍需实际所选包路径/SHA-256；不能仅凭大小绑定source=`6852872943daf5a9`。
- `Verifying image signature`只说明进入工具的签名查询路径。旧/无安全bootloader允许跳过，该行不是已验证密码学签名的证据。`Rebooting to application`也不证明应用已经运行或枚举USB。

随后用户明确在QGC尝试刷回官方PX4，报`Permission error while locking the device`后取消；本次尚未开始刷写。主机测试已确认`/tmp`创建报EDQUOT，详见[原始错误/权限/Qt路径与修复提案](20261031-qgc-lock.md)。这与新应用无Rust CDC分开处理，不再把已成功的新P1上传当未完成，也不把QGC此条报错当bootloader协议拒绝。

用户再补充：**“you are right, i deleted /tmp folder, then i flashed px4 back”**。按用户报告记为清理临时目录后刷回PX4成功，当前应用不再按新Rust记录；PX4实际启动、QGC心跳、此次官方版本/镜像hash与完整恢复日志尚未提供。主机锁阻塞关闭，不把成功刷写扩展为完整P1验收通过，也不据此认定新Rust启动根因。

## agent只读主机观察

此前USB/sysfs、pyserial枚举（未打开端口）发现：

```text
USB 8-1: 3185:0035 manufacturer='Auterion'
product='PX4 BL FMU v6X.x' serial='0'
/dev/serial/by-id/usb-Auterion_PX4_BL_FMU_v6X.x_0-if00 -> /dev/ttyACM0
未发现Rust 1209:0001
```

内核日志原始相关行：

```text
Oct 31 03:37:45 dev kernel: usb 8-1: USB disconnect, device number 5
Oct 31 03:37:48 dev kernel: usb 8-1: new full-speed USB device number 6 using xhci_hcd
Oct 31 03:37:48 dev kernel: usb 8-1: New USB device found, idVendor=3185, idProduct=0035, bcdDevice= 1.01
Oct 31 03:37:48 dev kernel: usb 8-1: New USB device strings: Mfr=1, Product=2, SerialNumber=3
Oct 31 03:37:48 dev kernel: usb 8-1: Product: PX4 BL FMU v6X.x
Oct 31 03:37:48 dev kernel: usb 8-1: Manufacturer: Auterion
Oct 31 03:37:48 dev kernel: usb 8-1: SerialNumber: 0
Oct 31 03:37:48 dev kernel: cdc_acm 8-1:1.0: ttyACM0: USB ACM device
Oct 31 03:38:32 dev kernel: usb 8-1: USB disconnect, device number 6
Oct 31 03:38:32 dev kernel: usb 8-1: new full-speed USB device number 7 using xhci_hcd
Oct 31 03:38:32 dev kernel: usb 8-1: New USB device found, idVendor=3185, idProduct=0035, bcdDevice= 1.01
Oct 31 03:38:33 dev kernel: usb 8-1: New USB device strings: Mfr=1, Product=2, SerialNumber=3
Oct 31 03:38:33 dev kernel: usb 8-1: Product: PX4 BL FMU v6X.x
Oct 31 03:38:33 dev kernel: usb 8-1: Manufacturer: Auterion
Oct 31 03:38:33 dev kernel: usb 8-1: SerialNumber: 0
Oct 31 03:38:33 dev kernel: cdc_acm 8-1:1.0: ttyACM0: USB ACM device
```

03:45:07（主机时区`-07:00`）再次只读检查仍为该bootloader身份；`/dev/ttyACM0`权限`0777 root:uucp`，agent用户当时可读写。可见`/proc`中短暂有`python`进程PID145676、FD3占用该端口，读取其命令行时进程已经结束，**用途/操作者未知**；不能据此判定QGC报错由端口占用引起。更早的定向进程搜索未见QGC/uploader/监看器，只代表那一次主机快照。

本机`~/.local/state/qgroundcontrol/startup-20261031.log`只有先前安装启动检查，没有此次Firmware失败记录；用户随后给出的原始错误已单独入档。03:54进程环境确认当前QGC使用本机官方5.1.5 AppImage，见[主机检查](20261031-qgc-lock.md)；先前笼统“no connection”报告的详细页面/操作顺序仍未知。

只读枚举本身仅证明**原bootloader USB可见**；用户补充日志另支持本次上传协议握手/写入/校验成功。两者仍不证明应用启动或官方PX4恢复正常，也不能区分人为进入bootloader、应用panic/HardFault后复位或其他启动失败。

## 源码对照与定位进展

- 旧基线使用HSI 64 MHz、标准Embassy线程executor；新P1增加480 MHz PLL1、启动断言、RTC访问、raw线程executor与TIM7替身。USB PLL3Q、VID/PID/serial与应用`0x08020000`边界保留。
- `src/platform.rs`在USB初始化前检查silicon revision=`0x2003`及全部时钟寄存器；断言失败会走`src/fault.rs`的panic→保留记录→BKP0请求原bootloader→复位。用户日志确认Revision V；**后续有效PANIC已精确定位到第91行时钟断言**。
- 固定`embassy-stm32-0.5.0/src/rcc/h.rs:497-502,789-809`依次初始化PLL1/2/3；未配置PLL2的分支对共享`PLLCKSELR`使用整寄存器`write`而非`modify`。保存的普通P1反汇编`0x0802aa42-0x0802aa4a`也显示向`0x58024428`写零。这是需进一步核查的共享寄存器写入风险，**PLL运行时字段写保护与本板实际读回尚未核实，不把源码写零直接等同于硬件PLL1失效**。
- 实际Dump入口和P0 HAL后`CR3=0x05000042`，SCUEN=0；该位是供电配置更新控制位。旧普通ELF错误要求`(CR3 & 7)==6`，修复后只检查`(CR3 & 3)==2`。回放实测CR3及预期P1配置确认旧包拒绝、新包通过；新包实际启动/其余寄存器仍待复验。

## 备份域互操作与旧记录丢失风险（源码推断）

以下源码链解释记录可能被清除的路径。**本次实际取得了有效`afaa140f499ff958`记录；没有证据证明更早记录曾被清空**：

- PX4 fmu-v6x 的 NuttX 配置把 RTC magic 放在 **BKP1**（`boards/px4/fmu-v6x/nuttx-config/nsh/defconfig:253` `CONFIG_STM32H7_RTC_MAGIC_REG=1`），RTC 时钟选择默认 **LSE**（`platforms/nuttx/NuttX/nuttx/arch/arm/src/stm32h7/Kconfig:1735`）。因此 PX4 的 `up_rtc_initialize` 如果发现 BKP1 不是 `0xfacefeed`，或 `RCC_BDCR.RTCSEL` 与 LSE 不一致，就会执行 **backup domain reset**（`platforms/nuttx/NuttX/nuttx/arch/arm/src/stm32h7/stm32_rtc.c:905-957` 与 `:971-987`）。备份域复位会清空 `BKP0..31`，也就是 P1 的故障记录 `BKP16..31`。
- 我们的 P1 在 `config.rcc.ls` 里只保留 LSI、否则 `LsConfig::off()`（`src/platform.rs:80-84`）。PX4 留下的是 LSE，所以 embassy 的 `bd::init` 判定不匹配并做 BDRST，随后把 `RTCSEL` 写成 DISABLE（`embassy-stm32-0.5.0/src/rcc/bd.rs:305-330`）。
- 在「P1失败 → BL → 刷回官方PX4」路径中，如果PX4启动时`RTCSEL`仍不是LSE，就会触发相应BDRST分支；仅恢复`BKP1`的magic不足以匹配RTC配置。这是源码推导，未实测该次路径中的BDCR变化。
- 用户报告刷回PX4成功，04:02也只读枚举到官方应用；这曾是怀疑旧记录可能被清空的依据。14:59 UTC获得的有效PANIC属于后来普通source=`afaa140f499ff958`，不能反推更早记录状态。

由此得到两条结论：

1. 实际Dump已采集并定位断言；此前重新触发失败的双刷方案已完成诊断用途，当前操作改为修复后普通P1复验。
2. 这是需要修复的互操作缺陷：当前配置与 PX4 的 LSE 不匹配时会复位备份域；仅恢复 BKP 字不足以保留 RTC 配置。修复应匹配观察到的 BDCR（LSE/LSI、`LSEDRV`、`RTCEN`）及生成的 `ClockMux.rtcsel`，并核对 HAL 后读回。

## 故障转储镜像（实际采集成功）

已实现 `--features fault-dump` 诊断镜像：在任何 HAL 初始化前捕获 `RCC_RSR`、`BKP0`、`BKP16..31` 与入口 RCC/PWR/FLASH/SYSCFG，随后用已知可用的 P0 时钟树（HSI 64 MHz + PLL3Q 48 MHz USB）启动，通过 USB CDC 循环输出；不写故障记录，只恢复捕获的 BKP 字，并提供 `!bootloader` 文本命令。构建/判读/恢复步骤见[故障转储镜像记录](20261031-fault-dump-image.md)。

实际使用的Dump source=`4f8bbf296699e6f2`，raw/padded=34788/34816 B，`.px4` SHA-256=`90d074c2876ba1be7e41a750255b6eae39fa7bb20dd8c931c0da70f7bb402095`；当时dump Clippy、fmt、打包/布局/向量、12项Rust与30项Python通过。工程师已采集6456 B、4份完整报告，见[原始文本](20261031-startup-dump.txt)及[诊断JSON](20261031-startup-monitor.json)。SCUEN修复后重建Dump为`15432408177fb110`，不与本次实际采集身份混用；最新产物见[备份域修复](20261031-backup-domain.md)。

监看工具已支持自动识别Dump、重枚举等待、保存原始字节及诊断JSON；当前59项Python通过，用户实际采集成功。具体行为与退出码见[记录](20261031-monitor-dump.md)。

## 下一步与恢复边界

**当轮交接方向**：自主调试完成断言定位、SCUEN最小修复及离线回归后，由工程师从Dump切回普通P1，完整命令见[交接步骤](20261031-startup-assertion.md#5-工程师复验从当前-dump-返回普通-p1)。随后普通CDC/MAVLink及QGC基本连接已恢复；最新软件修复和上板待验项见[备份域修复](20261031-backup-domain.md)。

HAL核对新增：生成的`ClockMux::init()`在`LsConfig::init()`之后再次修改`RCC_BDCR.RTCSEL`，默认`rtcsel=DISABLE`。备份域修复必须兼顾两处配置与HAL后读回。

历史诊断顺序为失败普通`afaa140f499ff958` → 原bootloader → Dump `4f8bbf296699e6f2` → 读取记录。当前`upload`重建的是已修复普通`4cba95aa54cd899a`，不能再按历史失败预期判断。`valid=NO`分阶段定位仍是条件性后备方案，本次没有触发。

1. 修复后的普通启动/QGC与P1时基/调度/恢复由工程师复验；保存完整输出、source/hash与采集记录。更早134208 B包身份和此次官方恢复的版本/hash/完整日志仍待补。
2. 保留失败P1归档与旧恢复包，不通过默认`firmware.sh upload`尝试回装旧版：该入口会重新构建并上传新P1。
3. 如工程师明确选择回装，显式使用保存的`.p0-baseline/20261031/v6x-rust-led.px4`（SHA-256=`d74b3b80e7c35bf4354ea568c09631f2fad362570caff414516ee0fe48a3c02d`），保存此次日志/CRC，再核对Rust枚举与旧版身份；旧包不是官方PX4镜像，不能替代P1-01官方恢复验收。
4. 协议探测/刷写/复位仍由工程师明确执行；原bootloader、option bytes及应用边界按既定约束保留。最小修复及新身份已生成，P1完整验收前不推进依赖它的硬件阶段。

历史软件归档与检查结果见[软件索引](README.md)；恢复资料不可得/无原备份的状态见[用户恢复报告](20261031-user-recovery-report.md)，不据此次报错推断板上原数据已丢失。

清理后只读核对：项目内P0旧恢复包与P1普通/注入包hash均未变，P1源码归档仍在；`/tmp/opencode`中的上游/构建临时目录已不存在，不能继续使用其旧路径，也未自动重新下载。

## 回装PX4后的上传入口错误（独立问题，已离线修复）

用户随后`firmware.sh upload`报`Expected one FMUv6X serial port, found []`。只读枚举发现官方应用`usb-Auterion_PX4_FMU_v6X.x_0-if00`，原脚本仅匹配Rust/BL，故在构建/上传前退出。默认模式已补齐，当轮新增6项host回归（总22项Python）与Bash语法通过，现场仅只读匹配成功；详见[记录](20261031-upload-discovery.md)。此入口修复与后来SCUEN断言修复分别有证据；普通P1/QGC基本连接已恢复，工程师完整复验状态见[最新索引](README.md)。
