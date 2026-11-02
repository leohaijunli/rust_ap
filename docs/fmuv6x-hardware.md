# Pixhawk 6X Rev 3 硬件台账

审查日期：2026-10-31；状态更新：2026-11-02。计划与进度见 [plan.md](../plan.md)、[TODO.md](../TODO.md)。

## 来源与证据边界

- PX4 源码根目录：`/home/dev/PX4-Autopilot-v1.17`。
- 固定版本：tag `v1.17.0`，commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`；本次查看的板级/bootloader 文件无本地修改。
- 下表的 PX4 路径均相对该根目录，引用为 `文件:行`。表中值是**源码配置或源码注释**；尚未逐项完成本板 HIL，不能据此认定所有器件已识别。
- 本板历史识别记录为 FMUM `0x003`、STM32H7 revision V（[README Hardware](../README.md#hardware)）。这是既有记录，本次审查没有重新探测或刷写。
- 工程师要求始终保留 **QGroundControl 刷回官方 PX4**。原 bootloader、USB 识别及应用入口保持；2026-10-31 的独立 QGC 往返和最终普通 Rust 回装已通过，见 [P1 HIL](acceptance/p1/20261031-hil.md)。

## 启动、flash 与 RAM

| 项 | 值 / 结论 | PX4 来源（文件:行） |
| --- | --- | --- |
| MCU / 架构配置 | STM32H753II / Cortex-M7 | `boards/px4/fmu-v6x/nuttx-config/bootloader/defconfig:19`；`boards/px4/fmu-v6x/default.px4board:2` |
| 用户 flash bank | bank 1：0x08000000–0x080FFFFF；bank 2：0x08100000–0x081FFFFF；各 8 个 128 KiB 扇区 | `boards/px4/fmu-v6x/nuttx-config/scripts/script.ld:41-44` |
| bootloader 保留 | 前 128 KiB | `platforms/nuttx/src/bootloader/stm/stm32_common/main.c:25` |
| 应用起点与大小 | 0x08020000，1920 KiB（末端 0x08200000，不含）；跨两个 bank | `boards/px4/fmu-v6x/src/hw_config.h:64`；`boards/px4/fmu-v6x/nuttx-config/scripts/script.ld:113` |
| .px4 容器 | board_id=53；image_maxsize=1966080 | `boards/px4/fmu-v6x/firmware.prototype:2-10` |
| bootloader 应用保留区 | 此板默认 APP_RESERVATION_SIZE=0；首次可擦物理扇区为 1，最后为 15 | `boards/px4/fmu-v6x/src/hw_config.h:76-77`；`boards/px4/fmu-v6x/src/hw_config.h:108-114` |
| 上传擦除行为 | ERASE 遍历扇区；板级函数跳过 sector 0，擦除非空的 sector 1–15。普通应用内的参数扇区不会因 linker 预留而被 uploader 保留 | `platforms/nuttx/src/bootloader/common/bl.c:750-787`；`platforms/nuttx/src/bootloader/stm/stm32_common/main.c:459-477` |
| 应用大小计算 | flash 总大小减 bootloader 与 APP_RESERVATION_SIZE；不表示任意尾部参数自动被保护 | `platforms/nuttx/src/bootloader/stm/stm32_common/main.c:74-82` |
| 软件进入 bootloader | RTC BKP0 写 0xb007b007；bootloader 识别后不尝试先启动应用、取消退出超时并清除签名 | `platforms/nuttx/src/bootloader/stm/stm32_common/main.c:103-128`；`platforms/nuttx/src/bootloader/stm/stm32_common/main.c:667-688` |
| 启动等待与接口 | BOOTLOADER_DELAY=5000；USB 与串口接口启用。不能仅凭该常量保证任意复位都有可用 USB 等待窗口 | `boards/px4/fmu-v6x/src/hw_config.h:64-74`；`platforms/nuttx/src/bootloader/stm/stm32_common/main.c:639-688` |
| USB 上电分支 | 有 BOARD_VBUS 时按引脚电平决定是否先进入 bootloader；本板实际 VBUS/复位顺序影响入口 | `platforms/nuttx/src/bootloader/stm/stm32_common/main.c:725-746` |
| 强制 bootloader 引脚 | 该板头文件的 PWM 跳线强制入口是注释示例，不能作为已可用的恢复方法 | `boards/px4/fmu-v6x/src/hw_config.h:88-102` |
| ITCM / DTCM | ITCM 64 KiB @0x00000000；DTCM 两段各 64 KiB @0x20000000、0x20010000 | `boards/px4/fmu-v6x/nuttx-config/scripts/script.ld:112-116` |
| AXI SRAM | 512 KiB @0x24000000 | `boards/px4/fmu-v6x/nuttx-config/scripts/script.ld:117` |
| D2 SRAM | SRAM1 128 KiB @0x30000000；SRAM2 128 KiB @0x30020000；SRAM3 32 KiB @0x30040000 | `boards/px4/fmu-v6x/nuttx-config/scripts/script.ld:118-120` |
| D3 / backup RAM | SRAM4 64 KiB @0x38000000；backup 4 KiB @0x38800000 | `boards/px4/fmu-v6x/nuttx-config/scripts/script.ld:121-122` |
| DMA 内存约束线索 | PX4 linker 注释说明 AXI/D2 不供 BDMA 访问，TCM 的访问能力与普通 SRAM 不同；实际 DMA1/2、BDMA、SDMMC2 DMA/IDMA 需逐项核对芯片资料与 HAL | `boards/px4/fmu-v6x/nuttx-config/scripts/script.ld:66-103` |

上述擦除行为属于所审源码；本板已安装 bootloader 的具体版本/构建还未重新确认。P3a 不得把“放另一 bank”当作升级保留策略。若需内部 flash 参数区，须同时约束代码与打包边界，并提供升级前导出、升级后恢复；不能为此修改原 bootloader。

## 时钟、状态灯与电源控制

| 项 | 值 / 结论 | PX4 来源（文件:行） |
| --- | --- | --- |
| HSE | 16 MHz 晶体 | `boards/px4/fmu-v6x/nuttx-config/include/board.h:58-73` |
| PX4 CPU 配置 | PLL1：HSE /1 ×60 /2，CPU 480 MHz；P1 Rust同一目标配置的寄存器和65 s TIM2时基检查通过，旧0.1.0为64 MHz | `boards/px4/fmu-v6x/nuttx-config/include/board.h:100-123`；`boards/px4/fmu-v6x/nuttx-config/include/board.h:159-175`；`src/platform.rs` |
| VOS / overdrive | NuttX 默认 VOS1；SYSCLK >400 MHz 时选择 overdrive，设置 SYSCFG ODEN 并等待 VOSRDY。此组合不能简化为“普通 VOS1 可直接跑 480 MHz” | `platforms/nuttx/NuttX/nuttx/arch/arm/src/stm32h7/stm32h7x3xx_rcc.c:57-61,106-131,900-934` |
| 供电选择 | 非 SMPS、非外部电源分支请求 LDOEN + SCUEN；CR3 低字节的上电后写入限制与等待 ACTVOSRDY 有明确说明。SCUEN 是更新控制位，不能要求它始终读回1；HAL与实际Dump核对见下节 | `platforms/nuttx/NuttX/nuttx/arch/arm/src/stm32h7/stm32h7x3xx_rcc.c:134-162,881-915` |
| flash latency | 板宏 BOARD_FLASH_WAITSTATES=2，NuttX 同时配置 WRHIGHFREQ 与 LATENCY；需按实际 AXI/flash 时钟和电压解释，不直接照抄到 Rust | `boards/px4/fmu-v6x/nuttx-config/include/board.h:267-285`；`platforms/nuttx/NuttX/nuttx/arch/arm/src/stm32h7/stm32h7x3xx_rcc.c:937-949` |
| USB 时钟 | PLL3Q：HSE /4 ×48 /4 = 48 MHz；USB 选择 PLL3 | `boards/px4/fmu-v6x/nuttx-config/include/board.h:143-157`；`boards/px4/fmu-v6x/nuttx-config/include/board.h:245-247` |
| FDCAN 时钟 | PX4 选择 HSE，STM32_FDCANCLK=STM32_HSE_FREQUENCY | `boards/px4/fmu-v6x/nuttx-config/include/board.h:259-263` |
| SDMMC2 软件时钟 | PL-a 启用 PLL1Q：HSE /1 ×60 /20 = 48 MHz，选择 PLL1Q；首轮请求 25 MHz，HAL v2 候选实际 24 MHz | `src/platform.rs`、`src/storage/sd_logger.rs`；17 项编译谓词通过，物理时钟待 H-04 |
| 状态 LED | 红 PE3、绿 PE4、蓝 PE5；开漏输出，初始化高电平；bootloader 定义低电平为亮 | `boards/px4/fmu-v6x/src/board_config.h:93-103`；`boards/px4/fmu-v6x/src/hw_config.h:81-84` |
| 传感器 1/2/3 电源控制 | Rev 3 SPI 总线表分别给出 PI11 / PF4 / PE7；公共控制函数高电平使能。项目只初始化 SPI2 对应 PF4；其余仅作来源记录 | `boards/px4/fmu-v6x/src/spi.cpp:87-97`；`platforms/nuttx/src/px4/stm/stm32_common/spi/spi.cpp:319-345` |
| SPI 复位后的等待 | PX4 复位 helper 重新上电后等待 100 us 再配置引脚；该值不代替 ICM-42688-P datasheet 的 POR/复位/配置等待 | `platforms/nuttx/src/px4/stm/stm32_common/spi/spi.cpp:469-484` |
| 传感器 4 电源 | PG8，高电平使能 | `boards/px4/fmu-v6x/src/board_config.h:269`；`boards/px4/fmu-v6x/src/board_config.h:291` |
| IMU 加热 | PB10，GPIO_HEATER_OUTPUT，初始低；第一版不启用 | `boards/px4/fmu-v6x/src/board_config.h:230-234` |
| SD 电源 | PC13，高电平使能 | `boards/px4/fmu-v6x/src/board_config.h:271`；`boards/px4/fmu-v6x/src/board_config.h:294` |
| 外部 5 V 电源控制 | PERIPH_nEN PG4、HIPOWER_nEN PG10，低有效；这不能单独证明 CAN 插座的供电连接 | `boards/px4/fmu-v6x/src/board_config.h:265-268`；`boards/px4/fmu-v6x/src/board_config.h:289-290` |
| 外部 armed 指示 | PE6：未解锁时输入上拉，解锁时输出低 | `boards/px4/fmu-v6x/src/board_config.h:236-244` |
| 版本识别配置 | ADC3，PH3/PH4 感测，PG0 驱动；V6X_3=FMUM ID 0x3 | `boards/px4/fmu-v6x/src/board_config.h:189-190`；`boards/px4/fmu-v6x/src/board_config.h:203-225` |

P1 已核对固定 HAL 的 LDO/VOS/overdrive/flash 路径；uploader 确认 silicon revision V，实际 Dump 的供电位见下节。修复后普通 P1 的平台寄存器及两组65 s时基检查已通过，测量范围见[HIL](acceptance/p1/20261031-hil.md)。

### Dump 寄存器解码核对（2026-10-31）

以下是 host 解码定义；后续实际Dump记录见下一节。本次交叉核对使用本地
`/home/dev/PX4-Autopilot` 的固定 commit `a2be9197b57b2d065e4c47ca7fc5b7565ee07282`
及锁定的 `stm32-metapac 19.0.0`；表中 PX4 头文件为
`platforms/nuttx/NuttX/nuttx/arch/arm/src/stm32h7/hardware/stm32h7x3xx_rcc.h`，
H753实际选用的PAC文件为 `src/peripherals/rcc_h7rm0433.rs`（`src/chips/stm32h753ii/pac.rs:807-808`）；本表已从先前通用`rcc_h7.rs`引用改为实际变体。

| 项 | 正确解释 | 来源（文件:行） |
| --- | --- | --- |
| PLL1/3 N | `[8:0]` 存储 N−1；`0x3b`→60、`0x2f`→48 | PX4 头文件 `:432-433,466-467`；PAC `:10679-10688,11927,11939` |
| USBSEL | `RCC_D2CCIP2R[21:20]`；值2为PLL3Q；Dump线字段`usbsel`存储完整寄存器 | PX4 头文件 `:597-602`；PAC `:9752-9761`；本项目 `src/fault_dump.rs:54-65,163-169` |
| SYSCLK状态 | `RCC_CFGR.SWS[5:3]`；`SW[2:0]`仅为请求选择，二者可不同 | PX4 头文件 `:228-233`；PAC `:8504-8513` |

### 启动断言实测与供电位核对（2026-10-31）

本节额外核对本地 `/home/dev/PX4-Autopilot`（`a2be9197b57b2d065e4c47ca7fc5b7565ee07282`）内 NuttX 子模块 **`9304289d9f2b304c6835c9d39c131648c5f804c2`**；所查 RCC/PWR 文件无本地修改。下表 NuttX 路径相对该子模块的 `arch/arm/src/stm32h7/`；与本页默认的 v1.17 板级参考分开固定。

| 项 | 值 / 结论 | 来源（文件:行） |
| --- | --- | --- |
| CR3 低三位 | BYPASS=bit0、LDOEN=bit1、SCUEN=bit2；SCUEN定义为供电配置更新使能，不是稳压器就绪状态 | NuttX `hardware/stm32h7x3xx_pwr.h:124-127`；`stm32-metapac 19.0.0/src/peripherals/pwr_h7rm0433.rs:434-466` |
| ST H753 定义 | 同上，Supply configuration update enable | ST `cmsis-device-h7` v1.10.4，commit `faccfec37f82f7a1319c21638111b0f7335de7fe`，`Include/stm32h753xx.h:14414-14422` |
| CR3 写入限制 | 固定源码注明低字节POR/write-once约束；HAL不能知道bootloader已执行的写入，写入请求不保证读回SCUEN=1 | NuttX `stm32h7x3xx_rcc.c:881-898`；PAC `pwr_h7rm0433.rs:429`；`embassy-stm32 0.5.0/src/rcc/h.rs:286-296` |
| 实测 CR3 | Dump入口和P0 HAL后均为`0x05000042`，LDOEN=1、BYPASS=0、SCUEN=0；尚未证明其读零的具体硬件机制 | [原始Dump](acceptance/p1/20261031-startup-dump.txt):4,12；重复报告一致 |
| 实测 PANIC | 有效CRC=`0xd209f8cf`，普通source=`afaa140f499ff958`，`src/platform.rs:91`的时钟断言；发生在HAL返回后、USB/任务启动前 | 原始Dump`:6-9`；[匹配构建报告](acceptance/p1/20261031-startup-before.build.json)；归档源码 |
| P1 最小修复 | 删除SCUEN读1要求；继续检查LDO/BYPASS、电压就绪及PLL/flash/TIM2；该轮普通source=`4cba95aa54cd899a`，最新产物见P1索引 | `src/platform.rs:87-130`；[修复与回放](acceptance/p1/20261031-startup-assertion.md) |
| 回放RCC地址 | base=`0x58024400`；CFGR/D1/D2/D3/PLLCKSELR/PLLCFGR/PLL1DIVR/PLL3DIVR/D2CCIP2R偏移=`0x10/18/1c/20/28/2c/30/40/54` | NuttX `hardware/stm32h7x3xx_memorymap.h:212`、`hardware/stm32h7x3xx_rcc.h:41-55` |
| 回放PWR/SYSCFG地址 | PWR base=`0x58024800`，CSR1/CR3/D3CR偏移=`0x04/0c/18`；SYSCFG.PWRCR=`0x5800042c` | NuttX `hardware/stm32h7x3xx_memorymap.h:185,213`、`hardware/stm32h7x3xx_pwr.h:31-36`、`hardware/stm32h7x3xx_syscfg.h:51` |
| 回放flash/TIM2地址 | FLASH.ACR=`0x52002000`；TIM2 base=`0x40000000`，PSC/ARR偏移=`0x28/2c` | NuttX `hardware/stm32h7x3xx_memorymap.h:87,171`、`hardware/stm32h7x3xx_flash.h:30`、`hardware/stm32_tim.h:69-70` |

回放只有CR3来自实际Dump，其余寄存器为预期P1模型；它验证真实ELF的断言条件，不证明全部故障前寄存器、HAL执行或物理频率。Dump的入口/P0快照不能还原旧P1完整现场。

### RTC/LSE备份域互操作（2026-10-31）

以下使用上一节固定的PX4/NuttX工作树、`embassy-stm32 0.5.0`及`stm32-metapac 19.0.0`。引用文件及生成mux的SHA-256见[输入清单](acceptance/p1/20261031-backup-domain/upstream-inputs.json)。

| 项 | 值 / 结论 | 来源（文件:行） |
| --- | --- | --- |
| LSE板级频率 | 32768 Hz；这是板宏，不是本板走时测量 | PX4 `boards/px4/fmu-v6x/nuttx-config/include/board.h:73` |
| 备份域地址 | BDCR=`0x58024470`、CSR=`0x58024474`；APB4ENR=`0x580244f4`，RTCAPBEN=bit16 | NuttX `hardware/stm32h7x3xx_rcc.h:60-61,82,991`；PAC `rcc_h7rm0433.rs:126-134,236-239` |
| RTC/BKP地址 | RTC=`0x58004000`；32个BKP字起点=`0x58004050`，步长4 B | NuttX `hardware/stm32h7x3xx_memorymap.h:195`；PAC `stm32h753ii/pac.rs:709,811`、`rtc_v2_h7.rs:111-113` |
| 保持配置掩码 | `0x0001833d`：LSEON、LSEBYP、LSEDRV、LSECSSON、RTCSEL、RTCEN、BDRST；LSERDY(bit1)/LSECSSD(bit6)是实时状态，另行观察 | NuttX `hardware/stm32h7x3xx_rcc.h:1127-1149`；PAC `rcc_h7rm0433.rs:5117-5216` |
| 实际PAC的LSE枚举 | `LOW=0`、`MEDIUM_HIGH=1`、`MEDIUM_LOW=2`、`HIGH=3`；通用`rcc_h7.rs`的两个中档相反 | PAC `stm32h753ii/pac.rs:807-808`、`rcc_h7rm0433.rs:11425-11433`；通用文件`:11509-11517` |
| 修订差异线索 | NuttX区分rev Y与其余修订的中档编码，并按DBGMCU修订选择表。因此此处PAC枚举名不作为本板物理驱动强度结论；修复只保持继承原始位值 | NuttX `stm32_lse.c:56-74,95-112`、`hardware/stm32h7x3xx_rcc.h:1131-1137` |
| HAL驱动转换 | `LseDrive`按variant名称转换回选用PAC；须匹配PAC枚举，不能直接按`LseDrive`数值解读寄存器 | HAL `rcc/bd.rs:34-55`；本项目`src/backup_domain.rs` |
| HAL复位条件 | 低速配置与BDCR不匹配时设置/清除BDRST；匹配则提前返回 | HAL `rcc/bd.rs:250-315`；NuttX `stm32_rtc.c:913-961,969-1000`提供备份域复位/恢复参考 |
| 第二处RTC选择 | `config.ls.init()`后仍调用`config.mux.init()`，生成代码再次写RTCSEL；两个配置必须一致 | HAL `rcc/h.rs:586,699`；生成`_generated.rs:3681-3682,3706-3720`（已归档） |
| LSI处理 | RTC选择LSI时启用；其它情况下HAL不关闭继承的LSI | HAL `rcc/bd.rs:160-168,191-215` |
| 回放附加MMIO | GPIOA..K=`0x58020000..0x58022800`，EXTI=`0x58000000`，DBGMCU=`0x5c001000` | NuttX `hardware/stm32h7x3xx_memorymap.h:184,201-211,227` |

支持继承RTC关闭/LSE/LSI，以及兼容的LSE停用/振荡/旁路和四种原始驱动位值。HSE作为RTC源、BDRST已置位、RTCSEL/RTCEN不一致、停止LSE却选LSE或旁路的状态，在HAL前拒绝。普通/注入版之后按故障路径处理；Dump的panic保持停机。三版各132场景真实ELF回放通过，范围和限制见[备份域证据](acceptance/p1/20261031-backup-domain.md)。板上BDCR/CSR及HAL边界保持已通过[HIL](acceptance/p1/20261031-hil.md)；P1 当轮未测 RTC 日历，2026-11-02 LSE 走时/写后回读已通过[时间记录](time-sync.md#2026-11-02-实测结论)。

## Rev 3 传感器与总线

### 源码预期器件（尚未作为本板实测清单）

用户于 2026-10-31 确认：本项目仅启用 **SPI2 ICM-42688-P**。下表保留其他器件的 PX4 配置作为板级来源记录，不构成驱动、探测、采样或日志任务。

| 项 | 值 / 结论 | PX4 来源（文件:行） |
| --- | --- | --- |
| SPI1 IMU | ICM-42670-P，CS PI9，DRDY PF2 | `boards/px4/fmu-v6x/src/spi.cpp:87-90` |
| SPI2 IMU | ICM-42688-P，CS PH5，DRDY PA10 | `boards/px4/fmu-v6x/src/spi.cpp:91-93` |
| SPI3 陀螺 | BMI088 gyro，CS PI8，DRDY PI7 | `boards/px4/fmu-v6x/src/spi.cpp:94-95` |
| SPI3 加速度计 | BMI088 accel，CS PI4，DRDY PI6 | `boards/px4/fmu-v6x/src/spi.cpp:96-97` |
| Rev 4 对比 | Rev 4 的 SPI3 为 ICM-20649；不作为本项目默认器件或需实现的变体 | `boards/px4/fmu-v6x/src/spi.cpp:111-120` |
| 内部磁力计启动 | 非 V6X001 分支启动内部 I2C BMM150；实际地址、总线与型号待测 | `boards/px4/fmu-v6x/init/rc.board_sensors:118-125` |
| 气压计启动 | 适用分支尝试内部 I2C BMP388 @0x77（受参数控制），并尝试外部 BMP388；不能据此确定物理数量 | `boards/px4/fmu-v6x/init/rc.board_sensors:130-147` |
| 气压计中断线索 | GPIO_I2C2_DRDY1_BMP388=PG5，是否对应实物/启用中断待验 | `boards/px4/fmu-v6x/src/board_config.h:140` |
| ICM-42688-P 安装方向 | 启动参数 `-R 6`，枚举为 ROTATION_YAW_270；从传感器轴到项目机体系的组合变换仍须六面/转动实测 | `boards/px4/fmu-v6x/init/rc.board_sensors:84-115`；`src/lib/conversion/rotation.h:52-63` |

### 总线引脚

| 项 | 值 | PX4 来源（文件:行） |
| --- | --- | --- |
| SPI1 | SCK PA5、MISO PG9、MOSI PB5 | `boards/px4/fmu-v6x/nuttx-config/include/board.h:427-429` |
| SPI2 | SCK PI1、MISO PI2、MOSI PI3 | `boards/px4/fmu-v6x/nuttx-config/include/board.h:431-433` |
| SPI3 | SCK PC10、MISO PC11、MOSI PB2 | `boards/px4/fmu-v6x/nuttx-config/include/board.h:435-437` |
| SPI5 | SCK PF7、MISO PH7、MOSI PF11 | `boards/px4/fmu-v6x/nuttx-config/include/board.h:439-441` |
| I2C1 / I2C2 | I2C1 SCL PB8/SDA PB9；I2C2 SCL PF1/SDA PF0 | `boards/px4/fmu-v6x/nuttx-config/include/board.h:455-462` |
| I2C3 / I2C4 | I2C3 SCL PA8/SDA PH8；I2C4 SCL PF14/SDA PF15 | `boards/px4/fmu-v6x/nuttx-config/include/board.h:467-474` |
| 独立 I2C 插座 / INA219 | 4 针：1=5 V、2=SCL3/PA8、3=SDA3/PH8、4=GND，信号 3.3 V；SEN0291 拨码 A0=0/A1=1 即 **0x44**（固件自动探测 0x40/41/44/45）、10 mΩ、±8 A、独立支路，工程师确认已连接。**注意**：模块 P3 通孔座为 1=VCC、2=SCL、3=SDA、4=GND（与飞控一致），P2 Gravity 座为 1=GND、2=SDA、3=SCL、4=VCC（与飞控整序相反），直插 Gravity 线会反接供电。I2C3 100 kHz、DMA1_CH4/5、P6；PG4 低使能 5 V 外设电源 | 固定 PX4 `i2c.cpp` / `board_config.h`；[Holybro 插座表](https://docs.holybro.com/autopilot/pixhawk-baseboards/pixhawk-baseboard-v2-ports.md#i2c-port)；[SEN0291 原理图](https://github.com/DFRobot/Wiki/raw/master/SEN0291/res/SEN0291%20(V1.0)%20Schematic.pdf)；[P4p](acceptance/ina219/README.md) |

P4.0 实测记录至少包含：日期、固件 hash、器件 ID、总线/CS/地址、识别序列、DRDY 实测、轴向/安装方向、原始输出文件和工程师确认。读到 ID 不能证明 DRDY、方向、FIFO 或校准已经正确。

## FRAM、EEPROM、SD、CAN 与串口

| 项 | 值 / 结论 | PX4 来源（文件:行） |
| --- | --- | --- |
| 参数 FRAM 总线 | SPI5，CS PG7 | `boards/px4/fmu-v6x/src/spi.cpp:102-104`；`boards/px4/fmu-v6x/nuttx-config/include/board.h:415-441` |
| 参数 FRAM 型号/布局 | 实物已识别为 **FM25V02A，32 KiB**：RDID=`7F 7F 7F 7F 7F 7F C2 22 08`、SR=0x00、MODE3、SPI5 配置读回 937.5 kHz。PX4 为一个 MTD_PARAMETERS 区 `/fs/mtd_params`；写入前备份末个非零字节在 `0x6C57`，`0x7800–0x7FFF` 当时全 0，用作 Rust 参数双槽 | `boards/px4/fmu-v6x/src/mtd.cpp:39-43`；`boards/px4/fmu-v6x/src/mtd.cpp:54-64`；[P3a 实测](acceptance/p3a/README.md) |
| base EEPROM | 注释 24LC64T，I2C3 @0x51；分区为板级版本/网络信息，不作本项目参数区 | `boards/px4/fmu-v6x/src/mtd.cpp:44-47`；`boards/px4/fmu-v6x/src/mtd.cpp:66-82` |
| IMU EEPROM | 注释 24LC64T，I2C4 @0x50；含校准/修订/身份分区，不作本项目参数区 | `boards/px4/fmu-v6x/src/mtd.cpp:48-51`；`boards/px4/fmu-v6x/src/mtd.cpp:84-104` |
| SD 总线 | **SDMMC2**；CK PD6、CMD PD7、D0 PB14、D1 PB15、D2 PG11、D3 PB4 | `boards/px4/fmu-v6x/nuttx-config/include/board.h:479-496` |
| SD 卡检测 | 尚无已核实的 card-detect GPIO；该 SDIO 初始化代码在无检测宏时假设插卡，不能据此承诺硬件热插拔通知 | `boards/px4/fmu-v6x/src/sdio.c:62-67`；`boards/px4/fmu-v6x/src/sdio.c:161-172` |
| CAN1 | RX PD0、TX PD1；头文件说明连到收发器 | `boards/px4/fmu-v6x/nuttx-config/include/board.h:405-411` |
| CAN2 | RX PB12、TX PB13；头文件说明连到收发器 | `boards/px4/fmu-v6x/nuttx-config/include/board.h:405-413` |
| TELEM1 | UART7；RX PF6、TX PE8、RTS PF8、CTS PE10 | `boards/px4/fmu-v6x/nuttx-config/include/board_dma_map.h:77-78`；`boards/px4/fmu-v6x/nuttx-config/include/board.h:396-399` |
| PX4IO 连接 | USART6，TX PC6/RX PC7；配置为 `/dev/ttyS5`、1.5 Mbit/s。第一版 RC 另选直连 UART | `boards/px4/fmu-v6x/src/board_config.h:59-72`；`boards/px4/fmu-v6x/nuttx-config/include/board.h:393-394` |
| USB FS | DM PA11、DP PA12、VBUS PA9 | `boards/px4/fmu-v6x/nuttx-config/include/board.h:522-527`；`boards/px4/fmu-v6x/src/board_config.h:308-312` |
| FMU PWM 1–4 | TIM5 CH4/CH3/CH2/CH1，PI0/PH12/PH11/PH10 | `boards/px4/fmu-v6x/src/timer_config.cpp:36-41`；`boards/px4/fmu-v6x/src/timer_config.cpp:67-71` |
| FMU PWM 5–8 | TIM4 CH2/CH3，PD13/PD14；TIM12 CH1/CH2，PH6/PH9 | `boards/px4/fmu-v6x/src/timer_config.cpp:43-47`；`boards/px4/fmu-v6x/src/timer_config.cpp:72-75` |

## DMA、缓存与 IRQ 来源

以下是 PX4 配置线索；Rust 资源提案见 [architecture.md](architecture.md)。PX4 的 DMAMAP 宏不是 Embassy 已分配的 stream，注释中的 DMA 编号也须与宏展开区分。

| 项 | 已查明来源 / 待验证边界 | PX4 来源（文件:行） |
| --- | --- | --- |
| SPI2 DMA | RX/TX 选择 DMA12 SPI2 RX/TX 的 `_0` 映射；实际 Embassy stream 与 DMAMUX 由 BSP 分配 | `boards/px4/fmu-v6x/nuttx-config/include/board_dma_map.h:42-43` |
| UART7 DMA | RX/TX 选择 `_1` 映射；该处注释写 DMA1，不能将注释当作 stream 编号 | `boards/px4/fmu-v6x/nuttx-config/include/board_dma_map.h:65-78` |
| SPI6 / BDMA | PX4 的外部 SPI6 映射为 BDMA；当前项目没有 SPI6 数据通路 | `boards/px4/fmu-v6x/nuttx-config/include/board_dma_map.h:83-87` |
| 缓存 / TCM | NSH 配置开启 DCACHE、DTCM、ICACHE；Rust 不自动继承这些设置 | `boards/px4/fmu-v6x/nuttx-config/nsh/defconfig:67-69` |
| DMA 可达内存 | linker 注释：TCM 由 CPU/MDMA 访问；AXI/D2 不供 BDMA；DMA1/2 缓冲不得按普通数组放 DTCM | `boards/px4/fmu-v6x/nuttx-config/scripts/script.ld:66-103` |
| ICM DRDY 向量 | PA10 属于共享 EXTI15_10，不能将它当成独立 EXTI10 IRQ | `boards/px4/fmu-v6x/src/spi.cpp:91-93`；`platforms/nuttx/NuttX/nuttx/arch/arm/include/stm32h7/stm32h7x3xx_irq.h:91` |
| FDCAN1 IRQ | interrupt 0/1 对应外设 IRQ 19/21 | `platforms/nuttx/NuttX/nuttx/arch/arm/include/stm32h7/stm32h7x3xx_irq.h:70-72` |
| 定时器 / SD / USB IRQ | TIM2=28、TIM7=55、SDMMC2=124、OTGFS=101；优先级由 Rust BSP 定义 | `platforms/nuttx/NuttX/nuttx/arch/arm/include/stm32h7/stm32h7x3xx_irq.h:79,110,157,180` |

HAL 另有直接证据：`embassy-stm32 0.5.0/src/can/fd/message_ram/mod.rs:9-20` 固定每 RX FIFO 3 帧、TX 3 帧；`src/can/fd/peripheral.rs:170-195` 的 abort 含忙循环；`src/sdmmc/mod.rs:1043-1142` 含同步命令与取消等待。它们属于 **HAL 源码能力**，须在 CAN-A02/LOG-A03 适配并测量，不能以 async API 名称推断有界。

## 外接设备：用户确认、厂商规格与参考源码

下表不使用 PX4 来推断厂商产品；资料访问日期为 2026-10-31。本板的实际设备配置仍待 H-02/H-03/H-10 回填。

| 项 | 确认 / 规格 | 证据 |
| --- | --- | --- |
| ESC 型号、数量、布局 | 用户确认 **4 × Vimdrones ESC Nano，Quad X** | 2026-10-31 用户消息；[产品页](https://shop.vimdrones.com/products/vimdrones-esc-nano) |
| ESC 产品规格 | AM32 DroneCAN NANO 20A，2S–6S；120 Ω 焊盘终端默认 OFF | [Vimdrones 开发文档](https://dev.vimdrones.com/products/vimdrones_esc_nano/) |
| ESC 固件目标 | `VIMDRONES_NANO_L431_CAN`；厂商列出的 CAN bootloader 为 `AM32_L431_BOOTLOADER_PA2_CAN`。实物版本尚未读取 | 同上；[接入教程](https://dev.vimdrones.com/tutorials/arducopter_with_am32_dronecan/) |
| ESC 参考源码 | AM32 `86cf1e55e56c7e7e622d01ba6855905504f15105`；node name `com.vimdrones.esc_nano` | [esc-upstream.json](acceptance/p0/esc-upstream.json)；该仓库 `Inc/targets.h:193-205` |
| 参考 CAN 位速率 | 80 MHz PCLK /8 /(1+8+1) = 1 Mbit/s，采样点 90%；飞控可用自己的等效位时序 | AM32 `Src/DroneCAN/sys_can_stm32.c:191-219` |
| 参考默认参数 | CAN_NODE=0（DNA）、ESC_INDEX=0、TELEM_RATE=25 Hz、DEBUG_RATE=0、REQUIRE_ARMING=1、REQUIRE_ZERO_THROTTLE=1 | AM32 `Src/DroneCAN/DroneCAN.c:150-157,1251-1253,1304-1310`；不是已读实物参数 |
| 命令 / ArmingStatus | RawCommand 按 ESC_INDEX 取值；ArmingStatus 只有 255 使 `dronecan_armed=true`；输入受解锁条件及滤波控制 | AM32 `Src/DroneCAN/DroneCAN.c:683-768` |
| 参考超时 | 最后有效 RawCommand 后 >250000 us，复位输入滤波并清零；机械停转时间待测 | AM32 `Src/DroneCAN/DroneCAN.c:1339-1363`；P4b-b 必须保持其他流量测试 |
| Status 语义 | 温度 K、RPM=`e_rpm*200/MOTOR_POLES`、error_count=desync 次数、power_rating_pct=0；频率按 TELEM_RATE | AM32 `Src/DroneCAN/DroneCAN.c:1119-1145,1319-1325` |
| H-RTK 协议 | 用户确认标准 DroneCAN；目标 DroneCAN 4-pin SKU12059，UART SKU12054/12058 分开 | 2026-10-31 用户消息；[Holybro 产品页](https://holybro.com/collections/dronecan-h-rtk/products/h-rtk-zed-f9p-rover) |
| H-RTK 软件终端 | 厂商列 H-RTK Rover 序列号 **12059002*** 及以后支持 CAN_TERMINATE 控制 120 Ω；须核对实物序列号/固件 | [Holybro 软件终端说明](https://docs.holybro.com/can-termination-resistance-software-toggle) |
| H-RTK 使用范围 | 单 rover GNSS/RTK；内部 RM3100/IMU/气压计、双接收机航向、PPS 分属未来扩展 | 用户需求与 [plan.md](../plan.md)；首批功能见 GNSS-01～03/RTK-01～02 |

四电机配置回填表见 [dronecan-notes.md](dronecan-notes.md)。实际 source node ID、ESC_INDEX、位置、旋向和极数须独立记录；不能用 Quad X 名称推导现有接线顺序。

## 当前 Rust 软件对比与验收

| 项 | 当前实现 | 来源 |
| --- | --- | --- |
| 链接 | 应用0x08020000 +1920 KiB；AXI 447 KiB static、1 KiB故障栈、64 KiB共享栈；sector0不覆盖 | `memory.x`；`tools/build_report.py` |
| 向量表 | cortex-m-rt 0.7.7 启用 set-vtor/paint-stack；所有非零向量与ELF/bin校验 | `Cargo.toml:14`、`Cargo.lock`；`tools/build_report.py` |
| 时钟/供电 | HSE16 MHz、PLL1 CPU480/HCLK240/PCLK120 MHz；PLL3Q USB48 MHz；LDO/Scale1+ODEN、flash LATENCY4/WRHIGHFREQ2，I-cache开/D-cache关；平台寄存器/时基检查通过 | `src/platform.rs:20-135`；HAL `rcc/h.rs:286-296,402-427`及上述PX4来源 |
| 时基/调度 | TIM2/P1、PSC239/ARR65535、1 MHz；TIM7/P3仅软件pend；USB/P6；线程raw executor/测量idle | `src/platform.rs`、`src/diagnostics.rs`、`src/main.rs` |
| LED/加热 | 保留PHY灯；PE3红故障/PE4绿1 Hz/PE5蓝启动开漏；PB10加热保持低；本板FMU灯目视项不适用 | `src/main.rs:46-72,118-147`；[HIL](acceptance/p1/20261031-hil.md#led适用范围) |
| 软件 reboot/故障 | BKP0=b007b007复位；BKP16..31版本化CRC故障；HardFault先切1 KiB栈；共享wrapper匹配RTC/LSE，HAL后比较全部BKP及BDCR，不再补写BKP | `src/backup_domain.rs`、`src/fault.rs`、`src/core/fault_record.rs`；[故障审查](acceptance/p1/fault-review.md) |
| FDCAN1 软件配置 | PD0/PD1，HSE 16 MHz，1 Mbit/s，采样点 87.5%；IT0/1=P2，TIM6_DAC I/O executor=P4；FIFO、单次发送、自动重传关闭 | `src/drivers/dronecan_can.rs`、`src/platform.rs`；[P4b-a 软件记录](acceptance/p4b-a/README.md)，电气/HIL 待 H-02/03 |
| SPI2 IMU 软件配置 | ICM-42688-P；PI1/PI3/PI2、PH5 CS、PF4 power、PA10 EXTI；DMA1_CH2/CH3、24 MHz 请求、8 kHz ODR、8×20 B FIFO、YAW270；EXTI P0/DMA P1/TIM7 executor P3 | `src/core/icm42688.rs`、`src/drivers/imu.rs`、`src/main.rs`；[P4 记录](acceptance/p4/README.md)，ID/实际 SPI/短时计数通过，DRDY 仪器/60 s/方向和恢复待验 |
| SDMMC2 软件配置 | PC13 默认低；显式 `!logger-start` 后高电平供电；PD6/PD7/PB14/PB15/PG11/PB4 四线、P7 IRQ、内部 IDMA、FAT 单写者 | `src/storage/sd_logger.rs`、`src/main.rs`；[PL-a 软件记录](acceptance/pl-a/README.md)，卡兼容/时延/故障待 H-04 |
| TELEM1 软件配置 | UART7 RX PF6、TX PE8；115200 8N1、DMA1_CH0/CH1、Async 传输；独立序列/解析/节拍，RTS PF8/CTS PE10 未启用待 H-03 | `src/comms/mavlink_uart7.rs`、`src/main.rs`；[P2 软件记录](acceptance/p2/README.md)，线序/电平/流控待 H-03 |
| 安装 bootloader 的识别 | 实测3185:0035、`PX4 BL FMU v6X.x`、协议v5、board ID=53；构建版本未知；独立QGC往返及普通回装通过 | [HIL](acceptance/p1/20261031-hil.md) |
| QGC / Inspector 实测 | QGC 5.1.5连接Rust；截图确认HEARTBEAT、SYS_STATUS、SYSTEM_TIME；AUTOPILOT_VERSION固件已就绪，连接/重连显示待 HIL 复核 | [QGC记录](qgc-notes.md)、[P2 软件记录](acceptance/p2/README.md) |

P1验收包source=`a946a529360eac83`；构建/资源/hash见[P1索引](acceptance/p1/README.md)，实际测量与恢复见[HIL](acceptance/p1/20261031-hil.md)。最近实测板载为普通 Rust source=`81d6c1221048f08a`；RTC、正常 SD/短时 IMU 日志已有有限 HIL，新标准消息软件见[记录](acceptance/messages/README.md)，未完成实物项仍按阶段验收。P4p INA219 实测板载为 `b2e70b0ecc7ab844`：单条 `DEBUG_VECT` V/A/W 在 Windows Inspector 同显、ULog `rust_current_sensor` 干净短日志 10 Hz 通过，见 [P4p](acceptance/ina219/README.md)。

## 验收与后续待核实

| 项 | 状态 / 归属 |
| --- | --- |
| QGC 刷回 PX4、再刷 Rust，含不依赖应用命令的 bootloader 入口 | **通过**：独立USB重插、Stable上传/校验、PX4 v1.17.0运行、最终普通Rust回装，见[HIL](acceptance/p1/20261031-hil.md) |
| 原 FRAM 参数、EEPROM 校准/身份的备份与恢复边界 | P3a；先只读识别/备份，保留官方 PX4 恢复资料 |
| 核心供电/VOS/flash latency、DMA 可达性、cache/MPU、资源冲突 | P1平台寄存器通过；DMA/cache/MPU与新增外设资源按PL/P4等阶段实测 |
| 实物传感器型号、数量、地址、方向与 DRDY | P4.0；当前只有源码预期，没有本次实测清单 |
| CAN 收发器型号、静默或 RX-MUX 控制、终端电阻、插座电源与针序 | 未核实；缺少对应实物/原理图证据，不推断存在或不存在控制 GPIO |
| ESC 实物固件、参数、node ID/ESC_INDEX、Status 周期、命令超时、旋向/极数 | 已确认 4 个 Vimdrones ESC Nano / Quad X；其余由 H-02、P4b/P5 实测 |
| H-RTK SKU/序列号/固件、节点与消息配置、RTCM 来源 | 用户确认标准 DroneCAN；H-10、P4b-g/r 实测 |
| SD 卡槽、卡型号/容量、卡检测和掉电行为 | 工程师提供卡；PL-a 实测，不以 PX4 配置替代 HAL/卡兼容性验证 |
| TELEM1/直连接收机的线序、电平、流控/反相 | P2/P5b 按本板与所选设备核实 |

P0、P1、P3a、RTC-01/TIME-01 已完成；P4b-a、PL-a、P2、P4 软件门通过，有限 HIL 已记录；完整实物门待 H-02/03/04 与工程师操作，M1 未开始。阶段状态见[TODO](../TODO.md)。
