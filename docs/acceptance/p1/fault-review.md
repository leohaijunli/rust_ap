# P1 HardFault 静态审查

日期：2026-10-31。此为 release ELF/源码审查，**不替代硬件故障恢复验收**。备份域修复后的普通`a946a529360eac83`/注入`0de073823b066c6e`已重核；产物/hash见[索引](README.md)，完整链见[普通](20261031-backup-domain/fault-chain-normal.txt)/[注入](20261031-backup-domain/fault-chain-injection.txt)。

## 入口与帧

- vector table 位于 `0x08020000`，HardFault 非零 Thumb 向量指向裸入口；`tools/build_report.py` 检查全部非零向量及 ELF/bin/容器一致性。
- 入口先 `cpsid i`，保留 EXC_RETURN，以 bit2 选择原 MSP/PSP，随后 `msr msp, 0x24070000`、ISB、直接分支到 Rust。切栈前没有 push、调用或局部栈使用。
- 独立栈 `0x2406fc00..0x24070000` 共 1024 B；共享栈 `0x24070000..0x24080000` 共 65536 B；普通 static allocation 不越过独立栈起点。
- `frame_is_readable` 限定合法 ARMv7-M EXC_RETURN、4 B 对齐、完整 32/104 B 栈帧位于共享应用栈，拒绝 stacking/unstacking/lazy-FP 错误。核心帧从 SP 开始，LR/PC/xPSR 为字偏移 5/6/7，扩展 FP 区在其后。参考固定 NuttX `arch/arm/src/armv7-m/arm_exception.S:118-137`（pin 见 provenance）。
- 反汇编先做完整边界和 CFSR 检查，再执行原帧三个 load；不能读取时 detail=0、PC/LR/xPSR 保持 0，仍尝试记录 CFSR/HFSR/MMFAR/BFAR/SP/EXC_RETURN。

## 当前普通/注入 release 调用链与栈

从 LLVM 反汇编逐项计算最大同时存活的 SP 下移；包括 push、pre-index str 与 sub sp。函数由 LTO 优化，结论仅适用于索引中的精确产物。

| 函数 | 本地 SP 下移 | 调用/行为 |
| --- | ---: | --- |
| 裸 HardFault 入口 | 0 B | 切 MSP，b 到 Rust |
| `fault::hard_fault` | 72 B | push 8 + sub 64；初始化 record 的 `__aeabi_memclr4` 8 B；调用 recover |
| `fault::recover` | 8 B | 调用 seal、write_record、reboot（顺序，不同时叠加） |
| `FaultRecord::seal` | 16 B | 调用 checksum |
| `fault_record::checksum` | 36 B | push 20 + str r11 4 + sub 12；叶函数，15×4×8 次 CRC 位处理 |
| `fault::write_record` | 8 B | 叶函数；DBP 检查最多10000次；invalidate magic、写15字、DSB、commit magic、DSB |
| `fault::reboot_to_bootloader` | 8 B | 有界 DBP 检查、BKP0=`b007b007`、DSB；调用 sys_reset |
| `SCB::sys_reset` | 8 B | AIRCR SYSRESETREQ + DSB；等待复位的叶循环 |

最深正常调用链：`hard_fault → recover → seal → checksum`，合计 **132 B**，相对 1024 B 预留剩余 **892 B**。`memclr4` 来自 Rust compiler-builtins，不是引入 C/C++ 驱动/运行库。已分别核对普通/注入 release：这组函数本地栈大小相同，但地址不同；归档各自完整调用链片段 `fault-chain.txt`，不能仅按普通包地址符号化注入故障。

上述路径未见递归、间接函数调用、fmt/USB/heap/定时器/锁调用，CRC 和备份写入为有界循环；sys_reset 等待硬件复位不设置软件超时。只计算软件调用栈：NMI 嵌套、硬件额外入栈、设备访问再故障/lockup、损坏 flash 或备份域不可写等不在成功保证内。

## 保持与限制

故障格式：16个小端u32，magic/version=`50314631`，kind/source/detail/file_hash、核心帧、故障寄存器、EXC_RETURN、IEEE CRC32。magic 最后提交，host 测试覆盖所有单 bit 损坏和中断提交；运行时只接受 kind 与 CRC 均合法的记录。

HAL 前缓存有效故障与复位原因；共享`backup_domain::init`另行快照全部32个BKP字及BDCR/CSR，匹配低速配置与RTC mux以避免BDRST，HAL后比较而不补写BKP。普通/注入版保持失败会进入既有故障路径；Dump报告失败结果。故障使用 BKP16..31，BKP0用于原bootloader请求。这不是已与官方 PX4 所有BKPx用途兼容的分配保证，切回 PX4 的保留行为不作为判据。合成回放和板上验收范围见[备份域证据](20261031-backup-domain.md)。

**只有 HardFault 裸入口切应急栈。** panic/DefaultHandler/stack guard 路径在共享栈上先构造记录；任意栈损坏或锁死不能保证软件恢复。需按 [操作单](bring-up.md) 实测受控 panic/udf 注入、持续供电的复位/回装保持与独立 QGC 恢复；不故意刷坏镜像测试。
