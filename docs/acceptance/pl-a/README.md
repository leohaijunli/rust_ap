# PL-a ULog / SD 软件验收

日期：2026-10-31。**LOG-A01～04 软件开发通过；LOG-A05 与实物 SD 验收待 H-04，PL-a 尚未关闭。P4b-a 依赖也仍未放行。**

## 已通过

| 检查 | 结果 |
| --- | --- |
| ULog | 纯 Rust/no_std 显式小端编码；Header、B/F/I/P/A/D/O、同步及兼容 padding；13 个 M1 topic 的格式长度与预算一致 |
| 独立解析 | 固定 `pyulog==1.3.0`；Rust 生成全 topic fixture，完整文件与尾部截断文件均解析通过，无 corruption |
| 有界入口 | 单一 1024 槽 MPSC 队列，总预算 ≤136 KiB；普通记录严格保留 ≥8 KiB 事件容量；dropout 与首条恢复记录成对提交 |
| 并发失败路径 | 多生产者、满队列、事件保留区满、恢复标记、`u32` 位置回绕通过；dropout 元数据仅用极短 critical section，记录复制不在临界区 |
| SDMMC2 | PC13 默认低且不访问卡；工程师显式 `!logger-start` 后供电。PLL1Q=48 MHz，SDMMC2 4-bit/P7，目标卡时钟 25 MHz（实际候选 24 MHz） |
| FAT/文件 | 固定 `embedded-fatfs` commit `518528cc…`；支持 MBR FAT 与 superfloppy；只创建新 `NNNNNNNN.ULG`，不覆盖/删除旧文件 |
| 写入/sync | 32 KiB、32 B 对齐 staging；定义区和每次 1 s sync 用兼容未知消息补齐 512 B，保持多块写条件；显式 file/fs flush |
| 数据源 | 零电机命令、DISARMED、CAN/ESC/节点状态与 stale/offline、参数 dirty/persisted、runtime health 已接入；未知 CPU/栈/调度值带有效性 |
| 软件检查 | fmt、普通/注入/Dump target Clippy、host Clippy、42 项 Rust 测试；pyulog 完整/截断检查；17 项编译时钟谓词 |
| release/打包 | Linux Rust 1.99.1；source=`4c18e2081a0874b5`；raw/padded=`335368/335392 B`；static RAM=`215560 B`、空余 `242168 B`；布局/向量/ELF/bin/PX4 一致 |

普通 `.px4` SHA-256：`02c3c74a1a7fd1c54de939c8bbeb724b8dfac8ca6be34b1f7e2333b8eb7a849c`。本轮没有烧录、访问 SD 卡或改变当前板载 P3a 固件。

## 集中硬件测试

- H-04：工程师提供卡型号、容量、MBR/FAT32 或 superfloppy 布局、cluster/空闲空间，并确认 1 s sync 与允许丢失窗口。
- LOG-A03/04：确认 PC13 供电、SDMMC2 4-bit、实际时钟/IDMA 可达内存、CMD25、多块写吞吐和 32 KiB p99/最大时延。
- LOG-A05：一份实物日志由 pyulog 独立解析；一次受控写入断电后恢复完整前缀；一次缺卡/卡忙/写入中拔卡，确认 CAN/P1 快路径继续并量化 USB/线程停顿。
- HAL 0.5.0 的 ACMD41、wait-idle 和取消路径含同步循环。默认不启动避免无卡影响；显式测试后仍须以实测判定，不能用 async timeout 声称可抢占。

日志在线列表/下载/删除仍归 PL-b；真实 IMU/AHRS/GNSS topic 随对应数据源阶段接入。当前结果仅允许继续软件开发，不放行依赖 PL-a 的硬件阶段。
