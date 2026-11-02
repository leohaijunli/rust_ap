# 时间基准：RTC、GNSS 与日志 UTC

更新时间：2026-11-02。上位约束见 [plan.md](../plan.md)，当前状态见 [TODO.md](../TODO.md)。

## 1. 现状

- 驱动：`src/drivers/rtc.rs`，直接读写 STM32H753 的 RTC/BDCR 寄存器，**不调用** `Rtc::new`、不置 `BDRST`，PX4 备份字和 QGC 恢复路径不受影响。
- **读日历的两个硬性约束（RTC-01 根因）**：`RTC_TR`/`RTC_DR` 是影子寄存器，只有软件清除 `RSF` 后硬件才重新同步。清除 `RSF` 是对写保护寄存器 `RTC_ISR` 的写入，**必须先解开 `WPR`**（依次写 `0xCA`、`0x53`）。同时，读取 `SSR` 或 `TR` 会锁住高位影子寄存器，直到读取 `DR` 才释放，所以快照必须以 `SSR → TR → DR` 的顺序结束；不能在 `DR` 后为做一致性检查再读一次 `SSR`。任一约束遗漏都会出现 `SSR` 递增而 `TR`/`DR` 停在旧值的假“日历冻结”。
- INIT 残留：`INIT` 位在 RTC 域内，MCU 复位不会清除；上一次在 RTCCLK 未到达时写日历会把 `INIT` 留在置位状态，同样让日历冻结。`init()` 与每次写日历前调用 `release_init_mode()` 主动清除并等待 `INITF` 回落。
- 启动判定：BDCR 的 `LSEON/LSERDY/RTCSEL=LSE/RTCEN` 不能证明 RTCCLK 到达 RTC，`SSR` 变化也不足以证明日历在走。`init()` 要求 `ticking()`（SSR 变化）**和** `counting()`（日历秒在 1.5 s 窗口内真的递增）同时成立，然后依次判定：
  1. `inherited`：沿用启动时的源，两项都成立就不改动；
  2. `lse-high`：`LSEDRV=HIGH` 重启 LSE 后重新选源，两项成立则采用；
  3. `lsi`：改用内部 LSI（`RTCSEL=LSI`、`PRER=127/249`），并关闭已确认无用的 LSE，避免下次启动 HAL 等待 `LSERDY` 卡死；
  4. `inherited-frozen`：回退全部失败但日历已是 ≥2020 的可信日期，只报告不覆盖；
  5. `none`：全部失败，`!time` 如实报告 `ticking=false`。
- 归档语义：恢复时保留格式有效的日历；只有日历本身无效才写入 2000-01-01（合法但不可信）。`wall_clock::plausible_unix` 以 ≥2020 判定“已授时”。
- 写日历：`RTC_PRER`/`TR`/`DR` 在 `WPR` 解锁后于 `INIT` 模式内写入，退出后再刷新影子；`!time set` 用 200 ms 的有界轮询回读，避免刚写完、影子尚未同步时误报失败。
- 切换时钟源按“清 `RTCEN` → 写 `RTCSEL` → 置 `RTCEN`”三次独立写完成，满足 `RTCSEL` 只能在 RTC 关闭时改写的要求。
- 诊断：`!time` / `!status` 输出 `RTC schema=3`：
  `source`、`ticking`、`counting`、`probe`、`prescaler_ok`、`recovered`、`stale_init`、`calendar`、`unix`、`valid`、`bdcr_before/after`，以及**实时**原始寄存器 `isr/prer/tr/dr/ssr`。

### 排查历程（不要重复的弯路）

1. 先按 BDCR 位判断“LSE 已就绪”，`!time` 显示 `running=true` 但日历不动 → 误判为晶振停振；
2. 再以 `SSR` 边沿验证 RTCCLK，`ticking=true` 但日历仍不动 → 误判为 `INIT` 卡死；
3. 第一个读协议问题是未解锁 `WPR` 就清 `RSF`，该写入被忽略；
4. 解锁后仍偶发冻结，最终定位到 `DR` 后再次读 `SSR` 会重新锁住影子。改为所有快照均以 `SSR → TR → DR` 结束后，同一块板的 LSE 连续走时且 `!time set` 回读成功。

### 2026-11-02 实测结论

- source=`81d6c1221048f08a`，raw/padded=`370064/370080 B`，static RAM=`231704 B`；经原 PX4 bootloader 上传、校验并运行。
- `!time` 连续读取与 30 s 间隔读取均为 `source=lse ticking=true counting=true prescaler_ok=true`，`unix` 按秒递增；`PRER=0x007f00ff`。
- `!time set 1700000000` 返回 `ok=true`；随后 `SYSTEM_TIME.time_unix_usec` 非零并递增。
- 新日志 `LOG_ENTRY.time_utc=1700000004`，不再是 `UnknownDate`；QGC GUI 文件名显示仍由工程师后续确认。


### 时钟源精度

| 源 | 频率 | 典型精度 | 日漂移 | 适用 |
| --- | --- | --- | --- | --- |
| LSE 晶振 | 32.768 kHz | ~±20 ppm | < 2 s | 首选，有电池时跨断电保持 |
| LSI RC | ~32 kHz | ±5% | 可达 1 h | 仅作 LSE 失效的后备，必须由 GNSS 周期校准 |

## 2. 时间入口与出口

| 方向 | 接口 | 规则 |
| --- | --- | --- |
| 入口 | MAVLink `SYSTEM_TIME.time_unix_usec` | 仅当落在 2020-01-01～2099-12-31 且与当前值差 >1 s 时写 RTC |
| 入口 | Console `!time set <unix\|ISO8601>` | 同上范围，回读误差 ≤1 s 才算成功 |
| 入口（后续） | DroneCAN GNSS `time_utc_usec` | 见第 3 节 |
| 出口 | MAVLink `SYSTEM_TIME.time_unix_usec` | 取 RTC；日历不可信时输出 0（QGC 不视为有效时间） |
| 出口 | `LOG_ENTRY.time_utc` | 文件 mtime（FAT 时间戳）换算 UNIX；<2020 输出 0 → QGC `UnknownDate` |
| 出口 | FAT 时间戳 | `FsOptions::time_provider(WallClock)`，写入/关闭时取当前 RTC |

## 3. GNSS 接入方案（P4b-g 起）

1. **数据来源**：H-RTK DroneCAN（SKU12059）解析后形成固定 PX4 标准 `sensor_gps` 快照，授时使用 `time_utc_usec`、`fix_type`、`eph`、`timestamp`；完整字段遵循[消息约定](message-contracts.md)。
2. **采纳条件**（全部满足才写入）：
   - `fix_type >= 3`（3D fix），且 `time_utc_usec` 落在 2020-01-01～2099-12-31；
   - 样本新鲜（`now - timestamp < 1 s`），且 `eph` 在可用范围内（阈值在 P4b-g 测试前固定）。
3. **写入策略**（与 PX4 v1.17 `GPS::setClock` 一致，并补充 LSI 场景）：
   - 偏差 `>= 5 s` 才写（`SET_CLOCK_DRIFT_TIME_S=5`）；
   - 源为 `lsi` 时额外每 60 s 或偏差 `>= 1 s` 校正一次（LSI 日漂移可达小时级）；
   - 写入失败只累计计数并在 `!time` 体现，不阻塞采样、不重试风暴。
4. **优先级**：GNSS > 地面站 `SYSTEM_TIME` > 继承的 RTC 值。无 GNSS 时才采纳地面站时间；两者都无时保持 RTC 现值，不回退到 2000-01-01。
5. **日志关联**：
   - 记录开始时若 UTC 可信，写入等价 PX4 的 `boot_time_utc_us` 信息项（ULog info）；
   - 文件 mtime 由文件系统在写入/关闭时取当前 RTC，QGC 用 `LOG_ENTRY.time_utc` 命名；
   - **不回填**已关闭日志的日期/名称，与 PX4 行为一致。
6. **不做的事**：不改写 ULog 内部 boot 相对时间戳；不把“无 GNSS”当故障告警；不在记录态触发列表/下载。

## 4. 验证清单

| 项 | 方法 | 判据 |
| --- | --- | --- |
| RTC 走时 | `!time` 间隔 30 s 读两次 | `ticking=true`，`unix` 递增约 30 |
| 走时跨重启 | `!time set` → 复位 | 日期保持（有备份电池时；无电池则回到 2000-01-01 并由 GNSS/GCS 重新授时） |
| 日志 UTC | 记录 10 s 后 `LOG_REQUEST_LIST` | `LOG_ENTRY.time_utc >= 1577836800`（已授时）或 0（未授时） |
| QGC 命名 | Log Download → Refresh | 已授时文件显示日期，未授时显示 UnknownDate；协议时间已通过，GUI 显示待确认 |
| GNSS 授时（后续） | 有星后看 `SYSTEM_TIME`/`!time` | `time_unix_usec` 与 GNSS UTC 一致，偏差 <1 s |
| LSI 漂移（后续） | LSI 模式 24 h 记录 | 未校准漂移符合 ±5% 预期，60 s 校准后 <1 s |
