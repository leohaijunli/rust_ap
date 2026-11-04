# P4b-g：H-RTK GNSS 接收

日期：2026-11-03。范围见 [plan.md](../../../plan.md#p4b-g--gnss-0103)；实现按 **PX4 对齐原则**（[plan.md 实现原则](../../../plan.md#3-m1-范围与实现边界)）：数据边界与换算严格对照固定 PX4 v1.17.0 `gnss.cpp` process_fixx 与 `GPS_RAW_INT.hpp`。

## 软件实现（提交 `61b2c22`，source=`588ab77941985319`）

- **Fix2(1063)/Auxiliary(1061) 解码**（`src/core/dronecan/gnss.rs`）：UAVCAN v0 DSDL 位序为字节内 MSB-first、多字节小端；`covariance` 为 6 位长度前缀动态数组（非 TAO）。4 项 host 测试与 P0 参考向量（`fix2_covariance_0/1`、`fix2_rtk_float`、`gnss_auxiliary`）逐字段对拍。
- **`process_fix2`**（`src/core/gps.rs`）：翻译 PX4 `process_fixx`——covariance 仅 1（标量）/6（对角）有效、其余按未知（-1）；Auxiliary DOP 2 s 内有效否则回退 Fix2 `pdop`；`c_variance_rad` 用速度 Jacobian（对角阵无交叉项）；UTC 按 GNSS/TAI/GPS 三种时间标准换算；`heading` 等无数据源字段保持 NaN，`noise_per_ms`/`jamming_indicator` 保持 -1。
- **标准 `sensor_gps` ULog**：完整 37 字段（147 B payload），副本 `tools/px4_msgs/SensorGps.msg`。日志队列扩容（`RUNTIME_RECORD_LEN=152`、`QUEUE_BYTES=168 KiB`、`MAX_RECORD_LEN=800`）承载完整标准字段；static RAM=`268680 B`。
- **`GPS_RAW_INT` 遥测**：USB/TELEM1 各 5 Hz，按 PX4 `GPS_RAW_INT.hpp` 换算（`eph/epv`=DOP×100、`h_acc/v_acc/vel_acc`=米×1e3 饱和、`vel`=cm/s、`cog`=cdeg）；数据停止 1 s 后 1 Hz `NO_GPS` 帧。
- **no_std 浮点**：新增 `libm 0.2.16`（MIT OR Apache-2.0，见 [provenance](../provenance.md)），host 测试与板上共用同一实现。

## 检查

| 项目 | 结果 |
| --- | --- |
| Rust host | 97 项通过（含 gnss 4 项向量对拍、gps 5 项 PX4 语义） |
| Python 相关 | 29 项通过（build_report/package/mavcan/p3a/decode） |
| fmt / Clippy | host+release、`-D warnings` 通过 |
| 打包/布局 | raw/padded=`412624/412640 B`，static RAM=`268680 B`，build_report 通过 |

## 板载验证（2026-11-03）

经原 PX4 bootloader 上传/校验（COM12，board 53），运行 source=`588ab77941985319`：

1. 心跳 `sys=1 autopilot=12`（PX4 兼容）、disarmed。
2. **`GPS_RAW_INT` 断链帧验证通过**：GPS 未接时 3.2 s 捕获 3 帧（1 Hz），`fix_type=0`、`eph/epv/vel/cog=65535`、`satellites_visible=255`，与 PX4 `GPS_RAW_INT.hpp` 的 NO_GPS 语义逐字段一致；发送链路（sensor_gps 快照缺省 → NO_GPS 分支 → USB MAVLink）实测连通。

## 待验（GPS 连接后）

- Fix2/Auxiliary 实际速率与 `!dronecan` GNSS 行（fix/sats/lat/lon/age）。
- QGC Inspector `GPS_RAW_INT` 正常帧（经纬度/高度/卫星数/eph 合理）。
- `sensor_gps` 日志 pyulog 独立解析（单位/时间戳/fix_type/速率）。
- 一次断线失效/恢复（含 NO_GPS 帧切换）。
- TIME-02 GNSS 授时（fix≥3 且样本新鲜，偏差 ≥5 s 写 RTC，见 [时间基准](../../time-sync.md)）。
- dropout 复验：GPS 日志加入后跑 [analyze_dropout.py](../../../../tools/analyze_dropout.py)（根因定量分析见 [logging-budget](../../logging-budget.md#31-dropout-定量分析2026-11-03)）。
