# 标准 PX4 / MAVLink 消息软件检查

日期：2026-11-02。**标准消息软件检查通过，source=`c2f3128aecc8a70b`。** P4/PL-a/P2/M1 的硬件门仍按 [plan.md](../../../plan.md) 收口。

## 修改与来源

- `src/core/px4_messages.rs` / ULog：完整 `sensor_accel`、`sensor_gyro`、`actuator_armed` 字段；FRD/SI、发布时间/采样时间、DeviceId、单样本裁剪/真实错误计数。
- 三个固定 `.msg` 与 LICENSE 来自 PX4 v1.17.0 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`，见 `tools/px4_msgs/`；[消息约定](../../message-contracts.md) 与[来源](../../provenance.md)。
- MAVLink 已使用 rust-mavlink 0.17.1 `common` 标准消息；本次复用既有协议证据。
- ULog 只注册 9 个已接入 topic；原始 DroneCAN / Rust 诊断采用独立前缀。IMU 每 8 样本抽取 1 个，各 topic 1 kHz、`samples=1`。
- 局部简化审视：共用 accel/gyro 的一致线布局编码，移除未接入的日志 schema，原逐字节镜像测试由独立 pyulog/PX4 输入校验替代。

## 检查

| 项目 | 结果 |
| --- | --- |
| fmt / 普通 target Clippy / host lib Clippy | 通过，Clippy `-D warnings` |
| 受影响 ULog 核心用例 | 4 passed；格式/长度、有界注册表、参数信息、逐 topic 时间倒退拒绝 |
| 独立 pyulog 1.3.0 | 固定 PX4 `.msg` 字段、FRD 数值/采样时间/DeviceId/裁剪/布尔状态，完整与尾部截断 fixture 全部通过、无 corruption |
| 普通 release / 打包 / 布局向量 | 通过；board 53、`0x08020000`、ELF/bin/PX4 一致，raw/padded=`370496/370496 B`，static RAM=`232184 B`、空余 `225544 B` |
| M1 预算复算 | 164322 B/s，25% 余量后 205403 B/s；3034 条/s，现有 960 普通槽的保守窗口约 316 ms |

本轮首次 `cargo fmt --check` 在检出 `33533e6` 上报 `src/core/ulog.rs:621` 多余闭合括号，已修复；该失败不改写 `81d6c1221048f08a` 的已存 HIL。

验证命令：

```sh
cargo fmt --check
cargo test --lib --target x86_64-unknown-linux-gnu --locked ulog::tests
cargo clippy --lib --target x86_64-unknown-linux-gnu --locked -- -D warnings
cargo run --example ulog_fixture --target x86_64-unknown-linux-gnu --locked -- target/standard-messages.ulg
.venv/bin/python tools/check_ulog.py target/standard-messages.ulg
cargo clippy --release --locked -- -D warnings
./tools/firmware.sh build
python3 tools/p0/budgets.py --profile m1
```

关键原始输出摘录：

```text
test result: ok. 4 passed; 0 failed; 0 ignored; 0 measured; 55 filtered out; finished in 0.00s
PX4 schemas, FRD sensor fields, pyulog full and truncated-tail checks: PASS
Packaged 370496 raw bytes as 370496 flash bytes
Board ID: 53; application address: 0x08020000
source=c2f3128aecc8a70b; report=/home/dev/work/px4/target/thumbv7em-none-eabihf/release/v6x-rust-led.build.json
```

产物在 `target/thumbv7em-none-eabihf/release/`（本地构建目录）：`.px4` SHA-256=`0d1a2d7ad92cb3dd1dd96425fa735e556ece9e7843ac984f4d8502ea57ac3ddf`，`.bin` SHA-256=`ad1dd2ae5ccd5e08dd170d890e6c02fb51357c59b1bd73b87158209f7b0a6f6f`。该软件包的硬件验证待工程师执行。

## 下一步

最近实测板载仍为 `81d6c1221048f08a`，短时日志使用旧 `imu_icm42688`。新 schema 实物核对复用下一次 P4 60 s/六面必验记录；CAN 映射、异常卡、双链路/QGC、GNSS 授时和 M1 依赖门继续待验。restart 原有通过标记先补直接证据引用。
