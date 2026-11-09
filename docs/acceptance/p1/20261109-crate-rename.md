# Crate 改名：v6x-rust-ap

日期：2026-11-09。按工程师要求，将 package/bin `v6x-rust-led` 改为 `v6x-rust-ap`，Rust 导入名改为 `v6x_rust_ap`。本轮不烧录，不更改估计器、参数、USB VID/PID/serial、原 bootloader、board ID 53 或 `0x08020000` 应用入口。

## 修改范围

- 同步 Cargo.toml/Cargo.lock、链接 map 名、板级导入和宿主例程、Windows/Linux 构建/上传/校验/报告工具、CI 产物路径；依赖版本与算法保持。
- 新 ELF 为 `target/thumbv7em-none-eabihf/release/v6x-rust-ap`，对应 `.bin/.px4/.map/.build.json` 使用相同前缀；打包 `git_identity=v6x-rust-ap`。
- 历史验收文件、原始日志/包与 source/hash 不改写；忽略的个人 IDE workspace 未改，已有 `.run/.vscode` 入口仍复用脚本。
- 基线是工程师已有提交 `3632b28`（PX4 reverse-timer at_rest）；本轮未改该实现，不用较早 `d21b326c3a753d15` 的板载结果冒充此版实测。

## 检查与产物

- `cargo test --locked --target x86_64-pc-windows-msvc --lib`：229 通过；host 库/测试及实际使用的 `ekf2_input/ekf2_standard/parameter_catalog` Clippy `-D warnings` 与构建通过。
- 普通与 `fault-dump` H753 Clippy、fmt、Git Bash `firmware.sh` 语法检查通过；未默认运行 all-targets/all-features 或重做未变数学的黄金对拍。
- 编译后的参数元数据 `--check` 通过；一次局部 `code-simplifier` 审视及逐文件核对确认源码仅改包名/导入，不引入兼容层，已有反向计时实现与依赖版本保持。
- 相关 Python 工具 46 项执行，45 通过、1 跳过。初次 Python 3.10 缺 `TestCase.enterContext`、默认 bash 误用 WSL；本轮临时按标准接口提供测试框架兼容方法并明确使用已安装 Git Bash，不改产品代码或用例断言。Windows 缺符号链接权限，Unix by-id 别名用例保留待 Linux CI，不宣称其本机通过。
- 普通构建/打包/分区/入口帧通过：source=`136329bb61f63100`，构建 `git_head=3632b28b34e930d7b87cd1471599d5dcee3c4528`，另含本轮改名；raw/padded=1220552/1220576 B、静态 RAM 374464 B（余 83264 B）、入口帧最大 11528 B、oversized=0。
- `.px4` 776945 B，SHA-256 `07767ef1cd232b399eb712b1f95552eff0ea1c3e35b792511758dadf944d9dc7`；报告 `target/thumbv7em-none-eabihf/release/v6x-rust-ap.build.json` 与 `stack-frames.json`。新 `git_identity` 和 board ID 已核对。

## 后续

- 本轮未开串口、刷写或改变板载状态；现有板载结果保留，不因 crate 改名自动重验或放行 E7/M1。
- [RTK/CAN/地面站独立步骤](../../rtk-can-validation.md)已成文；RTCM 转发、动态 ID 与 RM3100 接收仍未实现，后续开发/实体操作须按原阶段门另获授权。
