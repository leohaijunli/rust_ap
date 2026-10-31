# P0 软件探针与预算复现

本目录用于 host/交叉编译调研，结果索引见 [P0](../../docs/acceptance/p0/README.md)。脚本和探针均不打开板上设备；本板验收使用各阶段的工程师操作单。

以下命令从项目根目录运行。当前工具链为 **Rust 1.99.1**，须有 `thumbv7em-none-eabihf`、host target、rustfmt 和 llvm-tools；`rust-objdump` 由 cargo-binutils 0.4.0 提供。两个探针各有自己的 `Cargo.lock`，使用 `--locked`，与主固件依赖分开解析。

终端尚未加载 Rust 环境时，先运行 `source "$HOME/.cargo/env"`。

## 1. Cybflight core 独立 M7 / ABI

`core-abi/` 直接依赖 Cybflight commit `806af5648c336e7af81abefea31052db783387f5`，不继承其应用 BSP/时基/链接布局。复现到独立产物目录：

```sh
CARGO_TARGET_DIR=/tmp/opencode/rust-ap-p0-reproduce \
cargo +1.99.1 rustc --manifest-path tools/p0/core-abi/Cargo.toml \
  --lib --release --locked --target thumbv7em-none-eabihf -- --emit=obj
rust-objdump --full-contents --section=.rodata.P0_CORE_ABI \
  /tmp/opencode/rust-ap-p0-reproduce/thumbv7em-none-eabihf/release/deps/p0_core_abi-*.o
```

该 section 是 12 个小端 `u32`，依次为 INDI、ESKF、Simple SQP、Full SQP、BFGS workspace、MincoSnap 的 `(size, alignment)`。预期见 [core-abi.json](../../docs/acceptance/p0/core-abi.json)：1852、1068、35000、54788、55788、85616 B，align 均为 4。它们不包含临时栈/任务 future，也不是 M1 RAM 分配。

上游 Mahony 行为测试已包含在 Cybflight core 基线；完整命令见 [porting-notes](../../docs/porting-notes.md)。本 ABI 探针不测板上算法时序。

## 2. umi DroneCAN 异常诊断

`dronecan-probe/` 依赖 commit `0f5ffc906aa4527e57b8db343a7f133874eece27`：

```sh
CARGO_TARGET_DIR=/tmp/opencode/rust-ap-p0-reproduce \
cargo +1.99.1 run --manifest-path tools/p0/dronecan-probe/Cargo.toml \
  --locked --target x86_64-unknown-linux-gnu
```

该固定版本预期输出四个 `true`：短首帧 panic、错误 CRC 被接受、非法初始 toggle 被接受、服务 node 0 被接受。`catch_unwind` 捕获的 panic 仍会在 stderr 输出；进程 exit=0 表示诊断完成，**不是协议通过**。

上游 std 的 8 项测试通过、无 alloc M7 编译失败的原始命令和结果保存在 [probe-results.json](../../docs/acceptance/p0/probe-results.json)。候选缺陷与后续对拍范围见 [DroneCAN 记录](../../docs/dronecan-notes.md)。本轮不重复运行已记录的 Holsatus 依赖失败。

Rust 格式核对：

```sh
cargo +1.99.1 fmt --manifest-path tools/p0/core-abi/Cargo.toml -- --check
cargo +1.99.1 fmt --manifest-path tools/p0/dronecan-probe/Cargo.toml -- --check
```

## 3. DroneCAN 参考向量

`reference_vectors.py` 从两个本地、固定且无源码改动的仓库读取定义；可以换目录，commit 须匹配：

| 输入 | URL | commit |
| --- | --- | --- |
| pydronecan 1.0.27 | https://github.com/dronecan/pydronecan | `092d09cb9e6b115e262efe4cb42f7b65e6054f26` |
| 官方 DSDL | https://github.com/dronecan/DSDL | `b4653c7abc3c47cb31b16efa24ea755232774756` |

Python 基线为 3.14.8，项目 `.venv` 的依赖见 `tools/requirements.txt` 与 [环境记录](../../docs/acceptance/p0/environment.txt)。无需安装另一个浮动版本的 `dronecan` 包，脚本直接导入指定源码并显式加载指定 `uavcan/` 定义。

```sh
.venv/bin/python tools/p0/reference_vectors.py \
  --pydronecan /tmp/opencode/rust-ap-p0-20261031/pydronecan \
  --dsdl /tmp/opencode/rust-ap-p0-20261031/DSDL \
  --output /tmp/opencode/dronecan-vectors-reproduced.json
cmp docs/acceptance/p0/dronecan-vectors.json /tmp/opencode/dronecan-vectors-reproduced.json
```

预期为 **40 个参考样本 + 3 个拒绝样本**；输出包含 pins、DSDL 文件 hash、类型签名、payload 和经典 CAN 帧。回读只证明参考数据生成一致；Rust、传输会话与 HIL 验收仍归 CAN-A01/04。

## 4. CAN / ULog 预算

`budgets.py` 只依赖 Python 标准库，**默认 profile 为 `m1`**。历史 `budget-default.json` 必须显式指定 `exploration` 才能复现。

```sh
python3 tools/p0/budgets.py --profile m1 > /tmp/opencode/budget-m1.json
python3 tools/p0/budgets.py --profile exploration > /tmp/opencode/budget-default.json
python3 tools/p0/budgets.py --profile exploration --esc-count 8 --esc-slots 8 > /tmp/opencode/budget-eight-esc.json
python3 tools/p0/budgets.py --profile exploration --esc-count 8 --esc-slots 20 > /tmp/opencode/budget-sparse-esc.json
```

逐个 `cmp` 对照 `docs/acceptance/p0/` 中同名文件。预期 CAN 含重试额度为 33.028%、39.428%、58.892%、78.092%；ULog 分别为 111062、142793、158193、177393 B/s。

参数含义：`--esc-count` 是实物数量，`--esc-slots` 是最高 esc_index+1（包含空槽）；两者均影响预算。`--command-hz`、`--status-hz`、`--arming-hz`、`--fix-hz` 可作敏感性分析。脚本以 1 Mbit/s、保守经典扩展帧位填充和额外 10 个百分点错误/重试额度计算，不能证明实际延迟。

三 float32/200 Hz 的独立容量情景见 [DroneCAN 文档](../../docs/dronecan-notes.md)，尚未加入默认配置。修改实际 DSDL、topic 线格式或频率后应重新计算并记录 profile；旧探索输出保留作对照。
