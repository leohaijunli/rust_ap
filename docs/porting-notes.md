# P0 基线与 A3 移植记录

日期：2026-10-31。**用户已选择 A3**。当前收口范围为 [M1](m1-verification.md)；设备使用 Embassy，算法保持 host 可测。完整证据索引见 [P0 状态](acceptance/p0/README.md)。

## 1. 已保存的入口

- 工程 HEAD：`5daeca30fcd116203164f44128239e7dd8a884ef`，入口已有用户工作树改动。
- [baseline-entry.json](acceptance/p0/baseline-entry.json)：入口状态、25 个文件 SHA-256、三个固件产物。
- 持久副本在 `.p0-baseline/20261031/`（gitignore）：源码归档、入口 diff、ELF、BIN、PX4 包；路径/大小/校验值见 [baseline-preservation.json](acceptance/p0/baseline-preservation.json)。解包恢复应使用独立目录，避免覆盖现有工作。
- 重新构建的 ELF/BIN/PX4 包 hash 与入口一致，见 [built-artifacts.json](acceptance/p0/built-artifacts.json)。`.px4` SHA-256：`d74b3b80e7c35bf4354ea568c09631f2fad362570caff414516ee0fe48a3c02d`。
- 该基线仍是 **64 MHz、USB CDC/Generic MAVLink/PHY LED demo**。已有 RTC 软件重启；IMU、AHRS、CAN、SD 和 TELEM1 尚未实现。历史硬件结果见 [README 验证记录](../README.md#verification-status)，不是本轮新 HIL。

## 2. 固定来源

| 名称 / URL | 完整 commit | 用途 |
| --- | --- | --- |
| [holsatus-flight](https://github.com/holsatus/holsatus-flight) | `45dd8c479e0c73b36046e33fb90bbf5f43e1cbdc` | A1 对照；依用户决定保留失败、跳过修复 |
| [cybflight](https://github.com/cybird-robotics/cybflight) | `806af5648c336e7af81abefea31052db783387f5` | A2 对照、A3 算法候选 |
| [PX4 v1.17](https://github.com/PX4/PX4-Autopilot) | `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` | 本地板级/bootloader 配置来源 |
| PX4 现用 uploader 工作树 | `a2be9197b57b2d065e4c47ca7fc5b7565ee07282` | `Tools/px4_uploader.py` |
| 现用PX4工作树内NuttX | `9304289d9f2b304c6835c9d39c131648c5f804c2` | P1启动断言的供电/寄存器定义核对 |
| [ST CMSIS H7 v1.10.4](https://github.com/STMicroelectronics/cmsis-device-h7) | `faccfec37f82f7a1319c21638111b0f7335de7fe` | H753 SCUEN定义交叉核对，无板上C依赖 |
| [Unicorn 2.1.4](https://github.com/unicorn-engine/unicorn) | `8028ec436f2d9376525352dd38ed9ed6b9f6be10` | 可选host Cortex-M7真实ELF谓词回放 |

Holsatus/Cybflight 入口无本地改动、无子模块；两个 PX4 工作树的初始未跟踪日志与子模块完整表在 [upstreams-entry.json](acceptance/p0/upstreams-entry.json)。查阅板级文件无本地修改。协议、DSDL、生成器和 AM32 的 pins 见 [来源清单](provenance.md)。

本机上游目录为 `/tmp/opencode/rust-ap-p0-20261031/<仓库名>`。复现时可换路径，但须 `git checkout --detach <完整 commit>` 并核对工作树；不要使用浮动分支代替这些版本。

后续主机恢复更新（2026-10-31）：用户清理`/tmp`排除QGC锁配额阻塞后，上述临时上游/构建路径已不存在；这里保留历史路径，不代表目前仍可使用。项目内P0恢复包与P1源码/产物归档仍在，包hash重核一致。需要上游时在主磁盘恢复固定commit，不重新将大体积构建放`/tmp`；本次未自动重新下载。

## 3. 工具与依赖

| 项 | 当前工程 / A3 起点 | Cybflight 上游基线 |
| --- | --- | --- |
| Rust | **1.99.1**，LLVM 23.1.1 | **1.94.0**，LLVM 21.1.8 |
| target | `thumbv7em-none-eabihf`；host 显式 `x86_64-unknown-linux-gnu` | 相同；上游 host 固定 x86-64、禁用 FMA |
| HAL / executor | `embassy-stm32 0.5.0` / `embassy-executor 0.9.1` | 相同解析版本 |
| time / USB | `embassy-time 0.5.1` / `embassy-usb 0.5.1` | 相同解析版本；**tick=32768 Hz** |
| time tick | **1 MHz** | A3 不继承应用的 32768 Hz feature |
| sync / IO | sync 0.7.2、0.8.0；embedded-io 0.6.1、0.7.1 | 同样存在重复版本 |
| MAVLink | `mavlink 0.17.1` common，保留当前实现 | 应用使用自身协议/ESP bridge，需适配 |
| lockfile | `Cargo.lock`，98 包 metadata 清单 | 上游 `Cargo.lock`，196 包 metadata 清单 |

包数量包含构建/host 依赖，不是板上实际链接数量。完整版本、许可和 hash 见 [dependency-inventory.json](acceptance/p0/dependency-inventory.json)，lock 副本在 [locks/](acceptance/p0/locks/)。不同 IO trait 版本须显式适配，不以同名 trait 推断兼容。

工具：cargo-binutils **0.4.0**；Python **3.14.8**；pyserial **3.5**、pymavlink **2.4.49**、fastcrc **0.5.0**、lxml **6.1.3**。组件/版本详情见 [environment.txt](acceptance/p0/environment.txt)、[environment-pinned.txt](acceptance/p0/environment-pinned.txt)。RP2040 target 仅为上游编译检查安装。

现用 `tools/firmware.sh` 指向 `/home/dev/PX4-Autopilot/Tools/px4_uploader.py`；v1.17 中是 `Tools/px_uploader.py`，接口不同。构建/打包不需要连接硬件；upload/monitor 属于工程师操作。

P1启动断言定位另使用 `unicorn==2.1.4`，按[修复记录](acceptance/p1/20261031-startup-assertion.md)在独立host目录安装；前后真实ELF各14项回放通过。寄存器模型与实际Dump的证据边界、版本/许可见该记录及[来源台账](provenance.md)。

## 4. 实际软件基线

| 检查 | 实际结果 | 原始证据 |
| --- | --- | --- |
| 当前工程 check / release build / section size / 重复依赖 | 通过；check 含 fmt、Clippy、4 项打包测试 | [current-results.json](acceptance/p0/current-results.json) |
| Holsatus common host / std-test-device / RP2040 / H743 | 四项 exit=101，同一依赖解析错误；**构建未验证** | [holsatus-results.json](acceptance/p0/holsatus-results.json) |
| Cybflight core | 446 passed、12 ignored；其中 Mahony 12 项通过 | [cybflight-core-test.txt](acceptance/p0/cybflight-core-test.txt) |
| Cybflight drivers / vehicle-YAML / sim snapshot | 26 / 51 / 1 passed | [test-counts.json](acceptance/p0/test-counts.json) |
| Cybflight M7 core check / H743 release build | 通过 | [cybflight-results.json](acceptance/p0/cybflight-results.json) |
| A3 独立 git 依赖 core 的 M7 ABI 探针 | Rust 1.99.1 编译通过 | [core-abi.json](acceptance/p0/core-abi.json) |

Holsatus 错误：`common` 要求浮动 `mav-param` 的 `serde` feature，但当前该依赖没有此 feature（`common/Cargo.toml:143`）。没有生成可用 lockfile。D-HOLSATUS 已授权跳过修复/重跑；后续实际取用该代码时再单独修复和回归。

当前镜像：`.text=109224`、`.rodata=5736`、`.data=260`、`.bss=13552` B，另有 664 B 向量表；BIN 115884 B。Cybflight H743：`.text=746800`、`.rodata=145712`、`.data=19352`、`.bss=255864`、`.uninit=4`、`.bkpsram=2056` B。详见两个 `*-size.txt`。

**上游 H743 镜像链接于 0x08000000，不能上传到本板。** 它是编译证据，不是 H753 的资源/时序验收。ELF 文件含调试信息，其文件大小不等于 flash 占用。

### 复现已有成功检查

当前工程根目录：

```sh
./tools/firmware.sh check
./tools/firmware.sh build
cargo +1.99.1 size --release --locked -- -A
cargo +1.99.1 tree --locked --duplicates
```

固定 Cybflight 仓库根目录（保留其 `.cargo/config.toml` 的 host flags）：

```sh
export RUSTUP_TOOLCHAIN=1.94.0
export VEHICLE=sakura_vicon
cargo test -p cybflight-core --target x86_64-unknown-linux-gnu --release --locked
cargo test -p cybflight-drivers --target x86_64-unknown-linux-gnu --locked
cargo test -p vehicle-yaml --target x86_64-unknown-linux-gnu --release --locked
cargo test -p cybflight-sim --target x86_64-unknown-linux-gnu --profile release-host --locked --test regression_snapshot -- --nocapture
cargo check -p cybflight-core --target thumbv7em-none-eabihf --release --locked
cargo build -p cybflight --target thumbv7em-none-eabihf --release --locked
```

上述测试不包含 ignored 诊断/基准。各次命令、cwd、exit 与耗时已有 JSON；耗时是 host 执行/构建时间，不是 H753 WCET。

## 5. 实际模块与迁移边界

### Holsatus

- `common` 同时包含算法、参数、MAVLink 与 Embassy 任务（`common/src/tasks/mod.rs`），并非可直接替换的纯算法 crate。
- `device/{rp2040-dev,micoair-h743-v2,std-test-device}` 是不同平台入口；README 自述重构中。
- 旧接口 `common/src/hw_abstraction/mod.rs:5-23`：`Imu6Dof`、`Imu9Dof`、`OutputGroup`；新接口 `common/src/drivers/imu/mod.rs:20-56`：`ImuInitialize`、`ImuSensor`、`trigger::Trigger`。适配须逐调用点核对。
- RP2040 PIO/DShot 在 `device/rp2040-dev/src/dshot_pio.rs`，不能迁移为 H7 驱动。host 仿真仍依赖 common，当前也被依赖解析错误阻塞。
- Embassy git 短 rev `5bc8260`、浮动 `mav-param`/ESKF/MAVLink 分支和 FAT IO 0.6/0.7 适配均增加复用成本。本轮不继续解析其失败依赖。

### Cybflight / 已选择 A3

- `crates/cybflight_core`：no_std 数学/算法/参数 wire 逻辑，无 Embassy 依赖；适合固定 git rev 独立依赖。
- `crates/drivers`：传感器/RC/DShot、`BlockStore` 抽象；`crates/bsp/*`：三种 H743 BSP；`crates/cybflight`：全局 channels、控制、估计、任务、参数服务、MCAP/FAT/SDMMC 应用。
- `crates/{cybflight_sim,vehicle_yaml,params_derive}`：host 仿真、配置校验与 derive；这些并不等于板上任务框架。
- M1 首取 `cybflight_core::mahony::Mahony<f32>`。`update(gyro, accel, mag, dt)` 接受 rad/s、m/s²、可选磁场和 `core::time::Duration`，返回 `UnitQuaternion`；调用方校验有限值、时间与坐标。
- `ParamGroup` 提供 get/set/meta/find/write_name，名称容量 48 B；MAVLink 层另做 ≤16 ASCII 别名、碰撞/精确 REAL32 转换与范围校验。
- `cybflight-drivers::blackbox_storage::BlockStore` 是 512 B 异步单块接口；应用的多块写合并、FAT patch、SDMMC 与调度仍需本板适配，详见 [日志](logging-budget.md)。

A3 先沿用当前 HAL/MAVLink/USB/打包边界，按需接 core；不搬入上游 main、BSP、DShot 全局任务或启动地址。host 测试、M7 编译和 HIL 各自验收。INDI/NMPC/任务仅保存已有调查结论，后续细节依据 M1 硬件结果再确定。
