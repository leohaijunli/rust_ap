# 当前源码普通固件回装

日期：2026-11-07（UTC）。工程师授权检查状态并烧录当前固件；本轮没有修改固件源码。

## 构建与烧录

- 检查时工作区干净，HEAD 与实际远端 `main` 均为 `aaba83b7154a0ba1724930749b5f1601c1e19356`。
- 板上实际运行官方 PX4 v1.17.0，COM3，未解锁；不是历史记录中的 Rust 回装状态。
- 默认 release 目录仍是旧 source=`f5756b75d6ed78bf`，先归档，再用 `tools/build_firmware.ps1` 重建当前源码。
- 普通配置：版本 `0.2.0`，features=`[]`，source=`bf2f2a0bd91f5421`；raw/padded=`1069152/1069152 B`，static RAM=`254224 B`，最大入口栈帧 `11496 B`，0 超限。
- ELF/bin/包与当前 source 一致；board ID 53，向量表/应用入口 `0x08020000`，应用分区 `1966080 B`。
- 包：`target/thumbv7em-none-eabihf/release/v6x-rust-led.px4`。
- 包 SHA-256：`92b9d2bafdb4bcdbc4efcf5b5fcff7d21dca186ebd55d6e02702b8df1cdf19df`。
- 固定 uploader SHA-256=`f05929e1035e693005677e088e4e7085503b2e8ab3d7c8785f7bd84cd48880f8`；预检通过，经原 board 53 / protocol v5 bootloader 擦写应用并校验 CRC，25 s 完成，随后运行于 COM4。
- 没有使用 full-erase，没有更新 IO MCU，没有发送解锁、电机测试或非零输出命令。

首次构建被本机 PowerShell 执行策略阻止，改用仅当前进程的 `-ExecutionPolicy Bypass` 后通过，未修改全局策略。一次辅助只读探测误导入未安装的 `tomli`，改用现有 `tomllib` 后通过；两次失败均发生在烧录前。

## 烧录后只读结果

06:09:06–06:09:10 UTC，`post-flash.json` 的基础核对 `passed=true`：

| 项目 | 本次结果 |
| --- | --- |
| 启动/身份 | AUTOPILOT_VERSION 与 `!platform` 均为 `0.2.0` / `bf2f2a0bd91f5421`；3 个心跳均 disarmed，fault_injection=false |
| 平台 | VTOR=`08020000`，时钟 registers_ok=true，TIM8 `psc=239 arr=65535`；备份域配置与 BKP 均保持 |
| 主估计器 | `Filter/Running`，fault=None，config_errors=0，deadline_misses=0，max_update_us=559，导航未资格化 |
| 发布/积分 | 1 kHz 发布 missed=0；integration_error_ppm=0；传感器时间诊断残差 `-32394 ppm` 仍保留，不进入积分 |
| IMU/气压计 | ICM42688 WHO_AM_I=0x47，transfer/decode errors=0；BMP388 温压有效、errors=0 |
| 资源快照 | CPU 29.1%，共享栈 untouched=36688 B，guard_ok=true；这是一次快照，不是新做的完整时序验收 |
| 参数 | Stored/valid=true，slot=1/gen=4，dirty=true/pending_save=false；未设参、未保存。继承 `EST_EN=1/EST_ALGO=1` 自动运行，未发送 start/stop |
| 历史故障 | `f509ec6b85b6a4fa` 的旧 panic 记录 valid=true，未清除；不是本次固件新故障 |
| 日志 | Disabled，未启动新 SD 日志 |
| CAN/GNSS | CAN1 rx_frames=0，ESC trusted=0，GNSS none；总线错误/发送超时存在，不据此推断实体接线原因 |
| INA219/PX4IO | INA219 探测地址均未应答；PX4IO ready=false、Timeout，仍待工程师核对实物 |

## 证据与边界

本地证据：`target/flash-verify/20261107-current-firmware/`，包含构建/预检/上传/回读记录、只读核对脚本、`post-flash.json` 及前后产物。`target/` 被 Git 忽略，提交的是本记录，不是原始产物。

本次证明普通固件上传、CRC 与启动身份/平台保持正确。没有重做 QGC 独立恢复、SD 日志、双链路或完整 65 s 采集；复用未受源码改动影响的历史结果。M1、完整 E7 与各剩余实物门不放行。
