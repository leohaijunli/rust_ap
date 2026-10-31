# QGroundControl 安装与连接记录

日期：2026-10-31。官方QGC 5.1.5；本次独立USB入口刷回PX4、实际运行和最终普通Rust回装全部通过，P1已关闭，见[HIL](acceptance/p1/20261031-hil.md)。

## 本机安装

| 项 | 实际结果 |
| --- | --- |
| 主机 | Omarchy 4.0.6 / Arch Linux，x86_64，Hyprland Wayland |
| 发布 | [mavlink/qgroundcontrol v5.1.5](https://github.com/mavlink/qgroundcontrol/releases/tag/v5.1.5) |
| 安装文件 | `/home/dev/Applications/QGroundControl/5.1.5/QGroundControl-x86_64.AppImage`，195332600 B |
| SHA-256 | `9a47e4cf269d9e4f897f582f6b19aa7ebf1e9e67c9d68a532a45b541354bc344`，与发布digest一致 |
| 依赖 | fuse2 2.9.9-6；既有X11/xcb依赖；未安装ModemManager |
| 命令 / 菜单 | `~/.local/bin/qgroundcontrol`、`~/.local/share/applications/org.mavlink.qgroundcontrol.desktop` |
| GUI | 官方AppRun默认xcb，经XWayland运行；已映射且可连接飞控 |
| 权限 | `/etc/udev/rules.d/60-fmuv6x.rules`使用uaccess；采集时实际检查端口可读写 |

安装时原始记录在`~/.local/state/qgroundcontrol/`，发布元数据与SHA清单在安装目录。语音插件/主机定位提示不影响已确认的连接；语音未验收。

## 历史问题

- 工程师确认旧Rust可连接QGC、重插USB或复位能刷回PX4；早期消息清单/版本hash未完整绑定，见[工程师报告](acceptance/p1/20261031-user-recovery-report.md)。
- 早期新Rust上传/校验成功却无CDC，后经SCUEN和备份域修复恢复。该失败包身份未知，不沿用为当前包结论。
- QGC曾报`Permission error while locking the device`。tty权限正常，host测试确认`/tmp`创建报`Disk quota exceeded`；工程师清理后成功刷回，见[锁问题](acceptance/p1/20261031-qgc-lock.md)。
- PX4应用by-id不含BL，原上传入口漏匹配；已离线修复。现场使用明确`*PX4_BL*`端口回装成功，见[入口记录](acceptance/p1/20261031-upload-discovery.md)。
- 本轮QGC最初因X11 socket缺失中止；Wayland绕行后两次主机重启恢复X11。带NUL的临时文件、0 B输出和采集空档保留在`recovery/12-qgc-roundtrip/`，续接以`resume-after-restarts.md`为准。

## 本次结果

证据目录：`target/p1-hil-20261031-104403/recovery/12-qgc-roundtrip/after-host-restart/qgc-session/`，已纳入[封包](acceptance/p1/20261031-hil-evidence.md)。

- 普通source=`a946a529360eac83`；新Inspector截图确认HEARTBEAT（1 Hz、base_mode=0）、SYS_STATUS和SYSTEM_TIME。AUTOPILOT_VERSION重连后仍未出现，归P2跟进。
- 20:48:24 USB断开、20:48:25原BL枚举、20:49:06官方应用枚举；控制台有Erase/Program/Verify complete，截图显示PX4连接。
- 实际选择StableFirmware，包SHA-256=`3206a5f445d85be730a4f6158d8fdd414b9cfab27fa84bbc05c6a6752deac284`。不是预下载的GitHub包`8f2c4278…abba0d7f`。
- `03-px4-upgrade-complete.png`为Fly连接页。工程师确认“日志页已关闭，QGC已关”；完整升级UI文本没有保存。
- QGC退出码0。20:58 UTC独立串口采集确认PX4 v1.17.0、board_version=53和19个心跳。

21:28 UTC最终回装普通Rust `a946a529360eac83`，CRC、disarmed及无保留测试故障检查通过。P1恢复往返已完成；后续M1消息/参数按[当前计划](../plan.md#4-仅保留必要测试)验证。
