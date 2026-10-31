# PX4回装后上传入口：默认端口匹配遗漏（已离线修复）

日期：2026-10-31。**端口发现遗漏已修复；当轮22项Python测试与Bash语法检查通过，只读发现官方PX4串口。agent未打开串口、发送重启命令或刷写，尚未验证实际PX4→bootloader→Rust上传。** 后续实际Dump已定位启动断言，SCUEN最小修复及普通P1/QGC基本连接通过；最新软件修复与工程师待验项见[备份域修复](20261031-backup-domain.md)，历史过程见[诊断与交接](20261031-startup-assertion.md)。

## 用户错误与实际身份

用户清理`/tmp`后报告成功刷回PX4，随后执行：

```text
/bin/bash /home/dev/rust_ap/tools/firmware.sh upload
Expected one FMUv6X serial port, found []. Connect one board or set PX4_PORTS.
```

04:02:50（主机时区`-07:00`），agent只读sysfs/by-id枚举：

```text
USB 8-1: idVendor=3185 idProduct=0035
manufacturer=Auterion product=PX4 FMU v6X.x serial=0
/dev/serial/by-id/usb-Auterion_PX4_FMU_v6X.x_0-if00 -> /dev/ttyACM0
当前用户read/write access=True
```

这是官方PX4应用的产品/by-id身份，不含`BL`；本次应用和先前bootloader的VID/PID相同，不能只靠`3185:0035`区分两者。设备存在不等于已验证PX4心跳或完整运行状态。

## 确认的软件原因与最小修复

`tools/firmware.sh`原默认模式只有Rust CDC与`/dev/serial/by-id/*PX4_BL*`，两者都不匹配当前`PX4_FMU_v6X.x`应用串口。内联preflight因此在构建/调用uploader之前退出；本次错误不是串口锁、校验或新应用启动失败。

默认模式新增：

```text
/dev/serial/by-id/usb-Auterion_PX4_FMU_v6X.x_*-if00
```

保留原Rust/bootloader模式、`PX4_PORTS`覆盖、恰好一个端口与读写权限检查。传给固定uploader的模式同时包含应用与bootloader，供其重启后重新枚举使用。不扩大到任意tty、不改udev/权限，不修改应用、bootloader或option bytes。

## 本轮离线检查

新增`tools/test_firmware_ports.py`直接读取脚本的默认赋值/preflight，使用合成设备列表与权限mock，不执行上传脚本、不构建或打开tty：

- Rust CDC、官方PX4应用、bootloader均匹配。
- 其它PX4应用不被新增模式匹配。
- 无设备、多设备、无权限仍拒绝。
- 显式`PX4_PORTS`覆盖与重复模式去重保留。

```text
$ .venv/bin/python -m unittest discover -s tools -p 'test_*.py' -v
Ran 22 tests in 0.036s
OK

$ bash -n tools/firmware.sh
exit 0
```

仅执行默认赋值/`glob`/`os.access`的只读现场复核（没有调用uploader/build）：

```text
matches ['/dev/serial/by-id/usb-Auterion_PX4_FMU_v6X.x_0-if00']
read/write access True
```

历史P1归档的16项Python/11项Rust与target构建结果不改写；本轮只新增6项上传入口host测试，没有重新构建固件，也没有证明实际协议/刷写成功。

## 工程师下一步与安全边界

- 先关闭QGC/uploader/监看器。若只验证前置，可由用户执行`bash tools/firmware.sh upload --dry-run`：检查端口与重建普通包，不发送重启/刷写命令。
- 当时默认`upload`重建SCUEN修复后的普通P1，source=`4cba95aa54cd899a`，板上运行Dump；先发送`!bootloader`并确认原BL枚举，再按[工程师交接步骤](20261031-startup-assertion.md#5-工程师复验从当前-dump-返回普通-p1)上传及复验MAVLink/QGC。最新包身份见[备份域修复](20261031-backup-domain.md)。
- 回装已知良好旧Rust须显式使用保存的P0包和记录，不用默认`upload`替代。原P0/P1归档hash已重核保留；不自动故障注入、不访问FRAM、不推进依赖P1的硬件阶段。

关联：[主机锁排除与PX4刷回报告](20261031-qgc-lock.md)、[新Rust启动回归](20261031-startup-regression.md)。
