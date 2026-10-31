# P1：监看再次找不到端口——重枚举等待与诊断证据

日期：2026-10-31。用户报告：

```text
/bin/bash /home/dev/rust_ap/tools/firmware.sh monitor 10
Error: No matching Rust CDC device.
```

**当前只读发现能识别 Dump。已补有界等待及持久诊断日志；该短报错的确切发生时间、当时设备状态与完整输出仍未知。**

后续工程师已执行原命令成功采集6456 B，JSON为`outcome=ok`，有效PANIC定位到启动断言。SCUEN最小修复及复验步骤见[实际诊断](20261031-startup-assertion.md)。

## 现场核对

- 项目内 `tools/check_mavlink.py` 已包含首轮 Dump 分流修复；该错误完整格式还包含提示与已发现设备，不能仅凭用户提供的短行断言运行了旧版本。
- `.run/Monitor MAVLink.run.xml` 指向 `$PROJECT_DIR$/tools/firmware.sh monitor 10`，工作目录同为项目目录；shell 入口使用项目 `.venv/bin/python`。
- `serial.tools.list_ports` 与新 `select_port(allow_dump=True)` 均只读识别到 `/dev/ttyACM0`，VID:PID=`1209:0001`、serial=`V6X-RUST-DUMP`。另外32项为无USB身份的ttyS条目。
- 发现 `target/fault-dumps/20261031T142513.927420Z.txt`，大小 **0 B**，mtime=`2026-10-31 07:25:13.927045541 -0700`。离线解码返回 `No V6X_DUMP record found`。文件创建在串口打开之前，因此空文件本身不能证明串口曾成功打开或固件没有输出。

只读内核日志命令：

```sh
journalctl --dmesg --since '-30 min' --no-pager -o short-iso --grep='usb 8-1|ttyACM|V6X|cdc_acm'
```

相关节选（主机时区 `-07:00`）：

```text
2026-10-31T07:25:23-07:00 dev kernel: usb 8-1: USB disconnect, device number 31
2026-10-31T07:25:24-07:00 dev kernel: usb 8-1: New USB device found, idVendor=3185, idProduct=0035, bcdDevice= 1.01
2026-10-31T07:25:24-07:00 dev kernel: usb 8-1: Product: PX4 BL FMU v6X.x
2026-10-31T07:25:29-07:00 dev kernel: usb 8-1: USB disconnect, device number 32
2026-10-31T07:25:29-07:00 dev kernel: usb 8-1: New USB device found, idVendor=1209, idProduct=0001, bcdDevice= 0.10
2026-10-31T07:25:29-07:00 dev kernel: usb 8-1: Product: V6X Rust Dump
2026-10-31T07:25:29-07:00 dev kernel: usb 8-1: SerialNumber: V6X-RUST-DUMP
```

日志证明存在数秒无 Rust 应用端口的窗口，单次发现无法跨过它；触发这次USB断开/重启的原因未确认，不能认定为应用崩溃，也不能将用户错误时间直接绑定到此窗口。

## 修复

1. CLI 默认等待最多10秒，每250 ms重新只读枚举；无设备、bootloader或显式路径尚未出现时可等待。多候选立即报错。
2. 端口识别成功后才开始完整的 `--monitor` 时长；等待不占用10秒采集窗口。`--wait 0` 保留单次扫描方式，`p1_capture.py` 导入的 `find_port()` 仍只选普通固件、即时返回。
3. 增加 `firmware.sh ports` / `check_mavlink.py --list-ports`，只列身份并生成诊断记录。
4. 每次运行打印脚本绝对路径、SHA-256前12位；`target/monitor-logs/<UTC时间戳>.json` 保存完整脚本SHA-256、Python路径、参数、变化后的枚举快照、选中的模式/设备、错误与起止时间。默认目录遵循 `CARGO_TARGET_DIR`。
5. Dump 捕获结束显示精确字节数；只有串口正常打开并完成读取窗口但无数据时，才报告 `Dump CDC opened but received 0 bytes`。原始字节与断开时的部分文件继续保留。

## 验证

```sh
TMPDIR=/tmp/opencode .venv/bin/python -m unittest discover -s tools -p 'test_*.py' -v
bash -n tools/firmware.sh
/bin/bash /home/dev/rust_ap/tools/firmware.sh ports
git diff --check
```

结果：**59 tests / OK**，新增7项，覆盖空枚举→BL→Dump、等待截止、歧义不重试、等待后完整采集时长、失败持久证据、只读入口与非法等待值；Bash语法、diff检查通过。

真实 shell `ports` 入口只读结果：

```text
Tool: /home/dev/rust_ap/tools/check_mavlink.py sha256=ed1d16ce4c8c
Python: /home/dev/rust_ap/.venv/bin/python
Detected: /dev/ttyACM0 (V6X Rust Dump, 1209:0001, serial=V6X-RUST-DUMP); 32 other serial entries without USB identity
mode=dump /dev/ttyACM0
Diagnostics: /home/dev/rust_ap/target/monitor-logs/20261031T143524.679334Z.json
```

agent未打开实际串口、刷写、复位或注入故障。此次USB枚举和0 B文件不能替代有效故障报告或P1启动验收。

## 当前操作与 TODO

下述命令已由工程师执行成功；需要等待重枚举时，总耗时可达到约20秒：

```sh
/bin/bash /home/dev/rust_ap/tools/firmware.sh monitor 10
```

- [x] 现场核对、重枚举等待、运行证据与59项host回归。
- [x] 获得完整Dump文本与成功诊断JSON，见`20261031-startup-dump.txt`及`20261031-startup-monitor.json`。
- [x] P1-02启动断言已定位，SCUEN最小修复与离线回归通过。
- [ ] 工程师完成普通P1/QGC与恢复路径完整复验；基本连接已恢复，最新待验项见[备份域修复](20261031-backup-domain.md)。
