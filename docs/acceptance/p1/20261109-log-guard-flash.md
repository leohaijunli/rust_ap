# 日志门控／测试告警修复包回装

日期：2026-11-09。工程师单独授权“烧录”；仅回装既有合格普通包 `49b3552f38510053` 并只读核对启动，未追加外设测试、设参/保存、清故障、日志启停/删除或输出动作。

## 回装与保持：通过

- 包沿用[软件检查结果](../pl-a/20261108-baro-log-freshness.md)，本次不重新构建或重复 host/PX4 对拍。SHA-256=`b637ce78cc54be7d503b9071eaec9bf7e9a346af900acd944f1f3de02560c42d`；raw/padded=1,217,456/1,217,472 B。
- 烧录前 COM4 source=`81fd0d06601f9e2f`，3 个 Quadrotor/PX4/disarmed 心跳；227 项参数值/类型/顺序、历史故障与平台快照已保存。logger Disabled、pending_save=false。
- 原 PX4 bootloader board 53/protocol v5（COM3）上传与 CRC 校验完成，25 s；仅写应用区，原 bootloader/`0x08020000` 入口保持。恢复 COM4，source=`49b3552f38510053`，3 个 disarmed 心跳。
- 227 项参数值/类型/顺序、PARAMS 存储状态与历史故障全文均与烧录前相同：Stored、slot=1/gen=4/format=2、dirty=true、pending_save=false、last_error=None。没有写参数迁移或清除故障。
- VTOR=`08020000`、CLOCK 与前快照相同、registers_ok=true；BACKUP config_preserved/bkp_preserved=true。FPU ASPEN=true/LSPEN=false，Nearest、FZ/DN 关闭。

## 启动只读快照

回装后 UTC 00:42:21–00:42:31，继承既有估计器启用值，未执行 `!ekf2 start`：

- 主 EKF Running/Filter，fault/last_fault/numerics=None；config_errors/recoveries/deadline_misses=0，max_update=1223 µs（预算 2000 µs）。
- 1 kHz 发布 missed=0；磁融合最后快照 fused=true/rejected=false，不当作融合成功率，也不重启 yaw 根因调查。
- CPU 快照 33.1%，栈 untouched=31536 B/guard=true；PERIOD 漏期/超限/跳过均 0。USB tx_errors=62 是本次启动后的累计计数，不由此宣称完整链路验收或下载速率。
- logger Disabled，bytes_written/flushes/errors=0；未发送 LOG_ERASE，不重复工程师此前的日志删除验证。

该记录仅覆盖合格包回装、启动和保持核对；没有开启 SD 日志验证气压异常输入，没有 65 s/5 min 联合采集，不放行 M1、完整 E7、外设或飞行资格。后续仍以软件为主，另行刷写/实体动作须明确授权。

证据：`target/flash-49b355-20261109/` 的 `before-20261109T004114213318Z.json`、`after-20261109T004221811175Z.json`、`upload.log`、`verify.py`；两快照 passed=true。`verify.py` 复用既有参数/控制台读取，只有查询，不含设参、日志启停/删除或输出命令。
