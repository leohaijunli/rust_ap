# P1 本轮原始证据

- [封包](20261031-hil-evidence.tar.gz)：`target/p1-hil-20261031-104403/`快照，截止本次`13-px4-running`及提交前文档恢复记录。
- [清单](20261031-hil-evidence.sha256)：封包SHA-256；包内`SHA256SUMS`列出各成员hash。
- 包含TX/RX、上传/协议/测量结果、镜像、现场脚本、QGC日志/截图、失败与中断；不含Python字节码缓存。
- 34.55 MB失败上传日志、发现时损坏的临时文件和文档副本均原样保留；详见[HIL记录](20261031-hil.md)。

解包：`tar -xzf docs/acceptance/p1/20261031-hil-evidence.tar.gz -C <空目录>`。
