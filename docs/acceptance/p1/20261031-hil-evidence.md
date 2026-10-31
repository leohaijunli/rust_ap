# P1 本轮原始证据

原始目录：`target/p1-hil-20261031-104403/`；结果见[HIL记录](20261031-hil.md)。

| 封包 / SHA-256清单 | 范围 |
| --- | --- |
| [原始封包](20261031-hil-evidence.tar.gz) / [清单](20261031-hil-evidence.sha256) | 154文件，9074920 B；截止`13-px4-running`及提交前文档恢复记录 |
| [最终回装封包](20261031-final-return-evidence.tar.gz) / [清单](20261031-final-return-evidence.sha256) | 5文件，115114 B；`recovery/final_return.py`及`recovery/14-px4-to-normal-final/`全部输出 |

- 两包均经逐成员校验；各自`SHA256SUMS`保存文件hash，原始封包保持不变。
- 最终脚本复用的`qgc_return.py`、`run_step.py`、固定uploader和普通镜像在原始封包中，与本次使用文件逐字节一致；host工具为仓库`c962e87`基线。
- 原始封包包含TX/RX、上传/协议/测量、镜像、现场脚本及QGC日志/截图；34.55 MB失败上传日志、损坏临时文件和文档副本原样保留，不含Python字节码缓存。

解包：`tar -xzf <封包路径> -C <空目录>`。
