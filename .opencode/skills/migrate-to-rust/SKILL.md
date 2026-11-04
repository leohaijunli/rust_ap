---
name: migrate-to-rust
metadata:
  version: "1.0.0-project"
  category: ApplicationDevelopment
description: >-
  按本项目约束移植固定 C/C++ 等遗留数学核心到安全 Rust，记录真实测试、
  失败修复与确定性 A/B 对拍。适用于 EKF 等纯核心移植，不替换 Embassy、
  bootloader 或必要的板级 unsafe，不自动启动全量覆盖率、SIMD 或 PGO 工作。
---

# Migrate to Rust — 项目适配入口

工程师提供的完整技能及资源保留在 `docs/migrate-to-rust/`，不重复复制或重写。
开始使用前读取[原文](../../../docs/migrate-to-rust/migrate_to_rustL.md)与
[工程/遥测指南](../../../docs/migrate-to-rust/references/pgo_and_telemetry_guide.md)。

## 项目优先约束

- 当前范围、必要测试和软件/实物阶段门以 [plan.md](../../../plan.md)、
  [TODO.md](../../../TODO.md) 及 `AGENTS.md` 为准；E1-H～E6-H 连续推进。
- 只移植固定来源的数学核心，不克隆或执行未经确认的新外部仓库。
  已授权的固定 PX4 host 参考构建/运行不重复询问。
- `forbid(unsafe_code)` 限定 estimator 数学子树；保留板级必要的启动、
  MMIO、IRQ/DMA 代码。保持 no_std、无堆、原 PX4 bootloader 和应用入口。
- 使用既定容差和不可覆盖黄金数据。每个新失败根因最多自动修复 5 次；
  超预算时记录差异，不放宽容差或伪造通过。
- 复用已有未受影响结果；不默认运行 all-targets/all-features、不强制改
  PRNG、f32 运算次序或按行数拆分生成表达式。文件长度是审视信号，非阶段门。
- 95% 分支覆盖率、SoA/SIMD、四段性能测量、PGO 和叙事报告不是当前 E 系列
  软件资格门。需要时先明确范围，host 结果不替代 MCU 时序/内存资格。
  不修改目标 release 配置或使用 target-cpu=native 影响 H753 构建。

## 遥测工具

脚本：[telemetry.py](../../../docs/migrate-to-rust/scripts/telemetry.py)
模板：[infographic_template.html](../../../docs/migrate-to-rust/references/infographic_template.html)

从项目目录运行：

```sh
python3 docs/migrate-to-rust/scripts/telemetry.py --help
```

- 先核对脚本 CLI；使用独立账本，已有账本不得再次 init 覆盖。
- 从启用时记录实测耗时；历史结果可标注复用，但不补造历史时间、覆盖率、
  测试次数或速度收益。场景数量不能冒充不同 PRNG seed 数量。
- 未测覆盖率/RSS/PGO 保留 unknown 或 omitted；代码扫描器为正则估计，
  不是编译器安全证明、分支覆盖率或可靠的缺陷计数。
- 渲染会结束活动计时桶并可能设置 completed_at；只在本次记录范围完成后
  执行，范围说明必须明确 E 系列/实物待验状态，不能提前宣告 E6-H 完成。

编写时同时使用 `clean-code` / `rust-embedded`；收尾按 `code-simplifier`
做一次局部审视。此入口仅适配项目规则，不重新许可工程师提供的资源。
