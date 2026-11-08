# 项目 AI 编码技能

已安装到 `.opencode/skills/`，由 OpenCode 自动发现；项目规则见 [AGENTS.md](../AGENTS.md)。

| 技能 | 固定来源 | 本地用途 |
| --- | --- | --- |
| [clean-code](../.opencode/skills/clean-code/SKILL.md) | [liuchiawei/agent-skills](https://github.com/liuchiawei/agent-skills/blob/f5a7853d0d7db66ff7956fbaf504365c7971007d/skills/refactor/clean-code/SKILL.md)，`f5a7853d0d7db66ff7956fbaf504365c7971007d` | 按 KISS/YAGNI、清晰命名和实际需求编写代码 |
| [code-simplifier](../.opencode/skills/code-simplifier/SKILL.md) | [Anthropic code-simplifier](https://github.com/anthropics/claude-plugins-official/blob/ea0a38e1d671aa18a30431c9160e31193dc9860b/plugins/code-simplifier/agents/code-simplifier.md)，`ea0a38e1d671aa18a30431c9160e31193dc9860b` | 对本次改动做一次保持行为的精简 |
| [rust-embedded](../.opencode/skills/rust-embedded/SKILL.md) | [full-stack-skills/rust-skills](https://github.com/full-stack-skills/rust-skills/blob/b8a6af74af28ee86b8cbbbc3075cde0b31a5155e/skills/rust-embedded/SKILL.md)，`b8a6af74af28ee86b8cbbbc3075cde0b31a5155e` | `no_std` / Embassy、FPU 上下文/抢占、HAL/PAC、IRQ/DMA、超时/取消、构建与硬件证据；Apache-2.0 |
| [migrate-to-rust](../.opencode/skills/migrate-to-rust/SKILL.md) | 工程师提供的 v1.0.0 完整包，保留于 `docs/migrate-to-rust/` | 项目适配入口、确定性迁移/真实遥测；不改既定软件/硬件门，不声明额外许可 |

## 适配说明

- `clean-code` 的通用设计原则以项目措辞重述；上游该版本未附可识别的许可文件，不声明继承其许可。未复制上游完整文档和验证脚本表。
- `code-simplifier` 从 Claude Code agent 转成 OpenCode `SKILL.md`，保留 Apache-2.0 许可；移除固定模型选择，语言约定改为本项目 Rust/Python。
- 两者采用[计划中的必要测试](../plan.md#4-仅保留必要测试)，代替上游的 Web/TypeScript 审计脚本、固定函数行数和逐项确认流程。
- `rust-embedded` 于 2026-11-04 精简为中文项目适配版，保留三份上游参考及许可；固定来源后存入项目，不安装其他框架、技能包或刷写脚本。通用资源的硬件矩阵不增加阶段门，现有 Embassy、bootloader、panic 与必要测试规则保持。
- 2026-11-08 将实测 FPU 缺失根因固化为该技能的必查原则：统一入口初始化、ABI/抢占保护、镜像单次计算、坏方差保护与有界硬件回归；host 离线不替代异常上下文验证，无相关变更则复用已验结果。
- 本轮用于精简计划、TODO 和提交说明；后续开发按当前阶段实施，保持固件/协议行为与已有验收证据。

可显式调用：`@clean-code`、`@code-simplifier`、`@rust-embedded`、`@migrate-to-rust`。

## 工程师新增的移植参考

[migrate-to-rust v1.0.0](migrate-to-rust/migrate_to_rustL.md) 的正文、telemetry 脚本、PGO 指南和 HTML 模板已在新目录提供；通过 `.opencode/skills/migrate-to-rust/SKILL.md` 注册项目适配入口。此前只有单文件正文且资源缺失的事实保留，原文及资源不改写。

后续采用其连续推进、限定失败修复次数、确定性 A/B 对拍及真实遥测。`forbid(unsafe_code)` 只用于 estimator 数学核心，不覆盖必要的板级启动/MMIO；全量 95% 覆盖率、SoA/SIMD、PGO、叙事报告与 infographic 不自动加入 E 系列资格门。当前范围和必要测试仍以 [plan.md](../plan.md) 为准；性能/报告阶段需明确范围后启用，不伪造未测指标。
