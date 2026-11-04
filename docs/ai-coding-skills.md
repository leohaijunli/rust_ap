# 项目 AI 编码技能

已安装到 `.opencode/skills/`，由 OpenCode 自动发现；项目规则见 [AGENTS.md](../AGENTS.md)。

| 技能 | 固定来源 | 本地用途 |
| --- | --- | --- |
| [clean-code](../.opencode/skills/clean-code/SKILL.md) | [liuchiawei/agent-skills](https://github.com/liuchiawei/agent-skills/blob/f5a7853d0d7db66ff7956fbaf504365c7971007d/skills/refactor/clean-code/SKILL.md)，`f5a7853d0d7db66ff7956fbaf504365c7971007d` | 按 KISS/YAGNI、清晰命名和实际需求编写代码 |
| [code-simplifier](../.opencode/skills/code-simplifier/SKILL.md) | [Anthropic code-simplifier](https://github.com/anthropics/claude-plugins-official/blob/ea0a38e1d671aa18a30431c9160e31193dc9860b/plugins/code-simplifier/agents/code-simplifier.md)，`ea0a38e1d671aa18a30431c9160e31193dc9860b` | 对本次改动做一次保持行为的精简 |
| [rust-embedded](../.opencode/skills/rust-embedded/SKILL.md) | [full-stack-skills/rust-skills](https://github.com/full-stack-skills/rust-skills/blob/b8a6af74af28ee86b8cbbbc3075cde0b31a5155e/skills/rust-embedded/SKILL.md)，`b8a6af74af28ee86b8cbbbc3075cde0b31a5155e` | `no_std` / Embassy、HAL/PAC、IRQ/DMA、超时/取消、构建与硬件证据；Apache-2.0 |

## 适配说明

- `clean-code` 的通用设计原则以项目措辞重述；上游该版本未附可识别的许可文件，不声明继承其许可。未复制上游完整文档和验证脚本表。
- `code-simplifier` 从 Claude Code agent 转成 OpenCode `SKILL.md`，保留 Apache-2.0 许可；移除固定模型选择，语言约定改为本项目 Rust/Python。
- 两者采用[计划中的必要测试](../plan.md#4-仅保留必要测试)，代替上游的 Web/TypeScript 审计脚本、固定函数行数和逐项确认流程。
- `rust-embedded` 于 2026-11-04 精简为中文项目适配版，保留三份上游参考及许可；固定来源后存入项目，不安装其他框架、技能包或刷写脚本。通用资源的硬件矩阵不增加阶段门，现有 Embassy、bootloader、panic 与必要测试规则保持。
- 本轮用于精简计划、TODO 和提交说明；后续开发按当前阶段实施，保持固件/协议行为与已有验收证据。

可显式调用：`@clean-code`、`@code-simplifier`、`@rust-embedded`。
