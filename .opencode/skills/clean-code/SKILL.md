---
name: clean-code
description: Write or maintain clear, concise Rust/Embassy and Python code using simple design, meaningful names, and the project's essential-test policy.
---

# Clean Code — 项目适配版

参考 liuchiawei/agent-skills 的 clean-code，按本项目重述并适配。
来源、固定版本和改动见 [技能说明](../../../docs/ai-coding-skills.md)。

## 编写规则

- 先读当前任务及 `plan.md` / `TODO.md`，实现满足需求的最简单方案。
- 名称表达含义，沿用 Rust/Python 和周围代码的命名习惯。
- 函数职责单一；用提前返回降低嵌套，不机械限制行数或参数个数。
- 复用已经存在的实现；只有实际重复或职责边界需要时才提取抽象。
- 不为假设的未来需求增加层、工厂、配置或兼容分支。
- 注释解释原因、硬件约束或不变量，不逐行复述代码。
- 修改接口时同步实际调用者；保留必要的错误处理和有界行为。
- 文档只保留决定、当前状态、下一步和证据链接。

## 验证与交付

- 只运行 `plan.md` 第 4 节中与改动相关的必要检查。
- 复用未受改动影响的通过结果；文档调整不触发固件构建或上板测试。
- 出现新失败时定位并修复，再复验受影响项。
- 完成后用 `code-simplifier` 审视本次改动；一次审视足够，保留行为并简短报告结果。
