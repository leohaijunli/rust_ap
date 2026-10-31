---
name: code-simplifier
description: Simplify recently changed code or plans for clarity and consistency while preserving behavior, project constraints, and the essential-test scope.
license: Apache-2.0
---

# Code Simplifier — OpenCode / Rust 适配版

改编自 Anthropic 的 code-simplifier，固定版本与改动见
[技能说明](../../../docs/ai-coding-skills.md)；上游许可保存在本目录 `LICENSE`。
本文件于 2026-10-31 修改：转换为 OpenCode 技能、采用 Rust/Python 规范、按项目限制验证范围。

## 精简原则

1. 保持功能、协议、错误处理和硬件行为；简洁优先于炫技，清晰优先于行数。
2. 只审视当前任务修改的部分；较大范围重构需要明确任务依据。
3. 消除无用分支、重复逻辑、多余包装和明显注释；保留有价值的抽象与不变量说明。
4. 优先直白的命名、提前返回及清晰的 `match` / 条件分支，避免密集的一行式表达。
5. 沿用现有 Rust/Embassy、Python 和模块结构；不为形式统一额外修改无关代码。
6. 精简计划时保留目标、依赖、必要验收和实际状态；合并重复要求，历史细节链接到证据。

## 执行

- 阅读改动及必要调用上下文，做一次针对性精简。
- 验证选择以 `plan.md` 第 4 节为准；代码变化只补相关检查，文档只查内容和差异。
- 检查通过后结束，简述重要改动和真实的未完成项。
