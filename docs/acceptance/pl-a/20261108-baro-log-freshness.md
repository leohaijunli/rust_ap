# 气压日志新鲜度门控（仅软件）

日期：2026-11-08。工程师当前只有飞控本体、无外接设备，要求只做软件。板载仍为 `81fd0d06601f9e2f`；本轮没有串口操作、采集、刷写、设参/保存、日志启停/删除或输出动作。

## 问题与修改

- 已有待办：`sd_logger::record_baro` 先更新限流时间，再拒绝陈旧气压样本，可能让下一正常样本丢失日志名额。默认 10 Hz 下未取得实物丢失证据；host 用例用合法的 1 Hz 配置覆盖该边界。
- 共用 host 可测的 `log_config::baro_rate`，先执行原 `Sample::fresh`，通过后才使用原组限流器；日志未 Ready 时仍提前退出。
- 200 ms 新鲜度及包含边界不变，未来时间仍拒绝；补偿、采样驱动、参数/默认值、ULog 字段与 EKF 输入不变。未增加堆、队列、外设驱动或融合计数器。

## 必要检查

- 旧 `rate → fresh` 顺序下，新增回归检查在正常样本应允许的断言处失败；修后通过。最终用例覆盖顺序到来的陈旧/正常样本、1 Hz 限流边界、未来时间、重复样本及新鲜度边界，加入原共享状态用例，避免新增并发测试争用全局限流器。
- `cargo test --lib --target x86_64-pc-windows-msvc --locked log_config::`：2 项通过。
- 同命令筛选 `bmp388::`：2 项通过；气压小修阶段先跑这 4 项相关用例，随后按工程师追加告警修复范围执行完整 host 测试，见下节。
- `cargo fmt --check`、普通 `cargo clippy --release --locked -- -D warnings`、host `cargo clippy --lib --target x86_64-pc-windows-msvc --locked -- -D warnings` 通过。
- 气压修复初始候选 source=`2a2b272fcd059412` 构建/打包及入口帧检查通过：raw/padded=1,217,472 B，静态 RAM=374,400 B/余 83,328 B，最大入口帧=11,496 B、超限 0。RAM/入口帧不变，board ID 53/入口 `0x08020000` 保持；此包不含随后追加的告警清理，未回装。
- 初始候选归档在 `target/software-log-guard-20261108/baro-only-2a2b272fcd059412/`，原板载对应包在 `before-81fd0d06601f9e2f/`；本轮未重复构建不受影响的备用配置或运行 PX4 数学对拍。
- 已对本次修改做局部 `code-simplifier` 审视，保留直接门控和原状态/速率语义。

## 扩展检查告警与追加修复

首次额外执行 `cargo clippy --lib --tests --target x86_64-pc-windows-msvc --locked -- -D warnings`，报告 6 项当时未修改文件中的既有告警。此前常规 host 库检查不含测试代码，这次扩大到 `--tests` 才暴露这些写法：

- `core/dronecan/dsdl.rs:649`、`core/dronecan/gnss.rs:186`、`core/gps.rs:214`：`chunks_exact_to_as_chunks`。
- `core/estimator/range_sensor.rs:279`：`approx_constant`。
- `core/estimator/runtime.rs:549`：`items_after_test_module`。
- `core/estimator/standard_ulog.rs:461`：`assertions_on_constants`。

上述路径均在 `src/` 下；首次告警时已核对与 HEAD 无差异。工程师随后明确要求也修复这 6 项，追加修改如下，没有全局 allow：

- 三处测试十六进制拆分改 `as_chunks::<2>().0.iter()`，仍忽略末尾不足两字节的部分，已有参考字符串/解码向量不变。
- 测试常量改为原项目已用的 `f32::from_bits(0x3f350481)`；它与固定 `0.7071f` 逐位相同，不能换成值略不同的 `FRAC_1_SQRT_2`。
- `AidId` / `Measurement` 实现块移到测试模块前，签名/内容逐字相同。
- 日志尺寸不变量改成测试内的 const 断言，判据仍为 `RUNTIME_RECORD_LEN >= 218`。

追加验证：fmt、host 库及测试 Clippy `-D warnings` 通过；完整 `cargo test --lib --target x86_64-pc-windows-msvc --locked` **218 项通过**。离线核对确认 Runtime 实现内容、常量位值与十六进制参考向量不变；已做局部 `code-simplifier` 审视。

最终合并普通候选 source=`49b3552f38510053`：H753 Clippy `-D warnings`、实际构建/打包和入口帧检查通过。raw=1,217,456 B、padded=1,217,472 B、RAM=374,400 B/余 83,328 B、最大入口帧=11,496 B/超限 0；board 53/入口 `0x08020000` 保持，RAM/入口帧不变。产物及报告归档在 `target/software-log-guard-20261108/ordinary/`，未刷写。

包 SHA-256：`b637ce78cc54be7d503b9071eaec9bf7e9a346af900acd944f1f3de02560c42d`；归档逐文件校验与构建产物一致。

最初 Clippy 失败作为历史检查结果保留；硬件门、M1/E7 与飞行资格保持未放行。
