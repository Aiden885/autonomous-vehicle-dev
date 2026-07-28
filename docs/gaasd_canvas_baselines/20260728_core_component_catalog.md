# GAASD 基础组件当前版本快照

> 生成时间：`2026-07-28T11:14:11+08:00`  
> 组件库：`/home/aiden/gaasd_versions/gaasd-2.7.0.5/home/.gaasd/gaasd.db`  
> SHA256：`2a36f48d81ae0a03703b9d1fdd21bcc428b3618f7c0541fed03a5ef648d7e7fa`

该文件是版本快照，不是永久规范。GAASD 更新后应重新生成并比较。

| 组件 | 版本 | vendor | originId | 配置字段 | 输入端口 | 输出端口 |
| --- | --- | --- | --- | --- | --- | --- |
| `input` | `1.2.0` | `default` | `input_init_0.0.1` | `name`, `dataType`, `category`, `shape` | - | `out` |
| `output` | `1.2.0` | `default` | `output_init_0.0.1` | `name`, `dataType`, `category`, `shape`, `isReturnFlag` | `in` | - |
| `constant` | `1.2.0` | `default` | `constant_init_0.0.1` | `name`, `dataType`, `dataValue` | - | `out` |
| `add` | `1.2.0` | `default` | `f4900c21-96c9-50ab-8dd3-5741c3b496f7` | `inputNumber`, `isReal` | `a`, `b` | `result` |
| `subtract` | `1.2.0` | `default` | `1dc8c9e5-dfd3-5ddb-8d9d-4df76e3f99c3` | `inputNumber`, `isReal` | `a`, `b` | `result` |
| `multiply` | `1.2.0` | `default` | `6553dcc3-daeb-5325-9a37-1a1b9f293090` | `inputNumber`, `isReal` | `a`, `b` | `result` |
| `divide` | `1.2.0` | `default` | `1e16b7d4-dd62-5f11-ab09-496c3265d9c7` | `inputNumber`, `isReal` | `a`, `b` | `result` |
| `fmax` | `1.2.0` | `default` | `cc3347b5-9767-5ffc-b249-5efaf99633ff` | - | `x`, `y` | `return` |
| `fmin` | `1.2.0` | `default` | `862f40bd-f57b-5302-9ee5-b987e994a524` | - | `x`, `y` | `return` |
| `less-than` | `1.2.0` | `default` | `71d8dc95-5c68-54c4-87dd-c5f915021da7` | `inputNumber`, `isReal` | `a`, `b` | `result` |
| `greater-than` | `1.2.0` | `default` | `ae6ba502-8149-5b8a-a683-e0a0553b668e` | `inputNumber`, `isReal` | `a`, `b` | `result` |
| `equal` | `1.2.0` | `default` | `e9ae36ef-0308-57a9-a2d7-1557682b40bf` | `inputNumber`, `isReal` | `a`, `b` | `result` |
| `logic-and` | `1.2.0` | `default` | `98ec37fa-7a10-536d-8923-5a3a4ee679cd` | `inputNumber`, `isReal` | `a`, `b` | `result` |
| `logic-or` | `1.2.0` | `default` | `7d619955-769c-5a87-9b63-c65a5c80ced7` | `inputNumber`, `isReal` | `a`, `b` | `result` |
| `logic-not` | `1.2.0` | `default` | `83bfe4bc-d7a4-5e39-aa04-a221346c84cb` | `inputNumber`, `isReal` | `operand` | `result` |
| `read-local-param` | `1.2.0` | `default` | `read-local-param_init_0.0.1` | `operateKeys` | - | `out` |
| `read-local-state` | `1.2.0` | `default` | `read-local-state_init_0.0.1` | `operateKeys` | - | `out` |
| `write-local-state` | `1.2.0` | `default` | `write-local-state_init_0.0.1` | `operateKeys` | `in` | - |
| `truth-table` | `1.2.0` | `default` | `truth_table_init_0.0.1` | `conditions`, `actions`, `scenarios` | `u`, `v` | `y` |
| `oscilloscope` | `1.2.0` | `default` | `oscilloscope_init_0.0.1` | `inputNumber`, `simulateStep`, `simulateTotalTime` | `input_1`, `input_2` | - |
