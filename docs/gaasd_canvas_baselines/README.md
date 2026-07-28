# GAASD 画布基线说明

该目录保存 GAASD 环境和手动画布的**版本化事实快照**，用于软件升级后的差异比较，不作为永久格式规范。

当前文件：

| 文件 | 用途 |
| --- | --- |
| `20260728_newaccpro3_environment.json` | GAASD 前端、组件库和 `newaccpro3` 数据库指纹。 |
| `20260728_newaccpro3_audit.json` | 本次 Decision 分层后的通用结构审计结果。 |
| `20260728_core_component_catalog.md` | 当前组件库核心基础组件的版本、配置字段和端口快照。 |

当前基线结论：

```text
GAASD build version: 27.1.3
GAASD package version: 2.7.0
核心基础组件模板: 1.2.0
newaccpro3 cbdes.db SHA256:
ba85577b1a446cf0e0faa3cfdbcc5806656a2400e01322a97dd6a19b991ba96a
结构审计: PASS，0 errors，0 warnings
```

生成和比较命令见：

```text
docs/GAASD_手动画布构建知识库与复用流程.md
tools/gaasd_canvas_tool.py
```

本目录没有代替完整工程快照。完整工程只能在 GAASD 完全退出后使用 `snapshot` 命令生成。

