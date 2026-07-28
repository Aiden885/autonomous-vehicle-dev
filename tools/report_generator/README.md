# 周报生成工具

这个工具用于把每周整理好的 Markdown 内容生成 `.docx` 周报，版式参考 `docs/templates/weekly_report_template.docx` 和历史周报。

## 1. 安装依赖

```bash
python3 -m pip install -r tools/report_generator/requirements.txt
```

## 2. 编写输入文件

复制模板：

```bash
cp docs/weekly_inputs/template.md docs/weekly_inputs/2026-07-week3.md
```

填写以下几类内容：

- `## 进展：xxx`：本周已完成工作，每个进展模块会生成一张三列表格。
- `## 风险与问题`：当前风险、影响和应对措施。
- `## 下周工作计划`：下周计划项。
- `## 需要协调与帮助`：可选，没有可删除。

## 3. 生成周报

```bash
python3 tools/report_generator/generate_weekly_report.py \
  docs/weekly_inputs/2026-07-week3.md \
  -o docs/赵煜坤-7月第3周项目进展周报.docx
```

如果不传 `-o`，会默认输出到 `docs/{title}.docx`。

## 4. 输入格式

进展项支持两种写法：

```markdown
- 任务项 | 完成情况
- 任务项 | 负责人 | 完成情况
```

风险项：

```markdown
- 问题/风险 | 影响 | 应对措施
```

下周计划：

```markdown
- 计划项 | 计划内容
- 计划项 | 负责人 | 计划内容
```

## 5. 建议流程

每周写日报时，把每日核心总结先放入输入文件；周五只需要调整顺序、合并重复工作、补充风险和下周计划，然后运行生成脚本。
