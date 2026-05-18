# GAASD-CARLA Scenario Panel

本工具是本机调试用的场景启动面板，用于快速恢复和启动 `scenarios/` 中保存的 GAASD-CARLA 联调快照。

当前 UI 页面代码已纳入项目备份，核心文件为：

- `tools/gaasd_scenario_panel/app.py`
- `tools/gaasd_scenario_panel/templates/index.html`
- `tools/gaasd_scenario_panel/static/style.css`
- `tools/gaasd_scenario_panel/static/app.js`

2026-05-18 已确认：在 `ACC 跟车 CARLA-GAASD 联合仿真快照` 上点击“启动环境”，可以直接启动 CARLA + Bridge，且 CARLA 初始视角、前车位置、自车位置符合当前设计。

## 启动

```bash
cd ~/文档/Modularization
python3 tools/gaasd_scenario_panel/app.py
```

然后打开：

```text
http://127.0.0.1:8765
```

## 功能

- 自动扫描 `scenarios/*/scenario.yaml`
- 恢复 GAASD 工程快照
- 启动 CARLA + Bridge + 当前工况
- 停止联调环境
- 健康检查 CARLA / Bridge 端口
- 实时显示脚本日志
- 复制恢复后的 GAASD 工程路径

## 依赖

当前环境需要 Flask 和 PyYAML。如需重新安装：

```bash
python3 -m pip install -r tools/gaasd_scenario_panel/requirements.txt
```
