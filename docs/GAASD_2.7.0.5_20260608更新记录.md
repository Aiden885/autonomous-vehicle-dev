# GAASD 2.7.0.5（20260608 构建）更新记录

## 1. 更新结果

2026 年 6 月 9 日已完成隔离版 GAASD 的更新，系统旧版未被修改。

| 项目 | 当前路径/状态 |
|---|---|
| 系统旧版 | `/opt/gaasd`，`/usr/bin/gaasd` 仍指向该目录 |
| 隔离新版 | `/home/aiden/gaasd_versions/gaasd-2.7.0.5/app` |
| 隔离用户目录 | `/home/aiden/gaasd_versions/gaasd-2.7.0.5/home` |
| 启动脚本 | `/home/aiden/gaasd_versions/gaasd-2.7.0.5/run-gaasd-2.7.0.5.sh` |
| 回滚备份 | `/home/aiden/gaasd_versions/gaasd-2.7.0.5/rollback/20260609_143740` |

本次未执行安装包自带的 `install.sh`。该脚本会覆盖 `/opt/gaasd`、`/usr/bin/gaasd` 和系统桌面项，不适用于当前新旧版本并存方案。

## 2. 使用的更新包

| 更新包 | SHA-256 |
|---|---|
| `GAASD_SETUP_20260608.tar.gz` | `3f21759958c592f3df20711eb6fce08f94d906c6d3e64dc2e8dcf293df93c3af` |
| `gaasd_code_tools_20260606.tar` | `81d0866824bcbadbda22359178864201f7d228bf30a00af81ec9c44111bd8edb` |
| `清华组件包_0608_1.tar` | `7633afba77cfd25a3b17605e2ba5724f14beb438bae88ce7f00fc9c5d2d0ea4e` |

GUI 的产品版本仍为 `2.7.0.5`，属于同版本号下的 20260608 新构建。

## 3. 更新内容

- GUI 文件已替换为 20260608 构建。
- code tools 已更新为 20260606 包，`gaas_codegen` 和 `run_simulation.sh` 均为新文件。
- 清华组件库已更新为 0608_1 包，包含 28 个顶层组件、151 个组件 JSON 和 36 个字典 JSON。
- 基础组件数据库已由 `1.0.6` 更新为 `1.0.8`，并加入新版 `static_cast` 组件。
- `system_environment` 中 code tools 版本已更新为 `20260606`。
- 原有隔离用户数据、工程目录和 13 个自定义导入文件均保留。
- 启动脚本已清除 IDE 可能注入的 `ELECTRON_RUN_AS_NODE`，并自动创建新版所需的 `cacheFile` 目录。

## 4. 验证结果

- `gaasd.db` 和 `user.db`：`PRAGMA integrity_check = ok`。
- 组件表：73 个基础组件、1 个 `static_cast`、28 个清华顶层组件，无重复 `originId`。
- 组件详情表：73 条基础组件详情、1276 条 THICV 详情。
- `gaas_codegen --help`：正常，退出码为 0。
- `codescan --help`：正常，退出码为 0。
- `run_simulation.sh`：Shell 语法检查通过。
- GUI：使用隔离启动脚本连续运行完成数据库初始化，启动检查通过。
- 系统旧版入口：仍为 `/usr/bin/gaasd -> /opt/gaasd/gaasd`。

## 5. 启动方式

可从应用菜单打开 `GAASD 2.7.0.5`，也可在终端执行：

```bash
/home/aiden/gaasd_versions/gaasd-2.7.0.5/run-gaasd-2.7.0.5.sh
```

不要直接执行更新包中的 `install.sh`。

## 6. 已知限制

- 20260606 包内的 `codescan` 仍显示为“C代码扫描工具”，本次更新未提供可确认的新版 C++ `FuncModule` 扫描能力。
- GUI 冒烟测试中出现过由当前 MATLAB 环境库路径引起的 D-Bus 警告，但未阻断界面和数据库初始化。
- GAASD 团队仍在修复示波器、代码扫描和仿真脚本兼容问题，完整 ACC 工程需继续按实际生成与仿真结果验证。

## 7. 回滚位置

更新前的完整目录和数据库位于：

```text
/home/aiden/gaasd_versions/gaasd-2.7.0.5/rollback/20260609_143740/
├── app/
├── codeTools/
├── THICV/
├── database/
│   ├── gaasd.db
│   └── user.db
├── run-gaasd-2.7.0.5.sh
└── update-packages.txt
```

如需回滚，应先关闭隔离版 GAASD，再恢复上述三个目录和两个数据库。不要修改 `/opt/gaasd`。
