# APA 研究与 GAASD 架构

这里保存 APA 当前代码基线、分析文档及架构预览，不是已经搭建完成的 GAASD 工程。

## 先看这些文件

- [GAASD 模块架构设计](APA_GAASD模块架构设计_20261117.md)：当前分层、组件候选、状态与异步边界。
- [HTML 架构预览](APA_GAASD模块架构预览_20261117.html)：下载后用浏览器离线打开，无需插件。
- [联调代码评估](APA_CARLA联调代码评估_20261113.md)：1112 基线的已有检查与验证边界。
- [原 Simulink 原理详解](APA_Simulink全层级架构与模块原理详解.md)：历史架构与原理参考，不替代当前代码。

## 代码与范围

`联调代码审阅_20261113/apa_1112/` 保存 `apa_1112.zip` 的源码基线，
`联调代码审阅_20261113/carla_bridge_1112/` 保存对应 Bridge 源码、配置和场景资源。
两包是后续工作的第一参照；目前只做架构设计，没有修改算法或 Bridge。

已确定：APA 功能全部进入 GAASD，按平台具备异步能力设计；Bridge 职责和外部协议先保持不变。
平台的具体异步接口及生成行为尚待验证。

不保存安装包、原始 ZIP、Git 历史、缓存、CARLA PythonAPI 二进制包和编译输出。
APA 场景依赖的四份 `parking_maps/*.json.gz` 是运行数据，单独保留。
审阅环境临时创建的绝对路径软链接不入库；这份快照不等于可直接部署的发布包。
原始压缩包及本次文件摘要见 `snapshot_manifest.json`。

## 重新导出 HTML

在本目录运行：

```bash
node assets/architecture_preview/build_html_preview.mjs \
  assets/architecture_preview/APA-GAASD-module-architecture.canvas.tsx \
  APA_GAASD模块架构预览_20261117.html
```

只需 Node.js，无需 npm 安装。原 TSX 需要专用 Canvas 运行环境；普通 VS Code 阅读请使用 HTML。

本目录从本机 `/data/aiden/文档/Pro/APA` 同步；保留源码字节，不把已有的编译或回归记录表述为新增物理闭环验证。
