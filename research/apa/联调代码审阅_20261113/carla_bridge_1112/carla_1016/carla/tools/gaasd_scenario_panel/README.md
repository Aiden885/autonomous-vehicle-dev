# GAASD-CARLA 场景面板

扫描 `scenarios/*/scenario.yaml`，提供场景启动、参数设置、车辆观测和日志。APA 配置与协议见 [APA 场景说明](../../scenarios/apa/README.md)。

## 启动

在 `carla/` 目录执行：

```bash
python3 -m pip install -r tools/gaasd_scenario_panel/requirements.txt
python3 -m pip install -r tools/carla_bridge/requirements.txt
tools/gaasd_scenario_panel/start-panel.sh
```

默认地址为 `http://127.0.0.1:8088`，端口由 `GAASD_PANEL_PORT` 设置。Bridge 的 Python 须兼容 CARLA API；Pillow 用于影像编码。桌面入口可通过 `tools/gaasd_scenario_panel/install-desktop-launcher.sh` 安装。

## APA 操作

1. 启动环境，等待车辆反馈及算法连接。
2. 选择地图、目标车位和起点，再点击“应用设置并重置”。切图前须退出 APA 并停稳；设置仅在当前会话生效。
3. “仅规划路径”预览并保持驻车；“规划并泊入”重新规划后执行；“开始泊出”使用配置目标或本次泊入前保存的返回起点。
4. “退出并停车”保留位置和本轮数据；“重置任务”回到生效起点并清空本轮显示数据，磁盘日志保留。

车位显示空闲、占用、障碍阻挡或待确认；边缘受限单独标注，泊入仅接受已确认空闲。空闲不代表路径可行。底图异常时显示原因和“重新加载地图”，不在线采样。

命令以当前事件的最终回执为准，HTTP 202 表示执行中。失败保留草稿；超时且没有执行中回执时，可沿用原事件“重新确认上次操作”。加载和确认期间禁用冲突操作，退出保留抢占能力。

## 观测与报告

- 顶部统一显示生效地图/车位、算法连接和状态连接。任务栏显示当前阶段及原因。
- 轨迹为后轴中心，车身包络使用当前车型尺寸及倒角。厘米级间隙以几何净距为准；透视影像和显示线宽不作为碰撞依据。转向反馈表示已施加命令。
- “附近”和“完整输入”显示当前局部场景；“全部车位”只扩大车位目录视野，障碍仍限于当前输入窗口。图层开关仅影响显示，帧号、数量和来源位于“轨迹详情”。
- 浅灰车位边框包含占用及未列入目标下拉的车位，黄色仅标识生效目标。标线资产与当前 OpenDRIVE 摘要匹配，显示边框不作为算法约束。
- 俯视与轨迹图保持生效起点的固定朝向；倒车影像使用车载视角。同会话短暂断流保留画面，新会话或重置起点作废旧图。
- 报告使用打开时的快照，按 CARLA 帧去重并使用仿真时间。CSV 保留全部已采集反馈；加速度平滑仅用于曲线显示。

## 源码与接口

| 文件 | 职责 |
| --- | --- |
| `app.py` | 场景启动、HTTP API、遥测及命令回执 |
| `apa_input_map.py` | 提取同会话/任务的 Bridge 原始场景与边界，不重建几何 |
| `apa_parking_display.py` | 校验并合并标线资产和 Bridge 车位目录 |
| `templates/index.html`、`static/app.js`、`static/style.css` | 页面结构、交互及样式；APA 样式限定于 `is-apa` |

APA API 前缀为 `/api/scenarios/<id>`：

- `GET /telemetry`：状态、场景、轨迹和命令回执；断连或反馈不可用时不显示缓存实际值。
- `GET /parking-slots?session_id=...`：按地图会话加载显示目录，切图和断连作废旧请求。
- `POST /apa-command`：`plan_only/park_in/park_out/cancel/reset/configure/prepare_map`，携带 `session_id/task_id/event_id`；配置额外携带地图、车位及起点。

面板共用 `tools/carla_bridge/apa_scene_limits.py` 和 `apa_scene_diagnostics.py`。更新模板或后端后重载面板，更新静态资源后刷新网页。
