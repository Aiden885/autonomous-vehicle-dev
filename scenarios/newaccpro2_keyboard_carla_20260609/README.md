# newaccpro2 键盘 ACC 决策联调场景

启动：

```bash
scenarios/newaccpro2_keyboard_carla_20260609/run.sh
```

场景 UI 中可以使用“恢复工程”将随场景保存的 `newaccpro2` 画布快照恢复到
`project/newaccpro2`。覆盖恢复前，脚本会把现有工程改名备份，不会直接删除。

Pygame 摄像头窗口需要保持键盘焦点。按键映射：

| 按键 | commandType | 语义 |
|---|---:|---|
| E | 1 | 降低设定速度；待命时按当前速度启控 |
| Q | 2 | 提高设定速度；待命且有历史时继承参数启控 |
| T | 3 | 减小时距 |
| R | 4 | 增大时距 |
| W | 5 | 驾驶员油门状态，仅传递决策指令，本场景不做真实扭矩仲裁 |
| S | 6 | 驾驶员制动并退出 ACC，画布将控制使能置 0 |
| C | 7 | 取消 ACC |
| V | - | 切换摄像头视角 |
| ESC | - | 关闭摄像头窗口 |

E/Q/R/T/C 是单周期事件。W/S 从按下持续到释放；其中 S 的退出效果在首次收到 `commandType=6` 后即生效。

横向控制由 Bridge 执行，采用当前偏移与动态预瞄偏移加权的 PID；GAASD 画布负责 ACC 决策和纵向目标速度。

## 重点验收

1. 按 E 或 Q 启控后，确认 `enable=1`，自车开始执行 ACC 跟车。
2. 按 T/R，确认 `timeGap` 每次只变化一个步长。
3. 按 S，确认 `commandType=6`，随后 `controlEnabled` 和 `enable` 变为 0，自车进入制动。
4. 松开 S 后 `commandType` 回到 0，但 ACC 保持退出，不应自动重新启控。
5. 按 C，确认同样退出 ACC；再次按 E/Q 才允许重新启控。

当前 W 只验证 `commandType=5` 能进入 GAASD，暂不验收真实扭矩仲裁。
