# planningFigure/ACC 函数注释检查报告

检查时间：2026-04-23  
检查范围：`/home/dell/work/sda1/chenao2.0/thicv-pilot/planningFigure/ACC`  
检查依据：[组件编写规范](/home/dell/work/sda1/chenao2.0/gaasd_xibuzhilian/gaasdaemon/codescan_th/docs/组件编写规范.md)

## 检查结论

本次仅按当前规范中的注释硬性要求检查函数注释，不检查函数实现、宏注释、结构体注释。

发现 8 处不符合规范的问题，涉及 6 个函数注释块：

- `@param[IN/OUT]` 不符合现行规范。当前规范只允许通过两条 `@param[IN]` 和 `@param[OUT]` 表达输入且回写的参数，不允许写 `@param[IN/OUT]`。
- 多个 `@retval` 未按规范写出“返回类型 + 返回值变量名 + 返回说明”的完整形式。
- `void` 返回值未按规范写成 `@retval void 返回值为空`。

## 不符合项

### 1. `accSimStep` 源文件注释不符合规范

文件：[accSimStep.c](/home/dell/work/sda1/chenao2.0/thicv-pilot/planningFigure/ACC/accSimStep.c)

问题：

- 使用了 `@param[IN/OUT] AccSimState* sim ...`，不符合当前规范。
- `@retval void` 缺少返回说明；按规范应写为 `@retval void 返回值为空`。

建议：

```c
 * @param[IN] AccSimState* sim ACC仿真状态（读取当前状态）
 * @param[OUT] AccSimState* sim ACC仿真状态（原地更新为下一时刻状态）
 * @retval void 返回值为空
```

### 2. `accSimStep` 头文件注释不符合规范

文件：[accSimStep.h](/home/dell/work/sda1/chenao2.0/thicv-pilot/planningFigure/ACC/accSimStep.h)

问题：

- 使用了 `@param[IN/OUT] AccSimState* sim ...`，不符合当前规范。
- `@retval void` 缺少返回说明；按规范应写为 `@retval void 返回值为空`。

建议：

```c
 * @param[IN] AccSimState* sim ACC仿真状态（读取当前状态）
 * @param[OUT] AccSimState* sim ACC仿真状态（原地更新为下一时刻状态）
 * @retval void 返回值为空
```

### 3. `computeDistance1D` 源文件注释不符合规范

文件：[computeDistance1D.c](/home/dell/work/sda1/chenao2.0/thicv-pilot/planningFigure/ACC/computeDistance1D.c)

问题：

- `@retval double 车间距（m），正值表示前车在自车前方` 缺少返回值变量名。

建议：

```c
 * @retval double distance 车间距（m），正值表示前车在自车前方
```

### 4. `computeDistance1D` 头文件注释不符合规范

文件：[computeDistance1D.h](/home/dell/work/sda1/chenao2.0/thicv-pilot/planningFigure/ACC/computeDistance1D.h)

问题：

- `@retval double 车间距（m），正值表示前车在自车前方` 缺少返回值变量名。

建议：

```c
 * @retval double distance 车间距（m），正值表示前车在自车前方
```

### 5. `computeSpeedError` 源文件注释不符合规范

文件：[computeSpeedError.c](/home/dell/work/sda1/chenao2.0/thicv-pilot/planningFigure/ACC/computeSpeedError.c)

问题：

- `@retval double 速度误差（m/s），正值表示自车需要加速` 缺少返回值变量名。

建议：

```c
 * @retval double error 速度误差（m/s），正值表示自车需要加速
```

### 6. `computeSpeedError` 头文件注释不符合规范

文件：[computeSpeedError.h](/home/dell/work/sda1/chenao2.0/thicv-pilot/planningFigure/ACC/computeSpeedError.h)

问题：

- `@retval double 速度误差（m/s），正值表示自车需要加速` 缺少返回值变量名。

建议：

```c
 * @retval double error 速度误差（m/s），正值表示自车需要加速
```

### 7. `getVehicleV` 源文件与头文件注释不符合规范

文件：

- [getVehicleV.c](/home/dell/work/sda1/chenao2.0/thicv-pilot/planningFigure/ACC/getVehicleV.c)
- [getVehicleV.h](/home/dell/work/sda1/chenao2.0/thicv-pilot/planningFigure/ACC/getVehicleV.h)

问题：

- `@retval double 车辆速度（m/s）` 缺少返回值变量名。

建议：

```c
 * @retval double v 车辆速度（m/s）
```

### 8. `getVehicleX` 源文件与头文件注释不符合规范

文件：

- [getVehicleX.c](/home/dell/work/sda1/chenao2.0/thicv-pilot/planningFigure/ACC/getVehicleX.c)
- [getVehicleX.h](/home/dell/work/sda1/chenao2.0/thicv-pilot/planningFigure/ACC/getVehicleX.h)

问题：

- `@retval double 车辆纵向位置（m）` 缺少返回值变量名。

建议：

```c
 * @retval double x 车辆纵向位置（m）
```

## 符合规范的函数注释

以下函数注释按当前规范检查，未发现硬性不符合项：

- `accComputeTargetSpeed` in [accComputeTargetSpeed.c](/home/dell/work/sda1/chenao2.0/thicv-pilot/planningFigure/ACC/accComputeTargetSpeed.c)
- `accComputeTargetSpeed` in [accComputeTargetSpeed.h](/home/dell/work/sda1/chenao2.0/thicv-pilot/planningFigure/ACC/accComputeTargetSpeed.h)

## 备注

- 本报告未将 `#define`、`typedef struct` 注释计入“函数注释”检查范围。
- 本报告按当前 `codescan_th/docs/组件编写规范.md` 的现行文字执行，未额外扩展新规则。
