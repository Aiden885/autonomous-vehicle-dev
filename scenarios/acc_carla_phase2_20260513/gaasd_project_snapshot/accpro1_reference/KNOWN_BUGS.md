# accpro1 仿真工程 - 已知问题说明

当前工程**可以运行**，但依赖 `fix_accpro1.sh` 脚本在每次生成代码后手动修复。
以下问题是 GAASD 工具本身的缺陷或限制，在上游修复之前需保留该脚本。

---

## 运行流程（当前）

```
1. GAASD 生成代码
2. 运行 fix_accpro1.sh
3. GAASD 点运行（不再重新生成）
```

---

## Bug 列表

### Bug 1：新拖入组件 instanceId 为空字符串
**现象**：从组件库拖入新组件后，画布保存，重新打开出现"Error rendering component"，for 循环无法点开。

**根因**：`/opt/gaasd/resources/app/dist/Service/ProjectComponentService.js` 的 `insert()` 方法，对 `instanceId` 字段为空时没有生成 UUID，直接写入空字符串。

**已修复**：在 `insert()` 的 forEach 循环中增加了对 `instanceId` 的 UUID 自动生成逻辑。修复已直接应用到 GAASD 源码。

**状态**：✅ 已修复（本机生效，升级 GAASD 时需重新应用）

---

### Bug 2：复制变量组件后重命名导致崩溃
**现象**：复制一个 variable 类型组件，然后修改名称，GAASD 报"Error rendering component"，数据库中出现 instanceId 为空的残留记录。

**根因**：疑似与 Bug 1 同源，copy 操作未正确生成 instanceId。

**状态**：⚠️ 未完全验证（Bug 1 修复后未再复现）

---

### Bug 3：input 端口不支持指针类型（`VehicleState *`）
**现象**：在 for 循环体中新增 input 组件时，数据类型下拉列表没有 `VehicleState *` 选项，只有 `VehicleState`（值类型）。导致生成代码中函数签名为值类型，但外部传入的是指针，编译失败。

**具体错误**（生成代码）：
```c
// execute_dd594018.c 签名错误
VehicleState leadPoint        // 应为 VehicleState * leadPoint

// pidTestMain.c 中临时变量类型错误
double* tempXXX = &leadVehicle;  // 应为 VehicleState* tempXXX
```

**临时方案**：
1. 直接修改 `cbdes.db` 中该 input 组件的 `extensionProps`，将 `"dataType":"VehicleState"` 改为 `"dataType":"VehicleState *"`（已执行，GAASD 画布已生效）
2. `fix_accpro1.sh` Step 4/5/6 在每次生成代码后自动修正签名和头文件声明

**状态**：⚠️ 画布已修正，但每次生成代码后仍需 fix 脚本修补

---

### Bug 4：address-of 组件推断临时变量类型错误
**现象**：外层容器中 address-of 组件的输出，在 pidTestMain 生成代码里被推断为 `double*` 而非实际的 `VehicleState*`。

**根因**：GAASD 代码生成器未能正确传播 address-of 操作数的类型信息。

**临时方案**：`fix_accpro1.sh` Step 5 正则替换修正。

**状态**：⚠️ 依赖 fix 脚本

---

### Bug 5：protobuf-c 依赖缺失
**现象**：生成的 `accpro1.h` 包含 `#include "protobuf-c/protobuf-c.h"`，但 Docker 镜像中未安装该库，编译报错。

**临时方案**：`fix_accpro1.sh` Step 1 注释掉该 include，插入最小 stub 类型定义。

**状态**：⚠️ 依赖 fix 脚本（或在 Docker 中安装 libprotobuf-c-dev）

---

## fix_accpro1.sh 各步骤说明

| Step | 修复内容 | 对应 Bug |
|------|---------|---------|
| 1 | 注释 protobuf-c include，添加 stub | Bug 5 |
| 2 | 注入 `ACC_DESIRED_DIST`/`ACC_MAX_SPEED` 宏 | 无对应 Bug，GAASD 无宏支持 |
| 3 | 修复 `vehiclePoint->..x` 双点语法 | GAASD 代码生成偶发问题 |
| 4 | `VehicleState leadPoint` → `VehicleState * leadPoint` | Bug 3 |
| 5 | `double* tempXXX` → `VehicleState* tempXXX` | Bug 4 |
| 6 | 修正 `accpro1.h` 中 execute_dd594018 声明 | Bug 3 |
