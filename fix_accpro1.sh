#!/bin/bash
# 修复 GAASD 生成代码的问题，每次生成代码后运行此脚本，再点运行

PROJECT="/data/aiden/文档/Modularization/project/accpro1/icvos/src"

echo "=== Step 1: 修复 protobuf-c 依赖及缺失的 THICV 枚举类型 ==="
for FILE in \
    "$PROJECT/functions/accpro1.h" \
    "$PROJECT/oscilloscopeFunctions/accpro1.h"; do
    if [ -f "$FILE" ]; then
        python3 -W ignore -c "
import re
with open('$FILE', 'r') as f:
    content = f.read()
# 1. 注释掉 protobuf-c include，添加 stub（仅当原始未注释的 include 存在时）
if '#include \"protobuf-c/protobuf-c.h\"' in content and '// #include \"protobuf-c/protobuf-c.h\"' not in content:
    content = content.replace(
        '#include \"protobuf-c/protobuf-c.h\"',
        '// #include \"protobuf-c/protobuf-c.h\"\ntypedef struct { int dummy; } ProtobufCMessage;\ntypedef int Infopack__TrafficLight__State;\n#define PROTOBUF_C_MESSAGE_INIT(x) {0}'
    )
# 去重（多次运行时）
content = re.sub(r'(typedef struct \{ int dummy; \} ProtobufCMessage;\n)+',
                 'typedef struct { int dummy; } ProtobufCMessage;\n', content)
content = re.sub(r'(typedef int Infopack__TrafficLight__State;\n)+',
                 'typedef int Infopack__TrafficLight__State;\n', content)
with open('$FILE', 'w') as f:
    f.write(content)
print('Fixed protobuf: $FILE')
"
    fi
done

echo "=== Step 2: 修复 ACC_DESIRED_DIST / ACC_MAX_SPEED 宏缺失 ==="
for FILE in \
    "$PROJECT/functions/accComputeTargetSpeed/accComputeTargetSpeed.c" \
    "$PROJECT/oscilloscopeFunctions/accComputeTargetSpeed/accComputeTargetSpeed.c"; do
    if [ -f "$FILE" ]; then
        python3 -W ignore -c "
with open('$FILE', 'r') as f:
    content = f.read()
macro = '''#ifndef ACC_DESIRED_DIST
#define ACC_DESIRED_DIST (15.0)
#endif
#ifndef ACC_MAX_SPEED
#define ACC_MAX_SPEED (20.0 / 3.6)
#endif
'''
if 'ACC_DESIRED_DIST' not in content.split('accComputeTargetSpeed')[0]:
    content = content.replace('#include \"accpro1.h\"', '#include \"accpro1.h\"\n' + macro)
with open('$FILE', 'w') as f:
    f.write(content)
print('Fixed macros: $FILE')
"
    fi
done

echo "=== Step 3: 修复 vehiclePoint->..x 双点问题 ==="
for FILE in \
    "$PROJECT/functions/execute_dd594018/execute_dd594018.c" \
    "$PROJECT/oscilloscopeFunctions/execute_dd594018/execute_dd594018.c"; do
    if [ -f "$FILE" ]; then
        sed -i 's/->\.\./->/g' "$FILE"
        echo "Fixed double-dot: $FILE"
    fi
done

echo "=== Step 4: 修复 leadPoint 类型（VehicleState 值→指针，更新成员访问符） ==="
for FILE in \
    "$PROJECT/functions/execute_dd594018/execute_dd594018.c" \
    "$PROJECT/oscilloscopeFunctions/execute_dd594018/execute_dd594018.c"; do
    if [ -f "$FILE" ]; then
        python3 -W ignore -c "
with open('$FILE', 'r') as f:
    content = f.read()
# 已修过则跳过
if 'VehicleState * leadPoint' in content:
    print('Already patched: $FILE')
    exit()
# 函数签名：VehicleState leadPoint → VehicleState * leadPoint
content = content.replace(
    'VehicleState leadPoint  // [in] 函数输入端口',
    'VehicleState * leadPoint  // [in] 函数输入端口（指针，指向外部leadVehicle）'
)
# 成员访问：leadPoint.x → leadPoint->x, leadPoint.v → leadPoint->v
import re
content = re.sub(r'\bleadPoint\.x\b', 'leadPoint->x', content)
content = re.sub(r'\bleadPoint\.v\b', 'leadPoint->v', content)
content = re.sub(r'\bleadPoint\.a\b', 'leadPoint->a', content)
# vehicleModelUpdate(accel, dt, leadPoint) 传入指针，无需改动
with open('$FILE', 'w') as f:
    f.write(content)
print('Fixed leadPoint pointer: $FILE')
"
    fi
done

echo "=== Step 5: 修复 pidTestMain 中 temp_leadVehicle 指针类型（double* → VehicleState*） ==="
for FILE in \
    "$PROJECT/functions/pidTestMain_16044152_4d25_400b_a90c_555a157c2bda/pidTestMain_16044152_4d25_400b_a90c_555a157c2bda.c" \
    "$PROJECT/oscilloscopeFunctions/pidTestMain_16044152_4d25_400b_a90c_555a157c2bda/pidTestMain_16044152_4d25_400b_a90c_555a157c2bda.c"; do
    if [ -f "$FILE" ]; then
        python3 -W ignore -c "
with open('$FILE', 'r') as f:
    content = f.read()
if 'VehicleState* temp' in content or 'VehicleState * temp' in content:
    print('Already patched: $FILE')
    exit()
# GAASD 把 address-of 的临时变量生成为 double*，实际应是 VehicleState*
import re
content = re.sub(r'\bdouble\*\s+(temp\w+);(\s*\n\s*\1\s*=\s*&leadVehicle)', r'VehicleState * \1;\2', content)
with open('$FILE', 'w') as f:
    f.write(content)
print('Fixed temp pointer type: $FILE')
"
    fi
done

echo "=== Step 6: 修复 accpro1.h 中 execute_dd594018 声明（leadPoint 值→指针） ==="
for FILE in \
    "$PROJECT/functions/accpro1.h" \
    "$PROJECT/oscilloscopeFunctions/accpro1.h"; do
    if [ -f "$FILE" ]; then
        python3 -W ignore -c "
with open('$FILE', 'r') as f:
    content = f.read()
content = content.replace(
    'VehicleState leadPoint  // [in] 函数输入端口',
    'VehicleState * leadPoint  // [in] 函数输入端口'
)
with open('$FILE', 'w') as f:
    f.write(content)
print('Fixed header declaration: $FILE')
"
    fi
done

echo "=== Done. Now click Run in GAASD. ==="
