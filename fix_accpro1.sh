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
# 1. 注释掉 protobuf-c include，添加 stub
if '#include \"protobuf-c/protobuf-c.h\"' in content:
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
#define ACC_MAX_SPEED (12.0 / 3.6)
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

echo "=== Done. Now click Run in GAASD. ==="
