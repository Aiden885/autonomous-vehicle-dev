#!/usr/bin/env python3
"""
Workaround for GAASD gaas_codegen bug in AssemblyTruthTable.

Bug: condition_code variable is overwritten on every iteration of
condition_matching loop, causing if() wrappers to be lost when a
don't-care condition (value=0) follows a True condition (value=1).

This script reads the truth-table JSON config and regenerates the
correct C implementation, replacing the broken generated file.

Usage: python3 fix_truth_table.py
"""
import json
import os

PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
JSON_PATH = os.path.join(
    PROJECT_DIR,
    "icvos/blocks/functions/truth_table_ca11a4a0_e76b_470f_9504_1776d3c041b1.json"
)
OUT_PATH = os.path.join(
    PROJECT_DIR,
    "icvos/src/oscilloscopeFunctions/truth_table_ca11a4a0_e76b_470f_9504_1776d3c041b1",
    "truth_table_ca11a4a0_e76b_470f_9504_1776d3c041b1.c"
)


def generate_truth_table_c(json_path, out_path):
    with open(json_path) as f:
        data = json.load(f)

    base_config = None
    for ep in data["extensionProps"]:
        if ep["key"] == "base_config":
            base_config = ep["value"][0]
            break

    conditions = {c["id"]: c["condition"] for c in base_config["conditions"]}
    actions    = {a["id"]: a["action"]    for a in base_config["actions"]}

    # Read input/output port names
    inputs  = []
    outputs = []
    for ep in data["extensionProps"]:
        if ep["key"] == "function_input_ports":
            inputs = [(p["props"][1]["value"], p["props"][0]["value"])
                      for p in ep["value"]]  # (dataType, name)
        if ep["key"] == "function_output_ports":
            outputs = [(p["props"][1]["value"], p["props"][0]["value"])
                       for p in ep["value"]]

    func_name   = data["name"]
    return_type = outputs[0][0] if outputs else "int"
    return_name = outputs[0][1] if outputs else "y"

    # Build param list
    param_lines = []
    for i, (dtype, name) in enumerate(inputs):
        comment_map = {"u": "真值表输入端口 (state: 0=S0在控, 1=S1有史, 2=S2无史, 3=S3低速)",
                       "v": "真值表输入端口 (command: 0=无, 1=降速, 2=增速, 3=降距, 4=增距, 5=油门, 6=刹车, 7=取消)"}
        comma = "," if i < len(inputs) - 1 else ""
        param_lines.append(f"    {dtype} {name}{comma}  // [in] {comment_map.get(name, '')}")

    # Generate if-else chain from scenarios
    scenarios = base_config["scenarios"]
    branches = []
    has_else = False
    for scene in scenarios:
        cm = scene["condition_matching"]
        true_conds = [conditions[cid] for cid, val in cm.items() if val == 1]
        action = actions[scene["action_matching"]]

        if not true_conds:
            has_else = True
            branches.append(("else", None, action))
        else:
            combined = "&&".join(true_conds)
            branches.append(("if", combined, action))

    lines = []
    lines.append("/* 真值表 */")
    lines.append(f"{return_type} {func_name}(")
    lines.extend(param_lines)
    lines.append(")")
    lines.append("{")
    lines.append(f"    {return_type} {return_name};")

    for i, (op, cond, action) in enumerate(branches):
        if op == "else":
            lines.append(f"    }} else {{")
        elif i == 0:
            lines.append(f"    if({cond}) {{")
        else:
            lines.append(f"    }} else if({cond}) {{")
        lines.append(f"      {action};")

    lines.append("    }")
    lines.append("")
    lines.append(f"    return {return_name};")
    lines.append("}")
    lines.append("")

    content = "\n".join(lines)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w") as f:
        f.write(content)
    print(f"[OK] 生成: {out_path}")
    print("--- 生成内容预览 ---")
    print(content)


HEADER_PATH = os.path.join(
    PROJECT_DIR,
    "icvos/src/oscilloscopeFunctions/accpro2.h"
)

STUB_MARKER = "// Stub protobuf types"
STUB_CODE = (
    "// Stub protobuf types — not using protobuf-c library in this build\n"
    "typedef struct { int reserved; } ProtobufCMessage;\n"
)


def fix_header(header_path):
    with open(header_path) as f:
        content = f.read()

    # Remove protobuf-c include if present
    content = content.replace('#include "protobuf-c/protobuf-c.h"\n', "")

    # Inject stub types after TYPEDEFS_END if not already there
    if STUB_MARKER not in content:
        content = content.replace(
            "// TYPEDEFS_END",
            "// TYPEDEFS_END\n\n" + STUB_CODE
        )

    with open(header_path, "w") as f:
        f.write(content)
    print(f"[OK] 修复头文件: {header_path}")


if __name__ == "__main__":
    fix_header(HEADER_PATH)
    generate_truth_table_c(JSON_PATH, OUT_PATH)
