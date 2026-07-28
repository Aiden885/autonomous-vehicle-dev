#!/usr/bin/env python3
"""Patch GAASD generated CARLA headers and truth-table C files after code generation.

GAASD currently regenerates headers and C files with the following issues:

1. It includes protobuf-c headers that are not installed in ``ubuntuenv``.
2. It may omit the ``Infopack__TrafficLight__State`` enum while still using it
   in generated structs.
3. AssemblyTruthTable bug: the condition_code variable is overwritten on every
   iteration of condition_matching, causing if() wrappers to be lost for any
   scenario where a don't-care condition appears after a True condition.

This helper is idempotent and only patches generated files under the selected
GAASD project directory.
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from typing import Iterable, List, Tuple


PROTOBUF_INCLUDE = '#include "protobuf-c/protobuf-c.h"'
PROTOBUF_STUB = """/* P0 CARLA components do not use protobuf runtime APIs directly. Keep the
 * generated message structs compilable in the GAASD ubuntuenv image, where
 * protobuf-c headers are not installed. */
/* #include "protobuf-c/protobuf-c.h" */
typedef struct { int dummy; } ProtobufCMessage;
#define PROTOBUF_C_MESSAGE_INIT(x) {0}"""

ENUM_NAME = "Infopack__TrafficLight__State"
TRAFFIC_LIGHT_ENUM = """typedef enum {
    INFOPACK__TRAFFIC_LIGHT__STATE__RED_LIGHT = 0,
    INFOPACK__TRAFFIC_LIGHT__STATE__YELLOW_LIGHT = 1,
    INFOPACK__TRAFFIC_LIGHT__STATE__GREEN_LIGHT = 2
} Infopack__TrafficLight__State;
"""


def generated_headers(project_dir: Path) -> List[Path]:
    src_dir = project_dir / "icvos" / "src"
    headers: List[Path] = []
    for source_dir in (src_dir / "functions", src_dir / "oscilloscopeFunctions"):
        if not source_dir.exists():
            continue
        headers.extend(sorted(source_dir.glob("*.h")))
    return headers


def patch_header(path: Path) -> Tuple[bool, str]:
    if not path.exists():
        return False, f"SKIP {path}: not found"

    text = path.read_text(encoding="utf-8")
    updated = text

    lines = updated.splitlines()
    replaced_lines: List[str] = []
    include_replaced = False
    for line in lines:
        if line.strip() == PROTOBUF_INCLUDE:
            replaced_lines.extend(PROTOBUF_STUB.splitlines())
            include_replaced = True
        else:
            replaced_lines.append(line)
    if include_replaced:
        trailing_newline = "\n" if updated.endswith("\n") else ""
        updated = "\n".join(replaced_lines) + trailing_newline

    enum_missing = re.search(r"}\s*Infopack__TrafficLight__State\s*;", updated) is None
    enum_used = ENUM_NAME in updated or "INFOPACK__TRAFFIC_LIGHT__INIT" in updated
    if enum_missing and enum_used:
        marker = "// ENUMS_START"
        if marker not in updated:
            return False, f"ERROR {path}: enum section marker not found"
        updated = updated.replace(marker, f"{marker}\n\n{TRAFFIC_LIGHT_ENUM}", 1)

    if updated == text:
        return False, f"OK {path}: already patched"

    path.write_text(updated, encoding="utf-8")
    return True, f"PATCHED {path}"


def _build_truth_table_c(json_path: Path) -> str:
    """Generate correct C code from a truth-table JSON config.

    Workaround for gaas_codegen AssemblyTruthTable bug: condition_code is
    overwritten on every loop iteration, losing the if() wrapper when a
    don't-care condition follows a True condition.
    """
    data = json.loads(json_path.read_text(encoding="utf-8"))

    base_config = None
    for ep in data.get("extensionProps", []):
        if ep.get("key") == "base_config":
            base_config = ep["value"][0]
            break
    if base_config is None:
        return ""

    conditions = {c["id"]: c["condition"] for c in base_config.get("conditions", [])}
    actions    = {a["id"]: a["action"]    for a in base_config.get("actions", [])}

    inputs, outputs = [], []
    for ep in data.get("extensionProps", []):
        if ep.get("key") == "function_input_ports":
            inputs  = [(p["props"][1]["value"], p["props"][0]["value"]) for p in ep["value"]]
        if ep.get("key") == "function_output_ports":
            outputs = [(p["props"][1]["value"], p["props"][0]["value"]) for p in ep["value"]]

    func_name   = data["name"]
    return_type = outputs[0][0] if outputs else "int"
    return_name = outputs[0][1] if outputs else "y"

    param_lines = []
    for i, (dtype, name) in enumerate(inputs):
        comma = "," if i < len(inputs) - 1 else ""
        param_lines.append(f"    {dtype} {name}{comma}")

    branches = []
    for scene in base_config.get("scenarios", []):
        cm = scene.get("condition_matching", {})
        true_conds = [conditions[cid] for cid, val in cm.items() if val == 1 and cid in conditions]
        action = actions.get(scene.get("action_matching", ""), "")
        if not true_conds:
            branches.append(("else", None, action))
        else:
            branches.append(("if", "&&".join(true_conds), action))

    lines = [f"/* 真值表 */", f"{return_type} {func_name}("]
    lines.extend(param_lines)
    lines += [")", "{", f"    {return_type} {return_name};"]
    for i, (op, cond, action) in enumerate(branches):
        if op == "else":
            lines.append("    } else {")
        elif i == 0:
            lines.append(f"    if({cond}) {{")
        else:
            lines.append(f"    }} else if({cond}) {{")
        lines.append(f"      {action};")
    lines += ["    }", "", f"    return {return_name};", "}", ""]
    return "\n".join(lines)


def fix_truth_tables(project_dir: Path) -> List[str]:
    """Regenerate all truth-table C files in a project, bypassing the codegen bug."""
    messages: List[str] = []
    blocks_dir = project_dir / "icvos" / "blocks" / "functions"
    src_osc    = project_dir / "icvos" / "src" / "oscilloscopeFunctions"
    src_fn     = project_dir / "icvos" / "src" / "functions"

    for json_path in sorted(blocks_dir.glob("truth_table_*.json")):
        name = json_path.stem
        content = _build_truth_table_c(json_path)
        if not content:
            messages.append(f"SKIP {json_path.name}: could not parse config")
            continue
        for src_dir in (src_osc, src_fn):
            c_path = src_dir / name / f"{name}.c"
            if not c_path.parent.exists():
                continue
            existing = c_path.read_text(encoding="utf-8") if c_path.exists() else ""
            if existing == content:
                messages.append(f"OK {c_path}: already correct")
            else:
                c_path.write_text(content, encoding="utf-8")
                messages.append(f"PATCHED {c_path}")
    return messages


def sync_funcstep_from_main(project_dir: Path) -> List[str]:
    """Sync truth_table call arguments from src/functions/main.c into
    src/oscilloscopeFunctions/FuncStep.c.

    GAASD bug: 'generate code' updates main.c but not FuncStep.c, causing the
    oscilloscope simulation to run with stale constant values.
    """
    messages: List[str] = []
    fn_dir  = project_dir / "icvos" / "src" / "functions"
    osc_dir = project_dir / "icvos" / "src" / "oscilloscopeFunctions"

    for main_c in sorted(fn_dir.glob("*/main.c")):
        component = main_c.parent.name
        funcstep_c = osc_dir / component / "FuncStep.c"
        if not funcstep_c.exists():
            continue

        main_text     = main_c.read_text(encoding="utf-8")
        funcstep_text = funcstep_c.read_text(encoding="utf-8")

        # Extract every function call line from main.c and apply to FuncStep.c
        changed = False
        updated = funcstep_text
        for line in main_text.splitlines():
            stripped = line.strip()
            # Match assignment lines like: temp123 = some_func(args);
            m = re.match(r"(\w+)\s*=\s*(\w+)\s*\((.+)\)\s*;", stripped)
            if not m:
                continue
            func_name, args = m.group(2), m.group(3)
            # Find the matching call in FuncStep.c (may have different temp var)
            pattern = re.compile(
                r"(\w+)\s*=\s*" + re.escape(func_name) + r"\s*\([^)]*\)\s*;"
            )
            replacement = f"\\1 = {func_name}({args});"
            new_updated = pattern.sub(replacement, updated)
            if new_updated != updated:
                updated = new_updated
                changed = True

        if changed:
            funcstep_c.write_text(updated, encoding="utf-8")
            messages.append(f"SYNCED {funcstep_c}")
        else:
            messages.append(f"OK {funcstep_c}: in sync")
    return messages


def unique_existing_projects(projects: Iterable[str]) -> List[Path]:
    result: List[Path] = []
    seen = set()
    for raw in projects:
        path = Path(raw).expanduser().resolve()
        if path in seen:
            continue
        seen.add(path)
        result.append(path)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Patch GAASD generated CARLA headers, truth tables, and FuncStep files."
    )
    parser.add_argument(
        "--project",
        action="append",
        required=True,
        help="GAASD project directory, can be passed multiple times.",
    )
    args = parser.parse_args()

    changed_count = 0
    checked_count = 0
    for project_dir in unique_existing_projects(args.project):
        if not project_dir.exists():
            print(f"SKIP {project_dir}: project directory not found")
            continue
        print(f"PROJECT {project_dir}")
        for header in generated_headers(project_dir):
            changed, message = patch_header(header)
            checked_count = checked_count + 1
            changed_count = changed_count + (1 if changed else 0)
            print(message)
        for message in fix_truth_tables(project_dir):
            print(message)
        for message in sync_funcstep_from_main(project_dir):
            print(message)

    if checked_count == 0:
        print("ERROR no generated headers were checked")
        return 1

    print(f"DONE checked={checked_count} patched={changed_count}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
