#!/usr/bin/env python3
"""Patch GAASD generated CARLA headers after code generation.

GAASD currently regenerates ``carla.h`` with two issues for the CARLA P0
components:

1. It includes protobuf-c headers that are not installed in ``ubuntuenv``.
2. It may omit the ``Infopack__TrafficLight__State`` enum while still using it
   in generated structs.

This helper is idempotent and only patches generated headers under the selected
GAASD project directory.
"""

from __future__ import annotations

import argparse
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
    parser = argparse.ArgumentParser(description="Patch GAASD generated CARLA headers.")
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

    if checked_count == 0:
        print("ERROR no generated headers were checked")
        return 1

    print(f"DONE checked={checked_count} patched={changed_count}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
