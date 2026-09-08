#!/usr/bin/env python3
"""Apply reviewed Chinese display aliases to internal ACC/LKS canvas nodes."""

from __future__ import annotations

import argparse
import copy
import re
import sqlite3
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
MAPPING_DOCUMENT = ROOT / "docs" / "ACC_LKS_模块别名中文对照表_20260825.md"
PROJECTS = (
    ROOT / "project" / "newaccpro4",
    ROOT / "project" / "lks3",
)
DATABASE_NAMES = ("cbdes.db", "temp.db", "runtimeCanvas.db")

# Pangu runtime structure and external channels must remain technical identifiers.
# Internal input/output blocks may use Chinese display aliases because their
# actual interface names are stored separately in extensionProps.
PROTECTED_TYPES = {
    "app",
    "process",
    "module",
    "composite-block",
    "channel_input",
    "channel_output",
    "oscilloscope",
}
IDENTIFIER_PATTERN = re.compile(r"[A-Za-z][A-Za-z0-9_]*")
MAPPING_ROW_PATTERN = re.compile(r"^\| `([^`]+)` \| ([^|]+?) \|$", re.MULTILINE)


def load_mapping() -> dict[str, str]:
    """Load the reviewed English-to-Chinese alias table."""
    text = MAPPING_DOCUMENT.read_text(encoding="utf-8")
    mapping: dict[str, str] = {}
    for source_alias, target_alias in MAPPING_ROW_PATTERN.findall(text):
        existing = mapping.get(source_alias)
        if existing is not None and existing != target_alias:
            raise RuntimeError(f"Conflicting mapping for {source_alias}")
        mapping[source_alias] = target_alias
    if not mapping:
        raise RuntimeError(f"No alias mappings found in {MAPPING_DOCUMENT}")
    return mapping


def load_table(connection: sqlite3.Connection, table: str) -> dict[str, dict[str, Any]]:
    """Read a database table keyed by its id column."""
    connection.row_factory = sqlite3.Row
    return {
        row["id"]: dict(row)
        for row in connection.execute(f"SELECT * FROM {table} ORDER BY id")
    }


def is_target(row: dict[str, Any]) -> bool:
    """Return whether an internal node alias should be translated."""
    alias = row.get("alias") or ""
    return (
        row.get("componentType") not in PROTECTED_TYPES
        and IDENTIFIER_PATTERN.fullmatch(alias) is not None
    )


def build_plan(
    components: dict[str, dict[str, Any]], mapping: dict[str, str]
) -> dict[str, str]:
    """Build and validate component-id to Chinese-alias updates."""
    plan: dict[str, str] = {}
    missing: set[str] = set()
    aliases_by_parent: dict[tuple[str, str], str] = {}

    for component_id, row in components.items():
        alias = row.get("alias") or ""
        target_alias = alias
        if is_target(row):
            target_alias = mapping.get(alias, "")
            if not target_alias:
                missing.add(alias)
            else:
                plan[component_id] = target_alias

        sibling_key = (row.get("parentId") or "", target_alias)
        existing_id = aliases_by_parent.get(sibling_key)
        if existing_id is not None:
            raise RuntimeError(
                "Duplicate sibling alias after migration: "
                f"parent={sibling_key[0]} alias={target_alias} "
                f"components={existing_id},{component_id}"
            )
        aliases_by_parent[sibling_key] = component_id

    if missing:
        raise RuntimeError(f"Missing reviewed aliases: {sorted(missing)}")
    return plan


def verify_changes(
    before_components: dict[str, dict[str, Any]],
    after_components: dict[str, dict[str, Any]],
    before_connections: dict[str, dict[str, Any]],
    after_connections: dict[str, dict[str, Any]],
    plan: dict[str, str],
) -> None:
    """Verify that only planned project_component.alias values changed."""
    if before_connections != after_connections:
        raise RuntimeError("Canvas connections changed during alias migration")
    if before_components.keys() != after_components.keys():
        raise RuntimeError("Canvas component IDs changed during alias migration")

    for component_id, before in before_components.items():
        expected = copy.deepcopy(before)
        if component_id in plan:
            expected["alias"] = plan[component_id]
        if expected != after_components[component_id]:
            raise RuntimeError(f"Unexpected component change: {component_id}")


def inspect_database(database: Path, mapping: dict[str, str]) -> tuple[int, str]:
    """Return the planned update count and SQLite integrity result."""
    connection = sqlite3.connect(database)
    try:
        components = load_table(connection, "project_component")
        plan = build_plan(components, mapping)
        integrity = connection.execute("PRAGMA integrity_check").fetchone()[0]
        return len(plan), integrity
    finally:
        connection.close()


def update_database(database: Path, mapping: dict[str, str]) -> int:
    """Apply a validated alias-only transaction to one canvas database."""
    connection = sqlite3.connect(database)
    try:
        before_components = load_table(connection, "project_component")
        before_connections = load_table(connection, "project_connection")
        plan = build_plan(before_components, mapping)

        with connection:
            for component_id, target_alias in plan.items():
                connection.execute(
                    "UPDATE project_component SET alias = ? WHERE id = ?",
                    (target_alias, component_id),
                )

        after_components = load_table(connection, "project_component")
        after_connections = load_table(connection, "project_connection")
        verify_changes(
            before_components,
            after_components,
            before_connections,
            after_connections,
            plan,
        )
        integrity = connection.execute("PRAGMA integrity_check").fetchone()[0]
        if integrity != "ok":
            raise RuntimeError(f"SQLite integrity check failed: {database}")
        return len(plan)
    finally:
        connection.close()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--apply", action="store_true")
    args = parser.parse_args()
    mapping = load_mapping()

    for project in PROJECTS:
        print(f"[{project.name}]")
        for database_name in DATABASE_NAMES:
            database = project / "data" / database_name
            planned, integrity = inspect_database(database, mapping)
            print(f"  {database_name}: planned={planned} integrity={integrity}")
            if integrity != "ok":
                raise RuntimeError(f"SQLite integrity check failed: {database}")

    if not args.apply:
        print("Dry run complete. Use --apply to write the reviewed aliases.")
        return

    for project in PROJECTS:
        changed = 0
        for database_name in DATABASE_NAMES:
            changed += update_database(project / "data" / database_name, mapping)
        print(f"[{project.name}] updated rows across databases: {changed}")

        cbdes = project / "data" / "cbdes.db"
        temp = project / "data" / "temp.db"
        if cbdes.read_bytes() != temp.read_bytes():
            raise RuntimeError(f"cbdes.db and temp.db diverged: {project}")

    print("Chinese module alias migration completed successfully.")


if __name__ == "__main__":
    main()
