#!/usr/bin/env python3
"""Align editable GAASD component names with the latest naming convention."""

from __future__ import annotations

import argparse
import json
import shutil
import sqlite3
from datetime import datetime
from pathlib import Path
from typing import Any

from set_acc_lks_chinese_names import (
    ACC_NAMES,
    EDITABLE_ORIGIN_IDS,
    LKS_NAMES,
    ROOT,
)


PROJECTS = {
    ROOT / "project" / "newaccpro4": ACC_NAMES,
    ROOT / "project" / "lks3": LKS_NAMES,
}

DATABASE_NAMES = ("cbdes.db", "temp.db", "runtimeCanvas.db")
ENGLISH_NAME_OVERRIDES = {"acc进程": "AccProcess"}


def pascal_case(name: str) -> str:
    """Convert an existing lower-camel identifier to PascalCase."""
    if name in ENGLISH_NAME_OVERRIDES:
        return ENGLISH_NAME_OVERRIDES[name]
    if not name or not name[0].isascii() or not name[0].isalpha():
        raise ValueError(f"Cannot derive a PascalCase English name from: {name!r}")
    return name[0].upper() + name[1:]


def is_editable_component(row: sqlite3.Row) -> bool:
    return row["componentType"] != "app" and row["originId"] in EDITABLE_ORIGIN_IDS


def load_rows(connection: sqlite3.Connection, table: str) -> dict[str, dict[str, Any]]:
    connection.row_factory = sqlite3.Row
    return {
        row["id"]: dict(row)
        for row in connection.execute(f"SELECT * FROM {table} ORDER BY id")
    }


def build_mapping(
    connection: sqlite3.Connection, names: dict[str, str]
) -> dict[str, tuple[str, str, str]]:
    """Return row ID -> (old alias, PascalCase English name, Chinese alias)."""
    connection.row_factory = sqlite3.Row
    reverse_names = {chinese: english for english, chinese in names.items()}
    mapping: dict[str, tuple[str, str, str]] = {}

    for row in connection.execute(
        "SELECT id, parentId, alias, name, originId, componentType, properties "
        "FROM project_component ORDER BY parentId, id"
    ):
        if not is_editable_component(row):
            continue
        properties = json.loads(row["properties"] or "{}")
        chinese_name = properties.get("alias")
        if not isinstance(chinese_name, str) or not chinese_name:
            raise RuntimeError(f"Missing Chinese name for component {row['id']}")

        source_alias = row["alias"]
        if source_alias == chinese_name:
            source_alias = reverse_names.get(chinese_name, source_alias)
        english_name = pascal_case(source_alias)
        mapping[row["id"]] = (row["alias"], english_name, chinese_name)

    return mapping


def verify_sibling_aliases(
    connection: sqlite3.Connection, mapping: dict[str, tuple[str, str, str]]
) -> None:
    connection.row_factory = sqlite3.Row
    aliases_by_parent: dict[str, set[str]] = {}
    for row in connection.execute("SELECT id, parentId, alias FROM project_component"):
        alias = mapping.get(row["id"], ("", "", row["alias"]))[2]
        aliases = aliases_by_parent.setdefault(row["parentId"], set())
        if alias in aliases:
            raise RuntimeError(
                f"Duplicate sibling alias after migration: parent={row['parentId']} alias={alias}"
            )
        aliases.add(alias)


def verify_only_expected_fields_changed(
    before_components: dict[str, dict[str, Any]],
    after_components: dict[str, dict[str, Any]],
    before_connections: dict[str, dict[str, Any]],
    after_connections: dict[str, dict[str, Any]],
    mapping: dict[str, tuple[str, str, str]],
) -> None:
    if before_connections != after_connections:
        raise RuntimeError("Project connections changed during name migration")
    if before_components.keys() != after_components.keys():
        raise RuntimeError("Project component IDs changed during name migration")

    for component_id, before in before_components.items():
        after = after_components[component_id]
        if component_id not in mapping:
            if before != after:
                raise RuntimeError(f"Protected component changed: {component_id}")
            continue

        expected = dict(before)
        expected["name"] = mapping[component_id][1]
        expected["alias"] = mapping[component_id][2]
        if expected != after:
            raise RuntimeError(f"Unexpected component changes: {component_id}")


def inspect_database(database: Path, names: dict[str, str]) -> dict[str, Any]:
    connection = sqlite3.connect(database)
    connection.row_factory = sqlite3.Row
    integrity = connection.execute("PRAGMA integrity_check").fetchone()[0]
    mapping = build_mapping(connection, names)
    verify_sibling_aliases(connection, mapping)
    rows = load_rows(connection, "project_component")
    mismatches = [
        component_id
        for component_id, (_, english_name, chinese_alias) in mapping.items()
        if rows[component_id]["name"] != english_name
        or rows[component_id]["alias"] != chinese_alias
    ]
    connection.close()
    return {"integrity": integrity, "mapping": mapping, "mismatches": mismatches}


def update_database(database: Path, names: dict[str, str]) -> int:
    connection = sqlite3.connect(database)
    connection.row_factory = sqlite3.Row
    before_components = load_rows(connection, "project_component")
    before_connections = load_rows(connection, "project_connection")
    mapping = build_mapping(connection, names)
    verify_sibling_aliases(connection, mapping)

    changed = 0
    with connection:
        for component_id, (_, english_name, chinese_alias) in mapping.items():
            row = before_components[component_id]
            if row["name"] == english_name and row["alias"] == chinese_alias:
                continue
            connection.execute(
                "UPDATE project_component SET name = ?, alias = ? WHERE id = ?",
                (english_name, chinese_alias, component_id),
            )
            changed += 1

    after_components = load_rows(connection, "project_component")
    after_connections = load_rows(connection, "project_connection")
    verify_only_expected_fields_changed(
        before_components,
        after_components,
        before_connections,
        after_connections,
        mapping,
    )
    integrity = connection.execute("PRAGMA integrity_check").fetchone()[0]
    connection.close()
    if integrity != "ok":
        raise RuntimeError(f"Database integrity check failed: {database}")
    return changed


def backup_databases(project: Path, timestamp: str) -> list[Path]:
    backups = []
    for database_name in DATABASE_NAMES:
        source = project / "data" / database_name
        destination = source.with_name(
            f"{database_name}.before_component_name_pascal_chinese_{timestamp}"
        )
        shutil.copy2(source, destination)
        backups.append(destination)
    return backups


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--apply", action="store_true")
    args = parser.parse_args()

    for project, names in PROJECTS.items():
        report = inspect_database(project / "data" / "cbdes.db", names)
        print(f"[{project.name}] editable components: {len(report['mapping'])}")
        for _, (old_alias, english_name, chinese_alias) in report["mapping"].items():
            print(f"  {old_alias} -> name={english_name}, alias={chinese_alias}")
        for database_name in DATABASE_NAMES:
            database_report = inspect_database(project / "data" / database_name, names)
            print(
                f"  {database_name}: planned={len(database_report['mismatches'])} "
                f"integrity={database_report['integrity']}"
            )
            if database_report["integrity"] != "ok":
                raise RuntimeError(f"Database integrity check failed: {database_name}")

    if not args.apply:
        print("Dry run complete. Use --apply to write the validated migration.")
        return

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    for project, names in PROJECTS.items():
        backups = backup_databases(project, timestamp)
        changed = sum(
            update_database(project / "data" / database_name, names)
            for database_name in DATABASE_NAMES
        )
        print(f"[{project.name}] updated rows across databases: {changed}")
        for backup in backups:
            print(f"  backup: {backup}")

        if (project / "data" / "cbdes.db").read_bytes() != (
            project / "data" / "temp.db"
        ).read_bytes():
            raise RuntimeError(f"cbdes.db and temp.db diverged: {project}")

    print("Component name migration completed successfully.")


if __name__ == "__main__":
    main()
