#!/usr/bin/env python3
"""Normalize lks2 canvas aliases without changing ports or algorithm behavior."""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import sqlite3
import sys
from pathlib import Path
from typing import Any

from refactor_lks2_hierarchy import flattened_signature


ROOT_ID = "6d9ae1e1-3798-4640-91c6-8217a06e02aa"
PREVIEW_ID = "aa61974c-e982-4ef6-b6c2-e61dba56487d"
RAW_STEER_MULTIPLY_ID = "29549bf9-38ef-4bf7-b8a1-2ff749cebb2b"

SPECIAL_ALIASES = {
    ROOT_ID: "main",
    PREVIEW_ID: "previewDistanceCalculation",
    RAW_STEER_MULTIPLY_ID: "applySteerErrorGain",
}


def json_text(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, separators=(",", ":"))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def descendants(conn: sqlite3.Connection) -> list[sqlite3.Row]:
    return list(
        conn.execute(
            """
            WITH RECURSIVE tree(id) AS (
              SELECT ?
              UNION ALL
              SELECT component.id
              FROM project_component component
              JOIN tree ON component.parentId=tree.id
            )
            SELECT component.*
            FROM project_component component
            JOIN tree ON component.id=tree.id
            """,
            (ROOT_ID,),
        )
    )


def lower_camel(row: sqlite3.Row) -> str:
    special = SPECIAL_ALIASES.get(row["id"])
    if special is not None:
        return special
    alias = row["alias"]
    if alias and "A" <= alias[0] <= "Z":
        return alias[0].lower() + alias[1:]
    return alias


def structural(row: sqlite3.Row) -> bool:
    return row["componentType"] in {"run", "composite-block"}


def port_signature(
    conn: sqlite3.Connection,
) -> tuple[tuple[str, str, str], ...]:
    result: list[tuple[str, str, str]] = []
    for row in descendants(conn):
        if not structural(row):
            continue
        for section in json.loads(row["extensionProps"]):
            key = section.get("key")
            if key in {"function_input_ports", "function_output_ports"}:
                result.append((row["id"], key, json_text(section.get("value", []))))
    return tuple(sorted(result))


def atomic_configuration_signature(
    conn: sqlite3.Connection,
) -> tuple[tuple[str, str, str], ...]:
    result: list[tuple[str, str, str]] = []
    for row in descendants(conn):
        if structural(row):
            continue
        properties = json.loads(row["properties"] or "{}")
        properties.pop("alias", None)
        result.append(
            (
                row["id"],
                json_text(properties),
                row["extensionProps"] or "",
            )
        )
    return tuple(sorted(result))


def operate_key_signature(
    conn: sqlite3.Connection,
) -> tuple[tuple[str, str], ...]:
    result: list[tuple[str, str]] = []
    for row in descendants(conn):
        for section in json.loads(row["extensionProps"]):
            if section.get("key") != "base_config":
                continue
            for value in section.get("value", []):
                if not isinstance(value, dict):
                    continue
                for operate_key in value.get("operateKeys", []):
                    result.append((row["id"], operate_key))
    return tuple(sorted(result))


def update_alias(
    conn: sqlite3.Connection, component_id: str, alias: str
) -> None:
    row = conn.execute(
        "SELECT properties FROM project_component WHERE id=?", (component_id,)
    ).fetchone()
    if row is None:
        raise RuntimeError(f"Missing component: {component_id}")
    properties = json.loads(row["properties"] or "{}")
    properties["alias"] = alias
    conn.execute(
        "UPDATE project_component SET alias=?,properties=? WHERE id=?",
        (alias, json_text(properties), component_id),
    )


def update_instance_name(
    conn: sqlite3.Connection, component_id: str, instance_name: str
) -> None:
    row = conn.execute(
        "SELECT extensionProps FROM project_component WHERE id=?",
        (component_id,),
    ).fetchone()
    if row is None:
        raise RuntimeError(f"Missing component: {component_id}")
    sections = json.loads(row["extensionProps"])
    updated = False
    for section in sections:
        if section.get("key") != "base_config":
            continue
        values = section.get("value", [])
        if values and isinstance(values[0], dict):
            values[0]["instanceName"] = instance_name
            updated = True
    if not updated:
        raise RuntimeError(
            f"Composite component has no instanceName: {component_id}"
        )
    conn.execute(
        "UPDATE project_component SET extensionProps=? WHERE id=?",
        (json_text(sections), component_id),
    )


def validate_alias_plan(
    rows: list[sqlite3.Row], proposed: dict[str, str]
) -> None:
    aliases_by_parent: dict[str, set[str]] = {}
    for row in rows:
        alias = proposed[row["id"]]
        siblings = aliases_by_parent.setdefault(row["parentId"], set())
        if alias in siblings:
            raise RuntimeError(
                f"Alias collision after normalization: {row['parentId']}.{alias}"
            )
        siblings.add(alias)


def validate(conn: sqlite3.Connection, proposed: dict[str, str]) -> None:
    if conn.execute("PRAGMA integrity_check").fetchone()[0] != "ok":
        raise RuntimeError("SQLite integrity check failed")

    dangling = conn.execute(
        """
        SELECT COUNT(*)
        FROM project_connection connection
        LEFT JOIN project_component source
          ON source.id=json_extract(connection.source,'$.cell')
        LEFT JOIN project_component target
          ON target.id=json_extract(connection.target,'$.cell')
        WHERE source.id IS NULL OR target.id IS NULL
        """
    ).fetchone()[0]
    if dangling:
        raise RuntimeError(f"Found {dangling} dangling connection endpoints")

    cross_boundary = conn.execute(
        """
        SELECT COUNT(*)
        FROM project_connection connection
        JOIN project_component source
          ON source.id=json_extract(connection.source,'$.cell')
        JOIN project_component target
          ON target.id=json_extract(connection.target,'$.cell')
        WHERE source.parentId<>connection.parentId
           OR target.parentId<>connection.parentId
        """
    ).fetchone()[0]
    if cross_boundary:
        raise RuntimeError(f"Found {cross_boundary} cross-boundary connections")

    duplicate_targets = conn.execute(
        """
        SELECT COUNT(*) FROM (
          SELECT parentId,target,COUNT(*) AS count_value
          FROM project_connection
          GROUP BY parentId,target
          HAVING count_value>1
        )
        """
    ).fetchone()[0]
    if duplicate_targets:
        raise RuntimeError(f"Found {duplicate_targets} multiply-driven inputs")

    rows = descendants(conn)
    for row in rows:
        expected_alias = proposed[row["id"]]
        if row["alias"] != expected_alias:
            raise RuntimeError(
                f"Alias mismatch: {row['alias']} != {expected_alias}"
            )
        properties_alias = json.loads(row["properties"] or "{}").get("alias")
        if properties_alias != expected_alias:
            raise RuntimeError(
                f"properties.alias mismatch: {row['alias']}"
            )
        if (
            expected_alias
            and expected_alias[0].isascii()
            and (
                not ("a" <= expected_alias[0] <= "z")
                or "_" in expected_alias
            )
        ):
            raise RuntimeError(
                f"Non-lower-camel alias remains: {expected_alias}"
            )
        if not structural(row):
            continue
        instance_name = None
        for section in json.loads(row["extensionProps"]):
            if section.get("key") != "base_config":
                continue
            values = section.get("value", [])
            if values and isinstance(values[0], dict):
                instance_name = values[0].get("instanceName")
        if instance_name != expected_alias:
            raise RuntimeError(
                f"instanceName mismatch for {expected_alias}: {instance_name}"
            )


def normalize(db_path: Path) -> tuple[int, int, int]:
    conn = sqlite3.connect(db_path)
    conn.row_factory = sqlite3.Row
    conn.execute("PRAGMA foreign_keys=OFF")
    try:
        before_flat = flattened_signature(conn)
        before_ports = port_signature(conn)
        before_atomic = atomic_configuration_signature(conn)
        before_operate_keys = operate_key_signature(conn)
        with conn:
            rows = descendants(conn)
            proposed = {row["id"]: lower_camel(row) for row in rows}
            validate_alias_plan(rows, proposed)
            for row in rows:
                alias = proposed[row["id"]]
                update_alias(conn, row["id"], alias)
                if structural(row):
                    update_instance_name(conn, row["id"], alias)
            validate(conn, proposed)

            after_flat = flattened_signature(conn)
            after_ports = port_signature(conn)
            after_atomic = atomic_configuration_signature(conn)
            after_operate_keys = operate_key_signature(conn)
            if before_flat != after_flat:
                raise RuntimeError("Flattened algorithm connections changed")
            if before_ports != after_ports:
                raise RuntimeError("Composite port names or UUIDs changed")
            if before_atomic != after_atomic:
                raise RuntimeError("Atomic component configuration changed")
            if before_operate_keys != after_operate_keys:
                raise RuntimeError("Global parameter bindings changed")
        return len(before_flat), len(after_flat), len(rows)
    finally:
        conn.close()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--project",
        type=Path,
        default=Path("project/lks2"),
        help="Path to the lks2 GAASD project",
    )
    args = parser.parse_args()

    project = args.project.expanduser().resolve()
    cbdes = project / "data" / "cbdes.db"
    temp = project / "data" / "temp.db"
    if not cbdes.is_file() or not temp.is_file():
        raise RuntimeError(f"Missing GAASD databases under {project / 'data'}")
    if sha256(cbdes) != sha256(temp):
        raise RuntimeError("cbdes.db and temp.db differ before normalization")

    backup = cbdes.with_name("cbdes.db.before_lower_camel_20260731")
    if not backup.exists():
        shutil.copy2(cbdes, backup)

    before_count, after_count, component_count = normalize(cbdes)
    shutil.copy2(cbdes, temp)
    if sha256(cbdes) != sha256(temp):
        raise RuntimeError("cbdes.db and temp.db differ after synchronization")

    print(f"Normalized:    {cbdes}")
    print(f"Backup:        {backup}")
    print(f"Components:    {component_count}")
    print(f"Flat edges:    {before_count} -> {after_count}")
    print(f"SHA256:        {sha256(cbdes)}")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(1)
