#!/usr/bin/env python3
"""Rename newaccpro4's internal final output to an acceleration command.

The external Pangu channel field ``target_speed_mps`` is intentionally kept
unchanged.  Only the two composite boundaries that carry the already-computed
acceleration command are renamed; the intermediate target-speed signal remains
``targetSpeedMps``.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import sqlite3
import tempfile
from datetime import datetime
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROJECT = REPO_ROOT / "project" / "newaccpro4"

CONTROL_ID = "96195785-aad3-4e07-96eb-347ef47d3318"
CONTROL_OUTPUT_ID = "9714e549-d251-4336-809c-ce820b912407"
EXECUTE_ACC_ID = "23d2a150-83e4-470e-a9b7-189357a9dfd1"
EXECUTE_ACC_OUTPUT_ID = "38acc843-8fd7-48bb-958c-42cc95c50a31"
GENERATE_TARGET_SPEED_ID = "53728aa4-1a0c-4dbf-b2ed-63c9c2a4016a"
CHANNEL_OUTPUT_ID = "f2ad6e42-9917-4375-9b9e-c5bcacbc7d3a"

ACCELERATION_NAME = "accelerationCommandMps2"
ACCELERATION_ALIAS = "加速度命令"
OLD_OUTPUT_NAME = "targetSpeedMps"
CONTROL_DESCRIPTION = (
    "根据前车状态、跟车误差和决策参数生成ACC纵向加速度命令。"
)


def json_load(value: str | None, fallback: Any) -> Any:
    if not value:
        return fallback
    return json.loads(value)


def json_text(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, separators=(",", ":"))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def extension_value(extension: list[dict[str, Any]], key: str) -> Any:
    for item in extension:
        if item.get("key") == key:
            return item.get("value")
    raise RuntimeError(f"missing extension key: {key}")


def load_component(
    connection: sqlite3.Connection, component_id: str
) -> sqlite3.Row:
    row = connection.execute(
        "SELECT * FROM project_component WHERE id=?", (component_id,)
    ).fetchone()
    if row is None:
        raise RuntimeError(f"missing component: {component_id}")
    return row


def set_prop_name(props: list[dict[str, Any]], new_name: str) -> None:
    name_props = [item for item in props if item.get("key") == "name"]
    if len(name_props) != 1:
        raise RuntimeError(f"expected one name property, got {len(name_props)}")
    old_name = name_props[0].get("value")
    if old_name not in {OLD_OUTPUT_NAME, ACCELERATION_NAME}:
        raise RuntimeError(f"unexpected output property name: {old_name}")
    name_props[0]["value"] = new_name


def rename_parent_output(
    connection: sqlite3.Connection,
    component_id: str,
    port_id: str,
    update_description: bool,
) -> None:
    row = load_component(connection, component_id)
    if row["componentType"] != "composite-block":
        raise RuntimeError(f"expected composite-block: {component_id}")

    extension = json_load(row["extensionProps"], [])
    ports = extension_value(extension, "function_output_ports") or []
    matches = [port for port in ports if port.get("id") == port_id]
    if len(matches) != 1:
        raise RuntimeError(f"expected one output port {port_id}, got {len(matches)}")
    port = matches[0]
    if port.get("name") not in {OLD_OUTPUT_NAME, ACCELERATION_NAME}:
        raise RuntimeError(f"unexpected parent output name: {port.get('name')}")
    port["name"] = ACCELERATION_NAME
    if "alias" in port:
        port["alias"] = ACCELERATION_NAME
    set_prop_name(port.setdefault("props", []), ACCELERATION_NAME)

    values: list[Any] = [json_text(extension)]
    assignments = ["extensionProps=?"]
    if update_description:
        properties = json_load(row["properties"], {})
        properties["description"] = CONTROL_DESCRIPTION
        assignments.extend(["description=?", "properties=?"])
        values.extend([CONTROL_DESCRIPTION, json_text(properties)])
    values.append(component_id)
    connection.execute(
        f"UPDATE project_component SET {','.join(assignments)} WHERE id=?", values
    )


def rename_output_boundary(
    connection: sqlite3.Connection, component_id: str
) -> None:
    row = load_component(connection, component_id)
    if row["componentType"] != "output":
        raise RuntimeError(f"expected output boundary: {component_id}")
    if row["alias"] not in {"目标速度", ACCELERATION_ALIAS}:
        raise RuntimeError(f"unexpected output alias: {row['alias']}")

    extension = json_load(row["extensionProps"], [])
    configs = extension_value(extension, "base_config") or []
    if len(configs) != 1:
        raise RuntimeError(f"expected one output base_config: {component_id}")
    if configs[0].get("name") not in {OLD_OUTPUT_NAME, ACCELERATION_NAME}:
        raise RuntimeError(f"unexpected boundary name: {configs[0].get('name')}")
    configs[0]["name"] = ACCELERATION_NAME

    ports = extension_value(extension, "function_input_ports") or []
    if len(ports) != 1 or ports[0].get("id") != "in":
        raise RuntimeError(f"unexpected output boundary port: {component_id}")
    set_prop_name(ports[0].setdefault("props", []), ACCELERATION_NAME)

    connection.execute(
        "UPDATE project_component SET alias=?,extensionProps=? WHERE id=?",
        (ACCELERATION_ALIAS, json_text(extension), component_id),
    )


def output_port_name(row: sqlite3.Row, port_id: str) -> str:
    extension = json_load(row["extensionProps"], [])
    ports = extension_value(extension, "function_output_ports") or []
    matches = [port for port in ports if port.get("id") == port_id]
    if len(matches) != 1:
        raise RuntimeError(f"missing output port {port_id}")
    return str(matches[0].get("name"))


def boundary_name(row: sqlite3.Row) -> str:
    extension = json_load(row["extensionProps"], [])
    configs = extension_value(extension, "base_config") or []
    if len(configs) != 1:
        raise RuntimeError(f"unexpected boundary config: {row['id']}")
    return str(configs[0].get("name"))


def validate_database(
    connection: sqlite3.Connection,
    channel_before: tuple[Any, ...],
    connections_before: list[tuple[Any, ...]],
) -> None:
    if output_port_name(
        load_component(connection, CONTROL_ID), CONTROL_OUTPUT_ID
    ) != ACCELERATION_NAME:
        raise RuntimeError("ACC control output was not renamed")
    if output_port_name(
        load_component(connection, EXECUTE_ACC_ID), EXECUTE_ACC_OUTPUT_ID
    ) != ACCELERATION_NAME:
        raise RuntimeError("ExecuteAcc output was not renamed")

    for boundary_id in (CONTROL_OUTPUT_ID, EXECUTE_ACC_OUTPUT_ID):
        row = load_component(connection, boundary_id)
        if row["alias"] != ACCELERATION_ALIAS:
            raise RuntimeError(f"incorrect Chinese alias: {boundary_id}")
        if boundary_name(row) != ACCELERATION_NAME:
            raise RuntimeError(f"incorrect boundary name: {boundary_id}")

    target_speed = load_component(connection, GENERATE_TARGET_SPEED_ID)
    target_ports = json_load(target_speed["extensionProps"], [])
    target_ports = extension_value(target_ports, "function_output_ports") or []
    if len(target_ports) != 1 or target_ports[0].get("name") != OLD_OUTPUT_NAME:
        raise RuntimeError("intermediate target-speed output was modified")

    channel_after = tuple(load_component(connection, CHANNEL_OUTPUT_ID))
    if channel_after != channel_before:
        raise RuntimeError("protected ACC channel output changed")
    channel_extension = json_load(
        load_component(connection, CHANNEL_OUTPUT_ID)["extensionProps"], []
    )
    channel_ports = extension_value(channel_extension, "function_input_ports") or []
    if "target_speed_mps" not in {port.get("name") for port in channel_ports}:
        raise RuntimeError("fixed channel field target_speed_mps is missing")

    connections_after = [
        tuple(row)
        for row in connection.execute("SELECT * FROM project_connection ORDER BY id")
    ]
    if connections_after != connections_before:
        raise RuntimeError("project connections changed during naming migration")
    if connection.execute("PRAGMA integrity_check").fetchone()[0] != "ok":
        raise RuntimeError("SQLite integrity check failed")


def migrate_database(database: Path) -> None:
    connection = sqlite3.connect(database)
    connection.row_factory = sqlite3.Row
    try:
        channel_before = tuple(load_component(connection, CHANNEL_OUTPUT_ID))
        connections_before = [
            tuple(row)
            for row in connection.execute(
                "SELECT * FROM project_connection ORDER BY id"
            )
        ]
        with connection:
            rename_parent_output(
                connection, CONTROL_ID, CONTROL_OUTPUT_ID, update_description=True
            )
            rename_output_boundary(connection, CONTROL_OUTPUT_ID)
            rename_parent_output(
                connection,
                EXECUTE_ACC_ID,
                EXECUTE_ACC_OUTPUT_ID,
                update_description=False,
            )
            rename_output_boundary(connection, EXECUTE_ACC_OUTPUT_ID)
            validate_database(connection, channel_before, connections_before)
    finally:
        connection.close()


def apply_project(project: Path) -> Path:
    data_dir = project / "data"
    databases = [
        data_dir / "cbdes.db",
        data_dir / "temp.db",
        data_dir / "runtimeCanvas.db",
    ]
    for database in databases:
        if not database.is_file():
            raise RuntimeError(f"missing canvas database: {database}")
    if sha256(databases[0]) != sha256(databases[1]):
        raise RuntimeError("cbdes.db and temp.db differ before migration")

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup_dir = (
        REPO_ROOT
        / "backups"
        / f"newaccpro4_before_acceleration_output_rename_{timestamp}"
    )
    backup_dir.mkdir(parents=True, exist_ok=False)
    for database in databases:
        shutil.copy2(database, backup_dir / database.name)

    staged_paths: dict[Path, Path] = {}
    with tempfile.TemporaryDirectory(prefix="newaccpro4-output-rename-") as directory:
        stage_dir = Path(directory)
        for database in databases:
            staged = stage_dir / database.name
            shutil.copy2(database, staged)
            migrate_database(staged)
            staged_paths[database] = staged
        for database, staged in staged_paths.items():
            shutil.copy2(staged, database)

    if sha256(databases[0]) != sha256(databases[1]):
        raise RuntimeError("cbdes.db and temp.db differ after migration")
    return backup_dir


def check_project(project: Path) -> None:
    for name in ("cbdes.db", "temp.db", "runtimeCanvas.db"):
        database = project / "data" / name
        connection = sqlite3.connect(database)
        connection.row_factory = sqlite3.Row
        try:
            channel = tuple(load_component(connection, CHANNEL_OUTPUT_ID))
            connections = [
                tuple(row)
                for row in connection.execute(
                    "SELECT * FROM project_connection ORDER BY id"
                )
            ]
            validate_database(connection, channel, connections)
        finally:
            connection.close()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--project", type=Path, default=DEFAULT_PROJECT)
    parser.add_argument("--apply", action="store_true")
    args = parser.parse_args()

    if args.apply:
        backup_dir = apply_project(args.project.resolve())
        print(f"backup={backup_dir}")
    check_project(args.project.resolve())
    for database_name in ("cbdes.db", "temp.db", "runtimeCanvas.db"):
        database = args.project.resolve() / "data" / database_name
        print(f"{database_name} sha256={sha256(database)}")
    print("ACC internal acceleration-output naming is consistent")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
