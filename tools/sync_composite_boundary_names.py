#!/usr/bin/env python3
"""Synchronize composite child boundary names with their parent ports.

The GAASD canvas stores a composite port definition on the parent component and
an input/output boundary component inside the child canvas.  Naming migrations
must update both representations while preserving every UUID and connection.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import sqlite3
import tempfile
from collections import Counter
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

from normalize_acc_lks_naming_standard import ACC_SPEC, LKS_SPEC, NamingSpec


STRUCTURAL_TYPES = {"composite-block", "run"}
BOUNDARY_TYPES = {"input", "output"}
CHANNEL_TYPES = {"channel_input", "channel_output"}


@dataclass(frozen=True)
class BoundaryChange:
    parent_alias: str
    component_id: str
    component_type: str
    old_alias: str
    old_name: str
    new_name: str


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


def extension_value(extension_props: list[dict[str, Any]], key: str) -> Any:
    for item in extension_props:
        if item.get("key") == key:
            return item.get("value")
    return None


def base_config(extension_props: list[dict[str, Any]]) -> dict[str, Any]:
    values = extension_value(extension_props, "base_config") or []
    if values and isinstance(values[0], dict):
        return values[0]
    return {}


def port_name(port: dict[str, Any]) -> str:
    for prop in port.get("props", []) or []:
        if prop.get("key") == "name" and prop.get("value"):
            return str(prop["value"])
    return str(port.get("name") or port.get("alias") or "")


def port_type(port: dict[str, Any]) -> str:
    for prop in port.get("props", []) or []:
        if prop.get("key") == "dataType" and prop.get("value"):
            return str(prop["value"])
    return str(port.get("dataType") or "")


def canonical_name(name: str, spec: NamingSpec) -> str:
    return spec.signal_names.get(name, name)


def parent_ports(
    row: sqlite3.Row, direction: str
) -> list[tuple[str, str]]:
    extension_props = json_load(row["extensionProps"], [])
    key = "function_input_ports" if direction == "input" else "function_output_ports"
    return [
        (port_name(port), port_type(port))
        for port in extension_value(extension_props, key) or []
    ]


def child_boundary(
    row: sqlite3.Row, spec: NamingSpec
) -> tuple[str, str, str]:
    extension_props = json_load(row["extensionProps"], [])
    config = base_config(extension_props)
    raw_name = str(config.get("name") or row["alias"])
    return raw_name, canonical_name(raw_name, spec), str(config.get("dataType") or "")


def audit_database(
    connection: sqlite3.Connection, spec: NamingSpec
) -> tuple[list[BoundaryChange], list[str]]:
    changes: list[BoundaryChange] = []
    errors: list[str] = []

    parents = list(
        connection.execute(
            "SELECT * FROM project_component "
            "WHERE componentType IN ('composite-block','run') ORDER BY alias,id"
        )
    )
    for parent in parents:
        parent_inputs = parent_ports(parent, "input")
        parent_outputs = parent_ports(parent, "output")
        child_inputs: list[tuple[str, str]] = []
        child_outputs: list[tuple[str, str]] = []

        children = list(
            connection.execute(
                "SELECT * FROM project_component WHERE parentId=? "
                "AND componentType IN ('input','output') ORDER BY rowid",
                (parent["id"],),
            )
        )
        for child in children:
            raw_name, expected_name, data_type = child_boundary(child, spec)
            target = child_inputs if child["componentType"] == "input" else child_outputs
            target.append((expected_name, data_type))
            if raw_name != expected_name or child["alias"] != expected_name:
                changes.append(
                    BoundaryChange(
                        parent_alias=parent["alias"],
                        component_id=child["id"],
                        component_type=child["componentType"],
                        old_alias=child["alias"],
                        old_name=raw_name,
                        new_name=expected_name,
                    )
                )

        for direction, parent_values, child_values in (
            ("input", parent_inputs, child_inputs),
            ("output", parent_outputs, child_outputs),
        ):
            if Counter(parent_values) != Counter(child_values):
                errors.append(
                    f"{parent['alias']} {direction}: parent={parent_values}, "
                    f"canonical child={child_values}"
                )

        noncanonical_parent = [
            name
            for name, _ in parent_inputs + parent_outputs
            if canonical_name(name, spec) != name
        ]
        if noncanonical_parent:
            errors.append(
                f"{parent['alias']}: noncanonical parent ports={noncanonical_parent}"
            )

    return changes, errors


def capture_protected_state(connection: sqlite3.Connection) -> dict[str, Any]:
    component_columns = [
        row[1] for row in connection.execute("PRAGMA table_info(project_component)")
    ]
    protected_columns = [
        column
        for column in component_columns
        if column not in {"alias", "properties", "extensionProps"}
    ]
    protected_components = list(
        connection.execute(
            f"SELECT {','.join(protected_columns)} "
            "FROM project_component ORDER BY id"
        )
    )
    channels = list(
        connection.execute(
            "SELECT * FROM project_component "
            "WHERE componentType IN ('channel_input','channel_output') ORDER BY id"
        )
    )
    connections = list(
        connection.execute("SELECT * FROM project_connection ORDER BY id")
    )

    port_topology: list[tuple[str, str, str, str, str, str]] = []
    for row in connection.execute(
        "SELECT id,extensionProps FROM project_component ORDER BY id"
    ):
        extension_props = json_load(row["extensionProps"], [])
        for key in ("function_input_ports", "function_output_ports"):
            for port in extension_value(extension_props, key) or []:
                port_topology.append(
                    (
                        row["id"],
                        key,
                        str(port.get("id") or ""),
                        str(port.get("name") or ""),
                        str(port.get("alias") or ""),
                        str(port.get("direction") or ""),
                    )
                )

    return {
        "protected_components": protected_components,
        "channels": channels,
        "connections": connections,
        "port_topology": port_topology,
    }


def set_nested_port_name(
    extension_props: list[dict[str, Any]], component_type: str, new_name: str
) -> None:
    key = "function_output_ports" if component_type == "input" else "function_input_ports"
    ports = extension_value(extension_props, key) or []
    if len(ports) != 1:
        raise RuntimeError(
            f"Boundary component must have exactly one {key} entry, got {len(ports)}"
        )
    props = ports[0].setdefault("props", [])
    name_props = [prop for prop in props if prop.get("key") == "name"]
    if name_props:
        name_props[0]["value"] = new_name
    else:
        props.append({"key": "name", "value": new_name, "props": []})


def apply_changes(
    connection: sqlite3.Connection,
    spec: NamingSpec,
    changes: list[BoundaryChange],
) -> None:
    protected_before = capture_protected_state(connection)

    for change in changes:
        row = connection.execute(
            "SELECT * FROM project_component WHERE id=?", (change.component_id,)
        ).fetchone()
        if row is None:
            raise RuntimeError(f"Missing component: {change.component_id}")
        if row["componentType"] not in BOUNDARY_TYPES:
            raise RuntimeError(f"Not a boundary component: {change.component_id}")

        properties = json_load(row["properties"], {})
        extension_props = json_load(row["extensionProps"], [])
        config = base_config(extension_props)
        config["name"] = change.new_name
        properties["alias"] = change.new_name
        set_nested_port_name(extension_props, row["componentType"], change.new_name)

        connection.execute(
            "UPDATE project_component SET alias=?,properties=?,extensionProps=? WHERE id=?",
            (
                change.new_name,
                json_text(properties),
                json_text(extension_props),
                change.component_id,
            ),
        )

    duplicate = connection.execute(
        "SELECT parentId,alias,COUNT(*) FROM project_component "
        "GROUP BY parentId,alias HAVING COUNT(*)>1"
    ).fetchall()
    if duplicate:
        raise RuntimeError(f"Sibling alias collision after update: {duplicate}")

    remaining, errors = audit_database(connection, spec)
    if errors:
        raise RuntimeError("Boundary audit failed after update:\n" + "\n".join(errors))
    if remaining:
        raise RuntimeError(f"Boundary names still require synchronization: {remaining}")

    protected_after = capture_protected_state(connection)
    if protected_before != protected_after:
        raise RuntimeError("Protected UUID, wiring, channel, or component data changed")

    integrity = connection.execute("PRAGMA integrity_check").fetchone()[0]
    if integrity != "ok":
        raise RuntimeError(f"SQLite integrity check failed: {integrity}")


def update_project(project: Path, spec: NamingSpec, apply: bool) -> dict[str, Any]:
    cbdes = project / "data" / "cbdes.db"
    temp = project / "data" / "temp.db"
    if not cbdes.is_file() or not temp.is_file():
        raise RuntimeError(f"Missing cbdes.db or temp.db under {project}")
    cbdes_hash = sha256(cbdes)
    temp_hash = sha256(temp)

    connection = sqlite3.connect(cbdes)
    connection.row_factory = sqlite3.Row
    try:
        changes, errors = audit_database(connection, spec)
    finally:
        connection.close()
    if errors:
        raise RuntimeError("Boundary audit failed before update:\n" + "\n".join(errors))

    report: dict[str, Any] = {
        "project": str(project),
        "mode": "apply" if apply else "check",
        "databaseCopiesMatch": cbdes_hash == temp_hash,
        "changes": [change.__dict__ for change in changes],
    }
    if not apply or not changes:
        report["sha256"] = cbdes_hash
        return report

    if cbdes_hash != temp_hash:
        raise RuntimeError(f"cbdes.db and temp.db differ before update: {project}")

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    cbdes_backup = cbdes.with_name(f"cbdes.db.before_boundary_sync_{timestamp}")
    temp_backup = temp.with_name(f"temp.db.before_boundary_sync_{timestamp}")
    shutil.copy2(cbdes, cbdes_backup)
    shutil.copy2(temp, temp_backup)

    with tempfile.TemporaryDirectory(prefix="gaasd-boundary-sync-") as directory:
        staged = Path(directory) / "cbdes.db"
        shutil.copy2(cbdes, staged)
        staged_connection = sqlite3.connect(staged)
        staged_connection.row_factory = sqlite3.Row
        try:
            with staged_connection:
                apply_changes(staged_connection, spec, changes)
        finally:
            staged_connection.close()
        shutil.copy2(staged, cbdes)
        shutil.copy2(staged, temp)

    if sha256(cbdes) != sha256(temp):
        raise RuntimeError("cbdes.db and temp.db differ after update")
    report.update(
        {
            "backup": [str(cbdes_backup), str(temp_backup)],
            "sha256": sha256(cbdes),
        }
    )
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "projects",
        nargs="*",
        type=Path,
        default=[Path("project/newaccpro4"), Path("project/lks2")],
    )
    parser.add_argument("--apply", action="store_true")
    args = parser.parse_args()

    reports = []
    for project in args.projects:
        resolved = project.resolve()
        if resolved.name == "newaccpro4":
            spec = ACC_SPEC
        elif resolved.name == "lks2":
            spec = LKS_SPEC
        else:
            raise RuntimeError(f"Unsupported project: {resolved}")
        reports.append(update_project(resolved, spec, args.apply))

    print(json.dumps(reports, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
