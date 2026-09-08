#!/usr/bin/env python3
"""Split one GAASD input-channel view into vehicle and perception views.

The two canvas components continue to subscribe to the same Pangu channel.  Only
their selected fields and outgoing canvas connections differ.  This preserves
the middleware protocol while making the physical input categories explicit.
"""

from __future__ import annotations

import argparse
import copy
import json
import sqlite3
import sys
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable


REPO_ROOT = Path(__file__).resolve().parents[1]


@dataclass(frozen=True)
class ProjectConfig:
    project_name: str
    module_id: str
    channel_name: str
    vehicle_fields: tuple[str, ...]
    perception_fields: tuple[str, ...]
    vehicle_description: str
    perception_description: str
    vehicle_position: tuple[float, float]
    perception_position: tuple[float, float]
    vehicle_size: tuple[float, float]
    perception_size: tuple[float, float]
    field_aliases: tuple[tuple[str, str], ...]

    @property
    def project_dir(self) -> Path:
        return REPO_ROOT / "project" / self.project_name

    @property
    def perception_id(self) -> str:
        key = f"gaasd:{self.project_name}:{self.channel_name}:perceptionInputChannel"
        return str(uuid.uuid5(uuid.NAMESPACE_URL, key))

    @property
    def display_aliases(self) -> dict[str, str]:
        return dict(self.field_aliases)


PROJECTS = {
    "newaccpro3": ProjectConfig(
        project_name="newaccpro3",
        module_id="d3fdb632-0c78-41ab-a837-3bc336dba2df",
        channel_name="acc_input",
        vehicle_fields=("ego_speed_mps", "command_type", "frame_id"),
        perception_fields=("lead_speed_mps", "lead_distance_m"),
        vehicle_description="车内信息输入：自车速度、驾驶指令和帧标识。",
        perception_description="车外感知输入：前车速度和前车距离。",
        vehicle_position=(100.0, 175.0),
        perception_position=(100.0, 345.0),
        vehicle_size=(250.0, 118.0),
        perception_size=(250.0, 94.0),
        field_aliases=(
            ("ego_speed_mps", "egoV"),
            ("lead_speed_mps", "leadV"),
            ("lead_distance_m", "distance"),
            ("command_type", "commandType"),
            ("frame_id", "frameId"),
        ),
    ),
    "lks2": ProjectConfig(
        project_name="lks2",
        module_id="f97294f2-fdd5-4fa2-b6b8-cbe134e3e0b1",
        channel_name="lks_input",
        vehicle_fields=("ego_speed_mps", "brake_pressed", "driver_steer_norm", "frame_id"),
        perception_fields=("c0_m", "c1", "c2_per_m", "c3_per_m2", "curvature_per_m"),
        vehicle_description="车内信息输入：自车速度、制动状态、驾驶员转向和帧标识。",
        perception_description="车外感知输入：车道线三次多项式系数和曲率。",
        vehicle_position=(70.0, 145.0),
        perception_position=(70.0, 340.0),
        vehicle_size=(270.0, 142.0),
        perception_size=(270.0, 190.0),
        field_aliases=(
            ("ego_speed_mps", "egoV"),
            ("c0_m", "c0"),
            ("c1", "c1"),
            ("c2_per_m", "c2"),
            ("c3_per_m2", "c3"),
            ("curvature_per_m", "curvature"),
            ("brake_pressed", "brakePressed"),
            ("driver_steer_norm", "driverSteerNorm"),
            ("frame_id", "frameId"),
        ),
    ),
}


def load_json(raw: str | None, default: Any) -> Any:
    if not raw:
        return copy.deepcopy(default)
    return json.loads(raw)


def dump_json(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, separators=(",", ":"))


def extension_index(extension: list[dict[str, Any]]) -> dict[str, dict[str, Any]]:
    return {entry.get("key", ""): entry for entry in extension}


def port_name(port: dict[str, Any]) -> str:
    return str(port.get("name") or port.get("alias") or "").lstrip(".")


def select_channel_fields(
    extension: list[dict[str, Any]],
    fields: Iterable[str],
    display_aliases: dict[str, str],
) -> list[dict[str, Any]]:
    selected = tuple(fields)
    selected_set = set(selected)
    result = copy.deepcopy(extension)
    indexed = extension_index(result)

    base_config = indexed.get("base_config")
    if not base_config or not base_config.get("value"):
        raise ValueError("channel component is missing base_config")
    base_config["value"][0]["operateKeys"] = [f".{name}" for name in selected]

    outputs = indexed.get("function_output_ports")
    if outputs is None:
        raise ValueError("channel component is missing function_output_ports")
    all_ports = outputs.get("value", [])
    selected_ports = [port for port in all_ports if port_name(port) in selected_set]
    missing = selected_set - {port_name(port) for port in selected_ports}
    if missing:
        raise ValueError(f"selected channel fields have no output ports: {sorted(missing)}")
    for port in selected_ports:
        name = port_name(port)
        port["alias"] = display_aliases.get(name, name)
    outputs["value"] = selected_ports
    return result


def update_properties(raw: str | None, alias: str, description: str) -> str:
    properties = load_json(raw, {})
    properties["alias"] = alias
    properties["description"] = description
    return dump_json(properties)


def update_attribute(
    raw: str | None,
    parent_id: str,
    position: tuple[float, float],
    size: tuple[float, float],
) -> str:
    attribute = load_json(raw, {})
    canvas = attribute.setdefault(parent_id, {})
    canvas["position"] = {"x": position[0], "y": position[1]}
    canvas["size"] = {"width": size[0], "height": size[1]}
    canvas.setdefault("zIndex", 30)
    return dump_json(attribute)


def row_dict(row: sqlite3.Row) -> dict[str, Any]:
    return {column: row[column] for column in row.keys()}


def insert_component(connection: sqlite3.Connection, values: dict[str, Any]) -> None:
    columns = list(values)
    placeholders = ",".join("?" for _ in columns)
    connection.execute(
        f"INSERT INTO project_component ({','.join(columns)}) VALUES ({placeholders})",
        [values[column] for column in columns],
    )


def find_channel_components(
    connection: sqlite3.Connection, config: ProjectConfig
) -> list[sqlite3.Row]:
    return list(
        connection.execute(
            """
            SELECT * FROM project_component
            WHERE parentId=? AND componentType='channel_input' AND name=?
            ORDER BY id
            """,
            (config.module_id, config.channel_name),
        )
    )


def output_port_map(extension: list[dict[str, Any]]) -> dict[str, str]:
    outputs = extension_index(extension)["function_output_ports"]["value"]
    return {str(port["id"]): port_name(port) for port in outputs}


def set_component_view(
    connection: sqlite3.Connection,
    component_id: str,
    alias: str,
    description: str,
    fields: tuple[str, ...],
    position: tuple[float, float],
    size: tuple[float, float],
    display_aliases: dict[str, str],
) -> None:
    row = connection.execute(
        "SELECT * FROM project_component WHERE id=?", (component_id,)
    ).fetchone()
    if row is None:
        raise ValueError(f"component not found: {component_id}")
    extension = select_channel_fields(
        load_json(row["extensionProps"], []), fields, display_aliases
    )
    connection.execute(
        """
        UPDATE project_component
        SET alias=?, description=?, properties=?, extensionProps=?, attribute=?
        WHERE id=?
        """,
        (
            alias,
            description,
            update_properties(row["properties"], alias, description),
            dump_json(extension),
            update_attribute(row["attribute"], row["parentId"], position, size),
            component_id,
        ),
    )


def split_database(db_path: Path, config: ProjectConfig) -> dict[str, Any]:
    connection = sqlite3.connect(db_path)
    connection.row_factory = sqlite3.Row
    try:
        before_components = connection.execute(
            "SELECT COUNT(*) FROM project_component"
        ).fetchone()[0]
        before_connections = connection.execute(
            "SELECT COUNT(*) FROM project_connection"
        ).fetchone()[0]

        connection.execute("BEGIN IMMEDIATE")
        channels = find_channel_components(connection, config)
        if not channels:
            raise ValueError(f"{config.channel_name} was not found in {db_path}")

        by_alias = {row["alias"]: row for row in channels}
        vehicle = by_alias.get("vehicleInputChannel")
        perception = by_alias.get("perceptionInputChannel")
        if vehicle is None:
            if len(channels) != 1:
                raise ValueError(
                    f"cannot identify original {config.channel_name}: aliases="
                    f"{[row['alias'] for row in channels]}"
                )
            vehicle = channels[0]

        master_extension = load_json(vehicle["extensionProps"], [])
        master_ports = output_port_map(master_extension)
        if perception is not None:
            master_ports.update(output_port_map(load_json(perception["extensionProps"], [])))

        if perception is None:
            existing = connection.execute(
                "SELECT alias,componentType FROM project_component WHERE id=?",
                (config.perception_id,),
            ).fetchone()
            if existing is not None:
                raise ValueError(
                    f"deterministic component id is already used by {dict(existing)}"
                )
            clone = row_dict(vehicle)
            clone["id"] = config.perception_id
            insert_component(connection, clone)
            perception_id = config.perception_id
        else:
            perception_id = perception["id"]

        set_component_view(
            connection,
            vehicle["id"],
            "vehicleInputChannel",
            config.vehicle_description,
            config.vehicle_fields,
            config.vehicle_position,
            config.vehicle_size,
            config.display_aliases,
        )
        set_component_view(
            connection,
            perception_id,
            "perceptionInputChannel",
            config.perception_description,
            config.perception_fields,
            config.perception_position,
            config.perception_size,
            config.display_aliases,
        )

        channel_ids = {vehicle["id"], perception_id}
        field_owner = {field: vehicle["id"] for field in config.vehicle_fields}
        field_owner.update({field: perception_id for field in config.perception_fields})
        all_fields = set(field_owner)
        routed_fields: set[str] = set()

        edges = list(
            connection.execute(
                "SELECT id,source FROM project_connection WHERE parentId=?",
                (config.module_id,),
            )
        )
        for edge in edges:
            source = load_json(edge["source"], {})
            if source.get("cell") not in channel_ids:
                continue
            field = master_ports.get(str(source.get("port")))
            if field not in all_fields:
                raise ValueError(
                    f"cannot classify channel edge {edge['id']} port={source.get('port')}"
                )
            source["cell"] = field_owner[field]
            connection.execute(
                "UPDATE project_connection SET source=? WHERE id=?",
                (dump_json(source), edge["id"]),
            )
            routed_fields.add(field)

        missing_routes = all_fields - routed_fields
        if missing_routes:
            raise ValueError(f"channel fields have no outgoing connection: {sorted(missing_routes)}")

        integrity = connection.execute("PRAGMA integrity_check").fetchone()[0]
        if integrity != "ok":
            raise ValueError(f"SQLite integrity check failed: {integrity}")
        connection.commit()

        after_components = connection.execute(
            "SELECT COUNT(*) FROM project_component"
        ).fetchone()[0]
        after_connections = connection.execute(
            "SELECT COUNT(*) FROM project_connection"
        ).fetchone()[0]
        return {
            "db": str(db_path),
            "vehicle_id": vehicle["id"],
            "perception_id": perception_id,
            "components": [before_components, after_components],
            "connections": [before_connections, after_connections],
            "routed_fields": sorted(routed_fields),
        }
    except Exception:
        connection.rollback()
        raise
    finally:
        connection.close()


def component_to_block_child(
    row: sqlite3.Row, existing: dict[str, Any] | None
) -> dict[str, Any]:
    child = copy.deepcopy(existing) if existing is not None else {}
    is_channel = str(row["componentType"]).startswith("channel_")
    if is_channel:
        child["name"] = f"{row['name']}_{row['id'].replace('-', '_')}"
        child["cppClass"] = row["cppClass"] or ""
    else:
        child.setdefault("name", f"{row['name']}_{row['id'].replace('-', '_')}")
        child.setdefault("cppClass", row["cppClass"] or child["name"])
    child.update(
        {
            "componentType": row["componentType"],
            "alias": row["alias"],
            "description": row["description"] or "",
            "author": row["author"] or "",
            "version": row["version"] or "",
            "instanceId": row["id"],
            "originId": row["originId"] or "",
            "isCustom": bool(row["isCustom"]),
            "customType": row["customType"] or 0,
            "isAtomic": bool(row["isAtomic"]),
            "isSourceCode": row["isSourceCode"] or 0,
            "children": [],
            "parent": "",
            "tags": child.get("tags", row["tags"] or ""),
            "parentId": row["parentId"],
            "extensionProps": load_json(row["extensionProps"], []),
            "childComponents": load_json(row["childComponents"], []),
            "childConnections": load_json(row["childConnections"], []),
        }
    )
    return child


def sync_block_graph(path: Path, db_path: Path, config: ProjectConfig) -> None:
    if not path.exists():
        return
    root = json.loads(path.read_text(encoding="utf-8"))
    connection = sqlite3.connect(db_path)
    connection.row_factory = sqlite3.Row
    try:
        module = connection.execute(
            "SELECT * FROM project_component WHERE id=?", (config.module_id,)
        ).fetchone()
        if module is None:
            raise ValueError(f"module not found while syncing {path}")
        rows = list(
            connection.execute(
                "SELECT * FROM project_component WHERE parentId=?",
                (config.module_id,),
            )
        )
        existing = {
            child.get("instanceId"): child for child in root.get("childComponents", [])
        }

        def order(row: sqlite3.Row) -> tuple[int, str]:
            if row["componentType"] == "composite-block":
                return (0, row["alias"])
            if row["componentType"] == "channel_input" and row["alias"] == "vehicleInputChannel":
                return (1, row["alias"])
            if row["componentType"] == "channel_input":
                return (2, row["alias"])
            if row["componentType"] == "channel_output":
                return (3, row["alias"])
            return (4, row["alias"])

        root["childComponents"] = [
            component_to_block_child(row, existing.get(row["id"]))
            for row in sorted(rows, key=order)
        ]
        root["childConnections"] = []
        for edge in connection.execute(
            "SELECT * FROM project_connection WHERE parentId=? ORDER BY id",
            (config.module_id,),
        ):
            source = load_json(edge["source"], {})
            target = load_json(edge["target"], {})
            root["childConnections"].append(
                {
                    "fromInstanceId": source.get("cell"),
                    "fromInstancePortId": source.get("port"),
                    "toInstanceId": target.get("cell"),
                    "toInstancePortId": target.get("port"),
                    "extensionProps": load_json(edge["extensionProps"], []),
                }
            )
        root["extensionProps"] = load_json(module["extensionProps"], root.get("extensionProps", []))
    finally:
        connection.close()
    path.write_text(json.dumps(root, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def block_paths(config: ProjectConfig) -> list[Path]:
    module_token = config.module_id.replace("-", "_")
    blocks = config.project_dir / "icvos" / "blocks"
    return [
        blocks / "module" / f"module_empty_{module_token}.json",
        blocks / "functions" / f"callback_{module_token}.json",
    ]


def verify_database(db_path: Path, config: ProjectConfig) -> dict[str, Any]:
    connection = sqlite3.connect(db_path)
    connection.row_factory = sqlite3.Row
    try:
        channels = find_channel_components(connection, config)
        aliases = {row["alias"]: row for row in channels}
        if set(aliases) != {"vehicleInputChannel", "perceptionInputChannel"}:
            raise ValueError(f"unexpected channel aliases in {db_path}: {sorted(aliases)}")

        expected = {
            "vehicleInputChannel": set(config.vehicle_fields),
            "perceptionInputChannel": set(config.perception_fields),
        }
        actual: dict[str, set[str]] = {}
        for alias, row in aliases.items():
            extension = extension_index(load_json(row["extensionProps"], []))
            actual[alias] = {
                port_name(port)
                for port in extension["function_output_ports"].get("value", [])
            }
        if actual != expected:
            raise ValueError(f"channel field split mismatch in {db_path}: {actual}")

        orphan_edges = connection.execute(
            """
            SELECT COUNT(*) FROM project_connection edge
            LEFT JOIN project_component source ON source.id=json_extract(edge.source,'$.cell')
            LEFT JOIN project_component target ON target.id=json_extract(edge.target,'$.cell')
            WHERE source.id IS NULL OR target.id IS NULL
            """
        ).fetchone()[0]
        if orphan_edges:
            raise ValueError(f"orphan edges detected in {db_path}: {orphan_edges}")
        return {
            "db": str(db_path),
            "aliases": sorted(aliases),
            "fields": {alias: sorted(fields) for alias, fields in actual.items()},
            "components": connection.execute(
                "SELECT COUNT(*) FROM project_component"
            ).fetchone()[0],
            "connections": connection.execute(
                "SELECT COUNT(*) FROM project_connection"
            ).fetchone()[0],
        }
    finally:
        connection.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "projects",
        nargs="*",
        choices=sorted(PROJECTS),
        help="projects to update; defaults to both",
    )
    parser.add_argument(
        "--project-root",
        type=Path,
        help="optional root containing project/<name>; used for copy-based tests",
    )
    args = parser.parse_args()

    global REPO_ROOT
    if args.project_root:
        REPO_ROOT = args.project_root.resolve()

    reports: list[dict[str, Any]] = []
    selected_projects = args.projects or list(PROJECTS)
    for name in selected_projects:
        config = PROJECTS[name]
        for db_name in ("cbdes.db", "temp.db"):
            reports.append(split_database(config.project_dir / "data" / db_name, config))
        for path in block_paths(config):
            sync_block_graph(path, config.project_dir / "data" / "cbdes.db", config)
        reports.append(verify_database(config.project_dir / "data" / "cbdes.db", config))
        reports.append(verify_database(config.project_dir / "data" / "temp.db", config))

    print(json.dumps(reports, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
