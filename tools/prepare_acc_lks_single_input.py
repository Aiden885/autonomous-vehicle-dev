#!/usr/bin/env python3
"""Prepare the latest ACC/LKS canvases as single-input-channel projects.

The script repairs the deleted ACC input component from the last known-good
canvas backup, keeps the official Pangu channel field names unchanged, and
simplifies only the internal LKS polynomial aliases to c0/c1/c2/c3.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import shutil
import sqlite3
import sys
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
ACC_PROJECT = REPO_ROOT / "project" / "newaccpro4"
LKS_PROJECT = REPO_ROOT / "project" / "lks2"
ACC_TEMPLATE_DB = (
    ACC_PROJECT / "data" / "cbdes.db.before_decision_control_alias_20260810_153356"
)

ACC_MODULE_ID = "b6d54e1b-2577-4492-bcea-dfe707509e43"
ACC_INPUT_ID = "aa5b0421-b7ad-4069-951b-edf3f87e8c79"
ACC_TEMPLATE_PERCEPTION_ID = "9b243614-4a31-4af7-99c2-95eed1fde68a"
ACC_FIELDS = (
    "ego_speed_mps",
    "lead_speed_mps",
    "lead_distance_m",
    "command_type",
    "frame_id",
)

LKS_INTERNAL_NAMES = {
    "laneOffsetM": "c0",
    "laneHeadingCoeff": "c1",
    "laneCurvatureCoeffPerM": "c2",
    "laneCurvatureRateCoeffPerM2": "c3",
}


def load_json(raw: str | None, default: Any) -> Any:
    if not raw:
        return copy.deepcopy(default)
    return json.loads(raw)


def dump_json(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, separators=(",", ":"))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def extension_section(extension: list[dict[str, Any]], key: str) -> dict[str, Any]:
    for section in extension:
        if section.get("key") == key:
            return section
    raise ValueError(f"missing extension section: {key}")


def port_name(port: dict[str, Any]) -> str:
    return str(port.get("name") or port.get("alias") or "").lstrip(".")


def row_dict(row: sqlite3.Row) -> dict[str, Any]:
    return {column: row[column] for column in row.keys()}


def insert_row(connection: sqlite3.Connection, table: str, values: dict[str, Any]) -> None:
    columns = list(values)
    placeholders = ",".join("?" for _ in columns)
    connection.execute(
        f"INSERT INTO {table} ({','.join(columns)}) VALUES ({placeholders})",
        [values[column] for column in columns],
    )


def compact_channel_port(port: dict[str, Any], field: str) -> dict[str, Any]:
    data_type = None
    for prop in port.get("props", []):
        if prop.get("key") in {"dataType", "datatype"}:
            data_type = prop.get("value")
            break
    if not data_type:
        raise ValueError(f"channel field has no data type: {field}")
    return {
        "alias": field,
        "description": "通道输入属性输出端口",
        "direction": "Out",
        "id": f".{field}",
        "name": field,
        "props": [{"key": "dataType", "value": data_type, "props": []}],
    }


def repair_acc_database(database: Path, template_database: Path) -> dict[str, Any]:
    connection = sqlite3.connect(database)
    connection.row_factory = sqlite3.Row
    template = sqlite3.connect(template_database)
    template.row_factory = sqlite3.Row
    try:
        connection.execute("BEGIN IMMEDIATE")
        current_inputs = list(
            connection.execute(
                """
                SELECT * FROM project_component
                WHERE parentId=? AND componentType='channel_input' AND name='acc_input'
                """,
                (ACC_MODULE_ID,),
            )
        )
        if len(current_inputs) > 1:
            raise ValueError(f"ACC has multiple input components: {len(current_inputs)}")

        vehicle = template.execute(
            "SELECT * FROM project_component WHERE id=?", (ACC_INPUT_ID,)
        ).fetchone()
        perception = template.execute(
            "SELECT * FROM project_component WHERE id=?",
            (ACC_TEMPLATE_PERCEPTION_ID,),
        ).fetchone()
        if vehicle is None or perception is None:
            raise ValueError("ACC input templates are missing")

        if current_inputs:
            input_id = current_inputs[0]["id"]
            input_row = current_inputs[0]
        else:
            if connection.execute(
                "SELECT 1 FROM project_component WHERE id=?", (ACC_INPUT_ID,)
            ).fetchone():
                raise ValueError(f"ACC input id is already occupied: {ACC_INPUT_ID}")
            insert_row(connection, "project_component", row_dict(vehicle))
            input_id = ACC_INPUT_ID
            input_row = connection.execute(
                "SELECT * FROM project_component WHERE id=?", (input_id,)
            ).fetchone()

        template_ports: dict[str, dict[str, Any]] = {}
        for source_row in (vehicle, perception):
            extension = load_json(source_row["extensionProps"], [])
            section = extension_section(extension, "function_output_ports")
            for port in section.get("value", []):
                template_ports[port_name(port)] = port
        missing_ports = set(ACC_FIELDS) - set(template_ports)
        if missing_ports:
            raise ValueError(f"ACC template fields are missing: {sorted(missing_ports)}")

        extension = load_json(input_row["extensionProps"], [])
        base = extension_section(extension, "base_config")["value"][0]
        base["operateKeys"] = [f".{field}" for field in ACC_FIELDS]
        extension_section(extension, "function_input_ports")["value"] = []
        extension_section(extension, "function_output_ports")["value"] = [
            compact_channel_port(template_ports[field], field) for field in ACC_FIELDS
        ]

        alias = "vehicleInputChannel"
        description = "ACC输入：自车速度、前车速度、前车距离、驾驶指令和帧标识。"
        properties = load_json(input_row["properties"], {})
        properties["alias"] = alias
        properties["description"] = description
        attribute = load_json(input_row["attribute"], {})
        canvas = attribute.setdefault(ACC_MODULE_ID, {})
        canvas["position"] = {"x": -546, "y": -76}
        canvas["size"] = {"width": 234, "height": 174}
        canvas.setdefault("zIndex", 30)
        connection.execute(
            """
            UPDATE project_component
            SET alias=?,description=?,properties=?,extensionProps=?,attribute=?
            WHERE id=?
            """,
            (
                alias,
                description,
                dump_json(properties),
                dump_json(extension),
                dump_json(attribute),
                input_id,
            ),
        )

        input_ids = [row["id"] for row in current_inputs]
        input_ids.append(input_id)
        for component_id in set(input_ids):
            connection.execute(
                """
                DELETE FROM project_connection
                WHERE parentId=? AND json_extract(source,'$.cell')=?
                """,
                (ACC_MODULE_ID, component_id),
            )

        template_edges = list(
            template.execute(
                """
                SELECT * FROM project_connection
                WHERE parentId=? AND json_extract(source,'$.cell') IN (?,?)
                ORDER BY id
                """,
                (ACC_MODULE_ID, ACC_INPUT_ID, ACC_TEMPLATE_PERCEPTION_ID),
            )
        )
        routed_fields: set[str] = set()
        for edge in template_edges:
            values = row_dict(edge)
            source = load_json(values["source"], {})
            source_port = str(source.get("port") or "")
            if source_port.startswith("."):
                field = source_port[1:]
            else:
                marker = "_output_"
                if marker not in source_port or not source_port.endswith("_Out"):
                    raise ValueError(f"cannot identify ACC source field: {source_port}")
                field = source_port.split(marker, 1)[1][:-4]
            if field not in ACC_FIELDS:
                raise ValueError(f"unexpected ACC source field: {field}")
            source["cell"] = input_id
            source["port"] = f".{field}"
            values["source"] = dump_json(source)
            connection.execute("DELETE FROM project_connection WHERE id=?", (values["id"],))
            target_cell = load_json(values["target"], {}).get("cell")
            if not connection.execute(
                "SELECT 1 FROM project_component WHERE id=?", (target_cell,)
            ).fetchone():
                raise ValueError(f"ACC edge target is missing: {target_cell}")
            insert_row(connection, "project_connection", values)
            routed_fields.add(field)
        if routed_fields != set(ACC_FIELDS):
            raise ValueError(f"ACC fields are not fully routed: {sorted(routed_fields)}")

        module = connection.execute(
            "SELECT extensionProps FROM project_component WHERE id=?", (ACC_MODULE_ID,)
        ).fetchone()
        if module is None:
            raise ValueError("ACC module component is missing")
        module_extension = load_json(module["extensionProps"], [])
        module_ports = extension_section(module_extension, "module_input_ports")["value"]
        matching_ports = [port for port in module_ports if port.get("name") == "acc_input"]
        if len(matching_ports) != 1:
            raise ValueError(f"unexpected ACC module input count: {len(matching_ports)}")
        operate_props = [
            prop for prop in matching_ports[0].get("props", []) if prop.get("key") == "operateKeys"
        ]
        if len(operate_props) != 1:
            raise ValueError("ACC module input has no unique operateKeys property")
        operate_props[0]["value"] = [f".{field}" for field in ACC_FIELDS]
        connection.execute(
            """
            UPDATE project_component
            SET childComponents='[]',childConnections='[]',extensionProps=?
            WHERE id=?
            """,
            (dump_json(module_extension), ACC_MODULE_ID),
        )
        connection.execute(
            "DELETE FROM channel_manage WHERE channel_name='acc_input_perception'"
        )

        integrity = connection.execute("PRAGMA integrity_check").fetchone()[0]
        if integrity != "ok":
            raise ValueError(f"ACC SQLite integrity check failed: {integrity}")
        connection.commit()
        return {"input_id": input_id, "routed_fields": sorted(routed_fields)}
    except Exception:
        connection.rollback()
        raise
    finally:
        template.close()
        connection.close()


def replace_exact_strings(value: Any, mapping: dict[str, str]) -> Any:
    if isinstance(value, str):
        return mapping.get(value, value)
    if isinstance(value, list):
        return [replace_exact_strings(item, mapping) for item in value]
    if isinstance(value, dict):
        return {
            key: replace_exact_strings(item, mapping) for key, item in value.items()
        }
    return value


def simplify_lks_database(database: Path) -> dict[str, Any]:
    connection = sqlite3.connect(database)
    connection.row_factory = sqlite3.Row
    try:
        connection.execute("BEGIN IMMEDIATE")
        before_channels = list(
            connection.execute(
                """
                SELECT id,name,alias,extensionProps FROM project_component
                WHERE componentType LIKE 'channel_%' ORDER BY id
                """
            )
        )
        changed = 0
        rows = list(
            connection.execute(
                """
                SELECT id,parentId,alias,componentType,properties,extensionProps
                FROM project_component ORDER BY id
                """
            )
        )
        proposed: dict[str, str] = {}
        for row in rows:
            if row["componentType"].startswith("channel_"):
                proposed[row["id"]] = row["alias"]
            else:
                proposed[row["id"]] = LKS_INTERNAL_NAMES.get(row["alias"], row["alias"])
        siblings: dict[str, set[str]] = {}
        for row in rows:
            aliases = siblings.setdefault(row["parentId"], set())
            alias = proposed[row["id"]]
            if alias in aliases:
                raise ValueError(f"LKS alias collision under {row['parentId']}: {alias}")
            aliases.add(alias)

        for row in rows:
            if row["componentType"].startswith("channel_"):
                continue
            alias = proposed[row["id"]]
            properties = replace_exact_strings(
                load_json(row["properties"], {}), LKS_INTERNAL_NAMES
            )
            properties["alias"] = alias
            extension = replace_exact_strings(
                load_json(row["extensionProps"], []), LKS_INTERNAL_NAMES
            )
            connection.execute(
                """
                UPDATE project_component SET alias=?,properties=?,extensionProps=? WHERE id=?
                """,
                (alias, dump_json(properties), dump_json(extension), row["id"]),
            )
            if alias != row["alias"] or properties != load_json(row["properties"], {}) or extension != load_json(row["extensionProps"], []):
                changed += 1

        after_channels = list(
            connection.execute(
                """
                SELECT id,name,alias,extensionProps FROM project_component
                WHERE componentType LIKE 'channel_%' ORDER BY id
                """
            )
        )
        if [tuple(row) for row in before_channels] != [tuple(row) for row in after_channels]:
            raise ValueError("LKS external channel contract changed")
        integrity = connection.execute("PRAGMA integrity_check").fetchone()[0]
        if integrity != "ok":
            raise ValueError(f"LKS SQLite integrity check failed: {integrity}")
        connection.commit()
        return {"changed_components": changed}
    except Exception:
        connection.rollback()
        raise
    finally:
        connection.close()


def channel_fields(connection: sqlite3.Connection, name: str) -> list[str]:
    row = connection.execute(
        """
        SELECT extensionProps FROM project_component
        WHERE componentType='channel_input' AND name=?
        """,
        (name,),
    ).fetchone()
    if row is None:
        raise ValueError(f"input channel is missing: {name}")
    extension = load_json(row[0], [])
    ports = extension_section(extension, "function_output_ports").get("value", [])
    return [port_name(port) for port in ports]


def verify_project(project: Path, channel_name: str, expected_fields: tuple[str, ...]) -> dict[str, Any]:
    database = project / "data" / "cbdes.db"
    connection = sqlite3.connect(database)
    try:
        input_count = connection.execute(
            """
            SELECT COUNT(*) FROM project_component
            WHERE componentType='channel_input' AND name=?
            """,
            (channel_name,),
        ).fetchone()[0]
        if input_count != 1:
            raise ValueError(f"{project.name} input channel count is {input_count}")
        fields = channel_fields(connection, channel_name)
        if fields != list(expected_fields):
            raise ValueError(f"{project.name} input fields mismatch: {fields}")
        dangling = connection.execute(
            """
            SELECT COUNT(*) FROM project_connection edge
            LEFT JOIN project_component source
              ON source.id=json_extract(edge.source,'$.cell')
            LEFT JOIN project_component target
              ON target.id=json_extract(edge.target,'$.cell')
            WHERE source.id IS NULL OR target.id IS NULL
            """
        ).fetchone()[0]
        if dangling:
            raise ValueError(f"{project.name} has {dangling} dangling connections")
        integrity = connection.execute("PRAGMA integrity_check").fetchone()[0]
        if integrity != "ok":
            raise ValueError(f"{project.name} integrity check failed: {integrity}")
        return {
            "project": project.name,
            "input_components": input_count,
            "fields": fields,
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
    global REPO_ROOT, ACC_PROJECT, LKS_PROJECT, ACC_TEMPLATE_DB

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--repo-root", type=Path, default=REPO_ROOT, help="repository root"
    )
    args = parser.parse_args()

    REPO_ROOT = args.repo_root.resolve()
    ACC_PROJECT = REPO_ROOT / "project" / "newaccpro4"
    LKS_PROJECT = REPO_ROOT / "project" / "lks2"
    ACC_TEMPLATE_DB = (
        ACC_PROJECT / "data" / "cbdes.db.before_decision_control_alias_20260810_153356"
    )
    if not ACC_TEMPLATE_DB.is_file():
        raise FileNotFoundError(ACC_TEMPLATE_DB)

    for project in (ACC_PROJECT, LKS_PROJECT):
        cbdes = project / "data" / "cbdes.db"
        temp = project / "data" / "temp.db"
        if sha256(cbdes) != sha256(temp):
            raise ValueError(f"database pair differs before update: {project}")

    reports = [repair_acc_database(ACC_PROJECT / "data" / "cbdes.db", ACC_TEMPLATE_DB)]
    shutil.copy2(ACC_PROJECT / "data" / "cbdes.db", ACC_PROJECT / "data" / "temp.db")

    reports.append(simplify_lks_database(LKS_PROJECT / "data" / "cbdes.db"))
    shutil.copy2(LKS_PROJECT / "data" / "cbdes.db", LKS_PROJECT / "data" / "temp.db")

    sys.path.insert(0, str(REPO_ROOT / "tools"))
    import split_acc_lks_input_channels as channel_tools

    channel_tools.REPO_ROOT = REPO_ROOT
    lks_config = channel_tools.PROJECTS["lks2"]
    for path in channel_tools.block_paths(lks_config):
        channel_tools.sync_block_graph(
            path, LKS_PROJECT / "data" / "cbdes.db", lks_config
        )

    reports.append(verify_project(ACC_PROJECT, "acc_input", ACC_FIELDS))
    reports.append(
        verify_project(
            LKS_PROJECT,
            "lks_input",
            (
                "ego_speed_mps",
                "brake_pressed",
                "driver_steer_norm",
                "frame_id",
                "c0_m",
                "c1",
                "c2_per_m",
                "c3_per_m2",
                "curvature_per_m",
            ),
        )
    )
    for project in (ACC_PROJECT, LKS_PROJECT):
        cbdes = project / "data" / "cbdes.db"
        temp = project / "data" / "temp.db"
        if sha256(cbdes) != sha256(temp):
            raise ValueError(f"database pair differs after update: {project}")

    print(json.dumps(reports, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
