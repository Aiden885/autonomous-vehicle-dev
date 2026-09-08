#!/usr/bin/env python3
"""Add a provisional acceleration-command subsystem to newaccpro4.

The public ACC output is intentionally kept as ``targetSpeedMps`` because the
current GAASD channel has no acceleration field.  The value carried by that
port becomes acceleration in m/s^2 after this migration; it must not be used by
the current speed-command bridge without a matching interface update.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import shutil
import sqlite3
import sys
import uuid
from datetime import datetime
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
PROJECT_DIR = REPO_ROOT / "project" / "newaccpro4"
DATA_DIR = PROJECT_DIR / "data"
PRIMARY_DB = DATA_DIR / "cbdes.db"
TEMP_DB = DATA_DIR / "temp.db"

CONTROL_ID = "dd25dc36-745c-471a-8c2b-515194ab1e3f"
GENERATE_TARGET_SPEED_ID = "51f83135-f79a-42dc-8c89-9463090201b8"
GENERATE_TARGET_SPEED_OUTPUT_PORT = "64b07994-e9a6-5776-ad7f-70eae035483a"
CONTROL_EGO_SPEED_ID = "563b9aa3-6d87-4ce5-9d41-d1bde5fdf85d"
CONTROL_ENABLE_ID = "47b92561-30a7-4dc4-87db-8997cf7d27eb"
CONTROL_OUTPUT_ID = "01c10082-d84d-4e63-926d-0ddbce4b0ff2"
OLD_DIRECT_OUTPUT_EDGE_ID = "d5adb35d-19c9-4e6d-924e-056f3516d4d4"

COMPOSITE_TEMPLATE_ID = GENERATE_TARGET_SPEED_ID
INPUT_TEMPLATE_ID = "759b142e-6bfc-441e-98c6-904dd4487f8a"
OUTPUT_TEMPLATE_ID = "6a17bf88-7f6c-45d9-b733-1284fe6fd698"
SUBTRACT_TEMPLATE_ID = "a4222d8d-2a56-4242-ac3e-3d110da7bde1"
MULTIPLY_TEMPLATE_ID = "2f5978ae-f52b-45d0-9ffb-4978374ed9b0"
CONSTANT_TEMPLATE_ID = "d765cc32-6f4c-4784-8ac3-3ffefb40356b"
FMAX_TEMPLATE_ID = "9b2909db-9eb1-4f3b-9b9f-8997b3aab7dc"
FMIN_TEMPLATE_ID = "c8f57ad2-86d6-44bf-a4b5-cab26455d08b"

NAMESPACE = uuid.UUID("a3315ce9-7548-47ed-a907-4f8e8688238a")


def stable_id(label: str) -> str:
    return str(uuid.uuid5(NAMESPACE, label))


IDS = {
    "module": stable_id("newaccpro4:acceleration-command:component"),
    "moduleInstance": stable_id("newaccpro4:acceleration-command:instance"),
    "targetInput": stable_id("newaccpro4:acceleration-command:target-input"),
    "targetInputInstance": stable_id("newaccpro4:acceleration-command:target-input-instance"),
    "egoInput": stable_id("newaccpro4:acceleration-command:ego-input"),
    "egoInputInstance": stable_id("newaccpro4:acceleration-command:ego-input-instance"),
    "enableInput": stable_id("newaccpro4:acceleration-command:enable-input"),
    "enableInputInstance": stable_id("newaccpro4:acceleration-command:enable-input-instance"),
    "output": stable_id("newaccpro4:acceleration-command:output"),
    "outputInstance": stable_id("newaccpro4:acceleration-command:output-instance"),
    "speedError": stable_id("newaccpro4:acceleration-command:speed-error"),
    "speedErrorInstance": stable_id("newaccpro4:acceleration-command:speed-error-instance"),
    "gain": stable_id("newaccpro4:acceleration-command:gain"),
    "gainInstance": stable_id("newaccpro4:acceleration-command:gain-instance"),
    "rawAcceleration": stable_id("newaccpro4:acceleration-command:raw-acceleration"),
    "rawAccelerationInstance": stable_id("newaccpro4:acceleration-command:raw-acceleration-instance"),
    "minAcceleration": stable_id("newaccpro4:acceleration-command:min-acceleration"),
    "minAccelerationInstance": stable_id("newaccpro4:acceleration-command:min-acceleration-instance"),
    "lowerLimit": stable_id("newaccpro4:acceleration-command:lower-limit"),
    "lowerLimitInstance": stable_id("newaccpro4:acceleration-command:lower-limit-instance"),
    "maxAcceleration": stable_id("newaccpro4:acceleration-command:max-acceleration"),
    "maxAccelerationInstance": stable_id("newaccpro4:acceleration-command:max-acceleration-instance"),
    "upperLimit": stable_id("newaccpro4:acceleration-command:upper-limit"),
    "upperLimitInstance": stable_id("newaccpro4:acceleration-command:upper-limit-instance"),
    "enableGate": stable_id("newaccpro4:acceleration-command:enable-gate"),
    "enableGateInstance": stable_id("newaccpro4:acceleration-command:enable-gate-instance"),
    "targetPort": stable_id("newaccpro4:acceleration-command:port:target-speed"),
    "egoPort": stable_id("newaccpro4:acceleration-command:port:ego-speed"),
    "enablePort": stable_id("newaccpro4:acceleration-command:port:enable"),
    "accelerationPort": stable_id("newaccpro4:acceleration-command:port:acceleration"),
}


def edge_id(label: str) -> str:
    return stable_id(f"newaccpro4:acceleration-command:edge:{label}")


def json_text(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, separators=(",", ":"))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def row_dict(row: sqlite3.Row) -> dict[str, Any]:
    return {column: row[column] for column in row.keys()}


def load_component(connection: sqlite3.Connection, component_id: str) -> dict[str, Any]:
    row = connection.execute(
        "SELECT * FROM project_component WHERE id=?", (component_id,)
    ).fetchone()
    if row is None:
        raise RuntimeError(f"missing component template or boundary: {component_id}")
    return row_dict(row)


def insert_row(connection: sqlite3.Connection, table: str, values: dict[str, Any]) -> None:
    columns = list(values)
    placeholders = ",".join("?" for _ in columns)
    connection.execute(
        f"INSERT INTO {table} ({','.join(columns)}) VALUES ({placeholders})",
        [values[column] for column in columns],
    )


def layout(parent_id: str, x: float, y: float, width: float, height: float) -> str:
    return json_text(
        {
            parent_id: {
                "size": {"width": width, "height": height},
                "position": {"x": x, "y": y},
                "zIndex": 30,
            }
        }
    )


def port_props(name: str, data_type: str) -> list[dict[str, Any]]:
    return [
        {"key": "isReturnFlag", "value": False, "props": []},
        {"key": "name", "value": name, "props": []},
        {"key": "category", "value": 0, "props": []},
        {"key": "dataType", "value": data_type, "props": []},
        {"key": "shape", "value": [1], "props": []},
    ]


def composite_port(port_id: str, name: str, data_type: str, direction: str) -> dict[str, Any]:
    return {
        "id": port_id,
        "name": name,
        "direction": direction,
        "description": "函数输入端口" if direction == "In" else "函数输出端口",
        "props": port_props(name, data_type),
        "alias": name,
    }


def make_composite(template: dict[str, Any]) -> dict[str, Any]:
    row = copy.deepcopy(template)
    description = (
        "根据目标速度与自车速度误差计算纵向加速度命令，执行加减速度限幅和控制使能门控。"
    )
    row.update(
        {
            "id": IDS["module"],
            "parentId": CONTROL_ID,
            "instanceId": IDS["moduleInstance"],
            "name": "CalculateAccelerationCommand",
            "alias": "计算加速度命令",
            "description": description,
            "childComponents": "[]",
            "childConnections": "[]",
            "attribute": layout(CONTROL_ID, 650.0, -82.4, 328.0, 166.0),
        }
    )
    properties = json.loads(row["properties"])
    properties.update(
        {
            "name": "run",
            "cppClass": "run",
            "alias": "计算加速度命令",
            "description": description,
            "children": [],
            "parent": "",
        }
    )
    row["properties"] = json_text(properties)
    row["extensionProps"] = json_text(
        [
            {
                "key": "base_config",
                "value": [
                    {
                        "isCallback": False,
                        "param": [],
                        "state": [],
                        "instanceName": "calculateAccelerationCommand_1",
                    }
                ],
                "props": [],
            },
            {
                "key": "function_input_ports",
                "value": [
                    composite_port(IDS["targetPort"], "targetSpeedMps", "double", "In"),
                    composite_port(IDS["egoPort"], "egoSpeedMps", "double", "In"),
                    composite_port(IDS["enablePort"], "isControlEnabled", "int", "In"),
                ],
            },
            {
                "key": "function_output_ports",
                "value": [
                    composite_port(
                        IDS["accelerationPort"],
                        "accelerationCommandMps2",
                        "double",
                        "Out",
                    )
                ],
            },
        ]
    )
    return row


def make_boundary(
    template: dict[str, Any],
    component_id: str,
    instance_id: str,
    name: str,
    data_type: str,
    x: float,
    y: float,
    is_input: bool,
    width: float,
) -> dict[str, Any]:
    row = copy.deepcopy(template)
    row.update(
        {
            "id": component_id,
            "parentId": IDS["module"],
            "instanceId": instance_id,
            "alias": name,
            "childComponents": "[]",
            "childConnections": "[]",
            "attribute": layout(IDS["module"], x, y, width, 68.0),
        }
    )
    base = {
        "isReturnFlag": False,
        "name": name,
        "category": 0,
        "dataType": data_type,
        "shape": [1],
    }
    if is_input:
        output_port = {
            "name": "out",
            "dataType": data_type,
            "description": "参数输出",
            "alias": "out",
            "direction": "Out",
            "id": "out",
            "props": port_props(name, data_type),
        }
        extension = [
            {"key": "base_config", "value": [base]},
            {"key": "function_input_ports", "value": []},
            {"key": "function_output_ports", "value": [output_port]},
        ]
    else:
        input_port = {
            "name": "in",
            "dataType": data_type,
            "description": "参数输入",
            "alias": "in",
            "direction": "In",
            "id": "in",
            "props": port_props(name, data_type),
        }
        extension = [
            {"key": "base_config", "value": [base]},
            {"key": "function_input_ports", "value": [input_port]},
            {"key": "function_output_ports", "value": []},
        ]
    row["extensionProps"] = json_text(extension)
    return row


def make_atomic(
    template: dict[str, Any],
    component_id: str,
    instance_id: str,
    alias: str,
    x: float,
    y: float,
    width: float,
    height: float,
) -> dict[str, Any]:
    row = copy.deepcopy(template)
    row.update(
        {
            "id": component_id,
            "parentId": IDS["module"],
            "instanceId": instance_id,
            "alias": alias,
            "childComponents": "[]",
            "childConnections": "[]",
            "attribute": layout(IDS["module"], x, y, width, height),
        }
    )
    return row


def make_constant(
    template: dict[str, Any],
    component_id: str,
    instance_id: str,
    alias: str,
    value: float,
    x: float,
    y: float,
) -> dict[str, Any]:
    row = make_atomic(template, component_id, instance_id, alias, x, y, 190.0, 68.0)
    extension = json.loads(row["extensionProps"])
    extension[0]["value"] = [{"name": alias, "dataType": "double", "dataValue": value}]
    output = extension[2]["value"][0]
    output.update({"name": alias, "alias": alias, "dataType": "", "id": "out"})
    output["props"] = [
        {"key": "name", "value": alias, "props": []},
        {"key": "dataType", "value": "double", "props": []},
        {"key": "dataValue", "value": value, "props": []},
    ]
    row["extensionProps"] = json_text(extension)
    return row


def make_connection(
    parent_id: str,
    label: str,
    source_cell: str,
    source_port: str,
    target_cell: str,
    target_port: str,
) -> dict[str, Any]:
    return {
        "id": edge_id(label),
        "parentId": parent_id,
        "source": json_text({"cell": source_cell, "port": source_port}),
        "target": json_text({"cell": target_cell, "port": target_port}),
        "attribute": json_text({"from": "magnet"}),
        "extensionProps": json_text(
            [
                {
                    "key": "base_config",
                    "value": [],
                    "props": [{"key": "connectionType", "value": 0, "props": []}],
                }
            ]
        ),
    }


def rebuild_parent_cache(connection: sqlite3.Connection, parent_id: str) -> None:
    children = list(
        connection.execute(
            "SELECT id,instanceId FROM project_component WHERE parentId=? ORDER BY rowid",
            (parent_id,),
        )
    )
    instance_by_component = {row["id"]: row["instanceId"] for row in children}
    child_connections: list[dict[str, Any]] = []
    for row in connection.execute(
        "SELECT source,target,attribute,extensionProps FROM project_connection "
        "WHERE parentId=? ORDER BY rowid",
        (parent_id,),
    ):
        source = json.loads(row["source"])
        target = json.loads(row["target"])
        source_instance = instance_by_component.get(source["cell"])
        target_instance = instance_by_component.get(target["cell"])
        if not source_instance or not target_instance:
            raise RuntimeError(f"connection under {parent_id} references a non-child component")
        child_connections.append(
            {
                "fromInstanceId": source_instance,
                "fromInstancePortId": source["port"],
                "toInstanceId": target_instance,
                "toInstancePortId": target["port"],
                "extensionProps": json.loads(row["extensionProps"] or "[]"),
                "props": json.loads(row["attribute"] or "{}"),
            }
        )
    connection.execute(
        "UPDATE project_component SET childComponents=?,childConnections=? WHERE id=?",
        (
            json_text([row["instanceId"] for row in children]),
            json_text(child_connections),
            parent_id,
        ),
    )


def expected_internal_connections() -> list[dict[str, Any]]:
    parent = IDS["module"]
    return [
        make_connection(parent, "target-to-error", IDS["targetInput"], "out", IDS["speedError"], "subtract_input_a_In"),
        make_connection(parent, "ego-to-error", IDS["egoInput"], "out", IDS["speedError"], "subtract_input_b_In"),
        make_connection(parent, "error-to-raw", IDS["speedError"], "subtract_output_result_Out", IDS["rawAcceleration"], "multiply_input_a_In"),
        make_connection(parent, "gain-to-raw", IDS["gain"], "out", IDS["rawAcceleration"], "multiply_input_b_In"),
        make_connection(parent, "raw-to-lower-limit", IDS["rawAcceleration"], "multiply_output_result_Out", IDS["lowerLimit"], "fmax_input_x_In"),
        make_connection(parent, "min-to-lower-limit", IDS["minAcceleration"], "out", IDS["lowerLimit"], "fmax_input_y_In"),
        make_connection(parent, "lower-to-upper-limit", IDS["lowerLimit"], "fmax_output_return_Out", IDS["upperLimit"], "fmin_input_x_In"),
        make_connection(parent, "max-to-upper-limit", IDS["maxAcceleration"], "out", IDS["upperLimit"], "fmin_input_y_In"),
        make_connection(parent, "limited-to-enable", IDS["upperLimit"], "fmin_output_return_Out", IDS["enableGate"], "multiply_input_a_In"),
        make_connection(parent, "enable-to-gate", IDS["enableInput"], "out", IDS["enableGate"], "multiply_input_b_In"),
        make_connection(parent, "gate-to-output", IDS["enableGate"], "multiply_output_result_Out", IDS["output"], "in"),
    ]


def expected_outer_connections() -> list[dict[str, Any]]:
    return [
        make_connection(CONTROL_ID, "outer-target-input", GENERATE_TARGET_SPEED_ID, GENERATE_TARGET_SPEED_OUTPUT_PORT, IDS["module"], IDS["targetPort"]),
        make_connection(CONTROL_ID, "outer-ego-input", CONTROL_EGO_SPEED_ID, "out", IDS["module"], IDS["egoPort"]),
        make_connection(CONTROL_ID, "outer-enable-input", CONTROL_ENABLE_ID, "out", IDS["module"], IDS["enablePort"]),
        make_connection(CONTROL_ID, "outer-acceleration-output", IDS["module"], IDS["accelerationPort"], CONTROL_OUTPUT_ID, "in"),
    ]


def apply_migration(db_path: Path) -> None:
    connection = sqlite3.connect(db_path)
    connection.row_factory = sqlite3.Row
    try:
        connection.execute("BEGIN IMMEDIATE")
        existing = connection.execute(
            "SELECT name FROM project_component WHERE id=?", (IDS["module"],)
        ).fetchone()
        if existing is not None:
            raise RuntimeError("acceleration command subsystem already exists")

        templates = {
            "composite": load_component(connection, COMPOSITE_TEMPLATE_ID),
            "input": load_component(connection, INPUT_TEMPLATE_ID),
            "output": load_component(connection, OUTPUT_TEMPLATE_ID),
            "subtract": load_component(connection, SUBTRACT_TEMPLATE_ID),
            "multiply": load_component(connection, MULTIPLY_TEMPLATE_ID),
            "constant": load_component(connection, CONSTANT_TEMPLATE_ID),
            "fmax": load_component(connection, FMAX_TEMPLATE_ID),
            "fmin": load_component(connection, FMIN_TEMPLATE_ID),
        }

        components = [
            make_composite(templates["composite"]),
            make_boundary(templates["input"], IDS["targetInput"], IDS["targetInputInstance"], "targetSpeedMps", "double", -900.0, -190.0, True, 210.0),
            make_boundary(templates["input"], IDS["egoInput"], IDS["egoInputInstance"], "egoSpeedMps", "double", -900.0, -40.0, True, 190.0),
            make_boundary(templates["input"], IDS["enableInput"], IDS["enableInputInstance"], "isControlEnabled", "int", -900.0, 260.0, True, 220.0),
            make_boundary(templates["output"], IDS["output"], IDS["outputInstance"], "accelerationCommandMps2", "double", 940.0, -70.0, False, 280.0),
            make_atomic(templates["subtract"], IDS["speedError"], IDS["speedErrorInstance"], "speedErrorMps", -610.0, -120.0, 210.0, 106.0),
            make_constant(templates["constant"], IDS["gain"], IDS["gainInstance"], "accelerationGainPerS", 0.6, -610.0, 90.0),
            make_atomic(templates["multiply"], IDS["rawAcceleration"], IDS["rawAccelerationInstance"], "rawAccelerationMps2", -330.0, -120.0, 230.0, 106.0),
            make_constant(templates["constant"], IDS["minAcceleration"], IDS["minAccelerationInstance"], "minAccelerationMps2", -6.0, -330.0, 90.0),
            make_atomic(templates["fmax"], IDS["lowerLimit"], IDS["lowerLimitInstance"], "applyMinimumAcceleration", -30.0, -128.0, 250.0, 122.0),
            make_constant(templates["constant"], IDS["maxAcceleration"], IDS["maxAccelerationInstance"], "maxAccelerationMps2", 1.5, -30.0, 90.0),
            make_atomic(templates["fmin"], IDS["upperLimit"], IDS["upperLimitInstance"], "applyMaximumAcceleration", 290.0, -128.0, 250.0, 122.0),
            make_atomic(templates["multiply"], IDS["enableGate"], IDS["enableGateInstance"], "applyControlEnable", 610.0, -120.0, 230.0, 106.0),
        ]
        for component in components:
            insert_row(connection, "project_component", component)

        deleted_edge = connection.execute(
            "DELETE FROM project_connection WHERE id=?", (OLD_DIRECT_OUTPUT_EDGE_ID,)
        )
        if deleted_edge.rowcount != 1:
            raise RuntimeError("original GenerateTargetSpeed-to-output edge was not found")

        for edge in expected_internal_connections() + expected_outer_connections():
            insert_row(connection, "project_connection", edge)

        output = load_component(connection, CONTROL_OUTPUT_ID)
        output_attribute = json.loads(output["attribute"] or "{}")
        output_layout = output_attribute.setdefault(CONTROL_ID, {})
        output_layout["position"] = {"x": 1080.0, "y": -34.0}
        output_layout.setdefault("size", {"width": 166.0, "height": 68.0})
        output_layout.setdefault("zIndex", 30)
        connection.execute(
            "UPDATE project_component SET attribute=? WHERE id=?",
            (json_text(output_attribute), CONTROL_OUTPUT_ID),
        )

        rebuild_parent_cache(connection, IDS["module"])
        rebuild_parent_cache(connection, CONTROL_ID)
        connection.commit()
    except Exception:
        connection.rollback()
        raise
    finally:
        connection.close()


def constant_value(connection: sqlite3.Connection, component_id: str) -> float:
    row = connection.execute(
        "SELECT extensionProps FROM project_component WHERE id=?", (component_id,)
    ).fetchone()
    if row is None:
        raise RuntimeError(f"missing constant component: {component_id}")
    extension = json.loads(row[0])
    return float(extension[0]["value"][0]["dataValue"])


def check_database(db_path: Path) -> list[str]:
    errors: list[str] = []
    connection = sqlite3.connect(db_path)
    connection.row_factory = sqlite3.Row
    try:
        integrity = connection.execute("PRAGMA integrity_check").fetchone()[0]
        if integrity != "ok":
            errors.append(f"SQLite integrity check failed: {integrity}")

        module = connection.execute(
            "SELECT * FROM project_component WHERE id=?", (IDS["module"],)
        ).fetchone()
        if module is None:
            errors.append("CalculateAccelerationCommand subsystem is missing")
            return errors
        if module["parentId"] != CONTROL_ID or module["name"] != "CalculateAccelerationCommand":
            errors.append("acceleration subsystem is attached to the wrong parent or has the wrong name")

        expected_aliases = {
            "targetSpeedMps",
            "egoSpeedMps",
            "isControlEnabled",
            "accelerationCommandMps2",
            "speedErrorMps",
            "accelerationGainPerS",
            "rawAccelerationMps2",
            "minAccelerationMps2",
            "applyMinimumAcceleration",
            "maxAccelerationMps2",
            "applyMaximumAcceleration",
            "applyControlEnable",
        }
        actual_aliases = {
            row[0]
            for row in connection.execute(
                "SELECT alias FROM project_component WHERE parentId=?", (IDS["module"],)
            )
        }
        if actual_aliases != expected_aliases:
            errors.append(
                f"unexpected subsystem children: missing={sorted(expected_aliases-actual_aliases)}, "
                f"extra={sorted(actual_aliases-expected_aliases)}"
            )

        expected_edges = expected_internal_connections() + expected_outer_connections()
        for edge in expected_edges:
            count = connection.execute(
                "SELECT COUNT(*) FROM project_connection WHERE id=? AND parentId=? "
                "AND source=? AND target=?",
                (edge["id"], edge["parentId"], edge["source"], edge["target"]),
            ).fetchone()[0]
            if count != 1:
                errors.append(f"missing or changed edge: {edge['id']}")

        old_edge_count = connection.execute(
            "SELECT COUNT(*) FROM project_connection WHERE id=?", (OLD_DIRECT_OUTPUT_EDGE_ID,)
        ).fetchone()[0]
        if old_edge_count:
            errors.append("old target-speed direct output edge still exists")

        constants = {
            IDS["gain"]: 0.6,
            IDS["minAcceleration"]: -6.0,
            IDS["maxAcceleration"]: 1.5,
        }
        for component_id, expected in constants.items():
            if constant_value(connection, component_id) != expected:
                errors.append(f"constant value mismatch: {component_id}")

        output = connection.execute(
            "SELECT alias,extensionProps FROM project_component WHERE id=?", (CONTROL_OUTPUT_ID,)
        ).fetchone()
        output_extension = json.loads(output["extensionProps"])
        output_name = output_extension[0]["value"][0]["name"]
        if output["alias"] != "targetSpeedMps" or output_name != "targetSpeedMps":
            errors.append("public output boundary was renamed; it must remain targetSpeedMps")

        orphan_components = connection.execute(
            "SELECT COUNT(*) FROM project_component child LEFT JOIN project_component parent "
            "ON parent.id=child.parentId WHERE child.parentId<>'' AND parent.id IS NULL "
            "AND child.parentId NOT IN (SELECT projectId FROM project_info)"
        ).fetchone()[0]
        orphan_edges = connection.execute(
            "SELECT COUNT(*) FROM project_connection edge "
            "LEFT JOIN project_component source ON source.id=json_extract(edge.source,'$.cell') "
            "LEFT JOIN project_component target ON target.id=json_extract(edge.target,'$.cell') "
            "WHERE source.id IS NULL OR target.id IS NULL"
        ).fetchone()[0]
        if orphan_components:
            errors.append(f"orphan components: {orphan_components}")
        if orphan_edges:
            errors.append(f"orphan connections: {orphan_edges}")
    finally:
        connection.close()
    return errors


def make_backup() -> tuple[Path, Path]:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    primary_backup = DATA_DIR / f"cbdes.db.before_acceleration_command_{timestamp}"
    temp_backup = DATA_DIR / f"temp.db.before_acceleration_command_{timestamp}"
    shutil.copy2(PRIMARY_DB, primary_backup)
    shutil.copy2(TEMP_DB, temp_backup)
    return primary_backup, temp_backup


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--apply", action="store_true", help="apply the canvas migration")
    parser.add_argument("--check", action="store_true", help="validate the migrated canvas")
    args = parser.parse_args()
    if args.apply == args.check:
        parser.error("choose exactly one of --apply or --check")

    if args.check:
        errors = check_database(PRIMARY_DB)
        if errors:
            for error in errors:
                print(f"[FAIL] {error}")
            return 1
        print("[PASS] acceleration command canvas structure is valid")
        return 0

    if sha256(PRIMARY_DB) != sha256(TEMP_DB):
        raise RuntimeError("cbdes.db and temp.db differ; refusing to choose one implicitly")
    primary_backup, temp_backup = make_backup()
    apply_migration(PRIMARY_DB)
    errors = check_database(PRIMARY_DB)
    if errors:
        shutil.copy2(primary_backup, PRIMARY_DB)
        shutil.copy2(temp_backup, TEMP_DB)
        raise RuntimeError("migration validation failed: " + "; ".join(errors))
    shutil.copy2(PRIMARY_DB, TEMP_DB)
    if sha256(PRIMARY_DB) != sha256(TEMP_DB):
        raise RuntimeError("failed to synchronize cbdes.db and temp.db")
    print(f"[PASS] updated {PRIMARY_DB}")
    print(f"[PASS] synchronized {TEMP_DB}")
    print(f"[BACKUP] {primary_backup}")
    print(f"[BACKUP] {temp_backup}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
