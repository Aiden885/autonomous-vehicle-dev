#!/usr/bin/env python3
"""Refactor the newaccpro3 Decision canvas without changing its behavior."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import shutil
import sqlite3
import sys
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any


DECISION_ID = "ea2170e7-297c-4433-b70a-9c66e48988ad"
CONTROL_TEMPLATE_ID = "98d0f783-5cfa-413d-815a-dcd775b5dd13"
INPUT_TEMPLATE_ID = "eb1e8ad8-1ad9-4ebb-85ed-099f8e9d769d"
OUTPUT_TEMPLATE_ID = "4bf0119b-40f7-40eb-ac92-ec39b45a4b5d"
ORPHAN_TABLE_ID = "2c9f38da-a794-43d7-879a-a7d620752140"
NAMESPACE = uuid.UUID("4f43f564-8f1c-4e9b-a954-4cabf808ae15")


@dataclass(frozen=True)
class InputRoute:
    name: str
    data_type: str
    source_alias: str
    target_aliases: tuple[str, ...]


@dataclass(frozen=True)
class OutputRoute:
    name: str
    data_type: str
    source_alias: str
    target_aliases: tuple[str, ...]


@dataclass(frozen=True)
class ModuleSpec:
    alias: str
    description: str
    instance_name: str
    position: tuple[int, int]
    size: tuple[int, int]
    members: tuple[str, ...]
    renames: dict[str, str]
    inputs: tuple[InputRoute, ...]
    outputs: tuple[OutputRoute, ...]
    internal_positions: dict[str, tuple[int, int]]


MODULES = (
    ModuleSpec(
        alias="StateClassifier",
        description="根据车速、在控状态和历史状态计算ACC系统状态与当前控制使能。",
        instance_name="compositeBlockInstance_3",
        position=(-1260, -620),
        size=(300, 260),
        members=(
            "isLowSpeed",
            "NOT_NotLow",
            "NOT_Standby",
            "NOT_NoHist",
            "AND_S0",
            "AND_S1",
            "AND_S2",
            "i",
            "常量_1",
            "乘法运算_1",
            "乘法运算_2",
            "加法运算",
        ),
        renames={
            "isLowSpeed": "IsLowSpeed",
            "NOT_NotLow": "IsSpeedValid",
            "NOT_Standby": "IsStandby",
            "NOT_NoHist": "HasNoHistory",
            "AND_S0": "IsActive",
            "AND_S1": "IsStandbyWithHistory",
            "AND_S2": "IsStandbyWithoutHistory",
            "i": "StateCodeNoHistory",
            "常量_1": "StateCodeLowSpeed",
            "乘法运算_1": "EncodeLowSpeed",
            "乘法运算_2": "EncodeNoHistory",
            "加法运算": "EncodeSystemState",
        },
        inputs=(
            InputRoute("egoV", "double", "egoV", ("isLowSpeed",)),
            InputRoute("vMin", "double", "RP_vMin", ("isLowSpeed",)),
            InputRoute(
                "controlEnabled",
                "int",
                "RS_ControlEnabled",
                ("AND_S0", "NOT_Standby"),
            ),
            InputRoute(
                "hasHistory",
                "int",
                "RS_HasHistory",
                ("AND_S1", "NOT_NoHist"),
            ),
        ),
        outputs=(
            OutputRoute("systemState", "int", "加法运算", ("真值表",)),
            OutputRoute("enable", "int", "AND_S0", ("输出",)),
        ),
        internal_positions={
            "isLowSpeed": (-650, -360),
            "NOT_NotLow": (-400, -360),
            "NOT_Standby": (-650, -80),
            "NOT_NoHist": (-650, 200),
            "AND_S0": (-120, -360),
            "AND_S1": (-120, -80),
            "AND_S2": (-120, 200),
            "i": (120, 330),
            "常量_1": (120, 480),
            "乘法运算_1": (350, 430),
            "乘法运算_2": (350, 180),
            "加法运算": (610, 100),
        },
    ),
    ModuleSpec(
        alias="ControlStateUpdate",
        description="根据启控、制动和取消指令更新下一周期在控状态与历史状态。",
        instance_name="compositeBlockInstance_4",
        position=(-330, -650),
        size=(320, 260),
        members=(
            "常量_2",
            "常量_3",
            "常量_4",
            "常量_5",
            "等于判断_8",
            "等于判断_1",
            "等于判断_2",
            "等于判断_3",
            "OR_TurnOn",
            "OR_Engaged",
            "OR_BrakeCancel",
            "NOT_NoBrakeCancel",
            "AND_CtrlNext",
            "OR_HasHist",
        ),
        renames={
            "常量_2": "DecisionCodeEngage",
            "常量_3": "DecisionCodeResume",
            "常量_4": "CommandCodeBrake",
            "常量_5": "CommandCodeCancel",
            "等于判断_8": "IsEngageDecision",
            "等于判断_1": "IsResumeDecision",
            "等于判断_2": "IsBrakeCommand",
            "等于判断_3": "IsCancelCommand",
            "OR_TurnOn": "HasEngageRequest",
            "OR_Engaged": "KeepOrEngage",
            "OR_BrakeCancel": "HasCancelRequest",
            "NOT_NoBrakeCancel": "NoCancelRequest",
            "AND_CtrlNext": "NextControlEnabled",
            "OR_HasHist": "NextHasHistory",
        },
        inputs=(
            InputRoute(
                "decision", "int", "真值表", ("等于判断_8", "等于判断_1")
            ),
            InputRoute(
                "commandType", "int", "输入_2", ("等于判断_2", "等于判断_3")
            ),
            InputRoute(
                "controlEnabled", "int", "RS_ControlEnabled", ("OR_Engaged",)
            ),
            InputRoute("hasHistory", "int", "RS_HasHistory", ("OR_HasHist",)),
        ),
        outputs=(
            OutputRoute(
                "controlEnabledNext", "int", "AND_CtrlNext", ("WS_ControlEnabled",)
            ),
            OutputRoute("hasHistoryNext", "int", "OR_HasHist", ("WS_HasHistory",)),
        ),
        internal_positions={
            "常量_2": (-760, -420),
            "常量_3": (-760, -230),
            "常量_4": (-760, 20),
            "常量_5": (-760, 220),
            "等于判断_8": (-510, -420),
            "等于判断_1": (-510, -220),
            "等于判断_2": (-510, 20),
            "等于判断_3": (-510, 220),
            "OR_TurnOn": (-230, -330),
            "OR_Engaged": (20, -280),
            "OR_BrakeCancel": (-230, 120),
            "NOT_NoBrakeCancel": (20, 120),
            "AND_CtrlNext": (300, -180),
            "OR_HasHist": (570, 30),
        },
    ),
    ModuleSpec(
        alias="TimeGapUpdate",
        description="根据R3/R4决策增减目标时距，并执行最小和最大时距限制。",
        instance_name="compositeBlockInstance_5",
        position=(-520, 270),
        size=(320, 300),
        members=(
            "常量_8",
            "常量_9",
            "等于判断_6",
            "等于判断_7",
            "MUL_GapDn",
            "MUL_GapUp",
            "ADD_GapRaw",
            "SUB_GapRaw",
            "FMAX_Gap",
            "FMIN_Gap",
        ),
        renames={
            "常量_8": "DecisionCodeGapDown",
            "常量_9": "DecisionCodeGapUp",
            "等于判断_6": "IsGapDecrease",
            "等于判断_7": "IsGapIncrease",
            "MUL_GapDn": "GapDecreaseDelta",
            "MUL_GapUp": "GapIncreaseDelta",
            "ADD_GapRaw": "GapAfterIncrease",
            "SUB_GapRaw": "GapCandidate",
            "FMAX_Gap": "ApplyMinGap",
            "FMIN_Gap": "ApplyMaxGap",
        },
        inputs=(
            InputRoute(
                "decision", "int", "真值表", ("等于判断_6", "等于判断_7")
            ),
            InputRoute("timeGap", "double", "RS_TimeGap", ("ADD_GapRaw",)),
            InputRoute(
                "gapStep", "double", "GapStep", ("MUL_GapDn", "MUL_GapUp")
            ),
            InputRoute("minGap", "double", "MinGap", ("FMAX_Gap",)),
            InputRoute("maxGap", "double", "MaxGap", ("FMIN_Gap",)),
        ),
        outputs=(
            OutputRoute(
                "timeGapNext", "double", "FMIN_Gap", ("WS_TimeGap", "输出-1")
            ),
        ),
        internal_positions={
            "常量_8": (-760, -350),
            "常量_9": (-760, -120),
            "等于判断_6": (-510, -350),
            "等于判断_7": (-510, -120),
            "MUL_GapDn": (-220, -320),
            "MUL_GapUp": (-220, -70),
            "ADD_GapRaw": (40, -70),
            "SUB_GapRaw": (300, -160),
            "FMAX_Gap": (560, -160),
            "FMIN_Gap": (840, -160),
        },
    ),
    ModuleSpec(
        alias="MaxSpeedUpdate",
        description="根据R1/R2决策增减巡航速度上限，并执行最低速度限制。",
        instance_name="compositeBlockInstance_6",
        position=(-520, -160),
        size=(320, 260),
        members=(
            "常量_6",
            "常量_7",
            "等于判断_4",
            "等于判断_5",
            "MUL_SpdDn",
            "MUL_SpdUp",
            "SUB_SpdC",
            "ADD_SpdC",
            "FMAX_Spd",
        ),
        renames={
            "常量_6": "DecisionCodeSpeedDown",
            "常量_7": "DecisionCodeSpeedUp",
            "等于判断_4": "IsSpeedDecrease",
            "等于判断_5": "IsSpeedIncrease",
            "MUL_SpdDn": "SpeedDecreaseDelta",
            "MUL_SpdUp": "SpeedIncreaseDelta",
            "SUB_SpdC": "SpeedAfterDecrease",
            "ADD_SpdC": "SpeedCandidate",
            "FMAX_Spd": "ApplyMinSpeed",
        },
        inputs=(
            InputRoute(
                "decision", "int", "真值表", ("等于判断_4", "等于判断_5")
            ),
            InputRoute("maxSpeed", "double", "RS_MaxSpeed", ("SUB_SpdC",)),
            InputRoute(
                "speedStep", "double", "SpdStep", ("MUL_SpdDn", "MUL_SpdUp")
            ),
            InputRoute("minSpeed", "double", "MinSpd", ("FMAX_Spd",)),
        ),
        outputs=(
            OutputRoute(
                "maxSpeedNext", "double", "FMAX_Spd", ("WS_MaxSpeed", "输出-2")
            ),
        ),
        internal_positions={
            "常量_6": (-760, -300),
            "常量_7": (-760, -70),
            "等于判断_4": (-510, -300),
            "等于判断_5": (-510, -70),
            "MUL_SpdDn": (-220, -270),
            "MUL_SpdUp": (-220, -20),
            "SUB_SpdC": (40, -180),
            "ADD_SpdC": (300, -100),
            "FMAX_Spd": (580, -100),
        },
    ),
)


TOP_RENAMES = {
    "输入_2": "commandType",
    "输出": "enable",
    "输出-1": "timeGap",
    "输出-2": "maxSpeed",
    "真值表": "DecisionTable",
}


TOP_POSITIONS = {
    "egoV": (-1780, -700),
    "输入_2": (-1040, -820),
    "RP_vMin": (-1780, -560),
    "RS_ControlEnabled": (-1780, -420),
    "RS_HasHistory": (-1780, -280),
    "RS_MaxSpeed": (-1320, -150),
    "SpdStep": (-1320, 0),
    "MinSpd": (-1320, 150),
    "RS_TimeGap": (-1320, 340),
    "GapStep": (-1320, 490),
    "MinGap": (-1040, 420),
    "MaxGap": (-1040, 570),
    "真值表": (-760, -590),
    "WS_ControlEnabled": (80, -650),
    "WS_HasHistory": (80, -470),
    "输出": (430, -600),
    "WS_MaxSpeed": (0, -100),
    "输出-2": (350, -70),
    "WS_TimeGap": (0, 360),
    "输出-1": (350, 390),
}


def stable_id(label: str) -> str:
    return str(uuid.uuid5(NAMESPACE, label))


def json_text(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, separators=(",", ":"))


def endpoint(cell: str, port: str) -> str:
    return json_text({"cell": cell, "port": port})


def load_row(conn: sqlite3.Connection, component_id: str) -> dict[str, Any]:
    row = conn.execute(
        "SELECT * FROM project_component WHERE id = ?", (component_id,)
    ).fetchone()
    if row is None:
        raise RuntimeError(f"Missing component: {component_id}")
    return dict(row)


def components_by_alias(
    conn: sqlite3.Connection, parent_id: str
) -> dict[str, dict[str, Any]]:
    rows = conn.execute(
        "SELECT * FROM project_component WHERE parentId = ?", (parent_id,)
    ).fetchall()
    result: dict[str, dict[str, Any]] = {}
    for row in rows:
        item = dict(row)
        alias = item["alias"]
        if alias in result:
            raise RuntimeError(f"Duplicate alias under {parent_id}: {alias}")
        result[alias] = item
    return result


def insert_component(conn: sqlite3.Connection, row: dict[str, Any]) -> None:
    columns = list(row)
    placeholders = ",".join("?" for _ in columns)
    conn.execute(
        f"INSERT INTO project_component ({','.join(columns)}) VALUES ({placeholders})",
        [row[column] for column in columns],
    )


def update_alias(conn: sqlite3.Connection, component_id: str, alias: str) -> None:
    row = load_row(conn, component_id)
    properties = json.loads(row["properties"])
    properties["alias"] = alias
    conn.execute(
        "UPDATE project_component SET alias = ?, properties = ? WHERE id = ?",
        (alias, json_text(properties), component_id),
    )


def update_position(
    conn: sqlite3.Connection,
    component_id: str,
    old_parent: str,
    new_parent: str,
    position: tuple[int, int],
) -> None:
    row = load_row(conn, component_id)
    attributes = json.loads(row["attribute"] or "{}")
    layout = copy.deepcopy(attributes.get(old_parent, {}))
    if not layout and attributes:
        layout = copy.deepcopy(next(iter(attributes.values())))
    layout.setdefault("size", {"width": 170, "height": 102})
    layout["position"] = {"x": position[0], "y": position[1]}
    layout.setdefault("zIndex", 30)
    conn.execute(
        "UPDATE project_component SET attribute = ? WHERE id = ?",
        (json_text({new_parent: layout}), component_id),
    )


def port_props(name: str, data_type: str) -> list[dict[str, Any]]:
    return [
        {"key": "isReturnFlag", "value": False, "props": []},
        {"key": "name", "value": name, "props": []},
        {"key": "category", "value": 0, "props": []},
        {"key": "dataType", "value": data_type, "props": []},
        {"key": "shape", "value": [1], "props": []},
    ]


def composite_port(
    port_id: str, name: str, data_type: str, direction: str
) -> dict[str, Any]:
    return {
        "id": port_id,
        "name": name,
        "direction": direction,
        "description": "函数输入端口" if direction == "In" else "函数输出端口",
        "props": port_props(name, data_type),
    }


def create_boundary_node(
    template: dict[str, Any],
    node_id: str,
    parent_id: str,
    name: str,
    data_type: str,
    position: tuple[int, int],
    is_input: bool,
) -> dict[str, Any]:
    row = copy.deepcopy(template)
    row["id"] = node_id
    row["parentId"] = parent_id
    row["instanceId"] = ""
    row["alias"] = name
    properties = json.loads(row["properties"])
    properties["alias"] = name
    row["properties"] = json_text(properties)
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
    width = max(118, 64 + 8 * len(name))
    row["attribute"] = json_text(
        {
            parent_id: {
                "size": {"width": width, "height": 68},
                "position": {"x": position[0], "y": position[1]},
                "zIndex": 40,
            }
        }
    )
    return row


def find_edge(
    edges: list[dict[str, Any]], source_id: str, target_id: str
) -> dict[str, Any]:
    matches = []
    for edge in edges:
        source = json.loads(edge["source"])
        target = json.loads(edge["target"])
        if source["cell"] == source_id and target["cell"] == target_id:
            matches.append(edge)
    if len(matches) != 1:
        raise RuntimeError(
            f"Expected one edge {source_id} -> {target_id}, found {len(matches)}"
        )
    return matches[0]


def insert_connection(
    conn: sqlite3.Connection,
    template: dict[str, Any],
    parent_id: str,
    source: dict[str, str],
    target: dict[str, str],
    label: str,
) -> None:
    connection_id = stable_id(f"connection:{label}")
    row = copy.deepcopy(template)
    row["id"] = connection_id
    row["parentId"] = parent_id
    row["source"] = json_text(source)
    row["target"] = json_text(target)
    columns = list(row)
    placeholders = ",".join("?" for _ in columns)
    conn.execute(
        f"INSERT INTO project_connection ({','.join(columns)}) VALUES ({placeholders})",
        [row[column] for column in columns],
    )


def create_module(
    conn: sqlite3.Connection,
    spec: ModuleSpec,
    composite_template: dict[str, Any],
    input_template: dict[str, Any],
    output_template: dict[str, Any],
) -> tuple[str, dict[str, str], dict[str, str]]:
    module_id = stable_id(f"module:{spec.alias}")
    input_ids = {
        route.name: stable_id(f"module:{spec.alias}:input:{route.name}")
        for route in spec.inputs
    }
    output_ids = {
        route.name: stable_id(f"module:{spec.alias}:output:{route.name}")
        for route in spec.outputs
    }

    module = copy.deepcopy(composite_template)
    module["id"] = module_id
    module["parentId"] = DECISION_ID
    module["instanceId"] = ""
    module["alias"] = spec.alias
    module["description"] = spec.description
    module["childComponents"] = "[]"
    module["childConnections"] = "[]"
    properties = json.loads(module["properties"])
    properties["alias"] = spec.alias
    properties["description"] = spec.description
    module["properties"] = json_text(properties)
    module["extensionProps"] = json_text(
        [
            {
                "key": "base_config",
                "value": [
                    {
                        "isCallback": False,
                        "param": [],
                        "state": [],
                        "instanceName": spec.instance_name,
                    }
                ],
                "props": [],
            },
            {
                "key": "function_input_ports",
                "value": [
                    composite_port(input_ids[item.name], item.name, item.data_type, "In")
                    for item in spec.inputs
                ],
            },
            {
                "key": "function_output_ports",
                "value": [
                    composite_port(
                        output_ids[item.name], item.name, item.data_type, "Out"
                    )
                    for item in spec.outputs
                ],
            },
        ]
    )
    module["attribute"] = json_text(
        {
            DECISION_ID: {
                "size": {"width": spec.size[0], "height": spec.size[1]},
                "position": {"x": spec.position[0], "y": spec.position[1]},
                "zIndex": 40,
            }
        }
    )
    insert_component(conn, module)

    input_y = -420
    for index, route in enumerate(spec.inputs):
        node = create_boundary_node(
            input_template,
            input_ids[route.name],
            module_id,
            route.name,
            route.data_type,
            (-1040, input_y + index * 180),
            True,
        )
        insert_component(conn, node)

    output_y = -260
    for index, route in enumerate(spec.outputs):
        node = create_boundary_node(
            output_template,
            output_ids[route.name],
            module_id,
            route.name,
            route.data_type,
            (1110, output_y + index * 240),
            False,
        )
        insert_component(conn, node)

    return module_id, input_ids, output_ids


def validate(conn: sqlite3.Connection) -> None:
    integrity = conn.execute("PRAGMA integrity_check").fetchone()[0]
    if integrity != "ok":
        raise RuntimeError(f"SQLite integrity check failed: {integrity}")

    dangling = conn.execute(
        """
        SELECT COUNT(*)
        FROM project_connection c
        LEFT JOIN project_component s ON s.id = json_extract(c.source, '$.cell')
        LEFT JOIN project_component t ON t.id = json_extract(c.target, '$.cell')
        WHERE s.id IS NULL OR t.id IS NULL
        """
    ).fetchone()[0]
    if dangling:
        raise RuntimeError(f"Found {dangling} dangling connection endpoints")

    boundary_errors = conn.execute(
        """
        SELECT COUNT(*)
        FROM project_connection c
        JOIN project_component s ON s.id = json_extract(c.source, '$.cell')
        JOIN project_component t ON t.id = json_extract(c.target, '$.cell')
        WHERE s.parentId <> c.parentId OR t.parentId <> c.parentId
        """
    ).fetchone()[0]
    if boundary_errors:
        raise RuntimeError(f"Found {boundary_errors} cross-boundary connections")

    duplicate_targets = conn.execute(
        """
        SELECT COUNT(*) FROM (
          SELECT parentId, target, COUNT(*) AS n
          FROM project_connection
          GROUP BY parentId, target
          HAVING n > 1
        )
        """
    ).fetchone()[0]
    if duplicate_targets:
        raise RuntimeError(f"Found {duplicate_targets} multiply-driven input ports")

    aliases = [item.alias for item in MODULES]
    placeholders = ",".join("?" for _ in aliases)
    module_count = conn.execute(
        f"SELECT COUNT(*) FROM project_component WHERE parentId=? AND alias IN ({placeholders})",
        [DECISION_ID, *aliases],
    ).fetchone()[0]
    if module_count != len(MODULES):
        raise RuntimeError(f"Expected {len(MODULES)} new modules, found {module_count}")

    orphan_count = conn.execute(
        "SELECT COUNT(*) FROM project_component WHERE id=?", (ORPHAN_TABLE_ID,)
    ).fetchone()[0]
    if orphan_count:
        raise RuntimeError("Disconnected truth-table_1 still exists")


def refactor(db_path: Path) -> None:
    conn = sqlite3.connect(db_path)
    conn.row_factory = sqlite3.Row
    conn.execute("PRAGMA foreign_keys=OFF")
    try:
        with conn:
            current = components_by_alias(conn, DECISION_ID)
            if all(spec.alias in current for spec in MODULES):
                raise RuntimeError("Decision canvas is already refactored")

            required_aliases = {
                alias
                for spec in MODULES
                for alias in (
                    *spec.members,
                    *(route.source_alias for route in spec.inputs),
                    *(target for route in spec.inputs for target in route.target_aliases),
                    *(route.source_alias for route in spec.outputs),
                    *(target for route in spec.outputs for target in route.target_aliases),
                )
            }
            missing = sorted(required_aliases - set(current))
            if missing:
                raise RuntimeError(f"Missing expected Decision components: {missing}")

            orphan_edges = conn.execute(
                "SELECT COUNT(*) FROM project_connection WHERE source LIKE ? OR target LIKE ?",
                (f'%"{ORPHAN_TABLE_ID}"%', f'%"{ORPHAN_TABLE_ID}"%'),
            ).fetchone()[0]
            orphan_children = conn.execute(
                "SELECT COUNT(*) FROM project_component WHERE parentId=?",
                (ORPHAN_TABLE_ID,),
            ).fetchone()[0]
            if orphan_edges or orphan_children:
                raise RuntimeError("truth-table_1 is not safe to remove")

            composite_template = load_row(conn, CONTROL_TEMPLATE_ID)
            input_template = load_row(conn, INPUT_TEMPLATE_ID)
            output_template = load_row(conn, OUTPUT_TEMPLATE_ID)
            connection_template = dict(
                conn.execute("SELECT * FROM project_connection LIMIT 1").fetchone()
            )
            original_edges = [
                dict(row)
                for row in conn.execute(
                    "SELECT * FROM project_connection WHERE parentId=?", (DECISION_ID,)
                ).fetchall()
            ]
            alias_to_id = {alias: row["id"] for alias, row in current.items()}

            group_for_id: dict[str, str] = {}
            expected_crossing_ids: set[str] = set()
            route_edges: dict[tuple[str, str, str], list[dict[str, Any]]] = {}
            created: dict[str, tuple[str, dict[str, str], dict[str, str]]] = {}

            for spec in MODULES:
                module_id, input_ids, output_ids = create_module(
                    conn,
                    spec,
                    composite_template,
                    input_template,
                    output_template,
                )
                created[spec.alias] = (module_id, input_ids, output_ids)
                for member in spec.members:
                    component_id = alias_to_id[member]
                    group_for_id[component_id] = module_id

                for route in spec.inputs:
                    edges = []
                    for target_alias in route.target_aliases:
                        edge = find_edge(
                            original_edges,
                            alias_to_id[route.source_alias],
                            alias_to_id[target_alias],
                        )
                        edges.append(edge)
                        expected_crossing_ids.add(edge["id"])
                    route_edges[(spec.alias, "in", route.name)] = edges

                for route in spec.outputs:
                    edges = []
                    for target_alias in route.target_aliases:
                        edge = find_edge(
                            original_edges,
                            alias_to_id[route.source_alias],
                            alias_to_id[target_alias],
                        )
                        edges.append(edge)
                        expected_crossing_ids.add(edge["id"])
                    route_edges[(spec.alias, "out", route.name)] = edges

            actual_crossing_ids = set()
            for edge in original_edges:
                source = json.loads(edge["source"])
                target = json.loads(edge["target"])
                source_group = group_for_id.get(source["cell"], DECISION_ID)
                target_group = group_for_id.get(target["cell"], DECISION_ID)
                if source_group != target_group:
                    actual_crossing_ids.add(edge["id"])

            if actual_crossing_ids != expected_crossing_ids:
                missing_routes = sorted(actual_crossing_ids - expected_crossing_ids)
                extra_routes = sorted(expected_crossing_ids - actual_crossing_ids)
                raise RuntimeError(
                    "Crossing edge specification mismatch: "
                    f"unhandled={missing_routes}, unexpected={extra_routes}"
                )

            for spec in MODULES:
                module_id = created[spec.alias][0]
                for member in spec.members:
                    component_id = alias_to_id[member]
                    conn.execute(
                        "UPDATE project_component SET parentId=? WHERE id=?",
                        (module_id, component_id),
                    )
                    update_position(
                        conn,
                        component_id,
                        DECISION_ID,
                        module_id,
                        spec.internal_positions[member],
                    )

            for edge in original_edges:
                source = json.loads(edge["source"])
                target = json.loads(edge["target"])
                source_group = group_for_id.get(source["cell"], DECISION_ID)
                target_group = group_for_id.get(target["cell"], DECISION_ID)
                if source_group == target_group and source_group != DECISION_ID:
                    conn.execute(
                        "UPDATE project_connection SET parentId=? WHERE id=?",
                        (source_group, edge["id"]),
                    )
                elif source_group != target_group:
                    conn.execute(
                        "DELETE FROM project_connection WHERE id=?", (edge["id"],)
                    )

            for spec in MODULES:
                module_id, input_ids, output_ids = created[spec.alias]
                for route in spec.inputs:
                    edges = route_edges[(spec.alias, "in", route.name)]
                    original_source = json.loads(edges[0]["source"])
                    for edge in edges[1:]:
                        if json.loads(edge["source"]) != original_source:
                            raise RuntimeError(f"Input source mismatch: {spec.alias}.{route.name}")
                    insert_connection(
                        conn,
                        connection_template,
                        DECISION_ID,
                        original_source,
                        {"cell": module_id, "port": input_ids[route.name]},
                        f"top:{spec.alias}:input:{route.name}",
                    )
                    for index, edge in enumerate(edges):
                        insert_connection(
                            conn,
                            connection_template,
                            module_id,
                            {"cell": input_ids[route.name], "port": "out"},
                            json.loads(edge["target"]),
                            f"inside:{spec.alias}:input:{route.name}:{index}",
                        )

                for route in spec.outputs:
                    edges = route_edges[(spec.alias, "out", route.name)]
                    original_source = json.loads(edges[0]["source"])
                    for edge in edges[1:]:
                        if json.loads(edge["source"]) != original_source:
                            raise RuntimeError(f"Output source mismatch: {spec.alias}.{route.name}")
                    insert_connection(
                        conn,
                        connection_template,
                        module_id,
                        original_source,
                        {"cell": output_ids[route.name], "port": "in"},
                        f"inside:{spec.alias}:output:{route.name}",
                    )
                    for index, edge in enumerate(edges):
                        insert_connection(
                            conn,
                            connection_template,
                            DECISION_ID,
                            {"cell": module_id, "port": output_ids[route.name]},
                            json.loads(edge["target"]),
                            f"top:{spec.alias}:output:{route.name}:{index}",
                        )

            for old_alias, new_alias in TOP_RENAMES.items():
                update_alias(conn, alias_to_id[old_alias], new_alias)

            for spec in MODULES:
                for old_alias, new_alias in spec.renames.items():
                    update_alias(conn, alias_to_id[old_alias], new_alias)

            for alias, position in TOP_POSITIONS.items():
                update_position(
                    conn,
                    alias_to_id[alias],
                    DECISION_ID,
                    DECISION_ID,
                    position,
                )

            conn.execute(
                "DELETE FROM project_component WHERE id=?", (ORPHAN_TABLE_ID,)
            )

            validate(conn)
    finally:
        conn.close()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--project",
        type=Path,
        default=Path("project/newaccpro3"),
        help="Path to the GAASD project directory",
    )
    args = parser.parse_args()

    project = args.project.resolve()
    cbdes = project / "data" / "cbdes.db"
    temp = project / "data" / "temp.db"
    if not cbdes.is_file() or not temp.is_file():
        raise RuntimeError(f"Missing GAASD databases under {project / 'data'}")

    backup = cbdes.with_name("cbdes.db.before_decision_refactor")
    if not backup.exists():
        shutil.copy2(cbdes, backup)

    refactor(cbdes)
    shutil.copy2(cbdes, temp)
    if sha256(cbdes) != sha256(temp):
        raise RuntimeError("cbdes.db and temp.db differ after synchronization")

    print(f"Refactored: {cbdes}")
    print(f"Backup:     {backup}")
    print(f"SHA256:     {sha256(cbdes)}")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(1)
