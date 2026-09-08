#!/usr/bin/env python3
"""Split newaccpro3 control calculations into physical lower-camel subsystems."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import shutil
import sqlite3
import sys
import uuid
from collections import defaultdict, deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any


MAIN_ID = "2f3edfa5-8227-43eb-859d-1a6db0afd957"
DECISION_ID = "ea2170e7-297c-4433-b70a-9c66e48988ad"
CONTROL_ID = "98d0f783-5cfa-413d-815a-dcd775b5dd13"
INPUT_TEMPLATE_ID = "eb1e8ad8-1ad9-4ebb-85ed-099f8e9d769d"
OUTPUT_TEMPLATE_ID = "4bf0119b-40f7-40eb-ac92-ec39b45a4b5d"
NAMESPACE = uuid.UUID("22874af0-b893-48c7-a4e7-d8e943222cc3")

TARGET_MODULE_ALIAS = "TargetSpeedCalculation"
TARGET_MODULE_DESCRIPTION = (
    "根据自车速度、前车状态、目标时距和控制参数计算ACC目标速度。"
)

INPUT_TYPES = {
    "egoV": "double",
    "leadV": "double",
    "distance": "double",
    "timeGap": "double",
    "maxSpeed": "double",
    "enable": "int",
    "minDistance": "double",
    "kDist": "double",
    "kSpeed": "double",
}

CALCULATION_ALIASES = (
    "TimeGapDistance",
    "DesiredDistance",
    "DistanceError",
    "RelativeSpeed",
    "DistanceFeedback",
    "SpeedFeedback",
    "RawTargetSpeed",
    "NonNegativeTargetSpeed",
    "TargetSpeedLimiter",
    "EnableGate",
    "ZeroSpeed",
)

STRUCTURAL_INSTANCE_NAMES = {
    MAIN_ID: "main",
    DECISION_ID: "decision",
    CONTROL_ID: "control",
    "e5ab5065-42ec-59b6-927b-0e6104e7cea7": "stateClassifier",
    "b15eda5b-8d9a-53e8-ab7f-53c645c2f1f2": "controlStateUpdate",
    "13cf9c88-c8f8-51f9-b21d-69eaf1d44249": "timeGapUpdate",
    "4b924c5d-38b8-5e8f-bb96-9cdedaac2429": "maxSpeedUpdate",
}

STRUCTURAL_IDS = set(STRUCTURAL_INSTANCE_NAMES)

STRUCTURAL_ALIASES = {
    MAIN_ID: "main",
    DECISION_ID: "decision",
    CONTROL_ID: "control",
    "e5ab5065-42ec-59b6-927b-0e6104e7cea7": "stateClassifier",
    "b15eda5b-8d9a-53e8-ab7f-53c645c2f1f2": "controlStateUpdate",
    "13cf9c88-c8f8-51f9-b21d-69eaf1d44249": "timeGapUpdate",
    "4b924c5d-38b8-5e8f-bb96-9cdedaac2429": "maxSpeedUpdate",
}

SPECIAL_ALIAS_RENAMES = {
    "DecisionTable": "decisionTable",
    "GapStep": "readGapStep",
    "MaxGap": "readMaxGap",
    "MinGap": "readMinGap",
    "MinSpd": "readMinSpeed",
    "RP_vMin": "readVMin",
    "SpdStep": "readSpeedStep",
    "RS_ControlEnabled": "readControlEnabled",
    "RS_HasHistory": "readHasHistory",
    "RS_MaxSpeed": "readMaxSpeed",
    "RS_TimeGap": "readTimeGap",
    "WS_ControlEnabled": "writeControlEnabled",
    "WS_HasHistory": "writeHasHistory",
    "WS_MaxSpeed": "writeMaxSpeed",
    "WS_TimeGap": "writeTimeGap",
}

PARAMETER_RENAMES = {
    MAIN_ID: {
        "Kdist": "kDist",
        "MinDistance": "minDistance",
        "Kspeed": "kSpeed",
    },
    DECISION_ID: {
        "GapStep": "gapStep",
        "MinGap": "minGap",
        "MaxGap": "maxGap",
        "SpdStep": "speedStep",
        "MinSpd": "minSpeed",
    },
}

CONTROL_LAYOUT = {
    "egoV": (-1100, -680),
    "leadV": (-1100, -520),
    "distance": (-1100, -360),
    "timeGap": (-1100, -200),
    "maxSpeed": (-1100, -40),
    "enable": (-1100, 120),
    "minDistance": (-1100, 280),
    "kDist": (-1100, 440),
    "kSpeed": (-1100, 600),
    TARGET_MODULE_ALIAS: (-260, -80),
    "targetSpeed": (540, -10),
}

CALCULATION_LAYOUT = {
    "egoV": (-1450, -650),
    "timeGap": (-1450, -500),
    "minDistance": (-1450, -350),
    "distance": (-1450, -200),
    "kDist": (-1450, -50),
    "leadV": (-1450, 100),
    "kSpeed": (-1450, 250),
    "maxSpeed": (-1450, 400),
    "enable": (-1450, 550),
    "TimeGapDistance": (-980, -520),
    "DesiredDistance": (-680, -430),
    "DistanceError": (-350, -330),
    "DistanceFeedback": (-20, -300),
    "RelativeSpeed": (-680, 120),
    "SpeedFeedback": (-20, 100),
    "RawTargetSpeed": (320, -100),
    "ZeroSpeed": (330, 220),
    "NonNegativeTargetSpeed": (650, -100),
    "TargetSpeedLimiter": (970, -100),
    "EnableGate": (1290, -100),
    "targetSpeed": (1620, -100),
}


@dataclass(frozen=True)
class GroupSpec:
    alias: str
    description: str
    position: tuple[int, int]
    size: tuple[int, int]
    members: tuple[str, ...]
    internal_positions: dict[str, tuple[int, int]]


@dataclass(frozen=True)
class SignalSpec:
    name: str
    data_type: str
    source_alias: str
    target_aliases: tuple[str, ...]


CONTROL_GROUPS = (
    GroupSpec(
        alias="spacingFeedback",
        description="根据目标时距计算期望跟车距离，并形成距离误差反馈修正量。",
        position=(-360, -430),
        size=(380, 280),
        members=(
            "timeGapDistance",
            "desiredDistance",
            "distanceError",
            "distanceFeedback",
        ),
        internal_positions={
            "timeGapDistance": (-560, -420),
            "desiredDistance": (-220, -330),
            "distanceError": (120, -220),
            "distanceFeedback": (470, -120),
        },
    ),
    GroupSpec(
        alias="relativeSpeedFeedback",
        description="根据前车与自车的相对速度形成速度反馈修正量。",
        position=(-360, 260),
        size=(380, 250),
        members=("relativeSpeed", "speedFeedback"),
        internal_positions={
            "relativeSpeed": (-250, -120),
            "speedFeedback": (160, -80),
        },
    ),
    GroupSpec(
        alias="targetSpeedOutput",
        description="合成前车速度和两项反馈修正量，执行速度限幅与控制使能门控。",
        position=(430, -100),
        size=(400, 300),
        members=(
            "rawTargetSpeed",
            "zeroSpeed",
            "nonNegativeTargetSpeed",
            "targetSpeedLimiter",
            "enableGate",
        ),
        internal_positions={
            "rawTargetSpeed": (-520, -220),
            "zeroSpeed": (-520, 180),
            "nonNegativeTargetSpeed": (-160, -120),
            "targetSpeedLimiter": (210, -120),
            "enableGate": (580, -120),
        },
    ),
)

CONTROL_SIGNALS = (
    SignalSpec(
        "egoV",
        "double",
        "egoV",
        ("timeGapDistance", "relativeSpeed"),
    ),
    SignalSpec("timeGap", "double", "timeGap", ("timeGapDistance",)),
    SignalSpec("minDistance", "double", "minDistance", ("desiredDistance",)),
    SignalSpec("distance", "double", "distance", ("distanceError",)),
    SignalSpec("kDist", "double", "kDist", ("distanceFeedback",)),
    SignalSpec(
        "leadV",
        "double",
        "leadV",
        ("relativeSpeed", "rawTargetSpeed"),
    ),
    SignalSpec("kSpeed", "double", "kSpeed", ("speedFeedback",)),
    SignalSpec(
        "distanceFeedback",
        "double",
        "distanceFeedback",
        ("rawTargetSpeed",),
    ),
    SignalSpec(
        "speedFeedback",
        "double",
        "speedFeedback",
        ("rawTargetSpeed",),
    ),
    SignalSpec("maxSpeed", "double", "maxSpeed", ("targetSpeedLimiter",)),
    SignalSpec("enable", "int", "enable", ("enableGate",)),
    SignalSpec(
        "targetSpeed",
        "double",
        "enableGate",
        ("targetSpeed",),
    ),
)

FINAL_CONTROL_LAYOUT = {
    "egoV": (-1200, -700),
    "leadV": (-1200, -540),
    "distance": (-1200, -380),
    "timeGap": (-1200, -220),
    "maxSpeed": (-1200, -60),
    "enable": (-1200, 100),
    "minDistance": (-1200, 260),
    "kDist": (-1200, 420),
    "kSpeed": (-1200, 580),
    "spacingFeedback": (-360, -430),
    "relativeSpeedFeedback": (-360, 260),
    "targetSpeedOutput": (430, -100),
    "targetSpeed": (1080, -80),
}


def json_text(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, separators=(",", ":"))


def stable_id(label: str) -> str:
    return str(uuid.uuid5(NAMESPACE, label))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_component(conn: sqlite3.Connection, component_id: str) -> dict[str, Any]:
    row = conn.execute(
        "SELECT * FROM project_component WHERE id=?", (component_id,)
    ).fetchone()
    if row is None:
        raise RuntimeError(f"Missing component: {component_id}")
    return dict(row)


def components_by_alias(
    conn: sqlite3.Connection, parent_id: str
) -> dict[str, dict[str, Any]]:
    result: dict[str, dict[str, Any]] = {}
    for row in conn.execute(
        "SELECT * FROM project_component WHERE parentId=?", (parent_id,)
    ):
        item = dict(row)
        if item["alias"] in result:
            raise RuntimeError(
                f"Duplicate alias under {parent_id}: {item['alias']}"
            )
        result[item["alias"]] = item
    return result


def insert_component(conn: sqlite3.Connection, row: dict[str, Any]) -> None:
    columns = list(row)
    placeholders = ",".join("?" for _ in columns)
    conn.execute(
        f"INSERT INTO project_component ({','.join(columns)}) "
        f"VALUES ({placeholders})",
        [row[column] for column in columns],
    )


def update_alias(
    conn: sqlite3.Connection,
    component_id: str,
    alias: str,
    description: str | None = None,
) -> None:
    row = load_component(conn, component_id)
    properties = json.loads(row["properties"])
    properties["alias"] = alias
    values: list[Any] = [alias, json_text(properties)]
    sql = "UPDATE project_component SET alias=?, properties=?"
    if description is not None:
        properties["description"] = description
        values[1] = json_text(properties)
        sql += ", description=?"
        values.append(description)
    sql += " WHERE id=?"
    values.append(component_id)
    conn.execute(sql, values)


def update_position(
    conn: sqlite3.Connection,
    component_id: str,
    old_parent: str,
    new_parent: str,
    position: tuple[int, int],
    size: tuple[int, int] | None = None,
) -> None:
    row = load_component(conn, component_id)
    attributes = json.loads(row["attribute"] or "{}")
    layout = copy.deepcopy(attributes.get(old_parent, {}))
    if not layout and attributes:
        layout = copy.deepcopy(next(iter(attributes.values())))
    layout.setdefault("size", {"width": 170, "height": 102})
    if size is not None:
        layout["size"] = {"width": size[0], "height": size[1]}
    layout["position"] = {"x": position[0], "y": position[1]}
    layout.setdefault("zIndex", 40)
    conn.execute(
        "UPDATE project_component SET attribute=? WHERE id=?",
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
    alias: str | None = None,
) -> dict[str, Any]:
    node_alias = alias or name
    row = copy.deepcopy(template)
    row["id"] = node_id
    row["parentId"] = parent_id
    row["instanceId"] = ""
    row["alias"] = node_alias
    properties = json.loads(row["properties"])
    properties["alias"] = node_alias
    row["properties"] = json_text(properties)
    base = {
        "isReturnFlag": False,
        "name": name,
        "category": 0,
        "dataType": data_type,
        "shape": [1],
    }
    if is_input:
        extension = [
            {"key": "base_config", "value": [base]},
            {"key": "function_input_ports", "value": []},
            {
                "key": "function_output_ports",
                "value": [
                    {
                        "name": "out",
                        "dataType": data_type,
                        "description": "参数输出",
                        "alias": "out",
                        "direction": "Out",
                        "id": "out",
                        "props": port_props(name, data_type),
                    }
                ],
            },
        ]
    else:
        extension = [
            {"key": "base_config", "value": [base]},
            {
                "key": "function_input_ports",
                "value": [
                    {
                        "name": "in",
                        "dataType": data_type,
                        "description": "参数输入",
                        "alias": "in",
                        "direction": "In",
                        "id": "in",
                        "props": port_props(name, data_type),
                    }
                ],
            },
            {"key": "function_output_ports", "value": []},
        ]
    row["extensionProps"] = json_text(extension)
    row["attribute"] = json_text(
        {
            parent_id: {
                "size": {
                    "width": max(118, 64 + 8 * len(name)),
                    "height": 68,
                },
                "position": {"x": position[0], "y": position[1]},
                "zIndex": 40,
            }
        }
    )
    return row


def create_target_speed_module(
    conn: sqlite3.Connection,
    composite_template: dict[str, Any],
    input_template: dict[str, Any],
    output_template: dict[str, Any],
) -> tuple[str, dict[str, str], str]:
    module_id = stable_id("newaccpro3:control:targetSpeedCalculation")
    input_ids = {
        name: stable_id(f"newaccpro3:control:targetSpeedCalculation:input:{name}")
        for name in INPUT_TYPES
    }
    output_id = stable_id(
        "newaccpro3:control:targetSpeedCalculation:output:targetSpeed"
    )

    module = copy.deepcopy(composite_template)
    module["id"] = module_id
    module["parentId"] = CONTROL_ID
    module["instanceId"] = ""
    module["alias"] = TARGET_MODULE_ALIAS
    module["description"] = TARGET_MODULE_DESCRIPTION
    module["childComponents"] = "[]"
    module["childConnections"] = "[]"
    properties = json.loads(module["properties"])
    properties["alias"] = TARGET_MODULE_ALIAS
    properties["description"] = TARGET_MODULE_DESCRIPTION
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
                        "instanceName": "targetSpeedCalculation",
                    }
                ],
                "props": [],
            },
            {
                "key": "function_input_ports",
                "value": [
                    composite_port(input_ids[name], name, data_type, "In")
                    for name, data_type in INPUT_TYPES.items()
                ],
            },
            {
                "key": "function_output_ports",
                "value": [
                    composite_port(output_id, "targetSpeed", "double", "Out")
                ],
            },
        ]
    )
    module["attribute"] = json_text(
        {
            CONTROL_ID: {
                "size": {"width": 430, "height": 300},
                "position": {
                    "x": CONTROL_LAYOUT[TARGET_MODULE_ALIAS][0],
                    "y": CONTROL_LAYOUT[TARGET_MODULE_ALIAS][1],
                },
                "zIndex": 40,
            }
        }
    )
    insert_component(conn, module)

    for name, data_type in INPUT_TYPES.items():
        insert_component(
            conn,
            create_boundary_node(
                input_template,
                input_ids[name],
                module_id,
                name,
                data_type,
                CALCULATION_LAYOUT[name],
                True,
            ),
        )
    insert_component(
        conn,
        create_boundary_node(
            output_template,
            output_id,
            module_id,
            "targetSpeed",
            "double",
            CALCULATION_LAYOUT["targetSpeed"],
            False,
        ),
    )
    return module_id, input_ids, output_id


def insert_connection(
    conn: sqlite3.Connection,
    template: dict[str, Any],
    parent_id: str,
    source: dict[str, str],
    target: dict[str, str],
    label: str,
) -> None:
    row = copy.deepcopy(template)
    row["id"] = stable_id(f"connection:{label}")
    row["parentId"] = parent_id
    row["source"] = json_text(source)
    row["target"] = json_text(target)
    columns = list(row)
    placeholders = ",".join("?" for _ in columns)
    conn.execute(
        f"INSERT INTO project_connection ({','.join(columns)}) "
        f"VALUES ({placeholders})",
        [row[column] for column in columns],
    )


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


def create_control_group(
    conn: sqlite3.Connection,
    spec: GroupSpec,
    input_names: list[str],
    output_names: list[str],
    signal_types: dict[str, str],
    composite_template: dict[str, Any],
    input_template: dict[str, Any],
    output_template: dict[str, Any],
) -> tuple[str, dict[str, str], dict[str, str]]:
    module_id = stable_id(f"newaccpro3:control:group:{spec.alias}")
    input_ids = {
        name: stable_id(f"newaccpro3:control:group:{spec.alias}:input:{name}")
        for name in input_names
    }
    output_ids = {
        name: stable_id(f"newaccpro3:control:group:{spec.alias}:output:{name}")
        for name in output_names
    }

    module = copy.deepcopy(composite_template)
    module["id"] = module_id
    module["parentId"] = CONTROL_ID
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
                        "instanceName": spec.alias,
                    }
                ],
                "props": [],
            },
            {
                "key": "function_input_ports",
                "value": [
                    composite_port(
                        input_ids[name], name, signal_types[name], "In"
                    )
                    for name in input_names
                ],
            },
            {
                "key": "function_output_ports",
                "value": [
                    composite_port(
                        output_ids[name], name, signal_types[name], "Out"
                    )
                    for name in output_names
                ],
            },
        ]
    )
    module["attribute"] = json_text(
        {
            CONTROL_ID: {
                "size": {"width": spec.size[0], "height": spec.size[1]},
                "position": {"x": spec.position[0], "y": spec.position[1]},
                "zIndex": 40,
            }
        }
    )
    insert_component(conn, module)

    input_start = -420 - max(0, len(input_names) - 3) * 60
    for index, name in enumerate(input_names):
        insert_component(
            conn,
            create_boundary_node(
                input_template,
                input_ids[name],
                module_id,
                name,
                signal_types[name],
                (-1050, input_start + index * 180),
                True,
            ),
        )
    output_start = -100 - max(0, len(output_names) - 1) * 100
    for index, name in enumerate(output_names):
        boundary_alias = f"{name}Out" if name in spec.members else name
        insert_component(
            conn,
            create_boundary_node(
                output_template,
                output_ids[name],
                module_id,
                name,
                signal_types[name],
                (900, output_start + index * 200),
                False,
                boundary_alias,
            ),
        )
    return module_id, input_ids, output_ids


def external_ports(row: sqlite3.Row, section_key: str) -> list[dict[str, Any]]:
    for section in json.loads(row["extensionProps"]):
        if section.get("key") == section_key:
            return section.get("value", [])
    return []


def boundary_base_name(row: sqlite3.Row) -> str | None:
    for section in json.loads(row["extensionProps"]):
        if section.get("key") == "base_config":
            values = section.get("value", [])
            if values and isinstance(values[0], dict):
                return values[0].get("name")
    return None


def flattened_signature(
    conn: sqlite3.Connection,
) -> list[tuple[str, str, str, str]]:
    components = {
        row["id"]: row for row in conn.execute("SELECT * FROM project_component")
    }
    adjacency: dict[tuple[str, str], list[tuple[str, str]]] = defaultdict(list)
    for edge in conn.execute("SELECT source,target FROM project_connection"):
        source = json.loads(edge["source"])
        target = json.loads(edge["target"])
        adjacency[(source["cell"], source["port"])].append(
            (target["cell"], target["port"])
        )

    composite_ids = {
        component_id
        for component_id, row in components.items()
        if row["componentType"] in {"run", "composite-block"}
    }
    boundary_ids = {
        component_id
        for component_id, row in components.items()
        if row["componentType"] in {"input", "output"}
        and row["parentId"] in composite_ids
    }
    virtual_ids = composite_ids | boundary_ids

    for composite_id in composite_ids:
        composite = components[composite_id]
        children = [
            row for row in components.values() if row["parentId"] == composite_id
        ]
        input_children = [
            row for row in children if row["componentType"] == "input"
        ]
        output_children = [
            row for row in children if row["componentType"] == "output"
        ]

        def match_child(
            port: dict[str, Any], candidates: list[sqlite3.Row]
        ) -> sqlite3.Row:
            matches = [
                row
                for row in candidates
                if boundary_base_name(row) == port.get("name")
            ]
            if len(matches) == 1:
                return matches[0]
            same_id = [row for row in candidates if row["id"] == port.get("id")]
            if len(same_id) == 1:
                return same_id[0]
            if len(candidates) == 1:
                return candidates[0]
            raise RuntimeError(
                f"Cannot match composite port {composite_id}.{port.get('name')}"
            )

        for port in external_ports(composite, "function_input_ports"):
            child = match_child(port, input_children)
            adjacency[(composite_id, port["id"])].append((child["id"], "out"))
        for port in external_ports(composite, "function_output_ports"):
            child = match_child(port, output_children)
            adjacency[(child["id"], "in")].append((composite_id, port["id"]))

    signature: set[tuple[str, str, str, str]] = set()
    for source_endpoint in list(adjacency):
        if source_endpoint[0] in virtual_ids:
            continue
        queue: deque[tuple[str, str]] = deque(adjacency[source_endpoint])
        visited: set[tuple[str, str]] = set()
        while queue:
            endpoint = queue.popleft()
            if endpoint in visited:
                continue
            visited.add(endpoint)
            if endpoint[0] not in virtual_ids:
                signature.add(
                    (source_endpoint[0], source_endpoint[1], endpoint[0], endpoint[1])
                )
                continue
            queue.extend(adjacency.get(endpoint, []))
    return sorted(signature)


def descendants(conn: sqlite3.Connection, root_id: str) -> list[sqlite3.Row]:
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
            (root_id,),
        )
    )


def lower_camel(alias: str) -> str:
    if alias in SPECIAL_ALIAS_RENAMES:
        return SPECIAL_ALIAS_RENAMES[alias]
    if alias and "A" <= alias[0] <= "Z":
        return alias[0].lower() + alias[1:]
    return alias


def normalize_aliases(conn: sqlite3.Connection, target_module_id: str) -> None:
    structural_ids = STRUCTURAL_IDS | {target_module_id}
    proposed: dict[str, str] = {}
    rows = descendants(conn, MAIN_ID)
    for row in rows:
        if row["id"] in structural_ids:
            continue
        proposed[row["id"]] = lower_camel(row["alias"])

    parent_aliases: dict[str, set[str]] = defaultdict(set)
    for row in rows:
        alias = proposed.get(row["id"], row["alias"])
        if alias in parent_aliases[row["parentId"]]:
            raise RuntimeError(
                f"Alias collision after lower-camel normalization: "
                f"{row['parentId']}.{alias}"
            )
        parent_aliases[row["parentId"]].add(alias)

    for component_id, alias in proposed.items():
        row = load_component(conn, component_id)
        if alias != row["alias"]:
            update_alias(conn, component_id, alias)


def normalize_structure_names(
    conn: sqlite3.Connection, group_ids: dict[str, str]
) -> None:
    names = dict(STRUCTURAL_INSTANCE_NAMES)
    names.update({component_id: alias for alias, component_id in group_ids.items()})
    aliases = dict(STRUCTURAL_ALIASES)
    aliases.update({component_id: alias for alias, component_id in group_ids.items()})
    descriptions = {
        MAIN_ID: "ACC决策与纵向控制主流程。",
        DECISION_ID: "根据车辆状态和驾驶指令更新ACC状态、目标时距与速度上限。",
        CONTROL_ID: "根据前车状态、跟车误差和决策参数计算ACC目标速度。",
        "e5ab5065-42ec-59b6-927b-0e6104e7cea7": (
            "根据车速、在控记忆与历史状态识别ACC系统状态。"
        ),
        "b15eda5b-8d9a-53e8-ab7f-53c645c2f1f2": (
            "更新ACC控制使能、在控记忆和历史状态。"
        ),
        "13cf9c88-c8f8-51f9-b21d-69eaf1d44249": (
            "根据驾驶指令更新目标时距。"
        ),
        "4b924c5d-38b8-5e8f-bb96-9cdedaac2429": (
            "根据驾驶指令更新目标速度上限。"
        ),
    }
    descriptions.update(
        {
            group_ids[spec.alias]: spec.description
            for spec in CONTROL_GROUPS
        }
    )
    for component_id, alias in aliases.items():
        update_alias(conn, component_id, alias, descriptions.get(component_id))

    for component_id, instance_name in names.items():
        row = load_component(conn, component_id)
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
            raise RuntimeError(f"Missing base_config instanceName: {component_id}")
        conn.execute(
            "UPDATE project_component SET extensionProps=? WHERE id=?",
            (json_text(sections), component_id),
        )


def normalize_parameter_keys(conn: sqlite3.Connection) -> None:
    all_renames: dict[str, str] = {}
    for owner_id, renames in PARAMETER_RENAMES.items():
        all_renames.update(renames)
        row = load_component(conn, owner_id)
        sections = json.loads(row["extensionProps"])
        found: set[str] = set()
        for section in sections:
            if section.get("key") != "base_config":
                continue
            values = section.get("value", [])
            if not values or not isinstance(values[0], dict):
                continue
            for parameter in values[0].get("param", []):
                parameter_id = parameter.get("id")
                if parameter_id in renames:
                    old_name = parameter_id
                    new_name = renames[parameter_id]
                    parameter["id"] = new_name
                    parameter["name"] = new_name
                    found.add(old_name)
                    continue
                matching_old_names = [
                    old_name
                    for old_name, new_name in renames.items()
                    if parameter_id == new_name
                ]
                if len(matching_old_names) == 1:
                    old_name = matching_old_names[0]
                    parameter["name"] = renames[old_name]
                    found.add(old_name)
        if found != set(renames):
            raise RuntimeError(
                f"Parameter definitions differ for {owner_id}: "
                f"missing={sorted(set(renames)-found)}"
            )
        conn.execute(
            "UPDATE project_component SET extensionProps=? WHERE id=?",
            (json_text(sections), owner_id),
        )

    for row in descendants(conn, MAIN_ID):
        sections = json.loads(row["extensionProps"])
        changed = False
        for section in sections:
            if section.get("key") != "base_config":
                continue
            for value in section.get("value", []):
                if not isinstance(value, dict):
                    continue
                keys = value.get("operateKeys")
                if not isinstance(keys, list):
                    continue
                normalized = []
                for key in keys:
                    if not isinstance(key, str) or not key.startswith("param_."):
                        normalized.append(key)
                        continue
                    old_name = key.removeprefix("param_.")
                    normalized.append(f"param_.{all_renames.get(old_name, old_name)}")
                if normalized != keys:
                    value["operateKeys"] = normalized
                    changed = True
        if changed:
            conn.execute(
                "UPDATE project_component SET extensionProps=? WHERE id=?",
                (json_text(sections), row["id"]),
            )


def wrap_control(conn: sqlite3.Connection) -> str:
    current = components_by_alias(conn, CONTROL_ID)
    if TARGET_MODULE_ALIAS in current:
        raise RuntimeError("Control already contains TargetSpeedCalculation")
    missing = sorted(set(CALCULATION_ALIASES) - set(current))
    if missing:
        raise RuntimeError(f"Missing Control calculation blocks: {missing}")
    missing_boundaries = sorted((set(INPUT_TYPES) | {"targetSpeed"}) - set(current))
    if missing_boundaries:
        raise RuntimeError(f"Missing Control boundaries: {missing_boundaries}")

    calculation_ids = {current[alias]["id"] for alias in CALCULATION_ALIASES}
    input_boundary_ids = {
        name: current[name]["id"] for name in INPUT_TYPES
    }
    output_boundary_id = current["targetSpeed"]["id"]

    original_edges = [
        dict(row)
        for row in conn.execute(
            "SELECT * FROM project_connection WHERE parentId=?", (CONTROL_ID,)
        )
    ]
    internal_edges: list[dict[str, Any]] = []
    input_edges: dict[str, list[dict[str, Any]]] = {
        name: [] for name in INPUT_TYPES
    }
    output_edges: list[dict[str, Any]] = []
    for edge in original_edges:
        source = json.loads(edge["source"])
        target = json.loads(edge["target"])
        if source["cell"] in calculation_ids and target["cell"] in calculation_ids:
            internal_edges.append(edge)
            continue
        matched_input = next(
            (
                name
                for name, component_id in input_boundary_ids.items()
                if source["cell"] == component_id
            ),
            None,
        )
        if matched_input is not None and target["cell"] in calculation_ids:
            input_edges[matched_input].append(edge)
            continue
        if (
            source["cell"] in calculation_ids
            and target["cell"] == output_boundary_id
        ):
            output_edges.append(edge)
            continue
        raise RuntimeError(
            "Unexpected Control edge while wrapping: "
            f"{source['cell']}:{source['port']} -> "
            f"{target['cell']}:{target['port']}"
        )

    no_consumers = sorted(name for name, edges in input_edges.items() if not edges)
    if no_consumers:
        raise RuntimeError(f"Control inputs have no consumers: {no_consumers}")
    if len(output_edges) != 1:
        raise RuntimeError(
            f"Expected one targetSpeed output edge, found {len(output_edges)}"
        )

    composite_template = load_component(conn, CONTROL_ID)
    input_template = load_component(conn, INPUT_TEMPLATE_ID)
    output_template = load_component(conn, OUTPUT_TEMPLATE_ID)
    connection_template = dict(
        conn.execute("SELECT * FROM project_connection LIMIT 1").fetchone()
    )
    module_id, nested_input_ids, nested_output_id = create_target_speed_module(
        conn,
        composite_template,
        input_template,
        output_template,
    )

    for alias in CALCULATION_ALIASES:
        component_id = current[alias]["id"]
        conn.execute(
            "UPDATE project_component SET parentId=? WHERE id=?",
            (module_id, component_id),
        )
        update_position(
            conn,
            component_id,
            CONTROL_ID,
            module_id,
            CALCULATION_LAYOUT[alias],
        )

    for edge in internal_edges:
        conn.execute(
            "UPDATE project_connection SET parentId=? WHERE id=?",
            (module_id, edge["id"]),
        )
    crossing_ids = {
        edge["id"] for edges in input_edges.values() for edge in edges
    } | {edge["id"] for edge in output_edges}
    for edge_id in crossing_ids:
        conn.execute("DELETE FROM project_connection WHERE id=?", (edge_id,))

    for name, edges in input_edges.items():
        insert_connection(
            conn,
            connection_template,
            CONTROL_ID,
            {"cell": input_boundary_ids[name], "port": "out"},
            {"cell": module_id, "port": nested_input_ids[name]},
            f"control:targetSpeedCalculation:input:{name}",
        )
        for index, edge in enumerate(edges):
            insert_connection(
                conn,
                connection_template,
                module_id,
                {"cell": nested_input_ids[name], "port": "out"},
                json.loads(edge["target"]),
                f"targetSpeedCalculation:input:{name}:{index}",
            )

    original_output_source = json.loads(output_edges[0]["source"])
    insert_connection(
        conn,
        connection_template,
        module_id,
        original_output_source,
        {"cell": nested_output_id, "port": "in"},
        "targetSpeedCalculation:output:targetSpeed",
    )
    insert_connection(
        conn,
        connection_template,
        CONTROL_ID,
        {"cell": module_id, "port": nested_output_id},
        {"cell": output_boundary_id, "port": "in"},
        "control:targetSpeedCalculation:output:targetSpeed",
    )

    for alias, position in CONTROL_LAYOUT.items():
        if alias == TARGET_MODULE_ALIAS:
            update_position(
                conn, module_id, CONTROL_ID, CONTROL_ID, position, (430, 300)
            )
        else:
            update_position(
                conn,
                current[alias]["id"],
                CONTROL_ID,
                CONTROL_ID,
                position,
            )
    return module_id


def unwrap_legacy_target_module(
    conn: sqlite3.Connection, target_module_id: str
) -> None:
    control_children = components_by_alias(conn, CONTROL_ID)
    target_children = components_by_alias(conn, target_module_id)
    calculation_aliases = {
        "timeGapDistance",
        "desiredDistance",
        "distanceError",
        "relativeSpeed",
        "distanceFeedback",
        "speedFeedback",
        "rawTargetSpeed",
        "nonNegativeTargetSpeed",
        "targetSpeedLimiter",
        "enableGate",
        "zeroSpeed",
    }
    missing = sorted(calculation_aliases - set(target_children))
    if missing:
        raise RuntimeError(
            f"Legacy targetSpeedCalculation is missing blocks: {missing}"
        )

    calculation_ids = {
        target_children[alias]["id"] for alias in calculation_aliases
    }
    input_boundaries = {
        alias: row
        for alias, row in target_children.items()
        if row["componentType"] == "input"
    }
    output_boundaries = {
        alias: row
        for alias, row in target_children.items()
        if row["componentType"] == "output"
    }
    if set(input_boundaries) != set(INPUT_TYPES):
        raise RuntimeError(
            "Unexpected legacy targetSpeedCalculation inputs: "
            f"{sorted(input_boundaries)}"
        )
    if set(output_boundaries) != {"targetSpeed"}:
        raise RuntimeError(
            "Unexpected legacy targetSpeedCalculation outputs: "
            f"{sorted(output_boundaries)}"
        )

    outer_edges = [
        dict(row)
        for row in conn.execute(
            "SELECT * FROM project_connection WHERE parentId=?", (CONTROL_ID,)
        )
    ]
    module_outer_edges = []
    for edge in outer_edges:
        source = json.loads(edge["source"])
        target = json.loads(edge["target"])
        if source["cell"] == target_module_id or target["cell"] == target_module_id:
            module_outer_edges.append(edge)
        else:
            raise RuntimeError(
                "Control contains an unexpected edge beside the legacy wrapper"
            )
    if len(module_outer_edges) != len(INPUT_TYPES) + 1:
        raise RuntimeError(
            f"Unexpected legacy wrapper edge count: {len(module_outer_edges)}"
        )

    inner_edges = [
        dict(row)
        for row in conn.execute(
            "SELECT * FROM project_connection WHERE parentId=?",
            (target_module_id,),
        )
    ]
    input_id_to_alias = {
        row["id"]: alias for alias, row in input_boundaries.items()
    }
    output_id = output_boundaries["targetSpeed"]["id"]
    for edge in inner_edges:
        source = json.loads(edge["source"])
        target = json.loads(edge["target"])
        if source["cell"] in calculation_ids and target["cell"] in calculation_ids:
            conn.execute(
                "UPDATE project_connection SET parentId=? WHERE id=?",
                (CONTROL_ID, edge["id"]),
            )
            continue
        if source["cell"] in input_id_to_alias and target["cell"] in calculation_ids:
            signal_name = input_id_to_alias[source["cell"]]
            outer_source = {
                "cell": control_children[signal_name]["id"],
                "port": "out",
            }
            conn.execute(
                "UPDATE project_connection SET parentId=?,source=? WHERE id=?",
                (CONTROL_ID, json_text(outer_source), edge["id"]),
            )
            continue
        if source["cell"] in calculation_ids and target["cell"] == output_id:
            outer_target = {
                "cell": control_children["targetSpeed"]["id"],
                "port": "in",
            }
            conn.execute(
                "UPDATE project_connection SET parentId=?,target=? WHERE id=?",
                (CONTROL_ID, json_text(outer_target), edge["id"]),
            )
            continue
        raise RuntimeError(
            "Unexpected edge inside legacy targetSpeedCalculation: "
            f"{source['cell']} -> {target['cell']}"
        )

    for component_id in calculation_ids:
        conn.execute(
            "UPDATE project_component SET parentId=? WHERE id=?",
            (CONTROL_ID, component_id),
        )
    for edge in module_outer_edges:
        conn.execute("DELETE FROM project_connection WHERE id=?", (edge["id"],))
    boundary_ids = [
        row["id"] for row in (*input_boundaries.values(), *output_boundaries.values())
    ]
    placeholders = ",".join("?" for _ in boundary_ids)
    conn.execute(
        f"DELETE FROM project_component WHERE id IN ({placeholders})",
        boundary_ids,
    )
    conn.execute(
        "DELETE FROM project_component WHERE id=?", (target_module_id,)
    )

    stale = conn.execute(
        """
        SELECT COUNT(*) FROM project_connection
        WHERE json_extract(source,'$.cell')=?
           OR json_extract(target,'$.cell')=?
        """,
        (target_module_id, target_module_id),
    ).fetchone()[0]
    if stale:
        raise RuntimeError("Legacy targetSpeedCalculation still has connections")


def partition_control(conn: sqlite3.Connection) -> dict[str, str]:
    current = components_by_alias(conn, CONTROL_ID)
    all_members = {
        member for group in CONTROL_GROUPS for member in group.members
    }
    missing = sorted(
        (all_members | set(INPUT_TYPES) | {"targetSpeed"}) - set(current)
    )
    if missing:
        raise RuntimeError(f"Control is missing components before split: {missing}")

    alias_to_id = {alias: row["id"] for alias, row in current.items()}
    member_group: dict[str, str] = {}
    for group in CONTROL_GROUPS:
        for member in group.members:
            if member in member_group:
                raise RuntimeError(f"Control block assigned twice: {member}")
            member_group[member] = group.alias

    original_edges = [
        dict(row)
        for row in conn.execute(
            "SELECT * FROM project_connection WHERE parentId=?", (CONTROL_ID,)
        )
    ]
    signal_edges: dict[str, list[dict[str, Any]]] = {}
    expected_crossing_ids: set[str] = set()
    for signal in CONTROL_SIGNALS:
        edges = [
            find_edge(
                original_edges,
                alias_to_id[signal.source_alias],
                alias_to_id[target_alias],
            )
            for target_alias in signal.target_aliases
        ]
        signal_edges[signal.name] = edges
        source_group = member_group.get(signal.source_alias)
        for target_alias, edge in zip(signal.target_aliases, edges):
            if source_group != member_group.get(target_alias):
                expected_crossing_ids.add(edge["id"])

    actual_crossing_ids: set[str] = set()
    id_to_alias = {row["id"]: alias for alias, row in current.items()}
    for edge in original_edges:
        source = json.loads(edge["source"])
        target = json.loads(edge["target"])
        source_alias = id_to_alias[source["cell"]]
        target_alias = id_to_alias[target["cell"]]
        if member_group.get(source_alias) != member_group.get(target_alias):
            actual_crossing_ids.add(edge["id"])
    if actual_crossing_ids != expected_crossing_ids:
        raise RuntimeError(
            "Control split routes do not cover every crossing edge: "
            f"unhandled={sorted(actual_crossing_ids-expected_crossing_ids)}, "
            f"unexpected={sorted(expected_crossing_ids-actual_crossing_ids)}"
        )

    signal_types = {signal.name: signal.data_type for signal in CONTROL_SIGNALS}
    inputs_by_group = {group.alias: [] for group in CONTROL_GROUPS}
    outputs_by_group = {group.alias: [] for group in CONTROL_GROUPS}
    for signal in CONTROL_SIGNALS:
        source_group = member_group.get(signal.source_alias)
        target_groups = {
            member_group.get(target_alias) for target_alias in signal.target_aliases
        }
        if source_group is not None and any(
            target_group != source_group for target_group in target_groups
        ):
            outputs_by_group[source_group].append(signal.name)
        for target_group in target_groups:
            if target_group is not None and target_group != source_group:
                inputs_by_group[target_group].append(signal.name)

    composite_template = load_component(conn, CONTROL_ID)
    input_template = load_component(conn, INPUT_TEMPLATE_ID)
    output_template = load_component(conn, OUTPUT_TEMPLATE_ID)
    connection_template = dict(
        conn.execute("SELECT * FROM project_connection LIMIT 1").fetchone()
    )
    created: dict[
        str, tuple[str, dict[str, str], dict[str, str]]
    ] = {}
    for group in CONTROL_GROUPS:
        created[group.alias] = create_control_group(
            conn,
            group,
            inputs_by_group[group.alias],
            outputs_by_group[group.alias],
            signal_types,
            composite_template,
            input_template,
            output_template,
        )

    group_ids = {alias: values[0] for alias, values in created.items()}
    for group in CONTROL_GROUPS:
        group_id = group_ids[group.alias]
        for member in group.members:
            component_id = alias_to_id[member]
            conn.execute(
                "UPDATE project_component SET parentId=? WHERE id=?",
                (group_id, component_id),
            )
            update_position(
                conn,
                component_id,
                CONTROL_ID,
                group_id,
                group.internal_positions[member],
            )

    for edge in original_edges:
        source = json.loads(edge["source"])
        target = json.loads(edge["target"])
        source_group = member_group.get(id_to_alias[source["cell"]])
        target_group = member_group.get(id_to_alias[target["cell"]])
        if source_group == target_group and source_group is not None:
            conn.execute(
                "UPDATE project_connection SET parentId=? WHERE id=?",
                (group_ids[source_group], edge["id"]),
            )
        elif source_group != target_group:
            conn.execute("DELETE FROM project_connection WHERE id=?", (edge["id"],))

    for signal in CONTROL_SIGNALS:
        edges = signal_edges[signal.name]
        source_group = member_group.get(signal.source_alias)
        crossing_edges = [
            edge
            for target_alias, edge in zip(signal.target_aliases, edges)
            if member_group.get(target_alias) != source_group
        ]
        if not crossing_edges:
            continue
        original_source = json.loads(crossing_edges[0]["source"])
        if any(json.loads(edge["source"]) != original_source for edge in crossing_edges):
            raise RuntimeError(f"Signal source mismatch: {signal.name}")

        if source_group is None:
            top_source = original_source
        else:
            source_module_id, _, source_outputs = created[source_group]
            insert_connection(
                conn,
                connection_template,
                source_module_id,
                original_source,
                {"cell": source_outputs[signal.name], "port": "in"},
                f"inside:{source_group}:output:{signal.name}",
            )
            top_source = {
                "cell": source_module_id,
                "port": source_outputs[signal.name],
            }

        edges_by_target_group: dict[str | None, list[dict[str, Any]]] = defaultdict(list)
        for target_alias, edge in zip(signal.target_aliases, edges):
            target_group = member_group.get(target_alias)
            if target_group != source_group:
                edges_by_target_group[target_group].append(edge)

        for target_group, target_edges in edges_by_target_group.items():
            if target_group is None:
                for index, edge in enumerate(target_edges):
                    insert_connection(
                        conn,
                        connection_template,
                        CONTROL_ID,
                        top_source,
                        json.loads(edge["target"]),
                        f"top:{signal.name}:output:{index}",
                    )
                continue
            target_module_id, target_inputs, _ = created[target_group]
            insert_connection(
                conn,
                connection_template,
                CONTROL_ID,
                top_source,
                {
                    "cell": target_module_id,
                    "port": target_inputs[signal.name],
                },
                f"top:{signal.name}:input:{target_group}",
            )
            for index, edge in enumerate(target_edges):
                insert_connection(
                    conn,
                    connection_template,
                    target_module_id,
                    {"cell": target_inputs[signal.name], "port": "out"},
                    json.loads(edge["target"]),
                    f"inside:{target_group}:input:{signal.name}:{index}",
                )

    final_children = components_by_alias(conn, CONTROL_ID)
    for alias, position in FINAL_CONTROL_LAYOUT.items():
        if alias in group_ids:
            group = next(item for item in CONTROL_GROUPS if item.alias == alias)
            update_position(
                conn,
                group_ids[alias],
                CONTROL_ID,
                CONTROL_ID,
                position,
                group.size,
            )
        else:
            update_position(
                conn,
                final_children[alias]["id"],
                CONTROL_ID,
                CONTROL_ID,
                position,
            )
    return group_ids


def validate(conn: sqlite3.Connection, group_ids: dict[str, str]) -> None:
    integrity = conn.execute("PRAGMA integrity_check").fetchone()[0]
    if integrity != "ok":
        raise RuntimeError(f"SQLite integrity check failed: {integrity}")

    dangling = conn.execute(
        """
        SELECT COUNT(*)
        FROM project_connection connection
        LEFT JOIN project_component source
          ON source.id=json_extract(connection.source, '$.cell')
        LEFT JOIN project_component target
          ON target.id=json_extract(connection.target, '$.cell')
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
          ON source.id=json_extract(connection.source, '$.cell')
        JOIN project_component target
          ON target.id=json_extract(connection.target, '$.cell')
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

    expected_structures = dict(STRUCTURAL_ALIASES)
    expected_structures.update(
        {component_id: alias for alias, component_id in group_ids.items()}
    )
    for component_id, expected_alias in expected_structures.items():
        row = load_component(conn, component_id)
        if row["alias"] != expected_alias:
            raise RuntimeError(
                f"Structure alias mismatch: {row['alias']} != {expected_alias}"
            )
        if json.loads(row["properties"]).get("alias") != expected_alias:
            raise RuntimeError(
                f"Structure properties.alias mismatch: {expected_alias}"
            )

    control_children = components_by_alias(conn, CONTROL_ID)
    expected_control = (
        set(INPUT_TYPES) | {"targetSpeed"} | set(group_ids)
    )
    if set(control_children) != expected_control:
        raise RuntimeError(
            "Unexpected control children: "
            f"missing={sorted(expected_control-set(control_children))}, "
            f"extra={sorted(set(control_children)-expected_control)}"
        )

    expected_group_ports = {
        "spacingFeedback": {
            "inputs": {"egoV", "timeGap", "minDistance", "distance", "kDist"},
            "outputs": {"distanceFeedback"},
        },
        "relativeSpeedFeedback": {
            "inputs": {"egoV", "leadV", "kSpeed"},
            "outputs": {"speedFeedback"},
        },
        "targetSpeedOutput": {
            "inputs": {
                "leadV",
                "distanceFeedback",
                "speedFeedback",
                "maxSpeed",
                "enable",
            },
            "outputs": {"targetSpeed"},
        },
    }
    group_specs = {spec.alias: spec for spec in CONTROL_GROUPS}
    for group_alias, group_id in group_ids.items():
        rows = list(
            conn.execute(
                "SELECT * FROM project_component WHERE parentId=?", (group_id,)
            )
        )
        actual_members = {
            row["alias"]
            for row in rows
            if row["componentType"] not in {"input", "output"}
        }
        expected_members = set(group_specs[group_alias].members)
        if actual_members != expected_members:
            raise RuntimeError(
                f"Unexpected {group_alias} calculations: "
                f"missing={sorted(expected_members-actual_members)}, "
                f"extra={sorted(actual_members-expected_members)}"
            )
        actual_inputs = {
            boundary_base_name(row)
            for row in rows
            if row["componentType"] == "input"
        }
        actual_outputs = {
            boundary_base_name(row)
            for row in rows
            if row["componentType"] == "output"
        }
        expected_ports = expected_group_ports[group_alias]
        if actual_inputs != expected_ports["inputs"]:
            raise RuntimeError(
                f"Unexpected {group_alias} inputs: {sorted(actual_inputs)}"
            )
        if actual_outputs != expected_ports["outputs"]:
            raise RuntimeError(
                f"Unexpected {group_alias} outputs: {sorted(actual_outputs)}"
            )

    legacy_target_id = stable_id(
        "newaccpro3:control:targetSpeedCalculation"
    )
    legacy_target = conn.execute(
        "SELECT COUNT(*) FROM project_component WHERE id=?",
        (legacy_target_id,),
    ).fetchone()[0]
    if legacy_target:
        raise RuntimeError("Legacy targetSpeedCalculation still exists")

    for component_id, instance_name in {
        **STRUCTURAL_INSTANCE_NAMES,
        **{
            component_id: alias
            for alias, component_id in group_ids.items()
        },
    }.items():
        row = load_component(conn, component_id)
        sections = json.loads(row["extensionProps"])
        actual = None
        for section in sections:
            if section.get("key") == "base_config":
                values = section.get("value", [])
                if values and isinstance(values[0], dict):
                    actual = values[0].get("instanceName")
        if actual != instance_name:
            raise RuntimeError(
                f"Unexpected instanceName for {row['alias']}: {actual}"
            )

    for row in descendants(conn, MAIN_ID):
        alias = row["alias"]
        if not alias or not alias[0].isascii():
            continue
        if not ("a" <= alias[0] <= "z") or "_" in alias:
            raise RuntimeError(
                f"Non-lower-camel component alias remains: {alias}"
            )


def ensure_legacy_target_module(conn: sqlite3.Connection) -> str:
    target_module_id = stable_id(
        "newaccpro3:control:targetSpeedCalculation"
    )
    row = conn.execute(
        "SELECT * FROM project_component WHERE id=?", (target_module_id,)
    ).fetchone()
    if row is not None:
        if row["parentId"] != CONTROL_ID:
            raise RuntimeError(
                "Legacy targetSpeedCalculation has an unexpected parent"
            )
        return target_module_id

    current = components_by_alias(conn, CONTROL_ID)
    if TARGET_MODULE_ALIAS in current:
        return current[TARGET_MODULE_ALIAS]["id"]
    if set(group.alias for group in CONTROL_GROUPS) & set(current):
        raise RuntimeError("Control has already been split into functional groups")
    return wrap_control(conn)


def refactor(db_path: Path) -> tuple[int, int, dict[str, str]]:
    conn = sqlite3.connect(db_path)
    conn.row_factory = sqlite3.Row
    conn.execute("PRAGMA foreign_keys=OFF")
    try:
        before_signature = flattened_signature(conn)
        with conn:
            main = load_component(conn, MAIN_ID)
            if main["alias"] not in {"Main", "MainFlow", "main"}:
                raise RuntimeError(f"Unexpected main alias: {main['alias']}")
            control_children = components_by_alias(conn, CONTROL_ID)
            expected_group_aliases = {
                group.alias for group in CONTROL_GROUPS
            }
            existing_group_aliases = (
                expected_group_aliases & set(control_children)
            )
            if existing_group_aliases:
                if existing_group_aliases != expected_group_aliases:
                    raise RuntimeError(
                        "Control contains only part of the functional groups: "
                        f"{sorted(existing_group_aliases)}"
                    )
                group_ids = {
                    alias: control_children[alias]["id"]
                    for alias in expected_group_aliases
                }
                normalize_parameter_keys(conn)
                normalize_aliases(conn, "")
            else:
                target_module_id = ensure_legacy_target_module(conn)
                normalize_parameter_keys(conn)
                normalize_aliases(conn, target_module_id)
                unwrap_legacy_target_module(conn, target_module_id)
                group_ids = partition_control(conn)
            normalize_structure_names(conn, group_ids)
            validate(conn, group_ids)
            after_signature = flattened_signature(conn)
            if before_signature != after_signature:
                before_set = set(before_signature)
                after_set = set(after_signature)
                raise RuntimeError(
                    "Flattened behavior changed: "
                    f"missing={sorted(before_set-after_set)[:10]}, "
                    f"extra={sorted(after_set-before_set)[:10]}"
                )
        return len(before_signature), len(after_signature), group_ids
    finally:
        conn.close()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--project",
        type=Path,
        default=Path("project/newaccpro3"),
        help="Path to the GAASD project directory",
    )
    args = parser.parse_args()

    project = args.project.expanduser().resolve()
    cbdes = project / "data" / "cbdes.db"
    temp = project / "data" / "temp.db"
    if not cbdes.is_file() or not temp.is_file():
        raise RuntimeError(f"Missing GAASD databases under {project / 'data'}")
    if sha256(cbdes) != sha256(temp):
        raise RuntimeError("cbdes.db and temp.db differ before refactoring")

    backup = cbdes.with_name("cbdes.db.before_control_function_split_20260731")
    if not backup.exists():
        shutil.copy2(cbdes, backup)

    before_count, after_count, group_ids = refactor(cbdes)
    shutil.copy2(cbdes, temp)
    if sha256(cbdes) != sha256(temp):
        raise RuntimeError("cbdes.db and temp.db differ after synchronization")

    print(f"Refactored:    {cbdes}")
    print(f"Backup:        {backup}")
    print(
        "Control groups: "
        + ", ".join(f"{alias}={component_id}" for alias, component_id in group_ids.items())
    )
    print(f"Flat edges:    {before_count} -> {after_count}")
    print(f"SHA256:        {sha256(cbdes)}")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(1)
