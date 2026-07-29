#!/usr/bin/env python3
"""Refactor the lks2 canvas hierarchy without changing the flattened algorithm."""

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


ROOT_ID = "6d9ae1e1-3798-4640-91c6-8217a06e02aa"
DECISION_ID = "35bfe571-0409-485f-b070-0999518ce7bc"
PREVIEW_ID = "aa61974c-e982-4ef6-b6c2-e61dba56487d"
FUSION_ID = "408bc885-5a8d-48df-ac8b-d2cc40f1e9d6"
STEER_ID = "14f902c4-eeee-4130-bb60-0e0218c9aea7"
FINAL_GATE_ID = "0058a2ec-063b-4158-b8b6-4f089494b7ab"
SCOPE_ID = "8857fb05-55d4-4a65-aa51-ac2c5b1dfdf1"
ROOT_STEER_OUTPUT_ID = "914b0cf8-20fd-5c01-951d-8137ac40dee3"
DECISION_OUTPUT_PORT = "f250fbf3-cf44-48fa-8924-b130a2d18e7b"
STEER_OUTPUT_PORT = "e9ec6429-b6b0-403a-9729-2ab706341058"

NAMESPACE = uuid.UUID("2e8f2ae1-1fb2-46ba-bd9a-c0697644f2a1")


@dataclass(frozen=True)
class TargetRef:
    alias: str
    port: str | None = None


@dataclass(frozen=True)
class SignalSpec:
    name: str
    data_type: str
    source_alias: str
    targets: tuple[TargetRef, ...]


@dataclass(frozen=True)
class GroupSpec:
    alias: str
    description: str
    instance_name: str
    position: tuple[int, int]
    size: tuple[int, int]
    members: tuple[str, ...]
    renames: dict[str, str]
    internal_positions: dict[str, tuple[int, int]]


DECISION_GROUPS = (
    GroupSpec(
        alias="SpeedEnableCheck",
        description="根据自车速度和最低工作速度判断LKS速度条件是否满足。",
        instance_name="compositeBlockInstance_20",
        position=(-820, -330),
        size=(300, 190),
        members=("GE_SpeedOk", "NOT_SpeedOk", "lks_vMin"),
        renames={
            "GE_SpeedOk": "IsSpeedAboveMinimum",
            "NOT_SpeedOk": "IsSpeedUnavailable",
            "lks_vMin": "MinimumOperatingSpeed",
        },
        internal_positions={
            "lks_vMin": (-650, 150),
            "GE_SpeedOk": (-280, -40),
            "NOT_SpeedOk": (160, -40),
        },
    ),
    GroupSpec(
        alias="DriverOverrideCheck",
        description="根据驾驶员方向盘输入判断是否发生主动转向接管。",
        instance_name="compositeBlockInstance_21",
        position=(-820, 30),
        size=(300, 190),
        members=("Fabs_Steer", "GE_SteerHigh", "lks_driverSteerThreshold"),
        renames={
            "Fabs_Steer": "DriverSteerMagnitude",
            "GE_SteerHigh": "IsDriverOverride",
            "lks_driverSteerThreshold": "DriverOverrideThreshold",
        },
        internal_positions={
            "Fabs_Steer": (-420, -50),
            "lks_driverSteerThreshold": (-420, 170),
            "GE_SteerHigh": (40, -20),
        },
    ),
    GroupSpec(
        alias="ControlEnableDecision",
        description="汇总制动、低速和驾驶员主动转向条件，输出LKS控制使能。",
        instance_name="compositeBlockInstance_22",
        position=(-60, -170),
        size=(330, 230),
        members=("OR_Exit", "NOT_Exit"),
        renames={
            "OR_Exit": "HasDisableCondition",
            "NOT_Exit": "EnableLksControl",
        },
        internal_positions={
            "OR_Exit": (-200, -20),
            "NOT_Exit": (260, -20),
        },
    ),
)

DECISION_SIGNALS = (
    SignalSpec("egoV", "double", "egoV", (TargetRef("GE_SpeedOk"),)),
    SignalSpec(
        "speedUnavailable",
        "bool",
        "NOT_SpeedOk",
        (TargetRef("OR_Exit", "Component_merge_input_3"),),
    ),
    SignalSpec(
        "driverSteerNorm", "double", "driverSteerNorm", (TargetRef("Fabs_Steer"),)
    ),
    SignalSpec(
        "driverOverride",
        "bool",
        "GE_SteerHigh",
        (TargetRef("OR_Exit", "Component_merge_input_2"),),
    ),
    SignalSpec(
        "brakePressed",
        "bool",
        "brakePressed",
        (TargetRef("OR_Exit", "Component_merge_input_1"),),
    ),
    SignalSpec(
        "controlEnabled", "bool", "NOT_Exit", (TargetRef("controlEnabled"),)
    ),
)

LANE_GROUP = GroupSpec(
    alias="LaneErrorEvaluation",
    description="计算近、中、远三个预瞄点处的三次车道多项式横向误差。",
    instance_name="compositeBlockInstance_23",
    position=(-100, -240),
    size=(360, 260),
    members=("Eval_P1", "Eval_P2", "Eval_P3", "X1_near", "X2_mid", "常量"),
    renames={
        "Eval_P1": "NearPreviewError",
        "Eval_P2": "MiddlePreviewError",
        "Eval_P3": "FarPreviewError",
        "X1_near": "NearPreviewDistance",
        "X2_mid": "MiddlePreviewDistance",
        "常量": "PreviewDistanceDivisor",
    },
    internal_positions={
        "X1_near": (-760, -470),
        "常量": (-760, -180),
        "X2_mid": (-420, -260),
        "Eval_P1": (-40, -480),
        "Eval_P2": (-40, -90),
        "Eval_P3": (-40, 300),
    },
)

LANE_SIGNALS = (
    SignalSpec(
        "c0", "double", "c0", (TargetRef("Eval_P1"), TargetRef("Eval_P2"), TargetRef("Eval_P3"))
    ),
    SignalSpec(
        "c1", "double", "c1", (TargetRef("Eval_P1"), TargetRef("Eval_P2"), TargetRef("Eval_P3"))
    ),
    SignalSpec(
        "c2", "double", "c2", (TargetRef("Eval_P1"), TargetRef("Eval_P2"), TargetRef("Eval_P3"))
    ),
    SignalSpec(
        "c3", "double", "c3", (TargetRef("Eval_P1"), TargetRef("Eval_P2"), TargetRef("Eval_P3"))
    ),
    SignalSpec(
        "previewDistance",
        "double",
        "LKSPreviewDistance",
        (TargetRef("X2_mid"), TargetRef("Eval_P3")),
    ),
    SignalSpec("nearError", "double", "Eval_P1", (TargetRef("LKSErrorFusion"),)),
    SignalSpec("middleError", "double", "Eval_P2", (TargetRef("LKSErrorFusion"),)),
    SignalSpec("farError", "double", "Eval_P3", (TargetRef("LKSErrorFusion"),)),
)

STEER_GROUPS = (
    GroupSpec(
        alias="RawSteerCalculation",
        description="根据加权横向误差和比例增益计算未限幅归一化转向指令。",
        instance_name="compositeBlockInstance_24",
        position=(-820, -360),
        size=(310, 190),
        members=("Kp", "乘法运算"),
        renames={"Kp": "SteerErrorGain", "乘法运算": "RawSteerNorm"},
        internal_positions={"Kp": (-520, 120), "乘法运算": (-80, -20)},
    ),
    GroupSpec(
        alias="LateralAccelLimit",
        description="根据车速、轴距和横向加速度上限限制归一化转向指令。",
        instance_name="compositeBlockInstance_25",
        position=(-120, -250),
        size=(380, 300),
        members=(
            "ArgDiv",
            "AtanPhi",
            "FmaxLo",
            "FminUp",
            "NegLimit",
            "Numer",
            "ThetaLimit",
            "VSq",
            "VSqSafe",
            "lks_ayMax",
            "lks_frontWheelMaxRad",
            "lks_wheelBase",
            "常量",
        ),
        renames={
            "ArgDiv": "LateralAccelRatio",
            "AtanPhi": "SteerAngleLimitRad",
            "FmaxLo": "ApplyLowerSteerLimit",
            "FminUp": "ApplyUpperSteerLimit",
            "NegLimit": "NegativeSteerLimit",
            "Numer": "MaxLateralAccelMoment",
            "ThetaLimit": "NormalizedSteerLimit",
            "VSq": "SpeedSquare",
            "VSqSafe": "SafeSpeedSquare",
            "lks_ayMax": "MaxLateralAcceleration",
            "lks_frontWheelMaxRad": "MaxFrontWheelAngle",
            "lks_wheelBase": "WheelBase",
            "常量": "MinimumSpeedSquare",
        },
        internal_positions={
            "VSq": (-900, -380),
            "常量": (-900, -120),
            "VSqSafe": (-570, -280),
            "lks_wheelBase": (-900, 140),
            "lks_ayMax": (-900, 360),
            "Numer": (-560, 250),
            "ArgDiv": (-200, 20),
            "AtanPhi": (120, 20),
            "lks_frontWheelMaxRad": (80, 280),
            "ThetaLimit": (430, 40),
            "NegLimit": (690, 250),
            "FminUp": (700, -170),
            "FmaxLo": (970, -40),
        },
    ),
    GroupSpec(
        alias="SteerEnableGate",
        description="将限幅后的归一化转向量换算为弧度，并按LKS使能门控输出。",
        instance_name="compositeBlockInstance_26",
        position=(700, -170),
        size=(330, 230),
        members=("scale", "乘法运算_1", "lksSteerRad"),
        renames={
            "scale": "SteerScaleRad",
            "乘法运算_1": "ScaleSteerToRad",
            "lksSteerRad": "ApplyControlEnable",
        },
        internal_positions={
            "scale": (-520, 160),
            "乘法运算_1": (-100, -20),
            "lksSteerRad": (380, -20),
        },
    ),
)

STEER_SIGNALS = (
    SignalSpec(
        "weightedError", "double", "weightedError", (TargetRef("乘法运算"),)
    ),
    SignalSpec(
        "rawSteerNorm", "double", "乘法运算", (TargetRef("FminUp", "fmin_input_x_In"),)
    ),
    SignalSpec(
        "egoV",
        "double",
        "egoV",
        (
            TargetRef("VSq", "multiply_input_a_In"),
            TargetRef("VSq", "multiply_input_b_In"),
        ),
    ),
    SignalSpec(
        "limitedSteerNorm",
        "double",
        "FmaxLo",
        (TargetRef("乘法运算_1", "multiply_input_a_In"),),
    ),
    SignalSpec(
        "controlEnabled",
        "bool",
        "controlEnabled",
        (TargetRef("lksSteerRad", "multiply_input_b_In"),),
    ),
    SignalSpec(
        "steerRad", "double", "lksSteerRad", (TargetRef("steerRad"),)
    ),
)

CONTROL_GROUP = GroupSpec(
    alias="Control",
    description="完成预瞄距离、三点车道误差、误差加权和方向盘转角命令计算。",
    instance_name="compositeBlockInstance_27",
    position=(-120, -250),
    size=(500, 360),
    members=("LKSPreviewDistance", "LaneErrorEvaluation", "LKSErrorFusion", "LKSSteerControl"),
    renames={
        "LKSPreviewDistance": "PreviewDistance",
        "LKSErrorFusion": "ErrorWeightedSum",
        "LKSSteerControl": "SteerCommand",
    },
    internal_positions={
        "LKSPreviewDistance": (-850, -420),
        "LaneErrorEvaluation": (-330, -300),
        "LKSErrorFusion": (250, -250),
        "LKSSteerControl": (780, -180),
    },
)

CONTROL_SIGNALS = (
    SignalSpec(
        "egoV",
        "double",
        "egoV",
        (TargetRef("LKSPreviewDistance"), TargetRef("LKSSteerControl")),
    ),
    SignalSpec("c0", "double", "c0", (TargetRef("LaneErrorEvaluation"),)),
    SignalSpec("c1", "double", "c1", (TargetRef("LaneErrorEvaluation"),)),
    SignalSpec("c2", "double", "c2", (TargetRef("LaneErrorEvaluation"),)),
    SignalSpec("c3", "double", "c3", (TargetRef("LaneErrorEvaluation"),)),
    SignalSpec(
        "curvature", "double", "curvature", (TargetRef("LKSPreviewDistance"),)
    ),
    SignalSpec(
        "controlEnabled", "bool", "LKSDecision", (TargetRef("LKSSteerControl"),)
    ),
    SignalSpec(
        "steerRad",
        "double",
        "LKSSteerControl",
        (TargetRef("steerRad"), TargetRef("仿真示波器", "input_port_1")),
    ),
    SignalSpec(
        "previewDistance",
        "double",
        "LKSPreviewDistance",
        (TargetRef("previewDistance"),),
    ),
    SignalSpec(
        "weightedError",
        "double",
        "LKSErrorFusion",
        (TargetRef("weightedError"),),
    ),
)


def stable_id(label: str) -> str:
    return str(uuid.uuid5(NAMESPACE, label))


def core_endpoint(value: dict[str, Any]) -> dict[str, str]:
    return {"cell": value["cell"], "port": value["port"]}


def json_text(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, separators=(",", ":"))


def parse_sections(text: str) -> tuple[list[dict[str, Any]], dict[str, dict[str, Any]]]:
    sections = json.loads(text or "[]")
    return sections, {item["key"]: item for item in sections}


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
        "alias": name,
        "direction": direction,
        "description": "函数输入端口" if direction == "In" else "函数输出端口",
        "props": port_props(name, data_type),
    }


def load_row(conn: sqlite3.Connection, component_id: str) -> dict[str, Any]:
    row = conn.execute("SELECT * FROM project_component WHERE id=?", (component_id,)).fetchone()
    if row is None:
        raise RuntimeError(f"Missing component: {component_id}")
    return dict(row)


def components_by_alias(conn: sqlite3.Connection, parent_id: str) -> dict[str, dict[str, Any]]:
    result: dict[str, dict[str, Any]] = {}
    for row in conn.execute("SELECT * FROM project_component WHERE parentId=?", (parent_id,)):
        item = dict(row)
        if item["alias"] in result:
            raise RuntimeError(f"Duplicate alias under {parent_id}: {item['alias']}")
        result[item["alias"]] = item
    return result


def insert_component(conn: sqlite3.Connection, row: dict[str, Any]) -> None:
    columns = list(row)
    conn.execute(
        f"INSERT INTO project_component ({','.join(columns)}) VALUES ({','.join('?' for _ in columns)})",
        [row[column] for column in columns],
    )


def update_alias(
    conn: sqlite3.Connection, component_id: str, alias: str, description: str | None = None
) -> None:
    row = load_row(conn, component_id)
    properties = json.loads(row["properties"] or "{}")
    properties["alias"] = alias
    if description is not None:
        properties["description"] = description
    conn.execute(
        "UPDATE project_component SET alias=?, description=coalesce(?,description), properties=? WHERE id=?",
        (alias, description, json_text(properties), component_id),
    )


def update_position(
    conn: sqlite3.Connection,
    component_id: str,
    old_parent: str,
    new_parent: str,
    position: tuple[int, int],
    size: tuple[int, int] | None = None,
) -> None:
    row = load_row(conn, component_id)
    attributes = json.loads(row["attribute"] or "{}")
    if not isinstance(attributes, dict):
        attributes = {}
    layout = copy.deepcopy(attributes.get(old_parent, {}))
    if not layout and attributes:
        layout = copy.deepcopy(next(iter(attributes.values())))
    layout.setdefault("size", {"width": 180, "height": 92})
    if size is not None:
        layout["size"] = {"width": size[0], "height": size[1]}
    layout["position"] = {"x": position[0], "y": position[1]}
    layout.setdefault("zIndex", 40)
    conn.execute(
        "UPDATE project_component SET attribute=? WHERE id=?",
        (json_text({new_parent: layout}), component_id),
    )


def configure_boundary_row(
    row: dict[str, Any], alias: str, data_type: str, is_input: bool
) -> None:
    row["alias"] = alias
    properties = json.loads(row["properties"] or "{}")
    properties["alias"] = alias
    row["properties"] = json_text(properties)
    base = {
        "isReturnFlag": False,
        "name": alias,
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
            "props": port_props(alias, data_type),
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
            "props": port_props(alias, data_type),
        }
        extension = [
            {"key": "base_config", "value": [base]},
            {"key": "function_input_ports", "value": [input_port]},
            {"key": "function_output_ports", "value": []},
        ]
    row["extensionProps"] = json_text(extension)


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
    row["childComponents"] = "[]"
    row["childConnections"] = "[]"
    configure_boundary_row(row, name, data_type, is_input)
    row["attribute"] = json_text(
        {
            parent_id: {
                "size": {"width": max(132, 64 + len(name) * 8), "height": 68},
                "position": {"x": position[0], "y": position[1]},
                "zIndex": 50,
            }
        }
    )
    return row


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
    conn.execute(
        f"INSERT INTO project_connection ({','.join(columns)}) VALUES ({','.join('?' for _ in columns)})",
        [row[column] for column in columns],
    )


def find_edge(
    edges: list[dict[str, Any]],
    source_id: str,
    target_id: str,
    target_port: str | None,
) -> dict[str, Any]:
    matches = []
    for edge in edges:
        source = json.loads(edge["source"])
        target = json.loads(edge["target"])
        if source["cell"] != source_id or target["cell"] != target_id:
            continue
        if target_port is not None and target["port"] != target_port:
            continue
        matches.append(edge)
    if len(matches) != 1:
        raise RuntimeError(
            f"Expected one edge {source_id} -> {target_id}:{target_port}, found {len(matches)}"
        )
    return matches[0]


def create_group_module(
    conn: sqlite3.Connection,
    parent_id: str,
    spec: GroupSpec,
    input_ports: dict[str, str],
    output_ports: dict[str, str],
    signal_types: dict[str, str],
    composite_template: dict[str, Any],
    input_template: dict[str, Any],
    output_template: dict[str, Any],
) -> tuple[str, dict[str, str], dict[str, str]]:
    module_id = stable_id(f"module:{parent_id}:{spec.alias}")
    module = copy.deepcopy(composite_template)
    module["id"] = module_id
    module["parentId"] = parent_id
    module["instanceId"] = ""
    module["alias"] = spec.alias
    module["description"] = spec.description
    module["childComponents"] = "[]"
    module["childConnections"] = "[]"
    properties = json.loads(module["properties"] or "{}")
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
                    composite_port(port_id, name, signal_types[name], "In")
                    for name, port_id in input_ports.items()
                ],
            },
            {
                "key": "function_output_ports",
                "value": [
                    composite_port(port_id, name, signal_types[name], "Out")
                    for name, port_id in output_ports.items()
                ],
            },
        ]
    )
    module["attribute"] = json_text(
        {
            parent_id: {
                "size": {"width": spec.size[0], "height": spec.size[1]},
                "position": {"x": spec.position[0], "y": spec.position[1]},
                "zIndex": 40,
            }
        }
    )
    insert_component(conn, module)

    for index, (name, port_id) in enumerate(input_ports.items()):
        node = create_boundary_node(
            input_template,
            port_id,
            module_id,
            name,
            signal_types[name],
            (-1080, -430 + index * 160),
            True,
        )
        insert_component(conn, node)
    for index, (name, port_id) in enumerate(output_ports.items()):
        node = create_boundary_node(
            output_template,
            port_id,
            module_id,
            name,
            signal_types[name],
            (1180, -280 + index * 190),
            False,
        )
        insert_component(conn, node)
    return module_id, input_ports, output_ports


def partition_parent(
    conn: sqlite3.Connection,
    parent_id: str,
    groups: tuple[GroupSpec, ...],
    signals: tuple[SignalSpec, ...],
    composite_template: dict[str, Any],
    input_template: dict[str, Any],
    output_template: dict[str, Any],
    connection_template: dict[str, Any],
) -> dict[str, str]:
    current = components_by_alias(conn, parent_id)
    if any(group.alias in current for group in groups):
        raise RuntimeError(f"Parent {parent_id} is already partitioned")

    required = {member for group in groups for member in group.members}
    required.update(signal.source_alias for signal in signals)
    required.update(target.alias for signal in signals for target in signal.targets)
    missing = sorted(required - set(current))
    if missing:
        raise RuntimeError(f"Missing expected components under {parent_id}: {missing}")

    original_edges = [
        dict(row)
        for row in conn.execute(
            "SELECT * FROM project_connection WHERE parentId=?", (parent_id,)
        )
    ]
    alias_to_id = {alias: row["id"] for alias, row in current.items()}
    id_to_alias = {row["id"]: alias for alias, row in current.items()}
    group_for_alias = {
        member: group.alias for group in groups for member in group.members
    }

    expected_crossing: set[str] = set()
    signal_edges: dict[str, list[dict[str, Any]]] = {}
    for signal in signals:
        source_id = alias_to_id[signal.source_alias]
        edges = []
        for target in signal.targets:
            edge = find_edge(
                original_edges,
                source_id,
                alias_to_id[target.alias],
                target.port,
            )
            edges.append(edge)
            expected_crossing.add(edge["id"])
        signal_edges[signal.name] = edges

    actual_crossing: set[str] = set()
    for edge in original_edges:
        source = json.loads(edge["source"])
        target = json.loads(edge["target"])
        source_alias = id_to_alias.get(source["cell"])
        target_alias = id_to_alias.get(target["cell"])
        source_group = group_for_alias.get(source_alias)
        target_group = group_for_alias.get(target_alias)
        if source_group != target_group:
            actual_crossing.add(edge["id"])
    if actual_crossing != expected_crossing:
        raise RuntimeError(
            f"Crossing edge specification mismatch under {parent_id}: "
            f"unhandled={sorted(actual_crossing - expected_crossing)}, "
            f"unexpected={sorted(expected_crossing - actual_crossing)}"
        )

    signal_types = {signal.name: signal.data_type for signal in signals}
    created: dict[str, str] = {}
    interfaces: dict[str, tuple[dict[str, str], dict[str, str]]] = {}
    for group in groups:
        inputs: dict[str, str] = {}
        outputs: dict[str, str] = {}
        for signal in signals:
            source_group = group_for_alias.get(signal.source_alias)
            target_groups = {group_for_alias.get(target.alias) for target in signal.targets}
            if group.alias in target_groups and source_group != group.alias:
                inputs[signal.name] = stable_id(
                    f"module:{parent_id}:{group.alias}:input:{signal.name}"
                )
            if source_group == group.alias and any(
                target_group != group.alias for target_group in target_groups
            ):
                outputs[signal.name] = stable_id(
                    f"module:{parent_id}:{group.alias}:output:{signal.name}"
                )
        module_id, input_ids, output_ids = create_group_module(
            conn,
            parent_id,
            group,
            inputs,
            outputs,
            signal_types,
            composite_template,
            input_template,
            output_template,
        )
        created[group.alias] = module_id
        interfaces[group.alias] = (input_ids, output_ids)

    for group in groups:
        module_id = created[group.alias]
        for member in group.members:
            component_id = alias_to_id[member]
            conn.execute(
                "UPDATE project_component SET parentId=? WHERE id=?",
                (module_id, component_id),
            )
            update_position(
                conn,
                component_id,
                parent_id,
                module_id,
                group.internal_positions[member],
            )

    for edge in original_edges:
        source = json.loads(edge["source"])
        target = json.loads(edge["target"])
        source_alias = id_to_alias[source["cell"]]
        target_alias = id_to_alias[target["cell"]]
        source_group = group_for_alias.get(source_alias)
        target_group = group_for_alias.get(target_alias)
        if source_group == target_group and source_group is not None:
            conn.execute(
                "UPDATE project_connection SET parentId=? WHERE id=?",
                (created[source_group], edge["id"]),
            )
        elif source_group != target_group:
            conn.execute("DELETE FROM project_connection WHERE id=?", (edge["id"],))

    for signal in signals:
        edges = signal_edges[signal.name]
        original_source = core_endpoint(json.loads(edges[0]["source"]))
        if any(
            core_endpoint(json.loads(edge["source"])) != original_source
            for edge in edges[1:]
        ):
            raise RuntimeError(f"Signal source mismatch: {signal.name}")
        source_group = group_for_alias.get(signal.source_alias)
        if source_group is None:
            parent_source = original_source
        else:
            source_module = created[source_group]
            output_id = interfaces[source_group][1][signal.name]
            insert_connection(
                conn,
                connection_template,
                source_module,
                original_source,
                {"cell": output_id, "port": "in"},
                f"{parent_id}:{signal.name}:source-boundary",
            )
            parent_source = {"cell": source_module, "port": output_id}

        targets_by_group: dict[str | None, list[dict[str, str]]] = defaultdict(list)
        for edge, target_ref in zip(edges, signal.targets):
            target_group = group_for_alias.get(target_ref.alias)
            targets_by_group[target_group].append(
                core_endpoint(json.loads(edge["target"]))
            )

        for target_group, original_targets in targets_by_group.items():
            if target_group is None:
                for index, original_target in enumerate(original_targets):
                    insert_connection(
                        conn,
                        connection_template,
                        parent_id,
                        parent_source,
                        original_target,
                        f"{parent_id}:{signal.name}:parent-target:{index}",
                    )
            else:
                target_module = created[target_group]
                input_id = interfaces[target_group][0][signal.name]
                insert_connection(
                    conn,
                    connection_template,
                    parent_id,
                    parent_source,
                    {"cell": target_module, "port": input_id},
                    f"{parent_id}:{signal.name}:module-target:{target_group}",
                )
                for index, original_target in enumerate(original_targets):
                    insert_connection(
                        conn,
                        connection_template,
                        target_module,
                        {"cell": input_id, "port": "out"},
                        original_target,
                        f"{parent_id}:{signal.name}:inside-target:{target_group}:{index}",
                    )

    for group in groups:
        for old_alias, new_alias in group.renames.items():
            update_alias(conn, alias_to_id[old_alias], new_alias)
    return created


def add_composite_input(
    conn: sqlite3.Connection,
    composite_id: str,
    name: str,
    data_type: str,
    input_template: dict[str, Any],
) -> str:
    row = load_row(conn, composite_id)
    sections, indexed = parse_sections(row["extensionProps"])
    existing = indexed["function_input_ports"]["value"]
    if any(port.get("name") == name for port in existing):
        raise RuntimeError(f"Composite input already exists: {name}")
    port_id = stable_id(f"composite:{composite_id}:input:{name}")
    existing.append(composite_port(port_id, name, data_type, "In"))
    conn.execute(
        "UPDATE project_component SET extensionProps=? WHERE id=?",
        (json_text(sections), composite_id),
    )
    node = create_boundary_node(
        input_template,
        port_id,
        composite_id,
        name,
        data_type,
        (-1020, 380),
        True,
    )
    insert_component(conn, node)
    return port_id


def prepare_steer_enable_gate(
    conn: sqlite3.Connection,
    input_template: dict[str, Any],
    connection_template: dict[str, Any],
) -> None:
    control_input_id = add_composite_input(
        conn, STEER_ID, "controlEnabled", "bool", input_template
    )
    update_alias(conn, "9038fa34-75d2-4658-96a6-874dd194249e", "egoV")

    scale_id = "737599ea-943c-48dc-a84b-563b958204de"
    steer_output_boundary_id = "e9ec6429-b6b0-403a-9729-2ab706341058"
    internal_edge = conn.execute(
        """
        SELECT id FROM project_connection
        WHERE parentId=?
          AND json_extract(source,'$.cell')=?
          AND json_extract(target,'$.cell')=?
        """,
        (STEER_ID, scale_id, steer_output_boundary_id),
    ).fetchall()
    if len(internal_edge) != 1:
        raise RuntimeError("Unexpected SteerCommand output wiring")
    conn.execute("DELETE FROM project_connection WHERE id=?", (internal_edge[0]["id"],))

    root_gate_edges = conn.execute(
        """
        SELECT id FROM project_connection
        WHERE parentId=? AND (
          json_extract(source,'$.cell')=? OR json_extract(target,'$.cell')=?
        )
        """,
        (ROOT_ID, FINAL_GATE_ID, FINAL_GATE_ID),
    ).fetchall()
    if len(root_gate_edges) != 4:
        raise RuntimeError(f"Expected four top-level gate edges, found {len(root_gate_edges)}")
    for edge in root_gate_edges:
        conn.execute("DELETE FROM project_connection WHERE id=?", (edge["id"],))

    conn.execute(
        "UPDATE project_component SET parentId=? WHERE id=?", (STEER_ID, FINAL_GATE_ID)
    )
    update_position(conn, FINAL_GATE_ID, ROOT_ID, STEER_ID, (760, 20))

    insert_connection(
        conn,
        connection_template,
        STEER_ID,
        {"cell": scale_id, "port": "multiply_output_result_Out"},
        {"cell": FINAL_GATE_ID, "port": "multiply_input_a_In"},
        "steer:scale-to-enable-gate",
    )
    insert_connection(
        conn,
        connection_template,
        STEER_ID,
        {"cell": control_input_id, "port": "out"},
        {"cell": FINAL_GATE_ID, "port": "multiply_input_b_In"},
        "steer:control-enable-to-gate",
    )
    insert_connection(
        conn,
        connection_template,
        STEER_ID,
        {"cell": FINAL_GATE_ID, "port": "multiply_output_result_Out"},
        {"cell": steer_output_boundary_id, "port": "in"},
        "steer:gate-to-output",
    )
    insert_connection(
        conn,
        connection_template,
        ROOT_ID,
        {"cell": DECISION_ID, "port": DECISION_OUTPUT_PORT},
        {"cell": STEER_ID, "port": control_input_id},
        "root:decision-to-steer-enable",
    )
    insert_connection(
        conn,
        connection_template,
        ROOT_ID,
        {"cell": STEER_ID, "port": STEER_OUTPUT_PORT},
        {"cell": ROOT_STEER_OUTPUT_ID, "port": "in"},
        "root:steer-to-output",
    )
    insert_connection(
        conn,
        connection_template,
        ROOT_ID,
        {"cell": STEER_ID, "port": STEER_OUTPUT_PORT},
        {"cell": SCOPE_ID, "port": "input_port_1"},
        "root:steer-to-scope",
    )


def rename_external_port(
    conn: sqlite3.Connection,
    composite_id: str,
    section_key: str,
    old_name: str,
    new_name: str,
    data_type: str,
) -> None:
    row = load_row(conn, composite_id)
    sections, indexed = parse_sections(row["extensionProps"])
    matches = [
        port for port in indexed[section_key]["value"] if port.get("name") == old_name
    ]
    if len(matches) != 1:
        raise RuntimeError(
            f"Expected one port {old_name} on {composite_id}, found {len(matches)}"
        )
    port = matches[0]
    port["name"] = new_name
    port["alias"] = new_name
    for prop in port.get("props", []):
        if prop.get("key") == "name":
            prop["value"] = new_name
        elif prop.get("key") == "dataType":
            prop["value"] = data_type
    conn.execute(
        "UPDATE project_component SET extensionProps=? WHERE id=?",
        (json_text(sections), composite_id),
    )


def rename_boundary(
    conn: sqlite3.Connection, boundary_id: str, alias: str, data_type: str, is_input: bool
) -> None:
    row = load_row(conn, boundary_id)
    configure_boundary_row(row, alias, data_type, is_input)
    conn.execute(
        "UPDATE project_component SET alias=?, properties=?, extensionProps=? WHERE id=?",
        (alias, row["properties"], row["extensionProps"], boundary_id),
    )


def refine_existing_names(conn: sqlite3.Connection) -> None:
    update_alias(
        conn,
        PREVIEW_ID,
        "PreviewDistance",
        "根据车速和道路曲率计算LKS远预瞄距离。",
    )
    preview_renames = {
        "AbsCurv": "CurvatureMagnitude",
        "AlphaL": "PreviewScale",
        "BaseDist": "NominalPreviewDistance",
        "CurveReduce": "AppliedCurveReduction",
        "IsCurve": "IsCurvedRoad",
        "OneMinusRalpha": "CurveReductionRatio",
        "P_CurvThresh": "CurvatureThreshold",
        "P_Ralpha": "CurvePreviewScale",
        "PreviewDist": "AdjustedPreviewDistance",
        "lks_l0": "BasePreviewDistance",
        "lks_rt": "PreviewTimeGain",
        "乘法运算": "SpeedPreviewDistance",
        "常量": "One",
        "输出": "previewDistance",
    }
    for row in conn.execute(
        "SELECT id,alias FROM project_component WHERE parentId=?", (PREVIEW_ID,)
    ):
        if row["alias"] in preview_renames:
            update_alias(conn, row["id"], preview_renames[row["alias"]])

    eval_ids = (
        "a96057e9-5379-42d6-94a2-dd98930f3bd6",
        "d79c36b4-01b4-41fd-b98a-f76ca88a11c5",
        "b25a47b1-cf11-4375-a368-3b21bdca7e68",
    )
    eval_aliases = ("NearPreviewError", "MiddlePreviewError", "FarPreviewError")
    eval_descriptions = (
        "计算近预瞄点处的三次车道多项式横向误差。",
        "计算中预瞄点处的三次车道多项式横向误差。",
        "计算远预瞄点处的三次车道多项式横向误差。",
    )
    eval_inner_renames = {
        "C3x": "CubicTerm",
        "PlusC2": "AddQuadraticCoefficient",
        "Times_x1": "FirstMultiplyByDistance",
        "PlusC1": "AddLinearCoefficient",
        "Times_x2": "SecondMultiplyByDistance",
        "PlusC0": "AddOffsetCoefficient",
        "输出": "error",
    }
    for component_id, alias, description in zip(eval_ids, eval_aliases, eval_descriptions):
        update_alias(conn, component_id, alias, description)
        row = load_row(conn, component_id)
        sections, indexed = parse_sections(row["extensionProps"])
        output_port = indexed["function_output_ports"]["value"][0]
        old_name = output_port["name"]
        rename_external_port(
            conn, component_id, "function_output_ports", old_name, "error", "double"
        )
        for child in conn.execute(
            "SELECT id,alias,componentType FROM project_component WHERE parentId=?",
            (component_id,),
        ):
            if child["componentType"] == "output":
                rename_boundary(conn, child["id"], "error", "double", False)
            elif child["alias"] in eval_inner_renames:
                update_alias(conn, child["id"], eval_inner_renames[child["alias"]])

    update_alias(
        conn,
        FUSION_ID,
        "ErrorWeightedSum",
        "按三组权重合成近、中、远预瞄点横向误差。",
    )
    fusion_ports = (
        ("e1", "nearError", "949b4f12-162b-443b-92fd-5f63fd5784bc"),
        ("e2", "middleError", "b2629cb2-c776-4eb3-87f8-ff25a316f873"),
        ("e3", "farError", "e33e91b7-5f80-44e3-aa97-dd67d181dac9"),
    )
    for old_name, new_name, boundary_id in fusion_ports:
        rename_external_port(
            conn, FUSION_ID, "function_input_ports", old_name, new_name, "double"
        )
        rename_boundary(conn, boundary_id, new_name, "double", True)
    fusion_renames = {
        "Add_All": "SumWeightedErrors",
        "Mul_W1E1": "WeightNearError",
        "Mul_W2E2": "WeightMiddleError",
        "Mul_W3E3": "WeightFarError",
        "w1": "NearErrorWeight",
        "w2": "MiddleErrorWeight",
        "w3": "FarErrorWeight",
    }
    for child in conn.execute(
        "SELECT id,alias FROM project_component WHERE parentId=?", (FUSION_ID,)
    ):
        if child["alias"] in fusion_renames:
            update_alias(conn, child["id"], fusion_renames[child["alias"]])

    update_alias(
        conn,
        STEER_ID,
        "SteerCommand",
        "计算受横向加速度约束并经过使能门控的方向盘转角命令。",
    )
    update_alias(
        conn,
        DECISION_ID,
        "Decision",
        "根据车速、制动和驾驶员转向输入判断LKS是否允许控制。",
    )
    update_alias(
        conn,
        ROOT_ID,
        "MainFlow",
        "LKS决策与横向控制主流程。",
    )


def boundary_base_name(row: sqlite3.Row) -> str | None:
    _, indexed = parse_sections(row["extensionProps"])
    values = indexed.get("base_config", {}).get("value", [])
    if values and isinstance(values[0], dict):
        return values[0].get("name")
    return None


def external_ports(row: sqlite3.Row, section_key: str) -> list[dict[str, Any]]:
    _, indexed = parse_sections(row["extensionProps"])
    return indexed.get(section_key, {}).get("value", [])


def flattened_signature(conn: sqlite3.Connection) -> list[tuple[str, str, str, str]]:
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

    composite_types = {"run", "composite-block"}
    composite_ids = {
        component_id
        for component_id, row in components.items()
        if row["componentType"] in composite_types
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
        input_children = [row for row in children if row["componentType"] == "input"]
        output_children = [row for row in children if row["componentType"] == "output"]

        def match_child(port: dict[str, Any], candidates: list[sqlite3.Row]) -> sqlite3.Row:
            name = port.get("name")
            matches = [row for row in candidates if boundary_base_name(row) == name]
            if len(matches) == 1:
                return matches[0]
            same_id = [row for row in candidates if row["id"] == port.get("id")]
            if len(same_id) == 1:
                return same_id[0]
            if len(candidates) == 1:
                return candidates[0]
            raise RuntimeError(
                f"Cannot match composite port {composite_id}.{name} to a boundary"
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


def validate(conn: sqlite3.Connection) -> None:
    if conn.execute("PRAGMA integrity_check").fetchone()[0] != "ok":
        raise RuntimeError("SQLite integrity check failed")
    dangling = conn.execute(
        """
        SELECT COUNT(*) FROM project_connection c
        LEFT JOIN project_component s ON s.id=json_extract(c.source,'$.cell')
        LEFT JOIN project_component t ON t.id=json_extract(c.target,'$.cell')
        WHERE s.id IS NULL OR t.id IS NULL
        """
    ).fetchone()[0]
    if dangling:
        raise RuntimeError(f"Found {dangling} dangling connections")
    cross_boundary = conn.execute(
        """
        SELECT COUNT(*) FROM project_connection c
        JOIN project_component s ON s.id=json_extract(c.source,'$.cell')
        JOIN project_component t ON t.id=json_extract(c.target,'$.cell')
        WHERE s.parentId<>c.parentId OR t.parentId<>c.parentId
        """
    ).fetchone()[0]
    if cross_boundary:
        raise RuntimeError(f"Found {cross_boundary} cross-boundary connections")
    duplicate_targets = conn.execute(
        """
        SELECT COUNT(*) FROM (
          SELECT parentId,target,COUNT(*) count_value
          FROM project_connection GROUP BY parentId,target HAVING count_value>1
        )
        """
    ).fetchone()[0]
    if duplicate_targets:
        raise RuntimeError(f"Found {duplicate_targets} multiply-driven inputs")

    expected_root = {
        "egoV",
        "c0",
        "c1",
        "c2",
        "c3",
        "curvature",
        "brakePressed",
        "driverSteerNorm",
        "Decision",
        "Control",
        "OutputValid",
        "steerRad",
        "controlEnabled",
        "valid",
        "previewDistance",
        "weightedError",
        "仿真示波器",
    }
    actual_root = {
        row["alias"]
        for row in conn.execute(
            "SELECT alias FROM project_component WHERE parentId=?", (ROOT_ID,)
        )
    }
    if actual_root != expected_root:
        raise RuntimeError(
            f"Unexpected MainFlow children: missing={sorted(expected_root-actual_root)}, "
            f"extra={sorted(actual_root-expected_root)}"
        )
    required_modules = {
        "Decision": {"SpeedEnableCheck", "DriverOverrideCheck", "ControlEnableDecision"},
        "Control": {"PreviewDistance", "LaneErrorEvaluation", "ErrorWeightedSum", "SteerCommand"},
        "LaneErrorEvaluation": {"NearPreviewError", "MiddlePreviewError", "FarPreviewError"},
        "SteerCommand": {"RawSteerCalculation", "LateralAccelLimit", "SteerEnableGate"},
    }
    for parent_alias, expected in required_modules.items():
        parent = conn.execute(
            "SELECT id FROM project_component WHERE alias=?", (parent_alias,)
        ).fetchall()
        if len(parent) != 1:
            raise RuntimeError(f"Expected one module named {parent_alias}")
        actual = {
            row["alias"]
            for row in conn.execute(
                "SELECT alias FROM project_component WHERE parentId=? AND componentType='composite-block'",
                (parent[0]["id"],),
            )
        }
        if not expected.issubset(actual):
            raise RuntimeError(
                f"Hierarchy mismatch in {parent_alias}: missing={sorted(expected-actual)}"
            )


def arrange_root(conn: sqlite3.Connection, control_id: str) -> None:
    root_positions = {
        "egoV": (-1480, -620),
        "c0": (-1480, -500),
        "c1": (-1480, -380),
        "c2": (-1480, -260),
        "c3": (-1480, -140),
        "curvature": (-1480, -20),
        "brakePressed": (-1480, 100),
        "driverSteerNorm": (-1480, 220),
        "Decision": (-880, 260),
        "Control": (-300, -260),
        "OutputValid": (620, 130),
        "steerRad": (980, -430),
        "controlEnabled": (980, -300),
        "valid": (980, -170),
        "previewDistance": (980, -40),
        "weightedError": (980, 90),
        "仿真示波器": (1270, -380),
    }
    current = components_by_alias(conn, ROOT_ID)
    for alias, position in root_positions.items():
        update_position(conn, current[alias]["id"], ROOT_ID, ROOT_ID, position)
    update_position(conn, DECISION_ID, ROOT_ID, ROOT_ID, root_positions["Decision"], (360, 250))
    update_position(conn, control_id, ROOT_ID, ROOT_ID, root_positions["Control"], (500, 360))


def refactor(db_path: Path) -> tuple[int, int]:
    conn = sqlite3.connect(db_path)
    conn.row_factory = sqlite3.Row
    conn.execute("PRAGMA foreign_keys=OFF")
    try:
        before_signature = flattened_signature(conn)
        with conn:
            root = components_by_alias(conn, ROOT_ID)
            if "Control" in root or root.get("LKSDecision", {}).get("alias") == "Decision":
                raise RuntimeError("lks2 hierarchy already appears to be refactored")
            composite_template = load_row(conn, PREVIEW_ID)
            input_template = load_row(conn, "91d42f5c-caa1-5f90-a32a-925bd06c20ef")
            output_template = load_row(conn, ROOT_STEER_OUTPUT_ID)
            connection_template = dict(
                conn.execute("SELECT * FROM project_connection LIMIT 1").fetchone()
            )

            partition_parent(
                conn,
                DECISION_ID,
                DECISION_GROUPS,
                DECISION_SIGNALS,
                composite_template,
                input_template,
                output_template,
                connection_template,
            )
            lane_created = partition_parent(
                conn,
                ROOT_ID,
                (LANE_GROUP,),
                LANE_SIGNALS,
                composite_template,
                input_template,
                output_template,
                connection_template,
            )
            prepare_steer_enable_gate(conn, input_template, connection_template)
            partition_parent(
                conn,
                STEER_ID,
                STEER_GROUPS,
                STEER_SIGNALS,
                composite_template,
                input_template,
                output_template,
                connection_template,
            )
            control_created = partition_parent(
                conn,
                ROOT_ID,
                (CONTROL_GROUP,),
                CONTROL_SIGNALS,
                composite_template,
                input_template,
                output_template,
                connection_template,
            )
            refine_existing_names(conn)
            control_id = control_created["Control"]
            lane_id = lane_created["LaneErrorEvaluation"]
            update_alias(
                conn,
                lane_id,
                "LaneErrorEvaluation",
                "计算近、中、远三个预瞄点处的三次车道多项式横向误差。",
            )
            arrange_root(conn, control_id)
            validate(conn)
            after_signature = flattened_signature(conn)
            if before_signature != after_signature:
                before_set = set(before_signature)
                after_set = set(after_signature)
                raise RuntimeError(
                    "Flattened behavior changed: "
                    f"missing={sorted(before_set-after_set)[:8]}, "
                    f"extra={sorted(after_set-before_set)[:8]}"
                )
        return len(before_signature), len(after_signature)
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
    parser.add_argument("project", type=Path, nargs="?", default=Path("project/lks2"))
    args = parser.parse_args()
    project = args.project.resolve()
    cbdes = project / "data" / "cbdes.db"
    temp = project / "data" / "temp.db"
    if not cbdes.is_file() or not temp.is_file():
        raise RuntimeError(f"Missing cbdes.db or temp.db under {project / 'data'}")
    if sha256(cbdes) != sha256(temp):
        raise RuntimeError("cbdes.db and temp.db differ before refactoring")

    backup = cbdes.with_name("cbdes.db.before_lks_hierarchy_20260729")
    if not backup.exists():
        shutil.copy2(cbdes, backup)
    before_count, after_count = refactor(cbdes)
    shutil.copy2(cbdes, temp)
    if sha256(cbdes) != sha256(temp):
        raise RuntimeError("cbdes.db and temp.db differ after synchronization")
    print(f"Refactored: {cbdes}")
    print(f"Backup:     {backup}")
    print(f"Flat edges: {before_count} -> {after_count}")
    print(f"SHA256:     {sha256(cbdes)}")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(1)
