#!/usr/bin/env python3
"""Normalize ACC/LKS canvas identifiers without changing algorithm wiring."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import re
import shutil
import sqlite3
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any


STRUCTURAL_TYPES = {"composite-block", "run"}
CHANNEL_TYPES = {"channel_input", "channel_output"}
CONSTANT_PATTERN = re.compile(r"^[A-Z][A-Z0-9_]*$")
LOWER_CAMEL_PATTERN = re.compile(r"^[a-z][A-Za-z0-9]*$")


@dataclass(frozen=True)
class NamingSpec:
    project_kind: str
    container_aliases: dict[str, str]
    structural_aliases: dict[str, str]
    signal_names: dict[str, str]
    component_aliases: dict[str, str]
    constant_aliases: dict[str, str]
    local_names: dict[str, str]
    channel_port_aliases: dict[str, str]


ACC_SPEC = NamingSpec(
    project_kind="acc",
    container_aliases={
        "app": "accApplication",
        "module": "accModule",
        "process": "accProcess",
        "channel_output": "accOutputChannel",
        "oscilloscope": "simulationScope",
    },
    structural_aliases={
        "main": "executeAcc",
        "decision": "decision",
        "evaluateDecision": "decision",
        "control": "control",
        "calculateLongitudinalControl": "control",
        "stateClassifier": "classifySystemState",
        "controlStateUpdate": "updateControlState",
        "timeGapUpdate": "updateTimeGap",
        "maxSpeedUpdate": "updateMaxSpeed",
        "spacingFeedback": "calculateSpacingFeedback",
        "relativeSpeedFeedback": "calculateRelativeSpeedFeedback",
        "targetSpeedOutput": "generateTargetSpeed",
    },
    signal_names={
        "egoV": "egoSpeedMps",
        "leadV": "leadSpeedMps",
        "distance": "leadDistanceM",
        "timeGap": "timeGapS",
        "maxSpeed": "maxSpeedMps",
        "enable": "isControlEnabled",
        "minDistance": "minDistanceM",
        "kDist": "distanceErrorGainPerS",
        "kSpeed": "relativeSpeedGain",
        "targetSpeed": "targetSpeedMps",
        "controlEnabled": "isControlActive",
        "controlEnabledNext": "isControlActiveNext",
        "hasHistory": "hasControlHistory",
        "hasHistoryNext": "hasControlHistoryNext",
        "decision": "decisionCode",
        "vMin": "minimumOperatingSpeedMps",
        "gapStep": "timeGapStepS",
        "minGap": "minTimeGapS",
        "maxGap": "maxTimeGapS",
        "speedStep": "speedStepMps",
        "minSpeed": "minSpeedMps",
        "maxSpeedNext": "nextMaxSpeedMps",
        "timeGapNext": "nextTimeGapS",
        "distanceFeedback": "distanceFeedbackMps",
        "speedFeedback": "relativeSpeedFeedbackMps",
    },
    component_aliases={
        "decisionTable": "evaluateDecisionTable",
        "readVMin": "readMinimumOperatingSpeed",
        "readGapStep": "readTimeGapStep",
        "readMinGap": "readMinimumTimeGap",
        "readMaxGap": "readMaximumTimeGap",
        "readSpeedStep": "readSpeedStep",
        "readMinSpeed": "readMinimumSpeed",
        "readControlEnabled": "readControlActiveState",
        "readHasHistory": "readControlHistoryState",
        "readTimeGap": "readTimeGapState",
        "readMaxSpeed": "readMaxSpeedState",
        "writeControlEnabled": "writeControlActiveState",
        "writeHasHistory": "writeControlHistoryState",
        "writeTimeGap": "writeTimeGapState",
        "writeMaxSpeed": "writeMaxSpeedState",
        "keepOrEngage": "isActiveOrEngaged",
        "nextControlEnabled": "isNextControlActive",
        "nextHasHistory": "hasNextControlHistory",
        "noCancelRequest": "hasNoCancelRequest",
        "hasNoHistory": "hasNoControlHistory",
        "isActive": "isActiveAtValidSpeed",
        "encodeNoHistory": "encodeNoHistoryState",
        "encodeLowSpeed": "encodeLowSpeedState",
        "applyMinSpeed": "applyMinimumSpeed",
        "speedCandidate": "candidateMaxSpeedMps",
        "speedAfterDecrease": "decreasedMaxSpeedMps",
        "speedDecreaseDelta": "speedDecreaseMps",
        "speedIncreaseDelta": "speedIncreaseMps",
        "applyMinGap": "applyMinimumTimeGap",
        "applyMaxGap": "applyMaximumTimeGap",
        "gapAfterIncrease": "increasedTimeGapS",
        "gapCandidate": "candidateTimeGapS",
        "gapIncreaseDelta": "timeGapIncreaseS",
        "gapDecreaseDelta": "timeGapDecreaseS",
        "isGapIncrease": "isTimeGapIncrease",
        "isGapDecrease": "isTimeGapDecrease",
    },
    constant_aliases={
        "decisionCodeEngage": "DECISION_ENGAGE",
        "decisionCodeResume": "DECISION_RESUME",
        "commandCodeBrake": "COMMAND_BRAKE",
        "commandCodeCancel": "COMMAND_CANCEL",
        "decisionCodeSpeedDown": "DECISION_SPEED_DECREASE",
        "decisionCodeSpeedUp": "DECISION_SPEED_INCREASE",
        "decisionCodeGapDown": "DECISION_GAP_DECREASE",
        "decisionCodeGapUp": "DECISION_GAP_INCREASE",
        "stateCodeNoHistory": "STATE_STANDBY_NO_HISTORY",
        "stateCodeLowSpeed": "STATE_LOW_SPEED",
    },
    local_names={
        "kDist": "distanceErrorGainPerS",
        "minDistance": "minDistanceM",
        "kSpeed": "relativeSpeedGain",
        "vMin": "minimumOperatingSpeedMps",
        "gapStep": "timeGapStepS",
        "minGap": "minTimeGapS",
        "maxGap": "maxTimeGapS",
        "speedStep": "speedStepMps",
        "minSpeed": "minSpeedMps",
        "controlEnabled": "isControlActive",
        "hasHistory": "hasControlHistory",
        "timeGap": "timeGapS",
        "maxSpeed": "maxSpeedMps",
    },
    channel_port_aliases={
        # Channel aliases are part of the interface supplied by the GAASD team.
        # Keep them byte-for-byte compatible; only internal canvas identifiers
        # are normalized by this script.
        "ego_speed_mps": "ego_speed_mps",
        "command_type": "command_type",
        "frame_id": "frame_id",
        "lead_speed_mps": "leadV",
        "lead_distance_m": "distance",
        "target_speed_mps": "target_speed_mps",
        "enable": "enable",
    },
)


LKS_SPEC = NamingSpec(
    project_kind="lks",
    container_aliases={
        "app": "lksApplication",
        "process": "lksProcess",
        "module": "lksModule",
        "channel_output": "lksOutputChannel",
        "oscilloscope": "simulationScope",
    },
    structural_aliases={
        "main": "executeLks",
        "decision": "decision",
        "evaluateControlAvailability": "decision",
        "control": "control",
        "calculateLateralControl": "control",
        "speedEnableCheck": "checkOperatingSpeed",
        "driverOverrideCheck": "detectDriverOverride",
        "controlEnableDecision": "determineControlEnable",
        "previewDistanceCalculation": "calculatePreviewDistance",
        "laneErrorEvaluation": "calculatePreviewErrors",
        "nearPreviewError": "calculateNearPreviewError",
        "middlePreviewError": "calculateMiddlePreviewError",
        "farPreviewError": "calculateFarPreviewError",
        "errorWeightedSum": "calculateWeightedError",
        "steerCommand": "generateSteerCommand",
        "rawSteerCalculation": "calculateRawSteer",
        "lateralAccelLimit": "limitSteerByLateralAcceleration",
        "steerEnableGate": "applySteerEnable",
    },
    signal_names={
        "egoV": "egoSpeedMps",
        "c0": "c0",
        "c1": "c1",
        "c2": "c2",
        "c3": "c3",
        "laneOffsetM": "c0",
        "laneHeadingCoeff": "c1",
        "laneCurvatureCoeffPerM": "c2",
        "laneCurvatureRateCoeffPerM2": "c3",
        "curvature": "roadCurvaturePerM",
        "brakePressed": "isBrakePressed",
        "driverSteerNorm": "driverSteerNorm",
        "steerRad": "steerRad",
        "controlEnabled": "isControlEnabled",
        "speedUnavailable": "isSpeedUnavailable",
        "driverOverride": "isDriverOverride",
        "nearError": "nearLateralErrorM",
        "middleError": "middleLateralErrorM",
        "farError": "farLateralErrorM",
        "weightedError": "weightedLateralErrorM",
        "previewDistance": "previewDistanceM",
        "x": "previewPointDistanceM",
        "error": "lateralErrorM",
        "rawSteerNorm": "rawSteerNorm",
        "limitedSteerNorm": "limitedSteerNorm",
    },
    component_aliases={
        "driverSteerMagnitude": "calculateDriverSteerMagnitude",
        "driverOverrideThreshold": "readDriverSteerThreshold",
        "minimumOperatingSpeed": "readMinimumOperatingSpeed",
        "enableLksControl": "isControlAllowed",
        "nearErrorWeight": "readNearErrorWeight",
        "middleErrorWeight": "readMiddleErrorWeight",
        "farErrorWeight": "readFarErrorWeight",
        "weightNearError": "applyNearErrorWeight",
        "weightMiddleError": "applyMiddleErrorWeight",
        "weightFarError": "applyFarErrorWeight",
        "middlePreviewDistance": "calculateMiddlePreviewDistance",
        "nearPreviewDistance": "readNearPreviewDistance",
        "cubicTerm": "multiplyCubicCoefficient",
        "previewTimeGain": "readPreviewTimeGain",
        "basePreviewDistance": "readBasePreviewDistance",
        "curvatureThreshold": "readCurvatureThreshold",
        "curvePreviewScale": "readCurvePreviewScale",
        "curvatureMagnitude": "calculateCurvatureMagnitude",
        "speedPreviewDistance": "calculateSpeedPreviewDistance",
        "nominalPreviewDistance": "calculateNominalPreviewDistance",
        "curveReductionRatio": "calculateCurveReductionRatio",
        "appliedCurveReduction": "applyCurveReduction",
        "previewScale": "calculatePreviewScale",
        "adjustedPreviewDistance": "calculateAdjustedPreviewDistance",
        "steerErrorGain": "readSteerErrorGain",
        "wheelBase": "readWheelBase",
        "maxLateralAcceleration": "readMaxLateralAcceleration",
        "maxFrontWheelAngle": "readMaxFrontWheelAngle",
        "speedSquare": "calculateSpeedSquare",
        "safeSpeedSquare": "applyMinimumSpeedSquare",
        "maxLateralAccelMoment": "calculateLateralAccelMoment",
        "lateralAccelRatio": "calculateSteerAngleRatio",
        "steerAngleLimitRad": "calculateSteerLimitRad",
        "normalizedSteerLimit": "normalizeSteerLimit",
        "negativeSteerLimit": "negateSteerLimit",
        "steerScaleRad": "readSteerScaleRad",
    },
    constant_aliases={
        "one": "ONE",
        "previewDistanceDivisor": "PREVIEW_DISTANCE_DIVISOR",
        "minimumSpeedSquare": "MINIMUM_SPEED_SQUARED_M2PS2",
    },
    local_names={},
    channel_port_aliases={
        # Preserve the aliases exported by the fixed LKS channel component.
        "ego_speed_mps": "egoV",
        "brake_pressed": "brakePressed",
        "driver_steer_norm": "driverSteerNorm",
        "frame_id": "frameId",
        "c0_m": "c0",
        "c1": "c1",
        "c2_per_m": "c2",
        "c3_per_m2": "c3",
        "curvature_per_m": "curvature",
        "steer_rad": "steer_rad",
        "control_enabled": "control_enabled",
        "channel_output:frame_id": "frame_id",
    },
)


def json_text(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, separators=(",", ":"))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def extension_section(extension_props: list[dict[str, Any]], key: str) -> Any:
    for section in extension_props:
        if section.get("key") == key:
            return section.get("value")
    return None


def base_config(extension_props: list[dict[str, Any]]) -> dict[str, Any]:
    values = extension_section(extension_props, "base_config") or []
    return values[0] if values and isinstance(values[0], dict) else {}


def replace_prop_name(props: list[dict[str, Any]], mapping: dict[str, str]) -> None:
    for prop in props:
        if prop.get("key") != "name":
            continue
        value = prop.get("value")
        if value in mapping:
            prop["value"] = mapping[value]


def update_port_names(
    ports: list[dict[str, Any]], mapping: dict[str, str]
) -> None:
    for port in ports:
        old_name = port.get("name")
        old_alias = port.get("alias")
        if old_name in mapping:
            port["name"] = mapping[old_name]
            port["alias"] = mapping[old_name]
        elif old_alias in mapping:
            port["alias"] = mapping[old_alias]
        replace_prop_name(port.get("props", []), mapping)


def update_channel_aliases(
    extension_props: list[dict[str, Any]],
    mapping: dict[str, str],
    component_type: str,
) -> None:
    for key in ("function_input_ports", "function_output_ports"):
        for port in extension_section(extension_props, key) or []:
            protocol_name = port.get("name")
            expected = mapping.get(
                f"{component_type}:{protocol_name}", mapping.get(protocol_name)
            )
            if expected is not None:
                port["alias"] = expected


def update_truth_table(extension_props: list[dict[str, Any]]) -> None:
    table = base_config(extension_props)
    for condition in table.get("conditions", []):
        expression = condition.get("condition", "")
        expression = re.sub(r"\bu\b", "systemState", expression)
        expression = re.sub(r"\bv\b", "commandType", expression)
        condition["condition"] = expression
    for action in table.get("actions", []):
        expression = action.get("action", "")
        action["action"] = re.sub(r"\by\b", "decisionCode", expression)

    port_mapping = {
        "u": "systemState",
        "v": "commandType",
        "y": "decisionCode",
    }
    for key in ("function_input_ports", "function_output_ports"):
        update_port_names(extension_section(extension_props, key) or [], port_mapping)


def rename_local_declarations(
    extension_props: list[dict[str, Any]], mapping: dict[str, str]
) -> None:
    config = base_config(extension_props)
    for kind in ("param", "state"):
        for item in config.get(kind, []):
            old_name = item.get("name")
            if old_name not in mapping:
                continue
            new_name = mapping[old_name]
            item["name"] = new_name
            if item.get("id") == old_name:
                item["id"] = new_name


def rename_local_binding(
    extension_props: list[dict[str, Any]], mapping: dict[str, str]
) -> None:
    config = base_config(extension_props)
    keys = config.get("operateKeys", [])
    updated: list[str] = []
    for operate_key in keys:
        value = operate_key
        for prefix in ("param_.", "state_."):
            if not value.startswith(prefix):
                continue
            key = value[len(prefix) :]
            value = prefix + mapping.get(key, key)
            break
        updated.append(value)
    if keys:
        config["operateKeys"] = updated


def component_alias(row: sqlite3.Row, spec: NamingSpec) -> str:
    alias = row["alias"]
    component_type = row["componentType"]
    if component_type in spec.container_aliases:
        return spec.container_aliases[component_type]
    if component_type in STRUCTURAL_TYPES and alias in spec.structural_aliases:
        return spec.structural_aliases[alias]
    if component_type == "constant" and alias in spec.constant_aliases:
        return spec.constant_aliases[alias]
    if spec.project_kind == "acc" and component_type == "read-local-param":
        parameter_read_aliases = {
            "kDist": "readDistanceErrorGain",
            "minDistance": "readMinimumDistance",
            "kSpeed": "readRelativeSpeedGain",
        }
        if alias in parameter_read_aliases:
            return parameter_read_aliases[alias]
    if (
        spec.project_kind == "lks"
        and component_type == "greater-equal"
        and alias == "isDriverOverride"
    ):
        return "compareDriverSteerThreshold"
    if (
        spec.project_kind == "lks"
        and component_type == "logic-not"
        and alias == "isSpeedUnavailable"
    ):
        return "invertSpeedAvailability"
    if alias in spec.component_aliases:
        return spec.component_aliases[alias]
    if alias in spec.signal_names:
        return spec.signal_names[alias]
    return alias


def capture_invariants(connection: sqlite3.Connection) -> dict[str, Any]:
    identities = [
        tuple(row)
        for row in connection.execute(
            """
            SELECT id,projectId,parentId,originId,instanceId,name,cppClass,
                   componentType,isSourceCode,isAtomic,isCustom,customType
            FROM project_component ORDER BY id
            """
        )
    ]
    connections = [
        tuple(row)
        for row in connection.execute(
            "SELECT * FROM project_connection ORDER BY id"
        )
    ]
    hierarchy_payload = [
        tuple(row)
        for row in connection.execute(
            """
            SELECT id,childComponents,childConnections
            FROM project_component ORDER BY id
            """
        )
    ]
    port_ids: list[tuple[str, str, str, str]] = []
    channel_protocol: list[tuple[str, str, str, str]] = []
    constant_values: list[tuple[str, Any, Any]] = []
    local_values: list[tuple[str, str, int, Any, Any]] = []
    truth_scenarios: list[tuple[str, str]] = []

    for row in connection.execute(
        "SELECT id,componentType,extensionProps FROM project_component ORDER BY id"
    ):
        extension_props = json.loads(row["extensionProps"] or "[]")
        for key in ("function_input_ports", "function_output_ports"):
            for port in extension_section(extension_props, key) or []:
                port_ids.append(
                    (
                        row["id"],
                        key,
                        str(port.get("id") or ""),
                        str(port.get("direction") or ""),
                    )
                )
                if row["componentType"] in CHANNEL_TYPES:
                    channel_protocol.append(
                        (
                            row["id"],
                            key,
                            str(port.get("id") or ""),
                            str(port.get("name") or ""),
                        )
                    )
        config = base_config(extension_props)
        if row["componentType"] == "constant":
            constant_values.append(
                (row["id"], config.get("dataType"), config.get("dataValue"))
            )
        for kind in ("param", "state"):
            for index, item in enumerate(config.get(kind, [])):
                local_values.append(
                    (
                        row["id"],
                        kind,
                        index,
                        item.get("dataType"),
                        item.get("dataValue"),
                    )
                )
        if row["componentType"] == "truth-table":
            truth_scenarios.append(
                (row["id"], json_text(config.get("scenarios", [])))
            )

    return {
        "identities": identities,
        "connections": connections,
        "hierarchy_payload": hierarchy_payload,
        "port_ids": sorted(port_ids),
        "channel_protocol": sorted(channel_protocol),
        "constant_values": sorted(constant_values),
        "local_values": sorted(local_values),
        "truth_scenarios": sorted(truth_scenarios),
    }


def update_database(database: Path, spec: NamingSpec) -> tuple[int, int]:
    connection = sqlite3.connect(database)
    connection.row_factory = sqlite3.Row
    try:
        before = capture_invariants(connection)
        rows = list(connection.execute("SELECT * FROM project_component ORDER BY id"))
        proposed = {row["id"]: component_alias(row, spec) for row in rows}

        sibling_aliases: dict[str, set[str]] = {}
        for row in rows:
            aliases = sibling_aliases.setdefault(row["parentId"], set())
            alias = proposed[row["id"]]
            if alias in aliases:
                raise RuntimeError(
                    f"Alias collision under {row['parentId']}: {alias}"
                )
            aliases.add(alias)

        with connection:
            for row in rows:
                extension_props = json.loads(row["extensionProps"] or "[]")
                properties = json.loads(row["properties"] or "{}")
                new_alias = proposed[row["id"]]
                properties["alias"] = new_alias

                if row["componentType"] not in CHANNEL_TYPES:
                    for key in (
                        "function_input_ports",
                        "function_output_ports",
                    ):
                        update_port_names(
                            extension_section(extension_props, key) or [],
                            spec.signal_names,
                        )

                config = base_config(extension_props)
                if row["componentType"] in STRUCTURAL_TYPES:
                    config["instanceName"] = new_alias
                    rename_local_declarations(extension_props, spec.local_names)
                if row["componentType"] in {
                    "input",
                    "output",
                }:
                    old_name = config.get("name")
                    if old_name in spec.signal_names:
                        config["name"] = spec.signal_names[old_name]
                if row["componentType"] in {
                    "read-local-param",
                    "read-local-state",
                    "write-local-state",
                }:
                    rename_local_binding(extension_props, spec.local_names)
                if row["componentType"] == "constant":
                    old_alias = row["alias"]
                    if old_alias in spec.constant_aliases:
                        constant_name = spec.constant_aliases[old_alias]
                        config["name"] = constant_name
                        for key in (
                            "function_input_ports",
                            "function_output_ports",
                        ):
                            update_port_names(
                                extension_section(extension_props, key) or [],
                                {config.get("name", ""): constant_name,
                                 old_alias: constant_name,
                                 str(config.get("name", "")): constant_name},
                            )
                        for port in extension_section(
                            extension_props, "function_output_ports"
                        ) or []:
                            port["name"] = constant_name
                            port["alias"] = constant_name
                            for prop in port.get("props", []):
                                if prop.get("key") == "name":
                                    prop["value"] = constant_name
                if row["componentType"] == "truth-table":
                    update_truth_table(extension_props)

                connection.execute(
                    """
                    UPDATE project_component
                    SET alias=?, properties=?, extensionProps=?
                    WHERE id=?
                    """,
                    (
                        new_alias,
                        json_text(properties),
                        json_text(extension_props),
                        row["id"],
                    ),
                )

            validate_database(connection, spec, proposed, before)

        changed = sum(row["alias"] != proposed[row["id"]] for row in rows)
        return len(rows), changed
    finally:
        connection.close()


def validate_database(
    connection: sqlite3.Connection,
    spec: NamingSpec,
    proposed: dict[str, str],
    before: dict[str, Any],
) -> None:
    integrity = connection.execute("PRAGMA integrity_check").fetchone()[0]
    if integrity != "ok":
        raise RuntimeError(f"SQLite integrity check failed: {integrity}")
    after = capture_invariants(connection)
    for key in before:
        if before[key] != after[key]:
            raise RuntimeError(f"Algorithm invariant changed: {key}")

    dangling = connection.execute(
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

    for row in connection.execute(
        "SELECT id,alias,componentType,properties,extensionProps FROM project_component"
    ):
        alias = row["alias"]
        if alias != proposed[row["id"]]:
            raise RuntimeError(f"Unexpected alias for {row['id']}: {alias}")
        properties = json.loads(row["properties"] or "{}")
        if properties.get("alias") != alias:
            raise RuntimeError(f"properties.alias mismatch: {alias}")
        if row["componentType"] == "constant":
            if not CONSTANT_PATTERN.fullmatch(alias):
                raise RuntimeError(f"Constant is not UPPER_SNAKE_CASE: {alias}")
        elif not LOWER_CAMEL_PATTERN.fullmatch(alias):
            raise RuntimeError(f"Identifier is not lowerCamelCase: {alias}")

        extension_props = json.loads(row["extensionProps"] or "[]")
        config = base_config(extension_props)
        if row["componentType"] in STRUCTURAL_TYPES:
            if config.get("instanceName") != alias:
                raise RuntimeError(f"instanceName mismatch: {alias}")
        # Channel components are an externally supplied Pangu/Protobuf
        # contract.  Their selected fields and display aliases must survive a
        # naming pass byte-for-byte; only internal algorithm ports are renamed.

    rows_by_id = {
        row["id"]: row
        for row in connection.execute(
            "SELECT id,parentId,alias,componentType,extensionProps FROM project_component"
        )
    }
    for row in rows_by_id.values():
        if row["componentType"] not in {
            "read-local-param",
            "read-local-state",
            "write-local-state",
        }:
            continue
        extension_props = json.loads(row["extensionProps"] or "[]")
        keys = base_config(extension_props).get("operateKeys", [])
        if len(keys) != 1:
            raise RuntimeError(f"Unexpected local binding count: {row['alias']}")
        operate_key = keys[0]
        prefix = (
            "param_."
            if row["componentType"] == "read-local-param"
            else "state_."
        )
        if not operate_key.startswith(prefix):
            raise RuntimeError(f"Invalid local binding: {row['alias']}")
        parent = rows_by_id.get(row["parentId"])
        if parent is None:
            raise RuntimeError(f"Missing local declaration parent: {row['alias']}")
        parent_props = json.loads(parent["extensionProps"] or "[]")
        declaration_kind = "param" if prefix == "param_." else "state"
        declarations = {
            item.get("name")
            for item in base_config(parent_props).get(declaration_kind, [])
        }
        bound_name = operate_key[len(prefix) :]
        if bound_name not in declarations:
            raise RuntimeError(
                f"Local binding has no declaration: {row['alias']} -> {operate_key}"
            )

    if spec.project_kind == "acc":
        truth_rows = connection.execute(
            "SELECT extensionProps FROM project_component WHERE componentType='truth-table'"
        ).fetchall()
        for truth_row in truth_rows:
            extension_props = json.loads(truth_row["extensionProps"] or "[]")
            table = base_config(extension_props)
            expressions = [
                item.get("condition", "") for item in table.get("conditions", [])
            ] + [item.get("action", "") for item in table.get("actions", [])]
            if any(re.search(r"\b[uvy]\b", expression) for expression in expressions):
                raise RuntimeError("Truth-table placeholder identifiers remain")


def normalize_project(project: Path, spec: NamingSpec, dry_run: bool) -> None:
    cbdes = project / "data" / "cbdes.db"
    temp = project / "data" / "temp.db"
    if not cbdes.is_file() or not temp.is_file():
        raise RuntimeError(f"Missing GAASD databases under {project}")
    if sha256(cbdes) != sha256(temp):
        raise RuntimeError(f"cbdes.db and temp.db differ before update: {project}")

    if dry_run:
        with tempfile.TemporaryDirectory(prefix=f"{project.name}-naming-") as tmp:
            staged = Path(tmp) / "cbdes.db"
            shutil.copy2(cbdes, staged)
            count, changed = update_database(staged, spec)
            print(
                f"DRY-RUN PASS {project}: components={count}, renamed={changed}"
            )
        return

    backup = cbdes.with_name("cbdes.db.before_naming_standard_20260810")
    if not backup.exists():
        shutil.copy2(cbdes, backup)
    count, changed = update_database(cbdes, spec)
    shutil.copy2(cbdes, temp)
    if sha256(cbdes) != sha256(temp):
        raise RuntimeError(f"Database synchronization failed: {project}")
    print(f"UPDATED {project}: components={count}, renamed={changed}")
    print(f"BACKUP  {backup}")
    print(f"SHA256  {sha256(cbdes)}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--acc-project", type=Path, default=Path("project/newaccpro4")
    )
    parser.add_argument("--lks-project", type=Path, default=Path("project/lks2"))
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    normalize_project(args.acc_project.resolve(), ACC_SPEC, args.dry_run)
    normalize_project(args.lks_project.resolve(), LKS_SPEC, args.dry_run)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(1)
