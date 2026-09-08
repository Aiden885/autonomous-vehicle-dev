#!/usr/bin/env python3
"""Set editable GAASD component Chinese names without changing aliases.

GAASD stores the English instance alias in ``project_component.alias`` and the
new Chinese-name field in ``project_component.properties.alias``.  This tool
keeps custom Chinese names only where the current frontend exposes that field.
Read-only basic blocks use the official component-library Chinese name.  Graph
structure and interface data are verified to remain logically identical.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import sqlite3
from datetime import datetime
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
LIBRARY_DATABASE = Path(
    "/home/aiden/gaasd_versions/gaasd-2.7.0.5/home/.gaasd/gaasd.db"
)
EDITABLE_ORIGIN_IDS = {"process_empty", "module_empty", "initial-composite-block"}


ACC_NAMES = {
    "candidateMaxSpeedMps": "计算速度上限候选值",
    "encodeSystemState": "编码系统状态",
    "increasedTimeGapS": "计算增大后目标时距",
    "rawTargetSpeed": "合成原始目标速度",
    "accApplication": "ACC应用",
    "applyMaximumTimeGap": "应用目标时距上限",
    "applyMinimumSpeed": "应用速度下限",
    "applyMinimumTimeGap": "应用目标时距下限",
    "desiredDistance": "计算期望跟车距离",
    "nonNegativeTargetSpeed": "应用目标速度非负约束",
    "targetSpeedLimiter": "应用目标速度上限",
    "vehicleInputChannel": "ACC输入通道",
    "accOutputChannel": "ACC输出通道",
    "calculateRelativeSpeedFeedback": "计算相对速度反馈",
    "calculateSpacingFeedback": "计算车距反馈",
    "classifySystemState": "判定系统状态",
    "control": "控制",
    "decision": "决策",
    "executeAcc": "ACC主流程",
    "generateTargetSpeed": "生成目标速度",
    "updateControlState": "更新控制状态",
    "updateMaxSpeed": "更新速度上限",
    "updateTimeGap": "更新目标时距",
    "COMMAND_BRAKE": "制动指令常量",
    "COMMAND_CANCEL": "取消指令常量",
    "DECISION_ENGAGE": "启控决策常量",
    "DECISION_GAP_DECREASE": "减小时距决策常量",
    "DECISION_GAP_INCREASE": "增大时距决策常量",
    "DECISION_RESUME": "恢复决策常量",
    "DECISION_SPEED_DECREASE": "降低速度决策常量",
    "DECISION_SPEED_INCREASE": "提高速度决策常量",
    "STATE_LOW_SPEED": "低速状态常量",
    "STATE_STANDBY_NO_HISTORY": "无历史待命状态常量",
    "zeroSpeed": "零速度常量",
    "isBrakeCommand": "判断制动指令",
    "isCancelCommand": "判断取消指令",
    "isEngageDecision": "判断启控决策",
    "isResumeDecision": "判断恢复决策",
    "isSpeedDecrease": "判断降低速度决策",
    "isSpeedIncrease": "判断提高速度决策",
    "isTimeGapDecrease": "判断减小时距决策",
    "isTimeGapIncrease": "判断增大时距决策",
    "commandType": "驾驶指令类型",
    "decisionCode": "决策码",
    "distanceErrorGainPerS": "车距误差增益",
    "distanceFeedbackMps": "车距反馈速度",
    "egoSpeedMps": "自车速度",
    "hasControlHistory": "是否存在控制历史",
    "isControlActive": "在控状态",
    "isControlEnabled": "控制使能",
    "leadDistanceM": "前车距离",
    "leadSpeedMps": "前车速度",
    "maxSpeedMps": "速度上限",
    "maxTimeGapS": "最大目标时距",
    "minDistanceM": "最小跟车距离",
    "minSpeedMps": "最小目标速度",
    "minTimeGapS": "最小目标时距",
    "minimumOperatingSpeedMps": "最低工作车速",
    "relativeSpeedFeedbackMps": "相对速度反馈",
    "relativeSpeedGain": "相对速度增益",
    "speedStepMps": "速度调节步长",
    "timeGapS": "目标时距",
    "timeGapStepS": "时距调节步长",
    "isLowSpeed": "判断低速状态",
    "isActiveAtValidSpeed": "判定有效车速下在控",
    "isNextControlActive": "判定下一周期在控",
    "isStandbyWithHistory": "判定有历史待命状态",
    "isStandbyWithoutHistory": "判定无历史待命状态",
    "hasNoCancelRequest": "判断无退出请求",
    "hasNoControlHistory": "判断无控制历史",
    "isSpeedValid": "判断车速有效",
    "isStandby": "判定待命状态",
    "hasCancelRequest": "判定退出请求",
    "hasEngageRequest": "判定启控请求",
    "hasNextControlHistory": "判定下一周期控制历史",
    "isActiveOrEngaged": "判定在控或启控",
    "accModule": "ACC算法模块",
    "distanceFeedback": "计算车距反馈项",
    "enableGate": "应用控制使能门控",
    "encodeLowSpeedState": "编码低速状态",
    "encodeNoHistoryState": "编码无历史待命状态",
    "speedDecreaseMps": "计算减速步长",
    "speedFeedback": "计算相对速度反馈项",
    "speedIncreaseMps": "计算加速步长",
    "timeGapDecreaseS": "计算时距减小步长",
    "timeGapDistance": "计算时距跟车距离",
    "timeGapIncreaseS": "计算时距增大步长",
    "仿真示波器": "仿真示波器",
    "simulationScope": "仿真示波器",
    "hasControlHistoryNext": "下一周期是否存在控制历史",
    "isControlActiveNext": "下一周期在控状态",
    "nextMaxSpeedMps": "下一周期速度上限",
    "nextTimeGapS": "下一周期目标时距",
    "systemState": "系统状态",
    "targetSpeedMps": "目标速度",
    "acc进程": "ACC进程",
    "readDistanceErrorGain": "读取车距误差增益",
    "readMaximumTimeGap": "读取最大目标时距",
    "readMinimumDistance": "读取最小跟车距离",
    "readMinimumOperatingSpeed": "读取最低工作车速",
    "readMinimumSpeed": "读取最小目标速度",
    "readMinimumTimeGap": "读取最小目标时距",
    "readRelativeSpeedGain": "读取相对速度增益",
    "readSpeedStep": "读取速度调节步长",
    "readTimeGapStep": "读取时距调节步长",
    "readControlActiveState": "读取在控状态",
    "readControlHistoryState": "读取控制历史状态",
    "readMaxSpeedState": "读取速度上限状态",
    "readTimeGapState": "读取目标时距状态",
    "candidateTimeGapS": "计算目标时距候选值",
    "decreasedMaxSpeedMps": "计算减小后速度上限",
    "distanceError": "计算车距误差",
    "relativeSpeed": "计算相对速度",
    "evaluateDecisionTable": "执行决策真值表",
    "writeControlActiveState": "写入在控状态",
    "writeControlHistoryState": "写入控制历史状态",
    "writeMaxSpeedState": "写入速度上限状态",
    "writeTimeGapState": "写入目标时距状态",
}


LKS_NAMES = {
    "addLinearCoefficient": "累加一次项",
    "addOffsetCoefficient": "累加常数项",
    "addQuadraticCoefficient": "累加二次项",
    "calculateNominalPreviewDistance": "计算标称预瞄距离",
    "sumWeightedErrors": "合成加权预瞄误差",
    "lksApplication": "LKS应用",
    "applyLowerSteerLimit": "应用转向下限",
    "applyMinimumSpeedSquare": "应用车速平方下限",
    "applyUpperSteerLimit": "应用转向上限",
    "calculateCurvatureMagnitude": "计算道路曲率绝对值",
    "calculateDriverSteerMagnitude": "计算驾驶员转向绝对值",
    "calculateSteerLimitRad": "计算前轮转角限值",
    "vehicleInputChannel": "LKS输入通道",
    "lksOutputChannel": "LKS输出通道",
    "applySteerEnable": "转向角换算与使能控制",
    "calculateFarPreviewError": "计算远点预瞄误差",
    "calculateMiddlePreviewError": "计算中点预瞄误差",
    "calculateNearPreviewError": "计算近点预瞄误差",
    "calculatePreviewDistance": "计算预瞄距离",
    "calculatePreviewErrors": "计算三点预瞄误差",
    "calculateRawSteer": "计算原始转向指令",
    "calculateWeightedError": "计算加权预瞄误差",
    "checkOperatingSpeed": "判定工作车速条件",
    "control": "控制",
    "decision": "决策",
    "detectDriverOverride": "检测驾驶员转向接管",
    "determineControlEnable": "判定控制使能",
    "executeLks": "LKS主流程",
    "generateSteerCommand": "生成转向角指令",
    "limitSteerByLateralAcceleration": "横向加速度约束转向限幅",
    "MINIMUM_SPEED_SQUARED_M2PS2": "最小车速平方常量",
    "ONE": "单位常量",
    "PREVIEW_DISTANCE_DIVISOR": "中点预瞄距离除数",
    "calculateMiddlePreviewDistance": "计算中点预瞄距离",
    "calculateSteerAngleRatio": "计算转向限幅比值",
    "normalizeSteerLimit": "归一化转向限值",
    "compareDriverSteerThreshold": "比较驾驶员转向阈值",
    "isCurvedRoad": "判断弯道状态",
    "isSpeedAboveMinimum": "判断车速高于下限",
    "c0": "车道线常数项系数",
    "c1": "车道线一次项系数",
    "c2": "车道线二次项系数",
    "c3": "车道线三次项系数",
    "driverSteerNorm": "驾驶员归一化转向输入",
    "egoSpeedMps": "自车速度",
    "farLateralErrorM": "远点横向误差",
    "isBrakePressed": "制动状态",
    "isControlEnabled": "控制使能",
    "isDriverOverride": "驾驶员转向接管状态",
    "isSpeedUnavailable": "车速条件不满足",
    "limitedSteerNorm": "限幅后归一化转向指令",
    "middleLateralErrorM": "中点横向误差",
    "nearLateralErrorM": "近点横向误差",
    "previewDistanceM": "远预瞄距离",
    "previewPointDistanceM": "预瞄点距离",
    "rawSteerNorm": "原始归一化转向指令",
    "roadCurvaturePerM": "道路曲率",
    "weightedLateralErrorM": "加权横向误差",
    "lateralErrorM": "横向误差",
    "steerRad": "转向角指令",
    "invertSpeedAvailability": "生成车速不可用状态",
    "isControlAllowed": "生成控制允许状态",
    "hasDisableCondition": "汇总控制禁用条件",
    "lksModule": "LKS算法模块",
    "applyControlEnable": "应用控制使能",
    "applyCurveReduction": "应用曲率预瞄缩放",
    "applyFarErrorWeight": "加权远点预瞄误差",
    "applyMiddleErrorWeight": "加权中点预瞄误差",
    "applyNearErrorWeight": "加权近点预瞄误差",
    "applySteerErrorGain": "应用转向误差增益",
    "calculateAdjustedPreviewDistance": "计算曲率修正预瞄距离",
    "calculateLateralAccelMoment": "计算横向加速度约束项",
    "calculateSpeedPreviewDistance": "计算速度相关预瞄距离",
    "calculateSpeedSquare": "计算车速平方",
    "firstMultiplyByDistance": "计算三次项第一中间量",
    "multiplyCubicCoefficient": "计算车道线三次项",
    "scaleSteerToRad": "换算转向角",
    "secondMultiplyByDistance": "计算三次项第二中间量",
    "negateSteerLimit": "生成转向下限",
    "仿真示波器_1": "仿真示波器",
    "simulationScope": "仿真示波器",
    "readBasePreviewDistance": "读取基础预瞄距离",
    "readCurvatureThreshold": "读取道路曲率阈值",
    "readCurvePreviewScale": "读取弯道预瞄缩放系数",
    "readDriverSteerThreshold": "读取驾驶员转向阈值",
    "readFarErrorWeight": "读取远点误差权重",
    "readMaxFrontWheelAngle": "读取最大前轮转角",
    "readMaxLateralAcceleration": "读取最大横向加速度",
    "readMiddleErrorWeight": "读取中点误差权重",
    "readMinimumOperatingSpeed": "读取最低工作车速",
    "readNearErrorWeight": "读取近点误差权重",
    "readNearPreviewDistance": "读取近点预瞄距离",
    "readPreviewTimeGain": "读取预瞄时间增益",
    "readSteerErrorGain": "读取转向误差增益",
    "readSteerScaleRad": "读取转向角缩放系数",
    "readWheelBase": "读取车辆轴距",
    "calculateCurveReductionRatio": "计算曲率缩放差值",
    "calculatePreviewScale": "计算预瞄缩放系数",
    "lksProcess": "LKS进程",
}


PROJECTS = {
    ROOT / "project" / "newaccpro4": ACC_NAMES,
    ROOT / "project" / "lks3": LKS_NAMES,
}


def canonical_json(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"))


def database_invariants(connection: sqlite3.Connection) -> str:
    """Hash everything except the Chinese-name property being migrated."""
    connection.row_factory = sqlite3.Row
    component_rows = []
    for row in connection.execute("SELECT * FROM project_component ORDER BY id"):
        item = dict(row)
        properties = json.loads(item.get("properties") or "{}")
        properties.pop("alias", None)
        item["properties"] = properties
        component_rows.append(item)

    connection_rows = [
        dict(row)
        for row in connection.execute("SELECT * FROM project_connection ORDER BY id")
    ]
    payload = {"components": component_rows, "connections": connection_rows}
    return hashlib.sha256(canonical_json(payload).encode("utf-8")).hexdigest()


def load_official_names() -> dict[str, str]:
    if not LIBRARY_DATABASE.is_file():
        raise FileNotFoundError(LIBRARY_DATABASE)
    connection = sqlite3.connect(LIBRARY_DATABASE)
    rows = connection.execute("SELECT originId, alias FROM component").fetchall()
    connection.close()
    return {origin_id: alias for origin_id, alias in rows}


def load_baseline_names(database: Path | None) -> dict[str, str]:
    if database is None:
        return {}
    if not database.is_file():
        raise FileNotFoundError(database)
    connection = sqlite3.connect(database)
    connection.row_factory = sqlite3.Row
    names = {}
    for row in connection.execute("SELECT id, properties FROM project_component"):
        properties = json.loads(row["properties"] or "{}")
        names[row["id"]] = properties.get("alias", "")
    connection.close()
    return names


def is_frontend_editable(row: sqlite3.Row) -> bool:
    return row["componentType"] != "app" and row["originId"] in EDITABLE_ORIGIN_IDS


def editable_chinese_name(row: sqlite3.Row, names: dict[str, str]) -> str | None:
    """Resolve both the old English alias and the new Chinese alias convention."""
    alias = row["alias"]
    if alias in names:
        return names[alias]
    if alias in names.values():
        return alias

    english_name = row["name"] or ""
    lower_camel_name = english_name[:1].lower() + english_name[1:]
    return names.get(lower_camel_name)


def expected_chinese_name(
    row: sqlite3.Row,
    names: dict[str, str],
    official_names: dict[str, str],
    baseline_names: dict[str, str],
) -> str:
    if is_frontend_editable(row):
        chinese_name = editable_chinese_name(row, names)
        if chinese_name is None:
            raise KeyError(row["alias"])
        return chinese_name
    if row["originId"] in official_names:
        return official_names[row["originId"]]
    if row["id"] in baseline_names:
        return baseline_names[row["id"]]
    properties = json.loads(row["properties"] or "{}")
    return properties.get("alias", "")


def inspect_database(
    database: Path,
    names: dict[str, str],
    official_names: dict[str, str],
    baseline_database: Path | None = None,
) -> dict[str, Any]:
    connection = sqlite3.connect(database)
    connection.row_factory = sqlite3.Row
    rows = list(
        connection.execute(
            "SELECT id, alias, name, originId, componentType, properties "
            "FROM project_component"
        )
    )
    editable_rows = [row for row in rows if is_frontend_editable(row)]
    missing = sorted(
        row["alias"]
        for row in editable_rows
        if editable_chinese_name(row, names) is None
    )
    resolved_english_names = {
        row["alias"]
        if row["alias"] in names
        else (row["name"] or "")[:1].lower() + (row["name"] or "")[1:]
        for row in editable_rows
    }
    unused = sorted(names.keys() - resolved_english_names)
    baseline_names = load_baseline_names(baseline_database)
    mismatches = []
    for row in rows:
        properties = json.loads(row["properties"] or "{}")
        actual = properties.get("alias")
        expected = expected_chinese_name(row, names, official_names, baseline_names)
        if actual != expected:
            mismatches.append((row["id"], row["alias"], actual, expected))
    integrity = connection.execute("PRAGMA integrity_check").fetchone()[0]
    fingerprint = database_invariants(connection)
    connection.close()
    return {
        "rows": len(rows),
        "editable_rows": sum(is_frontend_editable(row) for row in rows),
        "missing": missing,
        "unused": unused,
        "mismatches": mismatches,
        "integrity": integrity,
        "fingerprint": fingerprint,
    }


def update_database(
    database: Path,
    names: dict[str, str],
    official_names: dict[str, str],
    baseline_database: Path | None,
) -> int:
    before = inspect_database(database, names, official_names, baseline_database)
    if before["integrity"] != "ok":
        raise RuntimeError(f"Database integrity check failed before update: {database}")
    if before["missing"]:
        raise RuntimeError(f"Unmapped aliases in {database}: {before['missing']}")

    connection = sqlite3.connect(database)
    connection.row_factory = sqlite3.Row
    baseline_names = load_baseline_names(baseline_database)
    changed = 0
    with connection:
        for row in connection.execute(
            "SELECT id, alias, name, originId, componentType, properties "
            "FROM project_component"
        ):
            properties = json.loads(row["properties"] or "{}")
            chinese_name = expected_chinese_name(
                row, names, official_names, baseline_names
            )
            if properties.get("alias") == chinese_name:
                continue
            properties["alias"] = chinese_name
            connection.execute(
                "UPDATE project_component SET properties = ? WHERE id = ?",
                (json.dumps(properties, ensure_ascii=False, separators=(",", ":")), row["id"]),
            )
            changed += 1
    connection.close()

    after = inspect_database(database, names, official_names, baseline_database)
    if after["integrity"] != "ok":
        raise RuntimeError(f"Database integrity check failed after update: {database}")
    if after["mismatches"]:
        raise RuntimeError(f"Chinese-name verification failed: {after['mismatches'][:5]}")
    if before["fingerprint"] != after["fingerprint"]:
        raise RuntimeError(f"Protected graph data changed unexpectedly: {database}")
    return changed


def backup_databases(project: Path, timestamp: str) -> list[Path]:
    backups = []
    for name in ("cbdes.db", "temp.db", "runtimeCanvas.db"):
        source = project / "data" / name
        if not source.is_file():
            raise FileNotFoundError(source)
        destination = source.with_name(
            f"{name}.before_chinese_name_permission_filter_{timestamp}"
        )
        shutil.copy2(source, destination)
        backups.append(destination)
    return backups


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--apply",
        action="store_true",
        help="Write Chinese names after validation; without this flag only inspect.",
    )
    parser.add_argument(
        "--baseline-timestamp",
        help=(
            "Use before_chinese_names_<timestamp> databases as fallback for "
            "project-local components absent from the official library."
        ),
    )
    args = parser.parse_args()
    official_names = load_official_names()

    for project, names in PROJECTS.items():
        print(f"[{project.name}] mapped aliases: {len(names)}")
        for database_name in ("cbdes.db", "temp.db", "runtimeCanvas.db"):
            database = project / "data" / database_name
            baseline = None
            if args.baseline_timestamp:
                baseline = database.with_name(
                    f"{database_name}.before_chinese_names_{args.baseline_timestamp}"
                )
            report = inspect_database(database, names, official_names, baseline)
            print(
                f"  {database_name}: rows={report['rows']} "
                f"editable={report['editable_rows']} "
                f"planned={len(report['mismatches'])} missing={len(report['missing'])} "
                f"integrity={report['integrity']}"
            )
            if report["missing"]:
                raise RuntimeError(f"Unmapped aliases: {report['missing']}")

    if not args.apply:
        print("Dry run complete. Use --apply to write the validated mappings.")
        return

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    for project, names in PROJECTS.items():
        backups = backup_databases(project, timestamp)
        changed = 0
        for database_name in ("cbdes.db", "temp.db", "runtimeCanvas.db"):
            database = project / "data" / database_name
            baseline = None
            if args.baseline_timestamp:
                baseline = database.with_name(
                    f"{database_name}.before_chinese_names_{args.baseline_timestamp}"
                )
            changed += update_database(database, names, official_names, baseline)
        print(f"[{project.name}] updated rows across databases: {changed}")
        for backup in backups:
            print(f"  backup: {backup}")

        cbdes = project / "data" / "cbdes.db"
        temp = project / "data" / "temp.db"
        if cbdes.read_bytes() != temp.read_bytes():
            raise RuntimeError(f"cbdes.db and temp.db diverged after update: {project}")

    print("Chinese-name migration completed successfully.")


if __name__ == "__main__":
    main()
