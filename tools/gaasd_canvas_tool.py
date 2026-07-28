#!/usr/bin/env python3
"""Inspect, snapshot, and fingerprint GAASD canvas projects.

The tool intentionally keeps canvas edits out of its scope.  It provides the
repeatable safety checks that must run before and after a dedicated refactor
script or a manual GAASD edit.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import os
import re
import shutil
import sqlite3
import sys
from collections import Counter, defaultdict, deque
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Iterable


CORE_COMPONENTS = (
    "input",
    "output",
    "constant",
    "add",
    "subtract",
    "multiply",
    "divide",
    "fmax",
    "fmin",
    "less-than",
    "greater-than",
    "equal",
    "logic-and",
    "logic-or",
    "logic-not",
    "read-local-param",
    "read-local-state",
    "write-local-state",
    "truth-table",
    "oscilloscope",
)

DYNAMIC_COMPONENTS = {
    "input",
    "output",
    "constant",
    "add",
    "multiply",
    "logic-and",
    "logic-or",
    "read-local-param",
    "read-local-state",
    "write-local-state",
    "truth-table",
    "oscilloscope",
}


@dataclass
class Finding:
    code: str
    message: str
    context: dict[str, Any] = field(default_factory=dict)


@dataclass
class AuditReport:
    project: str
    database: str
    generated_at: str
    project_info: dict[str, Any]
    statistics: dict[str, Any]
    errors: list[Finding]
    warnings: list[Finding]
    notes: list[Finding]

    @property
    def ok(self) -> bool:
        return not self.errors

    def to_dict(self) -> dict[str, Any]:
        result = asdict(self)
        result["ok"] = self.ok
        return result


def now_iso() -> str:
    return dt.datetime.now().astimezone().isoformat(timespec="seconds")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def json_load(value: Any, default: Any) -> Any:
    if value in (None, ""):
        return default
    if isinstance(value, (dict, list)):
        return value
    try:
        return json.loads(value)
    except (TypeError, json.JSONDecodeError):
        return default


def extension_section(extension_props: Any, key: str, default: Any) -> Any:
    for item in json_load(extension_props, []):
        if not isinstance(item, dict):
            continue
        if item.get("key") == key:
            value = item.get("value", default)
            if key == "base_config" and isinstance(value, list):
                return value[0] if value else default
            return value
    return default


def version_key(value: Any) -> tuple[int, ...]:
    numbers = re.findall(r"\d+", str(value or ""))
    return tuple(int(number) for number in numbers) or (0,)


def resolve_project(value: str | Path) -> tuple[Path, Path]:
    path = Path(value).expanduser().resolve()
    if path.is_file() and path.name.endswith(".db"):
        project = path.parent.parent if path.parent.name == "data" else path.parent
        return project, path
    database = path / "data" / "cbdes.db"
    if not database.is_file():
        raise RuntimeError(f"Missing GAASD canvas database: {database}")
    return path, database


def find_component_db(explicit: str | Path | None) -> Path | None:
    candidates: list[Path] = []
    if explicit:
        candidates.append(Path(explicit).expanduser())
    if os.environ.get("GAASD_COMPONENT_DB"):
        candidates.append(Path(os.environ["GAASD_COMPONENT_DB"]).expanduser())
    home = Path.home()
    candidates.extend(
        (
            home / "gaasd_versions/gaasd-2.7.0.5/home/.gaasd/gaasd.db",
            home / ".gaasd/gaasd.db",
        )
    )
    return next((path.resolve() for path in candidates if path.is_file()), None)


def connect(path: Path) -> sqlite3.Connection:
    connection = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
    connection.row_factory = sqlite3.Row
    return connection


def component_ports(row: sqlite3.Row) -> tuple[set[str], set[str]]:
    inputs: set[str] = set()
    outputs: set[str] = set()
    for key, target in (
        ("function_input_ports", inputs),
        ("function_output_ports", outputs),
    ):
        for port in extension_section(row["extensionProps"], key, []):
            if not isinstance(port, dict):
                continue
            for candidate in (port.get("id"), port.get("name"), port.get("alias")):
                if candidate:
                    target.add(str(candidate))
    return inputs, outputs


def project_metadata(connection: sqlite3.Connection) -> dict[str, Any]:
    if not table_exists(connection, "project_info"):
        return {}
    row = connection.execute("SELECT * FROM project_info LIMIT 1").fetchone()
    if not row:
        return {}
    result = dict(row)
    result["properties"] = json_load(result.get("properties"), {})
    return result


def table_exists(connection: sqlite3.Connection, name: str) -> bool:
    return bool(
        connection.execute(
            "SELECT 1 FROM sqlite_master WHERE type='table' AND name=?", (name,)
        ).fetchone()
    )


def current_templates(component_db: Path | None) -> dict[tuple[str, str], dict[str, Any]]:
    if component_db is None:
        return {}
    with connect(component_db) as connection:
        rows = connection.execute("SELECT * FROM component").fetchall()
    selected: dict[tuple[str, str], dict[str, Any]] = {}
    for row in rows:
        item = dict(row)
        key = (str(item.get("originId") or ""), str(item.get("name") or ""))
        previous = selected.get(key)
        rank = (
            version_key(item.get("version")),
            item.get("vendor") == "default",
            not bool(item.get("isCustom")),
        )
        if previous is None:
            selected[key] = item
            continue
        previous_rank = (
            version_key(previous.get("version")),
            previous.get("vendor") == "default",
            not bool(previous.get("isCustom")),
        )
        if rank > previous_rank:
            selected[key] = item
    return selected


def find_cycles(
    components: dict[str, sqlite3.Row], connections: list[sqlite3.Row]
) -> list[dict[str, Any]]:
    by_parent: dict[str, set[str]] = defaultdict(set)
    edges: dict[str, list[tuple[str, str]]] = defaultdict(list)
    for component in components.values():
        by_parent[str(component["parentId"] or "")].add(component["id"])
    for connection in connections:
        source = json_load(connection["source"], {})
        target = json_load(connection["target"], {})
        if source.get("cell") and target.get("cell"):
            edges[str(connection["parentId"] or "")].append(
                (str(source["cell"]), str(target["cell"]))
            )

    result: list[dict[str, Any]] = []
    for parent, nodes in by_parent.items():
        indegree = {node: 0 for node in nodes}
        outgoing: dict[str, set[str]] = defaultdict(set)
        for source, target in edges.get(parent, []):
            if source in nodes and target in nodes and target not in outgoing[source]:
                outgoing[source].add(target)
                indegree[target] += 1
        queue = deque(node for node, degree in indegree.items() if degree == 0)
        visited = 0
        while queue:
            node = queue.popleft()
            visited += 1
            for target in outgoing[node]:
                indegree[target] -= 1
                if indegree[target] == 0:
                    queue.append(target)
        if visited != len(nodes):
            aliases = sorted(
                components[node]["alias"] or components[node]["name"] or node
                for node, degree in indegree.items()
                if degree > 0
            )
            result.append({"parentId": parent, "components": aliases})
    return result


def rectangle(row: sqlite3.Row, parent_id: str) -> tuple[float, float, float, float] | None:
    attribute = json_load(row["attribute"], {})
    if not isinstance(attribute, dict):
        return None
    canvas = attribute.get(parent_id)
    if not isinstance(canvas, dict):
        return None
    position = canvas.get("position", {})
    size = canvas.get("size", {})
    try:
        return (
            float(position["x"]),
            float(position["y"]),
            float(size["width"]),
            float(size["height"]),
        )
    except (KeyError, TypeError, ValueError):
        return None


def find_overlaps(components: dict[str, sqlite3.Row]) -> list[dict[str, Any]]:
    siblings: dict[str, list[sqlite3.Row]] = defaultdict(list)
    for row in components.values():
        siblings[str(row["parentId"] or "")].append(row)
    overlaps: list[dict[str, Any]] = []
    margin = 4.0
    for parent, rows in siblings.items():
        placed = [(row, rectangle(row, parent)) for row in rows]
        placed = [(row, rect) for row, rect in placed if rect is not None]
        for index, (left, a) in enumerate(placed):
            ax, ay, aw, ah = a
            for right, b in placed[index + 1 :]:
                bx, by, bw, bh = b
                intersects = (
                    ax + margin < bx + bw
                    and bx + margin < ax + aw
                    and ay + margin < by + bh
                    and by + margin < ay + ah
                )
                if intersects:
                    overlaps.append(
                        {
                            "parentId": parent,
                            "left": left["alias"] or left["name"],
                            "right": right["alias"] or right["name"],
                        }
                    )
    return overlaps


def validate_truth_table(row: sqlite3.Row) -> list[Finding]:
    findings: list[Finding] = []
    config = extension_section(row["extensionProps"], "base_config", {})
    conditions = config.get("conditions", [])
    actions = config.get("actions", [])
    scenarios = config.get("scenarios", [])
    condition_ids = {item.get("id") for item in conditions}
    action_ids = {item.get("id") for item in actions}
    alias = row["alias"] or row["name"]
    if not conditions or not actions or not scenarios:
        findings.append(Finding("truth_table_empty", f"{alias} configuration is incomplete"))
        return findings
    default_count = 0
    for scenario in scenarios:
        matching = scenario.get("condition_matching", {})
        if set(matching) != condition_ids:
            findings.append(
                Finding(
                    "truth_table_condition_mismatch",
                    f"{alias} scenario does not cover exactly the configured conditions",
                    {"scenario": scenario.get("name") or scenario.get("id")},
                )
            )
        if any(value not in (-1, 0, 1) for value in matching.values()):
            findings.append(
                Finding(
                    "truth_table_invalid_match",
                    f"{alias} contains a match value outside -1/0/1",
                )
            )
        if scenario.get("action_matching") not in action_ids:
            findings.append(
                Finding(
                    "truth_table_missing_action",
                    f"{alias} scenario references a missing action",
                    {"scenario": scenario.get("name") or scenario.get("id")},
                )
            )
        if matching and all(value == -1 for value in matching.values()):
            default_count += 1
    if default_count != 1:
        findings.append(
            Finding(
                "truth_table_default_count",
                f"{alias} should contain exactly one all-don't-care default scenario",
                {"count": default_count},
            )
        )
    return findings


def audit_project(project_value: str | Path, component_db: Path | None) -> AuditReport:
    project, database = resolve_project(project_value)
    errors: list[Finding] = []
    warnings: list[Finding] = []
    notes: list[Finding] = []

    with connect(database) as connection:
        integrity = connection.execute("PRAGMA integrity_check").fetchone()[0]
        if integrity != "ok":
            errors.append(Finding("sqlite_integrity", str(integrity)))
        for table in ("project_component", "project_connection", "project_info"):
            if not table_exists(connection, table):
                errors.append(Finding("missing_table", f"Missing table: {table}"))
        if errors:
            return AuditReport(
                str(project), str(database), now_iso(), {}, {}, errors, warnings, notes
            )

        component_rows = connection.execute("SELECT * FROM project_component").fetchall()
        connection_rows = connection.execute("SELECT * FROM project_connection").fetchall()
        components = {row["id"]: row for row in component_rows}
        metadata = project_metadata(connection)

    bad_json = 0
    dangling = 0
    cross_boundary = 0
    duplicate_targets: Counter[tuple[str, str]] = Counter()
    unknown_source_ports = 0
    unknown_target_ports = 0
    port_cache = {identifier: component_ports(row) for identifier, row in components.items()}
    for edge in connection_rows:
        source = json_load(edge["source"], None)
        target = json_load(edge["target"], None)
        if not isinstance(source, dict) or not isinstance(target, dict):
            bad_json += 1
            continue
        source_row = components.get(source.get("cell"))
        target_row = components.get(target.get("cell"))
        if source_row is None or target_row is None:
            dangling += 1
            continue
        if source_row["parentId"] != edge["parentId"] or target_row["parentId"] != edge["parentId"]:
            cross_boundary += 1
        duplicate_targets[(str(edge["parentId"]), json.dumps(target, sort_keys=True))] += 1
        source_port = str(source.get("port") or "")
        target_port = str(target.get("port") or "")
        if source_port and source_port not in port_cache[source_row["id"]][1]:
            unknown_source_ports += 1
        if target_port and target_port not in port_cache[target_row["id"]][0]:
            unknown_target_ports += 1

    if bad_json:
        errors.append(Finding("invalid_connection_json", "Connections contain invalid JSON", {"count": bad_json}))
    if dangling:
        errors.append(Finding("dangling_endpoint", "Connections reference missing components", {"count": dangling}))
    if cross_boundary:
        errors.append(Finding("cross_boundary", "Connections bypass a subsystem boundary", {"count": cross_boundary}))
    duplicated = sum(1 for count in duplicate_targets.values() if count > 1)
    if duplicated:
        errors.append(Finding("multiple_drivers", "Input ports have more than one driver", {"count": duplicated}))
    if unknown_source_ports or unknown_target_ports:
        warnings.append(
            Finding(
                "unknown_ports",
                "Some connection port IDs are absent from the instance metadata",
                {"sources": unknown_source_ports, "targets": unknown_target_ports},
            )
        )

    root_parent_ids = {
        str(metadata.get("projectId") or ""),
        str(metadata.get("id") or ""),
        "",
    }
    missing_parents = [
        row for row in component_rows if row["parentId"] not in components and str(row["parentId"] or "") not in root_parent_ids
    ]
    if missing_parents:
        errors.append(
            Finding(
                "missing_parent",
                "Components reference a missing parent",
                {"aliases": [row["alias"] or row["name"] for row in missing_parents[:20]]},
            )
        )

    cycles = find_cycles(components, connection_rows)
    if cycles:
        errors.append(Finding("combinational_cycle", "Canvas contains directed cycles", {"cycles": cycles[:10]}))

    overlaps = find_overlaps(components)
    if overlaps:
        warnings.append(
            Finding(
                "layout_overlap",
                "Sibling components overlap in the stored layout",
                {"count": len(overlaps), "examples": overlaps[:15]},
            )
        )

    for row in component_rows:
        name = str(row["name"] or "")
        alias = row["alias"] or name
        base = extension_section(row["extensionProps"], "base_config", {})
        input_port_entries = extension_section(
            row["extensionProps"], "function_input_ports", []
        )
        if name in {"add", "multiply", "logic-and", "logic-or", "oscilloscope"}:
            expected = base.get("inputNumber")
            if isinstance(expected, int) and expected != len(input_port_entries):
                warnings.append(
                    Finding(
                        "dynamic_port_count",
                        f"{alias} inputNumber differs from stored input ports",
                        {"configured": expected, "ports": len(input_port_entries)},
                    )
                )
        if name in {"input", "output"}:
            missing = [key for key in ("name", "dataType") if base.get(key) in (None, "")]
            if missing:
                warnings.append(Finding("boundary_config", f"{alias} lacks boundary metadata", {"missing": missing}))
        if name == "constant":
            missing = [key for key in ("name", "dataType", "dataValue") if key not in base]
            if missing:
                warnings.append(Finding("constant_config", f"{alias} lacks constant metadata", {"missing": missing}))
        if name in {"read-local-param", "read-local-state", "write-local-state"}:
            keys = base.get("operateKeys", [])
            if not isinstance(keys, list) or len(keys) != 1:
                warnings.append(Finding("state_param_binding", f"{alias} should bind exactly one field", {"operateKeys": keys}))
        if name == "truth-table":
            errors.extend(validate_truth_table(row))

    temp_database = project / "data" / "temp.db"
    hash_status: dict[str, Any] = {"cbdes.db": sha256(database)}
    if temp_database.is_file():
        hash_status["temp.db"] = sha256(temp_database)
        if hash_status["cbdes.db"] != hash_status["temp.db"]:
            warnings.append(Finding("database_copy_drift", "cbdes.db and temp.db differ"))
    else:
        warnings.append(Finding("missing_temp_db", f"Missing {temp_database}"))

    templates = current_templates(component_db)
    drifted: list[dict[str, Any]] = []
    risky_drifted: list[dict[str, Any]] = []
    if templates:
        for row in component_rows:
            key = (str(row["originId"] or ""), str(row["name"] or ""))
            template = templates.get(key)
            if not template:
                continue
            if str(row["version"] or "") != str(template.get("version") or "") or int(row["isCustom"] or 0) != int(template.get("isCustom") or 0):
                item = {
                    "alias": row["alias"] or row["name"],
                    "name": row["name"],
                    "projectVersion": row["version"],
                    "libraryVersion": template.get("version"),
                    "projectCustom": row["isCustom"],
                    "libraryCustom": template.get("isCustom"),
                }
                drifted.append(item)
                if row["name"] in DYNAMIC_COMPONENTS:
                    risky_drifted.append(item)
        if drifted:
            notes.append(
                Finding(
                    "component_template_drift",
                    "Project instances differ from the current component library; this is not automatically an error",
                    {"count": len(drifted), "dynamicCount": len(risky_drifted), "examples": risky_drifted[:15] or drifted[:15]},
                )
            )
    else:
        notes.append(Finding("component_db_unavailable", "Component library comparison was skipped"))

    statistics = {
        "components": len(component_rows),
        "connections": len(connection_rows),
        "componentTypes": dict(Counter(str(row["componentType"] or row["name"]) for row in component_rows)),
        "componentVersions": dict(Counter(str(row["version"] or "") for row in component_rows)),
        "customComponents": sum(int(row["isCustom"] or 0) != 0 for row in component_rows),
        "databaseHashes": hash_status,
        "templateDrift": len(drifted),
        "dynamicTemplateDrift": len(risky_drifted),
    }
    return AuditReport(
        str(project), str(database), now_iso(), metadata, statistics, errors, warnings, notes
    )


def app_fingerprint(path: Path) -> dict[str, Any]:
    result: dict[str, Any] = {"path": str(path)}
    version_file = path / "version"
    package_file = path / "resources/app/package.json"
    if version_file.is_file():
        result["buildVersion"] = version_file.read_text(encoding="utf-8", errors="replace").strip()
        result["versionSha256"] = sha256(version_file)
    if package_file.is_file():
        package = json_load(package_file.read_text(encoding="utf-8"), {})
        result["packageVersion"] = package.get("version")
        result["packageSha256"] = sha256(package_file)
    return result


def environment_fingerprint(
    component_db: Path | None, project: str | Path | None = None
) -> dict[str, Any]:
    app_dirs: list[Path] = []
    configured = os.environ.get("GAASD_APP_DIRS")
    if configured:
        app_dirs.extend(Path(item).expanduser() for item in configured.split(os.pathsep))
    app_dirs.extend(
        (
            Path("/opt/gaasd"),
            Path.home() / "gaasd_versions/gaasd-2.7.0.5/app",
        )
    )
    unique_apps = []
    seen: set[str] = set()
    for path in app_dirs:
        resolved = str(path.resolve())
        if path.is_dir() and resolved not in seen:
            unique_apps.append(app_fingerprint(path.resolve()))
            seen.add(resolved)

    result: dict[str, Any] = {
        "generatedAt": now_iso(),
        "apps": unique_apps,
        "componentLibrary": None,
    }
    if component_db:
        templates = current_templates(component_db)
        result["componentLibrary"] = {
            "path": str(component_db),
            "sha256": sha256(component_db),
            "coreComponents": {
                name: [
                    {
                        "originId": item.get("originId"),
                        "version": item.get("version"),
                        "vendor": item.get("vendor"),
                        "isCustom": item.get("isCustom"),
                    }
                    for (origin, component_name), item in templates.items()
                    if component_name == name
                ]
                for name in CORE_COMPONENTS
            },
        }
    if project:
        project_path, database = resolve_project(project)
        result["project"] = {
            "path": str(project_path),
            "cbdesSha256": sha256(database),
            "tempSha256": sha256(project_path / "data/temp.db") if (project_path / "data/temp.db").is_file() else None,
        }
        with connect(database) as connection:
            result["project"]["projectInfo"] = project_metadata(connection)
    return result


def catalog_markdown(component_db: Path) -> str:
    with connect(component_db) as connection:
        components = connection.execute("SELECT * FROM component").fetchall()
        details = connection.execute("SELECT * FROM component_detail").fetchall()
    detail_by_origin = {row["originId"]: row for row in details}
    lines = [
        "# GAASD 基础组件当前版本快照",
        "",
        f"> 生成时间：`{now_iso()}`  ",
        f"> 组件库：`{component_db}`  ",
        f"> SHA256：`{sha256(component_db)}`",
        "",
        "该文件是版本快照，不是永久规范。GAASD 更新后应重新生成并比较。",
        "",
        "| 组件 | 版本 | vendor | originId | 配置字段 | 输入端口 | 输出端口 |",
        "| --- | --- | --- | --- | --- | --- | --- |",
    ]
    for name in CORE_COMPONENTS:
        matches = [row for row in components if row["name"] == name]
        matches.sort(key=lambda row: (version_key(row["version"]), row["vendor"] == "default"), reverse=True)
        if not matches:
            lines.append(f"| `{name}` | - | - | - | - | - | - |")
            continue
        row = matches[0]
        detail = detail_by_origin.get(row["originId"])
        extension = detail["extensionProps"] if detail else "[]"
        base = extension_section(extension, "base_config", {})
        inputs = extension_section(extension, "function_input_ports", [])
        outputs = extension_section(extension, "function_output_ports", [])
        config_keys = ", ".join(f"`{key}`" for key in base) or "-"
        input_names = ", ".join(f"`{item.get('name')}`" for item in inputs) or "-"
        output_names = ", ".join(f"`{item.get('name')}`" for item in outputs) or "-"
        lines.append(
            f"| `{name}` | `{row['version']}` | `{row['vendor']}` | `{row['originId']}` | {config_keys} | {input_names} | {output_names} |"
        )
    lines.append("")
    return "\n".join(lines)


def gaasd_processes() -> list[str]:
    matches: list[str] = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            parts = (entry / "cmdline").read_bytes().split(b"\0")
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        command = parts[0].decode(errors="replace") if parts else ""
        if Path(command).name == "gaasd":
            matches.append(" ".join(part.decode(errors="replace") for part in parts if part))
    return matches


def find_workspace(path: Path) -> Path:
    for parent in (path, *path.parents):
        if (parent / "WORKLOG.md").is_file():
            return parent
    return path.parent


def snapshot_project(
    project_value: str | Path,
    component_db: Path | None,
    backup_root: str | Path | None,
    label: str,
    allow_running: bool,
) -> Path:
    project, _ = resolve_project(project_value)
    running = gaasd_processes()
    if running and not allow_running:
        raise RuntimeError(
            "GAASD is running. Close it before a full project snapshot, or use --allow-running only if the risk is understood."
        )
    workspace = find_workspace(project)
    root = Path(backup_root).expanduser().resolve() if backup_root else workspace.with_name(workspace.name + "_backups")
    timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    destination = root / f"{project.name}_{label}_{timestamp}"
    root.mkdir(parents=True, exist_ok=True)
    shutil.copytree(project, destination, symlinks=True)
    report = audit_project(project, component_db)
    manifest = {
        "snapshotCreatedAt": now_iso(),
        "source": str(project),
        "destination": str(destination),
        "gaasdWasRunning": running,
        "environment": environment_fingerprint(component_db, project),
        "audit": report.to_dict(),
    }
    (destination / "canvas_snapshot_manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    return destination


def flatten_diff(left: Any, right: Any, prefix: str = "") -> list[str]:
    if isinstance(left, dict) and isinstance(right, dict):
        differences: list[str] = []
        for key in sorted(set(left) | set(right)):
            path = f"{prefix}.{key}" if prefix else key
            if key not in left:
                differences.append(f"+ {path} = {right[key]!r}")
            elif key not in right:
                differences.append(f"- {path} = {left[key]!r}")
            else:
                differences.extend(flatten_diff(left[key], right[key], path))
        return differences
    if left != right:
        return [f"~ {prefix}: {left!r} -> {right!r}"]
    return []


def print_report(report: AuditReport) -> None:
    status = "PASS" if report.ok else "FAIL"
    print(f"[{status}] {report.project}")
    print(f"  components:  {report.statistics.get('components', 0)}")
    print(f"  connections: {report.statistics.get('connections', 0)}")
    print(f"  errors:      {len(report.errors)}")
    print(f"  warnings:    {len(report.warnings)}")
    for label, findings in (("ERROR", report.errors), ("WARN", report.warnings), ("NOTE", report.notes)):
        for item in findings:
            context = f" {json.dumps(item.context, ensure_ascii=False)}" if item.context else ""
            print(f"  {label} {item.code}: {item.message}{context}")


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--component-db", help="Path to the active GAASD gaasd.db")
    subparsers = parser.add_subparsers(dest="command", required=True)

    audit = subparsers.add_parser("audit", help="Read-only canvas structure audit")
    audit.add_argument("project")
    audit.add_argument("--json-out")

    snapshot = subparsers.add_parser("snapshot", help="Copy a restorable project snapshot")
    snapshot.add_argument("project")
    snapshot.add_argument("--backup-root")
    snapshot.add_argument("--label", default="canvas_snapshot")
    snapshot.add_argument("--allow-running", action="store_true")

    catalog = subparsers.add_parser("catalog", help="Export the active core component catalog")
    catalog.add_argument("--output", required=True)

    fingerprint = subparsers.add_parser("fingerprint", help="Save GAASD and project version fingerprints")
    fingerprint.add_argument("--project")
    fingerprint.add_argument("--output", required=True)

    compare = subparsers.add_parser("compare", help="Compare a saved fingerprint with the current environment")
    compare.add_argument("baseline")
    compare.add_argument("--project")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    component_db = find_component_db(args.component_db)
    if args.command == "audit":
        report = audit_project(args.project, component_db)
        print_report(report)
        if args.json_out:
            write_json(Path(args.json_out), report.to_dict())
        return 0 if report.ok else 1
    if args.command == "snapshot":
        destination = snapshot_project(
            args.project,
            component_db,
            args.backup_root,
            args.label,
            args.allow_running,
        )
        print(destination)
        return 0
    if args.command == "catalog":
        if component_db is None:
            raise RuntimeError("No GAASD component database was found")
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(catalog_markdown(component_db), encoding="utf-8")
        print(output.resolve())
        return 0
    if args.command == "fingerprint":
        output = Path(args.output)
        write_json(output, environment_fingerprint(component_db, args.project))
        print(output.resolve())
        return 0
    if args.command == "compare":
        baseline = json_load(Path(args.baseline).read_text(encoding="utf-8"), {})
        current = environment_fingerprint(component_db, args.project)
        baseline.pop("generatedAt", None)
        current.pop("generatedAt", None)
        differences = flatten_diff(baseline, current)
        if differences:
            print("\n".join(differences))
            return 2
        print("No environment or project fingerprint changes detected")
        return 0
    return 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(1)
