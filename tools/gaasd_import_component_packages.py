#!/usr/bin/env python3
"""Import GAASD THICV component packages into an isolated GAASD install.

The GAASD GUI importer runs inside Electron with an initialized ORM. Calling the
compiled JS importer directly from plain node does not reliably update gaasd.db,
so this script mirrors the relevant import behavior with sqlite.
"""

from __future__ import annotations

import argparse
import copy
import datetime as _dt
import io
import json
import os
from pathlib import Path
import shutil
import sqlite3
import tarfile
import uuid


COMPONENT_COLUMNS = [
    "originId",
    "name",
    "cppClass",
    "alias",
    "version",
    "author",
    "abstractId",
    "componentType",
    "description",
    "tags",
    "isSourceCode",
    "isAtomic",
    "isCustom",
    "customType",
    "namespace",
    "vendor",
]


def safe_extract(tar: tarfile.TarFile, target: Path) -> None:
    target_resolved = target.resolve()
    for member in tar.getmembers():
        member_path = (target / member.name).resolve()
        if target_resolved != member_path and target_resolved not in member_path.parents:
            raise RuntimeError(f"Unsafe tar path: {member.name}")
    try:
        tar.extractall(target, filter="data")
    except TypeError:
        tar.extractall(target)


def unpack_component_package(package_path: Path, output_dir: Path) -> Path:
    thicv_dir = output_dir / "THICV"
    thicv_dir.mkdir(parents=True, exist_ok=True)
    with tarfile.open(package_path) as outer:
        names = set(outer.getnames())
        if "THICV.tar" not in names:
            raise RuntimeError(f"{package_path} does not contain THICV.tar")
        data = outer.extractfile("THICV.tar")
        if data is None:
            raise RuntimeError(f"Cannot read THICV.tar from {package_path}")
        with tarfile.open(fileobj=io.BytesIO(data.read())) as inner:
            safe_extract(inner, thicv_dir)
    return thicv_dir


def copy_overlay(src: Path, dst: Path) -> None:
    for root, dirs, files in os.walk(src):
        rel = Path(root).relative_to(src)
        out_root = dst / rel
        out_root.mkdir(parents=True, exist_ok=True)
        for dirname in dirs:
            (out_root / dirname).mkdir(exist_ok=True)
        for filename in files:
            shutil.copy2(Path(root) / filename, out_root / filename)


def read_component_rows(db_path: Path) -> dict[str, dict[str, object]]:
    rows: dict[str, dict[str, object]] = {}
    if not db_path.exists():
        return rows
    con = sqlite3.connect(db_path)
    con.row_factory = sqlite3.Row
    try:
        for row in con.execute("select * from component"):
            item = dict(row)
            origin_id = str(item.get("originId") or "").strip()
            if origin_id:
                rows[origin_id] = item
    finally:
        con.close()
    return rows


def merge_component_db(base_db: Path, package_dbs: list[Path], output_db: Path) -> int:
    if not base_db.exists():
        raise RuntimeError(f"Base cbdes.db missing: {base_db}")
    shutil.copy2(base_db, output_db)
    merged: dict[str, dict[str, object]] = {}
    for db_path in [base_db, *package_dbs]:
        merged.update(read_component_rows(db_path))

    con = sqlite3.connect(output_db)
    try:
        con.execute("delete from component")
        source_cols = [
            row[1]
            for row in con.execute("pragma table_info(component)").fetchall()
        ]
        placeholders = ",".join(["?"] * len(source_cols))
        insert_sql = (
            f"insert into component ({','.join(source_cols)}) values ({placeholders})"
        )
        for index, row in enumerate(merged.values(), start=1):
            row = dict(row)
            row["id"] = index
            values = [row.get(col) for col in source_cols]
            con.execute(insert_sql, values)
        con.commit()
    finally:
        con.close()
    return len(merged)


def normalize_list(value: object) -> list[object]:
    return value if isinstance(value, list) else []


def normalize_tags(value: object) -> str:
    if isinstance(value, list):
        values = [str(item) for item in value if item]
        return ",".join(values) if values else "common"
    if value:
        return str(value)
    return "common"


def json_dumps(value: object) -> str:
    return json.dumps(value, ensure_ascii=False, separators=(",", ":"))


def component_table_rows(cbdes_db: Path) -> list[dict[str, object]]:
    con = sqlite3.connect(cbdes_db)
    con.row_factory = sqlite3.Row
    try:
        rows = []
        for row in con.execute("select * from component"):
            item = dict(row)
            rows.append(
                {
                    "id": str(uuid.uuid4()),
                    "originId": item.get("originId") or "",
                    "name": item.get("name") or "",
                    "cppClass": item.get("cppClass") or "",
                    "alias": item.get("alias") or "",
                    "version": item.get("version") or "",
                    "author": item.get("author") or "",
                    "abstractId": item.get("abstractId") or "",
                    "componentType": item.get("componentType") or "",
                    "description": item.get("description") or "",
                    "tags": normalize_tags(item.get("tags")),
                    "isSourceCode": int(item.get("isSourceCode") or 0),
                    "isAtomic": int(item.get("isAtomic") or 0),
                    "isCustom": int(item.get("isCustom") or 0),
                    "customType": int(item.get("customType") or 0),
                    "namespace": item.get("namespace") or "",
                    "vendor": item.get("vendor") or "THICV",
                }
            )
        return rows
    finally:
        con.close()


def detail_rows_for_component(data: dict[str, object], vendor: str) -> list[dict[str, object]]:
    detail_rows: list[dict[str, object]] = []
    function_data = copy.deepcopy(data)
    child_ids: list[str] = []

    for child in normalize_list(function_data.get("childComponents")):
        if not isinstance(child, dict):
            continue
        child_data = copy.deepcopy(child)
        child_data["parentId"] = function_data.get("originId") or ""
        child_key = child_data.get("instanceId") or child_data.get("originId")
        if child_key:
            child_ids.append(str(child_key))
        detail_rows.extend(detail_rows_for_component(child_data, vendor))

    function_data["childComponents"] = child_ids
    vendor_value = str(function_data.get("vendor") or vendor or "THICV")
    props = {
        "name": function_data.get("name") or "",
        "cppClass": function_data.get("cppClass") or "",
        "alias": function_data.get("alias") or "",
        "version": function_data.get("version") or "",
        "author": function_data.get("author") or "",
        "abstractId": function_data.get("abstractId") or "",
        "componentType": function_data.get("componentType") or "",
        "description": function_data.get("description") or "",
        "tags": normalize_tags(function_data.get("tags")),
        "isSourceCode": int(function_data.get("isSourceCode") or 0),
        "isAtomic": int(function_data.get("isAtomic") or 0),
        "isCustom": int(function_data.get("isCustom") or 0),
        "customType": int(function_data.get("customType") or 0),
        "namespace": function_data.get("namespace") or "",
        "file_path": function_data.get("file_path"),
        "base_path": function_data.get("base_path"),
        "out_path": function_data.get("out_path"),
        "configProto": function_data.get("configProto") or "",
        "input_base_path": function_data.get("input_base_path"),
        "appname": function_data.get("appname"),
        "soc_name": function_data.get("soc_name"),
        "process_name": function_data.get("process_name"),
        "origin_node_name": function_data.get("origin_node_name"),
        "node_name": function_data.get("node_name"),
        "bind_cpu": function_data.get("bind_cpu"),
        "node_priority": function_data.get("node_priority"),
        "depend_nodes": normalize_list(function_data.get("depend_nodes")),
        "conf_filename": function_data.get("conf_filename"),
        "children": function_data.get("children") or [],
        "parent": function_data.get("parent"),
        "vendor": vendor_value,
    }
    detail_rows.append(
        {
            "id": function_data.get("id") or str(uuid.uuid4()),
            "instanceId": function_data.get("instanceId") or "",
            "originId": function_data.get("originId") or "",
            "parentId": function_data.get("parentId") or "",
            "properties": json_dumps(props),
            "childComponents": json_dumps(function_data.get("childComponents") or []),
            "childConnections": json_dumps(function_data.get("childConnections") or []),
            "extensionProps": json_dumps(function_data.get("extensionProps") or []),
            "vendor": vendor_value,
        }
    )
    return detail_rows


def component_detail_rows(component_dir: Path, vendor: str) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for json_path in sorted(component_dir.glob("*.json")):
        data = json.loads(json_path.read_text(encoding="utf-8"))
        rows.extend(detail_rows_for_component(data, vendor))
    return rows


def replace_rows(db_path: Path, staging_thicv: Path, vendor: str) -> tuple[int, int]:
    components = component_table_rows(staging_thicv / "cbdes.db")
    details = component_detail_rows(staging_thicv / "component", vendor)
    con = sqlite3.connect(db_path)
    try:
        con.execute("delete from component where vendor = ?", (vendor,))
        con.execute("delete from component_detail where vendor = ?", (vendor,))
        con.executemany(
            """
            insert into component
            (id, originId, name, cppClass, alias, version, author, abstractId,
             componentType, description, tags, isSourceCode, isAtomic, isCustom,
             customType, namespace, vendor)
            values
            (:id, :originId, :name, :cppClass, :alias, :version, :author,
             :abstractId, :componentType, :description, :tags, :isSourceCode,
             :isAtomic, :isCustom, :customType, :namespace, :vendor)
            """,
            components,
        )
        con.executemany(
            """
            insert into component_detail
            (id, instanceId, originId, parentId, properties, childComponents,
             childConnections, extensionProps, vendor)
            values
            (:id, :instanceId, :originId, :parentId, :properties,
             :childComponents, :childConnections, :extensionProps, :vendor)
            """,
            details,
        )
        con.commit()
    finally:
        con.close()
    return len(components), len(details)


def build_staging(current_thicv: Path, packages: list[Path], staging: Path) -> tuple[Path, int]:
    if staging.exists():
        shutil.rmtree(staging)
    staging.mkdir(parents=True)
    staging_thicv = staging / "THICV"
    shutil.copytree(current_thicv, staging_thicv)

    package_dirs: list[Path] = []
    for index, package in enumerate(packages):
        pkg_dir = staging / f"pkg_{index}"
        package_thicv = unpack_component_package(package, pkg_dir)
        package_dirs.append(package_thicv)
        copy_overlay(package_thicv, staging_thicv)

    merged_count = merge_component_db(
        current_thicv / "cbdes.db",
        [pkg / "cbdes.db" for pkg in package_dirs],
        staging_thicv / "cbdes.db",
    )
    return staging_thicv, merged_count


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--gaasd-root",
        default="/home/aiden/gaasd_versions/gaasd-2.7.0.5",
        help="Isolated GAASD root directory",
    )
    parser.add_argument("--package", action="append", required=True)
    parser.add_argument("--vendor", default="THICV")
    parser.add_argument("--apply", action="store_true")
    args = parser.parse_args()

    gaasd_root = Path(args.gaasd_root)
    vendor = args.vendor
    current_thicv = gaasd_root / "home" / "gaasd_server" / "components" / vendor
    gaasd_db = gaasd_root / "home" / ".gaasd" / "gaasd.db"
    packages = [Path(p) for p in args.package]
    staging = Path("/tmp") / f"gaasd_component_import_{_dt.datetime.now():%Y%m%d_%H%M%S}"

    for path in [current_thicv, gaasd_db, *packages]:
        if not path.exists():
            raise RuntimeError(f"Path not found: {path}")

    staging_thicv, merged_count = build_staging(current_thicv, packages, staging)
    component_count = len(list((staging_thicv / "component").glob("*.json")))
    dict_count = len(list((staging_thicv / "dictionaryData").glob("*.json")))

    print(f"staging={staging_thicv}")
    print(f"merged top components={merged_count}")
    print(f"component json files={component_count}")
    print(f"dictionary json files={dict_count}")

    if not args.apply:
        print("dry-run only; pass --apply to update GAASD")
        return 0

    timestamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    rollback = gaasd_root / "rollback" / f"component_import_{timestamp}"
    rollback.mkdir(parents=True, exist_ok=False)
    shutil.copytree(current_thicv, rollback / vendor)
    shutil.copy2(gaasd_db, rollback / "gaasd.db")

    shutil.rmtree(current_thicv)
    shutil.copytree(staging_thicv, current_thicv)
    comp_rows, detail_rows = replace_rows(gaasd_db, current_thicv, vendor)

    print(f"rollback={rollback}")
    print(f"updated component rows={comp_rows}")
    print(f"updated component_detail rows={detail_rows}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
