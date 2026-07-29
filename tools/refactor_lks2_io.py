#!/usr/bin/env python3
"""Add the official LKS channels and current oscilloscope to lks2 safely."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import shutil
import sqlite3
import sys
import uuid
from pathlib import Path
from typing import Any


ROOT_ALIAS = "lks"
MODULE_ALIAS = "空模块"
SCOPE_ALIAS = "仿真示波器"
LKS_INPUT_INSTANCE = "channel_input_lks_input_0.0.1_a6ef1d12"
LKS_OUTPUT_INSTANCE = "channel_output_lks_output_0.0.1_e8a12f40"
CONNECTION_EXTENSION = [
    {
        "key": "base_config",
        "value": [],
        "props": [{"key": "connectionType", "value": 0, "props": []}],
    }
]


def json_text(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, separators=(",", ":"))


def stable_id(project_id: str, label: str) -> str:
    return str(uuid.uuid5(uuid.NAMESPACE_URL, f"gaasd:{project_id}:lks-io:{label}"))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def parse_sections(raw: str) -> tuple[list[dict[str, Any]], dict[str, dict[str, Any]]]:
    sections = json.loads(raw or "[]")
    indexed = {item["key"]: item for item in sections}
    return sections, indexed


def props_value(port: dict[str, Any], key: str) -> Any:
    for prop in port.get("props", []):
        if prop.get("key") == key:
            return prop.get("value")
    return None


def set_prop(port: dict[str, Any], key: str, value: Any) -> None:
    for prop in port.setdefault("props", []):
        if prop.get("key") == key:
            prop["value"] = value
            return
    port["props"].append({"key": key, "value": value, "props": []})


def port_by_name(extension: str, section: str, name: str) -> dict[str, Any]:
    _, indexed = parse_sections(extension)
    for port in indexed[section].get("value", []):
        if port.get("name") == name:
            return port
    raise RuntimeError(f"Missing {section} port {name}")


def detail_by_instance(library: sqlite3.Connection, instance_id: str) -> sqlite3.Row:
    row = library.execute(
        "SELECT * FROM component_detail WHERE instanceId=? AND parentId=''",
        (instance_id,),
    ).fetchone()
    if row is None:
        row = library.execute(
            "SELECT * FROM component_detail WHERE instanceId=? LIMIT 1",
            (instance_id,),
        ).fetchone()
    if row is None:
        raise RuntimeError(f"Component detail not found: {instance_id}")
    return row


def generic_detail(
    library: sqlite3.Connection, name: str, version: str = "1.2.0"
) -> sqlite3.Row:
    component = library.execute(
        "SELECT * FROM component WHERE name=? AND version=?", (name, version)
    ).fetchone()
    if component is None:
        raise RuntimeError(f"Library component not found: {name} {version}")
    detail = library.execute(
        """
        SELECT * FROM component_detail
        WHERE originId=? AND instanceId=''
          AND json_extract(properties, '$.version')=?
        LIMIT 1
        """,
        (component["originId"], version),
    ).fetchone()
    if detail is None:
        raise RuntimeError(f"Library component detail not found: {name} {version}")
    return detail


def project_row_from_detail(
    detail: sqlite3.Row,
    *,
    component_id: str,
    project_id: str,
    parent_id: str,
    position: tuple[float, float],
    size: tuple[float, float],
    z_index: int,
) -> dict[str, Any]:
    properties = json.loads(detail["properties"])
    tags = properties.get("tags", "")
    if isinstance(tags, list):
        tags = ",".join(str(tag) for tag in tags)
    return {
        "id": component_id,
        "projectId": project_id,
        "parentId": parent_id,
        "originId": detail["originId"],
        "instanceId": detail["instanceId"],
        "name": properties.get("name", ""),
        "cppClass": properties.get("cppClass", ""),
        "alias": properties.get("alias", ""),
        "version": properties.get("version", ""),
        "author": properties.get("author", ""),
        "abstractId": properties.get("abstractId", ""),
        "componentType": properties.get("componentType", ""),
        "description": properties.get("description", ""),
        "tags": tags,
        "isSourceCode": int(bool(properties.get("isSourceCode", 0))),
        "isAtomic": int(bool(properties.get("isAtomic", 0))),
        "isCustom": int(bool(properties.get("isCustom", 0))),
        "customType": int(properties.get("customType", 0) or 0),
        "isOpen": 0,
        "properties": json_text(properties),
        "childComponents": detail["childComponents"] or "[]",
        "childConnections": detail["childConnections"] or "[]",
        "extensionProps": detail["extensionProps"] or "[]",
        "attribute": json_text(
            {
                parent_id: {
                    "size": {"width": size[0], "height": size[1]},
                    "position": {"x": position[0], "y": position[1]},
                    "zIndex": z_index,
                }
            }
        ),
        "namespace": properties.get("namespace", ""),
        "isComponentEdit": 0,
        "isEdited": 0,
        "nodeName": "",
        "vendor": detail["vendor"] or properties.get("vendor", "default"),
    }


def configure_boundary(
    row: dict[str, Any], *, alias: str, data_type: str, direction: str
) -> None:
    properties = json.loads(row["properties"])
    properties["alias"] = alias
    row["alias"] = alias
    row["properties"] = json_text(properties)

    sections, indexed = parse_sections(row["extensionProps"])
    base = indexed["base_config"]["value"][0]
    base.update(
        {
            "name": alias,
            "dataType": data_type,
            "category": 0,
            "shape": [1],
        }
    )
    if direction == "input":
        port = indexed["function_output_ports"]["value"][0]
        port["dataType"] = data_type
        set_prop(port, "isReturnFlag", False)
        set_prop(port, "name", alias)
        set_prop(port, "category", 0)
        set_prop(port, "dataType", data_type)
        set_prop(port, "shape", [1])
    else:
        base["isReturnFlag"] = False
        port = indexed["function_input_ports"]["value"][0]
        port["dataType"] = data_type
        set_prop(port, "isReturnFlag", False)
        set_prop(port, "name", alias)
        set_prop(port, "category", 0)
        set_prop(port, "dataType", data_type)
        set_prop(port, "shape", [1])
    row["extensionProps"] = json_text(sections)


def configure_constant(row: dict[str, Any], *, alias: str, data_type: str, value: Any) -> None:
    properties = json.loads(row["properties"])
    properties["alias"] = alias
    row["alias"] = alias
    row["properties"] = json_text(properties)
    sections, indexed = parse_sections(row["extensionProps"])
    indexed["base_config"]["value"][0].update(
        {"name": alias, "dataType": data_type, "dataValue": value}
    )
    port = indexed["function_output_ports"]["value"][0]
    port["name"] = alias
    port["alias"] = alias
    port["dataType"] = data_type
    set_prop(port, "datatype", data_type)
    row["extensionProps"] = json_text(sections)


def insert_component(connection: sqlite3.Connection, row: dict[str, Any]) -> None:
    columns = list(row)
    placeholders = ",".join("?" for _ in columns)
    connection.execute(
        f"INSERT INTO project_component ({','.join(columns)}) VALUES ({placeholders})",
        [row[column] for column in columns],
    )


def endpoint(cell: str, port: str) -> str:
    return json_text({"cell": cell, "magnet": "outer-circle", "port": port})


def add_connection(
    connection: sqlite3.Connection,
    *,
    connection_id: str,
    parent_id: str,
    source_cell: str,
    source_port: str,
    target_cell: str,
    target_port: str,
) -> None:
    connection.execute(
        """
        INSERT INTO project_connection
          (id, parentId, source, target, attribute, extensionProps)
        VALUES (?, ?, ?, ?, ?, ?)
        """,
        (
            connection_id,
            parent_id,
            endpoint(source_cell, source_port),
            endpoint(target_cell, target_port),
            json_text({"from": "magnet"}),
            json_text(CONNECTION_EXTENSION),
        ),
    )


def composite_port(port_id: str, name: str, data_type: str, direction: str) -> dict[str, Any]:
    return {
        "id": port_id,
        "name": name,
        "alias": name,
        "direction": direction,
        "description": "函数输入端口" if direction == "In" else "函数输出端口",
        "props": [
            {"key": "isReturnFlag", "value": False, "props": []},
            {"key": "name", "value": name, "props": []},
            {"key": "category", "value": 0, "props": []},
            {"key": "dataType", "value": data_type, "props": []},
            {"key": "shape", "value": [1], "props": []},
        ],
    }


def update_port_type(raw: str, section: str, port_name: str, data_type: str) -> str:
    sections, indexed = parse_sections(raw)
    for port in indexed[section].get("value", []):
        if port.get("name") == port_name:
            set_prop(port, "dataType", data_type)
            break
    else:
        raise RuntimeError(f"Port not found while changing type: {port_name}")
    return json_text(sections)


def verify_no_duplicate_alias(
    connection: sqlite3.Connection, parent_id: str, alias: str
) -> None:
    count = connection.execute(
        "SELECT COUNT(*) FROM project_component WHERE parentId=? AND alias=?",
        (parent_id, alias),
    ).fetchone()[0]
    if count:
        raise RuntimeError(f"Component already exists under parent: {alias}")


def apply(project: Path, library_db: Path) -> None:
    cbdes = project / "data" / "cbdes.db"
    temp = project / "data" / "temp.db"
    if not cbdes.is_file() or not temp.is_file():
        raise RuntimeError("cbdes.db or temp.db is missing")
    if sha256(cbdes) != sha256(temp):
        raise RuntimeError("cbdes.db and temp.db differ before the edit")

    with sqlite3.connect(library_db) as library:
        library.row_factory = sqlite3.Row
        input_channel_detail = detail_by_instance(library, LKS_INPUT_INSTANCE)
        output_channel_detail = detail_by_instance(library, LKS_OUTPUT_INSTANCE)
        scope_detail = generic_detail(library, "oscilloscope")
        input_detail = generic_detail(library, "input")
        output_detail = generic_detail(library, "output")
        constant_detail = generic_detail(library, "constant")

    database_backup = cbdes.with_name("cbdes.db.before_lks_channels_scope_20260729")
    shutil.copy2(cbdes, database_backup)

    connection = sqlite3.connect(cbdes)
    connection.row_factory = sqlite3.Row
    try:
        connection.execute("PRAGMA foreign_keys=OFF")
        connection.execute("BEGIN IMMEDIATE")

        root = connection.execute(
            "SELECT * FROM project_component WHERE alias=? AND componentType='composite-block'",
            (ROOT_ALIAS,),
        ).fetchone()
        module = connection.execute(
            "SELECT * FROM project_component WHERE alias=? AND componentType='module'",
            (MODULE_ALIAS,),
        ).fetchone()
        scope = connection.execute(
            "SELECT * FROM project_component WHERE alias=? AND componentType='oscilloscope'",
            (SCOPE_ALIAS,),
        ).fetchone()
        if root is None or module is None or scope is None:
            raise RuntimeError("Expected lks root, module, or oscilloscope is missing")
        if root["parentId"] != module["id"] or scope["parentId"] != root["id"]:
            raise RuntimeError("Unexpected lks2 hierarchy")
        project_id = root["projectId"]
        root_id = root["id"]
        module_id = module["id"]

        verify_no_duplicate_alias(connection, module_id, "输入通道")
        verify_no_duplicate_alias(connection, module_id, "输出通道")

        input_channel_id = stable_id(project_id, "channel-input")
        output_channel_id = stable_id(project_id, "channel-output")
        input_channel = project_row_from_detail(
            input_channel_detail,
            component_id=input_channel_id,
            project_id=project_id,
            parent_id=module_id,
            position=(40, 220),
            size=(245, 246),
            z_index=30,
        )
        output_channel = project_row_from_detail(
            output_channel_detail,
            component_id=output_channel_id,
            project_id=project_id,
            parent_id=module_id,
            position=(1100, 250),
            size=(245, 190),
            z_index=30,
        )
        insert_component(connection, input_channel)
        insert_component(connection, output_channel)

        input_specs = [
            ("egoV", "double", (-1510, -390)),
            ("c0", "double", (-1510, -290)),
            ("c1", "double", (-1510, -190)),
            ("c2", "double", (-1510, -90)),
            ("c3", "double", (-1510, 10)),
            ("curvature", "double", (-1510, 110)),
            ("brakePressed", "bool", (-1510, 210)),
            ("driverSteerNorm", "double", (-1510, 310)),
        ]
        output_specs = [
            ("steerRad", "double", (1170, -300)),
            ("controlEnabled", "bool", (1170, -190)),
            ("valid", "bool", (1170, -80)),
            ("previewDistance", "double", (1170, 30)),
            ("weightedError", "double", (1170, 140)),
        ]
        input_ids = {name: stable_id(project_id, f"input-{name}") for name, _, _ in input_specs}
        output_ids = {name: stable_id(project_id, f"output-{name}") for name, _, _ in output_specs}
        valid_constant_id = stable_id(project_id, "constant-valid")

        old_input_aliases = (
            "egoV",
            "c0",
            "c1",
            "c2",
            "c3",
            "curvature",
            "brakePresseddriverSteerNorm",
        )
        old_rows = connection.execute(
            f"""
            SELECT id FROM project_component
            WHERE parentId=? AND componentType='constant'
              AND alias IN ({','.join('?' for _ in old_input_aliases)})
            """,
            (root_id, *old_input_aliases),
        ).fetchall()
        old_ids = [row["id"] for row in old_rows]
        if len(old_ids) != len(old_input_aliases):
            raise RuntimeError(
                f"Expected {len(old_input_aliases)} test constants, found {len(old_ids)}"
            )
        for old_id in old_ids:
            connection.execute(
                """
                DELETE FROM project_connection
                WHERE json_extract(source, '$.cell')=?
                   OR json_extract(target, '$.cell')=?
                """,
                (old_id, old_id),
            )
            connection.execute("DELETE FROM project_component WHERE id=?", (old_id,))

        for name, data_type, position in input_specs:
            row = project_row_from_detail(
                input_detail,
                component_id=input_ids[name],
                project_id=project_id,
                parent_id=root_id,
                position=position,
                size=(180, 68),
                z_index=40,
            )
            configure_boundary(row, alias=name, data_type=data_type, direction="input")
            insert_component(connection, row)

        for name, data_type, position in output_specs:
            row = project_row_from_detail(
                output_detail,
                component_id=output_ids[name],
                project_id=project_id,
                parent_id=root_id,
                position=position,
                size=(190, 68),
                z_index=40,
            )
            configure_boundary(row, alias=name, data_type=data_type, direction="output")
            insert_component(connection, row)

        valid_constant = project_row_from_detail(
            constant_detail,
            component_id=valid_constant_id,
            project_id=project_id,
            parent_id=root_id,
            position=(900, 260),
            size=(170, 68),
            z_index=40,
        )
        configure_constant(
            valid_constant, alias="OutputValid", data_type="bool", value=True
        )
        insert_component(connection, valid_constant)

        root_sections, root_indexed = parse_sections(root["extensionProps"])
        root_indexed["function_input_ports"]["value"] = [
            composite_port(input_ids[name], name, data_type, "In")
            for name, data_type, _ in input_specs
        ]
        root_indexed["function_output_ports"]["value"] = [
            composite_port(output_ids[name], name, data_type, "Out")
            for name, data_type, _ in output_specs
        ]
        connection.execute(
            "UPDATE project_component SET extensionProps=?, attribute=? WHERE id=?",
            (
                json_text(root_sections),
                json_text(
                    {
                        module_id: {
                            "size": {"width": 520, "height": 330},
                            "position": {"x": 500, "y": 180},
                            "zIndex": 30,
                        }
                    }
                ),
                root_id,
            ),
        )

        # Align the two public boolean signals with the Pangu message contract.
        decision = connection.execute(
            "SELECT * FROM project_component WHERE parentId=? AND alias='LKSDecision'",
            (root_id,),
        ).fetchone()
        if decision is None:
            raise RuntimeError("LKSDecision is missing")
        decision_extension = update_port_type(
            decision["extensionProps"], "function_input_ports", "brakePressed", "bool"
        )
        decision_extension = update_port_type(
            decision_extension, "function_output_ports", "controlEnabled", "bool"
        )
        connection.execute(
            "UPDATE project_component SET extensionProps=? WHERE id=?",
            (decision_extension, decision["id"]),
        )
        for alias, data_type, section in (
            ("brakePressed", "bool", "input"),
            ("controlEnabled", "bool", "output"),
        ):
            child = connection.execute(
                "SELECT * FROM project_component WHERE parentId=? AND alias=?",
                (decision["id"], alias),
            ).fetchone()
            if child is None:
                raise RuntimeError(f"LKSDecision boundary is missing: {alias}")
            child_row = dict(child)
            configure_boundary(
                child_row, alias=alias, data_type=data_type, direction=section
            )
            connection.execute(
                "UPDATE project_component SET properties=?, extensionProps=? WHERE id=?",
                (child_row["properties"], child_row["extensionProps"], child["id"]),
            )

        # Replace the scope from the exact current library template, preserving its ID.
        scope_template = project_row_from_detail(
            scope_detail,
            component_id=scope["id"],
            project_id=project_id,
            parent_id=root_id,
            position=(1480, -245),
            size=(178, 100),
            z_index=43,
        )
        scope_template["alias"] = SCOPE_ALIAS
        scope_properties = json.loads(scope_template["properties"])
        scope_properties["alias"] = SCOPE_ALIAS
        scope_template["properties"] = json_text(scope_properties)
        update_columns = [column for column in scope_template if column != "id"]
        connection.execute(
            f"UPDATE project_component SET {','.join(f'{column}=?' for column in update_columns)} WHERE id=?",
            [scope_template[column] for column in update_columns] + [scope["id"]],
        )

        # Remove top-level edges that are replaced or repaired below.
        replace_targets = [
            ("aa61974c-e982-4ef6-b6c2-e61dba56487d", "0db5fd67-3722-459f-bd71-ac5e26ea7ce2"),
            ("aa61974c-e982-4ef6-b6c2-e61dba56487d", "8e993c2e-9652-43d3-92b9-57af79b5960f"),
            ("35bfe571-0409-485f-b070-0999518ce7bc", "3699b2db-94cc-4dad-9535-4fcf2eaf0f4a"),
            ("35bfe571-0409-485f-b070-0999518ce7bc", "19f99bb2-6431-4a41-ba2e-145f136311f8"),
            ("35bfe571-0409-485f-b070-0999518ce7bc", "bb2defb6-19ec-4f46-beee-a6a3e3340f9d"),
            ("14f902c4-eeee-4130-bb60-0e0218c9aea7", "57f3ef69-2e24-4288-9e5a-1e9285018a78"),
            ("a96057e9-5379-42d6-94a2-dd98930f3bd6", "044f2e47-a4fe-4d07-9a56-93dc78194e40"),
            ("a96057e9-5379-42d6-94a2-dd98930f3bd6", "b813f43e-e4b5-49df-8baf-36100041a637"),
            ("a96057e9-5379-42d6-94a2-dd98930f3bd6", "4ede2c16-1d64-412f-ac74-d8de0182fad8"),
            ("a96057e9-5379-42d6-94a2-dd98930f3bd6", "447179d7-55c9-4726-bc42-bb168a805027"),
            ("d79c36b4-01b4-41fd-b98a-f76ca88a11c5", "9d5b1ea7-8d33-46c5-b664-3c31552b1d55"),
            ("d79c36b4-01b4-41fd-b98a-f76ca88a11c5", "771a9fb7-c2ee-4183-96c1-ed8ba166271a"),
            ("d79c36b4-01b4-41fd-b98a-f76ca88a11c5", "ade97ba3-ce45-4469-bcc7-165aa341d4e2"),
            ("d79c36b4-01b4-41fd-b98a-f76ca88a11c5", "5cf0299e-e75f-4504-8da9-74228be60fc7"),
            ("b25a47b1-cf11-4375-a368-3b21bdca7e68", "a4d8624a-29fd-4927-86bd-b4c00da77643"),
            ("b25a47b1-cf11-4375-a368-3b21bdca7e68", "c0218325-ee7e-4637-8abe-a80febe61a43"),
            ("b25a47b1-cf11-4375-a368-3b21bdca7e68", "6ee8dd2e-c059-4e88-b307-9b1ad0411d57"),
            ("b25a47b1-cf11-4375-a368-3b21bdca7e68", "4b8dabbc-f698-42e3-8efc-df058ba35231"),
            ("b25a47b1-cf11-4375-a368-3b21bdca7e68", "2687a11e-a45f-411a-8c6d-5ae5bced4fff"),
        ]
        for target_cell, target_port in replace_targets:
            connection.execute(
                """
                DELETE FROM project_connection
                WHERE parentId=?
                  AND json_extract(target, '$.cell')=?
                  AND json_extract(target, '$.port')=?
                """,
                (root_id, target_cell, target_port),
            )
        connection.execute(
            "DELETE FROM project_connection WHERE parentId=? AND json_extract(target, '$.cell')=?",
            (root_id, scope["id"]),
        )

        target_map = {
            "egoV": [
                ("aa61974c-e982-4ef6-b6c2-e61dba56487d", "0db5fd67-3722-459f-bd71-ac5e26ea7ce2"),
                ("35bfe571-0409-485f-b070-0999518ce7bc", "3699b2db-94cc-4dad-9535-4fcf2eaf0f4a"),
                ("14f902c4-eeee-4130-bb60-0e0218c9aea7", "57f3ef69-2e24-4288-9e5a-1e9285018a78"),
            ],
            "curvature": [
                ("aa61974c-e982-4ef6-b6c2-e61dba56487d", "8e993c2e-9652-43d3-92b9-57af79b5960f")
            ],
            "c0": [
                ("a96057e9-5379-42d6-94a2-dd98930f3bd6", "044f2e47-a4fe-4d07-9a56-93dc78194e40"),
                ("d79c36b4-01b4-41fd-b98a-f76ca88a11c5", "9d5b1ea7-8d33-46c5-b664-3c31552b1d55"),
                ("b25a47b1-cf11-4375-a368-3b21bdca7e68", "a4d8624a-29fd-4927-86bd-b4c00da77643"),
            ],
            "c1": [
                ("a96057e9-5379-42d6-94a2-dd98930f3bd6", "b813f43e-e4b5-49df-8baf-36100041a637"),
                ("d79c36b4-01b4-41fd-b98a-f76ca88a11c5", "771a9fb7-c2ee-4183-96c1-ed8ba166271a"),
                ("b25a47b1-cf11-4375-a368-3b21bdca7e68", "c0218325-ee7e-4637-8abe-a80febe61a43"),
            ],
            "c2": [
                ("a96057e9-5379-42d6-94a2-dd98930f3bd6", "4ede2c16-1d64-412f-ac74-d8de0182fad8"),
                ("d79c36b4-01b4-41fd-b98a-f76ca88a11c5", "ade97ba3-ce45-4469-bcc7-165aa341d4e2"),
                ("b25a47b1-cf11-4375-a368-3b21bdca7e68", "6ee8dd2e-c059-4e88-b307-9b1ad0411d57"),
            ],
            "c3": [
                ("a96057e9-5379-42d6-94a2-dd98930f3bd6", "447179d7-55c9-4726-bc42-bb168a805027"),
                ("d79c36b4-01b4-41fd-b98a-f76ca88a11c5", "5cf0299e-e75f-4504-8da9-74228be60fc7"),
                ("b25a47b1-cf11-4375-a368-3b21bdca7e68", "4b8dabbc-f698-42e3-8efc-df058ba35231"),
            ],
            "brakePressed": [
                ("35bfe571-0409-485f-b070-0999518ce7bc", "19f99bb2-6431-4a41-ba2e-145f136311f8")
            ],
            "driverSteerNorm": [
                ("35bfe571-0409-485f-b070-0999518ce7bc", "bb2defb6-19ec-4f46-beee-a6a3e3340f9d")
            ],
        }
        for name, targets in target_map.items():
            for index, (target_cell, target_port) in enumerate(targets):
                add_connection(
                    connection,
                    connection_id=stable_id(project_id, f"inner-{name}-{index}"),
                    parent_id=root_id,
                    source_cell=input_ids[name],
                    source_port="out",
                    target_cell=target_cell,
                    target_port=target_port,
                )

        preview_component = "aa61974c-e982-4ef6-b6c2-e61dba56487d"
        preview_port = "aee13f76-f24e-43e4-9d58-501534eb2faa"
        decision_component = "35bfe571-0409-485f-b070-0999518ce7bc"
        decision_port = "f250fbf3-cf44-48fa-8924-b130a2d18e7b"
        weighted_component = "408bc885-5a8d-48df-ac8b-d2cc40f1e9d6"
        weighted_port = "e324b082-528c-4f6e-b77d-87b5c95aecb8"
        steer_component = "0058a2ec-063b-4158-b8b6-4f089494b7ab"
        steer_port = "multiply_output_result_Out"

        add_connection(
            connection,
            connection_id=stable_id(project_id, "repair-preview-to-p3"),
            parent_id=root_id,
            source_cell=preview_component,
            source_port=preview_port,
            target_cell="b25a47b1-cf11-4375-a368-3b21bdca7e68",
            target_port="2687a11e-a45f-411a-8c6d-5ae5bced4fff",
        )
        output_sources = {
            "steerRad": (steer_component, steer_port),
            "controlEnabled": (decision_component, decision_port),
            "valid": (valid_constant_id, "out"),
            "previewDistance": (preview_component, preview_port),
            "weightedError": (weighted_component, weighted_port),
        }
        for name, (source_cell, source_port) in output_sources.items():
            add_connection(
                connection,
                connection_id=stable_id(project_id, f"inner-output-{name}"),
                parent_id=root_id,
                source_cell=source_cell,
                source_port=source_port,
                target_cell=output_ids[name],
                target_port="in",
            )
        for index, (source_cell, source_port) in enumerate(
            ((steer_component, steer_port), (decision_component, decision_port)), start=1
        ):
            add_connection(
                connection,
                connection_id=stable_id(project_id, f"scope-{index}"),
                parent_id=root_id,
                source_cell=source_cell,
                source_port=source_port,
                target_cell=scope["id"],
                target_port=f"input_port_{index}",
            )

        # Module-level channel wiring.
        input_channel_ports = {
            port["name"]: port["id"]
            for port in parse_sections(input_channel["extensionProps"])[1][
                "function_output_ports"
            ]["value"]
        }
        output_channel_ports = {
            port["name"]: port["id"]
            for port in parse_sections(output_channel["extensionProps"])[1][
                "function_input_ports"
            ]["value"]
        }
        external_to_internal = {
            "ego_speed_mps": "egoV",
            "c0_m": "c0",
            "c1": "c1",
            "c2_per_m": "c2",
            "c3_per_m2": "c3",
            "curvature_per_m": "curvature",
            "brake_pressed": "brakePressed",
            "driver_steer_norm": "driverSteerNorm",
        }
        for external_name, internal_name in external_to_internal.items():
            add_connection(
                connection,
                connection_id=stable_id(project_id, f"module-input-{external_name}"),
                parent_id=module_id,
                source_cell=input_channel_id,
                source_port=input_channel_ports[external_name],
                target_cell=root_id,
                target_port=input_ids[internal_name],
            )
        internal_to_external = {
            "steerRad": "steer_rad",
            "controlEnabled": "control_enabled",
            "valid": "valid",
            "previewDistance": "preview_distance_m",
            "weightedError": "weighted_error_m",
        }
        for internal_name, external_name in internal_to_external.items():
            add_connection(
                connection,
                connection_id=stable_id(project_id, f"module-output-{external_name}"),
                parent_id=module_id,
                source_cell=root_id,
                source_port=output_ids[internal_name],
                target_cell=output_channel_id,
                target_port=output_channel_ports[external_name],
            )
        add_connection(
            connection,
            connection_id=stable_id(project_id, "module-frame-id"),
            parent_id=module_id,
            source_cell=input_channel_id,
            source_port=input_channel_ports["frame_id"],
            target_cell=output_channel_id,
            target_port=output_channel_ports["frame_id"],
        )

        module_sections, module_indexed = parse_sections(module["extensionProps"])
        output_base = parse_sections(output_channel["extensionProps"])[1]["base_config"][
            "value"
        ][0]
        channel_config = output_base["channelConfig"]
        module_indexed["module_output_ports"]["value"] = [
            {
                "alias": channel_config["channel_name"],
                "description": "",
                "direction": "Out",
                "id": output_channel_id,
                "portType": "out",
                "name": channel_config["channel_name"],
                "isShow": True,
                "props": [
                    {"key": "channel_name", "value": channel_config["channel_name"]},
                    {"key": "bind_code", "value": channel_config["bind_code"]},
                    {
                        "key": "channel_var_name",
                        "value": channel_config["channel_var_name"],
                    },
                    {"key": "channel_type", "value": channel_config["channel_type"]},
                    {"key": "message_type", "value": channel_config["message_type"]},
                    {"key": "channel_id", "value": channel_config["channel_id"]},
                    {"key": "task_id", "value": channel_config["task_id"]},
                    {"key": "operateKeys", "value": output_base["operateKeys"]},
                ],
            }
        ]
        connection.execute(
            "UPDATE project_component SET extensionProps=? WHERE id=?",
            (json_text(module_sections), module_id),
        )

        result = connection.execute("PRAGMA integrity_check").fetchone()[0]
        if result != "ok":
            raise RuntimeError(f"SQLite integrity check failed: {result}")
        connection.commit()
    except Exception:
        connection.rollback()
        raise
    finally:
        connection.close()

    shutil.copy2(cbdes, temp)
    if sha256(cbdes) != sha256(temp):
        raise RuntimeError("Failed to synchronize temp.db")
    print(f"Updated: {cbdes}")
    print(f"Backup:  {database_backup}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("project", type=Path)
    parser.add_argument(
        "--library-db",
        type=Path,
        default=Path(
            "/home/aiden/gaasd_versions/gaasd-2.7.0.5/home/.gaasd/gaasd.db"
        ),
    )
    args = parser.parse_args()
    try:
        apply(args.project.resolve(), args.library_db.resolve())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
