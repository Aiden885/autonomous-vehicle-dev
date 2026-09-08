#!/usr/bin/env python3
"""Regression tests for the ACC acceleration-output naming migration."""

from __future__ import annotations

import json
import shutil
import sqlite3
import sys
import tempfile
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
TOOLS_DIR = REPO_ROOT / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from rename_newaccpro4_acceleration_output import (  # noqa: E402
    ACCELERATION_NAME,
    CHANNEL_OUTPUT_ID,
    EXECUTE_ACC_ID,
    EXECUTE_ACC_OUTPUT_ID,
    CONTROL_ID,
    CONTROL_OUTPUT_ID,
    GENERATE_TARGET_SPEED_ID,
    migrate_database,
)


PROJECT_DIR = REPO_ROOT / "project" / "newaccpro4"


def load_json(value: str) -> list[dict]:
    return json.loads(value)


def extension_value(extension: list[dict], key: str):
    for item in extension:
        if item.get("key") == key:
            return item.get("value")
    raise AssertionError(f"missing extension key: {key}")


def component(connection: sqlite3.Connection, component_id: str) -> sqlite3.Row:
    row = connection.execute(
        "SELECT * FROM project_component WHERE id=?", (component_id,)
    ).fetchone()
    if row is None:
        raise AssertionError(f"missing component: {component_id}")
    return row


def parent_output_name(row: sqlite3.Row, port_id: str) -> str:
    ports = extension_value(load_json(row["extensionProps"]), "function_output_ports")
    port = next(item for item in ports if item.get("id") == port_id)
    return str(port["name"])


def boundary_name(row: sqlite3.Row) -> str:
    configs = extension_value(load_json(row["extensionProps"]), "base_config")
    return str(configs[0]["name"])


class AccelerationOutputMigrationTest(unittest.TestCase):
    def test_migration_renames_only_internal_acceleration_output_chain(self) -> None:
        source = PROJECT_DIR / "data" / "cbdes.db"
        with tempfile.TemporaryDirectory(prefix="acc-output-name-test-") as directory:
            database = Path(directory) / "canvas.db"
            shutil.copy2(source, database)

            before = sqlite3.connect(database)
            before.row_factory = sqlite3.Row
            channel_before = tuple(component(before, CHANNEL_OUTPUT_ID))
            connections_before = before.execute(
                "SELECT * FROM project_connection ORDER BY id"
            ).fetchall()
            before.close()

            migrate_database(database)

            after = sqlite3.connect(database)
            after.row_factory = sqlite3.Row
            self.assertEqual(
                parent_output_name(component(after, CONTROL_ID), CONTROL_OUTPUT_ID),
                ACCELERATION_NAME,
            )
            self.assertEqual(
                parent_output_name(
                    component(after, EXECUTE_ACC_ID), EXECUTE_ACC_OUTPUT_ID
                ),
                ACCELERATION_NAME,
            )
            self.assertEqual(
                boundary_name(component(after, CONTROL_OUTPUT_ID)), ACCELERATION_NAME
            )
            self.assertEqual(
                boundary_name(component(after, EXECUTE_ACC_OUTPUT_ID)),
                ACCELERATION_NAME,
            )
            self.assertEqual(component(after, CONTROL_OUTPUT_ID)["alias"], "加速度命令")
            self.assertEqual(
                component(after, EXECUTE_ACC_OUTPUT_ID)["alias"], "加速度命令"
            )

            target_speed = component(after, GENERATE_TARGET_SPEED_ID)
            target_speed_ports = extension_value(
                load_json(target_speed["extensionProps"]), "function_output_ports"
            )
            self.assertEqual(target_speed_ports[0]["name"], "targetSpeedMps")

            channel_after = component(after, CHANNEL_OUTPUT_ID)
            self.assertEqual(tuple(channel_after), channel_before)
            channel_ports = extension_value(
                load_json(channel_after["extensionProps"]), "function_input_ports"
            )
            self.assertIn("target_speed_mps", {port["name"] for port in channel_ports})
            self.assertEqual(
                after.execute("SELECT * FROM project_connection ORDER BY id").fetchall(),
                connections_before,
            )
            self.assertEqual(after.execute("PRAGMA integrity_check").fetchone()[0], "ok")
            after.close()

    def test_project_databases_use_acceleration_name_but_keep_channel_field(self) -> None:
        for database_name in ("cbdes.db", "temp.db", "runtimeCanvas.db"):
            with self.subTest(database=database_name):
                connection = sqlite3.connect(PROJECT_DIR / "data" / database_name)
                connection.row_factory = sqlite3.Row
                self.assertEqual(
                    parent_output_name(
                        component(connection, CONTROL_ID), CONTROL_OUTPUT_ID
                    ),
                    ACCELERATION_NAME,
                )
                self.assertEqual(
                    parent_output_name(
                        component(connection, EXECUTE_ACC_ID), EXECUTE_ACC_OUTPUT_ID
                    ),
                    ACCELERATION_NAME,
                )
                channel = component(connection, CHANNEL_OUTPUT_ID)
                ports = extension_value(
                    load_json(channel["extensionProps"]), "function_input_ports"
                )
                self.assertIn("target_speed_mps", {port["name"] for port in ports})
                connection.close()


if __name__ == "__main__":
    unittest.main()
