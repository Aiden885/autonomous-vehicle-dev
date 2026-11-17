import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


PANEL_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PANEL_DIR))

import app as panel_app  # noqa: E402


class ScenarioOrderApiTest(unittest.TestCase):
    def create_scenario(self, root: Path, scenario_id: str, created_at=None) -> None:
        scenario_dir = root / scenario_id
        scenario_dir.mkdir()
        lines = [f"name: {scenario_id}"]
        if created_at is not None:
            lines.append(f'created_at: "{created_at}"')
        (scenario_dir / "scenario.yaml").write_text(
            "\n".join(lines) + "\n",
            encoding="utf-8",
        )

    def test_scenarios_are_ordered_oldest_first_with_latest_at_bottom(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            scenarios_root = Path(temp_dir)
            self.create_scenario(scenarios_root, "a_latest", "2026-08-28")
            self.create_scenario(scenarios_root, "b_oldest", "2026-05-01")
            self.create_scenario(
                scenarios_root,
                "c_middle",
                "2026-08-20 10:30:00 +0800",
            )
            self.create_scenario(scenarios_root, "z_undated")

            with mock.patch.object(panel_app, "SCENARIOS_DIR", scenarios_root):
                response = panel_app.app.test_client().get("/api/scenarios")

        self.assertEqual(response.status_code, 200)
        scenario_ids = [item["id"] for item in response.get_json()["scenarios"]]
        self.assertEqual(
            scenario_ids,
            ["z_undated", "b_oldest", "c_middle", "a_latest"],
        )


if __name__ == "__main__":
    unittest.main()
