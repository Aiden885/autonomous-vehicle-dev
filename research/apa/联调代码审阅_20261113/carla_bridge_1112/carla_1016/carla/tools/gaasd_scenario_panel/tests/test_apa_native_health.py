import sys
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import app as panel_app


class ApaNativeHealthTest(unittest.TestCase):
    def test_native_apa_health_does_not_require_docker(self):
        pangu = {"runtime": "auto", "app_name": "apa", "processes": ["ApaZmqInputModule", "apa", "ApaZmqOutputModule"]}
        with mock.patch.object(panel_app, "socket_open", return_value=True), \
             mock.patch.object(panel_app, "local_pangu_processes_running", return_value=True), \
             mock.patch.object(panel_app.subprocess, "run") as docker:
            health = panel_app.health_for({"pangu": pangu})
        self.assertTrue(health["pangu_process"])
        docker.assert_not_called()

    def test_auto_runtime_preserves_docker_fallback(self):
        with mock.patch.object(panel_app, "socket_open", return_value=False), \
             mock.patch.object(panel_app, "local_pangu_processes_running", return_value=False), \
             mock.patch.object(panel_app.subprocess, "run", return_value=mock.Mock(returncode=0)) as docker:
            health = panel_app.health_for({"pangu": {"runtime": "auto", "container_name": "pangu_x86"}})
        self.assertTrue(health["pangu_process"])
        self.assertEqual(docker.call_args.args[0][:3], ["docker", "exec", "pangu_x86"])

    def test_native_health_requires_all_three_nodes(self):
        names = ["ApaZmqInputModule", "apa", "ApaZmqOutputModule"]
        entries = []
        for name in names:
            entry = mock.Mock()
            entry.read_bytes.return_value = f"/install/dataflow_runner\0--app_module=apa\0--process_name={name}\0".encode()
            entries.append(entry)
        with mock.patch.object(panel_app.Path, "glob", return_value=entries):
            self.assertTrue(panel_app.local_pangu_processes_running({"app_name": "apa", "processes": names}))
        with mock.patch.object(panel_app.Path, "glob", return_value=entries[:2]):
            self.assertFalse(panel_app.local_pangu_processes_running({"app_name": "apa", "processes": names}))


if __name__ == "__main__":
    unittest.main()
