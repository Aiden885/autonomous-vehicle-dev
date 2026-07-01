#!/usr/bin/env python3
from __future__ import annotations

import os
import re
import socket
import subprocess
import sys
import threading
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from flask import Flask, jsonify, render_template, request, Response

try:
    import yaml
except ImportError:  # pragma: no cover - fallback for minimal environments
    yaml = None


PANEL_DIR = Path(__file__).resolve().parent
REPO_ROOT = PANEL_DIR.parents[1]
SCENARIOS_DIR = REPO_ROOT / "scenarios"
DEFAULT_PORT = int(os.environ.get("GAASD_PANEL_PORT", "8765"))
MAX_LOG_LINES = 3000
SCENARIO_ID_PATTERN = re.compile(r"^[A-Za-z0-9._-]+$")
DRIVER_COMMAND_KEYS = {
    "e": "E 启控/降速",
    "q": "Q 加速/继承启控",
    "t": "T 减小时距",
    "r": "R 增大时距",
    "c": "C 取消 ACC",
    "s": "S 制动退出",
    "0": "0 释放踏板",
}

app = Flask(__name__)


class ScenarioState:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.status = "idle"
        self.action = ""
        self.last_error = ""
        self.process: Optional[subprocess.Popen[str]] = None
        self.logs: List[Dict[str, str]] = []

    def append(self, level: str, message: str) -> None:
        now = datetime.now().strftime("%H:%M:%S")
        entry = {"time": now, "level": level, "message": message.rstrip()}
        with self.lock:
            self.logs.append(entry)
            if len(self.logs) > MAX_LOG_LINES:
                self.logs = self.logs[-MAX_LOG_LINES:]

    def snapshot_logs(self, since: int) -> Tuple[int, List[Dict[str, str]]]:
        with self.lock:
            total = len(self.logs)
            since = max(0, min(since, total))
            return total, self.logs[since:]

    def snapshot(self) -> Dict[str, str]:
        with self.lock:
            return {
                "status": self.status,
                "action": self.action,
                "last_error": self.last_error,
            }


STATES: Dict[str, ScenarioState] = {}


def get_state(scenario_id: str) -> ScenarioState:
    if scenario_id not in STATES:
        STATES[scenario_id] = ScenarioState()
    return STATES[scenario_id]


def load_yaml_file(path: Path) -> Dict[str, Any]:
    if yaml is None:
        raise RuntimeError("PyYAML is required. Install it with: python3 -m pip install PyYAML")
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    return data if isinstance(data, dict) else {}


def safe_rel(path: Path) -> str:
    try:
        return str(path.relative_to(REPO_ROOT))
    except ValueError:
        return str(path)


def scenario_dir_for(scenario_id: str) -> Optional[Path]:
    if not SCENARIO_ID_PATTERN.fullmatch(scenario_id):
        return None
    scenarios_root = SCENARIOS_DIR.resolve()
    candidate = (SCENARIOS_DIR / scenario_id).resolve()
    try:
        candidate.relative_to(scenarios_root)
    except ValueError:
        return None
    if candidate.exists() and candidate.is_dir() and (candidate / "scenario.yaml").exists():
        return candidate
    return None


def scenario_stop_script(scenario_dir: Path) -> Path:
    local_script = scenario_dir / "stop.sh"
    if local_script.exists():
        return local_script
    return scenario_dir / "bridge_snapshot" / "tools" / "carla_bridge" / "stop-gaasd-carla-manual.sh"


def scenario_scripts(scenario_dir: Path) -> Dict[str, str]:
    stop_script = scenario_stop_script(scenario_dir)
    return {
        "restore": safe_rel(scenario_dir / "restore_gaasd_project.sh"),
        "start": safe_rel(scenario_dir / "run.sh"),
        "stop": safe_rel(stop_script),
        "readme": safe_rel(scenario_dir / "README.md"),
    }


def absolute_restore_path(data: Dict[str, Any]) -> str:
    projects = data.get("gaasd_projects") or {}
    closed_loop = projects.get("carla_closed_loop") or {}
    restore_path = closed_loop.get("restore_default_path", "")
    if not restore_path:
        return ""
    path = Path(str(restore_path))
    if not path.is_absolute():
        path = REPO_ROOT / path
    return str(path)


def scenario_summary(scenario_dir: Path) -> Dict[str, Any]:
    data = load_yaml_file(scenario_dir / "scenario.yaml")
    scene = data.get("carla_scene") or {}
    env = data.get("environment") or {}
    expected = data.get("expected_result") or {}
    signals = data.get("gaasd_signals") or {}

    scenario_id = scenario_dir.name
    raw_name = str(data.get("name", scenario_id))
    display_name = raw_name
    if raw_name == scenario_id and data.get("description"):
        display_name = str(data.get("description"))
    state = get_state(scenario_id)
    return {
        "id": scenario_id,
        "name": display_name,
        "description": data.get("description", ""),
        "created_at": data.get("created_at", ""),
        "directory": safe_rel(scenario_dir),
        "restore_path": absolute_restore_path(data),
        "environment": {
            "carla_version": env.get("carla_version", ""),
            "carla_root": env.get("carla_root", ""),
            "python": env.get("python", ""),
        },
        "scene": {
            "map": scene.get("map", ""),
            "ego_spawn_index": scene.get("ego_spawn_index", ""),
            "lead_distance_m": scene.get("lead_distance_m", ""),
            "lead_speed_mps": scene.get("lead_speed_mps", ""),
            "lead_behavior": scene.get("lead_behavior", ""),
            "lead_placement": scene.get("lead_placement", ""),
        },
        "signals": signals.get("observed", []),
        "expected": {
            "duration_sec": expected.get("duration_sec", ""),
            "target_distance_m": expected.get("target_distance_m", ""),
            "expected_behavior": expected.get("expected_behavior", ""),
        },
        "scripts": scenario_scripts(scenario_dir),
        "state": state.snapshot(),
    }


def list_scenarios() -> List[Dict[str, Any]]:
    if not SCENARIOS_DIR.exists():
        return []
    items = []
    for path in sorted(SCENARIOS_DIR.iterdir()):
        if path.is_dir() and (path / "scenario.yaml").exists():
            items.append(scenario_summary(path))
    return items


def script_path(scenario_dir: Path, kind: str) -> Optional[Path]:
    if kind == "restore":
        return scenario_dir / "restore_gaasd_project.sh"
    if kind == "start":
        return scenario_dir / "run.sh"
    if kind == "stop":
        return scenario_stop_script(scenario_dir)
    return None


def generated_fix_project_paths(data: Dict[str, Any]) -> List[str]:
    candidates: List[Path] = []
    restore_path = absolute_restore_path(data)
    if restore_path:
        candidates.append(Path(restore_path))

    # Current working project used during active debugging.
    candidates.append(REPO_ROOT / "project" / "carla")

    result: List[str] = []
    seen = set()
    for candidate in candidates:
        path = candidate if candidate.is_absolute() else REPO_ROOT / candidate
        resolved = path.resolve()
        if resolved in seen or not resolved.exists():
            continue
        seen.add(resolved)
        result.append(str(resolved))
    return result


def run_script_job(scenario_id: str, action: str, command: List[str]) -> Tuple[bool, str]:
    state = get_state(scenario_id)
    with state.lock:
        if state.process is not None and state.process.poll() is None:
            return False, f"当前仍在执行 {state.action}，请等待完成。"
        state.status = "running"
        state.action = action
        state.last_error = ""

    def worker() -> None:
        state.append("info", f"$ {' '.join(command)}")
        env = os.environ.copy()
        env["PYTHONUNBUFFERED"] = "1"
        try:
            process = subprocess.Popen(
                command,
                cwd=str(REPO_ROOT),
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
            with state.lock:
                state.process = process

            assert process.stdout is not None
            for line in process.stdout:
                state.append("log", line)

            return_code = process.wait()
            with state.lock:
                state.process = None
                state.action = ""
                if return_code == 0:
                    if action == "start":
                        state.status = "ready"
                    elif action == "stop":
                        state.status = "idle"
                    elif action == "restore":
                        state.status = "restored"
                    else:
                        state.status = "idle"
                    state.last_error = ""
                else:
                    state.status = "error"
                    state.last_error = f"{action} failed with code {return_code}"
            if return_code == 0:
                state.append("ok", f"{action} 完成")
            else:
                state.append("error", f"{action} 失败，退出码 {return_code}")
        except Exception as exc:  # pragma: no cover - operational guard
            with state.lock:
                state.process = None
                state.action = ""
                state.status = "error"
                state.last_error = str(exc)
            state.append("error", str(exc))

    threading.Thread(target=worker, daemon=True).start()
    return True, "started"


def socket_open(host: str, port: int, timeout: float = 0.35) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def health_for(data: Dict[str, Any]) -> Dict[str, Any]:
    bridge = data.get("bridge") or {}
    pangu = data.get("pangu") or {}
    carla_host = "127.0.0.1"
    pub_port = 5701
    control_port = 5702
    pangu_process_ok = False
    pangu_container = str(pangu.get("container_name", "newaccpro3_pangu_carla"))
    pangu_process = str(pangu.get("process_name", "ZmqBridgeModule"))

    pub = str(bridge.get("zmq_pub", ""))
    control = str(bridge.get("zmq_control", ""))
    if pub.rsplit(":", 1)[-1].isdigit():
        pub_port = int(pub.rsplit(":", 1)[-1])
    if control.rsplit(":", 1)[-1].isdigit():
        control_port = int(control.rsplit(":", 1)[-1])

    bridge_pub_ok = socket_open(carla_host, pub_port)
    bridge_control_ok = socket_open(carla_host, control_port)
    if pangu:
        try:
            check = subprocess.run(
                [
                    "docker",
                    "exec",
                    pangu_container,
                    "bash",
                    "-lc",
                    f"pgrep -f 'dataflow_runner.*--process_name={pangu_process}' >/dev/null",
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=1.5,
                check=False,
            )
            pangu_process_ok = check.returncode == 0
        except Exception:
            pangu_process_ok = False
    # Never probe CARLA's RPC port with bare TCP: CARLA 0.9.x can crash on disconnect.
    # This is a Bridge-readiness proxy; startup performs authoritative CARLA RPC checks.
    carla_ok = bridge_pub_ok and bridge_control_ok
    return {
        "carla": carla_ok,
        "bridge_pub": bridge_pub_ok,
        "bridge_control": bridge_control_ok,
        "pangu_process": pangu_process_ok,
        "ports": {
            "carla": 2000,
            "bridge_pub": pub_port,
            "bridge_control": control_port,
            "pangu_process": pangu_process,
        },
    }


def bridge_control_endpoint(data: Dict[str, Any]) -> str:
    bridge = data.get("bridge") or {}
    return str(bridge.get("zmq_control", "tcp://127.0.0.1:5702"))


def scenario_python(data: Dict[str, Any], fallback: str = "python3") -> str:
    env = data.get("environment") or {}
    return str(env.get("python") or os.environ.get("GAASD_PANEL_PYTHON") or fallback)


def send_driver_command(scenario_id: str, data: Dict[str, Any], key: str) -> Tuple[bool, str]:
    normalized = key.strip().lower()
    if normalized not in DRIVER_COMMAND_KEYS:
        return False, "unsupported driver command"

    script = REPO_ROOT / "tools" / "pangu_acc_closed_loop" / "keyboard_command_publisher.py"
    if not script.exists():
        return False, f"missing command publisher: {script}"

    state = get_state(scenario_id)
    endpoint = bridge_control_endpoint(data)
    command = [
        "python3",
        str(script),
        "--endpoint",
        endpoint,
        "--once",
        normalized,
    ]
    state.append("info", f"发送驾驶指令 {DRIVER_COMMAND_KEYS[normalized]} -> {endpoint}")
    try:
        result = subprocess.run(
            command,
            cwd=str(REPO_ROOT),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=8,
            check=False,
        )
    except Exception as exc:
        state.append("error", f"驾驶指令发送失败: {exc}")
        return False, str(exc)

    if result.stdout.strip():
        for line in result.stdout.splitlines():
            state.append("log", line)
    if result.returncode != 0:
        state.append("error", f"驾驶指令发送失败，退出码 {result.returncode}")
        return False, f"command failed with code {result.returncode}"
    state.append("ok", f"驾驶指令已发送: {DRIVER_COMMAND_KEYS[normalized]}")
    return True, "sent"


def boost_ego_speed(scenario_id: str, data: Dict[str, Any]) -> Tuple[bool, str]:
    state = get_state(scenario_id)
    state.append("info", "执行辅助起步 boost ego speed")
    try:
        command = boost_ego_command(data)
        result = subprocess.run(
            command,
            cwd=str(REPO_ROOT),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=12,
            check=False,
        )
    except Exception as exc:
        state.append("error", f"辅助起步失败: {exc}")
        return False, str(exc)

    if result.stdout.strip():
        for line in result.stdout.splitlines():
            state.append("log", line)
    if result.returncode != 0:
        state.append("error", f"辅助起步失败，退出码 {result.returncode}")
        return False, f"boost failed with code {result.returncode}"
    state.append("ok", "辅助起步完成")
    return True, "boosted"


def boost_ego_command(data: Dict[str, Any]) -> List[str]:
    script = REPO_ROOT / "tools" / "carla_bridge" / "boost-ego-speed.py"
    if not script.exists():
        raise FileNotFoundError(f"missing boost script: {script}")

    env = data.get("environment") or {}
    python_bin = scenario_python(data, "python3.8")
    carla_root = str(env.get("carla_root", "/home/aiden/snap/code/app/carla-package"))
    return [
        python_bin,
        str(script),
        "--carla-root",
        carla_root,
        "--speed-mps",
        "2.0",
        "--duration-sec",
        "4.0",
    ]


@app.route("/")
def index() -> str:
    return render_template("index.html")


@app.get("/api/scenarios")
def api_scenarios() -> Response:
    return jsonify({"repo_root": str(REPO_ROOT), "scenarios": list_scenarios()})


@app.get("/api/scenarios/<scenario_id>")
def api_scenario(scenario_id: str) -> Response:
    scenario_dir = scenario_dir_for(scenario_id)
    if scenario_dir is None:
        return jsonify({"error": "scenario not found"}), 404
    return jsonify(scenario_summary(scenario_dir))


@app.post("/api/scenarios/<scenario_id>/restore")
def api_restore(scenario_id: str) -> Response:
    scenario_dir = scenario_dir_for(scenario_id)
    if scenario_dir is None:
        return jsonify({"error": "scenario not found"}), 404
    script = script_path(scenario_dir, "restore")
    if script is None or not script.exists():
        return jsonify({"error": "restore script not found"}), 404
    force = bool((request.get_json(silent=True) or {}).get("force", False))
    command = ["bash", str(script)]
    if force:
        command.append("--force")
    ok, message = run_script_job(scenario_id, "restore", command)
    return jsonify({"ok": ok, "message": message}), (202 if ok else 409)


@app.post("/api/scenarios/<scenario_id>/start")
def api_start(scenario_id: str) -> Response:
    scenario_dir = scenario_dir_for(scenario_id)
    if scenario_dir is None:
        return jsonify({"error": "scenario not found"}), 404
    script = script_path(scenario_dir, "start")
    if script is None or not script.exists():
        return jsonify({"error": "start script not found"}), 404
    ok, message = run_script_job(scenario_id, "start", ["bash", str(script)])
    return jsonify({"ok": ok, "message": message}), (202 if ok else 409)


@app.post("/api/scenarios/<scenario_id>/stop")
def api_stop(scenario_id: str) -> Response:
    scenario_dir = scenario_dir_for(scenario_id)
    if scenario_dir is None:
        return jsonify({"error": "scenario not found"}), 404
    script = script_path(scenario_dir, "stop")
    if script is None or not script.exists():
        return jsonify({"error": "stop script not found"}), 404
    ok, message = run_script_job(scenario_id, "stop", ["bash", str(script)])
    return jsonify({"ok": ok, "message": message}), (202 if ok else 409)


@app.post("/api/scenarios/<scenario_id>/fix-generated")
def api_fix_generated(scenario_id: str) -> Response:
    scenario_dir = scenario_dir_for(scenario_id)
    if scenario_dir is None:
        return jsonify({"error": "scenario not found"}), 404

    script = REPO_ROOT / "tools" / "carla_bridge" / "fix-gaasd-generated-code.py"
    if not script.exists():
        return jsonify({"error": "fix script not found"}), 404

    data = load_yaml_file(scenario_dir / "scenario.yaml")
    project_paths = generated_fix_project_paths(data)
    if not project_paths:
        return jsonify({"error": "no GAASD project directory found to patch"}), 404

    command = [sys.executable, str(script)]
    for project_path in project_paths:
        command.extend(["--project", project_path])

    ok, message = run_script_job(scenario_id, "fix-generated", command)
    return jsonify({"ok": ok, "message": message, "projects": project_paths}), (202 if ok else 409)


@app.post("/api/scenarios/<scenario_id>/driver-command")
def api_driver_command(scenario_id: str) -> Response:
    scenario_dir = scenario_dir_for(scenario_id)
    if scenario_dir is None:
        return jsonify({"error": "scenario not found"}), 404
    payload = request.get_json(silent=True) or {}
    key = str(payload.get("key", ""))
    data = load_yaml_file(scenario_dir / "scenario.yaml")
    ok, message = send_driver_command(scenario_id, data, key)
    return jsonify({"ok": ok, "message": message}), (200 if ok else 400)


@app.post("/api/scenarios/<scenario_id>/kickstart")
def api_kickstart(scenario_id: str) -> Response:
    scenario_dir = scenario_dir_for(scenario_id)
    if scenario_dir is None:
        return jsonify({"error": "scenario not found"}), 404
    data = load_yaml_file(scenario_dir / "scenario.yaml")
    state = get_state(scenario_id)
    state.append("info", "执行辅助启控: boost ego speed + E")
    try:
        boost_process = subprocess.Popen(
            boost_ego_command(data),
            cwd=str(REPO_ROOT),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        threading.Event().wait(0.5)
        command_ok, command_message = send_driver_command(scenario_id, data, "e")
        stdout, _ = boost_process.communicate(timeout=12)
        if stdout.strip():
            for line in stdout.splitlines():
                state.append("log", line)
        boost_ok = boost_process.returncode == 0
        boost_message = "boosted" if boost_ok else f"boost failed with code {boost_process.returncode}"
    except Exception as exc:
        state.append("error", f"辅助启控失败: {exc}")
        return jsonify({"ok": False, "message": str(exc)}), 400

    ok = boost_ok and command_ok
    message = "kickstarted" if ok else f"boost={boost_message}, command={command_message}"
    if ok:
        state.append("ok", "辅助启控完成")
    else:
        state.append("error", f"辅助启控失败: {message}")
    return jsonify({"ok": ok, "message": message}), (200 if ok else 400)


@app.get("/api/scenarios/<scenario_id>/logs")
def api_logs(scenario_id: str) -> Response:
    if scenario_dir_for(scenario_id) is None:
        return jsonify({"error": "scenario not found"}), 404
    since = int(request.args.get("since", "0") or "0")
    cursor, logs = get_state(scenario_id).snapshot_logs(since)
    return jsonify({"cursor": cursor, "logs": logs, "state": get_state(scenario_id).snapshot()})


@app.get("/api/scenarios/<scenario_id>/health")
def api_health(scenario_id: str) -> Response:
    scenario_dir = scenario_dir_for(scenario_id)
    if scenario_dir is None:
        return jsonify({"error": "scenario not found"}), 404
    data = load_yaml_file(scenario_dir / "scenario.yaml")
    health = health_for(data)
    if request.args.get("log", "1") != "0":
        get_state(scenario_id).append(
            "info",
            "健康检查: CARLA={carla}, PUB={bridge_pub}, CONTROL={bridge_control}".format(**health),
        )
    return jsonify(health)


@app.get("/api/scenarios/<scenario_id>/readme")
def api_readme(scenario_id: str) -> Response:
    scenario_dir = scenario_dir_for(scenario_id)
    if scenario_dir is None:
        return jsonify({"error": "scenario not found"}), 404
    readme = scenario_dir / "README.md"
    if not readme.exists():
        return jsonify({"error": "README not found"}), 404
    return Response(readme.read_text(encoding="utf-8"), mimetype="text/plain; charset=utf-8")


if __name__ == "__main__":
    print(f"GAASD-CARLA Scenario Panel: http://127.0.0.1:{DEFAULT_PORT}")
    app.run(host="0.0.0.0", port=DEFAULT_PORT, debug=False, threaded=True)
