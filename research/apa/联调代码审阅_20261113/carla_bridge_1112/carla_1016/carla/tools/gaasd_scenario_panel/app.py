#!/usr/bin/env python3
from __future__ import annotations

import ipaddress
import http.client
import os
import json
import re
import signal
import socket
import subprocess
import sys
import struct
import threading
import time
import urllib.error
import urllib.request
import zlib
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from flask import Flask, jsonify, render_template, request, Response
from apa_input_map import build_input_map
from apa_parking_display import build_parking_display

try:
    import yaml
except ImportError:  # pragma: no cover - fallback for minimal environments
    yaml = None

try:
    import zmq
except ImportError:  # pragma: no cover - operational fallback
    zmq = None


PANEL_DIR = Path(__file__).resolve().parent
REPO_ROOT = PANEL_DIR.parents[1]
SCENARIOS_DIR = REPO_ROOT / "scenarios"
PARAMETER_STATE_DIR = Path.home() / ".config" / "gaasd-scenario-panel" / "parameters"
DEFAULT_PORT = int(os.environ.get("GAASD_PANEL_PORT", "8088"))
MAX_LOG_LINES = 3000
SCENARIO_ID_PATTERN = re.compile(r"^[A-Za-z0-9._-]+$")
DRIVER_COMMAND_KEYS = {
    "e": "E 待命启控/在控降速",
    "q": "Q 待命恢复/在控增速",
    "t": "T 减小时距",
    "r": "R 增大时距",
    "w": "W 驾驶员油门",
    "c": "C 取消 ACC",
    "s": "S 制动退出",
    "0": "0 释放踏板",
}
DRIVER_COMMAND_TOPIC = "gaasd.carla.driver_command.v1"
DRIVER_VEHICLE_TOPIC = "gaasd.carla.driver_vehicle_cmd.v1"
APA_COMMAND_TOPIC = "gaasd.carla.apa_command.v1"
APA_STATUS_TOPIC = "gaasd.carla.apa_status.v1"
APA_TASK_LOCK = threading.Lock()
APA_PENDING_COMMANDS: Dict[str, Dict[str, Any]] = {}
CAMERA_FRAME_LOCK = threading.Lock()
CAMERA_FRAMES: Dict[Tuple[str, str], Tuple[bytes, str, float, str, int]] = {}


def _png_chunk(tag: bytes, data: bytes) -> bytes:
    return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)


def camera_placeholder_png() -> bytes:
    width, height = 16, 9
    raw = b"".join(b"\x00" + bytes([11, 18, 32]) * width for _ in range(height))
    return (
        b"\x89PNG\r\n\x1a\n"
        + _png_chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
        + _png_chunk(b"IDAT", zlib.compress(raw, 1))
        + _png_chunk(b"IEND", b"")
    )


CAMERA_PLACEHOLDER_PNG = camera_placeholder_png()
DRIVER_COMMAND_TYPES = {
    "0": 0,
    "e": 1,
    "q": 2,
    "t": 3,
    "r": 4,
    "w": 5,
    "s": 6,
    "c": 7,
}
DRIVER_COMMAND_DESCRIPTIONS = {
    "0": "驾驶员踏板释放",
    "e": "待命时按当前车速启控；在控时降低设定速度",
    "q": "待命有历史时恢复控制；在控时提高设定速度",
    "t": "减小目标时距",
    "r": "增大目标时距",
    "w": "驾驶员持续踩下油门",
    "s": "驾驶员持续制动并退出 ACC",
    "c": "取消 ACC 控制",
}
LKS_DRIVER_STATE_TOPIC = "gaasd.carla.driver_state_cmd.v1"
EGO_STATE_TOPIC = "gaasd.carla.ego_state.v1"
LEAD_VEHICLE_TOPIC = "gaasd.carla.lead_vehicle.v1"
CHASSIS_FEEDBACK_TOPIC = "gaasd.carla.chassis_feedback.v1"
TELEMETRY_TOPIC_PREFIX = "gaasd.carla."
TELEMETRY_STALE_SEC = 1.5
CARLA_FRAME_PROFILES = {
    "stable": {
        "label": "稳定 20 FPS",
        "extra_args": "-windowed -Resx=800 -Resy=600 -fps=20 -nosound -graphicsadapter=0 -norhithread",
        "fixed_delta_seconds": "0.05",
    },
    "normal": {
        "label": "正常 30 FPS",
        "extra_args": "-windowed -Resx=800 -Resy=600 -fps=30 -nosound -graphicsadapter=0 -norhithread",
        "fixed_delta_seconds": "0.03333333333333333",
    },
}
CARLA_QUALITY_PROFILES = {
    "low": "省资源画质 (Low)",
    "normal": "普通画质 (Epic)",
}

app = Flask(__name__)
app.config["TEMPLATES_AUTO_RELOAD"] = True


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


class BridgeTelemetryMonitor:
    def __init__(self, endpoint: str) -> None:
        self.endpoint = endpoint
        self.lock = threading.Lock()
        self.messages: Dict[str, Dict[str, Any]] = {}
        self.thread = threading.Thread(
            target=self._run,
            name=f"bridge-telemetry-{endpoint.rsplit(':', 1)[-1]}",
            daemon=True,
        )
        self.thread.start()

    def _run(self) -> None:
        if zmq is None:
            return
        context = zmq.Context.instance()
        subscriber = context.socket(zmq.SUB)
        subscriber.setsockopt(zmq.LINGER, 0)
        subscriber.setsockopt_string(zmq.SUBSCRIBE, TELEMETRY_TOPIC_PREFIX)
        subscriber.connect(self.endpoint)
        try:
            while True:
                if subscriber.poll(250, zmq.POLLIN) == 0:
                    continue
                parts = subscriber.recv_multipart()
                if len(parts) < 2:
                    continue
                topic = parts[0].decode("utf-8", errors="replace")
                try:
                    message = json.loads(parts[-1].decode("utf-8"))
                except (UnicodeDecodeError, ValueError):
                    continue
                payload = message.get("payload") if isinstance(message, dict) else None
                if not isinstance(payload, dict):
                    continue
                header = message.get("header")
                with self.lock:
                    self.messages[topic] = {
                        "received_at": time.time(),
                        "header": header if isinstance(header, dict) else {},
                        "payload": payload,
                    }
        finally:
            subscriber.close(0)

    def snapshot(self) -> Dict[str, Dict[str, Any]]:
        with self.lock:
            return {
                topic: {
                    "received_at": entry["received_at"],
                    "header": dict(entry["header"]),
                    "payload": dict(entry["payload"]),
                }
                for topic, entry in self.messages.items()
            }


class AccDiagnosticState:
    def __init__(
        self,
        initial_time_gap_s: float = 1.8,
        time_gap_step_s: float = 0.2,
        min_time_gap_s: float = 1.0,
        max_time_gap_s: float = 5.0,
        min_distance_m: float = 5.0,
    ) -> None:
        self.lock = threading.Lock()
        self.initial_time_gap_s = float(initial_time_gap_s)
        self.time_gap_step_s = float(time_gap_step_s)
        self.min_time_gap_s = float(min_time_gap_s)
        self.max_time_gap_s = float(max_time_gap_s)
        self.min_distance_m = float(min_distance_m)
        self.desired_time_gap_s = self.initial_time_gap_s
        self.last_command: Optional[Dict[str, Any]] = None

    def reset(self) -> None:
        with self.lock:
            self.desired_time_gap_s = self.initial_time_gap_s
            self.last_command = None

    def record_command(self, key: str, control_active: Optional[bool]) -> None:
        normalized = key.strip().lower()
        estimated_applied = True
        with self.lock:
            if normalized in {"t", "r"}:
                estimated_applied = control_active is True
                if estimated_applied:
                    direction = -1.0 if normalized == "t" else 1.0
                    candidate = self.desired_time_gap_s + direction * self.time_gap_step_s
                    self.desired_time_gap_s = max(
                        self.min_time_gap_s,
                        min(self.max_time_gap_s, candidate),
                    )
            self.last_command = {
                "key": normalized,
                "label": DRIVER_COMMAND_KEYS.get(normalized, normalized.upper()),
                "timestamp": time.time(),
                "estimated_applied": estimated_applied,
                "control_active": control_active,
            }

    def snapshot(self) -> Dict[str, Any]:
        with self.lock:
            return {
                "desired_time_gap_s": self.desired_time_gap_s,
                "time_gap_step_s": self.time_gap_step_s,
                "min_time_gap_s": self.min_time_gap_s,
                "max_time_gap_s": self.max_time_gap_s,
                "min_distance_m": self.min_distance_m,
                "last_command": dict(self.last_command) if self.last_command else None,
            }


STATES: Dict[str, ScenarioState] = {}
CONTROL_PUBLISHERS: Dict[str, Any] = {}
CONTROL_PUBLISHERS_LOCK = threading.Lock()
VEHICLE_STATE_LOCK = threading.Lock()
VEHICLE_STATE: Dict[str, Tuple[Dict[str, Any], float]] = {}
VEHICLE_PUBLISH_THREAD: Optional[threading.Thread] = None
TELEMETRY_MONITORS: Dict[str, BridgeTelemetryMonitor] = {}
TELEMETRY_MONITORS_LOCK = threading.Lock()
ACC_DIAGNOSTIC_STATES: Dict[str, AccDiagnosticState] = {}
ACC_DIAGNOSTIC_STATES_LOCK = threading.Lock()
PANEL_SHUTDOWN_LOCK = threading.Lock()
PANEL_SHUTDOWN_STARTED = False


def carla_frame_environment(mode: str) -> Dict[str, str]:
    profile = CARLA_FRAME_PROFILES.get(mode)
    if profile is None:
        raise ValueError(f"unsupported CARLA frame mode: {mode}")
    return {
        "CARLA_EXTRA_ARGS": profile["extra_args"],
        "CARLA_FIXED_DELTA_SECONDS": profile["fixed_delta_seconds"],
    }


def carla_quality_environment(mode: str) -> Dict[str, str]:
    if mode not in CARLA_QUALITY_PROFILES:
        raise ValueError(f"unsupported CARLA quality mode: {mode}")
    return {"CARLA_QUALITY_MODE": mode}


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


def parameter_schema(data: Dict[str, Any]) -> List[Dict[str, Any]]:
    raw = data.get("parameters") or []
    return [item for item in raw if isinstance(item, dict) and item.get("key") and item.get("env")]


def parameter_state_path(scenario_id: str) -> Path:
    return PARAMETER_STATE_DIR / f"{scenario_id}.json"


def parameter_defaults(data: Dict[str, Any]) -> Dict[str, Any]:
    return {str(item["key"]): item.get("default") for item in parameter_schema(data)}


def load_parameter_values(scenario_id: str, data: Dict[str, Any]) -> Dict[str, Any]:
    values = parameter_defaults(data)
    path = parameter_state_path(scenario_id)
    if path.exists():
        try:
            saved = json.loads(path.read_text(encoding="utf-8"))
            if isinstance(saved, dict):
                for key in values:
                    if key in saved:
                        values[key] = saved[key]
        except (OSError, ValueError):
            pass
    return values


def validate_parameter_values(
    data: Dict[str, Any], payload: Dict[str, Any]
) -> Tuple[bool, Any]:
    schema = parameter_schema(data)
    if not schema:
        return False, "scenario does not expose parameters"
    current = parameter_defaults(data)
    groups: Dict[str, List[float]] = {}
    for item in schema:
        key = str(item["key"])
        raw = payload.get(key, current[key])
        try:
            value = float(raw)
        except (TypeError, ValueError):
            return False, f"{key} must be numeric"
        minimum = float(item.get("min", value))
        maximum = float(item.get("max", value))
        if value < minimum or value > maximum:
            return False, f"{key} must be in [{minimum}, {maximum}]"
        if item.get("integer"):
            if not value.is_integer():
                return False, f"{key} must be an integer"
            current[key] = int(value)
        else:
            current[key] = value
        sum_group = str(item.get("sum_group", ""))
        if sum_group:
            groups.setdefault(sum_group, []).append(value)
    for group, group_values in groups.items():
        if abs(sum(group_values) - 1.0) > 1.0e-6:
            return False, f"{group} must sum to 1.0"
    return True, current


def save_parameter_values(scenario_id: str, values: Dict[str, Any]) -> None:
    PARAMETER_STATE_DIR.mkdir(parents=True, exist_ok=True)
    path = parameter_state_path(scenario_id)
    temporary = path.with_suffix(".tmp")
    temporary.write_text(
        json.dumps(values, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    temporary.replace(path)


def parameter_environment(data: Dict[str, Any], values: Dict[str, Any]) -> Dict[str, str]:
    result: Dict[str, str] = {}
    for item in parameter_schema(data):
        key = str(item["key"])
        if key in values:
            result[str(item["env"])] = str(values[key])
    return result


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
        "control_profile": data.get(
            "control_profile", "lks" if scenario_id.lower().startswith("lks") else "acc"
        ),
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
        "has_parameters": bool(parameter_schema(data)),
        "scripts": scenario_scripts(scenario_dir),
        "state": state.snapshot(),
    }


def scenario_created_at_sort_key(item: Dict[str, Any]) -> Tuple[Any, ...]:
    raw_value = str(item.get("created_at") or "").strip()
    if not raw_value:
        return (0, 0, 0, 0, 0, 0, 0, str(item.get("id", "")))

    normalized = re.sub(
        r"\s*([+-]\d{2})(\d{2})$",
        r"\1:\2",
        raw_value,
    )
    if normalized.endswith("Z"):
        normalized = normalized[:-1] + "+00:00"
    try:
        created_at = datetime.fromisoformat(normalized)
    except ValueError:
        return (0, 0, 0, 0, 0, 0, 0, str(item.get("id", "")))
    return (
        1,
        created_at.year,
        created_at.month,
        created_at.day,
        created_at.hour,
        created_at.minute,
        created_at.second,
        str(item.get("id", "")),
    )


def list_scenarios() -> List[Dict[str, Any]]:
    if not SCENARIOS_DIR.exists():
        return []
    items = []
    for path in sorted(SCENARIOS_DIR.iterdir()):
        if path.is_dir() and (path / "scenario.yaml").exists():
            items.append(scenario_summary(path))
    items.sort(key=scenario_created_at_sort_key)
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


def run_script_job(
    scenario_id: str,
    action: str,
    command: List[str],
    env_overrides: Optional[Dict[str, str]] = None,
) -> Tuple[bool, str]:
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
        if env_overrides:
            env.update(env_overrides)
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


def terminate_running_panel_jobs() -> None:
    for state in STATES.values():
        with state.lock:
            process = state.process
        if process is None or process.poll() is not None:
            continue
        try:
            process.terminate()
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=2)
        except OSError:
            pass


def stop_all_scenario_processes() -> List[str]:
    failures: List[str] = []
    if not SCENARIOS_DIR.exists():
        return failures

    for scenario_dir in sorted(SCENARIOS_DIR.iterdir()):
        if not scenario_dir.is_dir() or not (scenario_dir / "scenario.yaml").exists():
            continue
        stop_script = scenario_stop_script(scenario_dir)
        if not stop_script.exists():
            continue
        print(f"[Panel] stopping scenario resources: {scenario_dir.name}", flush=True)
        try:
            result = subprocess.run(
                ["bash", str(stop_script)],
                cwd=str(REPO_ROOT),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=30,
                check=False,
            )
        except Exception as exc:
            failures.append(f"{scenario_dir.name}: {exc}")
            continue
        if result.stdout.strip():
            print(result.stdout.rstrip(), flush=True)
        if result.returncode != 0:
            failures.append(
                f"{scenario_dir.name}: stop exited with code {result.returncode}"
            )
    return failures


def panel_pid_file() -> Path:
    runtime_root = Path(os.environ.get("XDG_RUNTIME_DIR") or "/tmp")
    return runtime_root / "gaasd-scenario-panel" / "panel.pid"


def remove_panel_pid_file() -> None:
    path = panel_pid_file()
    try:
        if path.exists() and path.read_text(encoding="utf-8").strip() == str(os.getpid()):
            path.unlink()
    except OSError:
        pass


def panel_shutdown_worker() -> None:
    terminate_running_panel_jobs()
    failures = stop_all_scenario_processes()
    for failure in failures:
        print(f"[Panel] shutdown warning: {failure}", flush=True)
    remove_panel_pid_file()
    print("[Panel] scenario resources stopped; shutting down panel", flush=True)
    os.kill(os.getpid(), signal.SIGTERM)


def start_panel_shutdown() -> bool:
    global PANEL_SHUTDOWN_STARTED
    with PANEL_SHUTDOWN_LOCK:
        if PANEL_SHUTDOWN_STARTED:
            return False
        PANEL_SHUTDOWN_STARTED = True
    threading.Thread(
        target=panel_shutdown_worker,
        name="panel-shutdown",
        daemon=False,
    ).start()
    return True


def socket_open(host: str, port: int, timeout: float = 0.35) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def local_pangu_processes_running(pangu: Dict[str, Any]) -> bool:
    expected = set(pangu.get("processes") or [pangu.get("process_name", "apa")])
    app_name = str(pangu.get("app_name", "apa"))
    found = set()
    for entry in Path("/proc").glob("[0-9]*/cmdline"):
        try:
            args = entry.read_bytes().decode(errors="replace").strip("\0").split("\0")
        except OSError:
            continue
        if not args or Path(args[0]).name != "dataflow_runner":
            continue
        if f"--app_module={app_name}" in args:
            found.update(name for name in expected if f"--process_name={name}" in args)
    return bool(expected) and expected <= found


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
    runtime = str(pangu.get("runtime", "docker"))
    if pangu and runtime in {"auto", "local"}:
        pangu_process_ok = local_pangu_processes_running(pangu)
    if pangu and not pangu_process_ok and runtime != "local":
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
    if data.get("control_profile") == "apa":
        telemetry = build_apa_telemetry_snapshot(get_telemetry_monitor(bridge_publish_endpoint(data)).snapshot())
        carla_ok = bool(telemetry["online"] and telemetry["initial_pose"] and telemetry["vehicle_feedback_available"])
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


def camera_http_base(data: Dict[str, Any]) -> str:
    bridge = data.get("bridge") or {}
    return str(bridge.get("camera_http", "http://127.0.0.1:5704")).rstrip("/")


def fetch_camera_frame(base_url: str, view: str = "rear", session_id: str = "", revision: int = 0) -> Tuple[bytes, str]:
    cache_key = (base_url, view)
    request_obj = urllib.request.Request(f"{base_url}/{view}.jpg", headers={"Accept": "image/jpeg"})
    try:
        with urllib.request.urlopen(request_obj, timeout=0.35) as response:
            payload = response.read()
            mime = str(response.headers.get_content_type() or "image/jpeg")
            source_session = response.headers.get("X-Camera-Session", session_id)
            source_revision = int(response.headers.get("X-Camera-Revision", "0"))
            source_age = float(response.headers.get("X-Camera-Age-Sec", "0"))
            if (source_session == session_id and source_revision == revision and 0.0 <= source_age <= TELEMETRY_STALE_SEC and
                    mime == "image/jpeg" and payload.startswith(b"\xff\xd8") and payload.endswith(b"\xff\xd9")):
                with CAMERA_FRAME_LOCK:
                    CAMERA_FRAMES[cache_key] = (payload, mime, time.monotonic(), session_id, revision)
                return payload, mime
    except (urllib.error.URLError, http.client.HTTPException, TimeoutError, ValueError, OSError):
        pass
    with CAMERA_FRAME_LOCK:
        cached = CAMERA_FRAMES.get(cache_key)
        if cached is None or cached[3:] != (session_id, revision):
            # A new map or reset pose must not inherit the old view or its age.
            cached = CAMERA_FRAMES[cache_key] = (b"", "image/png", 0.0, session_id, revision)
    return (cached[0], cached[1]) if cached[0] else (CAMERA_PLACEHOLDER_PNG, "image/png")


def camera_stream_status(base_url: str, session_id: str = "", revision: int = 0) -> Dict[str, str]:
    now = time.monotonic()
    states = {}
    with CAMERA_FRAME_LOCK:
        for view in ("rear", "overhead"):
            cached = CAMERA_FRAMES.get((base_url, view))
            if cached is None or cached[3:] != (session_id, revision):
                states[view] = "waiting"
            elif not cached[0]:
                states[view] = "unavailable"
            elif 0.0 <= now - cached[2] <= TELEMETRY_STALE_SEC:
                states[view] = "live"
            else:
                states[view] = "stale"
    return states


def build_apa_telemetry_snapshot(messages: Dict[str, Dict[str, Any]], now: Optional[float] = None) -> Dict[str, Any]:
    current_time = time.time() if now is None else float(now)
    ego, ego_received = telemetry_entry(messages, EGO_STATE_TOPIC)
    status, status_received = telemetry_entry(messages, APA_STATUS_TOPIC)
    header = (messages.get(APA_STATUS_TOPIC) or {}).get("header") or {}
    latest_received = status_received
    age_sec = current_time - latest_received if latest_received else None
    online = age_sec is not None and 0.0 <= age_sec <= TELEMETRY_STALE_SEC
    feedback_available = status.get("vehicle_feedback_available", True) is not False
    ego_matches = (bool(status.get("session_id")) and
                   ego.get("session_id") == status.get("session_id") and
                   ego.get("task_id") == status.get("task_id") and
                   0.0 <= current_time - ego_received <= TELEMETRY_STALE_SEC)
    speed = finite_number(status.get("vehicle_speed"))
    if speed is None and ego_matches:
        speed = finite_number(ego.get("vehicle_speed", ego.get("VehicleSpeed")))
    gear = str(status.get("gear") or "").upper()
    if gear not in {"P", "R", "N", "D"}:
        gear_pos = finite_number(ego.get("gear_pos", ego.get("GearPos"))) if ego_matches else None
        gear = {0: "N", 1: "D", 2: "R", 3: "P"}.get(gear_pos, "")
    parking_enabled = bool(status.get("parking_enabled", False))
    result = {
        "online": online,
        "age_sec": age_sec,
        "status_received_at": status_received,
        "carla_frame_id": finite_number(header.get("frame_id")),
        "sim_time_sec": finite_number(header.get("sim_time_sec")),
        "session_id": status.get("session_id", ""),
        "task_id": status.get("task_id"),
        "map_name": status.get("map_name", ""),
        "parking_map": status.get("parking_map", {}),
        "map_choices": status.get("map_choices", []),
        "map_transition": status.get("map_transition", {}),
        "history_revision": status.get("history_revision", 0),
        "camera_revision": status.get("camera_revision", 0),
        "vehicle_geometry": status.get("vehicle_geometry", {}),
        "initial_pose": status.get("initial_pose"),
        "start_mode": status.get("start_mode", "slot"),
        "slot_choices": status.get("slot_choices", []),
        "slot_occupancy": status.get("slot_occupancy", {}),
        "geometry": status.get("geometry", {}),
        "vehicle_feedback_available": feedback_available,
        "action_switch_max_speed_mps": finite_number(status.get("action_switch_max_speed_mps")),
        "last_task_command": status.get("last_task_command", {}),
        "safety_state": status.get("safety_state", {}),
        "sync_error": status.get("sync_error", ""),
        "control_status": status.get("control_status", ""),
        "control_phase": status.get("control_phase", 0),
        "control_diagnostic": status.get("control_diagnostic", {}),
        "replan": status.get("replan", {}),
        "gear_feedback_source": status.get("gear_feedback_source", ""),
        "parking_enabled": parking_enabled,
        "planning_only": bool(status.get("planning_only", False)),
        "phase": str(status.get("phase") or ("park" if parking_enabled else "manual")),
        "algorithm_linked": online and bool(status.get("algorithm_linked", False)),
        "algorithm_has_output": online and bool(status.get("algorithm_has_output", False)),
        "waiting_for_path": bool(status.get("waiting_for_path", False)),
        "slot_id": status.get("slot_id"),
        "slot_entry": status.get("slot_entry", 1),
        "slot_kind": status.get("slot_kind") or "",
        "physical_slot_id": status.get("physical_slot_id", status.get("slot_id")),
        "parking_direction": status.get("parking_direction") or "",
        "path_point_count": int(finite_number(status.get("path_point_count")) or 0),
        "ego_speed_mps": speed,
        "gear": gear,
        "applied_gear": str(status.get("applied_gear") or gear),
        "throttle": finite_number(status.get("throttle")),
        "brake": finite_number(status.get("brake")),
        "steer": finite_number(status.get("steer")),
        "hand_brake": None if "hand_brake" not in status else bool(status.get("hand_brake")),
        "ignition_switch": str(status.get("ignition_switch") or ""),
        "camera_ready": bool(status.get("camera_ready", False)),
        "camera_http": status.get("camera_http") or "",
        "simulation_rate": finite_number(status.get("simulation_rate")) if online else None,
        "parking_action": int(finite_number(status.get("parking_action")) or 1),
        "plan_success": None if "plan_success" not in status else status.get("plan_success"),
        "algorithm_gear_req": int(finite_number(status.get("algorithm_gear_req")) or 0),
        "algorithm_gear": str(status.get("algorithm_gear") or ""),
        "algorithm_target_accel": finite_number(status.get("algorithm_target_accel")),
        "algorithm_target_speed": finite_number(status.get("algorithm_target_speed")),
        "algorithm_steer_rad": finite_number(status.get("algorithm_steer_rad")),
        "algorithm_message": str(status.get("algorithm_message") or ""),
        "expected_speed_mps": finite_number(status.get("expected_speed_mps")),
        "expected_accel_mps2": finite_number(status.get("expected_accel_mps2")),
        "expected_brake": finite_number(status.get("expected_brake")),
        "expected_throttle": finite_number(status.get("expected_throttle")),
        "expected_gear": str(status.get("expected_gear") or ""),
        "expected_steer": finite_number(status.get("expected_steer")),
        "expected_steer_wheel_deg": finite_number(status.get("expected_steer_wheel_deg")),
        "actual_steer_wheel_deg": finite_number(status.get("actual_steer_wheel_deg")),
        "actual_speed_mps": finite_number(status.get("actual_speed_mps", speed)),
        "actual_accel_mps2": finite_number(status.get("actual_accel_mps2")),
        "expected_x": finite_number(status.get("expected_x")),
        "expected_y": finite_number(status.get("expected_y")),
        "vehicle_x": finite_number(status.get("vehicle_x")),
        "vehicle_y": finite_number(status.get("vehicle_y")),
        "vehicle_yaw": finite_number(status.get("vehicle_yaw")),
        "algorithm_path_x": status.get("algorithm_path_x") if isinstance(status.get("algorithm_path_x"), list) else [],
        "algorithm_path_y": status.get("algorithm_path_y") if isinstance(status.get("algorithm_path_y"), list) else [],
        "algorithm_path_yaw": status.get("algorithm_path_yaw") if isinstance(status.get("algorithm_path_yaw"), list) else [],
        "algorithm_path_odo": status.get("algorithm_path_odo") if isinstance(status.get("algorithm_path_odo"), list) else [],
        "trajectory": status.get("trajectory") if isinstance(status.get("trajectory"), dict) else {},
        "actual_path_x": status.get("actual_path_x") if isinstance(status.get("actual_path_x"), list) else [],
        "actual_path_y": status.get("actual_path_y") if isinstance(status.get("actual_path_y"), list) else [],
        "slot_corners": status.get("slot_corners") if isinstance(status.get("slot_corners"), list) else [],
        "input_map": build_input_map(messages, status, current_time, TELEMETRY_STALE_SEC)
            if online and feedback_available else {"available": False,
                "reason": "feedback_unavailable" if online else "offline"},
    }
    if not feedback_available:
        # A fresh status heartbeat can report a vehicle-feedback fault. Never
        # present its cached measurements (or another task's ego packet) as live.
        for key in ("ego_speed_mps", "actual_speed_mps", "actual_accel_mps2", "actual_steer_wheel_deg",
                    "vehicle_x", "vehicle_y", "vehicle_yaw", "throttle", "brake", "steer", "hand_brake"):
            result[key] = None
        result["gear"] = result["applied_gear"] = ""
    return result


def bridge_apa_command_endpoint(data: Dict[str, Any]) -> str:
    bridge = data.get("bridge") or {}
    return str(bridge.get("zmq_apa_command", "tcp://127.0.0.1:5703"))


def bridge_publish_endpoint(data: Dict[str, Any]) -> str:
    bridge = data.get("bridge") or {}
    return str(bridge.get("zmq_pub", "tcp://127.0.0.1:5701"))


def get_telemetry_monitor(endpoint: str) -> BridgeTelemetryMonitor:
    with TELEMETRY_MONITORS_LOCK:
        monitor = TELEMETRY_MONITORS.get(endpoint)
        if monitor is None:
            monitor = BridgeTelemetryMonitor(endpoint)
            TELEMETRY_MONITORS[endpoint] = monitor
        return monitor


def acc_diagnostic_config(data: Dict[str, Any]) -> Dict[str, float]:
    diagnostics = data.get("diagnostics") or {}
    raw = diagnostics.get("acc") or {}
    return {
        "initial_time_gap_s": float(raw.get("initial_time_gap_s", 1.8)),
        "time_gap_step_s": float(raw.get("time_gap_step_s", 0.2)),
        "min_time_gap_s": float(raw.get("min_time_gap_s", 1.0)),
        "max_time_gap_s": float(raw.get("max_time_gap_s", 5.0)),
        "min_distance_m": float(raw.get("min_distance_m", 5.0)),
    }


def get_acc_diagnostic_state(
    scenario_id: str, data: Dict[str, Any]
) -> AccDiagnosticState:
    with ACC_DIAGNOSTIC_STATES_LOCK:
        diagnostic = ACC_DIAGNOSTIC_STATES.get(scenario_id)
        if diagnostic is None:
            diagnostic = AccDiagnosticState(**acc_diagnostic_config(data))
            ACC_DIAGNOSTIC_STATES[scenario_id] = diagnostic
        return diagnostic


def reset_acc_diagnostic_state(scenario_id: str, data: Dict[str, Any]) -> None:
    get_acc_diagnostic_state(scenario_id, data).reset()


def finite_number(value: Any) -> Optional[float]:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if number != number or number in {float("inf"), float("-inf")}:
        return None
    return number


def nested_number(payload: Dict[str, Any], *path: str) -> Optional[float]:
    value: Any = payload
    for key in path:
        if not isinstance(value, dict) or key not in value:
            return None
        value = value[key]
    return finite_number(value)


def telemetry_entry(
    messages: Dict[str, Dict[str, Any]], topic: str
) -> Tuple[Dict[str, Any], float]:
    entry = messages.get(topic) or {}
    payload = entry.get("payload") or {}
    received_at = finite_number(entry.get("received_at")) or 0.0
    return (payload if isinstance(payload, dict) else {}), received_at


def bounded_measurement(value: Optional[float], maximum: float = 9999.0) -> Optional[float]:
    if value is None or abs(value) > maximum:
        return None
    return value


def build_acc_telemetry_snapshot(
    messages: Dict[str, Dict[str, Any]],
    diagnostic: Dict[str, Any],
    now: Optional[float] = None,
) -> Dict[str, Any]:
    current_time = time.time() if now is None else float(now)
    ego, ego_received = telemetry_entry(messages, EGO_STATE_TOPIC)
    lead, lead_received = telemetry_entry(messages, LEAD_VEHICLE_TOPIC)
    chassis, chassis_received = telemetry_entry(messages, CHASSIS_FEEDBACK_TOPIC)
    latest_received = max(ego_received, lead_received, chassis_received)
    age_sec = current_time - latest_received if latest_received else None
    online = age_sec is not None and 0.0 <= age_sec <= TELEMETRY_STALE_SEC
    ego_fresh = 0.0 <= current_time - ego_received <= TELEMETRY_STALE_SEC
    lead_fresh = 0.0 <= current_time - lead_received <= TELEMETRY_STALE_SEC
    chassis_fresh = 0.0 <= current_time - chassis_received <= TELEMETRY_STALE_SEC

    chassis_speed = finite_number(chassis.get("speed_mps")) if chassis_fresh else None
    ego_speed = chassis_speed
    if ego_speed is None and ego_fresh:
        ego_speed = nested_number(ego, "velocity", "speed_mps")

    lead_valid = bool(lead.get("valid", False)) and lead_fresh
    lead_speed = finite_number(lead.get("lead_speed_mps")) if lead_valid else None
    clearance = bounded_measurement(
        finite_number(lead.get("clearance_m")) if lead_valid else None
    )
    actual_time_gap = bounded_measurement(
        finite_number(lead.get("time_gap_sec")) if lead_valid else None,
        maximum=999.0,
    )
    ttc = bounded_measurement(
        finite_number(lead.get("ttc_sec")) if lead_valid else None,
        maximum=999.0,
    )
    control_enabled = bool(chassis.get("control_enable", False)) and chassis_fresh
    if not online:
        control_state = "offline"
    elif not chassis_fresh:
        control_state = "unknown"
    else:
        control_state = "active" if control_enabled else "standby"
    desired_time_gap = finite_number(diagnostic.get("desired_time_gap_s"))
    min_distance = finite_number(diagnostic.get("min_distance_m", 5.0))
    desired_distance = None
    distance_error = None
    if ego_speed is not None and desired_time_gap is not None and min_distance is not None:
        desired_distance = max(ego_speed * desired_time_gap, min_distance)
        if clearance is not None:
            distance_error = clearance - desired_distance

    last_command = diagnostic.get("last_command")
    if isinstance(last_command, dict):
        last_command = dict(last_command)
        timestamp = finite_number(last_command.get("timestamp"))
        last_command["age_sec"] = (
            max(0.0, current_time - timestamp) if timestamp is not None else None
        )
    else:
        last_command = None

    return {
        "online": online,
        "age_sec": max(0.0, age_sec) if age_sec is not None else None,
        "control_state": control_state,
        "control_enabled": control_enabled,
        "lead_valid": lead_valid,
        "ego_speed_mps": ego_speed if online else None,
        "lead_speed_mps": lead_speed if online else None,
        "relative_speed_mps": (
            finite_number(lead.get("relative_speed_mps")) if lead_valid and online else None
        ),
        "clearance_m": clearance if online else None,
        "actual_time_gap_s": actual_time_gap if online else None,
        "ttc_s": ttc if online else None,
        "desired_time_gap_s": desired_time_gap,
        "desired_time_gap_source": "panel_command_estimate",
        "desired_distance_m": desired_distance if online else None,
        "distance_error_m": distance_error if online else None,
        "target_accel_mps2": (
            finite_number(chassis.get("target_accel_mps2")) if chassis_fresh else None
        ),
        "actual_accel_mps2": (
            finite_number(chassis.get("actual_accel_mps2")) if chassis_fresh else None
        ),
        "filtered_accel_mps2": (
            finite_number(chassis.get("filtered_accel_mps2")) if chassis_fresh else None
        ),
        "accel_feedforward_mps2": (
            finite_number(chassis.get("accel_feedforward_mps2"))
            if chassis_fresh
            else None
        ),
        "accel_feedback_mps2": (
            finite_number(chassis.get("accel_feedback_mps2"))
            if chassis_fresh
            else None
        ),
        "accel_integral_error_mps": (
            finite_number(chassis.get("accel_integral_error_mps"))
            if chassis_fresh
            else None
        ),
        "accel_controller_saturated": (
            bool(chassis.get("accel_controller_saturated", False))
            if chassis_fresh
            else None
        ),
        "throttle": finite_number(chassis.get("throttle")) if chassis_fresh else None,
        "brake": finite_number(chassis.get("brake")) if chassis_fresh else None,
        "last_command": last_command,
    }


def current_acc_control_active(endpoint: str) -> Optional[bool]:
    with TELEMETRY_MONITORS_LOCK:
        monitor = TELEMETRY_MONITORS.get(endpoint)
    if monitor is None:
        return None
    chassis, received_at = telemetry_entry(monitor.snapshot(), CHASSIS_FEEDBACK_TOPIC)
    if time.time() - received_at > TELEMETRY_STALE_SEC:
        return None
    return bool(chassis.get("control_enable", False))


def publish_control_message(
    endpoint: str,
    topic: str,
    payload: Dict[str, Any],
    source: str,
    repeat_count: int = 4,
    interval_sec: float = 0.02,
) -> Tuple[bool, str]:
    if zmq is None:
        return False, "pyzmq is not installed"

    with CONTROL_PUBLISHERS_LOCK:
        publisher = CONTROL_PUBLISHERS.get(endpoint)
        if publisher is None:
            context = zmq.Context.instance()
            publisher = context.socket(zmq.PUB)
            publisher.setsockopt(zmq.LINGER, 0)
            publisher.connect(endpoint)
            CONTROL_PUBLISHERS[endpoint] = publisher
            # Avoid the first click being lost to ZMQ PUB/SUB slow-joiner setup.
            time.sleep(0.35)

        topic_bytes = topic.encode("utf-8")
        repeat = max(1, int(repeat_count))
        for index in range(repeat):
            message = {
                "header": {
                    "protocol": "gaasd_carla_bridge",
                    "protocol_version": "0.3.0",
                    "message_type": topic,
                    "timestamp_unix_ms": int(time.time() * 1000),
                    "source": source,
                },
                "payload": payload,
            }
            frame = json.dumps(message, separators=(",", ":")).encode("utf-8")
            publisher.send_multipart([topic_bytes, frame])
            if index + 1 < repeat:
                time.sleep(max(0.0, interval_sec))
    return True, "sent"


def scenario_python(data: Dict[str, Any], fallback: str = "python3") -> str:
    env = data.get("environment") or {}
    return str(env.get("python") or os.environ.get("GAASD_PANEL_PYTHON") or fallback)


def send_driver_command(
    scenario_id: str,
    data: Dict[str, Any],
    key: str,
    write_log: bool = True,
) -> Tuple[bool, str]:
    normalized = key.strip().lower()
    if normalized not in DRIVER_COMMAND_KEYS:
        return False, "unsupported driver command"

    state = get_state(scenario_id)
    endpoint = bridge_control_endpoint(data)
    command_type = DRIVER_COMMAND_TYPES[normalized]
    mode = "level" if normalized in {"w", "s", "0"} else "pulse"
    active = command_type != 0
    if write_log:
        state.append(
            "info", f"发送驾驶指令 {DRIVER_COMMAND_KEYS[normalized]} -> {endpoint}"
        )
    ok, message = publish_control_message(
        endpoint=endpoint,
        topic=DRIVER_COMMAND_TOPIC,
        payload={
            "command_type": command_type,
            "mode": mode,
            "active": active,
            "event_id": time.time_ns(),
            "description": DRIVER_COMMAND_DESCRIPTIONS[normalized],
        },
        source="gaasd_scenario_panel",
        repeat_count=1,
    )
    if not ok:
        if write_log:
            state.append("error", f"驾驶指令发送失败: {message}")
        return False, message
    get_acc_diagnostic_state(scenario_id, data).record_command(
        normalized, current_acc_control_active(bridge_publish_endpoint(data))
    )
    if write_log:
        state.append("ok", f"驾驶指令已发送: {DRIVER_COMMAND_KEYS[normalized]}")
    return True, message


def apa_receipt_matches(receipt: Dict[str, Any], command: Dict[str, Any]) -> bool:
    return (bool(command.get("event_id")) and receipt.get("event_id") == command["event_id"] and
            receipt.get("source_session_id") == command.get("session_id") and
            receipt.get("source_task_id") == command.get("task_id") and
            receipt.get("action") == command.get("action") and
            (command.get("action") != "configure" or "map_name" not in command or
             receipt.get("map_name") == command["map_name"]))


def send_apa_command(scenario_id: str, data: Dict[str, Any], payload: Dict[str, Any]) -> Tuple[Dict[str, Any], int]:
    action = payload.get("action")
    if not isinstance(action, str) or action not in {"plan_only", "park_in", "park_out", "cancel", "reset", "configure", "prepare_map"}:
        return {"error": "不支持的 APA 操作"}, 400
    requested_event = payload.get("event_id")
    if requested_event is not None and (not isinstance(requested_event, str) or
            re.fullmatch(r"[A-Za-z0-9_-]{1,64}", requested_event) is None):
        return {"error": "操作事件号无效"}, 400
    monitor = get_telemetry_monitor(bridge_publish_endpoint(data))
    endpoint = bridge_apa_command_endpoint(data)
    # Serialize submission, not the acknowledgement wait. A cancel may replace
    # an unconfirmed command while that command's HTTP request is still open.
    with APA_TASK_LOCK:
        if get_state(scenario_id).status == "running":
            return {"error": "环境正在启动或停止，请等待完成后再操作"}, 409
        snapshot = build_apa_telemetry_snapshot(monitor.snapshot())
        last_command = snapshot.get("last_task_command") or {}
        if not snapshot["session_id"] or (not snapshot["online"] and action != "cancel"):
            return {"error": "泊车 Bridge 未就绪，请先启动环境或重启 Bridge"}, 409
        if payload.get("session_id") != snapshot["session_id"]:
            return {"error": "Bridge 会话已变化，请等待实时状态更新后重试"}, 409
        if requested_event and last_command.get("event_id") == requested_event:
            if not apa_receipt_matches(last_command, payload):
                return {"error": "操作事件号与原操作身份不一致"}, 409
            running = last_command.get("state") in {"running", "loading"}
            response = {"ok": bool(last_command.get("ok")), "pending": running,
                        "event_id": requested_event, "message": last_command.get("message", ""),
                        "telemetry": snapshot}
            return response, 202 if running else 200 if last_command.get("ok") else 409
        changing_map = (action == "configure" and "map_name" in payload and
                        payload["map_name"] != snapshot["map_name"])
        if action == "prepare_map":
            preparation_scope = payload.get("preparation_scope", "map")
            if preparation_scope not in ("map", "current", "all_slots"):
                return {"error": "地图准备范围无效"}, 400
            preparation = snapshot.get("geometry", {}).get("map_preparation", {})
            available = preparation.get("current_available", preparation.get("available")) if preparation_scope == "current" else preparation.get("available")
            speed = snapshot.get("ego_speed_mps")
            if (snapshot["parking_enabled"] or speed is None or abs(speed) > 0.02 or
                    not available):
                return {"error": "请在支持地图准备的场景退出 APA 并停稳后操作"}, 409
        if changing_map:
            choice = next((row for row in snapshot["map_choices"]
                           if row.get("map_name") == payload["map_name"]), None)
            if choice is None or not choice.get("available"):
                return {"error": "所选地图不可用，请检查地图安装和车位数据"}, 400
            speed = snapshot.get("ego_speed_mps")
            if snapshot["parking_enabled"] or speed is None or abs(speed) > 0.02:
                return {"error": "请先退出 APA 并停稳，再切换地图"}, 409
        pending = APA_PENDING_COMMANDS.get(endpoint)
        if pending and pending["session_id"] != snapshot["session_id"]:
            pending = None
            APA_PENDING_COMMANDS.pop(endpoint, None)
        if (pending and pending.get("unconfirmed") and
                apa_receipt_matches(last_command, pending) and
                last_command.get("state") not in {"running", "loading"}):
            pending = None
            APA_PENDING_COMMANDS.pop(endpoint, None)
        # Resume the original event; Bridge's event cache prevents another reset.
        retrying = (pending and pending.get("unconfirmed") and requested_event == pending["event_id"]
                    and action == pending["action"] and payload.get("task_id") == pending["task_id"])
        if pending and not retrying and (action != "cancel" or
                        (pending["action"] == "cancel" and not pending.get("unconfirmed"))):
            return {"error": "操作尚在确认中；需要停止时请退出 APA", "telemetry": snapshot,
                    "pending_event_id": pending["event_id"],
                    "pending_command": {key: pending[key] for key in
                        ("event_id", "session_id", "task_id", "action", "map_name") if key in pending}}, 409
        cancels_transition = action == "cancel" and (
            (pending and payload.get("task_id") == pending["task_id"]) or
            (last_command.get("source_task_id") == payload.get("task_id") and
             last_command.get("task_id") == snapshot["task_id"]))
        if payload.get("task_id") != snapshot["task_id"] and not cancels_transition:
            return {"error": "车辆任务已变化，请等待实时状态更新后重试"}, 409
        event_id = requested_event or str(time.time_ns())
        command = {key: payload[key] for key in ("map_name", "slot_id", "start_mode", "initial_pose", "preparation_scope") if key in payload}
        command.update(action=action, event_id=event_id, session_id=snapshot["session_id"],
                       task_id=payload["task_id"] if cancels_transition else snapshot["task_id"])
        if action == "cancel" and pending:
            # The start may reach Bridge before its new task ID reaches the UI.
            command["cancel_pending_event_id"] = pending["event_id"]
        elif cancels_transition and last_command.get("event_id"):
            # An acknowledgement can clear the pending ticket before the UI
            # receives the new task identity. Follow only that exact transition.
            command["cancel_pending_event_id"] = last_command["event_id"]
        # Replace the retained park/manual inputs before dispatching the task event.
        # Bind the hold to the old task: after reset it cannot overwrite new state.
        if action in {"reset", "configure", "cancel", "prepare_map"}:
            hold = normalize_vehicle_payload({"apa_phase": "manual", "apa_exit": False,
                "gear": "P", "hand_brake": True, "brake": 1.0, "ignition_switch": "ON",
                "session_id": snapshot["session_id"], "task_id": snapshot["task_id"]})
            with VEHICLE_STATE_LOCK:
                VEHICLE_STATE[endpoint] = (hold, 0.0)
        ok, message = publish_control_message(endpoint=endpoint, topic=APA_COMMAND_TOPIC,
            payload=command, source="gaasd_scenario_panel", repeat_count=3)
        if not ok:
            return {"error": message}, 503
        if pending:
            pending["superseded"] = True
        ticket = dict(command, superseded=False, unconfirmed=False)
        APA_PENDING_COMMANDS[endpoint] = ticket
    deadline = time.monotonic() + (480.0 if changing_map else 8.0)
    next_retry = time.monotonic() + 0.5
    try:
        while time.monotonic() < deadline:
            snapshot = build_apa_telemetry_snapshot(monitor.snapshot())
            receipts = [snapshot.get("last_task_command") or {}]
            if changing_map:
                receipts.append(snapshot.get("map_transition") or {})
            result = next((receipt for receipt in receipts
                           if receipt.get("event_id") == event_id), {})
            if ticket["superseded"]:
                return {"error": "原操作已被退出请求取代", "telemetry": snapshot}, 409
            matched = apa_receipt_matches(result, command)
            if matched and result.get("state") in {"running", "loading"}:
                # Bridge has admitted this event. Geometry preparation can
                # take minutes; finish HTTP submission and poll its receipt.
                with APA_TASK_LOCK:
                    ticket["unconfirmed"] = True
                return {"ok": True, "pending": True, "event_id": event_id,
                        "message": result.get("message", "操作正在执行"), "telemetry": snapshot}, 202
            if matched and result.get("state") not in {"running", "loading"}:
                state = get_state(scenario_id)
                if result.get("ok") and action in {"reset", "configure"}:
                    with state.lock:
                        state.logs.clear()
                state.append("ok" if result.get("ok") else "error", str(result.get("message", "")))
                return {"ok": bool(result.get("ok")), "message": result.get("message", ""),
                    "telemetry": snapshot}, (200 if result.get("ok") else 409)
            if snapshot["session_id"] != command["session_id"] and not (changing_map and matched):
                return {"error": "操作期间 Bridge 已重启，请刷新状态后重试", "telemetry": snapshot}, 409
            if time.monotonic() >= next_retry:
                # Bridge caches results by event_id, so retries cannot repeat a reset.
                with APA_TASK_LOCK:
                    if not ticket["superseded"]:
                        publish_control_message(endpoint=endpoint, topic=APA_COMMAND_TOPIC,
                            payload=command, source="gaasd_scenario_panel", repeat_count=1)
                next_retry = time.monotonic() + 0.5
            time.sleep(0.04)
        with APA_TASK_LOCK:
            if ticket["superseded"]:
                return {"error": "原操作已被退出请求取代", "telemetry": snapshot}, 409
            ticket["unconfirmed"] = True
        return {"error": "尚未收到操作确认，操作可能仍在执行，请先查看实时状态",
                "event_id": event_id, "telemetry": snapshot}, 504
    finally:
        with APA_TASK_LOCK:
            if APA_PENDING_COMMANDS.get(endpoint) is ticket and not ticket["unconfirmed"]:
                APA_PENDING_COMMANDS.pop(endpoint)


def reconcile_apa_pending(scenario_id: str, data: Dict[str, Any], snapshot: Dict[str, Any]) -> None:
    """Finalize a submitted operation when its delayed receipt arrives."""
    endpoint = bridge_apa_command_endpoint(data)
    with APA_TASK_LOCK:
        ticket = APA_PENDING_COMMANDS.get(endpoint)
        if not ticket or not ticket.get("unconfirmed") or ticket.get("superseded"):
            return
        receipt = next((item for item in (snapshot.get("last_task_command"), snapshot.get("map_transition"))
                        if isinstance(item, dict) and item.get("event_id") == ticket["event_id"]), None)
        if not receipt or receipt.get("state") in {"running", "loading"}:
            return
        if not apa_receipt_matches(receipt, ticket):
            return
        APA_PENDING_COMMANDS.pop(endpoint, None)
        state = get_state(scenario_id)
        if receipt.get("ok") and ticket["action"] in {"reset", "configure"}:
            with state.lock:
                state.logs.clear()
        state.append("ok" if receipt.get("ok") else "error", str(receipt.get("message", "")))


def normalize_vehicle_payload(payload: Dict[str, Any]) -> Dict[str, Any]:
    ignition = str(payload.get("ignition_switch", "OFF")).upper()
    if ignition not in {"OFF", "ACC", "ON", "START"}:
        ignition = "OFF"
    gear = str(payload.get("gear", "D")).upper()
    if gear not in {"P", "R", "N", "D"}:
        gear = "D"
    throttle = max(0.0, min(1.0, float(payload.get("throttle", 0.0))))
    if "steering_angle" in payload and "steer" not in payload:
        steer = max(-1.0, min(1.0, float(payload.get("steering_angle", 0.0)) / 720.0))
    else:
        steer = max(-1.0, min(1.0, float(payload.get("steer", 0.0))))
    brake = max(0.0, min(1.0, float(payload.get("brake", 0.0))))
    hand_brake = bool(payload.get("hand_brake", ignition == "OFF"))
    if ignition not in {"ON", "START"}:
        throttle = 0.0
    if ignition == "OFF":
        hand_brake = True
    apa_phase = str(payload.get("apa_phase") or "").strip().lower()
    if apa_phase not in {"manual", "search", "park", "complete"}:
        apa_phase = "park" if bool(payload.get("apa_enable", False)) else "manual"
    try:
        parking_action = int(payload.get("parking_action", 1) or 1)
    except (TypeError, ValueError):
        parking_action = 1
    if parking_action not in {1, 2}:
        parking_action = 1
    steer_rad = payload.get("steer_rad", None)
    normalized = {
        "throttle": throttle,
        "brake": brake,
        "steer": steer,
        "steering_angle": steer * 720.0,
        "hand_brake": hand_brake,
        "gear": gear,
        "ignition_switch": ignition,
        "preserve_steer": bool(payload.get("preserve_steer", False)),
        "apa_enable": apa_phase == "park" or bool(payload.get("apa_enable", False)),
        "apa_phase": apa_phase,
        "apa_exit": bool(payload.get("apa_exit", False)),
        "parking_action": parking_action,
        "lka_enable": bool(payload.get("lka_enable", False)),
        "aeb_enable": bool(payload.get("aeb_enable", False)),
        "acc_enable": bool(payload.get("acc_enable", False)),
        "lc_command": str(payload.get("lc_command", "") or ""),
    }
    if steer_rad is not None:
        normalized["steer_rad"] = max(-1.2, min(1.2, float(steer_rad)))
    if "session_id" in payload:
        normalized["session_id"] = str(payload.get("session_id") or "")
        normalized["task_id"] = payload.get("task_id")
    return normalized


def _vehicle_publish_loop() -> None:
    while True:
        time.sleep(0.05)
        with VEHICLE_STATE_LOCK:
            snapshot = list(VEHICLE_STATE.items())
        for endpoint, (payload, expires_at) in snapshot:
            if expires_at and time.monotonic() >= expires_at:
                continue
            publish_control_message(
                endpoint=endpoint,
                topic=DRIVER_VEHICLE_TOPIC,
                payload=payload,
                source="gaasd_scenario_panel",
                repeat_count=1,
                interval_sec=0.0,
            )


def ensure_vehicle_publisher() -> None:
    global VEHICLE_PUBLISH_THREAD
    if VEHICLE_PUBLISH_THREAD is not None and VEHICLE_PUBLISH_THREAD.is_alive():
        return
    VEHICLE_PUBLISH_THREAD = threading.Thread(target=_vehicle_publish_loop, daemon=True)
    VEHICLE_PUBLISH_THREAD.start()


def send_vehicle_command(data: Dict[str, Any], payload: Dict[str, Any]) -> Tuple[bool, str]:
    bridge = data.get("bridge") or {}
    endpoint = bridge_apa_command_endpoint(data) if bridge.get("zmq_apa_command") else bridge_control_endpoint(data)
    normalized = normalize_vehicle_payload(payload)
    with VEHICLE_STATE_LOCK:
        # Stop replaying APA manual input when browser heartbeats disappear.
        expires_at = time.monotonic() + 0.5 if bridge.get("zmq_apa_command") else 0.0
        VEHICLE_STATE[endpoint] = (normalized, expires_at)
    ensure_vehicle_publisher()
    return publish_control_message(
        endpoint=endpoint,
        topic=DRIVER_VEHICLE_TOPIC,
        payload=normalized,
        source="gaasd_scenario_panel",
        repeat_count=2,
        interval_sec=0.02,
    )


def send_lks_driver_state(
    scenario_id: str, data: Dict[str, Any], brake_pressed: bool, driver_steer_norm: float
) -> Tuple[bool, str]:
    steer = max(-1.0, min(1.0, float(driver_steer_norm)))
    endpoint = bridge_control_endpoint(data)
    state = get_state(scenario_id)
    ok, message = publish_control_message(
        endpoint=endpoint,
        topic=LKS_DRIVER_STATE_TOPIC,
        payload={
            "brake_pressed": bool(brake_pressed),
            "driver_steer_norm": steer,
        },
        source="gaasd_scenario_panel",
        repeat_count=8,
        interval_sec=0.04,
    )
    if not ok:
        state.append("error", f"LKS驾驶输入发送失败: {message}")
        return False, message
    return True, message


def read_scenario_result(scenario_dir: Path, data: Dict[str, Any]) -> Tuple[bool, Any]:
    configured = str((data.get("results") or {}).get("latest_summary", ""))
    if not configured:
        return False, "scenario does not define results.latest_summary"
    path = Path(configured)
    if not path.is_absolute():
        path = scenario_dir / path
    resolved = path.resolve()
    allowed_roots = (Path("/tmp").resolve(), scenario_dir.resolve())
    if not any(resolved == root or root in resolved.parents for root in allowed_roots):
        return False, "result path is outside allowed roots"
    if not resolved.exists():
        return False, f"result not ready: {resolved}"
    try:
        return True, json.loads(resolved.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        return False, str(exc)


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
    configured_root = str(
        env.get("carla_root", Path.home() / "snap/code/app/carla-package")
    )
    carla_root = str(Path(configured_root).expanduser())
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


@app.post("/api/panel/shutdown")
def api_panel_shutdown() -> Response:
    try:
        is_loopback = ipaddress.ip_address(request.remote_addr or "").is_loopback
    except ValueError:
        is_loopback = False
    if not is_loopback:
        return jsonify({"error": "panel shutdown is only available locally"}), 403
    started = start_panel_shutdown()
    return (
        jsonify(
            {
                "ok": True,
                "already_stopping": not started,
                "message": "正在停止 CARLA、Bridge、Pangu 和测试平台后端",
            }
        ),
        202,
    )


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
    data = load_yaml_file(scenario_dir / "scenario.yaml")
    if str(data.get("control_profile", "acc")) == "acc":
        reset_acc_diagnostic_state(scenario_id, data)
    values = load_parameter_values(scenario_id, data)
    env_overrides = parameter_environment(data, values)
    payload = request.get_json(silent=True) or {}
    frame_mode = str(payload.get("carla_frame_mode", "stable"))
    quality_mode = str(payload.get("carla_quality_mode", "low"))
    try:
        env_overrides.update(carla_frame_environment(frame_mode))
    except ValueError:
        return jsonify({"error": "CARLA 帧率模式只支持 stable 或 normal"}), 400
    try:
        env_overrides.update(carla_quality_environment(quality_mode))
    except ValueError:
        return jsonify({"error": "CARLA 画质模式只支持 low 或 normal"}), 400
    get_state(scenario_id).append(
        "info", f"CARLA 帧率模式: {CARLA_FRAME_PROFILES[frame_mode]['label']}"
    )
    get_state(scenario_id).append(
        "info", f"CARLA 画质模式: {CARLA_QUALITY_PROFILES[quality_mode]}"
    )
    if env_overrides:
        get_state(scenario_id).append(
            "info",
            "使用场景参数: " + ", ".join(f"{key}={value}" for key, value in env_overrides.items()),
        )
    ok, message = run_script_job(
        scenario_id, "start", ["bash", str(script)], env_overrides=env_overrides
    )
    return jsonify({"ok": ok, "message": message}), (202 if ok else 409)


@app.get("/api/scenarios/<scenario_id>/parameters")
def api_parameters(scenario_id: str) -> Response:
    scenario_dir = scenario_dir_for(scenario_id)
    if scenario_dir is None:
        return jsonify({"error": "scenario not found"}), 404
    data = load_yaml_file(scenario_dir / "scenario.yaml")
    schema = parameter_schema(data)
    if not schema:
        return jsonify({"error": "scenario does not expose parameters"}), 404
    return jsonify({"schema": schema, "values": load_parameter_values(scenario_id, data)})


@app.post("/api/scenarios/<scenario_id>/parameters")
def api_save_parameters(scenario_id: str) -> Response:
    scenario_dir = scenario_dir_for(scenario_id)
    if scenario_dir is None:
        return jsonify({"error": "scenario not found"}), 404
    data = load_yaml_file(scenario_dir / "scenario.yaml")
    payload = request.get_json(silent=True) or {}
    if payload.get("reset"):
        values = parameter_defaults(data)
    else:
        raw_values = payload.get("values")
        if not isinstance(raw_values, dict):
            return jsonify({"error": "values must be an object"}), 400
        ok, result = validate_parameter_values(data, raw_values)
        if not ok:
            return jsonify({"error": result}), 400
        values = result
    save_parameter_values(scenario_id, values)
    get_state(scenario_id).append("ok", "场景参数已保存，下次启动生效")
    return jsonify({"ok": True, "values": values})


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
    write_log = bool(payload.get("write_log", True))
    data = load_yaml_file(scenario_dir / "scenario.yaml")
    ok, message = send_driver_command(
        scenario_id, data, key, write_log=write_log
    )
    return jsonify({"ok": ok, "message": message}), (200 if ok else 400)


@app.post("/api/scenarios/<scenario_id>/apa-command")
def api_apa_command(scenario_id: str) -> Response:
    scenario_dir = scenario_dir_for(scenario_id)
    if scenario_dir is None:
        return jsonify({"error": "scenario not found"}), 404
    payload = request.get_json(silent=True) or {}
    if not isinstance(payload, dict):
        return jsonify({"error": "无效的 APA 指令"}), 400
    data = load_yaml_file(scenario_dir / "scenario.yaml")
    if data.get("control_profile") != "apa":
        return jsonify({"error": "当前场景不是泊车场景"}), 400
    result, code = send_apa_command(scenario_id, data, payload)
    return jsonify(result), code


@app.post("/api/scenarios/<scenario_id>/vehicle-cmd")
def api_vehicle_cmd(scenario_id: str) -> Response:
    scenario_dir = scenario_dir_for(scenario_id)
    if scenario_dir is None:
        return jsonify({"error": "scenario not found"}), 404
    payload = request.get_json(silent=True) or {}
    if not isinstance(payload, dict):
        return jsonify({"error": "invalid vehicle command"}), 400
    data = load_yaml_file(scenario_dir / "scenario.yaml")
    try:
        ok, message = send_vehicle_command(data, payload)
    except (TypeError, ValueError) as exc:
        return jsonify({"ok": False, "message": str(exc)}), 400
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


@app.post("/api/scenarios/<scenario_id>/lks-driver-state")
def api_lks_driver_state(scenario_id: str) -> Response:
    scenario_dir = scenario_dir_for(scenario_id)
    if scenario_dir is None:
        return jsonify({"error": "scenario not found"}), 404
    payload = request.get_json(silent=True) or {}
    try:
        brake_pressed = bool(payload.get("brake_pressed", False))
        driver_steer_norm = float(payload.get("driver_steer_norm", 0.0))
    except (TypeError, ValueError):
        return jsonify({"error": "invalid LKS driver state"}), 400
    data = load_yaml_file(scenario_dir / "scenario.yaml")
    ok, message = send_lks_driver_state(
        scenario_id, data, brake_pressed, driver_steer_norm
    )
    return jsonify({"ok": ok, "message": message}), (200 if ok else 400)


@app.get("/api/scenarios/<scenario_id>/results/latest")
def api_latest_result(scenario_id: str) -> Response:
    scenario_dir = scenario_dir_for(scenario_id)
    if scenario_dir is None:
        return jsonify({"error": "scenario not found"}), 404
    data = load_yaml_file(scenario_dir / "scenario.yaml")
    ok, result = read_scenario_result(scenario_dir, data)
    return jsonify({"ok": ok, "result": result} if ok else {"error": result}), (200 if ok else 404)


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


@app.get("/api/scenarios/<scenario_id>/parking-slots")
def api_parking_slots(scenario_id: str) -> Response:
    scenario_dir = scenario_dir_for(scenario_id)
    if scenario_dir is None:
        return jsonify({"error": "scenario not found"}), 404
    data = load_yaml_file(scenario_dir / "scenario.yaml")
    if data.get("control_profile") != "apa":
        return jsonify({"error": "parking display is only available for APA"}), 404
    monitor = get_telemetry_monitor(bridge_publish_endpoint(data))
    status, received = telemetry_entry(monitor.snapshot(), APA_STATUS_TOPIC)
    if (not status.get("session_id") or not 0 <= time.time() - received <= TELEMETRY_STALE_SEC
            or status.get("map_transition", {}).get("state") == "loading"):
        return jsonify({"error": "等待当前地图车位数据"}), 503
    if request.args.get("session_id") != status["session_id"]:
        return jsonify({"error": "地图会话已更新"}), 409
    try:
        catalog = build_parking_display(status, scenario_dir)
    except (OSError, ValueError, KeyError, TypeError):
        return jsonify({"error": "当前地图车位标线暂不可用"}), 503
    return jsonify(catalog), 200, {"Cache-Control": "no-store"}


@app.get("/api/scenarios/<scenario_id>/telemetry")
def api_telemetry(scenario_id: str) -> Response:
    scenario_dir = scenario_dir_for(scenario_id)
    if scenario_dir is None:
        return jsonify({"error": "scenario not found"}), 404
    data = load_yaml_file(scenario_dir / "scenario.yaml")
    profile = str(data.get("control_profile", "acc"))
    monitor = get_telemetry_monitor(bridge_publish_endpoint(data))
    if profile == "apa":
        snapshot = build_apa_telemetry_snapshot(monitor.snapshot())
        reconcile_apa_pending(scenario_id, data, snapshot)
        snapshot["camera_streams"] = camera_stream_status(camera_http_base(data), snapshot["session_id"], snapshot["camera_revision"])
        return jsonify(snapshot)
    if profile != "acc":
        return jsonify({"error": "telemetry dashboard currently supports ACC and APA scenarios"}), 400
    diagnostic = get_acc_diagnostic_state(scenario_id, data).snapshot()
    return jsonify(build_acc_telemetry_snapshot(monitor.snapshot(), diagnostic))


@app.get("/api/scenarios/<scenario_id>/rear-camera", defaults={"view": "rear"})
@app.get("/api/scenarios/<scenario_id>/overhead-camera", defaults={"view": "overhead"})
def api_camera(scenario_id: str, view: str) -> Response:
    scenario_dir = scenario_dir_for(scenario_id)
    if scenario_dir is None:
        return jsonify({"error": "scenario not found"}), 404
    data = load_yaml_file(scenario_dir / "scenario.yaml")
    if str(data.get("control_profile", "acc")) != "apa":
        return jsonify({"error": "cameras are only available for APA scenarios"}), 400
    camera_base = camera_http_base(data)
    monitor = get_telemetry_monitor(bridge_publish_endpoint(data))

    if request.args.get("snapshot") == "1":
        status, _ = telemetry_entry(monitor.snapshot(), APA_STATUS_TOPIC)
        session_id = status.get("session_id", "")
        revision = status.get("camera_revision", 0)
        requested_session = request.args.get("session_id", "")
        requested_revision = request.args.get("revision", "0")
        if not session_id or requested_session != session_id or requested_revision != str(revision):
            return Response(status=409, headers={"Cache-Control": "no-store"})
        payload, mime = fetch_camera_frame(camera_base, view, session_id, revision)
        current, _ = telemetry_entry(monitor.snapshot(), APA_STATUS_TOPIC)
        if current.get("session_id", "") != session_id or current.get("camera_revision", 0) != revision:
            return Response(status=409, headers={"Cache-Control": "no-store"})
        frame_state = camera_stream_status(camera_base, session_id, revision)[view]
        if mime != "image/jpeg":
            return Response(status=503, headers={"Cache-Control": "no-store"})
        return Response(payload, mimetype=mime, headers={
            "Cache-Control": "no-store", "X-Camera-State": frame_state,
            "X-Camera-Session": session_id, "X-Camera-Revision": str(revision),
        })

    def generate():
        while True:
            status, _ = telemetry_entry(monitor.snapshot(), APA_STATUS_TOPIC)
            session_id = status.get("session_id", "")
            revision = status.get("camera_revision", 0)
            payload, mime = fetch_camera_frame(camera_base, view, session_id, revision)
            current, _ = telemetry_entry(monitor.snapshot(), APA_STATUS_TOPIC)
            if current.get("session_id", "") != session_id or current.get("camera_revision", 0) != revision:
                continue
            yield (
                b"--frame\r\nContent-Type: "
                + mime.encode("ascii")
                + b"\r\nContent-Length: "
                + str(len(payload)).encode("ascii")
                + b"\r\n\r\n"
                + payload
                + b"\r\n"
            )
            time.sleep(0.12)

    return Response(
        generate(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
        headers={"Cache-Control": "no-store, no-cache, must-revalidate"},
    )


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
