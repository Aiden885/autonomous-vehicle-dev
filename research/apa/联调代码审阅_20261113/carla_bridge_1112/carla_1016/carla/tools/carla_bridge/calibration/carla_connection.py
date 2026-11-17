"""Shared APA connection settings and read-only CARLA version RPC diagnostics."""

import json
import math
import time
from pathlib import Path

DEFAULT_CONFIG_PATH = Path(__file__).resolve().parents[3] / "scenarios/apa/bridge_config.json"


def connection_options(config_path=None, host=None, port=None, timeout_sec=None):
    path = Path(config_path or DEFAULT_CONFIG_PATH).resolve()
    config = json.loads(path.read_text())["carla"]
    options = {"config_path": str(path), "host": config.get("host", "127.0.0.1") if host is None else host,
               "port": config.get("port", 2000) if port is None else port,
               "timeout_sec": config.get("timeout_sec", 30.0) if timeout_sec is None else timeout_sec}
    if (not isinstance(options["host"], str) or not options["host"].strip() or
            type(options["port"]) is not int or not 1 <= options["port"] <= 65535 or
            type(options["timeout_sec"]) not in (int, float) or
            not math.isfinite(options["timeout_sec"]) or not 0 < options["timeout_sec"] <= 60):
        raise ValueError("CARLA host/port/timeout must be a nonempty host, port 1..65535 and timeout (0,60] s")
    return options


def network_context():
    try:

        routes = Path("/proc/net/route").read_text().splitlines()[1:]
        return {"ipv4_route_count": len(routes),
                "ipv4_default_route": any(row.split()[1] == "00000000" for row in routes)}
    except (OSError, IndexError):
        return {"ipv4_route_count": None, "ipv4_default_route": None}


class CarlaRPCError(RuntimeError):
    def __init__(self, report):
        self.report = report
        super().__init__(f"CARLA RPC {report['host']}:{report['port']} failed after "
                         f"{report['elapsed_sec']:.3f}s: {report['error']}. " + " ".join(report["hints"]))


def connect_checked(carla_api, options):
    """Use only CARLA RPC; never tick, load a world, or send a bare TCP probe."""
    report = dict(options, network=network_context())
    start = time.monotonic()
    try:
        client = carla_api.Client(options["host"], options["port"])
        client.set_timeout(options["timeout_sec"])
        report["client_version"] = client.get_client_version()
        report["server_version"] = client.get_server_version()
    except (RuntimeError, OSError) as exc:
        report.update(ok=False, elapsed_sec=round(time.monotonic() - start, 6), error=str(exc),
                      hints=["核对 CARLA 进程、RPC 端口、配置地址及服务是否仍在启动。",
                             "在可访问 CARLA 服务的终端重试连接；隔离环境中的超时不能证明服务故障。"])
        if report["network"]["ipv4_route_count"] == 0:
            report["hints"].insert(0, "当前进程的 IPv4 路由表为空，需先核对网络隔离。")
        raise CarlaRPCError(report) from exc
    report.update(ok=True, elapsed_sec=round(time.monotonic() - start, 6),
                  version_match=report["client_version"] == report["server_version"])
    return client, report
