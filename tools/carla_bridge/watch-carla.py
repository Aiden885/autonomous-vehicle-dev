#!/usr/bin/env python3
"""CARLA 第三人称观察窗口：挂载摄像头到自车，实时显示跟车效果与 HUD。"""

from __future__ import annotations

import argparse
import glob
import json
import os
import signal
import sys
import time
import weakref

import numpy as np

try:
    import zmq
except ImportError as exc:
    raise SystemExit("pyzmq is required: python3.8 -m pip install pyzmq") from exc


DRIVER_COMMAND_TOPIC = "gaasd.carla.driver_command.v1"
PULSE_KEY_COMMANDS = {
    "e": (1, "降低设定速度/当前速度启控"),
    "q": (2, "提高设定速度/继承参数启控"),
    "t": (3, "减小时距"),
    "r": (4, "增大时距"),
    "c": (7, "取消 ACC"),
}
LEVEL_KEY_COMMANDS = {
    "w": (5, "驾驶员油门"),
    "s": (6, "驾驶员制动并退出 ACC"),
}


def add_carla_python_paths(root: str) -> None:
    paths = [os.path.join(root, "PythonAPI", "carla")]
    paths.extend(glob.glob(os.path.join(root, "PythonAPI", "carla", "dist", "carla-*-py3*.egg")))
    for path in reversed(paths):
        if path and os.path.exists(path) and path not in sys.path:
            sys.path.insert(0, path)


def get_actor_by_role(world, role_name: str):
    for actor in world.get_actors().filter("vehicle.*"):
        if actor.attributes.get("role_name") == role_name:
            return actor
    return None


def actor_speed_mps(actor) -> float:
    v = actor.get_velocity()
    return (v.x ** 2 + v.y ** 2 + v.z ** 2) ** 0.5


def actor_distance(a, b) -> float:
    la = a.get_transform().location
    lb = b.get_transform().location
    return ((la.x - lb.x) ** 2 + (la.y - lb.y) ** 2) ** 0.5


def main() -> int:
    parser = argparse.ArgumentParser(description="CARLA 跟车观察窗口")
    parser.add_argument("--carla-root", default="/home/aiden/snap/code/app/carla-package")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--timeout-sec", type=float, default=10.0)
    parser.add_argument("--ego-role-name", default="hero")
    parser.add_argument("--lead-role-name", default="gaasd_lead")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fov", type=int, default=90)
    parser.add_argument("--command-endpoint", default="tcp://127.0.0.1:5702")
    args = parser.parse_args()

    add_carla_python_paths(args.carla_root)
    import carla  # type: ignore
    import pygame  # type: ignore

    # ---------- 连接 CARLA ----------
    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout_sec)
    world = client.get_world()
    print(f"[Watch] 已连接 CARLA，地图: {world.get_map().name}")

    command_context = zmq.Context.instance()
    command_pub = command_context.socket(zmq.PUB)
    command_pub.setsockopt(zmq.LINGER, 0)
    command_pub.connect(args.command_endpoint)
    command_sequence = [0]
    level_keys_down = set()
    last_command = [0, "无指令"]
    last_level_heartbeat = [0.0]
    stop_requested = [False]

    def request_stop(_signum, _frame) -> None:
        stop_requested[0] = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    time.sleep(0.2)
    print(f"[Watch] 驾驶指令连接: {args.command_endpoint}")

    # 等待自车出现
    ego = None
    print(f"[Watch] 等待自车 role_name={args.ego_role_name} ...")
    deadline = time.monotonic() + 15.0
    while ego is None and time.monotonic() < deadline and not stop_requested[0]:
        ego = get_actor_by_role(world, args.ego_role_name)
        if ego is None:
            time.sleep(0.3)
    if ego is None:
        print("[Watch] 未找到自车，退出")
        command_pub.close(0)
        return 1
    print(f"[Watch] 找到自车 id={ego.id}")

    # ---------- 摄像头蓝图 ----------
    bp_lib = world.get_blueprint_library()
    cam_bp = bp_lib.find("sensor.camera.rgb")
    cam_bp.set_attribute("image_size_x", str(args.width))
    cam_bp.set_attribute("image_size_y", str(args.height))
    cam_bp.set_attribute("fov", str(args.fov))

    # 三个视角：后方跟随、驾驶员视角、高空俯视
    camera_transforms = [
        carla.Transform(carla.Location(x=-8.0, z=6.0),  carla.Rotation(pitch=-25.0)),  # 第三人称
        carla.Transform(carla.Location(x=1.6,  z=1.7)),                                  # 驾驶员视角
        carla.Transform(carla.Location(x=-12.0, z=10.0), carla.Rotation(pitch=-35.0)),  # 高空
    ]
    cam_index = [0]

    # 挂载摄像头
    camera = world.spawn_actor(cam_bp, camera_transforms[0], attach_to=ego)
    print(f"[Watch] 摄像头已挂载（第三人称视角），按 V 切换视角")

    # ---------- Pygame ----------
    pygame.init()
    pygame.font.init()
    screen = pygame.display.set_mode((args.width, args.height), pygame.HWSURFACE | pygame.DOUBLEBUF)
    pygame.display.set_caption("CARLA ACC 联调 — E/Q调速 R/T调距 W油门 S制动退出 C取消")
    clock = pygame.time.Clock()

    try:
        font_large = pygame.font.SysFont("monospace", 26, bold=True)
        font_small = pygame.font.SysFont("monospace", 20)
    except Exception:
        font_large = pygame.font.Font(None, 30)
        font_small = pygame.font.Font(None, 22)

    # 共享最新帧（callback 写，主循环读）
    latest_frame = [None]

    def on_image(image):
        arr = np.frombuffer(image.raw_data, dtype=np.uint8)
        arr = np.reshape(arr, (image.height, image.width, 4))[:, :, :3][:, :, ::-1]
        latest_frame[0] = pygame.surfarray.make_surface(arr.swapaxes(0, 1))

    weak_cam = weakref.ref(camera)
    camera.listen(lambda img: on_image(img))

    def switch_view():
        cam_index[0] = (cam_index[0] + 1) % len(camera_transforms)
        c = weak_cam()
        if c:
            c.set_transform(camera_transforms[cam_index[0]])
        view_names = ["第三人称", "驾驶员", "高空俯视"]
        print(f"[Watch] 切换视角 → {view_names[cam_index[0]]}")

    def send_driver_command(command_type: int, mode: str, active: bool, description: str) -> None:
        command_sequence[0] += 1
        message = {
            "protocol": "gaasd_carla_bridge",
            "protocol_version": "0.3.0",
            "message_type": DRIVER_COMMAND_TOPIC,
            "sequence": command_sequence[0],
            "timestamp_unix_ms": int(time.time() * 1000),
            "source": "watch_carla_keyboard",
            "payload": {
                "command_type": int(command_type),
                "mode": mode,
                "active": bool(active),
                "event_id": command_sequence[0],
                "description": description,
            },
        }
        text = json.dumps(message, ensure_ascii=False, separators=(",", ":"))
        command_pub.send_multipart(
            [DRIVER_COMMAND_TOPIC.encode("utf-8"), text.encode("utf-8")]
        )
        last_command[0] = int(command_type)
        last_command[1] = description
        print(
            f"[Watch] commandType={command_type} mode={mode} "
            f"active={int(active)} {description}"
        )

    def active_level_command():
        if "s" in level_keys_down:
            return LEVEL_KEY_COMMANDS["s"]
        if "w" in level_keys_down:
            return LEVEL_KEY_COMMANDS["w"]
        return 0, "踏板释放"

    def update_level_command() -> None:
        command_type, description = active_level_command()
        send_driver_command(
            command_type,
            mode="level",
            active=command_type != 0,
            description=description,
        )
        last_level_heartbeat[0] = time.monotonic()

    def render_hud(surface):
        """在画面左上角叠加 HUD 信息。"""
        lead = get_actor_by_role(world, args.lead_role_name)

        ego_v   = actor_speed_mps(ego) * 3.6   # km/h
        lead_v  = actor_speed_mps(lead) * 3.6 if lead else None
        dist    = actor_distance(ego, lead)     if lead else None
        rel_spd = (lead_v - ego_v)              if lead_v is not None else None

        lines = [
            ("自车速度",   f"{ego_v:6.1f} km/h"),
            ("前车速度",   f"{lead_v:6.1f} km/h" if lead_v is not None else "  未检测"),
            ("跟车距离",   f"{dist:6.1f} m"      if dist   is not None else "  未检测"),
            ("相对速度",   f"{rel_spd:+6.1f} km/h" if rel_spd is not None else "  N/A"),
            ("驾驶指令",   f"{last_command[0]} {last_command[1]}"),
        ]

        # 半透明背景
        panel_w, panel_h = 430, len(lines) * 30 + 20
        panel = pygame.Surface((panel_w, panel_h), pygame.SRCALPHA)
        panel.fill((0, 0, 0, 150))
        surface.blit(panel, (10, 10))

        # 标题
        title = font_large.render("ACC 跟车监控", True, (100, 200, 255))
        surface.blit(title, (20, 14))

        for i, (label, value) in enumerate(lines):
            y = 44 + i * 28
            label_surf = font_small.render(f"{label}:", True, (200, 200, 200))
            value_surf = font_small.render(value,      True, (255, 255, 100))
            surface.blit(label_surf, (20,  y))
            surface.blit(value_surf, (150, y))

        # 视角提示
        view_names = ["第三人称", "驾驶员", "高空俯视"]
        hint = font_small.render(
            f"视角: {view_names[cam_index[0]]}  "
            "[E/Q调速 R/T调距 W油门 S制动退出 C取消 V切换]",
            True,
            (160, 160, 160),
        )
        surface.blit(hint, (10, args.height - 28))

    # ---------- 主循环 ----------
    print("[Watch] 窗口已启动")
    print("[Watch] E/Q 调速或启控，R/T 调整时距，W 油门，S 制动并退出 ACC，C 取消")
    running = True
    while running and not stop_requested[0]:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_v:
                    switch_view()
                else:
                    key_name = pygame.key.name(event.key).lower()
                    pulse_command = PULSE_KEY_COMMANDS.get(key_name)
                    if pulse_command is not None:
                        send_driver_command(
                            pulse_command[0],
                            mode="pulse",
                            active=True,
                            description=pulse_command[1],
                        )
                    elif key_name in LEVEL_KEY_COMMANDS and key_name not in level_keys_down:
                        level_keys_down.add(key_name)
                        update_level_command()
            elif event.type == pygame.KEYUP:
                key_name = pygame.key.name(event.key).lower()
                if key_name in LEVEL_KEY_COMMANDS and key_name in level_keys_down:
                    level_keys_down.remove(key_name)
                    update_level_command()

        if level_keys_down and time.monotonic() - last_level_heartbeat[0] >= 0.1:
            update_level_command()

        # 渲染最新帧
        frame = latest_frame[0]
        if frame is not None:
            screen.blit(frame, (0, 0))
        else:
            screen.fill((30, 30, 30))

        render_hud(screen)
        pygame.display.flip()
        clock.tick(30)

    if level_keys_down:
        level_keys_down.clear()
        update_level_command()
    camera.stop()
    camera.destroy()
    command_pub.close(0)
    pygame.quit()
    print("[Watch] 退出")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
