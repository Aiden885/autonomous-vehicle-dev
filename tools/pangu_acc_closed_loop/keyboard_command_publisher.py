#!/usr/bin/env python3
"""Keyboard driver-command publisher for ACC tests.

It sends gaasd.carla.driver_command.v1 to the bridge command endpoint 5702.
Pulse commands are emitted on key down. Level commands are emitted on key
down/up and are repeated by the real bridge or mock bridge.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass

import zmq


DRIVER_TOPIC = "gaasd.carla.driver_command.v1"
PULSE_KEYS = {
    "e": (1, "降低设定速度/当前速度启控"),
    "q": (2, "提高设定速度/继承参数启控"),
    "t": (3, "减小时距"),
    "r": (4, "增大时距"),
    "c": (7, "取消ACC"),
}
LEVEL_KEYS = {
    "w": (5, "驾驶员油门"),
    "s": (6, "驾驶员制动并退出ACC"),
}


@dataclass
class Publisher:
    socket: zmq.Socket
    event_id: int = 0

    def send(self, command_type: int, mode: str, active: bool, description: str) -> None:
        self.event_id += 1
        message = {
            "protocol": "gaasd_carla_bridge",
            "protocol_version": "0.3.0",
            "message_type": DRIVER_TOPIC,
            "sequence": self.event_id,
            "timestamp_unix_ms": int(time.time() * 1000),
            "source": "pangu_keyboard_command_publisher",
            "payload": {
                "command_type": int(command_type),
                "mode": mode,
                "active": bool(active),
                "event_id": self.event_id,
                "description": description,
            },
        }
        text = json.dumps(message, ensure_ascii=False, separators=(",", ":"))
        self.socket.send_multipart([DRIVER_TOPIC.encode("utf-8"), text.encode("utf-8")])
        print(f"commandType={command_type} mode={mode} active={int(active)} {description}")


def interactive_text(pub: Publisher) -> None:
    print("输入 e/q/t/r/c 发送脉冲指令；输入 w 或 s 按下 level；输入 0 释放 level；输入 exit 退出。")
    while True:
        text = input("> ").strip().lower()
        if text in {"exit", "quit"}:
            return
        if text == "0":
            pub.send(0, "level", False, "踏板释放")
        elif text in PULSE_KEYS:
            command_type, description = PULSE_KEYS[text]
            pub.send(command_type, "pulse", True, description)
        elif text in LEVEL_KEYS:
            command_type, description = LEVEL_KEYS[text]
            pub.send(command_type, "level", True, description)
        else:
            print("未知指令，可用：e q t r c w s 0 exit")


def send_once(pub: Publisher, key: str) -> None:
    normalized = key.strip().lower()
    if normalized == "0":
        pub.send(0, "level", False, "踏板释放")
    elif normalized in PULSE_KEYS:
        command_type, description = PULSE_KEYS[normalized]
        pub.send(command_type, "pulse", True, description)
    elif normalized in LEVEL_KEYS:
        command_type, description = LEVEL_KEYS[normalized]
        pub.send(command_type, "level", True, description)
    else:
        raise ValueError("未知指令，可用：e q t r c w s 0")


def pygame_loop(pub: Publisher) -> None:
    import pygame

    pygame.init()
    screen = pygame.display.set_mode((560, 220))
    pygame.display.set_caption("GAASD ACC Driver Command")
    font = pygame.font.Font(None, 28)
    active_levels: set[str] = set()
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                key = pygame.key.name(event.key).lower()
                if key in PULSE_KEYS:
                    command_type, description = PULSE_KEYS[key]
                    pub.send(command_type, "pulse", True, description)
                elif key in LEVEL_KEYS and key not in active_levels:
                    active_levels.add(key)
                    command_type, description = LEVEL_KEYS[key]
                    pub.send(command_type, "level", True, description)
            elif event.type == pygame.KEYUP:
                key = pygame.key.name(event.key).lower()
                if key in active_levels:
                    active_levels.remove(key)
                    pub.send(0, "level", False, "踏板释放")

        screen.fill((245, 247, 250))
        lines = [
            "E/Q: speed -/+    T/R: gap -/+    C: cancel",
            "Hold W: driver throttle    Hold S: driver brake",
            "Close this window to exit.",
        ]
        for idx, line in enumerate(lines):
            surface = font.render(line, True, (22, 27, 34))
            screen.blit(surface, (28, 36 + idx * 42))
        pygame.display.flip()
        pygame.time.wait(20)

    pygame.quit()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--endpoint", default="tcp://127.0.0.1:5702")
    parser.add_argument("--text", action="store_true", help="使用终端输入模式")
    parser.add_argument("--once", help="发送一次指令后退出，可用：e/q/t/r/c/w/s/0")
    args = parser.parse_args()

    ctx = zmq.Context.instance()
    socket = ctx.socket(zmq.PUB)
    socket.connect(args.endpoint)
    time.sleep(0.2)
    pub = Publisher(socket)

    if args.once:
        send_once(pub, args.once)
        time.sleep(0.05)
        return
    if args.text:
        interactive_text(pub)
        return
    try:
        pygame_loop(pub)
    except Exception as exc:
        print(f"pygame 模式不可用，切换终端输入模式：{exc}", file=sys.stderr)
        interactive_text(pub)


if __name__ == "__main__":
    main()
