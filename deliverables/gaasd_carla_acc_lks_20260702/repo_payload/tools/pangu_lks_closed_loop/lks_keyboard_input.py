#!/usr/bin/env python3.8
"""Publish LKS driver steering/brake inputs from a small Pygame window."""

from __future__ import annotations

import argparse
import json
import time

import pygame
import zmq


TOPIC = "gaasd.carla.driver_state_cmd.v1"


def message(brake_pressed: bool, driver_steer_norm: float) -> list[bytes]:
    payload = {
        "header": {
            "protocol": "gaasd_carla_bridge",
            "protocol_version": "0.3.0",
            "message_type": TOPIC,
            "timestamp_unix_ms": int(time.time() * 1000),
            "source": "lks_keyboard_input",
        },
        "payload": {
            "brake_pressed": brake_pressed,
            "driver_steer_norm": driver_steer_norm,
        },
    }
    return [TOPIC.encode("utf-8"), json.dumps(payload, separators=(",", ":")).encode("utf-8")]


def draw(screen: pygame.Surface, font: pygame.font.Font, brake: bool, steer: float) -> None:
    screen.fill((242, 246, 252))
    rows = [
        "LKS Driver Input",
        "Hold A / D: steering takeover",
        "Hold B: brake and exit LKS",
        "Release keys: zero input, LKS resumes",
        "ESC: close",
        f"brakePressed={int(brake)}  driverSteerNorm={steer:+.2f}",
    ]
    colors = [(25, 35, 52), (70, 83, 105), (70, 83, 105), (70, 83, 105), (70, 83, 105), (28, 96, 210)]
    for index, (row, color) in enumerate(zip(rows, colors)):
        screen.blit(font.render(row, True, color), (28, 24 + index * 38))
    pygame.display.flip()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--endpoint", default="tcp://127.0.0.1:5702")
    parser.add_argument("--steer-norm", type=float, default=0.3)
    parser.add_argument("--frequency-hz", type=float, default=20.0)
    args = parser.parse_args()

    steer_magnitude = max(0.0, min(1.0, abs(args.steer_norm)))
    frequency_hz = max(5.0, args.frequency_hz)
    context = zmq.Context.instance()
    publisher = context.socket(zmq.PUB)
    publisher.setsockopt(zmq.LINGER, 0)
    publisher.connect(args.endpoint)

    pygame.init()
    screen = pygame.display.set_mode((620, 280))
    pygame.display.set_caption("GAASD-CARLA LKS Driver Input")
    font = pygame.font.SysFont("DejaVu Sans", 22)
    clock = pygame.time.Clock()
    running = True
    time.sleep(0.3)
    try:
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    running = False

            keys = pygame.key.get_pressed()
            brake_pressed = bool(keys[pygame.K_b])
            left_pressed = bool(keys[pygame.K_a])
            right_pressed = bool(keys[pygame.K_d])
            if left_pressed == right_pressed:
                driver_steer_norm = 0.0
            else:
                driver_steer_norm = -steer_magnitude if left_pressed else steer_magnitude

            publisher.send_multipart(message(brake_pressed, driver_steer_norm))
            draw(screen, font, brake_pressed, driver_steer_norm)
            clock.tick(frequency_hz)
    finally:
        # Repeat the release state so Bridge cannot retain a stale takeover input.
        for _ in range(4):
            publisher.send_multipart(message(False, 0.0))
            time.sleep(0.05)
        publisher.close(0)
        pygame.quit()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
