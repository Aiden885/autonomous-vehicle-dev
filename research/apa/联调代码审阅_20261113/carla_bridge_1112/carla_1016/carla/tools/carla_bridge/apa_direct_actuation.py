"""Validate final C actuator commands."""
import math

ACTUATOR_OWNER = "C:apaMocoActuate:v2"


def validate_actuator(payload):
    command = payload.get("actuator")
    if not isinstance(command, dict) or command.get("owner") != ACTUATOR_OWNER:
        raise ValueError("missing C-owned actuator command; install matching APA nodes")
    for name in ("throttle", "brake"):
        value = command.get(name)
        if type(value) not in (int, float) or not math.isfinite(value) or not 0 <= value <= 1:
            raise ValueError(f"invalid actuator.{name}")
    if type(command.get("hand_brake")) is not bool:
        raise ValueError("invalid actuator.hand_brake")
    if type(command.get("gear_pos")) is not int or command["gear_pos"] not in range(4):
        raise ValueError("invalid actuator.gear_pos")
    if type(command.get("launch_hold")) is not bool:
        raise ValueError("invalid actuator.launch_hold")
    if command["throttle"] > 0 and (command["hand_brake"] or
            (command["brake"] > 0 and not command["launch_hold"])):
        raise ValueError("contradictory actuator pedals")
    if (payload.get("StopReq") or not payload.get("enable", payload.get("ControlEnabled", False))):
        if command["throttle"] != 0 or command["brake"] < 1:
            raise ValueError("stop requires the C brake-hold command")
    if command["gear_pos"] in (0, 3) and command["throttle"] != 0:
        raise ValueError("traction in neutral/park")
    return command
