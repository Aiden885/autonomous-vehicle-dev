"""Read-only steering telemetry. No values from here feed vehicle control."""

import math
import time
from collections import deque


def empty_steering_observation(reason="not_sampled"):
    return {
        "frame_id": None, "sim_time_sec": None,
        "physical_wheels": {"valid": False, "reason": reason,
                            "front_left_deg": None, "front_right_deg": None},
        "rear_axle_motion": {"valid": False, "reason": reason,
                             "curvature_inv_m": None},
    }


class SteeringObservation:

    WHEEL_PERIOD_SEC = 0.1
    WINDOW_SEC = 0.5
    MIN_TRAVEL_M = 0.02
    MIN_SPEED_MPS = 0.05

    def __init__(self):
        self.identity = None
        self.previous = None
        self.window = deque()
        self.wheel_time = None
        self.payload = empty_steering_observation()

    def sample(self, identity, frame, sim_time, x, y, yaw, speed, read_wheels):
        if identity != self.identity:
            self.__init__()
            self.identity = identity
        previous = self.previous
        if previous and frame == previous[0] and sim_time == previous[1]:
            return self.payload
        reset_time = previous and (frame <= previous[0] or sim_time <= previous[1])
        if reset_time:
            self.window.clear()
            self.wheel_time = None
        wheels = self.payload["physical_wheels"]
        if self.wheel_time is None or sim_time-self.wheel_time >= self.WHEEL_PERIOD_SEC-1.e-6:
            started = time.monotonic()
            wheels = {"valid": False, "reason": "unavailable",
                      "front_left_deg": None, "front_right_deg": None,
                      "source": "carla.Vehicle.get_wheel_steer_angle",

                      "queried_after_frame_id": frame, "sim_time_sec": sim_time}
            try:
                left, right = (float(value) for value in read_wheels())
                if not (math.isfinite(left) and math.isfinite(right)):
                    raise ValueError("nonfinite physical wheel angle")
                wheels.update(valid=True, reason="", front_left_deg=left, front_right_deg=right)
            except Exception as error:

                wheels["reason"] = type(error).__name__ + ": " + str(error)
            wheels["query_duration_ms"] = (time.monotonic()-started)*1000.0
            self.wheel_time = sim_time

        motion = {"valid": False, "reason": "insufficient_travel",
                  "curvature_inv_m": None, "source": "rear_axle_pose_delta",
                  "min_travel_m": self.MIN_TRAVEL_M,
                  "sign_convention": "wrapped_yaw_delta / signed_longitudinal_travel"}
        current = (frame, sim_time, x, y, yaw, speed)
        reason = ""
        if not previous:
            reason = "first_sample"
        elif reset_time:
            reason = "time_reset"
        elif sim_time-previous[1] > self.WINDOW_SEC:
            reason = "sample_gap"
        elif abs(speed) < self.MIN_SPEED_MPS:
            reason = "low_speed"
        elif speed*previous[5] <= 0 or abs(previous[5]) < self.MIN_SPEED_MPS:
            reason = "direction_change_or_start"
        else:
            dx, dy = x-previous[2], y-previous[3]
            dyaw = math.atan2(math.sin(yaw-previous[4]), math.cos(yaw-previous[4]))
            mid_yaw = previous[4]+dyaw/2.0
            ds = dx*math.cos(mid_yaw)+dy*math.sin(mid_yaw)
            lateral = -dx*math.sin(mid_yaw)+dy*math.cos(mid_yaw)
            if math.hypot(dx, dy) > max(0.25, 4*max(abs(speed), abs(previous[5]))*(sim_time-previous[1])):
                reason = "pose_discontinuity"
            elif abs(dyaw) > 0.25:
                reason = "yaw_discontinuity"
            elif ds*speed < 0:
                reason = "motion_direction_mismatch"
            else:
                self.window.append((previous[0], previous[1], sim_time, ds, dyaw, lateral))
                while self.window and sim_time-self.window[0][1] > self.WINDOW_SEC+1.e-6:
                    self.window.popleft()
                travel = sum(item[3] for item in self.window)
                heading = sum(item[4] for item in self.window)
                motion.update(signed_travel_m=travel, yaw_delta_rad=heading,
                              lateral_travel_m=sum(item[5] for item in self.window),
                              start_frame_id=self.window[0][0], end_frame_id=frame,
                              window_sec=sim_time-self.window[0][1])
                if abs(travel) >= self.MIN_TRAVEL_M:
                    motion.update(valid=True, reason="", curvature_inv_m=heading/travel)
        if reason:
            self.window.clear()
            motion["reason"] = reason
        self.previous = current
        self.payload = {"frame_id": frame, "sim_time_sec": sim_time,
                        "physical_wheels": wheels, "rear_axle_motion": motion}
        return self.payload
