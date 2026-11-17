"""Convert equivalent bicycle steering to CARLA's inner-wheel command."""
import math


def validate_steering_geometry(profile):
    model = profile.get('steering_model', 'bicycle')
    if model == 'bicycle':
        return
    if model != 'ackermann_inner_wheel':
        raise ValueError('unsupported vehicle steering_model')
    for key in ('front_track_m', 'carla_max_steer_rad'):
        value = profile.get(key)
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'vehicle profile {key} must be finite and positive')
    inner = profile['carla_max_steer_rad']
    if inner >= math.pi/2:
        raise ValueError('CARLA inner wheel lock must be below pi/2')
    radius = profile['wheelbase_m']/math.tan(inner) + profile['front_track_m']/2
    equivalent = math.atan(profile['wheelbase_m']/radius)
    if abs(equivalent-profile['max_steer_rad']) > 1.e-8:
        raise ValueError('equivalent steering lock does not match Ackermann geometry')
    if profile['planning_min_turn_radius_m'] <= radius:
        raise ValueError('planning radius must leave headroom beyond the physical wheel lock')


def equivalent_to_carla(steer_rad, limit, profile):
    steer = max(-limit, min(limit, steer_rad))
    if profile.get('steering_model', 'bicycle') != 'ackermann_inner_wheel':
        return steer/limit
    if abs(steer) < 1.e-12:
        return 0.0
    length = profile['wheelbase_m']
    inner = math.atan2(length, length/abs(math.tan(steer))-profile['front_track_m']/2)
    return math.copysign(min(1.0, inner/profile['carla_max_steer_rad']), steer)


def carla_to_equivalent(command, limit, profile):
    command = max(-1.0, min(1.0, command))
    if profile.get('steering_model', 'bicycle') != 'ackermann_inner_wheel':
        return command*limit
    inner = abs(command)*profile['carla_max_steer_rad']
    if inner < 1.e-12:
        return 0.0
    length = profile['wheelbase_m']
    radius = length/math.tan(inner)+profile['front_track_m']/2
    return math.copysign(math.atan(length/radius), command)
