"""Fit a fixed-heading rear-axle goal to the shared park-in body envelope."""
import math


class GoalBlocked(ValueError):
    def __init__(self, obstacle_id):
        super().__init__('车位内不存在避开实体障碍的泊入目标')
        self.obstacle_id = obstacle_id


def body_corners(vehicle, margin=0.0):
    front = vehicle['rear_to_front_m'] + margin
    rear = vehicle['rear_to_back_m'] + margin
    side = vehicle['width_m']/2 + margin
    fl, fw = vehicle['front_chamfer_lon_m'], vehicle['front_chamfer_lat_m']
    rl, rw = vehicle['rear_chamfer_lon_m'], vehicle['rear_chamfer_lat_m']
    return [(front-fl, side), (front, side-fw), (front, -side+fw), (front-fl, -side),
            (-rear+rl, -side), (-rear, -side+rw), (-rear, side-rw), (-rear+rl, side)]


def parking_footprint(vehicle, pose, margin=0.0):
    """Shared body octagon at a rear-axle pose; margin is an axis-wise expansion, m."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return [{'x': x+c*u-s*v, 'y': y+s*u+c*v} for u, v in body_corners(vehicle, margin)]


def goal_tolerance(slot, vehicle):
    if ('goal_tol_pos' in slot) != ('goal_tol_yaw' in slot):
        raise ValueError('目标位置和航向容差必须同时提供')
    position, yaw_deg = slot.get('goal_tol_pos', 0), slot.get('goal_tol_yaw', 0)
    if not all(type(v) in (int, float) and math.isfinite(v) and v >= 0 for v in (position, yaw_deg)):
        raise ValueError('泊入目标容差必须为有限非负数')

    return (min(vehicle['goal_position_tolerance_m'], position or vehicle['goal_position_tolerance_m']),
            min(vehicle['goal_yaw_tolerance_rad'], math.radians(yaw_deg) or vehicle['goal_yaw_tolerance_rad']))


def _support(normal, corners, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return max(normal[0]*(c*x-s*y) + normal[1]*(s*x+c*y) for x, y in corners)


def _goal_constraints(slot, vehicle, yaw, bottom_reserve=0.0, include_entry=False):
    points = [(slot[f'slot_pt{i}_x'], slot[f'slot_pt{i}_y']) for i in range(1, 5)]
    centre = tuple(sum(p[i] for p in points) / 4 for i in range(2))
    margin = vehicle['parkin_margin_m']
    corners = body_corners(vehicle, margin)
    actual = body_corners(vehicle)
    entry = int(slot.get("slot_entry", 1)) - 1
    constraints = []
    for i, a in enumerate(points):
        if i == entry and not include_entry:
            continue
        b = points[(i + 1) % 4]
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        if length <= 1e-6:
            raise ValueError('车位边界长度为零')
        nx, ny = -(b[1] - a[1]) / length, (b[0] - a[0]) / length
        if nx * (centre[0] - a[0]) + ny * (centre[1] - a[1]) < 0:
            nx, ny = -nx, -ny
        bottom = i == (entry + 2) % 4
        bound = nx * a[0] + ny * a[1] + _support((-nx, -ny), actual if bottom else corners, yaw)
        if bottom:
            parallel = str(slot.get('apa_slot_kind', slot.get('slot_kind', ''))).lower() in ('2', 'parallel', 'ht')
            bound += vehicle['parkin_physical_margin_m' if parallel else 'parkin_back_margin_m'] + bottom_reserve
        constraints.append((nx, ny, bound))
    return constraints


def centered_goal(slot, vehicle, yaw):
    """Fit a body-centred goal to the three closed edges; keep the entrance open."""
    points = [(slot[f'slot_pt{i}_x'], slot[f'slot_pt{i}_y']) for i in range(1, 5)]
    entry = int(slot.get("slot_entry", 1)) - 1
    p1, p2, p3, p4 = [points[(entry + i) % 4] for i in range(4)]
    ex, ey = (p1[0]+p2[0])/2, (p1[1]+p2[1])/2
    dx, dy = (p3[0]+p4[0])/2-ex, (p3[1]+p4[1])/2-ey
    if math.hypot(dx, dy) <= 1.e-6:
        raise ValueError('车位深度为零')
    offset = (vehicle['rear_to_front_m']-vehicle['rear_to_back_m'])/2
    x, y = ex-offset*math.cos(yaw), ey-offset*math.sin(yaw)
    def interval(reserve):
        lower, upper = 0.0, 1.0
        for nx, ny, bound in _goal_constraints(slot, vehicle, yaw, reserve):
            slope = nx*dx+ny*dy
            required = bound+1.e-3-nx*x-ny*y
            if abs(slope) <= 1.e-9:
                if required > 0.0:
                    return 1.0, 0.0
            elif slope > 0.0:
                lower = max(lower, required/slope)
            else:
                upper = min(upper, required/slope)
        return lower, upper

    lower, upper = interval(0.0)
    if lower > upper:
        raise ValueError('车位中轴线不存在满足泊入包络的目标')
    reserve = max(0.25, vehicle['parkin_back_margin_m']+0.05)-vehicle['parkin_back_margin_m']
    preferred_lower, preferred_upper = interval(reserve)

    fraction = (max(preferred_lower, min(preferred_upper, 0.5))
                if preferred_lower <= preferred_upper else (lower+upper)/2)
    return x+fraction*dx, y+fraction*dy, yaw


def _collision_interval(body, polygon, origin, direction, lower, upper):
    """Exact SAT overlap interval for a fixed-heading body translated along one axis."""
    for shape in (body, polygon):
        for a, b in zip(shape, shape[1:]+shape[:1]):
            nx, ny = a[1]-b[1], b[0]-a[0]
            if nx == ny == 0:
                continue
            body_min = min(nx*x+ny*y for x, y in body)
            body_max = max(nx*x+ny*y for x, y in body)
            obstacle_min = min(nx*(x-origin[0])+ny*(y-origin[1]) for x, y in polygon)
            obstacle_max = max(nx*(x-origin[0])+ny*(y-origin[1]) for x, y in polygon)
            slope = nx*direction[0]+ny*direction[1]
            if abs(slope) <= 1.e-12*math.hypot(nx, ny):
                if body_max < obstacle_min or body_min > obstacle_max:
                    return None
            else:
                a, b = (obstacle_min-body_max)/slope, (obstacle_max-body_min)/slope
                lower, upper = max(lower, min(a, b)), min(upper, max(a, b))
                if lower > upper:
                    return None
    return lower, upper


def parallel_centered_goal(slot, vehicle, yaw, obstacles=()):
    """Balance the free longitudinal interval inside the slot and static geometry."""
    center = tuple(sum(slot[f'slot_pt{i}_{axis}'] for i in range(1, 5))/4
                   for axis in ('x', 'y'))
    direction = math.cos(yaw), math.sin(yaw)
    offset = (vehicle['rear_to_front_m']-vehicle['rear_to_back_m'])/2
    pose = fit_goal(slot, vehicle, (center[0]-offset*direction[0], center[1]-offset*direction[1]), yaw)
    lower, upper = -math.inf, math.inf
    for nx, ny, bound in _goal_constraints(slot, vehicle, yaw, include_entry=True):
        slope = nx*direction[0]+ny*direction[1]
        required = bound+1.e-3-nx*pose[0]-ny*pose[1]
        if slope > 1.e-9:
            lower = max(lower, required/slope)
        elif slope < -1.e-9:
            upper = min(upper, required/slope)
        elif required > 1.e-9:
            raise ValueError('平行车位无法完整容纳泊入包络')
    if not math.isfinite(lower) or not math.isfinite(upper) or lower > upper:
        raise ValueError('平行车位纵向目标区间无效')
    free = [(lower, upper)]
    envelopes = {}
    for obstacle in obstacles:
        if math.hypot(obstacle.get('velocity_x', 0.0), obstacle.get('velocity_y', 0.0)) > 1.e-4:
            continue
        polygon = [(p['x'], p['y']) for p in obstacle['footprint']]
        margin = obstacle.get('goal_margin_m', vehicle['parkin_physical_margin_m'])
        if margin not in envelopes:
            c, s = direction
            corners = [(c*u-s*v, s*u+c*v) for u, v in body_corners(vehicle, margin)]
            xs, ys = zip(*corners)
            bounds = (pose[0]+min(c*lower, c*upper)+min(xs),
                      pose[1]+min(s*lower, s*upper)+min(ys),
                      pose[0]+max(c*lower, c*upper)+max(xs),
                      pose[1]+max(s*lower, s*upper)+max(ys))
            envelopes[margin] = corners, bounds
        corners, bounds = envelopes[margin]
        if (max(x for x, _ in polygon) < bounds[0] or min(x for x, _ in polygon) > bounds[2] or
                max(y for _, y in polygon) < bounds[1] or min(y for _, y in polygon) > bounds[3]):
            continue
        blocked = _collision_interval(corners, polygon, pose, direction, lower, upper)
        if blocked is None:
            continue
        start, end = blocked[0]-1.e-3, blocked[1]+1.e-3
        free = [(a, min(b, start)) for a, b in free if a < start] + [
                (max(a, end), b) for a, b in free if b > end]
        if not free:
            raise GoalBlocked(obstacle.get('id', 'explicit_boundary'))
    lower, upper = max(free, key=lambda interval: (interval[1]-interval[0], -abs(sum(interval))))
    shift = (lower+upper)/2
    return pose[0]+shift*direction[0], pose[1]+shift*direction[1], yaw


def fit_goal(slot, vehicle, preferred, yaw, adjust=True):
    """Fit the nearest rear-axle goal using side-envelope and bottom-clearance constraints."""
    constraints = _goal_constraints(slot, vehicle, yaw)
    if all(nx * preferred[0] + ny * preferred[1] > bound + 1e-5 for nx, ny, bound in constraints):
        return preferred[0], preferred[1], yaw
    if not adjust:
        raise ValueError('显式后轴目标不满足泊入包络')

    extent = 2 * (vehicle['rear_to_front_m'] + vehicle['rear_to_back_m'] + 2*vehicle['parkin_margin_m'])
    points = [(slot[f'slot_pt{i}_x'], slot[f'slot_pt{i}_y']) for i in range(1, 5)]
    xs, ys = zip(*points)
    region = [(min(xs)-extent, min(ys)-extent), (max(xs)+extent, min(ys)-extent),
              (max(xs)+extent, max(ys)+extent), (min(xs)-extent, max(ys)+extent)]

    for nx, ny, bound in constraints:
        clipped = []
        for a, b in zip(region, region[1:] + region[:1]):
            da, db = nx*a[0]+ny*a[1]-bound-1e-3, nx*b[0]+ny*b[1]-bound-1e-3
            if da >= 0:
                clipped.append(a)
            if (da >= 0) != (db >= 0):
                t = da / (da - db)
                clipped.append((a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1])))
        region = clipped
    if not region:
        raise ValueError('车位不存在满足泊入包络的目标')
    candidates = []
    for a, b in zip(region, region[1:] + region[:1]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        norm = dx*dx + dy*dy
        t = max(0.0, min(1.0, ((preferred[0]-a[0])*dx+(preferred[1]-a[1])*dy)/norm)) if norm else 0.0
        candidates.append((a[0]+t*dx, a[1]+t*dy))
    x, y = min(candidates, key=lambda p: (p[0]-preferred[0])**2 + (p[1]-preferred[1])**2)
    return x, y, yaw
