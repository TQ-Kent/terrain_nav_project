"""Load config/maze_test.yaml, apply overrides, and write the Nav2 params with the speeds in."""
import copy
import math
import os
import tempfile

import yaml


def load(path):
    with open(path) as f:
        return yaml.safe_load(f)


def set_dotted(cfg, key, value):
    node = cfg
    parts = key.split('.')
    for p in parts[:-1]:
        node = node.setdefault(p, {})
    if parts[-1] not in node and len(parts) > 1 and parts[0] not in cfg:
        raise KeyError(key)
    node[parts[-1]] = value


def apply_overrides(cfg, overrides):
    """overrides: 'a.b=1,c.d=x' string or list of 'k=v'. Values are parsed as YAML."""
    if isinstance(overrides, str):
        overrides = [o for o in overrides.split(',') if o.strip()]
    for item in overrides:
        key, _, value = item.partition('=')
        key = key.strip()
        top = key.split('.')[0]
        if top not in cfg:
            raise KeyError(f"unknown config section '{top}' in override '{item}'")
        set_dotted(cfg, key, yaml.safe_load(value))
    return cfg


def _octagon(r):
    pts = [[round(r * math.cos(k * math.pi / 4), 3), round(r * math.sin(k * math.pi / 4), 3)] for k in range(8)]
    return str(pts)


def _rotation_zone(cm, radius, reverse_zone=True):
    """Replace FootprintApproach by a velocity_polygon (Nav2 Iron+, nav2 issue #3313).

    Turning in place uses a disc of `radius` (just inside the robot, so a wall that the robot
    can reach never lies inside it and rotation is never blocked). Every other command keeps the
    usual approach check, now on a static disc of the full robot radius instead of the costmap
    footprint (sub-polygons only take static points).
    """
    old = cm['FootprintApproach']
    r_full = 0.22
    common = dict(linear_min=-10.0, linear_max=10.0, theta_min=-10.0, theta_max=10.0)
    cm['polygons'] = ['VelApproach']
    del cm['FootprintApproach']
    cm['VelApproach'] = {
        'type': 'velocity_polygon', 'action_type': 'approach', 'holonomic': False,
        'time_before_collision': old['time_before_collision'],
        'simulation_time_step': old['simulation_time_step'], 'min_points': old['min_points'],
        'visualize': True, 'enabled': True,    # published on /VelApproach so RViz shows the active zone
        'velocity_polygons': ['rotate', 'reverse', 'drive'] if reverse_zone else ['rotate', 'drive'],
        'rotate': dict(points=_octagon(radius), linear_min=-0.02, linear_max=0.02,
                       theta_min=-10.0, theta_max=10.0),
        'drive': dict(points=_octagon(r_full), **common),
    }
    if reverse_zone:
        # Backing up only has to clear what is behind the robot: the rear rectangle (base box ends 0.197 m behind the
        # axle, wheels +-0.153 m). With the drive disc a wall touching the front or side blocked the backup
        # recovery too, and nothing could free the robot (g4, 2026-10-19).
        cm['VelApproach']['reverse'] = dict(points='[[0.0, 0.153], [0.0, -0.153], [-0.2, -0.153], [-0.2, 0.153]]',
                                            linear_min=-10.0, linear_max=-0.02, theta_min=-10.0, theta_max=10.0)


# Named footprints (base_footprint frame, x forward, y left), built from the TurtleBot3 waffle_pi URDF in the
# Unity project: the two drive wheels sit at y = +-0.144 m, 0.018 m thick, so their outer faces are at +-0.153 m
# (the wheels, not the 0.266 m body, set the width); the base box spans x -0.197 .. +0.069 m; the rear casters
# (x = -0.177 m) are inside that. Padded by 0.02 m on every side.
_WHEEL_OUTER_Y = 0.144 + 0.018 / 2
_PAD = 0.02
_HW, _FRONT, _REAR = _WHEEL_OUTER_Y + _PAD, 0.069 + _PAD, 0.197 + _PAD
FOOTPRINTS = {'waffle': [[_FRONT, _HW], [_FRONT, -_HW], [-_REAR, -_HW], [-_REAR, _HW]]}
# Planning radius for the global costmap when a footprint is set. The rectangle's origin is off-centre, so its
# inscribed radius is only 0.09 m and a planner using it routes through gaps the controller's footprint check
# then refuses ("collision ahead"). 0.17 m = half the body width plus padding.
PLANNING_RADIUS = {}   # opt-in only: costmap.planning_radius. {'waffle': 0.17} stalled in Unity (2026-10-19)


def _costmap(p, cm):
    """Opt-in costmap shape: `footprint` (list of [x, y], base_footprint frame) replaces the
    robot_radius circle, `inflation_radius` / `cost_scaling_factor` retune both inflation layers.
    A rectangle that matches the real robot lets the planner use gaps the circle closes."""
    radius = float(cm.get('robot_radius') or 0)
    if radius:
        # Plain circle for planning, controller checks and the monitor. Nav2 takes the inscribed radius of a
        # rectangle from its shortest edge from the origin (0.089 m at the front), so a rectangle needs a
        # gap of 2 * (half width + 0.089 m) on the sides, wider than a circle sized to the wheels.
        for name in ('local_costmap', 'global_costmap'):
            node = p[name][name]['ros__parameters']
            node.pop('footprint', None)
            node['robot_radius'] = radius
            layer = node['inflation_layer']
            if cm.get('inflation_radius'):
                layer['inflation_radius'] = float(cm['inflation_radius'])
            if cm.get('cost_scaling_factor'):
                layer['cost_scaling_factor'] = float(cm['cost_scaling_factor'])
        vp = p['collision_monitor']['ros__parameters'].get('VelApproach')
        if vp:
            # Approach disc just inside the wheel outer face (0.153 m): a wall point inside it means contact. At the
            # costmap radius (0.17 m) the planner lets the robot sit on the zone edge, so wall points land inside it
            # and the monitor zeroed the command (0.8 and 1.0 m/s stalls, 2026-10-19).
            zone = float(cm.get('drive_zone') or radius)
            vp['drive']['points'] = _octagon(zone / math.cos(math.pi / 8))   # flat sides at `zone`
        return
    fp = cm.get('footprint') or None
    plan_r = float(cm.get('planning_radius') or 0)
    if isinstance(fp, str):
        plan_r = plan_r or PLANNING_RADIUS.get(fp, 0)
        fp = FOOTPRINTS[fp]
    cm = dict(cm, footprint=fp)
    if fp:
        # Approach check on the same shape, otherwise a disc wider than the gap stops the robot there.
        vp = p['collision_monitor']['ros__parameters'].get('VelApproach')
        if vp:
            vp['drive']['points'] = str([[round(float(x), 3), round(float(y), 3)] for x, y in fp])
    for name, key in (('local_costmap', 'local_costmap'), ('global_costmap', 'global_costmap')):
        node = p[name][key]['ros__parameters']
        if cm.get('footprint') and name == 'global_costmap' and plan_r:
            node['robot_radius'] = plan_r              # planner keeps a circle, the controller uses the footprint
        elif cm.get('footprint'):
            node.pop('robot_radius', None)
            node['footprint'] = str([[round(float(x), 3), round(float(y), 3)] for x, y in cm['footprint']])
        layer = node['inflation_layer']
        if cm.get('inflation_radius'):
            layer['inflation_radius'] = float(cm['inflation_radius'])
        if cm.get('cost_scaling_factor'):
            layer['cost_scaling_factor'] = float(cm['cost_scaling_factor'])


def _update_typed(dst, src):
    """dst.update(src), but a number keeps the type of the value it replaces: Nav2 declares many parameters as
    double and refuses an int (cost_travel_multiplier=4 aborted the whole bringup), and YAML parses '4' as int."""
    for k, v in (src or {}).items():
        old = dst.get(k)
        if isinstance(old, float) and isinstance(v, int) and not isinstance(v, bool):
            v = float(v)
        dst[k] = v


def nav2_params(base_path, speed, tolerance=None, tag='', governor_enabled=False, costmap=None, rpp=None, planner=None):
    """Copy base Nav2 params with `speed` written into every controller and the smoother."""
    with open(base_path) as f:
        p = yaml.safe_load(f)
    v, rev = float(speed['max_linear']), float(speed['max_reverse'])
    w = float(speed['max_angular'])
    a, aw = float(speed['linear_accel']), float(speed['angular_accel'])

    c = p['controller_server']['ros__parameters']
    c['FollowPath'].update(max_vel_x=v, max_speed_xy=v, max_vel_theta=w,
                           acc_lim_x=a, decel_lim_x=-a, acc_lim_theta=aw, decel_lim_theta=-aw)
    c['MPPI'].update(vx_max=v, vx_min=-rev, wz_max=w)
    c['RPP'].update(desired_linear_vel=v, rotate_to_heading_angular_vel=w, max_angular_accel=aw)
    c['Graceful'].update(v_linear_max=v, v_angular_max=w)
    # collision_monitor "approach" slows/stops the robot when a wall is closer than
    # time_before_collision * speed. A fixed 1.2 s look-ahead grows with speed (0.6 m at 0.5 m/s)
    # and pins the robot at corner walls in 1.5 m corridors. Keep the look-ahead *distance* constant.
    lookahead = speed.get('collision_lookahead')
    if lookahead:
        p['collision_monitor']['ros__parameters']['FootprintApproach']['time_before_collision'] = \
            round(max(0.4, float(lookahead) / v), 3)
    if speed.get('rotation_zone'):
        _rotation_zone(p['collision_monitor']['ros__parameters'], float(speed['rotation_zone']),
                       reverse_zone=bool(speed.get('reverse_zone', True)))
    if governor_enabled:
        # Insert the governor between smoother and collision monitor. The monitor stays the last
        # stage before /cmd_vel and its thresholds are untouched.
        p['collision_monitor']['ros__parameters']['cmd_vel_in_topic'] = 'cmd_vel_governed'
    if tolerance:
        c['general_goal_checker'].update(xy_goal_tolerance=float(tolerance['xy']),
                                         yaw_goal_tolerance=float(tolerance['yaw']))
        c['FollowPath']['xy_goal_tolerance'] = float(tolerance['xy'])
        c['progress_checker']['required_movement_radius'] = float(tolerance['progress_radius'])
        if tolerance.get('progress_time'):
            # seconds without progress_radius of movement before the controller reports failure and the BT starts its
            # recoveries (clear costmap, spin, backup); the Nav2 default is 10 s
            c['progress_checker']['movement_time_allowance'] = float(tolerance['progress_time'])
    s = p['velocity_smoother']['ros__parameters']
    s['max_velocity'] = [v, 0.0, w]
    s['min_velocity'] = [-rev, 0.0, -w]
    s['max_accel'] = [a, 0.0, aw]
    s['max_decel'] = [-a, 0.0, -aw]

    if costmap:
        _costmap(p, costmap)
    if planner:
        # per-planner parameters, e.g. {'Smac2D': {'cost_travel_multiplier': 4.0}}, written as given
        for name, params in planner.items():
            _update_typed(p['planner_server']['ros__parameters'][name], params)
    if rpp:
        _update_typed(p['controller_server']['ros__parameters']['RPP'], rpp)   # any RPP parameter, written as given

    out = os.path.join(tempfile.gettempdir(), f'terrain_nav_params_maze_{os.getpid()}{tag}.yaml')
    with open(out, 'w') as f:
        yaml.safe_dump(p, f, sort_keys=False)
    return out


def merged(config_path, overrides=''):
    return apply_overrides(copy.deepcopy(load(config_path)), overrides)
