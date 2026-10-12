"""Load config/maze_test.yaml, apply overrides, and write the Nav2 params with the speeds in."""
import copy
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


def nav2_params(base_path, speed, tolerance=None, tag=''):
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
    if tolerance:
        c['general_goal_checker'].update(xy_goal_tolerance=float(tolerance['xy']),
                                         yaw_goal_tolerance=float(tolerance['yaw']))
        c['FollowPath']['xy_goal_tolerance'] = float(tolerance['xy'])
        c['progress_checker']['required_movement_radius'] = float(tolerance['progress_radius'])
    s = p['velocity_smoother']['ros__parameters']
    s['max_velocity'] = [v, 0.0, w]
    s['min_velocity'] = [-rev, 0.0, -w]
    s['max_accel'] = [a, 0.0, aw]
    s['max_decel'] = [-a, 0.0, -aw]

    out = os.path.join(tempfile.gettempdir(), f'terrain_nav_params_maze_{os.getpid()}{tag}.yaml')
    with open(out, 'w') as f:
        yaml.safe_dump(p, f, sort_keys=False)
    return out


def merged(config_path, overrides=''):
    return apply_overrides(copy.deepcopy(load(config_path)), overrides)
