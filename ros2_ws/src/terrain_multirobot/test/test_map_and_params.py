import os
import sys

import yaml

from terrain_multirobot.map_tools import layout_to_grid, read_layout

HERE = os.path.dirname(__file__)
SRC = os.path.abspath(os.path.join(HERE, '..', '..'))
LAYOUT = os.path.join(SRC, 'terrain_navigation', 'maps', 'maze_seed42.txt')
PARAMS = os.path.join(SRC, 'terrain_navigation', 'config', 'nav2_params_maze.yaml')


def free_at(img, ox, oy, x, y):
    col = int((x - ox) / 0.05)
    row = len(img) - 1 - int((y - oy) / 0.05)
    return img[row][col] == 254


def test_start_cell_free_and_matches_headless_walls():
    sys.path.insert(0, os.path.join(SRC, 'terrain_navigation'))
    import numpy as np
    from terrain_navigation.headless_sim import CHAR, ORIGIN_SHIFT, load_layout
    wall = load_layout(LAYOUT)
    img, ox, oy = layout_to_grid(read_layout(LAYOUT))
    assert free_at(img, ox, oy, 0.0, 0.0)
    mismatch = 0
    for x in np.arange(-1.2, 11.0, 0.3):
        for y in np.arange(-11.0, 1.2, 0.3):
            cx = int(np.floor((-y + ORIGIN_SHIFT) / CHAR))
            cz = int(np.floor((x + ORIGIN_SHIFT) / CHAR))
            if 0 <= cx < wall.shape[0] and 0 <= cz < wall.shape[1]:
                if wall[cx, cz] == free_at(img, ox, oy, x, y):
                    mismatch += 1
    assert mismatch == 0


def test_namespace_params():
    sys.path.insert(0, os.path.join(SRC, 'terrain_nav_bringup'))
    from terrain_nav_bringup.maze_config import namespace_params
    out = namespace_params(PARAMS, 'robot2', tag='_test')
    p = yaml.safe_load(open(out))
    os.remove(out)
    assert p['bt_navigator']['ros__parameters']['robot_base_frame'] == 'robot2/base_link'
    assert p['collision_monitor']['ros__parameters']['base_frame_id'] == 'robot2/base_footprint'
    local = p['local_costmap']['local_costmap']['ros__parameters']
    assert local['robot_base_frame'] == 'robot2/base_link' and local['global_frame'] == 'odom'
    assert local['obstacle_layer']['scan']['topic'] == 'scan'
    assert p['global_costmap']['global_costmap']['ros__parameters']['static_layer']['map_topic'] == '/map'
