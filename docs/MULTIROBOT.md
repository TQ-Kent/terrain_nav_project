# Multi-robot task allocation (CBBA) in Unity

Branch `feature/multirobot`. Plan: `~/.claude/plans/cached-plotting-dongarra.md`. Reference implementation:
`/home/ken/robotic/cbba_test` (Choi, Brunet, How 2009), vendored in `terrain_multirobot/terrain_multirobot/cbba/`.

## Status (2026-10-26)

| Part | State |
|---|---|
| CBBA allocator per robot (`allocation.py`), async tests | done, 3 tests pass (random cases, 3 robots x 6 tasks: every task once, robots agree, score within 5 % of the lock-step reference) |
| Messages `terrain_multirobot_msgs`, `cbba_agent`, `task_publisher`, `tf_relay` nodes | built; agent not yet run against a real Nav2 + simulator |
| Static maze map from the layout file (`map_tools.py`) | done, tested against the headless simulator's wall grid |
| Per-robot Nav2 params (`maze_config.namespace_params`) and `bringup_multirobot.launch.py` | two namespaced Nav2 stacks start without errors **without Unity**; no robot has driven yet |
| Unity: `RobotNamespace`, namespaced cmd_vel, scan, TF; menu `TerrainNav > Build Maze Scene (2/3 robots)` | **written, never compiled or run**: open Unity, check the console, build the scene |
| Robot-robot avoidance, `localization:=slam`, range-limited links, path-length scoring | not done |

## How it fits together

- Unity publishes one global `/tf`. With a `RobotNamespace` component on a robot, its topics are `/robotK/scan`, `/robotK/cmd_vel` and its frames `robotK/base_link`, ... `map` and `odom` stay shared (`map` -> `odom` identity, `odom` = Unity world origin), so a robot's pose in `map` is its true pose.
- Nav2's `navigation_launch.py` reads `/robotK/tf`, so `tf_relay` copies the global TF there. The launch pushes the namespace itself (that file does not).
- v1 uses a known map (`map_server`, `/map`) so every task point is reachable; no SLAM or exploration. Task points are in the map frame (robot 0's start).
- Each `cbba_agent` loops at 2 Hz: read the other robots' `/cbba/messages`, resolve conflicts, bid, broadcast. When its state is unchanged for 6 steps and it has heard every robot, it sends its path to its own Nav2 task by task. Scoring uses straight-line distance (a wall makes this wrong; path-length scoring is next).

## Run

```bash
# Unity: TerrainNav > Build Maze Scene (3 robots), open Assets/Scenes/MazeScene_3robots.unity, press Play
cd ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install && source install/setup.bash
ros2 launch terrain_multirobot bringup_multirobot.launch.py robots:=3 profile:=reliable tasks:=tasks_demo.yaml
```
Press Play again before every run. Stop the launch with `kill -TERM <launch pid>`; its child processes can outlive it, so check with `pgrep -af nav2_`.

## Changes against the reference `cbba/`
`main.py` was left out; the rest is unchanged. `simulation.simulate()` stays as the offline oracle used in the tests; the robots do not use it.

## Known single-robot issues that carry over
Start-cell wall lock, RPP `collision ahead` silences, 31-74 s spread on one maze: `SESSION_2026-10-19.md` sections 3 and 9, `GOVERNOR.md`.
