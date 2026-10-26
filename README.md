# Terrain Nav: 2D SLAM + Nav2 maze navigation (Unity 6 + ROS 2 Jazzy)

Version 1.0.0. A TurtleBot3 simulated in Unity 6 explores an unknown maze toward a known goal using `slam_toolbox`, Nav2 and a goal-biased frontier explorer. Four global planners and four local controllers can be compared, in Unity or in a built-in headless simulator.

```
Unity 6 (TurtleBot3, lidar, /clock, TF) <-> ROS-TCP-Endpoint <-> slam_toolbox + Nav2 + goal_explorer -> /cmd_vel -> Unity
```

## Layout

| Path | Contents |
| :-- | :-- |
| `unity_project/` | Unity 6000.0.84f1 project. Editor menu **TerrainNav** builds `MainScene` and `MazeScene`; `RunTracker` times runs and draws the path |
| `ros2_ws/src/terrain_navigation/` | Nav2 and SLAM params, `goal_explorer` (frontier exploration), `headless_sim` (Unity-free simulator), `set_algo` (live planner/controller switch), maze map |
| `ros2_ws/src/terrain_nav_bringup/` | Launch files (`bringup_maze_2d`, `bringup_baseline_2d`, `bringup_simulation`), `config/maze_test.yaml` (all test settings), RViz config |
| `ros2_ws/src/ROS-TCP-Endpoint/` | Git submodule (Unity Technologies, `main-ros2`) |
| `tools/` | `benchmark.py` (planner x controller trials and governor runs), `halt_analysis.py` (where a run loses time), `gen_maze.py` and `mazes/` (generated test mazes), governor helpers |
| `docs/` | See below |

## Quick start

```bash
git clone --recurse-submodules <repo> && cd terrain_nav_project/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install && source install/setup.bash

# Without Unity (headless simulator):
ros2 launch terrain_nav_bringup bringup_maze_2d.launch.py simulator:=headless goal_x:=10.0 goal_y:=-10.0

# With Unity: open unity_project, TerrainNav -> Build Maze Scene, press Play, then
ros2 launch terrain_nav_bringup bringup_maze_2d.launch.py goal_x:=10.0 goal_y:=-10.0 planner:=Smac2D controller:=MPPI
```

Planners: `GridBased`, `Smac2D`, `SmacHybrid`, `ThetaStar`. Controllers: `FollowPath` (DWB), `MPPI`, `RPP`, `Graceful`. Switch live with `ros2 run terrain_navigation set_algo --planner X --controller Y`. Optional speed governor between the smoother and the collision monitor: `set:=governor.enabled=true` (default off, see `docs/GOVERNOR.md`).

## Documentation

- [`docs/USER_GUIDE.md`](docs/USER_GUIDE.md): staged setup and test procedure, speed settings, troubleshooting.
- [`docs/ALGORITHMS.md`](docs/ALGORITHMS.md): what each planner and controller does, with the math.
- [`docs/RESULTS.md`](docs/RESULTS.md): planner x controller benchmark (raw data in `docs/results/benchmark_combo.csv`).
- [`docs/SESSION_2026-10-19.md`](docs/SESSION_2026-10-19.md): Unity stall investigation: findings, the `profile:=reliable` setup (5/5 runs), results, gotchas, next ideas.
- [`docs/PLANNING_COMPARISON.md`](docs/PLANNING_COMPARISON.md): planner compute time here vs published figures and similar projects.
- [`docs/GOVERNOR.md`](docs/GOVERNOR.md): adaptive speed governor (research prototype, off by default): design, parameters, tests, benchmark, results.
- [`docs/MULTIROBOT_PREP.md`](docs/MULTIROBOT_PREP.md): single-robot assumptions and a suggested order for the multi-robot upgrade.
- [`docs/SYSTEM_REVIEW.md`](docs/SYSTEM_REVIEW.md): how the maze test works and where the time is lost.

## Known limitations

- Localization defaults to Unity ground truth; `localization:=slam` snaps in long corridors.
- Robots can lock up against a wall (collision monitor zeroes `/cmd_vel`); see `SYSTEM_REVIEW.md`.
- One maze (seed 42), two trials per planner/controller pair; 3D terrain work is not part of this version.
