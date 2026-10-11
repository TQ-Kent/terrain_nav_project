# Project Status

Last updated: 2026-10-10. Repo: `~/robotic/terrain_nav_project` (own Git repo; the forked example repo `Robotics-Nav2-SLAM-Example` is separate and not the active work).
Latest running version: tag `checkpoint-2` (commit `c8e8de2`), merged into `main` via PR #1. `checkpoint-1` is the earlier base.

## Goal and architecture
Compare 2D Nav2 global planners and local controllers at runtime, as a baseline before the paused 3D terrain work.
Unity 6 (TurtleBot3, 180-beam lidar, `/clock`, TF) <-> ROS-TCP-Endpoint (`main-ros2`, submodule) <-> `slam_toolbox` + Nav2 -> `/cmd_vel` -> Unity.
Algorithms are switched live with `ros2 run terrain_navigation set_algo` (BT selector topics).

## Implemented features
- Unity robot, scan, TF, clock, `/cmd_vel` drive; `MainScene` built by an editor menu (**TerrainNav -> Build Main Scene**).
- Nav2 params with 4 planners (`GridBased`, `Smac2D`, `SmacHybrid`, `ThetaStar`) and 4 controllers (`FollowPath` = DWB, `MPPI`, `RPP`, `Graceful`).
- Launch: `ros2 launch terrain_nav_bringup bringup_baseline_2d.launch.py [planner:= controller:=]`.
- `slam_toolbox_2d.yaml`: map update 1 s, min travel 0.1 m / 0.2 rad.
- Global costmap: fixed 40 x 40 m, no static layer (goals outside the explored map are accepted).
- Lidar reports no-hit beams as `+inf`, range includes the 0.12 m start offset, `RangeMetersMax` 3.5; `inf_is_valid` on both costmap scan sources.
- RViz: top-down view, no `/terrain_points` display, global costmap hidden by default.

## Current working state
Runs end to end in the Unity editor per user report ("test ran successfully" on the latest version). 3D milestones 3-8 are paused.

## Tests performed
| Test | Result | Evidence |
| :-- | :-- | :-- |
| Stages 0-4 and 5 (build, bridge, drive, scan, SLAM+Nav2) | passed | observed this session: `/map` grew, `map->odom` from `slam_toolbox`, nodes active |
| Stage 6 (goals), clean launch | passed | observed: two goals succeeded within tolerance (0.16 m, 0.15 m) |
| Stage 7 (runtime switching) | reported passed by user | not observed by me |
| `/map` rate after SLAM change | 1.0 Hz (was 0.2 Hz) | measured with `ros2 topic hz` |
| Goal outside explored map | passed | goal (3, 3) reached in Unity; before the change: `outside bounds`, error 204 |
| Map vs scene geometry | matches | all 138 occupied cells on the 4 cylinders |
| Plan vs real path | within 6.5 cm | recorded in Unity, 59 samples |
| Headless simulator, old vs fixed lidar | fixed run: 3/3 goals; old run: 2/3 (third stalled) | single run each; simulator, not Unity |
| Lidar script/prefab change in Unity | reported passed by user | I did not observe it |

## Known bugs and unresolved issues
- Robot stalls near a cylinder inside the inflation zone; DWB reports `No valid trajectories out of 419`. Cause not found.
- The earlier 7-pair planner/controller table is not a fair comparison (different start poses, single trials).
- Restarting Play in Unity resets `/clock` while ROS keeps running: `slam_toolbox` and RViz then flood `TF_OLD_DATA`. Restart the ROS launch whenever Play is restarted. No automatic fix yet.
- Nav2 startup warnings (missing `current_progress_checker`, `current_goal_checker`, dock database, error-code params) are harmless but unfixed; RViz logs `rviz/glsl120/indexed_8bit_image.vert`.
- Duplicate `ROSClockPublisher` on `ROSClock` (self-disables); `AGV diag:` log spam; `rootVel=-1` in that log is unchecked.
- Not verified: whether the `+inf` change improves the SLAM map itself (the headless test showed no SLAM difference; the costmap improvement is real).
- The speed-up recommendations are untested.

## Important technical decisions
- `slam_toolbox` owns `map->odom`; Unity publishes only `odom`.
- Fixed-size global costmap instead of a static layer, so goals can lie outside the explored area. Trade-off: planners use lidar-marked obstacles, not the SLAM map.
- Algorithms loaded at startup and chosen by selector topic, so no relaunch is needed.
- `ROS-TCP-Endpoint` pinned as a submodule to `main-ros2` at `e9537d5` (the ROS 1 `main` branch does not build).

## Files changed (since the initial commit)
`ros2_ws/src/terrain_navigation/config/{nav2_params_2d.yaml,slam_toolbox_2d.yaml}`, `ros2_ws/src/terrain_nav_bringup/{launch/bringup_baseline_2d.launch.py,rviz/terrain_nav.rviz}`, `unity_project/Assets/{Scripts/Sensors/LaserScanSensor.cs,Prefabs/TurtleBot3ManualConfig.prefab}`, `ALGORITHM_TEST_PLAN.md`, `PROJECT_STATUS.md`.

## Next steps
1. Decide the open questions at the end of `ALGORITHM_TEST_PLAN.md` (headless vs Unity bed, speed profile, extra algorithms).
2. Measure baselines (Unity startup, Nav2 activation, goal times).
3. Find and fix the stall near obstacles before comparing algorithms.
4. Build the headless simulator, scenarios and `benchmark_runner.py`; run the planner sweep, then the controller sweep.
