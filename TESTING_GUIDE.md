# Testing Guide: Unity 6 + ROS 2 Jazzy 2D SLAM/Nav2 (TurtleBot3)

Work through the stages in order. Each stage has a **Pass** check; do not continue until it passes.
Paths assume `~/robotic/terrain_nav_project`.

## Stage 0: Prerequisites and build

```bash
cd ~/robotic/terrain_nav_project/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```
Use `source ~/robotic/terrain_nav_project/ros2_ws/install/setup.bash` in every new terminal.

**Pass:** build finishes with no errors; `ros2 run terrain_navigation set_algo --list` prints the planners and controllers.

## Stage 1: Unity project opens cleanly

1. Open `unity_project` in Unity 6000.0.84f1 and let it import (the TurtleBot meshes take a while the first time).
2. Open the Console and clear it.
3. **Robotics → ROS Settings**: IP `127.0.0.1`, port `10000`, protocol **ROS2**.
4. **TerrainNav → Build Main Scene** (creates `Assets/Scenes/MainScene.unity` with the TurtleBot prefab at the origin).

**Pass:** zero compile errors; the Hierarchy shows `TurtleBot3ManualConfig`, `Ground`, `ROSTestPublisher`.
**If it fails:**
- Missing-script warnings on the prefab: remapped script GUIDs did not resolve. Reimport the prefab, or re-add the component by hand.
- Meshes pink: URP material conversion needed. Select the materials, then Edit → Rendering → Materials → Convert Selected Built-in Materials to URP.

**Before Stage 5 (SLAM):** select the robot's `ROSTransformTreePublisher` and remove `map` from `Global Frame Ids`, leaving only `odom`. Otherwise Unity and `slam_toolbox` both publish `map→odom`.

## Stage 2: Bridge, clock, TF

Terminal A:
```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0 -p ROS_PORT:=10000
```
Press **Play** in Unity. Terminal B:
```bash
ros2 topic list
ros2 topic hz /clock
ros2 topic echo /tf --once
ros2 run tf2_tools view_frames
```

**Pass:**
- Terminal A logs a connection from `127.0.0.1`; Unity's top-right ROS indicator turns green.
- `/clock` publishes continuously.
- `/tf` is present and the tree shows `odom → base_footprint → base_link → ...` (plus `base_scan`).

**If it fails:** wrong IP/port or protocol not ROS2; firewall on port 10000; Unity must be in Play mode.

## Stage 3: Drive the robot

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
# or, with the teleop package installed:
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
**Pass:** the robot moves forward in Unity and stops about 0.5 s after you stop publishing (`ROSTimeout`). Positive `angular.z` turns left. `ros2 run tf2_ros tf2_echo odom base_footprint` shows the pose changing.
**If it fails:** `AGVController` mode must be **ROS**; check the topic is `cmd_vel` and the robot's physics layer is `TurtleBot3Manual`.

## Stage 4: Laser scan

```bash
ros2 topic hz /scan
ros2 topic echo /scan --once | head -30
```
Add a few boxes in front of the robot in the Unity scene.

**Pass:** about 10 Hz, `frame_id: base_scan`, ranges shorter in front of obstacles. In RViz2 (fixed frame `odom`) the LaserScan display lines up with the boxes in Unity.
**Common bug:** the scan ray mask must not hit the robot itself (the prefab's `LayerMaskName: TurtleBot3Manual` handles this); if every range is about 0.1 m, check the layer.

## Stage 5: 2D SLAM + Nav2

Stop Terminal A's manual endpoint (the launch file starts its own), then:
```bash
ros2 launch terrain_nav_bringup bringup_baseline_2d.launch.py
```
Press Play in Unity. Drive around with teleop.

**Pass:**
- `ros2 topic echo /map --once` returns a grid, and the map grows in RViz2 as you drive.
- `ros2 run tf2_ros tf2_echo map odom` works (published by `slam_toolbox`).
- `ros2 lifecycle get /planner_server` and `/controller_server` report `active`.

**If it fails:**
- Nodes stuck waiting for TF: `/clock` is not publishing, or nodes are not using sim time (`use_sim_time:=true` is the default).
- `map→odom` jumping or duplicated: Unity is still publishing `map` (see the Stage 1 note).

## Stage 6: Navigate to a goal

In RViz2, use **Nav2 Goal** (or the 2D Goal Pose button) to set a goal on free map space.

**Pass:** a global path appears, the robot drives to the goal and stops within about 0.25 m.

## Stage 7: Switch algorithms at runtime

Do these between goals; the next goal uses the new choice.
```bash
ros2 run terrain_navigation set_algo --list
ros2 run terrain_navigation set_algo --planner Smac2D
ros2 run terrain_navigation set_algo --controller MPPI
ros2 run terrain_navigation set_algo --planner SmacHybrid --controller RPP
```
Or choose the starting pair at launch: `... bringup_baseline_2d.launch.py planner:=ThetaStar controller:=Graceful`.

For each pair, send the same goal from the same start pose and record success, path length and time.

| Planner | Controller | Reached? | Time (s) | Path (m) | Notes |
| :--- | :--- | :---: | :---: | :---: | :--- |
| GridBased | FollowPath | | | | |
| Smac2D | FollowPath | | | | |
| SmacHybrid | MPPI | | | | |
| ThetaStar | RPP | | | | |

**Pass:** the Nav2 logs show the planner/controller changing and the path shape differs between planners.
**Not yet verified in this project:** whether the selector topics take effect on a live Nav2; if the path never changes, check `ros2 topic info /planner_selector` for a subscriber from `bt_navigator`.

## Stage 8: Maze with a known goal (unknown map)

Why: with the baseline launch the planner treats unexplored space as free, so in a maze it plans through unseen walls and bounces between dead ends. The maze launch plans only through space SLAM has seen and explores toward the goal.

1. In Unity: **TerrainNav -> Build Maze Scene** (writes `Assets/Scenes/MazeScene.unity`, 6x6 seeded maze, 12.5 m square, 1.5 m corridors; the Console prints the layout and goal). Open the scene and press Play.
2. Copy-paste run (goal already included: far corner of the maze, ROS x=10, y=-10). Press Play in Unity first:
```bash
cd ~/robotic/terrain_nav_project/ros2_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch terrain_nav_bringup bringup_maze_2d.launch.py goal_x:=10.0 goal_y:=-10.0
```
Build first if needed: `colcon build --symlink-install`. Add `planner:=Smac2D controller:=MPPI` to pick algorithms. Other options (all optional; do not type square brackets): `goal_weight:=1.0` (pull of exploration toward the goal, 0 = nearest frontier first), `stall_timeout:=40.0`, `localization:=ground_truth|slam`. The goal is in the map frame, whose origin is the robot start (assumes the robot starts at the origin facing Unity +z).
3. About 20 s after launch `goal_explorer` starts. Watch its log: `frontier -> (x, y)` lines, then `GOAL -> ...` once the goal is reachable through known space, then `GOAL REACHED`.
4. The test stops itself: if the robot moves less than 0.15 m for `stall_timeout` seconds (default 40, sim time) `goal_explorer` logs `STALLED`, writes the costmap it used to `/tmp/goal_explorer_stall.npz`, and the whole launch shuts down. Use `stall_timeout:=0` to disable.
5. `localization:=ground_truth` (default) publishes a static identity `map->odom` (Unity's odom is the true pose) and runs `slam_toolbox` as a pure mapper (`slam_toolbox_2d_gt.yaml`: no scan matching, no loop closure, no TF). Use `localization:=slam` to let `slam_toolbox` localize. In the maze that mode snaps: `map->odom` jumped 1.8 m in 1 s in a long corridor (observed), the robot teleports in RViz, and the costmap splits.
6. Change algorithms live with `set_algo` as in Stage 7.

**Pass:** the robot reaches the goal without returning into an already-explored dead end.
**Verified so far:** one end-to-end Unity run with `localization:=ground_truth` reached the goal (10, -10) with default planner/controller; no `map->odom` jumps. `localization:=slam` was observed to fail (see 5). Not yet repeated across planners/controllers or trials; the shutdown-on-goal change was built but not re-run.
If `goal_explorer` logs `no frontier left and goal unreachable`, check the goal is inside the maze and the start pose matches the assumption above.

## Quick troubleshooting

| Symptom | Likely cause |
| :--- | :--- |
| Unity compile errors about `Clock` | Old `TerrainNavigation.Clock` namespace left behind; it is now `TerrainNavigation.ClockSync` |
| No connection | Endpoint not running, wrong port, ROS Settings protocol not ROS2 |
| Robot ignores `/cmd_vel` | `AGVController` in Keyboard mode, or topic mismatch |
| Nav2 never activates | No TF or `/clock`; `slam_toolbox` has not produced `/map` yet |
| Robot spins or oscillates | Wheel `trackWidth`/`wheelRadius` mismatch vs. the controller's assumptions |
| Planner "failed to create plan" | Goal in unknown or inflated space; pick a goal in mapped free space |
| `malformed launch argument '[planner:='` | Square brackets in a usage line were typed literally; omit them |

## Problems found while running this guide (2026-10-08) and their fixes

| Symptom | Cause | Fix (already applied) |
| :--- | :--- | :--- |
| Compile errors `Clock` namespace | `TerrainNavigation.Clock` namespace shadowed the `Clock` class | namespace renamed `TerrainNavigation.ClockSync` |
| TurtleBot pink | no URP pipeline asset assigned | copied `Assets/Settings/UniversalRenderPipelineAsset*.asset`, assigned in Graphics + all Quality levels |
| Robot ignores `/cmd_vel`, wheels do not turn | URDF wheel inertia (~1e-5) is too small for Unity 6 to follow the drive | `AGVController.wheelInertia = 1e-3` applied in the first FixedUpdate |
| Robot 4.5x too fast | prefab instance in the scene scaled 5x | scale must be (1,1,1); `AGVController` now warns otherwise |
| Turn rate wrong | wheel speed difference used the full track width | uses half of it |
| `/clock`, `/scan` and TF only ~1 Hz | Unity throttled (Ultra quality, v-sync, window unfocused under Wayland) | `runInBackground`, v-sync off, Low default quality, 60 fps cap |
| Two `map->odom` publishers | prefab TF frames were `{map, odom}` | now `{odom}` only; `slam_toolbox` owns `map->odom` |
| Nav2 stays `unconfigured` | (1) MPPI `model_dt` 0.05 != controller period 0.1 (2) `collision_monitor`, `route_server`, `docking_server` had no params (3) Galactic `plugin_lib_names` list crashed `bt_navigator` ("ID already registered") (4) `default_nav_to_pose_bt_xml: ""` means an empty tree | fixed in `nav2_params_2d.yaml` and `nav2_params_3d.yaml` |

Operational traps: stale `lifecycle_manager` / launch processes from an earlier run keep fighting a new launch (check `ps -eo pid,args | grep -E "lifecycle_manager|ros2 launch"` before relaunching); and `pkill -f` patterns can match your own shell command.

## Verified results (2D SLAM + Nav2, Unity editor, TurtleBot3)

All 11 managed nodes reached `active`. A 1 m goal succeeded with the default pair. Runtime switching via `set_algo`, alternating 1 m goals (goal tolerance 0.25 m):

| Planner | Controller | Result | Time | Notes |
| :--- | :--- | :---: | :---: | :--- |
| GridBased | FollowPath | OK | ~0 s | robot already within tolerance (not a valid timing) |
| Smac2D | FollowPath | OK | 13.5 s | 14 re-plans |
| SmacHybrid | FollowPath | OK | 3.3 s | 4 re-plans |
| ThetaStar | FollowPath | OK | 13.3 s | 13 re-plans |
| GridBased | MPPI | OK | 3.5 s | |
| GridBased | RPP | OK | 22.4 s | slow; "Failed to make progress" logged during testing |
| GridBased | Graceful | OK | 4.3 s | |

Caveat: these runs started from different poses, so times and path lengths are indicative only. Controller switching clearly changed behaviour; planner switching is supported by differing plan shapes and re-plan counts but was not isolated with an identical start pose. Use the logger idea in the plan (Section 0.3) for a fair comparison.
