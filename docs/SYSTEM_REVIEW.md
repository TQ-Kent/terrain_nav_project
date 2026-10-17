# System review: how the maze test works, and where the time is lost

Written 2026-10-12 against branch `feature/maze-frontier-exploration` (commit `aac975d` plus `tools/halt_analysis.py`).
"Measured" means I recorded it in a run; "reasoned" means it follows from the code or settings but was not tested; "not verified" says what is missing.

---

## 1. Short answer: what keeps halting the robot

Measured on the headless simulator, Smac2D + RPP at 1.0 m/s, 7 runs (`tools/halt_analysis.py`):

| Run | Outcome | Time (s) | Controller wanted to move, final `/cmd_vel` was 0 |
| :-- | :-- | --: | --: |
| b | reached | 55.0 | 0.0 s |
| fast1 | **stalled** | 87.5 | 39.2 s (32.9 s of the last 35 s) |
| fast2 | **stalled** | 82.5 | 39.7 s (33.7 s of the last 35 s) |
| fast3 | **stalled** | 82.5 | 39.1 s (32.9 s of the last 35 s) |
| slow1 (old 1 s explorer loop) | **stalled** | 56.0 | 39.8 s (33.3 s of the last 35 s) |
| slow2 (old 1 s explorer loop) | reached | 62.0 | 0.0 s |
| slow3 (old 1 s explorer loop) | reached | 53.6 | 0.0 s |

(`a` is a fourth reached run, 53.3 s, recorded before the lock-up detector existed.)

**The biggest single loss is a lock-up against a wall.** The robot ends up with its centre within its own radius of a wall (the costmap cell under it has the "inscribed" cost 99). The local controller keeps commanding up to 1.0 m/s, the **collision monitor** (`FootprintApproach`) cuts the final `/cmd_vel` to 0, and the robot sits still. Nothing reports a failure, so:
1. Nav2's progress checker waits 10 s (`movement_time_allowance`) before it logs `Failed to make progress` and aborts the goal.
2. Nav2 then starts a recovery (a spin), which is blocked by the same wall.
3. `goal_explorer` keeps its target and waits. After 40 s of no movement (`stall_timeout`) it ends the whole test as `STALLED`.

In 4 of the 7 runs above this happened and the run was lost: 33 to 40 s of standing still, then the test stopped. In the 3 runs that reached the goal it never happened (0.0 s). With one maze, one goal, and seven runs this is a rough rate (4 of 7 at this speed), not a measured probability, but it dominates everything else:

| Cause of lost time | Cost | Evidence |
| :-- | :-- | :-- |
| **A. Wall lock-up** (collision monitor holds, no failure is reported) | the whole run: 33–40 s stopped, then the test ends | 4 of 7 runs; `cmd_nav` 1.00, `cmd_smooth` 1.00, `cmd_final` 0.00 at the stall; costmap cell under the robot = 99 |
| **B. Controller holds speed down** (RPP slows for curves) | 29 % of a successful 1.0 m/s run (about 16 s of 55 s) commanded below 80 % of top speed | run `b` |
| **C. Stop-and-go at frontier goals** | braking before a stop 7–14 %, speeding up after it 6–9 %, plus short halts | runs `a`, `b` |
| **D. Acceleration lag** | 9–16 % "commanded full speed but not there yet" | runs `a`, `b`, `c` |
| **E. Slow start of each goal** | negligible now: mean 0.03–0.26 s from a goal being accepted to the robot moving | runs `a`, `b`, `c`, fast/slow |

Causes already fixed earlier today (see section 6) were larger in Unity: the explorer's Python flood fill took 0.15–0.4 s per cycle on a 1 s loop, the lidar saw only 3.5 m, SLAM rejected the 360-beam scans, and a second ROS system on the LAN corrupted `/clock`. Together with the loop change they cut a Unity run from 163 s (0.23 m/s average) to 72.7 s (0.48 m/s) at the same 0.5 m/s limit, but they were changed together, so I cannot say how much each contributed.

At 0.5 m/s with DWB + GridBased (run `c`) the robot spends 75 % of the run at near-top speed, with no halt over 0.7 s; there the time is simply distance divided by speed. The problems above grow with speed.

---

## 2. What the system is

A simulated TurtleBot3 explores an unknown maze toward a goal whose position it is told. Two interchangeable simulators feed the same ROS 2 stack:

```
   Unity 6 (MazeScene)                                     ROS 2 Jazzy
 ┌────────────────────────────┐  TCP 10000   ┌──────────────────────────────────────────────┐
 │ TurtleBot3 (ArticulationBody)│◄───────────►│ ROS-TCP-Endpoint                              │
 │  LaserScanSensor  /scan 10Hz│  scan, tf,   │      │ /scan  /tf  /clock        ▲ /cmd_vel   │
 │  TF publisher     /tf 20Hz  │  clock       │      ▼                           │            │
 │  ROSClockPublisher /clock 50Hz│            │ slam_toolbox (mapper only) ──► /map            │
 │  AGVController ◄── /cmd_vel │             │ static map->odom (identity)                    │
 │  RunTracker (timer + trace) │             │      │                                         │
 └────────────────────────────┘              │      ▼                                         │
        or                                    │ Nav2: global costmap ─► planner ─► controller  │
 ┌────────────────────────────┐              │       ▲ (static layer = /map)      │           │
 │ headless_sim.py (Python)    │──same topics►│       │                  velocity_smoother     │
 │  ray-marched 360 beams      │              │       │                          ▼             │
 │  circle robot, accel limits │              │ goal_explorer ──NavigateToPose──► collision_monitor ─► /cmd_vel
 └────────────────────────────┘              └──────────────────────────────────────────────┘
```

### 2.1 Unity side (`unity_project/Assets`)

| File | What it does |
| :-- | :-- |
| `Editor/BuildMazeScene.cs` | menu **TerrainNav → Build Maze Scene**: carves a seeded 6 x 6 maze (seed 42, 1.5 m corridors, 0.5 m walls, 4 extra openings so there are loops), places a green `GoalMarker` at the far corner (ROS x 10, y -10), adds `RunTracker` to the robot |
| `Prefabs/TurtleBot3ManualConfig.prefab` | the robot: articulated chassis and wheels, `AGVController`, lidar, TF and clock publishers |
| `Scripts/Kinematics/AGVController.cs` | `/cmd_vel` (Twist) to wheel speeds; caps 1.0 m/s and 2.0 rad/s (`maxLinearSpeed`, `maxRotationalSpeed`); stops if no command arrives for 0.5 s (`ROSTimeout`) |
| `Scripts/Sensors/LaserScanSensor.cs` | 360 raycasts per scan, 10 Hz, range 0.12–10 m; a beam with no hit is reported as `+inf`; first and last beam sit on the end angles so the message's `angle_increment` matches the beam count |
| `Scripts/Kinematics/ROSTransformTreePublisher.cs` | `/tf` at 20 Hz: `odom -> base_footprint -> base_link -> ...` (Unity publishes no `map` frame here) |
| `Scripts/Clock/ROSClockPublisher.cs` | `/clock` at 50 Hz from Unity time, so every ROS node runs on simulation time |
| `Scripts/Tools/RunTracker.cs` | on-screen timer and red path line: the timer starts when the robot first moves 5 cm and stops within 0.4 m of the goal marker; **R** teleports the robot back to its start pose and clears both |

### 2.2 ROS side (`ros2_ws/src`)

| Package / file | What it does |
| :-- | :-- |
| `terrain_nav_bringup/config/maze_test.yaml` | the single control file: goal, planner, controller, speeds, tolerances, exploration settings, simulator choice |
| `terrain_nav_bringup/terrain_nav_bringup/maze_config.py` | merges `maze_test.yaml` with `set:=` overrides and writes the speeds into the Nav2 parameters of all four controllers and the velocity smoother |
| `terrain_nav_bringup/launch/bringup_maze_2d.launch.py` | starts everything: simulator or TCP endpoint, static `map -> odom`, `slam_toolbox`, Nav2, `goal_explorer`; sets `ROS_LOCALHOST_ONLY=1`; shuts the whole launch down when `goal_explorer` exits |
| `terrain_navigation/config/nav2_params_maze.yaml` | Nav2 parameters for the maze test (planners, controllers, costmaps, collision monitor) |
| `terrain_navigation/config/slam_toolbox_2d_gt.yaml` | `slam_toolbox` as a pure mapper: no scan matching, no loop closure, no TF output; map update every 1.0 s |
| `terrain_navigation/terrain_navigation/goal_explorer.py` | the exploration node (section 3) |
| `terrain_navigation/terrain_navigation/frontier_logic.py` | pure numpy/scipy frontier search used by the node |
| `terrain_navigation/terrain_navigation/headless_sim.py` | Unity replacement for benchmarks: same topics, no wheel slip, no physics contacts, no lidar noise |
| `terrain_navigation/terrain_navigation/set_algo.py` | switches the planner or controller at runtime through the BT selector topics |
| `tools/benchmark.py` | runs many headless trials and writes a CSV |
| `tools/halt_analysis.py` | records a run and classifies every halt (section 5) |

### 2.3 Why it is built this way

- **Ground-truth localization.** Unity's odometry is the true pose, so `map -> odom` is a fixed identity. Letting `slam_toolbox` localize made `map -> odom` jump 1.82 m in 1 s in a long corridor (observed in Unity). The map origin is the robot's start, so the goal (10, -10) is in that frame.
- **Static layer + `allow_unknown: false`.** Nav2 plans only through space SLAM has seen. The earlier version treated unknown space as free, planned straight through unseen walls, and bounced between dead ends.
- **Frontier exploration instead of one distant goal.** A planner that cannot see a route has nothing to plan; the explorer gives Nav2 a sequence of reachable frontier goals.

---

## 3. How the exploration works (`goal_explorer.py`, `frontier_logic.py`)

Every 0.25 s (and immediately when a frontier is reached):
1. Read the **global costmap** (not `/map`: it is what the planner will see, including inflation).
2. Flood-fill from the robot over cells that are known, free, and at least `clearance` (0.05 m) from any cell at cost 99 or above (scipy shortest paths, about 20 ms; a pure-Python search took 0.15–0.4 s).
3. A **frontier** is a reachable cell next to unknown space; clusters under 3 cells are ignored.
4. Score each frontier: `path length from the robot + goal_weight x straight distance to the goal` (`goal_weight` 1.0). Take the lowest.
5. Keep the current target unless a new one is better by `switch_margin` (1.0 m); send the best one to Nav2 `NavigateToPose`. A target that fails is blacklisted for 30 s.
6. If the goal cell itself is reachable through known space, send the goal.
7. If no frontier exists and the robot's own cell is inside the obstacle band, send it to the nearest clear cell (the "escape" fix; it has not yet triggered in any real run).

The test ends on **GOAL REACHED** (within 0.25 m), **STALLED** (moved under 0.15 m in 40 s) or **TIMEOUT** (`max_time`). Because the node is "required", the whole launch then shuts down.

### The latency chain behind each decision

| Stage | Rate or delay | Source |
| :-- | :-- | :-- |
| Lidar scan | 10 Hz | `LaserScanSensor` |
| `slam_toolbox` map | every 1.0 s | `map_update_interval: 1.0` |
| Global costmap (static layer from `/map`) | updates and publishes at 1 Hz | `nav2_params_maze.yaml` |
| Explorer decision | 0.25 s loop | `exploration.tick_period` |
| Nav2 replans the path | about every 1 s while a goal is active (one `PLAN` per second in the recordings) | behavior tree |
| Controller | 10 Hz | `controller_frequency: 10.0` |
| Velocity smoother | 20 Hz | `smoothing_frequency` |

So what the explorer knows about newly seen space can be up to about 2 s old (map update plus costmap update). At 1.0 m/s that is up to 2 m of driving on stale knowledge. This is **reasoned, not measured**: I did not time the age of the data at the moment of decisions.

---

## 4. How Nav2 is set up here

| Part | Setting | Value |
| :-- | :-- | :-- |
| Costmaps | robot radius, inflation radius, cost scaling | 0.22 m, 0.55 m, 3.0 |
| Global costmap | size, update | follows the SLAM map (static layer), 1 Hz |
| Local costmap | size, update | 6 x 6 m rolling window, 5 Hz |
| Goal checker | xy / yaw tolerance | 0.25 m / 6.3 rad (heading ignored) |
| Progress checker | `required_movement_radius`, `movement_time_allowance` | 0.15 m (set by the launch), 10 s |
| Collision monitor | polygon, action, look-ahead | footprint, `approach`, `max(0.4 s, 0.31 m / max_linear)`, `min_points: 6` |
| Planners | `GridBased` (NavFn A*), `Smac2D`, `SmacHybrid`, `ThetaStar` | see `RESULTS.md` |
| Controllers | `FollowPath` (DWB), `MPPI`, `RPP`, `Graceful` | same document |
| RPP slowdown | `regulated_linear_scaling_min_radius` / `_min_speed` | 0.9 m / 0.1 m/s |

The command path is: controller → `/cmd_vel_nav` → velocity smoother → `/cmd_vel_smoothed` → collision monitor → `/cmd_vel` → simulator. The recorder taps all three stages, which is how the lock-up was found.

---

## 5. How the measurement was done

`tools/halt_analysis.py` records, in simulation time, the robot pose (headless `/sim/ground_truth`), the three command stages above, Nav2 goal status changes and every new global plan. `analyze` then splits the run, from the first goal to the last successful one, into:
- **moving** (speed at least 80 % of the 98th-percentile speed), **slow** (moving, below that) and **halted** (under 0.05 m/s for at least 0.3 s);
- halts split by cause: no active goal, goal just sent, controller commands nothing, controller rotating in place, smoother or collision monitor output zero;
- slow time split into braking before a goal stop, speeding up after one, controller commanding below 80 %, and "commanded full speed but not there yet";
- a separate line that finds the lock-up: controller output above 0.3 m/s while the final `/cmd_vel` is 0, over the whole recording and over its last 35 s.

```bash
export ROS_LOCALHOST_ONLY=1; source /opt/ros/jazzy/setup.bash; source ros2_ws/install/setup.bash
python3 tools/halt_analysis.py record /tmp/run &      # start first
ros2 launch terrain_nav_bringup bringup_maze_2d.launch.py simulator:=headless \
  set:=rviz=false,exploration.result_file=/tmp/run.json
python3 tools/halt_analysis.py analyze /tmp/run
```

Limits of the measurement:
- **Headless only.** The recorder needs the simulator's ground-truth pose. It has not been run on Unity (a `/tf` based version would be needed). Whether Unity's halts match is not verified; all three Unity runs at 0.5, 0.8 and 1.0 m/s reached the goal, so the lock-up was not seen there, but there were only three.
- The analyzer window ends at the last successful goal, so a stalled tail is only visible through the lock-up line, not the share table.
- Run `c` (DWB, 0.5 m/s) and `b`, `a` are single runs. Their shares differ a lot from run to run (halted share was 19 % in `a`, 1 % in `b`).
- Angular commands were added to the recorder after run `a`; the 5.2 s halt in `a` at (2.35, -6.34) with the controller outputting no forward speed is therefore not classified (probably a turn toward a frontier behind the robot; not verified).
- I inspected the stall costmap for one stalled run (`fast3`: cost 99 under the robot, a frontier still available, an escape cell available). I did not inspect the other three.

---

## 6. What was already fixed (before this review)

| Problem | Fix | Effect |
| :-- | :-- | :-- |
| Pure-Python flood fill 0.15–0.4 s per cycle | scipy shortest paths (identical results, checked) | about 0.02 s per cycle |
| 1 s explorer loop, then a wait after every frontier | 0.25 s loop plus an immediate re-plan on arrival | goal-to-first-movement 0.03–0.26 s |
| Lidar range 3.5 m, so long corridors showed no scan | 10 m, 360 beams, costmaps and SLAM matched | Unity 163 s to 72.7 s together with the items above (not separated) |
| slam_toolbox rejected scans (`360 readings, expected 361`) | exact `angle_increment` | map builds |
| Foreign ROS nodes (UR arm sim on the LAN) corrupted `/clock` and `/tf` | `ROS_LOCALHOST_ONLY=1` in the launch | zero "jump back in time" warnings since |
| Unity publishers dead after a script recompile in Play mode | procedure: stop Play, compile, Play, then launch | see `USER_GUIDE.md` Stage 8b |

---

## 7. Recommended fixes, in order of expected gain (none implemented, none tested)

1. **Detect and break the lock-up (cause A).** Highest value. Options, cheapest first:
   - In `goal_explorer`, watch the robot's progress while a goal is active. If it moved under about 0.1 m in 3–4 s, cancel the goal, blacklist the target and send the escape goal (or a short backup). Today it waits 10 s for Nav2 and then 40 s for the stall stop.
   - Lower `movement_time_allowance` from 10 s to about 4 s so Nav2 gives up sooner.
   - Let the escape logic also trigger when a frontier exists but the robot's own cell is at cost 99 (today it only triggers when no frontier is found).
2. **Stop paths from touching walls (the cause of A).** Raise `clearance` so frontier goals sit away from the inscribed band, and make planners treat cost 99 as an obstacle or raise `cost_travel_multiplier` / inflation `cost_scaling_factor`. Check the collision monitor's `time_before_collision` and `min_points` as well, because at 1.0 m/s the stop zone is only 0.4 m.
3. **RPP slowdowns (cause B).** Reduce `regulated_linear_scaling_min_radius` (0.9 m is wider than most maze turns) and raise `regulated_linear_scaling_min_speed` (0.1 m/s). Re-run and watch the "controller commanding below 80 %" share (29 % now).
4. **Avoid stopping at every frontier (cause C).** Send the next frontier before the robot arrives (the explorer already replaces goals mid-motion; most goals show as replaced while moving). Check RPP's approach slow-down distance, which slows the robot near the end of every path.
5. **Faster map updates.** `map_update_interval` and the global costmap `update_frequency` / `publish_frequency` are 1 Hz; raising them to 4–5 Hz cuts the age of the explorer's data (reasoned, not measured).

After each change re-run `tools/halt_analysis.py` several times (the run-to-run spread is large) and judge by the lock-up line and the time shares, not by one finish time.

---

## 8. Known open issues

- The `Failed to make progress` aborts in the planner/controller benchmark stalls (`RESULTS.md`) are very likely the same lock-up; not checked for each trial.
- DWB stalling in 4 of 8 trials and MPPI in 2 of 8 may also be this lock-up or something separate; not checked.
- The R reset in Unity has never been exercised.
- Unity timer: the goal radius was widened to 0.4 m after the timer missed one goal; confirmed working in the one Unity run since.
- Unity is only run for single trials; no repeat statistics exist for Unity.
- Also open: Play restarts reset `/clock` while ROS keeps running, and Nav2 start-up warnings.
