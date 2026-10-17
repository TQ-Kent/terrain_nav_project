# Planner and controller comparison (maze, headless simulator)

Run on 2026-10-12. Raw data: `docs/results/benchmark_combo.csv` (one row per trial). Script: `tools/benchmark.py --only combo`.
Code state: commit `ab8d54f` plus the new `combo` group in `tools/benchmark.py` and the `RunTracker` goal-radius change.

## 1. What was tested

All 16 pairs of 4 global planners and 4 local controllers, 2 trials each (32 trials).

| Setting | Value |
| :-- | :-- |
| Test bed | built-in headless simulator (`simulator:=headless`), not Unity. Unity was stopped. |
| Maze, goal | seeded maze (seed 42), start (0, 0), goal (10, -10) m, map unknown at the start |
| Exploration | goal-biased frontier exploration, 0.25 s loop, `localization: ground_truth` |
| Speed | `max_linear` 1.0 m/s, `max_angular` 2.0 rad/s, `linear_accel` 2.0 m/s^2 |
| Lidar | 360 beams, 10 m range |
| Trial ends | goal reached (within 0.25 m), `STALLED` (moved less than 0.15 m in 40 s) or 300 s |
| Metrics | simulated time from the first goal sent, path length driven, outcome |

Before this run three things that had corrupted earlier benchmarks were fixed: a foreign ROS system on the LAN (a UR arm Gazebo sim) disturbing `/clock` and `/tf` (the launch now sets `ROS_LOCALHOST_ONLY=1`), the lidar range of 3.5 m, and a lidar scan-size mismatch that SLAM rejected. All 32 trials ran with no clock jumps, TF errors or scan errors. Earlier planner and controller tables (`benchmark.csv`) were taken before those fixes and should not be used.

Reproduce (Unity must not be in Play mode):
```bash
source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash
python3 tools/benchmark.py --only combo --out docs/results/benchmark_combo.csv
```

## 2. Results

Each cell: trials reached out of 2, mean time and mean path of the trials that reached the goal.

| Planner \ Controller | FollowPath | MPPI | RPP | Graceful |
| :-- | :-- | :-- | :-- | :-- |
| GridBased | 0/2, - | 1/2, 82 s, 35.6 m | 2/2, 44 s, 32.2 m | 2/2, 58 s, 36.1 m |
| Smac2D | 1/2, 52 s, 35.0 m | 1/2, 92 s, 37.5 m | 2/2, 42 s, 32.7 m | 2/2, 60 s, 36.4 m |
| SmacHybrid | 2/2, 48 s, 35.7 m | 2/2, 84 s, 34.5 m | 2/2, 57 s, 35.8 m | 2/2, 82 s, 38.8 m |
| ThetaStar | 1/2, 52 s, 33.9 m | 2/2, 66 s, 34.9 m | 2/2, 51 s, 35.5 m | 0/2, - |

By controller (8 trials each) and by planner (8 trials each). Time and path are means over the trials that reached the goal only.

| | Reached | Mean time (s) | Mean path (m) |
| :-- | :-- | --: | --: |
| FollowPath | 4/8 | 50.1 | 35.1 |
| MPPI | 6/8 | 79.0 | 35.3 |
| RPP | 8/8 | 48.6 | 34.0 |
| Graceful | 6/8 | 66.6 | 37.1 |

| | Reached | Mean time (s) | Mean path (m) |
| :-- | :-- | --: | --: |
| GridBased | 5/8 | 57.2 | 34.5 |
| Smac2D | 6/8 | 58.0 | 35.1 |
| SmacHybrid | 8/8 | 67.9 | 36.2 |
| ThetaStar | 5/8 | 57.1 | 35.0 |

### Ranking: lowest time and lowest distance together

Score = mean of (time / best time) and (path / best path), where the best time (42.2 s) and best path (32.2 m) are the lowest means among all pairs. 1.000 would be best on both; lower is better. Pairs that reached 2 of 2 are listed first; pairs that reached 1 of 2 are not comparable, because their averages come from a single run.

| # | Pair | Reached | Time (s) | Path (m) | Score |
| --: | :-- | :-- | --: | --: | --: |
| 1 | Smac2D+RPP | 2/2 | 42.2 | 32.7 | 1.007 |
| 2 | GridBased+RPP | 2/2 | 44.3 | 32.2 | 1.025 |
| 3 | SmacHybrid+FollowPath | 2/2 | 48.5 | 35.7 | 1.129 |
| 4 | ThetaStar+RPP | 2/2 | 51.0 | 35.5 | 1.157 |
| 5 | SmacHybrid+RPP | 2/2 | 56.9 | 35.8 | 1.230 |
| 6 | GridBased+Graceful | 2/2 | 57.7 | 36.1 | 1.245 |
| 7 | Smac2D+Graceful | 2/2 | 59.8 | 36.4 | 1.274 |
| 8 | ThetaStar+MPPI | 2/2 | 65.8 | 34.9 | 1.323 |
| 9 | SmacHybrid+MPPI | 2/2 | 84.0 | 34.5 | 1.532 |
| 10 | SmacHybrid+Graceful | 2/2 | 82.4 | 38.8 | 1.581 |
| 11 | ThetaStar+FollowPath | 1/2 | 51.8 | 33.9 | 1.140 |
| 12 | Smac2D+FollowPath | 1/2 | 51.8 | 35.0 | 1.158 |
| 13 | GridBased+MPPI | 1/2 | 82.0 | 35.6 | 1.526 |
| 14 | Smac2D+MPPI | 1/2 | 92.2 | 37.5 | 1.676 |

Pairs with 0 of 2 reached: GridBased+FollowPath, ThetaStar+Graceful.

### Reading it

- **Best overall: RPP.** It reached the goal in 8 of 8 trials, with the lowest mean time (48.6 s) and the lowest mean path (34.0 m) of the four controllers. `Smac2D+RPP` (42.2 s, 32.7 m) and `GridBased+RPP` (44.3 s, 32.2 m) are the two best pairs. The difference between those two is within what two trials can show, so do not rank one above the other.
- **Path length barely depends on the pair.** Means were 32–39 m for every pair that reached the goal; the maze and the explorer decide most of the route. Time differs far more, from 42 s to 92 s.
- **MPPI is the slowest** (79 s mean) and stalled in 2 of 8 trials. **Graceful** is reliable only with some planners (6 of 8) and drives the longest path (37.1 m). **DWB (`FollowPath`)** is fast when it works (50 s) but stalled in 4 of 8 trials.
- **SmacHybrid was the most reliable planner** (8 of 8) but the slowest (67.9 s mean), probably because its kinematically feasible paths are longer and slower to follow. This was not measured separately.
- **Stalls:** in the six stalled trials I checked, the controller logged 3 to 4 `Failed to make progress` aborts before the explorer's 40 s stall stop. I did not find the root cause. They are very likely the wall lock-up described in `SYSTEM_REVIEW.md`.

## 3. Limits of this comparison

- **Two trials per pair on one maze and one goal.** A pair reaching 2 of 2 is not a measured success rate. Treat the ranking as a ranking, not a probability.
- **Headless simulator, not Unity:** no wheel slip, no physics contacts, no lidar noise, and a simple acceleration model. Unity results with `GridBased+FollowPath` (72.7 s / 34.7 m at 0.5 m/s, 56.0 s / 41.1 m at 1.0 m/s) and `GridBased+MPPI` (70.4 s / 33.6 m at 0.5 m/s) are single runs and are not directly comparable. No other pair has been run in Unity.
- **One speed.** At 1.0 m/s the ranking may differ from the ranking at 0.5 m/s.
- **Default tuning only.** Each algorithm uses the parameters in `nav2_params_maze.yaml`, not tuned per algorithm. A badly tuned controller looks worse than it is.

## 4. The planners

A global planner draws a path from the robot to the goal on the global costmap. In this test it only plans through space the robot has already seen (`allow_unknown: false`), and `goal_explorer` hands it frontier goals.

| Selector name | Algorithm (Nav2 plugin) | How it plans | Settings used here | Trade-offs |
| :-- | :-- | :-- | :-- | :-- |
| `GridBased` | NavFn (`nav2_navfn_planner`) | A* on the costmap grid (`use_astar: true`; without it, Dijkstra). Cost of each cell comes from the costmap. | `tolerance: 0.5` (may end up to 0.5 m from the goal), `allow_unknown: false` | Fast and simple. Paths follow the grid, so they can be jagged, and it knows nothing about the robot's turning limits. |
| `Smac2D` | `nav2_smac_planner::SmacPlanner2D` | A* on the costmap grid with 8 neighbours, plus a path smoother. | `tolerance: 0.25`, `max_planning_time: 2.0`, `cost_travel_multiplier: 2.0` (a stronger pull away from obstacles), smoother `w_smooth: 0.3` | Smoother paths than NavFn at a similar cost. Still ignores the robot's turning limits. |
| `SmacHybrid` | `nav2_smac_planner::SmacPlannerHybrid` | Hybrid-A*: searches over (x, y, heading), so the path obeys a minimum turning radius. Dubins motion model (forward only). | `motion_model_for_search: DUBIN`, `angle_quantization_bins: 72`, `minimum_turning_radius: 0.15`, `cost_penalty: 2.0`, `max_planning_time: 3.0`, `smooth_path: true`, `downsample_costmap: false` | Paths the robot can actually follow, so a controller tracks them well (8 of 8 reached). Slower to compute and the paths are longer. Forward-only paths in tight corridors can be awkward. |
| `ThetaStar` | `nav2_theta_star_planner` | Any-angle A*: checks line of sight between nodes, so paths are not limited to 8 directions. | defaults from `nav2_params_maze.yaml` | Short, straight paths. In general, straight segments can pass close to walls and corners; whether that caused the `ThetaStar+Graceful` stalls here was not checked. |

## 5. The controllers

A local controller turns the global path and the nearby (local) costmap into velocity commands. Speed limits from the launch (`speed.max_linear` and the others) are written into all four.

| Selector name | Algorithm (Nav2 plugin) | How it drives | Settings used here | Trade-offs |
| :-- | :-- | :-- | :-- | :-- |
| `FollowPath` | DWB (`dwb_core::DWBLocalPlanner`) | Samples velocity pairs (20 forward x 20 turn), simulates each for 1.7 s, scores them with critics and takes the best. | `vx_samples: 20`, `vtheta_samples: 20`, `sim_time: 1.7`; critics RotateToGoal, Oscillation, BaseObstacle, GoalAlign, PathAlign, PathDist, GoalDist | Light on CPU and fast when it works (50 s mean). It can find no acceptable trajectory near walls and report failure, and it stalled in 4 of 8 trials here. |
| `MPPI` | Model Predictive Path Integral (`nav2_mppi_controller`) | Samples 2000 random trajectories of 28 steps (0.1 s each, a 2.8 s horizon), weights them by cost and blends them into one command. | `time_steps: 28`, `model_dt: 0.1`, `batch_size: 2000`; critics Constraint, Goal, GoalAngle, PathAlign, PathFollow, PathAngle, PreferForward, Cost | Handles tight, cluttered spaces well in theory and gives smooth motion. The most CPU-heavy, and it was the slowest here (79 s mean) with 2 stalls in 8. Needs tuning per robot. |
| `RPP` | Regulated Pure Pursuit (`nav2_regulated_pure_pursuit_controller`) | Steers toward a point on the path a lookahead distance ahead, and reduces speed in sharp curves and near obstacles ("regulated"). | `lookahead_dist: 0.6` (range 0.3–0.9), `lookahead_time: 1.5`, `use_regulated_linear_velocity_scaling: true`, `use_velocity_scaled_lookahead_dist: false` | Simple, cheap and predictable. Best results here (8 of 8, 48.6 s, 34.0 m). It follows the path closely, so a path that hugs a wall is followed into the wall. |
| `Graceful` | Graceful Motion (`nav2_graceful_controller`) | A smooth-approach control law that drives toward a target point on the path and curves into it, without sharp stops. | `motion_target_dist: 0.6`, `min_lookahead: 0.25`, `max_lookahead: 1.0`, `k_phi: 2.0`, `k_delta: 1.0`, `beta: 0.4`, `lambda: 2.0` | Smooth motion, but it drove the longest paths (37.1 m) and stalled twice with `ThetaStar`. |

## 6. Recommendation and next steps

- **Use RPP as the controller.** With `Smac2D` or `GridBased` it was best on both time and distance, and it reached the goal in every trial. If a feasible, smooth path matters more than time, pair it with `SmacHybrid`.
- **Avoid MPPI and `GridBased+FollowPath` in this maze until the stalls are understood.**
- **Before relying on this ranking:** repeat the top three pairs (`Smac2D+RPP`, `GridBased+RPP`, `SmacHybrid+FollowPath`) in Unity several times, and add trials at 0.5 m/s and on a second maze or goal.
- **Open issue:** the `Failed to make progress` aborts in stalled trials have no confirmed cause.
