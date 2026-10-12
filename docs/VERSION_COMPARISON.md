# checkpoint-2 vs checkpoint-3: what changed in exploration

Written 2026-10-11. checkpoint-2 = the earlier baseline (`bringup_baseline_2d.launch.py`, goals sent by hand from RViz).
checkpoint-3 = maze test (`bringup_maze_2d.launch.py`, goal given at launch, robot explores toward it).
"Measured" means I ran it (headless simulator, see section 4); "observed" means you saw it in Unity and told me.

## 1. The problem checkpoint-3 fixes

You observed in the maze with checkpoint-2: the robot walks back along already-known routes because the planner always takes the shortest path to the goal, even when the correct way first leads away from the goal.

Cause (code, not guesswork): checkpoint-2 planned in a fixed 40 x 40 m global costmap with **no static layer**, and all planners had `allow_unknown: true`. Unexplored space therefore counted as free, so the shortest path to the goal ran straight through walls the robot had not seen yet. Each time the lidar revealed a wall the path was replanned, usually back out of the dead end it was in, then into the next dead end. A shortest-path planner with no memory of "this branch is closed" does this in any maze. No choice among the four planners or four controllers changes it.

## 2. Side by side

| | checkpoint-2 | checkpoint-3 |
| :-- | :-- | :-- |
| Who sends goals | You, in RViz, one at a time | `goal_explorer` node; the final goal comes from the launch / config |
| Goal knowledge | Goal must be on the map you drive to | Goal location known at launch, map unknown |
| Planning space | Fixed 40 x 40 m costmap, unknown = free (`allow_unknown: true`) | SLAM map as static layer, unknown = not traversable (`allow_unknown: false`); `nav2_params_maze.yaml` |
| How the robot decides where to go | Planner draws the shortest line to the goal through unknown space | Frontier exploration (below); the goal is only sent once reachable through known space |
| Dead ends | Re-entered repeatedly (planner never learns a branch is closed) | A fully seen dead end has no frontier left, so it is never chosen again |
| Localization | `slam_toolbox` owns `map->odom` | Default `ground_truth`: static identity `map->odom`, `slam_toolbox` only maps (`slam_toolbox_2d_gt.yaml`); `slam` still selectable |
| Stopping | Manual | Stops itself on goal reached, robot stalled (`stall_timeout`) or `max_time` |
| Settings | Edited in several YAML files | One file: `terrain_nav_bringup/config/maze_test.yaml` (+ `set:=` overrides) |
| Test bed | Unity only | Unity, or headless simulator (`simulator:=headless`) |

## 3. How the new exploration works (`frontier_logic.py`, `goal_explorer.py`)

1. Read the **global costmap** (what the planner will see: inflation, lidar marks). Reading `/map` instead produced goals the planner rejected, because the costmap lags SLAM by about a second and inflates differently.
2. Cells the robot can reach through known free space (BFS from the robot; cells at lethal cost are blocked) are "reachable".
3. A **frontier** is a reachable cell next to unknown space. Clusters smaller than `min_frontier_cells` are ignored.
4. Choose the frontier minimising `path_length(robot -> frontier) + goal_weight * straight_distance(frontier -> goal)`. `goal_weight` 0 gives nearest-frontier exploration; larger values pull toward the goal.
5. Send it to Nav2 `NavigateToPose`. Keep the current target unless a new one is better by `switch_margin`. A target that fails is blacklisted for `blacklist_seconds`.
6. As soon as the goal cell is reachable through known space, send the goal itself.

Why it does not loop: exploring a branch removes its frontiers permanently (known space only grows), so there is nothing left to attract the robot back, and the goal term keeps it from wandering to unrelated parts of the maze.

## 4. Measured comparison (headless simulator, same maze, same goal, old speed 0.26 m/s)

Single run each; the simulator is deterministic apart from Nav2 timing, but one run is not a statistic. Goal (10, -10) m from the start. Test ends on goal, `STALLED` (no 0.15 m move in 40 s) or 300 s.

| Version | Localization | Outcome | Sim time (s) | Path driven (m) | Avg speed (m/s) |
| :-- | :-- | :-- | --: | --: | --: |
| checkpoint-2 behaviour (direct goal, unknown = free) | slam_toolbox | **stalled** | 154 | 21.6 | 0.14 |
| checkpoint-2 behaviour | ground truth | **timeout, goal not reached** | 300 | 50.8 | 0.17 |
| checkpoint-3 exploration | slam_toolbox | **stalled** | 105 | 14.8 | 0.14 |
| checkpoint-3 exploration | ground truth | **reached** | 157 | 36.0 | 0.23 |

Reading it:
- The checkpoint-2 behaviour never reached the goal. With perfect localization it drove 50.8 m in 300 s (the straight-line distance is 14.1 m and the shortest route is about 35 m, and it did not get through; I did not record its path shape, so the loop itself is not shown here) and was still going, consistent with what you saw in Unity. Its Nav2 log shows repeated `Failed to make progress` aborts.
- checkpoint-3 with ground-truth localization reached the goal. It drove 36.0 m; the shortest route through the maze, computed on the layout with 0.3 m wall clearance, is about 35 m (4-connected grid, so an upper bound). The goal-biased exploration therefore wasted very little distance in this maze. One maze and one goal only, so this may not generalise.
- checkpoint-3 with `slam_toolbox` still stalled here, in a simulator with noise-free scans. The log shows DWB `No valid trajectories out of 419` next to a wall. That is the known stall near obstacles (PROJECT_STATUS.md), not an exploration failure. In Unity the same mode also showed `map->odom` jumping 1.82 m in 1 s, which is why ground truth is the default.
- Not measured: how often each version succeeds (needs many trials with randomised start poses or mazes) and Unity numbers for checkpoint-3 beyond the one successful run you saw in RViz.

## 5. What did not change
Planners, controllers, costmap inflation, tolerances (xy 0.25 m), lidar settings and the Unity robot are identical, apart from the speed work in `SPEED_REPORT.md` (checkpoint-3 + later).

## 6. Limits of the new approach
- It needs the goal location in advance; it is not general exploration.
- With `localization: slam` it still fails in long corridors (scan matching), so results with SLAM are not comparable to ground truth.
- It uses frontiers found from a 2D costmap; it has no notion of a path being "probably dead end".
- The headless simulator is not Unity: no wheel slip, no physics contacts, no lidar noise, and a simple acceleration model.
