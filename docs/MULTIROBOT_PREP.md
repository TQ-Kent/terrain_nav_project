# Starting point for the multi-robot upgrade

State on 2026-10-26: everything is single-robot. Verified by grep, not yet by a multi-robot run.

## Single-robot assumptions to remove
- Global topic and frame names (`scan`, `odom`, `cmd_vel*`, `map`, `base_link`) are used unprefixed in `headless_sim.py` (7 places),
  `goal_explorer.py` (2), `speed_governor_node.cpp` (7) and `governor_math.cpp` (4), and in the Nav2 YAML (`nav2_params_*.yaml`).
  Needs a per-robot namespace plus TF prefix (`robot1/base_link`) and remapped `/tf`.
- `bringup_maze_2d.launch.py` starts one Nav2 stack, one SLAM, one explorer and one simulator. It also sets `ROS_LOCALHOST_ONLY=1`
  (keep it; a foreign ROS system on the LAN corrupted earlier runs).
- `headless_sim.py` simulates one robot; `RunTracker` and `benchmark.py` time one goal.
- `goal_explorer` picks frontiers for one robot; with several robots it needs shared frontier claims (task allocation).
- Mapping: `slam_toolbox` is one instance. Options are one SLAM per robot plus map merge, or a shared map.
- Unity: one robot prefab in `BuildMazeScene.cs`; the TCP endpoint carries one robot.
- Governor: per-robot instance works unchanged once names are namespaced; it does not yet treat other robots as obstacles.

## Open single-robot issues to carry over (do not lose)
Start-cell wall lock from the Collision Monitor, RPP `collision ahead` silences, time spread 35-61 s on one maze:
`SESSION_2026-10-19.md` sections 3 and 9, and `GOVERNOR.md` "Wall-lock failure". Profile `reliable` is the best known setup.

## Suggested order
1. Namespace one robot end to end (launch with `PushRosNamespace`, nav2 `namespace`/`use_namespace`, TF remap) and check parity with today's benchmark.
2. Headless sim with N robots and robot-robot collisions.
3. Frontier allocation across robots; then map merging; then Unity.
