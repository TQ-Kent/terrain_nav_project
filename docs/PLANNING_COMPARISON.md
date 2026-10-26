# Planning time: this project vs other systems

Written 2026-10-19. Question: is replanning time what stalls the robot? Short answer from our own
measurement: no. Planning takes about 20-30 ms; the stalls come from "no valid path" results.

## Our measurement (headless, ground_truth, maze seed 42, Nav2 Jazzy)

`tools/time_planners.py` sends `ComputePathToPose` to each planner on the live global costmap while the
explorer runs (goal = a free cell 3-8 m from the robot). Raw data: `docs/results/planner_timing_headless.json`.
Times are action round trips in milliseconds, so they include ~20 ms of action/spin overhead and the
four planners are not separable at this map size.

| Planner | calls | median ms | p95 ms | max ms | no path |
|---|---|---|---|---|---|
| NavFn (GridBased) | 492 | 24.3 | 33.5 | 271.4 | 115 |
| Smac2D | 492 | 21.4 | 29.9 | 47.9 | 115 |
| ThetaStar | 492 | 21.3 | 29.7 | 46.0 | 115 |
| SmacHybrid | 491 | 21.2 | 29.2 | 46.0 | 116 |

About 19 % of calls found no path. The goal is any free cell, some of which are in pockets the robot
cannot reach, so this is an upper bound on the no-path rate, not the explorer's real rate.
`Planner loop missed its desired rate` appeared 0 times in this run, and `Costmap timed out` in 4 of 378 historic planner logs.
Not measured: Unity runs (the Unity bridge adds latency), `localization:=slam`, larger maps.

## Other systems

| System | Planner / map | Planning time | Stalls reported | Source |
|---|---|---|---|---|
| **This project** (headless) | NavFn, Smac2D, Theta*, Smac Hybrid; 12.5 m maze, 0.05 m | 21-24 ms median, 30-34 ms p95 | explorer stalls on no-path | measured here |
| Nav2 Smac README, one ~75 m path | 2D A* / Hybrid-A* / State Lattice / NavFn | 243 / 144 / 113 / 146 ms | n/a | [Smac README](https://github.com/ros-navigation/navigation2/blob/main/nav2_smac_planner/README.md) |
| Nav2 Smac README, ranges | 147,456 and 344,128 cell maps, 72 bins | 2-20 ms and 30-200 ms | n/a | same |
| Nav2 docs, Theta* | one 87.5 m path | about 46 ms | n/a | [Theta* docs](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/thetastar/configuring_thetastar/) |
| Macenski et al. 2024 (arXiv 2401.13078) | Smac feasible planners vs 2D A* and NavFn | 39-42 ms for the feasible planners; reported 50 % faster than cost-aware 2D A*, 38 % faster than NavFn | n/a | [paper](https://arxiv.org/pdf/2401.13078) (numbers from a search summary; the PDF text could not be read here) |
| Macenski, ROSCon 2022 | design figures | 2D A* 20-200 ms, Hybrid-A* 20-60 ms, State Lattice 30-65 ms | n/a | [slides](http://download.ros.org/downloads/roscon/2022/On%20Use%20of%20Nav2%20Smac%20Planners.pdf) |
| kavinaya2808, 2 TurtleBot3, Unity + Jazzy | NavFn A*, RPP, inflation 0.35 m | not reported | not reported; blacklists failed goals, `goal_timeout` 60 s | [repo](https://github.com/kavinaya2808/Autonomous-Multi-Robot-Slam-Mapping) |
| itsakhash, TurtleBot3 Gazebo | MPPI, frontier node | not reported | none; 3 runs 181-282 s, 0 failed goals | [repo](https://github.com/itsakhash/ros2-autonomous-explorer) |

Hardware differs between rows and is mostly not stated, so the rows are not directly comparable.

## What this says

- Our planning time is in the same range as the published Nav2 figures (tens of ms). It is not slow.
- No project found reports a planning-time stall. The two closest ones (Unity + Jazzy, TurtleBot3 + frontier)
  report no planning times, and both handle failed or timed-out goals by blacklisting them, which our
  explorer also does.
- The failure that does show up in our logs is `Failed to create plan`, a no-path result. Community reports
  of the same message on narrow passages point to footprint and inflation, not planner speed
  (e.g. [Husarion forum](https://community.husarion.com/t/navigating-narrow-passages-1-1-1-2m-with-husarion-ugv-autonomy-inflation-radius-floor-vs-tight-corridors/2228)):
  the circle `robot_radius` 0.22 and inflation 0.55 close gaps below about 0.44 m. Opt-in `costmap.footprint`
  and `costmap.inflation_radius` in `maze_test.yaml` address that; they are not validated on a tight-gap scene yet.
- If a stall still looks like slow replanning, the cost may be elsewhere: `Costmap timed out waiting for update`
  after a costmap clear or resize, the explorer's `stall_timeout`, or the controller's progress check. Those
  need a log of the stalled run to confirm.
