# Algorithm Comparison Plan: 2D SLAM + Nav2 (Unity 6, ROS 2 Jazzy)

Written 2026-10-10 on top of `checkpoint-2`. Everything here is a plan unless it says **verified**.
"Verified" means I saw it run this session; "reported" means you told me it worked and I did not observe it.

## 1. Algorithms available

Checked against `/opt/ros/jazzy/share` and `ros2_ws/src/terrain_navigation/config/nav2_params_2d.yaml`.

### Global planners (switch with `set_algo --planner`)

| ID | Plugin | Idea | Configured | Evidence so far |
| :-- | :-- | :-- | :-: | :-- |
| `GridBased` | `nav2_navfn_planner::NavfnPlanner` | Dijkstra / A* on the grid; fast, paths hug obstacles | yes (default) | **verified** in Unity goals and in the headless simulator |
| `Smac2D` | `nav2_smac_planner::SmacPlanner2D` | A* with smoother paths and cost-aware search | yes | reported (stage 7), 1 m goals only |
| `SmacHybrid` | `nav2_smac_planner::SmacPlannerHybrid` | Hybrid-A*, kinematically feasible (Dubins) paths | yes | reported (stage 7), 1 m goals only |
| `ThetaStar` | `nav2_theta_star_planner::ThetaStarPlanner` | any-angle A*, straighter paths | yes | reported (stage 7), 1 m goals only |
| `SmacLattice` | `nav2_smac_planner::SmacPlannerLattice` | state lattice with motion primitives | **no** | needs a `lattice_filepath` primitives file we do not have |

### Local controllers (switch with `set_algo --controller`)

| ID | Plugin | Idea | Configured | Evidence so far |
| :-- | :-- | :-- | :-: | :-- |
| `FollowPath` | `dwb_core::DWBLocalPlanner` | samples velocities, scores trajectories with critics | yes (default) | **verified**; stalls near obstacles in the Unity run (see section 6) |
| `MPPI` | `nav2_mppi_controller::MPPIController` | sampling-based model-predictive control | yes | reported (stage 7) |
| `RPP` | `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` | follows the path with a lookahead point and slows near obstacles | yes | reported; earlier session logged "Failed to make progress" |
| `Graceful` | `nav2_graceful_controller::GracefulController` | smooth pose-to-pose motion law | yes | reported (stage 7) |
| `RotationShim` | `nav2_rotation_shim_controller::RotationShimController` | wrapper: rotates toward the path first, then hands over to another controller | **no** | installed only |

### Other pieces
- **Smoothers installed:** `SimpleSmoother` (the only one configured), `SavitzkyGolaySmoother`, `ConstrainedSmoother`. I have **not** checked whether the default behavior tree actually runs the smoother, so do not assume it affects paths.
- **SLAM / localization:** only `slam_toolbox` is installed (Cartographer is not). `nav2_amcl` and `nav2_map_server` are installed, so localization on a saved map is possible.
- **Not installed:** TEB and Vector Pursuit controllers. Adding them means installing a package, so they are out of scope unless you want them.

## 2. What is wrong with the earlier comparison

The earlier 7-pair table (`TESTING_GUIDE.md`) is only indicative:
- start poses differed between runs, so times and path lengths are not comparable;
- one entry (about 0 s) was invalid because the robot was already at the goal;
- one trial per pair; goals were 1 m in open space, which does not separate planners;
- RPP and DWB stalls may come from shared parameters (progress checker, inflation), not the algorithm;
- the Unity scene has only four cylinders, so planner differences barely show.

## 3. Test design

### 3.1 Controlled variables (kept identical for every run)
- same robot, speed limits, costmap and inflation settings, tolerances (`xy 0.25 m`, `yaw 0.25 rad`);
- same start pose, reset before each trial;
- same goal list per scenario, same costmap state at start (clear costmaps before each trial);
- same lidar settings (180 beams, 3.5 m, `+inf` misses);
- one run changes only the algorithm under test.

### 3.2 Localization mode
Run two sets, because SLAM adds noise that can hide algorithm differences:
- **A. Ground-truth localization** (Unity publishes `map->odom` as identity, SLAM off): isolates planner and controller. This is the primary comparison.
- **B. With `slam_toolbox`:** repeat only the top candidates from A to check they still hold up with a growing map.

### 3.3 Scenarios (add to Unity and to the headless simulator)
| ID | Layout | What it separates |
| :-- | :-- | :-- |
| S1 | open floor, 4 cylinders (**exists**) | baseline, long straight goals |
| S2 | corridor, 1.0 m wide | path-hugging, oscillation |
| S3 | doorway / 0.7 m gap | narrow-gap handling, inflation settings |
| S4 | U-shaped dead end with goal behind it | planner global reasoning, replanning, recoveries |
| S5 | cluttered field (about 15 obstacles) | local controller quality, compute cost |

Each scenario gets 3 fixed start/goal pairs, 10 trials each.

### 3.4 Metrics
Logged per trial to CSV: success (goal reached within tolerance, no abort), time to goal, actual path length (integrated from `map->base_footprint` TF), planned path length (first `/plan`), minimum clearance to obstacles, number of recoveries / aborts (BT log), planning time (`ComputePathToPose` result), peak and mean controller loop rate, and CPU use of `planner_server` and `controller_server`.
Report median and inter-quartile range per cell, and success rate with a confidence interval. Ten trials per cell is the minimum for success rates to mean anything.

### 3.5 Matrix size
Full factorial: 4 planners x 4 controllers x 5 scenarios x 3 goals x 10 trials = 2,400 runs. Too many for Unity at real time, so use stages:

1. **Planner sweep:** 4 planners (plus `SmacLattice` if primitives are generated) x 1 fixed controller, 5 scenarios.
2. **Controller sweep:** 1 fixed planner x 4 controllers, 5 scenarios.
3. **Cross matrix** of the best 2 planners x best 2 controllers.
4. **Tuning check:** repeat the winners with one tuned parameter set each, so a default-parameter weakness is not mistaken for an algorithm weakness.

## 4. Tooling to build

1. **Headless simulator** (a hardened version of the stand-in used this session at `.../scratchpad/fake_unity.py`): `/clock`, TF, `/scan`, diff-drive with collision, all scenarios as data, a speed-up factor, and a reset service. Runs without Unity, so sweeps can run unattended.
2. **`benchmark_runner.py`** (new ROS 2 node): loops over (planner, controller, scenario, trial); resets the robot; clears costmaps; calls `set_algo`; sends the goal; records metrics; enforces a timeout; writes CSV.
3. **`analyze.py`:** tables and plots from the CSV.
4. **Unity reset hook:** a small script that teleports the robot to a start pose on a ROS message, needed for Unity runs.

## 5. Order of work

| Step | Task | Done when |
| :-- | :-- | :-- |
| 0 | Measure baselines: Unity startup, Nav2 activation, sim speed, goal times | numbers recorded in `PROJECT_STATUS.md` |
| 1 | Build headless simulator + scenarios S1-S5 | all five scenarios load; robot drives and collides |
| 2 | Build `benchmark_runner.py`; dry-run 2 trials | CSV with sane values |
| 3 | Fix the shared stall causes (inflation, progress checker) before comparing | DWB and RPP no longer stall in S1 |
| 4 | Planner sweep, then controller sweep (headless) | tables produced, reviewed |
| 5 | Confirm winners in Unity (mode A, then B) | results agree with headless within expected noise |
| 6 | Write findings | table + short conclusions |

## 6. Known issues that must be settled first
- **Stall near obstacles (observed in Unity):** the robot parked about 0.46 m from a cylinder face, inside the inflation zone (cost 66 at its centre, 99 about 0.15 m ahead). DWB then reported `No valid trajectories out of 419`. I did not find out why DWB finds nothing at that cost. This must be solved or all DWB numbers will measure the stall, not the algorithm.
- **Fixed-size global costmap:** the global costmap is a fixed 40 x 40 m arena with no static layer, so planners work on obstacles the lidar has marked, not on the SLAM map. Results from mode B differ from a static-layer setup.
- **Single-run evidence:** the old-vs-fixed comparison earlier this session was one run per case.
- **Lidar change in Unity:** you reported the Unity run succeeded after the lidar fix; I did not observe it, and the headless test only covers the ROS side.

## 7. Speed recommendations (running faster)

Facts measured or read from the config:
- Every speed limit is 0.26 m/s: DWB `max_vel_x` and `max_speed_xy`, MPPI `vx_max`, velocity smoother `max_velocity`; RPP `desired_linear_vel` is 0.22. Observed in the simulated runs: 17-26 s for 3-4 m goals, an average near 0.2 m/s (Unity: a 4 m goal took about 16 s in one recorded run).
- Nav2 reported active about 6 s after Unity connected; the launch applies the initial algorithm after a fixed 15 s timer.
- `/clock` is published by Unity and the stack runs on sim time, so slowing or speeding Unity moves ROS with it. `Clock.cs` already has a `UnityScaled` mode that uses `Time.timeScale`.
- Global costmap is 800 x 800 cells, published in full (`always_send_full_costmap: true`) once per second. Not yet measured as a cost, but it is the largest message in the system.
- SLAM `map_update_interval` is 1.0 s (was 5.0 s).

Recommendations, in priority order. None of these has been applied; the speed-up factors are untested.

| # | Change | Expected effect | Risk |
| :-: | :-- | :-- | :-- |
| 1 | Add a named "fast" profile: 0.5 m/s in DWB, MPPI, RPP and the velocity smoother together (all four must change, or the lowest one wins) | goals about 2x faster | results no longer match a real TurtleBot3; shorter stopping margin with a 3.5 m lidar |
| 2 | Run headless benchmarks with the simulator and a speed factor of 2-5 | sweeps finish in minutes | the simulator is not Unity physics |
| 3 | In Unity, set `Time.timeScale` to 2 and keep `/clock` on `UnityScaled` | everything runs 2x in sim time | planner and controller get half the real time per sim second; check that the 10 Hz loops still hold |
| 4 | Shrink the global costmap from 40 m to about 20 m, or stop sending it in full | less bandwidth and CPU | goals outside 20 m fail |
| 5 | Set `map_update_interval` to 0.5 s | map appears sooner in RViz | more SLAM CPU |
| 6 | Replace the fixed 15 s `set_algo` timer with a wait for the lifecycle nodes to be active | saves several seconds per launch | none |
| 7 | Use the built Linux player (`unity_project/Build`) with `-batchmode -nographics` for headless Unity runs; in the editor, enable Enter Play Mode options with domain reload off | faster start; no rendering cost | none expected; not yet tried |

Step 0 of section 5 measures Unity startup and the real effect of each change before any of this is applied.

## 8. Decisions needed from you
1. Do you want the headless simulator as the primary test bed (fast, repeatable) with Unity confirming the winners, or Unity only?
2. Is the "fast" profile (0.5 m/s) acceptable, or must results match the real TurtleBot3 limits (0.26 m/s)?
3. Should `SmacLattice` be added (needs generating a primitives file)?
4. Are TEB or Vector Pursuit worth installing?
