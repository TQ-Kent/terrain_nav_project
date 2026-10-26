# Adaptive speed governor (research prototype)

A C++ ROS 2 node (`terrain_speed_governor/speed_governor`) that lowers the forward speed of the
Nav2 command using path curvature, obstacle clearance, braking distance and perception
freshness. **It is a prototype. Nothing here shows that it is safe or that it is faster; see
[Evaluation](#evaluation) for what has actually been measured, and its limits.** It is not
claimed to be novel: speed limiting from curvature, clearance and stopping distance is standard
practice (Nav2's own `RegulatedPurePursuit` does the first two in a simpler form).

## Status

| Item | State |
|---|---|
| Node, parameters, diagnostics, launch integration | implemented |
| Curvature, clearance, braking, perception-confidence bounds | implemented |
| Unit tests (38 gtests), fault-injection test | implemented, passing (see [Testing](#testing)) |
| Benchmark A–D, true-clearance / stall / timing logging | implemented (headless only) |
| Unity | **not run** (no braking/reaction measurement, no benchmark) |
| `localization:=slam` evaluation | **not run** (all results use `ground_truth`) |
| Pre-smoother insertion, reverse-motion obstacle checks, rotating-in-place checks | not implemented |

## Command chain

```
controller_server ─► /cmd_vel_nav ─► velocity_smoother ─► /cmd_vel_smoothed
                                                               │
                      governor disabled ───────────────────────┤ (stock Nav2)
                      governor enabled:  speed_governor ─► /cmd_vel_governed
                                                               ▼
                                          collision_monitor ─► /cmd_vel ─► robot
```

Verified against the code: `navigation_launch.py` hard-codes the controller output
(`cmd_vel_nav`) and the smoother input, so the governor sits **after** the smoother, not before
it as TASK.md sketched. The only input remap needed is `collision_monitor.cmd_vel_in_topic`
(`cmd_vel_smoothed` → `cmd_vel_governed`), written by `maze_config.nav2_params(...,
governor_enabled=True)`. No Nav2 file is modified. The Collision Monitor is still the last stage
and its thresholds are unchanged. The governor only ever reduces the linear speed magnitude of
the command it receives, so it cannot push the robot through something the monitor would stop.

## Enable / disable

`config/maze_test.yaml` → `governor.enabled` (default `false`, which is the unmodified Nav2 chain).

```bash
ros2 launch terrain_nav_bringup bringup_maze_2d.launch.py simulator:=headless \
  set:=algorithms.planner=Smac2D,algorithms.controller=RPP,governor.enabled=true,speed.max_linear=0.8
```

`speed.max_linear`, `max_reverse`, `max_angular`, `linear_accel` are passed to the governor as
`v_max`, `max_reverse`, `omega_max`, `accel`/`decel`, so its cap always equals the controller cap.
Any other `governor.<key>` in the YAML (or `set:=governor.a_lat_max=0.8`) is a ROS parameter of
the node; defaults live in `terrain_speed_governor/config/governor.yaml`.

## Algorithm

All bounds are computed at 20 Hz (node clock, follows `/clock`) from the latest messages.
`v_in` is the smoothed command, `v_max` the speed cap.

**Curvature.** The global plan (`/received_global_plan`, the path the controller is actually
following) is resampled to `path_spacing` (0.1 m), smoothed with a `smooth_window`-point moving
average (endpoints fixed) and curvature is the unsigned Menger curvature of points `curvature_k`
samples apart: `κ = 2·|cross(b−a, c−a)| / (|ab|·|bc|·|ca|)`. Coincident points give 0 (no division
by ~0). For each sample `i` up to `curve_lookahead` ahead at arc distance `s_i`:

```
v_c,i = min( sqrt(a_lat_max / max(κ_i, κ_eps)),  omega_max / max(κ_i, κ_eps) )
v_curve = min_i sqrt( v_c,i² + 2·curve_preview_decel·s_i )         # may still be braking into a curve
```

Smoothing softens grid-like staircases; a 90° corner of 0.3 m radius is tested to stay above 1.5 1/m
(true 3.3 1/m), i.e. a genuine tight turn is reduced but not erased. If the robot is further than
`path_max_offset` from the plan, the curvature bound is skipped.

**Clearance.** From the local costmap (`/local_costmap/costmap`, odom frame). Cells with cost ≥
`occupied_threshold` (100 = lethal) and unknown cells (−1) are obstacles; cells outside the window
count as obstacles in the braking sweep. `clearance` = distance to the nearest obstacle cell
centre inside a ±`clearance_half_angle` (0.6 rad) cone and `clearance_range` (1 m) minus
`robot_radius` (0.22 m). Then
`v_clear = v_floor + (v_max − v_floor)·clamp((c − c_min)/(c_full − c_min), 0, 1)`.
The floor (`clearance_floor_speed`, 0.15 m/s) is deliberate: a first pilot without it froze the
robot beside a wall for 39 s (see [Evaluation](#evaluation)). Clearance only slows; the braking
bound handles obstacles that are actually in the way.

**Braking.** `d_free` is the distance the circular footprint can travel along the commanded arc
(`curvature = ω/v`) before it overlaps an obstacle cell (obstacle cells already inside the radius
and not ahead of the robot are ignored: grazing). Then, from
`d_stop = v·t_react + v²/(2·a_brake) + margin`:

```
v_brake = −a·t_r + sqrt( (a·t_r)² + 2·a·max(d_free − margin, 0) )
```

Defaults `reaction_time = 0.30 s`, `braking_decel = 1.5 m/s²`, `safety_margin = 0.05 m` come from a
step test in the headless simulator (`tools/measure_braking.py`, results in
`docs/results/braking_headless.json`): effective decel ≈ 2.0 m/s² (the simulator's own limit),
reaction ≤ 0.22 s at the 0.1 s resolution of `/sim/ground_truth`. The defaults are 75 % of the
deceleration and a rounded-up delay. **These numbers are for the idealised headless simulator
(no slip, no latency jitter). The Unity robot has not been measured.**

**Perception confidence.** Inputs: scan age, local-costmap age, TF availability/age
(`costmap frame → base_link`), scan validity (≥ `min_valid_scan_fraction` usable beams; NaN or out
of range is unusable, +inf is free), costmap validity, and the fraction of forward-cone scan
beams that are usable. Hard faults — scan older than `scan_timeout` (1 s), costmap older than
`costmap_timeout` (3 s), never received, invalid scan/costmap, TF failure — command **zero** (with
`emergency_decel`) and stay at zero until inputs have been fault-free for `recovery_cycles`
(10 ticks = 0.5 s). Below the hard limits, confidence scales `v_max` linearly between the `*_warn`
and `*_timeout` ages and with forward coverage (0 at ≤ 50 % usable beams, 1 at ≥ 90 %).
*Limitations:* the local costmap in this project does not track unknown space, so "observed" is
judged from scan beams only; stale-data timeouts are heuristics, not derived from sensor models.

**Combination.** `v_target = min(|v_in|, v_max (or max_reverse), v_curve, v_clear, v_brake,
v_conf)`, then a creep floor `min_speed` (0.05 m/s) when only curvature or clearance limits the
speed and there is room, then a rate limit (`accel`, `decel`, `emergency_decel` for braking /
confidence / fault reductions), and finally `v_out ≤ |v_in|`. The binding bound is reported as
`reason`. Differences from the TASK.md formulation: the clearance floor, the preview term, the
creep floor and hysteresis on faults.

**Steering.** Angular velocity is scaled by `v_out/|v_in|` to keep the commanded arc, but never
below `omega_min_ratio` (0.5) of the request, so a slowed robot can still turn away; a circular
robot rotates safely in place. In a fault state both are zero. Reverse motion is limited by
`max_reverse`, ramped and fault-stopped, but **not** checked for obstacles behind the robot.

## Topics

| | topic | type |
|---|---|---|
| in | `cmd_vel_smoothed` | `geometry_msgs/Twist` |
| in | `received_global_plan` | `nav_msgs/Path` |
| in | `local_costmap/costmap` | `nav_msgs/OccupancyGrid` |
| in | `scan` | `sensor_msgs/LaserScan` |
| in | `cmd_vel` (final command, diagnostics only) | `geometry_msgs/Twist` |
| in | TF `base_link` in the costmap frame and plan frame | |
| out | `cmd_vel_governed` | `geometry_msgs/Twist` |
| out | `speed_governor/status` | `std_msgs/String` (JSON, 20 Hz) |

The status JSON has the requested/governed/final speeds, each bound (`v_curve`, `v_clear`,
`v_brake`, `v_conf`), the binding `reason`, `fault`, `clearance`, `d_free`, `kappa`, scan and
costmap age, `compute_us` and a `stall_hint`. `stall_hint` is diagnostic only (it changes no
behaviour): `monitor_zeroed_governed_cmd` (the governor passed motion, the final command is ~0),
`governor_limit`, or `perception_fault`. Startup/shutdown: no output until a fresh command and all
inputs exist; with no fresh command (`cmd_timeout` 0.5 s) it publishes nothing and resets its
speed state to zero.

## Wall-lock failure

The symptom (controller requests ≈ 0.6-0.8 m/s, final `/cmd_vel` = 0, robot at rest against a wall)
is now confirmed in the per-sample logs of the 0.8 m/s re-run (`governor_logs.tar.gz`, `logs_gov/cap0.8/*_samples.csv`).
In `D_MPPI_gov` seed 102 trial 1 the governor requested 0.45 m/s with 1.7 m free ahead and
0.15 m clearance, and `stall_hint = monitor_zeroed_governed_cmd` held for the rest of the trial:
the Collision Monitor's `FootprintApproach` zeroes the whole twist, so the robot cannot rotate out
either. The same lock appears with the governor off (`A_RPP`, `C_MPPI`, same spot near the start,
(0.55, -0.64)). The robot enters it by leaving the start cell at the 0.8 m/s cap and meeting a wall
that the scan only reveals during the first turn (free distance 2.0 m to 0.3 m within 0.2 s), so
the governor brakes too late there. No safety threshold or Collision Monitor setting was changed,
and the governor does not fix this failure. Possible next steps (not done): cap speed until the
first scan sweep has mapped the start cell; make the explorer/BT back up when `stall_hint` holds. The first was tried (startup cap, see
Results) and did not prevent the lock.

## Testing

```bash
cd ros2_ws && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select terrain_speed_governor terrain_nav_bringup terrain_navigation
colcon test --packages-select terrain_speed_governor && colcon test-result --verbose   # 38 gtests
source install/setup.bash
ROS_DOMAIN_ID=77 python3 src/terrain_speed_governor/test/fault_injection_test.py        # node-level faults
```

Unit tests cover curvature (line, circle, staircase vs genuine corner, degenerate paths),
curve-speed formula, clearance (empty, wall ahead/behind, unknown cells, corridor widths, invalid
grid), swept free distance (distances, arcs, grazing, window edge), braking inversion and edge
cases, confidence faults, and the rate limiter. The fault-injection test runs the real node with
synthetic inputs: healthy pass-through, wall ahead, stale/invalid scan, lost TF, stale costmap,
hysteresis and recovery, request never exceeded.

`freeDistance` returns the last swept step that was still free (found 2026-10-26: it used to return
the first colliding step, up to one half-cell step too optimistic, which the fault-injection test caught).
Not covered: `colcon test` does not run the fault-injection script; there is no automated
whole-stack regression test; Unity was not run.

## Benchmark

```bash
source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash
python3 tools/gen_maze.py --seed 7 --out /tmp/maze7.txt                 # headless-only seeded mazes
python3 tools/benchmark.py --only governor --split dev  --caps 0.5 --trials 1   # pilot / tuning seeds 1-5
python3 tools/benchmark.py --only governor --split test --caps 0.5 0.8 --trials 5  # held-out seeds 101-105
python3 tools/measure_braking.py --layout open_field.txt --out docs/results/braking_headless.json
```

Configurations (all Smac2D, `localization:=ground_truth`, same maze, goal (10, −10), Collision
Monitor, smoother and costmap settings): `A_RPP`, `B_RPP_gov`, `C_MPPI`, `D_MPPI_gov`. Add
`--configs` to select, `--set governor.a_lat_max=0.8` to override, `--out` to choose the file
(default `docs/results/governor_benchmark.csv`, appended, never overwritten). Each trial also runs
`tools/governor_log.py`, which writes `<logdir>/<trial>_samples.csv` (10 Hz: pose, actual speed,
requested/smoothed/governed/final speeds, governor reason/fault) and `_summary.json`.

Columns: `outcome` (reached / stalled / timeout / wall_timeout / no_result), `sim_time_s` (from
exploration start; includes exploration and recoveries), `path_m`, `v_mean`/`v_p95` (actual
speed from 10 Hz ground-truth differences, from first motion), `wall_contacts` (simulator
collision count), `min_clearance_true_m` (from the true pose and the maze file, independent of the
costmap), `stalled_s` (requested > 0.2 m/s while actual < 0.02 m/s), `recoveries` /
`progress_failures` / `controller_missed_rate` (counted from the Nav2 log text — best effort),
`scan_age_max_s` / `costmap_age_max_s` (clock minus stamp on receipt), `governor_compute_us_p95`
(governor tick time; controller timing is only the missed-rate count).
Always report success rate with times: time is only meaningful for `reached` trials.

## Results (headless, ground_truth, Smac2D; preliminary)

Held-out seeds 101-105, 2 trials per seed and config (10 trials per row), caps 0.5 and 0.8 m/s.
Raw data: `docs/results/governor_test_cap*.{csv,jsonl}`. Table: `python3 tools/summarize_governor.py docs/results/governor_test_cap0.5.csv docs/results/governor_test_cap0.8.csv`.

| cap | config | reached/10 | time s (reached only) | stalls |
|---|---|---|---|---|
| 0.5 | A RPP | 8 | 66.0 | 2 |
| 0.5 | B RPP+gov | 9 | 62.0 | 1 |
| 0.5 | C MPPI | 7 | 91.4 | 3 |
| 0.5 | D MPPI+gov | 8 | 60.8 | 2 |
| 0.8 | A RPP | 5 | 42.1 | 5 |
| 0.8 | B RPP+gov | 7 | 41.0 | 3 |
| 0.8 | C MPPI | 5 | 50.1 | 5 |
| 0.8 | D MPPI+gov | 10 | 59.9 | 0 |

Failure analysis (from the jsonl, 21 non-reached trials; per-sample logs were not kept):

- Cap 0.5: 9 failures end at about 45-65 s with path length 2-3 m, `stalled_s` about 0.1 and true
  clearance about 0.16 m. The robot was not held at a wall; this looks like the explorer or planner
  giving up early. It occurs in baseline and governor configs alike.
- Cap 0.8: 11 failures have `stalled_s` of 38-45 s and true clearance 0.01-0.03 m, so the robot
  was pressed against a wall. This is consistent with the wall-lock hypothesis but is NOT confirmed:
  `stall_hint` was not examined then because the logs were gone; the re-run logs confirm it (see
  Wall-lock failure). Governor rows B also show this (3 of 10).
- A re-run of seed 105, cap 0.5, A_RPP, which failed before, reached the goal (90.7 s). Outcomes are
  not reproducible per seed, so exploration randomness is a major confound.

**Re-run at 0.8 m/s** (same seeds, 3 trials each, 15 per row; `governor_test_cap0.8_rerun.*`,
per-sample logs in `logs_gov/cap0.8/` inside `docs/results/governor_logs.tar.gz`) did NOT reproduce the 10/10 above:

| config | reached/15 | time s (reached) | lock stalls |
|---|---|---|---|
| A RPP | 12 | 55.1 | 2 |
| B RPP+gov | 14 | 53.6 | 0 |
| C MPPI | 13 | 60.0 | 2 |
| D MPPI+gov | 11 | 53.1 | 4 |

Paired time differences are +4.1 s (1 seed) and +3.6 s (2 seeds). Pooling both 0.8 m/s runs,
success is 17/25 vs 21/25 (RPP vs RPP+gov) and 18/25 vs 21/25 (MPPI vs MPPI+gov), but the runs
disagree for MPPI+gov (10/10, then 11/15), so no direction is established. Of the 10 re-run
failures, 8 are the wall lock at the start cell or a corner (baseline and governor alike) and 2
are early exits.

**Startup cap, dev seeds 1-5, 0.8 m/s, 3 trials each (15 per row;
`docs/results/governor_dev_startup_cap0.8.csv`, logs in `logs_gov/dev_startup/` inside `docs/results/governor_logs.tar.gz`).** The governor now
holds 0.3 m/s until the robot is 1.5 m from its start (`startup_speed`, `startup_distance`).

| config | reached/15 | time s (reached) | stalls |
|---|---|---|---|
| A RPP | 14 | 50.9 | 1 |
| B RPP+gov | 13 | 54.8 | 2 |
| C MPPI | 13 | 64.5 | 2 |
| D MPPI+gov | 13 | 68.1 | 2 |

Result: no benefit. Paired time differences are +3.2 s (RPP, sd 2.1) and +4.7 s (MPPI, sd 5.0), so
the governor is slightly slower, and success is equal or lower. The start-cell wall lock still
happens (seed 3, baseline and governor, at about (0.5, -0.4..-0.7), ~20 s in, inside the 1.5 m
cap zone). `stall_hint` held for ~420 cycles in governor trials. So speed at entry is not the
cause; the lock looks geometric (robot at an angle in a tight corner, `FootprintApproach` zeroes
the whole twist). Do not describe the startup cap as a fix. Seeds 1-5 are dev seeds, so this is
not a held-out result.

Interpretation: the sample is too small and too noisy to support a claim that the governor is
faster or safer. The only consistent pattern is a larger minimum clearance at cap 0.5 (0.09-0.12 m
vs 0.02-0.04 m). Paired time differences (e.g. -33 s, sd 70 s) are not significant.
Not done: Unity runs, `localization:=slam`, more trials, a 1.0 m/s cap.

## Known limitations

- Headless only; Unity braking/reaction not measured. Headless mazes beyond seed 42 are generated
  by `tools/gen_maze.py` (not Unity-identical).
- Insertion is after the smoother (see above), forward motion only for obstacle checks.
- Costmap-based clearance and sweep use a circular footprint of 0.22 m; no non-circular footprint.
- Parameters other than braking were not tuned (the pilot only found the clearance-floor bug).
- Staleness timeouts and the confidence formula are heuristics.
- Benchmark runs share a machine and run in real time; CPU load can change timing between
  configurations. `controller_missed_rate` is the only load indicator.
- `colcon test` does not run `test/fault_injection_test.py`; run it manually (`ROS_DOMAIN_ID=77`).

## Raw run logs

The per-trial logs and per-sample CSVs behind the tables above are in `docs/results/governor_logs.tar.gz`
(top-level folder `logs_gov/`, with `cap0.8/`, `dev_startup/` and `run.out`). Unpack with
`tar xzf docs/results/governor_logs.tar.gz`; the folder is not kept in the tree.
