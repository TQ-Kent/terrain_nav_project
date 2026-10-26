# Speed governor: session handoff (2026-10-17)

Work stopped here on request. Nothing is committed. `MazeScene.unity` was already modified
before this work and was not touched.

## What exists (all uncommitted)

- New package `ros2_ws/src/terrain_speed_governor/` (C++, ament_cmake): `governor_math` library,
  `speed_governor` node, `config/governor.yaml`, 38 gtests, `test/fault_injection_test.py`.
- Integration: `maze_test.yaml` (`governor:` block, `headless.layout`), `maze_config.py`
  (`governor_enabled` retargets `collision_monitor.cmd_vel_in_topic`), `bringup_maze_2d.launch.py`,
  `terrain_nav_bringup/package.xml`.
- Tools: `tools/benchmark.py` (`--only governor`, `--split dev|test`, `--seeds`, `--caps`,
  `--trials`, `--configs`, `--set`), `gen_maze.py`, `governor_log.py`, `measure_braking.py`,
  `summarize_governor.py`.
- Docs: `docs/GOVERNOR.md` (design, parameters, tests, benchmark how-to), short pointers in
  `README.md`, `ALGORITHMS.md`, `USER_GUIDE.md`.
- Data: `docs/results/braking_headless.json`, `governor_test_cap0.5.{csv,jsonl}`,
  `governor_test_cap0.8.{csv,jsonl}` (held-out seeds 101-105, 2 trials per config and seed).

## Verified

- `colcon build` OK; 38 gtests pass; fault-injection test: all checks pass (run with
  `ROS_DOMAIN_ID=77`).
- Headless runs work with the governor on and off; the default (off) leaves the stock chain.
- Insertion is after the velocity smoother (Nav2's launch hard-codes the earlier remaps), a
  deviation from TASK.md.

## Held-out results (headless, ground_truth, Smac2D, seeds 101-105, 10 trials per row)

| cap | config | reached | time s (reached) | stalls |
|---|---|---|---|---|
| 0.5 | A RPP | 8/10 | 66.0 | 2 |
| 0.5 | B RPP+gov | 9/10 | 62.0 | 1 |
| 0.5 | C MPPI | 7/10 | 91.4 | 3 |
| 0.5 | D MPPI+gov | 8/10 | 60.8 | 2 |
| 0.8 | A RPP | 5/10 | 42.1 | 5 |
| 0.8 | B RPP+gov | 7/10 | 41.0 | 3 |
| 0.8 | C MPPI | 5/10 | 50.1 | 5 |
| 0.8 | D MPPI+gov | 10/10 | 59.9 | 0 |

Full table: `python3 tools/summarize_governor.py docs/results/governor_test_cap0.5.csv docs/results/governor_test_cap0.8.csv`.

**These do not support a claim yet.** With 10 trials per row (5 seeds, 2 trials) the differences
in success rate are 1 to 5 trials and the time comparisons are against 1 to 3 seeds where both
sides always reached the goal. The paired time differences are noisy (e.g. -33 s with sd 70 s).
The governor rows show fewer stalls and a larger true minimum clearance at 0.5 m/s, but the
sample is small, and the trials are not independent of exploration randomness. Do not report
"faster" or "safer" from this.

## Open items, in order

1. Look at the failed trials (`outcome: stalled`), especially those with `path_m` around 3 and
   `stalled_s` near 0 in the jsonl: they look like the explorer giving up early, not the robot
   being held still, and they occur in baseline configs too. Check `/tmp/gov/test_cap*/` logs
   (may be gone; rerun a failing seed with `--seeds N --configs A_RPP` to reproduce) and
   `halt_analysis.py`. Decide whether the generated mazes or the explorer cause them.
2. Check whether `stall_hint=monitor_zeroed_governed_cmd` appears in the failures
   (`*_samples.csv` column `stall_hint`) to test the wall-lock hypothesis.
3. More trials (TASK.md asks for at least 5 per seed and config) if time allows; add a 1.0 m/s cap.
4. Add a Results section to `docs/GOVERNOR.md` and `docs/RESULTS.md` with the table above and
   the caveats; add a `docs/GOVERNOR.md` note that `colcon test` does not run the
   fault-injection script.
5. Not done: Unity runs, `localization:=slam`, braking measured on Unity, reverse obstacle checks,
   a tidy-up of the comment at the end of `generate()` in `tools/gen_maze.py` (it describes the
   start/goal cells confusingly; the layout itself was verified by BFS for seeds 1-5).
6. Parameters other than the braking defaults are untuned. The dev seeds (1-5) are still free
   for tuning; seeds 101-105 have now been used once for evaluation, so use fresh seeds for any
   further test after tuning.

## Gotchas

- Never run two simulators on one `ROS_DOMAIN_ID`; a second `/clock` publisher ruined a pilot
  trial. The benchmarks used domains 31 and 32.
- `ros2 run` leaves its child node alive when only the wrapper is terminated; kill the process
  group. `pkill -f` patterns killed the tool shell here; use PIDs from `ps`.
- Status JSON must stay valid JSON (a bare `nan` was fixed).
- A first pilot found a governor-caused 39 s stall (side wall froze the robot); fixed with the
  clearance floor and a narrower cone, with a regression test. Seed 1 is a dev seed for that reason.

## Commands

```bash
cd ros2_ws && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select terrain_speed_governor terrain_nav_bringup terrain_navigation
colcon test --packages-select terrain_speed_governor && colcon test-result --verbose
source install/setup.bash && cd ..
ROS_DOMAIN_ID=77 python3 ros2_ws/src/terrain_speed_governor/test/fault_injection_test.py
python3 tools/benchmark.py --only governor --seeds 1 --caps 0.8 --trials 1 --out /tmp/pilot.csv --logdir /tmp/bench
```
