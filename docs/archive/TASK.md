# Task: Implement Adaptive Speed Planning for ROS 2 Jazzy + Nav2 Search-and-Rescue Robot

## 1. Your role

Act as a senior robotics software engineer specializing in ROS 2 Jazzy, Nav2, differential-drive mobile robots, autonomous exploration, motion planning, and robot safety.

You are working inside an existing project. Your job is to understand its current architecture, implement a well-integrated adaptive speed governor, and verify the implementation with tests and experiments.

Do not rebuild the project from scratch. Preserve existing working functionality unless a change is necessary and justified.

## 2. Project context

Project: Terrain Nav — 2D SLAM + Nav2 maze navigation.

Expected environment:
- ROS 2 Jazzy
- Ubuntu
- TurtleBot3-style differential-drive robot
- Unity 6 simulator and a headless Python simulator
- `slam_toolbox` for mapping
- Nav2 for planning and control
- A frontier-based exploration node
- A velocity smoother and Collision Monitor

The project currently supports these global planners:
- `GridBased` / NavFn
- `Smac2D`
- `SmacHybrid`
- `ThetaStar`

It supports these local controllers:
- `FollowPath` / DWB
- `MPPI`
- `RPP` / Regulated Pure Pursuit
- `Graceful`

The current benchmark uses a 6×6 maze, a fixed start and goal, and a headless simulator. Existing results suggest that the robot can sometimes stall near walls when Collision Monitor blocks its final velocity command. Treat this as a failure mode to investigate, not a conclusively established root cause.

The project has previously worked with `localization:=ground_truth`. SLAM-based localization has shown instability in some long corridors. Keep these two evaluation modes separate.

Before editing anything, verify all of these details against the actual repository. Do not assume that the descriptions above or any topic names are still accurate.

## 3. Main objective

Implement a **Clearance- and Uncertainty-Aware Adaptive Speed Governor** that helps the robot navigate unknown mazes faster while maintaining reliable obstacle avoidance.

Research question:

"Can an adaptive speed governor that considers path curvature, obstacle clearance, braking distance, and perception confidence reduce mission completion time during unknown-maze exploration without worsening safety or mission success?"

This is a research prototype. Do not claim that the algorithm is novel, safe, or superior before experiments support those claims.

## 4. Required workflow: inspect first, plan second, implement third

### Phase A — Inspect the repository

Start by checking:
- Current working directory and Git status.
- Project structure and ROS package layout.
- Existing documentation: `README.md`, `ALGORITHMS.md`, `RESULTS.md`, `SYSTEM_REVIEW.md`, and `USER_GUIDE.md`, if present.
- Nav2 launch files and parameter files.
- Current controller, velocity smoother, and Collision Monitor configuration.
- Actual velocity command topics and remappings.
- Existing ROS nodes, custom interfaces, launch conventions, and build dependencies.
- Existing benchmark and logging scripts.
- Available tests and how the workspace is built.

Trace the complete command flow from the Nav2 controller to the robot's final velocity command.

Identify the best integration point for the new governor. Avoid creating competing publishers or bypassing existing safety layers.

Do not edit files during this phase.

### Phase B — Produce an implementation plan

Write a concise technical plan covering:
1. Proposed architecture and command flow.
2. Files to add or modify.
3. ROS topics, message types, parameters, and update rate.
4. Algorithm and equations.
5. Safety behavior and failure handling.
6. Unit, integration, and simulation tests.
7. Benchmark methodology.
8. Risks, assumptions, and unresolved questions.

Explicitly identify any differences between the documented project and the actual code.

If a critical architectural fact is unclear, investigate further. Ask me only if the uncertainty cannot be resolved from the repository or available environment.

Do not begin implementation until the plan is internally consistent and the current command chain is understood.

## 5. Algorithm requirements

Implement a configurable speed governor that calculates a target linear speed using four main constraints.

### A. Curvature-aware speed

Estimate local path curvature from the actual navigation path.

Use a numerically stable estimate and handle noisy or grid-like path segments. Apply appropriate smoothing without smoothing away genuine tight turns.

A starting speed bound is:

v_curve = sqrt(a_lat_max / max(abs(curvature), epsilon))

Expose `a_lat_max` and the curvature-related settings as ROS parameters.

This is a heuristic for a differential-drive robot, not a complete dynamic model. Respect angular velocity and acceleration limits separately.

### B. Clearance-aware speed

Estimate clearance between the robot footprint and nearby obstacles using the existing costmap or an appropriate distance-transform method.

Reduce speed as clearance decreases.

Account for:
- The actual robot footprint or radius.
- Costmap resolution and inflation.
- Invalid or unavailable costmap data.
- Unknown cells.
- Narrow corridors and corners.

Never treat unknown space as confirmed free space. Do not bypass Nav2's collision checking or Collision Monitor.

### C. Braking-distance-aware speed

Use a model such as:

d_stop = v * reaction_time
       + v^2 / (2 * braking_deceleration)
       + safety_margin

Calculate a permitted speed based on available free distance and measured braking behavior.

Estimate or measure reaction delay and effective deceleration in the simulator. Do not assume theoretical braking values are achievable.

A predicted collision, insufficient stopping distance, or invalid safety-critical sensor input must result in conservative behavior.

### D. Perception-confidence-aware speed

Account for measurable indicators such as:
- Age of the latest relevant scan.
- Age of the local costmap.
- Transform availability and freshness.
- Invalid sensor messages.
- Whether the forward area is adequately observed.

Define the confidence calculation explicitly and document its limitations.

Stale or invalid safety-critical data must not be handled merely by applying a small positive speed multiplier. Choose a conservative stop or recovery behavior appropriate to the fault, while preserving the existing Nav2 safety architecture.

### E. Combine the constraints

A starting formulation is:

v_target = min(
    v_max,
    v_curve,
    v_clearance,
    v_braking,
    v_confidence
)

This formulation may be refined if the implementation requires it. Explain any changes.

Apply acceleration and deceleration constraints so that the speed command changes smoothly. Handle forward and reverse commands correctly if reverse motion is supported.

Do not arbitrarily force the angular command to zero or destroy the local controller's steering behavior. However, ensure the final linear/angular command remains consistent with collision avoidance and the robot's motion constraints.

Do not rely on this speed equation alone as a safety guarantee.

## 6. ROS 2 integration requirements

Prefer a separate ROS 2 C++ node if that fits the existing architecture.

First inspect whether the repository has a suitable command-chain insertion point. Preserve this order conceptually:

Nav2 controller
    -> adaptive speed governor
    -> velocity smoother
    -> Collision Monitor
    -> robot command interface

Adapt the actual topic names to the existing launch configuration. Do not blindly introduce example topic names.

The governor must:
- Subscribe to the controller's velocity command.
- Obtain path information through an appropriate existing interface or a justified new one.
- Obtain clearance and observation-freshness information.
- Publish the governed command to the next stage.
- Use configurable control rate and input timeouts.
- Avoid blocking callbacks and unnecessary expensive work.
- Validate parameters at startup.
- Expose useful diagnostics and throttled logs.
- Have a clearly defined startup and shutdown behavior.

Inspect the installed ROS 2 Jazzy and Nav2 interfaces before using APIs. Do not copy APIs from Humble or Rolling without verifying compatibility.

Avoid modifying Nav2 source code unless there is no reasonable alternative. Do not introduce unnecessary third-party dependencies.

## 7. Safety and existing failure investigation

The project has observed a potential wall-lock failure where the controller continues requesting motion but the final command is zeroed by Collision Monitor.

Investigate this behavior using logs and code. Determine whether the governor can help distinguish:
- A genuine obstacle stop.
- A controller progress failure.
- Stale sensor or costmap information.
- A command-chain configuration problem.
- A robot that cannot physically escape its current pose.

Do not automatically increase collision-monitor thresholds, reduce safety margins, disable collision checking, or force motion through an obstacle to improve completion time.

Keep the existing safety configuration unchanged for initial baseline comparisons. If a configuration change becomes necessary, document and evaluate it separately.

The governor must not prevent Collision Monitor from stopping the robot.

## 8. Implementation and testing

Implement incrementally.

### Stage 1: Minimal working governor
- Integrate the command chain.
- Implement curvature-aware speed limiting.
- Add parameters and diagnostics.
- Add unit tests for curvature estimation, speed bounds, and acceleration limiting.

### Stage 2: Obstacle awareness
- Add clearance estimation.
- Test narrow corridors, corners, and obstacles at different distances.
- Verify behavior with the existing footprint and costmap.

### Stage 3: Braking
- Add stopping-distance calculations.
- Measure simulated braking performance.
- Test emergency stops and obstacles appearing at different distances.

### Stage 4: Perception confidence
- Add stale-scan and stale-costmap detection.
- Test invalid transforms and missing inputs.
- Verify conservative failure behavior and recovery.

### Stage 5: Full integration
- Build the ROS workspace.
- Run unit tests.
- Run a basic integration test.
- Run headless simulation if available.
- Run Unity only if it is available and can be tested safely in the current environment.

Use the repository's established build and test commands. Do not invent successful test results. If a test cannot run because a dependency, simulator, or hardware resource is unavailable, report that clearly.

Do not modify unrelated packages to make a superficial build pass.

## 9. Experimental evaluation

Extend the existing benchmark tooling rather than creating an unrelated benchmark framework.

Compare these four configurations:

A. Smac2D + RPP  
B. Smac2D + RPP + adaptive governor  
C. Smac2D + MPPI  
D. Smac2D + MPPI + adaptive governor

First confirm that each configuration can actually be selected through the project's runtime configuration mechanism.

For matched comparisons, hold constant:
- Maze seed and geometry.
- Start and goal protocol.
- Localization mode.
- Maximum speed allowance.
- Costmap and footprint settings.
- Collision Monitor and smoother settings.
- Exploration configuration.
- Goal tolerance and timeout rules.

Test several speed caps, starting conservatively. Do not assume the maximum speed that works in one maze will be safe in all mazes.

Start with a small pilot experiment to validate the metrics and detect implementation bugs. Then use multiple maze seeds and repeated trials. If practical, use at least five seeds and five trials per configuration and seed.

Separate development/tuning seeds from held-out evaluation seeds. Do not tune parameters on the test set.

Record:
- Mission success rate.
- Total mission completion time, including exploration and recovery.
- Mean and 95th-percentile actual linear speed.
- Path length.
- Collision count.
- Minimum obstacle clearance.
- Stalled duration.
- Recovery count.
- Controller and governor computation/update timing.
- Sensor and costmap staleness.
- Requested, governed, smoothed, final commanded, and actual velocities.

Report failures separately. Never report completion time only for successful trials without also reporting success rate.

Do not mix headless and Unity measurements into one aggregate result without a justified analysis.

Add machine-readable CSV or JSON output and a concise summary report. Preserve existing benchmark outputs and avoid overwriting historical results.

## 10. Documentation

Update the relevant project documentation with:
- Architecture and command flow.
- Algorithm equations and assumptions.
- New ROS parameters and defaults.
- Launch and run instructions.
- Safety limitations.
- Build and test commands.
- Benchmark instructions.
- Metrics and interpretation.
- Known limitations and unimplemented features.

Clearly distinguish implemented functionality from planned work.

Do not describe the algorithm as state of the art or research-novel unless supported by a literature review.

## 11. Git and change management

- Inspect Git status before modifying files.
- Preserve all existing user changes.
- Do not discard, reset, or overwrite unrelated work.
- Do not commit, push, or change branches unless explicitly requested.
- Avoid editing generated build outputs or installed artifacts.
- Keep changes focused and reviewable.
- Never claim a test passed unless you actually ran it and observed the result.

## 12. Definition of done

The task is complete only when:
1. The governor integrates into the actual Nav2 command chain.
2. The ROS workspace builds, or any environmental blocker is clearly documented.
3. Unit tests cover the main mathematical functions and safety edge cases.
4. The original navigation configuration can still run with the governor disabled.
5. The enabled and disabled modes can be compared on the same maze.
6. Logs expose enough information to explain speed reductions and stops.
7. The benchmark produces machine-readable results.
8. Documentation explains how to reproduce the tests.
9. A final report lists changed files, key design decisions, test commands and results, unresolved risks, and recommended next steps.

## 13. How to proceed now

Begin with repository inspection and the implementation plan.

Do not edit any files until you have verified the actual architecture and completed the plan.

After the plan is established, implement the stages incrementally. Test each stage before proceeding to the next, and continue until the implementation is complete or a concrete blocker prevents further progress.

At the end, provide:
- A summary of the implemented algorithm.
- A list of changed files.
- Exact build and test commands executed.
- Actual test results.
- How to enable and disable the governor.
- How to run the benchmark.
- Known limitations and next steps.
---

# Progress status (updated 2026-10-17): resume from here

Full details: `docs/archive/GOVERNOR_HANDOFF.md` (older, partly superseded by `docs/GOVERNOR.md`), design in `docs/GOVERNOR.md`.
Nothing is committed. `unity_project/Assets/Scenes/MazeScene.unity` was already modified before this work: leave it.

## Done (sections 4-9 of this task, headless only)
- Phase A/B inspection and plan done. Verified chain: controller -> `/cmd_vel_nav` -> velocity_smoother -> `/cmd_vel_smoothed` -> collision_monitor -> `/cmd_vel`. Deviation: the governor is inserted AFTER the smoother (Nav2's `navigation_launch.py` hard-codes the earlier remaps), i.e. smoother -> `speed_governor` -> `/cmd_vel_governed` -> collision_monitor.
- Stages 1-5 implemented: package `ros2_ws/src/terrain_speed_governor/` (curvature, clearance, braking, perception confidence, fault stop with hysteresis), 38 gtests passing, `test/fault_injection_test.py` all passing (`ROS_DOMAIN_ID=77`).
- Enable with `set:=governor.enabled=true` (default off = stock chain). Braking defaults measured in the headless sim (`docs/results/braking_headless.json`).
- Benchmark extended (`tools/benchmark.py --only governor`), plus `gen_maze.py`, `governor_log.py`, `measure_braking.py`, `summarize_governor.py`. Held-out run on seeds 101-105 (caps 0.5 and 0.8, 2 trials) is in `docs/results/governor_test_cap*.csv`.
- Docs: `docs/GOVERNOR.md`, plus pointers in README, ALGORITHMS, USER_GUIDE.

## Result so far (do not over-claim)
Success counts of 10 trials: cap 0.5: RPP 8, RPP+gov 9, MPPI 7, MPPI+gov 8; cap 0.8: RPP 5, RPP+gov 7, MPPI 5, MPPI+gov 10. Samples are small and paired time differences are noisy; no claim of faster or safer is supported yet.

## Remaining (in order)
1. Investigate the failed (stalled) trials, many look like early explorer give-ups also seen in baseline configs; check `stall_hint` in the `*_samples.csv` logs for the wall-lock hypothesis.
2. More trials (>= 5 per seed/config), a 1.0 m/s cap; use fresh seeds if parameters are tuned (dev seeds 1-5 are free, 101-105 already used once).
3. Add a Results section to `docs/GOVERNOR.md` and `docs/RESULTS.md` with the caveats.
4. Not done: Unity runs, `localization:=slam`, Unity braking measurement, reverse obstacle checks, tidy comment at end of `generate()` in `tools/gen_maze.py`.
5. Final report required by section 13: summary, changed files (`git status`), exact commands and results, enable/disable, benchmark how-to, limitations.

## Gotchas
Never run two simulators on one `ROS_DOMAIN_ID`. Kill process groups, not just `ros2 run` wrappers; avoid `pkill -f` in the tool shell.
