# Planners and controllers: what they are, their role, and the math

Written 2026-10-12. This explains the four global planners and four local controllers used in the maze test (`algorithms.planner`, `algorithms.controller` in `maze_test.yaml`). Each algorithm section gives the idea, the core equations, the settings used in this project (from `nav2_params_maze.yaml`), and what to expect.

**How sure I am.** The general algorithms (A*, Dijkstra, Hybrid-A*, Theta*, pure pursuit, dynamic window, MPPI, the Park-Kuipers control law) are standard and the equations below are the textbook forms. Nav2's implementations add details (extra cost terms, tuning constants, smoothing) that I did not re-derive from the source code. Where I am describing a Nav2 detail from memory and not from the code in this repository, the text says so. The numbers in worked examples are computed from this project's settings.

---

## 1. The two roles

Robot navigation is split into two layers that work at different scales:

| | Global planner | Local controller |
| :-- | :-- | :-- |
| Question it answers | *Which route should the robot take to the goal?* | *What speed and turn rate should the wheels get right now?* |
| Output | A path: a list of poses from the robot to the goal | A velocity command (v, ω): forward speed and turn rate |
| Looks at | The whole known map (global costmap) | The area around the robot (local costmap, 6 x 6 m) and the current path |
| Runs | When a goal arrives, then re-plans about once a second | 10 times a second (`controller_frequency: 10`) |
| Thinks about the robot's motion? | Mostly no (SmacHybrid is the exception) | Yes: speed limits, acceleration, turning, obstacles |
| Failure looks like | "No path found" | Robot does not move, oscillates, or hits something |

### How they fit in this project

```
goal_explorer ── NavigateToPose (a frontier or the goal) ──► Nav2 behavior tree
                                                              │
        global costmap ──► PLANNER   ── path ──►  CONTROLLER ──► velocity smoother ──► collision monitor ──► /cmd_vel ──► robot
        (map + inflation)  (GridBased|Smac2D|     (FollowPath|MPPI|    (limits acceleration)   (stops before a wall)
                            SmacHybrid|ThetaStar)  RPP|Graceful)
                                                      ▲
                                  local costmap (scan, 6 x 6 m) ┘
```

1. `goal_explorer` picks a frontier and sends it as a goal.
2. The behavior tree asks the **planner** for a path on the global costmap (and asks again about once per second while the goal is active).
3. The behavior tree hands the latest path to the **controller**, which outputs a velocity 10 times a second.
4. The **velocity smoother** limits how fast the command may change (acceleration limit), and the **collision monitor** can reduce or zero it if the robot is about to hit something. That final command goes to the robot.

The planner and controller are chosen at run time through two topics (`/planner_selector`, `/controller_selector`); `set_algo` or the launch's `algorithms.*` settings publish to them.

### The costmap, which both of them use

A costmap is a grid (here 5 cm cells) where each cell holds a cost: 0 = free, up to 254 = occupied. Cells near walls get a decaying cost ("inflation"), so paths prefer the middle of corridors:

```
cost(d) = 252 · exp( −cs · (d − r_inscribed) )     for  r_inscribed < d < r_inflation
```

with d = distance from the cell to the nearest obstacle, r_inscribed = robot radius (0.22 m), r_inflation = 0.55 m, cs = `cost_scaling_factor` = 3.0. Inside the robot radius the cell is "inscribed" (253): the robot's centre cannot be there without touching the wall.

| Distance from wall d | Cost (0–254) | As published (0–100) |
| --: | --: | --: |
| 0.22 m | 252 | 99 |
| 0.30 m | 198 | 78 |
| 0.40 m | 147 | 58 |
| 0.50 m | 109 | 43 |
| 0.55 m | 94 | 37 |
| beyond 0.55 m | 0 | 0 |

The published value 99 is the one that appears in the explorer's stall dumps: a robot sitting on a 99 cell has its centre within one robot radius of a wall. See `SYSTEM_REVIEW.md` for why that matters.

---

## 2. Global planners

All four search a graph made from the costmap: each free cell is a node, neighbouring cells are connected, and moving between them has a cost. The planner finds the lowest-cost route from the robot's cell *s* to the goal cell *g*. Here they plan only through known space (`allow_unknown: false`).

### 2.1 Background: Dijkstra and A*

Both keep a priority queue of nodes to expand and, for each node n, a best-known cost-so-far g(n).

- **Dijkstra** expands the node with the smallest g(n). It finds the cheapest path and spreads outward evenly in all directions.
- **A\*** expands the node with the smallest

  `f(n) = g(n) + h(n)`

  where h(n) is a heuristic estimate of the remaining cost to the goal. If h never overestimates (it is "admissible", for example the straight-line distance), A* still finds the cheapest path but explores far fewer nodes, because it favours nodes pointing toward the goal. Dijkstra is A* with h = 0.

Grid moves: 4-connected (up, down, left, right) or 8-connected (also diagonals, cost √2 times the cell size). The path cost is

`C(path) = Σ over steps  step_length · (1 + penalty(cost of the cell))`

so a route through high-cost cells (close to a wall) can be longer in cost than a longer route through the middle.

### 2.2 `GridBased`: NavFn (`nav2_navfn_planner`)

**Idea.** Compute a *navigation function*: for every cell, the cheapest cost to reach the goal, spreading a wavefront out from the goal. The path is then found by walking downhill (steepest descent of the cost surface) from the robot to the goal.

**Equations.**
- Potential: `P(g) = 0`, and for any other cell `P(n) = min over neighbours m ( P(m) + c(n, m) )`, where c is the cost of stepping between cells (a neutral base cost plus a term proportional to the costmap value; I recall the constants as 50 and 0.8 from NavFn's source, not re-checked here).
- Path: from the robot's cell, repeatedly move to the neighbouring location with the lowest P (gradient descent) until the goal.
- With `use_astar: true` (used here) the wavefront is expanded with an A* ordering (cost-so-far plus a straight-line heuristic) and stops when it reaches the robot, instead of filling the whole map.

**Settings here.** `use_astar: true`, `tolerance: 0.5` (the path may end up to 0.5 m from the goal), `allow_unknown: false`.

**What to expect.** Fast and simple. The path follows the grid, so it can have small kinks, and it knows nothing about the robot's turning limits. Nav2 can smooth it afterwards.

### 2.3 `Smac2D`: 2D A* (`nav2_smac_planner::SmacPlanner2D`)

**Idea.** A* on the costmap grid with 8 neighbours, with the cell cost weighted into the step cost, followed by a smoothing step.

**Equations.**
- Search: `f(n) = g(n) + h(n)`; h is a distance estimate to the goal that ignores obstacles.
- Step cost (simplified; the exact form is in Nav2's source and was not re-derived here): `Δg = step_length · (1 + cost_travel_multiplier · cell_cost / 252)`. A larger `cost_travel_multiplier` pushes the path further from walls.
- Smoother: minimizes a weighted sum of "stay close to the original path" and "no sharp bends":

  `E = w_data · Σ‖xᵢ − xᵢ⁰‖² + w_smooth · Σ‖xᵢ₋₁ − 2xᵢ + xᵢ₊₁‖²`

  The second term is the discrete second derivative (bending). Here `w_smooth: 0.3`.

**Settings here.** `tolerance: 0.25`, `max_planning_time: 2.0 s`, `cost_travel_multiplier: 2.0`, smoother on.

**What to expect.** Similar speed to NavFn with smoother paths. Still ignores the robot's turning limits.

### 2.4 `SmacHybrid`: Hybrid-A* (`nav2_smac_planner::SmacPlannerHybrid`)

**Idea.** Plan in (x, y, heading), not just (x, y), so that the path is one a car-like or turn-limited robot can really drive. Each expansion applies a short *motion primitive* (a straight piece or an arc of the minimum turning radius) instead of hopping to a neighbouring cell centre.

**Equations.**
- State: `s = (x, y, θ)`, with θ quantized into bins; here 72 bins = 5° each. Each continuous state is rounded to its bin for the visited check, but the continuous values are kept, which is what makes the path smooth.
- Motion (forward-only Dubins model here): for an arc of length ℓ and curvature κ ∈ {−1/R_min, 0, +1/R_min}:

  `θ' = θ + κℓ`,  `x' = x + (sin θ' − sin θ)/κ`,  `y' = y − (cos θ' − cos θ)/κ`  (for κ ≠ 0; for κ = 0, `x' = x + ℓ cos θ`, `y' = y + ℓ sin θ`).

  Here R_min = `minimum_turning_radius` = 0.15 m.
- Heuristic: the larger of two admissible estimates, `h = max( h_nonholonomic , h_2D )`: the length of the shortest turn-limited path that ignores obstacles, and the obstacle-aware 2D A* distance from the goal. Taking the max keeps it admissible and much tighter.
- Cost: step length, plus a penalty from the cell cost (`cost_penalty: 2.0`), plus penalties for changing direction.
- When the search gets near the goal, it also tries a direct analytic Dubins connection ("analytic expansion", ratio 3.5 in the file).
- Optional smoothing of the final path (`smooth_path: true`).

**Settings here.** `motion_model_for_search: DUBIN`, `angle_quantization_bins: 72`, `minimum_turning_radius: 0.15`, `cost_penalty: 2.0`, `max_planning_time: 3.0 s`, `downsample_costmap: false`.

**What to expect.** Paths a controller can follow without sharp corners, so controllers did well with it (8 of 8 trials reached the goal in the benchmark). Slower to compute and the paths were longer; it was the slowest planner on average (67.9 s mean) in `RESULTS.md`.

### 2.5 `ThetaStar`: any-angle A* (`nav2_theta_star_planner`)

**Idea.** A* where a node's *parent* can be any earlier node that has a clear straight line to it, not just the neighbouring cell. This removes the staircase pattern and gives straight segments at any angle.

**Equations.** When expanding from node s to a neighbour s′, with p = parent(s):
- If there is a **line of sight** from p to s′ (the straight line crosses no blocked cell):

  `g(s′) = g(p) + ‖p − s′‖`, and `parent(s′) = p`

- Otherwise the normal A* rule: `g(s′) = g(s) + ‖s − s′‖`, and `parent(s′) = s`.

The line-of-sight check is done on the grid (a Bresenham-style walk). Because the path is built from straight segments between "visible" nodes, it is shorter than a grid path and has few turns.

**What to expect.** Short, straight paths. A straight segment can pass close to a wall corner, because line of sight only checks for blocked cells, not for margin. Whether that explains the `ThetaStar+Graceful` stalls in the benchmark was not checked.

### 2.6 Planner summary

| | Searches in | Path quality | Respects turning limits | Compute | Here |
| :-- | :-- | :-- | :-- | :-- | :-- |
| `GridBased` (NavFn A*) | (x, y), grid moves | grid-shaped | no | lowest | 5 of 8 trials reached the goal |
| `Smac2D` | (x, y), 8 neighbours + smoother | smoother | no | low | 6 of 8 |
| `SmacHybrid` | (x, y, θ) | smooth, drivable | yes | highest | 8 of 8, slowest |
| `ThetaStar` | (x, y), any angle | shortest, straight | no | low | 5 of 8 |

(Trial counts are over all four controllers each, from the benchmark; two trials per pair, one maze.)

---

## 3. Local controllers

The controller receives the global path, the robot's current pose and speed, and the local costmap, and outputs `(v, ω)`: forward speed and turn rate. The robot here is a differential drive (two wheels), so its motion is

`ẋ = v cos θ`,  `ẏ = v sin θ`,  `θ̇ = ω`

and the wheel speeds follow from v and ω through the track width. Limits come from `speed.*` (for example `max_linear` 1.0 m/s, `max_angular` 2.0 rad/s, `linear_accel` 2.0 m/s²).

### 3.1 `FollowPath`: DWB, the Dynamic Window approach (`dwb_core::DWBLocalPlanner`)

**Idea.** Try many possible velocity commands, simulate each one forward a short time, score each simulated trajectory, and pick the best.

**Equations.**
- **Dynamic window:** only velocities reachable within one control step are considered. With current speed v, acceleration limit a and period Δt:

  `v ∈ [ max(v_min, v − a·Δt),  min(v_max, v + a·Δt) ]`, and likewise for ω.

  Example: at v = 0.5 m/s, a = 2 m/s², Δt = 0.1 s the window is 0.3 to 0.7 m/s.
- **Sampling:** `vx_samples: 20` and `vtheta_samples: 20` give 20 x 20 = 400 candidate pairs. (An earlier DWB log line read "No valid trajectories out of 419", so DWB evidently adds a few extra candidates beyond the 400; I did not check which.)
- **Simulation:** each pair (v, ω) is held for `sim_time: 1.7 s` using the motion equations above, giving a trajectory.
- **Scoring:** every critic gives a trajectory a score; the total is a weighted sum

  `J(trajectory) = Σᵢ wᵢ · cᵢ(trajectory)`

  and the trajectory with the lowest J wins; its first (v, ω) is sent. A trajectory that a critic marks invalid (for example it hits an obstacle) is thrown away.
- **Critics used here:** `BaseObstacle` (cost of the cells along the trajectory), `PathAlign` and `PathDist` (stay near the global path), `GoalAlign` and `GoalDist` (head toward and approach the goal), `RotateToGoal` (turn to the goal heading at the end), `Oscillation` (penalize flip-flopping).

**What to expect.** Cheap to compute and fast when it works (mean 50 s in the benchmark). If every sampled trajectory is invalid (for example the robot is hard against a wall), it reports failure and the robot stops; it stalled in 4 of 8 benchmark trials.

### 3.2 `MPPI`: Model Predictive Path Integral (`nav2_mppi_controller`)

**Idea.** Sample thousands of random control sequences, simulate them, then take a cost-weighted average of them as the new control sequence. It solves a small optimal-control problem every cycle ("model predictive control") without needing gradients.

**Equations.** Let the control sequence be `U = (u₀, …, u_{T−1})` with `u = (v, ω)`, T = `time_steps` = 28 steps of `model_dt` = 0.1 s (a 2.8 s horizon).
1. Sample K = `batch_size` = 2000 noise sequences `εₖ ~ N(0, Σ)`, and form candidate controls `vₖ,ₜ = uₜ + εₖ,ₜ`.
2. Roll each candidate through the motion model, `x_{t+1} = f(xₜ, vₜ)`, and compute its cost

   `Sₖ = Σₜ q(xₜ, vₖ,ₜ)  +  φ(x_T)`

   where q is the sum of the critic costs (stay on the path, avoid high-cost cells, face the goal, prefer forward motion, respect speed limits) and φ is a terminal cost.
3. Weight each rollout. With temperature λ (`temperature` in the parameters) and ρ = min over k of Sₖ (subtracted for numerical stability):

   `wₖ = exp( −(Sₖ − ρ) / λ ) / η`, with η = Σₖ exp( −(Sₖ − ρ) / λ )

   Low-cost rollouts get weight near 1, high-cost ones near 0.
4. Update the sequence: `uₜ ← uₜ + Σₖ wₖ · εₖ,ₜ`.
5. Send `u₀` to the robot, shift the sequence one step forward and repeat next cycle (the log line "Control sequence shifting is ON").

Because it averages the whole population, MPPI can choose smooth motion in cluttered places where DWB's coarse sampling finds no valid trajectory.

**Critics used here:** `ConstraintCritic`, `GoalCritic`, `GoalAngleCritic`, `PathAlignCritic`, `PathFollowCritic`, `PathAngleCritic`, `PreferForwardCritic`, `CostCritic` (`CostCritic` has a collision cost of 1,000,000 for cells that would touch a wall).

**What to expect.** The most computation (2000 rollouts x 28 steps, 10 times per second) and needs tuning. It was the slowest controller here (mean 79 s) and stalled in 2 of 8 trials.

### 3.3 `RPP`: Regulated Pure Pursuit (`nav2_regulated_pure_pursuit_controller`)

**Idea.** Pick a *lookahead point* on the path a fixed distance ahead of the robot, and steer along the circular arc that goes from the robot to that point. Then *regulate* (reduce) the forward speed in sharp curves, near the end of the path and optionally near obstacles.

**Equations.**
- Let L be the lookahead distance (`lookahead_dist: 0.6 m`) and y the lateral offset of the lookahead point in the robot's own frame (to its left is positive). The arc through the point has curvature

  `κ = 2y / L²`   (turn radius `R = 1/κ = L² / 2y`)

  and the commands are `v` (see below) and `ω = v · κ`.
  Example (L = 0.6): y = 0.1 m gives κ = 0.56 /m (R = 1.8 m); y = 0.3 m gives κ = 1.67 /m (R = 0.6 m).
- **Regulated speed.** The desired speed is `v_des` (set from `speed.max_linear`). It is scaled down when the path curves tighter than `regulated_linear_scaling_min_radius` = 0.9 m:

  `v = v_des · min( 1 , R / R_min )`, never below `regulated_linear_scaling_min_speed` = 0.1 m/s

  With R_min = 0.9 m and v_des = 1.0 m/s:

  | Path turn radius R | Speed |
  | --: | --: |
  | 2.0 m or more | 1.00 m/s |
  | 0.9 m | 1.00 m/s |
  | 0.5 m | 0.56 m/s |
  | 0.3 m | 0.33 m/s |

  This is the "controller commanding below 80 % of top speed" share in `SYSTEM_REVIEW.md` (29 % of a successful 1.0 m/s run): the maze has many right-angle corners whose path radius is under 0.9 m.
- Also: slow down near the end of the path (an approach distance), optional slow-down near high-cost cells (`use_cost_regulated_linear_velocity_scaling: false` here), and rotate in place toward the path heading when the robot is facing the wrong way (`rotate_to_heading_angular_vel: 1.0` rad/s).

**What to expect.** Simple, cheap and predictable. Best result in the benchmark (8 of 8, mean 48.6 s). It follows the path closely, so a path that hugs a wall is followed into the wall.

### 3.4 `Graceful`: smooth control law (`nav2_graceful_controller`)

**Idea.** A closed-form control law, from Park and Kuipers (2011), that drives a robot to a target pose along a smooth curve, slowing down more in tight turns. The target here is a point on the path `motion_target_dist` = 0.6 m ahead.

**Equations.** Express the target in polar coordinates relative to the robot:
- `r` = distance to the target,
- `δ` = angle between the robot's heading and the line to the target,
- `φ` (or θ) = angle between the target's heading and the line of sight.

The path curvature is

`κ = −(1/r) · [ k_δ · ( δ − atan(−k_φ · φ) ) + ( 1 + k_φ / (1 + (k_φ · φ)²) ) · sin δ ]`

and the speed shrinks as the curvature grows:

`v = v_max / ( 1 + β · |κ|^λ )`,  then `ω = κ · v`.

Here `k_phi: 2.0`, `k_delta: 1.0`, `beta: 0.4`, `lambda: 2.0`. With v_max = 1.0 m/s the speed rule gives:

| Curvature κ | Speed |
| --: | --: |
| 0 (straight) | 1.00 m/s |
| 0.5 /m | 0.91 m/s |
| 1.0 /m | 0.71 m/s |
| 2.0 /m | 0.38 m/s |
| 3.0 /m | 0.22 m/s |

(I wrote the control law from the paper's standard form and Nav2's parameter names; the exact signs and the way Nav2 maps `k_phi` and `k_delta` onto the terms were not checked against the Nav2 source.)

**What to expect.** Smooth motion without sudden stops. In the benchmark it drove the longest paths (mean 37.1 m) and stalled twice with `ThetaStar`.

### 3.5 Controller summary

| | Method | Compute | Handles tight spaces | Weak point | Benchmark (8 trials) |
| :-- | :-- | :-- | :-- | :-- | :-- |
| `FollowPath` (DWB) | sample 400 (v, ω) pairs, score 1.7 s trajectories | low | medium | fails if all samples are invalid | 4 reached, 50.1 s, 35.1 m |
| `MPPI` | 2000 random rollouts, weighted average | highest | good in theory | slow, needs tuning | 6 reached, 79.0 s, 35.3 m |
| `RPP` | pure pursuit + speed regulation | lowest | medium | follows a wall-hugging path into the wall | 8 reached, 48.6 s, 34.0 m |
| `Graceful` | Park-Kuipers control law | low | medium | longer paths | 6 reached, 66.6 s, 37.1 m |

---

## 4. Why the pair matters

- The planner decides *where* the robot goes; the controller decides *how it gets there*. A path that is good for one controller can be bad for another: SmacHybrid's smooth, drivable paths suit every controller, while a straight Theta* path that skims a corner may be refused by a controller or the collision monitor.
- Neither layer guarantees clearance from walls on its own. Clearance comes from the costmap inflation (section 1), from how much each algorithm weights cell cost (`cost_travel_multiplier`, `cost_penalty`, the DWB and MPPI obstacle critics), and finally from the collision monitor, which only stops the robot, it does not steer it away.
- Both run on a map that lags the real world by up to about 2 s here (SLAM map at 1 Hz, global costmap at 1 Hz), which matters more as speed rises.

An optional speed governor (curvature, clearance, braking distance, perception freshness) can sit between the velocity smoother and the collision monitor: see `GOVERNOR.md`. With it enabled the command chain is controller ► `/cmd_vel_nav` ► smoother ► `/cmd_vel_smoothed` ► governor ► `/cmd_vel_governed` ► collision monitor ► `/cmd_vel`.

See `RESULTS.md` for the measured results and `SYSTEM_REVIEW.md` for where time is lost.

## 5. References (for the equations)

- Hart, Nilsson, Raphael (1968): A*. Dijkstra (1959).
- Nash, Daniel, Koenig, Felner (2007): Theta*, any-angle path planning.
- Dolgov, Thrun, Montemerlo, Diebel (2010): Hybrid-A* for autonomous driving. Dubins (1957): shortest curves with a turn limit.
- Fox, Burgard, Thrun (1997): the Dynamic Window approach to collision avoidance.
- Williams et al. (2017): information-theoretic MPC (the basis of MPPI).
- Coulter (1992): the pure pursuit path-tracking method; Macenski et al. (2023): Regulated Pure Pursuit.
- Park, Kuipers (2011): a smooth control law for graceful motion of differential-drive robots.
- Macenski et al.: the Nav2 documentation and source for the exact plugin behaviour.
