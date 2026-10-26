# Credits and sources

Existing work consulted for the speed governor and the wall-lock investigation (searched 2026-10-19).
Nothing here is copied code unless stated. "Candidate" means found but not yet adopted or tested in this project.

## Nav2 (Apache-2.0), https://github.com/ros-navigation/navigation2

| Component | Used for | Status |
|---|---|---|
| `nav2_collision_monitor`, `velocity_polygon` type (Iron and later, present in the installed Jazzy 1.3.13) | Selects a different zone depending on the commanded velocity. Candidate fix for the wall lock: a rotation-only polygon (linear near 0) that does not block turning. Keys confirmed in the installed library: `velocity_polygons`, `linear_min`, `linear_max`, `theta_min`, `theta_max`, `holonomic`. | Candidate, not tested. Needs the user's approval because it changes Collision Monitor configuration. |
| Issue [#3313](https://github.com/ros-navigation/navigation2/issues/3313) | Describes the same failure (a stop zone leaves the robot stuck) and the maintainers' design that led to `velocity_polygons`. | Background |
| `nav2_regulated_pure_pursuit_controller` ([docs](https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html)) | Existing curvature and obstacle-proximity speed regulation (`use_regulated_linear_velocity_scaling`, `use_cost_regulated_linear_velocity_scaling`, `curvature_lookahead_dist`). Overlaps with the governor's curvature and clearance terms, so it is the natural baseline. | Already in use (config RPP) |
| Speed Filter ([docs](https://docs.nav2.org/configuration/packages/costmap-plugins/speed_filter.html)) | Publishes a `speed_limit` topic from a mask. Static, map-based, so it does not fit unknown mazes. | Reviewed, not adopted |
| `nav2_velocity_smoother`, `nav2_graceful_controller` | Already part of the command chain. | In use |
| `IsStuck` BT condition ([docs](https://docs.nav2.org/configuration/packages/bt-plugins/conditions/IsStuck.html)) | Stuck detection to trigger recovery. | Candidate |

## Formulas

- Curvature speed bound `v = sqrt(a_lat_max / |kappa|)` is the standard lateral-acceleration limit, also described in patent literature on curvature-based speed limiting. It is generic, with no code reused.
- Stopping distance `d = v * t_reaction + v^2 / (2 * a)` is the textbook formula.

## Not found

No ready-made open-source Nav2 node that combines curvature, clearance, braking and perception-confidence speed limiting was found. The governor in `ros2_ws/src/terrain_speed_governor/` remains this project's own code.

## Planning-time comparison (docs/PLANNING_COMPARISON.md)

Figures quoted from the Nav2 Smac planner README and Theta* docs, Macenski et al. (arXiv 2401.13078),
Macenski's ROSCon 2022 slides, and two public TurtleBot3 repositories
([kavinaya2808/Autonomous-Multi-Robot-Slam-Mapping](https://github.com/kavinaya2808/Autonomous-Multi-Robot-Slam-Mapping),
[itsakhash/ros2-autonomous-explorer](https://github.com/itsakhash/ros2-autonomous-explorer)). No code was copied from them.
