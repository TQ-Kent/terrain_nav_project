// Pure (ROS-free) math for the adaptive speed governor, so it can be unit tested.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace terrain_speed_governor
{

struct Point2
{
  double x{0.0};
  double y{0.0};
};

// ---------------------------------------------------------------- curvature

// Resample a polyline at a fixed arc-length spacing (first and last point kept).
std::vector<Point2> resamplePath(const std::vector<Point2> & path, double spacing);

// Moving-average smoothing of a resampled path. `window` is the odd number of points averaged;
// the window shrinks symmetrically at the ends so both endpoints stay fixed.
std::vector<Point2> smoothPath(const std::vector<Point2> & path, int window);

// Unsigned Menger curvature (1/m) at every point of a resampled path, using points `k` samples
// before and after (k >= 1). Ends (where the triple does not fit) are 0.
std::vector<double> pathCurvature(const std::vector<Point2> & path, int k);

struct CurveProfileParams
{
  double a_lat_max{0.6};     // m/s^2 lateral acceleration budget
  double kappa_eps{1e-3};    // 1/m floor on curvature (straight line -> huge speed)
  double omega_max{1.5};     // rad/s: v <= omega_max / |kappa|
  double decel{1.0};         // m/s^2 deceleration assumed when previewing a curve
  double v_max{0.5};
};

// Speed permitted now, given curvature samples ahead at `spacing` metres (index 0 = at robot).
// Point i limits speed to sqrt(vc_i^2 + 2*decel*s_i), i.e. the robot may still be braking into
// the curve. Returns v_max when kappa is empty.
double curveSpeed(const std::vector<double> & kappa, double spacing, const CurveProfileParams & p);

// Index of the path point nearest to (x, y); -1 when empty.
int nearestIndex(const std::vector<Point2> & path, double x, double y);

// ------------------------------------------------------------------ costmap

// Occupancy grid (nav_msgs convention: -1 unknown, 0..100 cost). Row-major, origin = lower-left
// corner of cell (0,0); axis-aligned with the frame the robot pose is given in.
struct Grid
{
  int width{0};
  int height{0};
  double resolution{0.05};
  double origin_x{0.0};
  double origin_y{0.0};
  std::vector<int8_t> data;

  bool valid() const
  {
    return width > 0 && height > 0 && resolution > 0.0 &&
           data.size() == static_cast<size_t>(width) * static_cast<size_t>(height);
  }
};

struct OccupancyParams
{
  int occupied_threshold{99};      // cost >= this is an obstacle (99 = inscribed, 100 = lethal)
  bool unknown_is_occupied{true};  // unknown (-1) is never treated as free
};

// True when (x, y) is an obstacle. Points outside the grid count as occupied (unknown).
bool isOccupied(const Grid & g, double x, double y, const OccupancyParams & o);

// Clearance between a circular footprint of `radius` at (x, y) and the nearest obstacle cell
// centre within the forward sector (|angle - heading| <= half_angle) and `range`. Returns `range`
// when nothing is found, and may be negative if an obstacle is already inside the radius.
double forwardClearance(const Grid & g, double x, double y, double heading, double radius,
                        double range, double half_angle, const OccupancyParams & o);

// Distance the centre can travel along the commanded arc (curvature = omega/v, sign gives turn
// direction; straight when ~0) before the circular footprint of `radius` touches an obstacle cell.
// Cells already inside the radius and not ahead of the robot are ignored (robot grazing a wall).
// Returns `max_dist` when the whole arc is free.
double freeDistance(const Grid & g, double x, double y, double heading, double curvature,
                    double radius, double max_dist, const OccupancyParams & o);

// ---------------------------------------------------------------- constraints

// Linear speed from clearance: v_floor at c <= c_min, v_max at c >= c_full, linear in between.
// The floor is deliberate: clearance to a wall beside the robot must slow it, never freeze it
// (a robot hugging a wall can always drive along it). Obstacles actually in the way are handled
// by the braking bound (swept free distance), which does go to zero.
double clearanceSpeed(double clearance, double c_min, double c_full, double v_floor, double v_max);

// Largest v with  v*t_react + v^2/(2*decel) + margin <= d_free.
double brakingSpeed(double d_free, double t_react, double decel, double margin);

// Stopping distance for speed v (the formula brakingSpeed inverts).
double stoppingDistance(double v, double t_react, double decel, double margin);

enum class Fault
{
  kNone = 0,
  kScanStale,
  kScanInvalid,
  kCostmapStale,
  kCostmapInvalid,
  kTransformFailure,
  kNoCommand
};

const char * faultName(Fault f);

struct PerceptionInputs
{
  double scan_age{0.0};              // s since last scan stamp (<0 = never received)
  double costmap_age{0.0};           // s since last costmap stamp (<0 = never received)
  bool tf_ok{true};                  // map/odom -> base transform fresh and available
  bool scan_valid{true};             // last scan parsed with enough valid beams
  bool costmap_valid{true};
  double forward_valid_fraction{1.0};  // fraction of forward-cone beams with a usable range
};

struct ConfidenceParams
{
  double scan_warn{0.3};
  double scan_timeout{1.0};
  double costmap_warn{1.0};
  double costmap_timeout{3.0};
  double min_forward_fraction{0.5};   // below this the forward area counts as unobserved
  double full_forward_fraction{0.9};  // at/above this coverage has no effect
};

struct Confidence
{
  double scale{1.0};          // 0..1 multiplier on v_max (degraded but usable)
  Fault fault{Fault::kNone};  // != kNone: stop, do not just slow down
};

Confidence evaluateConfidence(const PerceptionInputs & in, const ConfidenceParams & p);

// ------------------------------------------------------------------- limiting

// Move `prev` toward `target` by at most accel*dt (speeding up in magnitude) or decel*dt (slowing).
double rampLimit(double prev, double target, double accel, double decel, double dt);

}  // namespace terrain_speed_governor
