#include "terrain_speed_governor/governor_math.hpp"

#include <algorithm>
#include <cmath>

namespace terrain_speed_governor
{

namespace
{
double dist(const Point2 & a, const Point2 & b) {return std::hypot(a.x - b.x, a.y - b.y);}

double wrapAngle(double a)
{
  while (a > M_PI) {a -= 2.0 * M_PI;}
  while (a < -M_PI) {a += 2.0 * M_PI;}
  return a;
}

// Cell value at world point, or -2 when outside the grid.
int cellValue(const Grid & g, double x, double y)
{
  const int ix = static_cast<int>(std::floor((x - g.origin_x) / g.resolution));
  const int iy = static_cast<int>(std::floor((y - g.origin_y) / g.resolution));
  if (ix < 0 || iy < 0 || ix >= g.width || iy >= g.height) {return -2;}
  return g.data[static_cast<size_t>(iy) * static_cast<size_t>(g.width) + static_cast<size_t>(ix)];
}

bool occupiedValue(int v, const OccupancyParams & o)
{
  if (v == -2) {return true;}                 // outside the grid: unknown
  if (v < 0) {return o.unknown_is_occupied;}  // -1 unknown
  return v >= o.occupied_threshold;
}
}  // namespace

// ---------------------------------------------------------------- curvature

std::vector<Point2> resamplePath(const std::vector<Point2> & path, double spacing)
{
  std::vector<Point2> out;
  if (path.empty() || spacing <= 0.0) {return out;}
  out.push_back(path.front());
  double carried = 0.0;  // distance walked since the last emitted point
  for (size_t i = 1; i < path.size(); ++i) {
    const Point2 & a = path[i - 1];
    const Point2 & b = path[i];
    const double seg = dist(a, b);
    if (seg < 1e-9) {continue;}
    double pos = spacing - carried;  // distance along this segment of the next sample
    while (pos <= seg) {
      const double t = pos / seg;
      out.push_back({a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)});
      pos += spacing;
    }
    carried = seg - (pos - spacing);
  }
  if (dist(out.back(), path.back()) > 1e-6) {
    // Keep the true end unless it is within a fraction of a sample of the last point.
    if (dist(out.back(), path.back()) > 0.25 * spacing) {out.push_back(path.back());}
  }
  return out;
}

std::vector<Point2> smoothPath(const std::vector<Point2> & path, int window)
{
  if (window < 3 || path.size() < 3) {return path;}
  const int half = window / 2;
  const int n = static_cast<int>(path.size());
  std::vector<Point2> out(path.size());
  for (int i = 0; i < n; ++i) {
    const int h = std::min({half, i, n - 1 - i});
    double sx = 0.0;
    double sy = 0.0;
    for (int j = i - h; j <= i + h; ++j) {
      sx += path[static_cast<size_t>(j)].x;
      sy += path[static_cast<size_t>(j)].y;
    }
    const double cnt = 2.0 * h + 1.0;
    out[static_cast<size_t>(i)] = {sx / cnt, sy / cnt};
  }
  return out;
}

std::vector<double> pathCurvature(const std::vector<Point2> & path, int k)
{
  std::vector<double> kappa(path.size(), 0.0);
  if (k < 1) {k = 1;}
  const int n = static_cast<int>(path.size());
  for (int i = k; i + k < n; ++i) {
    const Point2 & a = path[static_cast<size_t>(i - k)];
    const Point2 & b = path[static_cast<size_t>(i)];
    const Point2 & c = path[static_cast<size_t>(i + k)];
    const double ab = dist(a, b);
    const double bc = dist(b, c);
    const double ca = dist(c, a);
    const double denom = ab * bc * ca;
    if (denom < 1e-12) {continue;}  // coincident points: leave 0, never divide by ~0
    const double cross = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    kappa[static_cast<size_t>(i)] = 2.0 * std::fabs(cross) / denom;
  }
  return kappa;
}

double curveSpeed(const std::vector<double> & kappa, double spacing, const CurveProfileParams & p)
{
  double v = p.v_max;
  for (size_t i = 0; i < kappa.size(); ++i) {
    const double k = std::max(std::fabs(kappa[i]), p.kappa_eps);
    const double vc = std::min(std::sqrt(p.a_lat_max / k), p.omega_max / k);
    const double s = static_cast<double>(i) * spacing;
    v = std::min(v, std::sqrt(vc * vc + 2.0 * p.decel * s));
  }
  return v;
}

int nearestIndex(const std::vector<Point2> & path, double x, double y)
{
  int best = -1;
  double bd = 1e18;
  for (size_t i = 0; i < path.size(); ++i) {
    const double d = std::hypot(path[i].x - x, path[i].y - y);
    if (d < bd) {
      bd = d;
      best = static_cast<int>(i);
    }
  }
  return best;
}

// ------------------------------------------------------------------ costmap

bool isOccupied(const Grid & g, double x, double y, const OccupancyParams & o)
{
  if (!g.valid()) {return true;}
  return occupiedValue(cellValue(g, x, y), o);
}

double forwardClearance(const Grid & g, double x, double y, double heading, double radius,
                        double range, double half_angle, const OccupancyParams & o)
{
  if (!g.valid()) {return -1.0;}
  const int r_cells = static_cast<int>(std::ceil(range / g.resolution));
  const int cx = static_cast<int>(std::floor((x - g.origin_x) / g.resolution));
  const int cy = static_cast<int>(std::floor((y - g.origin_y) / g.resolution));
  double best = range;
  for (int dy = -r_cells; dy <= r_cells; ++dy) {
    for (int dx = -r_cells; dx <= r_cells; ++dx) {
      const int ix = cx + dx;
      const int iy = cy + dy;
      const double wx = g.origin_x + (ix + 0.5) * g.resolution;
      const double wy = g.origin_y + (iy + 0.5) * g.resolution;
      const double d = std::hypot(wx - x, wy - y);
      if (d >= range + radius) {continue;}
      if (d > 1e-9 && std::fabs(wrapAngle(std::atan2(wy - y, wx - x) - heading)) > half_angle) {
        continue;
      }
      const bool outside = ix < 0 || iy < 0 || ix >= g.width || iy >= g.height;
      if (outside) {continue;}  // beyond the window is not evidence of an obstacle for clearance
      const int v = g.data[static_cast<size_t>(iy) * static_cast<size_t>(g.width) +
          static_cast<size_t>(ix)];
      if (occupiedValue(v, o)) {best = std::min(best, d - radius);}
    }
  }
  return std::min(best, range);
}

double freeDistance(const Grid & g, double x, double y, double heading, double curvature,
                    double radius, double max_dist, const OccupancyParams & o)
{
  if (!g.valid()) {return 0.0;}
  const double step = std::max(0.5 * g.resolution, 1e-3);
  const double hx = std::cos(heading);
  const double hy = std::sin(heading);
  const int r_cells = static_cast<int>(std::ceil(radius / g.resolution)) + 1;

  double px = x;
  double py = y;
  double th = heading;
  for (double s = 0.0; s <= max_dist + 1e-9; s += step) {
    const int cx = static_cast<int>(std::floor((px - g.origin_x) / g.resolution));
    const int cy = static_cast<int>(std::floor((py - g.origin_y) / g.resolution));
    for (int dy = -r_cells; dy <= r_cells; ++dy) {
      for (int dx = -r_cells; dx <= r_cells; ++dx) {
        const double wx = g.origin_x + (cx + dx + 0.5) * g.resolution;
        const double wy = g.origin_y + (cy + dy + 0.5) * g.resolution;
        if (std::hypot(wx - px, wy - py) > radius) {continue;}
        if (!occupiedValue(cellValue(g, wx, wy), o)) {continue;}
        // Grazing: a cell that is already inside the radius at the start and is not ahead of
        // the robot is not something driving forward runs into.
        const bool near_start = std::hypot(wx - x, wy - y) <= radius + g.resolution;
        const bool ahead = (wx - x) * hx + (wy - y) * hy > 0.0;
        if (near_start && !ahead) {continue;}
        return std::max(0.0, s - step);  // last step that was still free (conservative)
      }
    }
    px += step * std::cos(th);
    py += step * std::sin(th);
    th += step * curvature;
  }
  return max_dist;
}

// ---------------------------------------------------------------- constraints

double clearanceSpeed(double clearance, double c_min, double c_full, double v_floor, double v_max)
{
  v_floor = std::clamp(v_floor, 0.0, v_max);
  if (!(clearance > c_min)) {return v_floor;}  // also covers NaN
  if (c_full <= c_min || clearance >= c_full) {return v_max;}
  return v_floor + (v_max - v_floor) * (clearance - c_min) / (c_full - c_min);
}

double brakingSpeed(double d_free, double t_react, double decel, double margin)
{
  const double avail = d_free - margin;
  if (avail <= 0.0 || decel <= 0.0) {return 0.0;}
  const double b = decel * t_react;
  return -b + std::sqrt(b * b + 2.0 * decel * avail);
}

double stoppingDistance(double v, double t_react, double decel, double margin)
{
  v = std::max(v, 0.0);
  return v * t_react + (decel > 0.0 ? v * v / (2.0 * decel) : 0.0) + margin;
}

const char * faultName(Fault f)
{
  switch (f) {
    case Fault::kNone: return "none";
    case Fault::kScanStale: return "scan_stale";
    case Fault::kScanInvalid: return "scan_invalid";
    case Fault::kCostmapStale: return "costmap_stale";
    case Fault::kCostmapInvalid: return "costmap_invalid";
    case Fault::kTransformFailure: return "tf_failure";
    case Fault::kNoCommand: return "no_command";
  }
  return "unknown";
}

Confidence evaluateConfidence(const PerceptionInputs & in, const ConfidenceParams & p)
{
  Confidence c;
  // Hard faults first: stale/invalid safety-critical data means stop, not "go a bit slower".
  if (in.scan_age < 0.0 || in.scan_age > p.scan_timeout || !std::isfinite(in.scan_age)) {
    c.scale = 0.0;
    c.fault = Fault::kScanStale;
    return c;
  }
  if (!in.scan_valid) {
    c.scale = 0.0;
    c.fault = Fault::kScanInvalid;
    return c;
  }
  if (!in.tf_ok) {
    c.scale = 0.0;
    c.fault = Fault::kTransformFailure;
    return c;
  }
  if (in.costmap_age < 0.0 || in.costmap_age > p.costmap_timeout || !std::isfinite(in.costmap_age)) {
    c.scale = 0.0;
    c.fault = Fault::kCostmapStale;
    return c;
  }
  if (!in.costmap_valid) {
    c.scale = 0.0;
    c.fault = Fault::kCostmapInvalid;
    return c;
  }

  auto ramp = [](double age, double warn, double timeout) {
      if (age <= warn) {return 1.0;}
      return std::clamp((timeout - age) / std::max(timeout - warn, 1e-9), 0.0, 1.0);
    };
  const double f_scan = ramp(in.scan_age, p.scan_warn, p.scan_timeout);
  const double f_map = ramp(in.costmap_age, p.costmap_warn, p.costmap_timeout);
  // Coverage: 0 at/below min_forward_fraction, 1 at/above full_forward_fraction.
  const double f_cov = std::clamp(
    (in.forward_valid_fraction - p.min_forward_fraction) /
    std::max(p.full_forward_fraction - p.min_forward_fraction, 1e-9), 0.0, 1.0);
  c.scale = std::min({f_scan, f_map, f_cov});
  return c;
}

// ------------------------------------------------------------------- limiting

double rampLimit(double prev, double target, double accel, double decel, double dt)
{
  if (target >= prev) {return std::min(target, prev + std::max(accel, 0.0) * dt);}
  return std::max(target, prev - std::max(decel, 0.0) * dt);
}

}  // namespace terrain_speed_governor
