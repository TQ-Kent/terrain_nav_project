#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "terrain_speed_governor/governor_math.hpp"

using namespace terrain_speed_governor;  // NOLINT

namespace
{
std::vector<Point2> line(double len, double step)
{
  std::vector<Point2> p;
  for (double x = 0.0; x <= len + 1e-9; x += step) {p.push_back({x, 0.0});}
  return p;
}

std::vector<Point2> circle(double r, double step_rad, double sweep)
{
  std::vector<Point2> p;
  for (double a = 0.0; a <= sweep + 1e-9; a += step_rad) {p.push_back({r * std::sin(a), r * (1 - std::cos(a))});}
  return p;
}

// Free grid of size n x n cells (res m), origin so that world (0,0) is the centre.
Grid freeGrid(int n, double res)
{
  Grid g;
  g.width = g.height = n;
  g.resolution = res;
  g.origin_x = g.origin_y = -0.5 * n * res;
  g.data.assign(static_cast<size_t>(n) * n, 0);
  return g;
}

void setCell(Grid & g, double x, double y, int8_t v)
{
  const int ix = static_cast<int>(std::floor((x - g.origin_x) / g.resolution));
  const int iy = static_cast<int>(std::floor((y - g.origin_y) / g.resolution));
  g.data[static_cast<size_t>(iy) * g.width + ix] = v;
}

void wallX(Grid & g, double x, double y0, double y1)
{
  for (double y = y0; y <= y1; y += g.resolution) {setCell(g, x, y, 100);}
}
}  // namespace

// ---------------------------------------------------------------- curvature
TEST(Curvature, StraightLineIsZero)
{
  const auto k = pathCurvature(resamplePath(line(3.0, 0.5), 0.1), 3);
  for (double v : k) {EXPECT_NEAR(v, 0.0, 1e-9);}
}

TEST(Curvature, CircleMatchesInverseRadius)
{
  const double r = 1.2;
  const auto path = resamplePath(circle(r, 0.01, 1.5), 0.05);
  const auto k = pathCurvature(path, 3);
  for (size_t i = 3; i + 3 < k.size(); ++i) {EXPECT_NEAR(k[i], 1.0 / r, 0.02);}
}

TEST(Curvature, CoincidentPointsAreFinite)
{
  std::vector<Point2> p(10, Point2{1.0, 1.0});
  for (double v : pathCurvature(p, 1)) {EXPECT_TRUE(std::isfinite(v)); EXPECT_EQ(v, 0.0);}
}

TEST(Curvature, ShortPathDoesNotCrash)
{
  EXPECT_TRUE(pathCurvature({}, 3).empty());
  EXPECT_EQ(pathCurvature({{0, 0}, {1, 0}}, 3).size(), 2u);
  EXPECT_TRUE(resamplePath({}, 0.1).empty());
}

TEST(Curvature, ResampleKeepsSpacingAndEnds)
{
  const auto r = resamplePath({{0, 0}, {1, 0}}, 0.1);
  ASSERT_GE(r.size(), 11u);
  EXPECT_NEAR(r.front().x, 0.0, 1e-9);
  EXPECT_NEAR(r.back().x, 1.0, 0.05);
  EXPECT_NEAR(r[1].x - r[0].x, 0.1, 1e-9);
}

TEST(Curvature, StaircaseSmoothedBelowRawButTurnSurvives)
{
  // Grid-like staircase heading diagonally: raw curvature is huge, smoothing must tame it.
  std::vector<Point2> stairs;
  for (int i = 0; i < 12; ++i) {
    stairs.push_back({0.05 * i, 0.05 * i});
    stairs.push_back({0.05 * (i + 1), 0.05 * i});
  }
  const auto raw = resamplePath(stairs, 0.05);
  double raw_max = 0.0, sm_max = 0.0;
  for (double v : pathCurvature(raw, 1)) {raw_max = std::max(raw_max, v);}
  for (double v : pathCurvature(smoothPath(raw, 7), 3)) {sm_max = std::max(sm_max, v);}
  EXPECT_LT(sm_max, 0.5 * raw_max);

  // A genuine 90 degree corner (radius ~0.3 m rounded) must still show curvature after smoothing.
  std::vector<Point2> corner = line(1.0, 0.05);
  for (double a = 0.05; a <= M_PI / 2 + 1e-9; a += 0.05) {
    corner.push_back({1.0 + 0.3 * std::sin(a), 0.3 * (1 - std::cos(a))});
  }
  for (double y = 0.3; y <= 1.3; y += 0.05) {corner.push_back({1.3, y});}
  const auto kc = pathCurvature(smoothPath(resamplePath(corner, 0.05), 5), 3);
  double kmax = 0.0;
  for (double v : kc) {kmax = std::max(kmax, v);}
  EXPECT_GT(kmax, 1.5);  // true curvature 3.3 1/m, smoothing may soften but not erase it
}

TEST(CurveSpeed, StraightAndEmptyGiveVMax)
{
  CurveProfileParams p;
  EXPECT_DOUBLE_EQ(curveSpeed({}, 0.1, p), p.v_max);
  EXPECT_DOUBLE_EQ(curveSpeed(std::vector<double>(20, 0.0), 0.1, p), p.v_max);
}

TEST(CurveSpeed, MatchesFormulaAtRobot)
{
  CurveProfileParams p;
  p.v_max = 2.0;
  p.omega_max = 100.0;
  // Curvature 4 1/m right here: sqrt(0.6/4) = 0.387
  EXPECT_NEAR(curveSpeed({4.0}, 0.1, p), std::sqrt(0.6 / 4.0), 1e-9);
}

TEST(CurveSpeed, AngularLimitBinds)
{
  CurveProfileParams p;
  p.v_max = 2.0;
  p.a_lat_max = 100.0;
  p.omega_max = 1.0;
  EXPECT_NEAR(curveSpeed({2.0}, 0.1, p), 0.5, 1e-9);  // omega/kappa
}

TEST(CurveSpeed, PreviewAllowsFasterFarCurve)
{
  CurveProfileParams p;
  p.v_max = 2.0;
  p.omega_max = 100.0;
  std::vector<double> near_curve{4.0};
  std::vector<double> far_curve(30, 0.0);
  far_curve[29] = 4.0;
  EXPECT_GT(curveSpeed(far_curve, 0.1, p), curveSpeed(near_curve, 0.1, p));
}

TEST(CurveSpeed, NeverAboveVMaxOrNegative)
{
  CurveProfileParams p;
  p.v_max = 0.5;
  const double v = curveSpeed({1e-12, 0.0, 50.0}, 0.1, p);
  EXPECT_GE(v, 0.0);
  EXPECT_LE(v, 0.5);
}

// ---------------------------------------------------------------- costmap
TEST(Clearance, EmptyGridGivesRange)
{
  const Grid g = freeGrid(100, 0.05);
  OccupancyParams o;
  EXPECT_NEAR(forwardClearance(g, 0, 0, 0, 0.22, 1.0, 1.2, o), 1.0, 1e-9);
}

TEST(Clearance, WallAheadSubtractsRadius)
{
  Grid g = freeGrid(100, 0.05);
  wallX(g, 0.8, -1.0, 1.0);
  OccupancyParams o;
  EXPECT_NEAR(forwardClearance(g, 0, 0, 0, 0.22, 2.0, 1.2, o), 0.8 - 0.22, 0.04);
}

TEST(Clearance, WallBehindIsIgnored)
{
  Grid g = freeGrid(100, 0.05);
  wallX(g, -0.4, -1.0, 1.0);
  OccupancyParams o;
  EXPECT_NEAR(forwardClearance(g, 0, 0, 0, 0.22, 1.0, 1.2, o), 1.0, 1e-9);
}

TEST(Clearance, UnknownCountsAsObstacle)
{
  Grid g = freeGrid(100, 0.05);
  for (double y = -1.0; y <= 1.0; y += 0.05) {setCell(g, 0.6, y, -1);}
  OccupancyParams o;
  EXPECT_LT(forwardClearance(g, 0, 0, 0, 0.22, 1.0, 1.2, o), 0.45);
  o.unknown_is_occupied = false;
  EXPECT_NEAR(forwardClearance(g, 0, 0, 0, 0.22, 1.0, 1.2, o), 1.0, 1e-9);
}

TEST(Clearance, NarrowCorridorSmallerThanWide)
{
  OccupancyParams o;
  Grid narrow = freeGrid(100, 0.05);
  Grid wide = freeGrid(100, 0.05);
  for (double x = -1.0; x <= 1.0; x += 0.05) {
    setCell(narrow, x, 0.35, 100);
    setCell(narrow, x, -0.35, 100);
    setCell(wide, x, 0.75, 100);
    setCell(wide, x, -0.75, 100);
  }
  EXPECT_LT(forwardClearance(narrow, 0, 0, 0, 0.22, 1.0, 1.4, o),
    forwardClearance(wide, 0, 0, 0, 0.22, 1.0, 1.4, o));
}

TEST(Clearance, InvalidGridIsConservative)
{
  Grid g;
  OccupancyParams o;
  EXPECT_LT(forwardClearance(g, 0, 0, 0, 0.22, 1.0, 1.2, o), 0.0);
  EXPECT_EQ(freeDistance(g, 0, 0, 0, 0, 0.22, 2.0, o), 0.0);
}

TEST(FreeDistance, StraightToWall)
{
  Grid g = freeGrid(100, 0.05);
  wallX(g, 1.0, -1.0, 1.0);
  OccupancyParams o;
  EXPECT_NEAR(freeDistance(g, 0, 0, 0, 0.0, 0.2, 2.0, o), 0.8, 0.06);
}

TEST(FreeDistance, ObstaclesAtDifferentDistances)
{
  OccupancyParams o;
  double prev = -1.0;
  for (double wx : {0.5, 0.8, 1.2, 1.6}) {
    Grid g = freeGrid(100, 0.05);
    wallX(g, wx, -1.0, 1.0);
    const double d = freeDistance(g, 0, 0, 0, 0.0, 0.2, 2.0, o);
    EXPECT_GT(d, prev);
    prev = d;
  }
}

TEST(FreeDistance, ArcAvoidsWallStraightHits)
{
  Grid g = freeGrid(100, 0.05);
  wallX(g, 1.0, -0.1, 0.1);  // short wall right ahead
  OccupancyParams o;
  const double straight = freeDistance(g, 0, 0, 0, 0.0, 0.2, 2.0, o);
  const double curved = freeDistance(g, 0, 0, 0, 1.5, 0.2, 2.0, o);  // sharp left arc
  EXPECT_GT(curved, straight);
}

TEST(FreeDistance, GrazingWallBesideDoesNotBlock)
{
  Grid g = freeGrid(100, 0.05);
  for (double x = -1.0; x <= 2.0; x += 0.05) {setCell(g, x, 0.2, 100);}  // wall alongside, inside radius
  OccupancyParams o;
  // Cells ahead along the wall are within the radius too, so not free, but must not be zero for
  // cells beside/behind: a robot with the wall only beside it can still drive away from it.
  Grid g2 = freeGrid(100, 0.05);
  for (double x = -1.0; x <= 0.0; x += 0.05) {setCell(g2, x, 0.2, 100);}  // only behind/beside
  EXPECT_GT(freeDistance(g2, 0.0, 0, 0, 0.0, 0.22, 1.0, o), 0.9);
}

TEST(FreeDistance, OutsideWindowIsUnknownNotFree)
{
  const Grid g = freeGrid(40, 0.05);  // 2 m window
  OccupancyParams o;
  EXPECT_LT(freeDistance(g, 0, 0, 0, 0.0, 0.2, 5.0, o), 1.0);
}

// ---------------------------------------------------------------- constraints
TEST(Clearance, SpeedMapping)
{
  EXPECT_EQ(clearanceSpeed(0.0, 0.03, 0.3, 0.0, 0.5), 0.0);
  EXPECT_EQ(clearanceSpeed(-0.1, 0.03, 0.3, 0.0, 0.5), 0.0);
  EXPECT_DOUBLE_EQ(clearanceSpeed(0.5, 0.03, 0.3, 0.0, 0.5), 0.5);
  EXPECT_NEAR(clearanceSpeed(0.165, 0.03, 0.3, 0.0, 0.5), 0.25, 1e-9);
}

TEST(Clearance, FloorKeepsRobotMovingBesideWall)
{
  // Regression: a wall beside the robot (clearance ~0) once froze it (pilot, MPPI + governor).
  EXPECT_DOUBLE_EQ(clearanceSpeed(0.0, 0.03, 0.3, 0.15, 0.5), 0.15);
  EXPECT_DOUBLE_EQ(clearanceSpeed(-0.2, 0.03, 0.3, 0.15, 0.5), 0.15);
  EXPECT_NEAR(clearanceSpeed(0.165, 0.03, 0.3, 0.15, 0.5), 0.325, 1e-9);
  EXPECT_DOUBLE_EQ(clearanceSpeed(1.0, 0.03, 0.3, 0.15, 0.5), 0.5);
  EXPECT_DOUBLE_EQ(clearanceSpeed(0.0, 0.03, 0.3, 9.0, 0.5), 0.5);  // floor capped at v_max
}

TEST(Clearance, WallBesideOutsideForwardConeIgnored)
{
  Grid g = freeGrid(100, 0.05);
  for (double x = -1.0; x <= 1.0; x += 0.05) {setCell(g, x, 0.25, 100);}  // wall along the left side
  OccupancyParams o;
  // Cells exactly beside the robot are at 90 degrees: outside a +-0.6 rad cone, so range is returned
  // unless the wall is ahead-diagonal; check a cone of 0.3 rad sees nothing within 0.8 m.
  EXPECT_GT(forwardClearance(g, 0, 0, 0, 0.22, 0.2, 0.3, o), 0.19);
}

TEST(Braking, SatisfiesStoppingDistanceExactly)
{
  for (double d : {0.2, 0.5, 1.0, 2.0}) {
    const double v = brakingSpeed(d, 0.15, 1.0, 0.05);
    EXPECT_NEAR(stoppingDistance(v, 0.15, 1.0, 0.05), d, 1e-9);
  }
}

TEST(Braking, ZeroWhenNoRoom)
{
  EXPECT_EQ(brakingSpeed(0.05, 0.15, 1.0, 0.05), 0.0);
  EXPECT_EQ(brakingSpeed(0.0, 0.15, 1.0, 0.05), 0.0);
  EXPECT_EQ(brakingSpeed(-1.0, 0.15, 1.0, 0.05), 0.0);
  EXPECT_EQ(brakingSpeed(1.0, 0.15, 0.0, 0.05), 0.0);  // invalid decel: conservative
}

TEST(Braking, MonotonicInDistanceAndDecel)
{
  EXPECT_LT(brakingSpeed(0.5, 0.15, 1.0, 0.05), brakingSpeed(1.0, 0.15, 1.0, 0.05));
  EXPECT_LT(brakingSpeed(1.0, 0.15, 0.5, 0.05), brakingSpeed(1.0, 0.15, 1.0, 0.05));
  EXPECT_LT(brakingSpeed(1.0, 0.5, 1.0, 0.05), brakingSpeed(1.0, 0.0, 1.0, 0.05));
}

// ------------------------------------------------------------- confidence
TEST(Confidence, FreshInputsFullConfidence)
{
  PerceptionInputs in;
  in.scan_age = 0.05;
  in.costmap_age = 0.2;
  const auto c = evaluateConfidence(in, ConfidenceParams{});
  EXPECT_EQ(c.fault, Fault::kNone);
  EXPECT_DOUBLE_EQ(c.scale, 1.0);
}

TEST(Confidence, StaleScanIsFaultNotMultiplier)
{
  PerceptionInputs in;
  in.scan_age = 1.5;
  const auto c = evaluateConfidence(in, ConfidenceParams{});
  EXPECT_EQ(c.fault, Fault::kScanStale);
  EXPECT_EQ(c.scale, 0.0);
}

TEST(Confidence, StaleCostmapAndMissingInputsFault)
{
  PerceptionInputs in;
  in.costmap_age = 5.0;
  EXPECT_EQ(evaluateConfidence(in, ConfidenceParams{}).fault, Fault::kCostmapStale);
  in = PerceptionInputs{};
  in.scan_age = -1.0;  // never received
  EXPECT_EQ(evaluateConfidence(in, ConfidenceParams{}).fault, Fault::kScanStale);
  in = PerceptionInputs{};
  in.costmap_age = -1.0;
  EXPECT_EQ(evaluateConfidence(in, ConfidenceParams{}).fault, Fault::kCostmapStale);
}

TEST(Confidence, NaNAgeAndInvalidFlagsFault)
{
  PerceptionInputs in;
  in.scan_age = std::numeric_limits<double>::quiet_NaN();
  EXPECT_NE(evaluateConfidence(in, ConfidenceParams{}).fault, Fault::kNone);
  in = PerceptionInputs{};
  in.scan_valid = false;
  EXPECT_EQ(evaluateConfidence(in, ConfidenceParams{}).fault, Fault::kScanInvalid);
  in = PerceptionInputs{};
  in.tf_ok = false;
  EXPECT_EQ(evaluateConfidence(in, ConfidenceParams{}).fault, Fault::kTransformFailure);
  in = PerceptionInputs{};
  in.costmap_valid = false;
  EXPECT_EQ(evaluateConfidence(in, ConfidenceParams{}).fault, Fault::kCostmapInvalid);
}

TEST(Confidence, DegradesBetweenWarnAndTimeout)
{
  ConfidenceParams p;
  PerceptionInputs in;
  in.scan_age = 0.65;  // halfway between 0.3 and 1.0
  const auto c = evaluateConfidence(in, p);
  EXPECT_EQ(c.fault, Fault::kNone);
  EXPECT_NEAR(c.scale, 0.5, 1e-9);
}

TEST(Confidence, PoorForwardCoverageReducesScale)
{
  PerceptionInputs in;
  in.forward_valid_fraction = 0.7;
  EXPECT_NEAR(evaluateConfidence(in, ConfidenceParams{}).scale, 0.5, 1e-9);
  in.forward_valid_fraction = 0.3;
  EXPECT_EQ(evaluateConfidence(in, ConfidenceParams{}).scale, 0.0);
}

// ---------------------------------------------------------------- ramp
TEST(Ramp, AccelerationLimited)
{
  EXPECT_NEAR(rampLimit(0.0, 1.0, 1.5, 3.0, 0.05), 0.075, 1e-12);
}

TEST(Ramp, DecelerationLimitedAndReachesTarget)
{
  EXPECT_NEAR(rampLimit(0.5, 0.0, 1.5, 3.0, 0.05), 0.35, 1e-12);
  EXPECT_EQ(rampLimit(0.1, 0.0, 1.5, 3.0, 0.05), 0.0);
  EXPECT_EQ(rampLimit(0.5, 0.5, 1.5, 3.0, 0.05), 0.5);
}

TEST(Ramp, NeverOvershootsTarget)
{
  double v = 0.0;
  for (int i = 0; i < 100; ++i) {
    v = rampLimit(v, 0.3, 1.5, 3.0, 0.05);
    EXPECT_LE(v, 0.3 + 1e-12);
  }
  EXPECT_NEAR(v, 0.3, 1e-12);
}

TEST(Ramp, ZeroOrNegativeLimitsDoNotMove)
{
  EXPECT_EQ(rampLimit(0.2, 0.5, 0.0, 1.0, 0.05), 0.2);
  EXPECT_EQ(rampLimit(0.2, 0.0, 1.0, -1.0, 0.05), 0.2);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
