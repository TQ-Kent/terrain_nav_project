// Adaptive speed governor: sits between the Nav2 velocity smoother and the Collision Monitor.
//   /cmd_vel_smoothed --> speed_governor --> /cmd_vel_governed --> collision_monitor --> /cmd_vel
// It can only lower the linear speed magnitude of the incoming command. It never raises a speed,
// never talks to the robot directly and never touches the Collision Monitor's thresholds.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "terrain_speed_governor/governor_math.hpp"

using namespace terrain_speed_governor;  // NOLINT

namespace
{
struct Pose2
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

enum Reason {kInput = 0, kVMax, kCurve, kClearance, kBraking, kConfidence, kFault, kStartup, kNumReasons};
const char * kReasonNames[kNumReasons] =
{"input", "v_max", "curvature", "clearance", "braking", "confidence", "fault", "startup"};
}  // namespace

class SpeedGovernor : public rclcpp::Node
{
public:
  SpeedGovernor()
  : Node("speed_governor")
  {
    declareParams();
    validateParams();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    auto qos_sensor = rclcpp::SensorDataQoS();
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_in_topic_, 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr m) {
        cmd_ = *m;
        cmd_time_ = now();
        have_cmd_ = true;
      });
    final_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      final_cmd_topic_, 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr m) {
        final_v_ = m->linear.x;
        final_time_ = now();
      });
    plan_sub_ = create_subscription<nav_msgs::msg::Path>(
      plan_topic_, rclcpp::QoS(5),
      [this](nav_msgs::msg::Path::ConstSharedPtr m) {onPlan(*m);});
    costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      costmap_topic_, rclcpp::QoS(1).transient_local().reliable(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m) {onCostmap(*m);});
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, qos_sensor,
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr m) {onScan(*m);});

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_out_topic_, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("speed_governor/status", 10);

    // Node-clock timer: follows /clock when use_sim_time is set, so dt stays correct at any real-time factor.
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate_hz_), [this]() {tick();});
    RCLCPP_INFO(
      get_logger(), "speed_governor up: %s -> %s, v_max %.2f m/s, %.0f Hz", cmd_in_topic_.c_str(),
      cmd_out_topic_.c_str(), v_max_, rate_hz_);
  }

private:
  // ------------------------------------------------------------ parameters
  template<typename T>
  T param(const std::string & name, const T & def)
  {
    return declare_parameter<T>(name, def);
  }

  void declareParams()
  {
    cmd_in_topic_ = param<std::string>("cmd_in_topic", "cmd_vel_smoothed");
    cmd_out_topic_ = param<std::string>("cmd_out_topic", "cmd_vel_governed");
    final_cmd_topic_ = param<std::string>("final_cmd_topic", "cmd_vel");
    plan_topic_ = param<std::string>("plan_topic", "received_global_plan");
    costmap_topic_ = param<std::string>("costmap_topic", "local_costmap/costmap");
    scan_topic_ = param<std::string>("scan_topic", "scan");
    base_frame_ = param<std::string>("base_frame", "base_link");

    rate_hz_ = param<double>("rate_hz", 20.0);
    cmd_timeout_ = param<double>("cmd_timeout", 0.5);
    tf_timeout_ = param<double>("tf_timeout", 0.5);
    v_max_ = param<double>("v_max", 0.5);
    max_reverse_ = param<double>("max_reverse", 0.1);
    accel_ = param<double>("accel", 1.5);
    decel_ = param<double>("decel", 1.5);
    emergency_decel_ = param<double>("emergency_decel", 3.0);
    min_speed_ = param<double>("min_speed", 0.05);
    omega_min_ratio_ = param<double>("omega_min_ratio", 0.5);
    startup_speed_ = param<double>("startup_speed", 0.0);
    startup_distance_ = param<double>("startup_distance", 1.5);
    recovery_cycles_ = param<int>("recovery_cycles", 10);

    curve_.a_lat_max = param<double>("a_lat_max", 0.6);
    curve_.kappa_eps = param<double>("kappa_eps", 1e-3);
    curve_.omega_max = param<double>("omega_max", 1.5);
    curve_.decel = param<double>("curve_preview_decel", 1.0);
    path_spacing_ = param<double>("path_spacing", 0.1);
    smooth_window_ = param<int>("smooth_window", 5);
    curvature_k_ = param<int>("curvature_k", 3);
    curve_lookahead_ = param<double>("curve_lookahead", 2.0);
    path_max_offset_ = param<double>("path_max_offset", 1.0);

    robot_radius_ = param<double>("robot_radius", 0.22);
    occ_.occupied_threshold = param<int>("occupied_threshold", 100);
    occ_.unknown_is_occupied = param<bool>("unknown_is_occupied", true);
    clearance_min_ = param<double>("clearance_min", 0.03);
    clearance_full_ = param<double>("clearance_full", 0.30);
    clearance_range_ = param<double>("clearance_range", 1.0);
    clearance_half_angle_ = param<double>("clearance_half_angle", 0.6);
    clearance_floor_ = param<double>("clearance_floor_speed", 0.15);

    t_react_ = param<double>("reaction_time", 0.15);
    brake_decel_ = param<double>("braking_decel", 1.0);
    safety_margin_ = param<double>("safety_margin", 0.05);
    look_ahead_max_ = param<double>("look_ahead_max", 2.0);

    conf_.scan_warn = param<double>("scan_warn", 0.3);
    conf_.scan_timeout = param<double>("scan_timeout", 1.0);
    conf_.costmap_warn = param<double>("costmap_warn", 1.0);
    conf_.costmap_timeout = param<double>("costmap_timeout", 3.0);
    conf_.min_forward_fraction = param<double>("min_forward_fraction", 0.5);
    conf_.full_forward_fraction = param<double>("full_forward_fraction", 0.9);
    scan_half_angle_ = param<double>("scan_forward_half_angle", 0.8);
    min_valid_scan_fraction_ = param<double>("min_valid_scan_fraction", 0.2);
  }

  void validateParams() const
  {
    auto bad = [](const std::string & m) {throw std::invalid_argument("speed_governor: " + m);};
    if (rate_hz_ <= 0.0 || rate_hz_ > 200.0) {bad("rate_hz must be in (0, 200]");}
    if (v_max_ <= 0.0) {bad("v_max must be > 0");}
    if (max_reverse_ < 0.0) {bad("max_reverse must be >= 0");}
    if (accel_ <= 0.0 || decel_ <= 0.0 || emergency_decel_ < decel_) {
      bad("accel/decel must be > 0 and emergency_decel >= decel");
    }
    if (min_speed_ < 0.0 || min_speed_ > v_max_) {bad("min_speed must be in [0, v_max]");}
    if (omega_min_ratio_ < 0.0 || omega_min_ratio_ > 1.0) {bad("omega_min_ratio must be in [0,1]");}
    if (curve_.a_lat_max <= 0.0 || curve_.kappa_eps <= 0.0 || curve_.omega_max <= 0.0) {
      bad("a_lat_max, kappa_eps, omega_max must be > 0");
    }
    if (path_spacing_ <= 0.0 || smooth_window_ < 1 || curvature_k_ < 1) {
      bad("path_spacing, smooth_window, curvature_k must be positive");
    }
    if (robot_radius_ <= 0.0) {bad("robot_radius must be > 0");}
    if (clearance_full_ <= clearance_min_) {bad("clearance_full must exceed clearance_min");}
    if (clearance_floor_ < 0.0 || clearance_floor_ > v_max_) {bad("clearance_floor_speed must be in [0, v_max]");}
    if (clearance_half_angle_ <= 0.0 || clearance_half_angle_ > M_PI) {bad("clearance_half_angle must be in (0, pi]");}
    if (brake_decel_ <= 0.0 || t_react_ < 0.0 || safety_margin_ < 0.0) {
      bad("braking_decel > 0, reaction_time >= 0, safety_margin >= 0 required");
    }
    if (conf_.scan_timeout <= conf_.scan_warn || conf_.costmap_timeout <= conf_.costmap_warn) {
      bad("timeouts must exceed warn ages");
    }
    if (conf_.full_forward_fraction <= conf_.min_forward_fraction) {
      bad("full_forward_fraction must exceed min_forward_fraction");
    }
  }

  // ------------------------------------------------------------ callbacks
  void onPlan(const nav_msgs::msg::Path & m)
  {
    std::vector<Point2> pts;
    pts.reserve(m.poses.size());
    for (const auto & p : m.poses) {
      if (!std::isfinite(p.pose.position.x) || !std::isfinite(p.pose.position.y)) {
        plan_.clear();  // corrupt plan: ignore curvature rather than use it
        return;
      }
      pts.push_back({p.pose.position.x, p.pose.position.y});
    }
    plan_frame_ = m.header.frame_id;
    plan_ = smoothPath(resamplePath(pts, path_spacing_), smooth_window_);
    plan_kappa_ = pathCurvature(plan_, curvature_k_);
  }

  void onCostmap(const nav_msgs::msg::OccupancyGrid & m)
  {
    Grid g;
    g.width = static_cast<int>(m.info.width);
    g.height = static_cast<int>(m.info.height);
    g.resolution = m.info.resolution;
    g.origin_x = m.info.origin.position.x;
    g.origin_y = m.info.origin.position.y;
    g.data.assign(m.data.begin(), m.data.end());
    // Rotated costmaps are not supported (Nav2 rolling windows are axis aligned).
    const double yaw = tf2::getYaw(m.info.origin.orientation);
    costmap_valid_ = g.valid() && std::fabs(yaw) < 1e-3;
    grid_ = std::move(g);
    costmap_frame_ = m.header.frame_id;
    costmap_stamp_ = rclcpp::Time(m.header.stamp, get_clock()->get_clock_type());
    have_costmap_ = true;
  }

  void onScan(const sensor_msgs::msg::LaserScan & m)
  {
    scan_stamp_ = rclcpp::Time(m.header.stamp, get_clock()->get_clock_type());
    have_scan_ = true;
    size_t valid = 0;
    size_t fwd_total = 0;
    size_t fwd_valid = 0;
    for (size_t i = 0; i < m.ranges.size(); ++i) {
      const float r = m.ranges[i];
      // inf = no return within range (free space); NaN or out-of-bounds = unusable beam.
      const bool ok = !std::isnan(r) && (std::isinf(r) || (r >= m.range_min && r <= m.range_max));
      if (ok) {++valid;}
      const double a = m.angle_min + static_cast<double>(i) * m.angle_increment;
      if (std::fabs(std::atan2(std::sin(a), std::cos(a))) <= scan_half_angle_) {
        ++fwd_total;
        if (ok) {++fwd_valid;}
      }
    }
    scan_valid_ = !m.ranges.empty() && m.angle_increment != 0.0f &&
      static_cast<double>(valid) / static_cast<double>(m.ranges.size()) >= min_valid_scan_fraction_;
    fwd_fraction_ = fwd_total ? static_cast<double>(fwd_valid) / static_cast<double>(fwd_total) : 0.0;
  }

  // ------------------------------------------------------------ helpers
  bool lookupPose(const std::string & frame, Pose2 & out, bool check_age)
  {
    try {
      const auto tf = tf_buffer_->lookupTransform(frame, base_frame_, tf2::TimePointZero);
      if (check_age) {
        const rclcpp::Time stamp(tf.header.stamp, get_clock()->get_clock_type());
        if (stamp.nanoseconds() != 0 && (now() - stamp).seconds() > tf_timeout_) {return false;}
      }
      out.x = tf.transform.translation.x;
      out.y = tf.transform.translation.y;
      out.yaw = tf2::getYaw(tf.transform.rotation);
      return std::isfinite(out.x) && std::isfinite(out.y) && std::isfinite(out.yaw);
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  // ------------------------------------------------------------ main loop
  void tick()
  {
    const auto t0 = std::chrono::steady_clock::now();
    const rclcpp::Time t = now();
    const double dt = 1.0 / rate_hz_;

    if (!have_cmd_ || (t - cmd_time_).seconds() > cmd_timeout_) {
      // No fresh upstream command: nothing to govern. Track state so a restart ramps from zero.
      prev_v_ = 0.0;
      clean_cycles_ = 0;
      return;
    }

    const double v_in = cmd_.linear.x;
    const double w_in = cmd_.angular.z;

    // ---- perception confidence
    Pose2 pose_cm;  // base in the costmap frame
    const bool tf_cm = have_costmap_ && lookupPose(costmap_frame_, pose_cm, true);
    PerceptionInputs pin;
    pin.scan_age = have_scan_ ? (t - scan_stamp_).seconds() : -1.0;
    pin.costmap_age = have_costmap_ ? (t - costmap_stamp_).seconds() : -1.0;
    pin.tf_ok = tf_cm;
    pin.scan_valid = scan_valid_;
    pin.costmap_valid = costmap_valid_;
    pin.forward_valid_fraction = fwd_fraction_;
    // Without a costmap there is no frame to look up: report that as a costmap fault, not a TF one.
    if (!have_costmap_) {pin.tf_ok = true;}
    const Confidence conf = evaluateConfidence(pin, conf_);

    double v_curve = v_max_;
    double v_clear = v_max_;
    double v_brake = v_max_;
    double clearance = clearance_range_;
    double d_free = look_ahead_max_;
    double kappa_ahead = 0.0;
    const bool forward = v_in > 0.01;

    if (forward && conf.fault == Fault::kNone) {
      // ---- curvature along the actual plan
      Pose2 pose_pl;
      if (!plan_.empty() && lookupPose(plan_frame_, pose_pl, false)) {
        const int idx = nearestIndex(plan_, pose_pl.x, pose_pl.y);
        if (idx >= 0 &&
          std::hypot(plan_[static_cast<size_t>(idx)].x - pose_pl.x,
          plan_[static_cast<size_t>(idx)].y - pose_pl.y) <= path_max_offset_)
        {
          const size_t n = std::min(
            plan_kappa_.size() - static_cast<size_t>(idx),
            static_cast<size_t>(curve_lookahead_ / path_spacing_) + 1);
          std::vector<double> ahead(
            plan_kappa_.begin() + idx, plan_kappa_.begin() + idx + static_cast<long>(n));
          curve_.v_max = v_max_;
          v_curve = curveSpeed(ahead, path_spacing_, curve_);
          for (double k : ahead) {kappa_ahead = std::max(kappa_ahead, k);}
        }
      }
      // ---- clearance and braking from the local costmap
      clearance = forwardClearance(
        grid_, pose_cm.x, pose_cm.y, pose_cm.yaw, robot_radius_, clearance_range_,
        clearance_half_angle_, occ_);
      v_clear = clearanceSpeed(clearance, clearance_min_, clearance_full_, clearance_floor_, v_max_);
      const double arc = std::fabs(v_in) > 0.01 ? w_in / v_in : 0.0;
      d_free = freeDistance(
        grid_, pose_cm.x, pose_cm.y, pose_cm.yaw, arc, robot_radius_, look_ahead_max_, occ_);
      v_brake = brakingSpeed(d_free, t_react_, brake_decel_, safety_margin_);
    }
    const double v_conf = conf.scale * v_max_;

    // ---- combine: the minimum of every bound; remember which one is binding
    double target = std::fabs(v_in);
    Reason reason = kInput;
    auto consider = [&](double bound, Reason r) {
        if (bound < target) {
          target = bound;
          reason = r;
        }
      };
    if (v_in >= 0.0) {
      consider(v_max_, kVMax);
    } else {
      consider(max_reverse_, kVMax);
    }
    if (forward) {
      consider(v_curve, kCurve);
      consider(v_clear, kClearance);
      consider(v_brake, kBraking);
    }
    consider(v_conf, kConfidence);

    // Startup cap: the first scan sweep has not mapped the start cell yet, so walls appear within
    // a fraction of a second during the first turn. Stay slow until the robot has moved
    // startup_distance from where it began (0 disables).
    if (startup_speed_ > 0.0 && tf_cm) {
      if (!have_start_) {
        start_x_ = pose_cm.x;
        start_y_ = pose_cm.y;
        have_start_ = true;
      }
      if (std::hypot(pose_cm.x - start_x_, pose_cm.y - start_y_) < startup_distance_) {
        if (forward) {consider(startup_speed_, kStartup);}
      } else {
        startup_speed_ = 0.0;
      }
    }

    // Fault handling with hysteresis: stop and stay stopped until inputs have been good for a while.
    if (conf.fault != Fault::kNone) {
      clean_cycles_ = 0;
      fault_latched_ = conf.fault;
    } else if (fault_latched_ != Fault::kNone) {
      if (++clean_cycles_ >= recovery_cycles_) {fault_latched_ = Fault::kNone;}
    }
    if (fault_latched_ != Fault::kNone) {
      target = 0.0;
      reason = kFault;
    }

    // Creep floor: keep moving slowly when only curvature/clearance limit us and there is room.
    // Collision Monitor remains the last word; the floor never applies to braking, confidence,
    // fault or zero-clearance situations.
    if (forward && fault_latched_ == Fault::kNone && (reason == kCurve || reason == kClearance) &&
      clearance > clearance_min_ && v_brake > min_speed_)
    {
      target = std::min(std::fabs(v_in), std::max(target, min_speed_));
    }

    // ---- acceleration limiting (magnitude); emergency decel for safety-driven reductions
    const bool urgent = reason == kBraking || reason == kConfidence || reason == kFault;
    double out_mag = rampLimit(
      prev_v_, target, accel_, urgent ? emergency_decel_ : decel_, dt);
    out_mag = std::min(out_mag, std::fabs(v_in));  // never exceed what the smoother asked for
    prev_v_ = out_mag;

    geometry_msgs::msg::Twist out = cmd_;
    if (fault_latched_ != Fault::kNone) {
      out.linear.x = 0.0;
      out.angular.z = 0.0;
    } else {
      out.linear.x = (v_in >= 0.0 ? 1.0 : -1.0) * out_mag;
      if (std::fabs(v_in) > 0.01) {
        // Keep the commanded arc but never freeze steering: a circular robot can rotate safely.
        const double ratio = std::max(out_mag / std::fabs(v_in), omega_min_ratio_);
        out.angular.z = w_in * std::min(ratio, 1.0);
      }
    }
    cmd_pub_->publish(out);

    publishStatus(t, v_in, w_in, out, v_curve, v_clear, v_brake, v_conf, reason, conf,
      clearance, d_free, kappa_ahead, pin,
      std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count());
  }

  void publishStatus(
    const rclcpp::Time & t, double v_in, double w_in, const geometry_msgs::msg::Twist & out,
    double v_curve, double v_clear, double v_brake, double v_conf, Reason reason,
    const Confidence & conf, double clearance, double d_free, double kappa, const PerceptionInputs & pin,
    double compute_us)
  {
    // Stall hint: the controller wants to move but the final command is ~0.
    const bool final_fresh = final_time_.nanoseconds() != 0 && (t - final_time_).seconds() < 0.5;
    const bool wants = v_in > 0.3;
    const bool final_zero = final_fresh && std::fabs(final_v_) < 0.02;
    const char * hint = "none";
    if (wants && final_zero) {
      if (fault_latched_ != Fault::kNone) {
        hint = "perception_fault";
      } else if (out.linear.x > 0.1) {
        hint = "monitor_zeroed_governed_cmd";  // governor passed motion, final cmd is zero
      } else {
        hint = "governor_limit";
      }
      stalled_s_ += 1.0 / rate_hz_;
    } else {
      stalled_s_ = 0.0;
    }

    char final_str[24] = "null";  // JSON has no NaN
    if (final_fresh) {std::snprintf(final_str, sizeof(final_str), "%.3f", final_v_);}
    char buf[900];
    std::snprintf(
      buf, sizeof(buf),
      "{\"t\":%.3f,\"req_v\":%.3f,\"req_w\":%.3f,\"out_v\":%.3f,\"out_w\":%.3f,\"final_v\":%s,"
      "\"v_curve\":%.3f,\"v_clear\":%.3f,\"v_brake\":%.3f,\"v_conf\":%.3f,\"reason\":\"%s\","
      "\"fault\":\"%s\",\"clearance\":%.3f,\"d_free\":%.3f,\"kappa\":%.3f,\"scan_age\":%.3f,"
      "\"costmap_age\":%.3f,\"fwd_frac\":%.3f,\"conf\":%.3f,\"stall_hint\":\"%s\","
      "\"stalled_s\":%.2f,\"compute_us\":%.1f}",
      t.seconds(), v_in, w_in, out.linear.x, out.angular.z, final_str, v_curve,
      v_clear, v_brake, v_conf, kReasonNames[reason], faultName(fault_latched_), clearance, d_free,
      kappa, pin.scan_age, pin.costmap_age, pin.forward_valid_fraction, conf.scale, hint,
      stalled_s_, compute_us);
    std_msgs::msg::String s;
    s.data = buf;
    status_pub_->publish(s);

    if (reason != last_logged_reason_ || fault_latched_ != last_logged_fault_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000, "limit=%s fault=%s req_v=%.2f out_v=%.2f (curve %.2f clear %.2f brake %.2f conf %.2f)",
        kReasonNames[reason], faultName(fault_latched_), v_in, out.linear.x, v_curve, v_clear,
        v_brake, v_conf);
      last_logged_reason_ = reason;
      last_logged_fault_ = fault_latched_;
    }
    (void)w_in;
  }

  // ------------------------------------------------------------ state
  std::string cmd_in_topic_, cmd_out_topic_, final_cmd_topic_, plan_topic_, costmap_topic_,
    scan_topic_, base_frame_;
  double rate_hz_{20.0}, cmd_timeout_{0.5}, tf_timeout_{0.5}, v_max_{0.5}, max_reverse_{0.1},
    accel_{1.5}, decel_{1.5}, emergency_decel_{3.0}, min_speed_{0.05}, omega_min_ratio_{0.5};
  int recovery_cycles_{10};
  CurveProfileParams curve_;
  double path_spacing_{0.1}, curve_lookahead_{2.0}, path_max_offset_{1.0};
  int smooth_window_{5}, curvature_k_{3};
  double robot_radius_{0.22}, clearance_min_{0.03}, clearance_full_{0.3}, clearance_range_{1.0},
    clearance_half_angle_{0.6}, clearance_floor_{0.15};
  OccupancyParams occ_;
  double t_react_{0.15}, brake_decel_{1.0}, safety_margin_{0.05}, look_ahead_max_{2.0};
  ConfidenceParams conf_;
  double scan_half_angle_{0.8}, min_valid_scan_fraction_{0.2};

  geometry_msgs::msg::Twist cmd_;
  rclcpp::Time cmd_time_{0, 0, RCL_ROS_TIME};
  bool have_cmd_{false};
  double final_v_{0.0};
  rclcpp::Time final_time_{0, 0, RCL_ROS_TIME};

  std::vector<Point2> plan_;
  std::vector<double> plan_kappa_;
  std::string plan_frame_{"map"};

  Grid grid_;
  std::string costmap_frame_{"odom"};
  rclcpp::Time costmap_stamp_{0, 0, RCL_ROS_TIME};
  bool have_costmap_{false}, costmap_valid_{false};

  rclcpp::Time scan_stamp_{0, 0, RCL_ROS_TIME};
  bool have_scan_{false}, scan_valid_{false};
  double fwd_fraction_{0.0};

  double prev_v_{0.0};
  Fault fault_latched_{Fault::kNone};
  Reason last_logged_reason_{kNumReasons};
  double startup_speed_{0.0};
  double startup_distance_{1.5};
  bool have_start_{false};
  double start_x_{0.0};
  double start_y_{0.0};
  Fault last_logged_fault_{Fault::kNone};
  int clean_cycles_{0};
  double stalled_s_{0.0};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_, final_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<SpeedGovernor>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("speed_governor"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
