#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <base_local_planner/costmap_model.h>
#include <costmap_2d/footprint.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <global_planner/planner_core.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf/tf.h>

namespace robot_slam {

class SmoothGlobalPlanner : public nav_core::BaseGlobalPlanner {
public:
  SmoothGlobalPlanner()
      : initialized_(false), costmap_ros_(NULL), costmap_(NULL),
        inscribed_radius_(0.0), circumscribed_radius_(0.0),
        enabled_(true), sample_step_(0.02), sparse_distance_(0.06),
        sharp_turn_threshold_rad_(65.0 * kPi / 180.0),
        smooth_radius_(0.18), min_endpoint_distance_(0.20),
        max_radius_fraction_(0.45), collision_check_unknown_as_blocked_(true) {}

  void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros) {
    if (initialized_) {
      ROS_WARN("[SmoothGlobalPlanner] already initialized");
      return;
    }

    name_ = name;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    footprint_spec_ = costmap_ros_->getRobotFootprint();
    if (!footprint_spec_.empty()) {
      costmap_2d::calculateMinAndMaxDistances(footprint_spec_,
                                              inscribed_radius_,
                                              circumscribed_radius_);
    }
    world_model_.reset(new base_local_planner::CostmapModel(*costmap_));
    inner_planner_.reset(new global_planner::GlobalPlanner());

    ros::NodeHandle private_nh("~/" + name_);
    double sharp_turn_threshold_deg = 65.0;
    private_nh.param("enabled", enabled_, true);
    private_nh.param("sample_step", sample_step_, 0.02);
    private_nh.param("sparse_distance", sparse_distance_, 0.06);
    private_nh.param("sharp_turn_threshold_deg", sharp_turn_threshold_deg, 65.0);
    private_nh.param("smooth_radius", smooth_radius_, 0.18);
    private_nh.param("min_endpoint_distance", min_endpoint_distance_, 0.20);
    private_nh.param("max_radius_fraction", max_radius_fraction_, 0.45);
    private_nh.param("collision_check_unknown_as_blocked",
                     collision_check_unknown_as_blocked_, true);
    sharp_turn_threshold_rad_ = sharp_turn_threshold_deg * kPi / 180.0;

    sample_step_ = std::max(0.005, sample_step_);
    sparse_distance_ = std::max(0.01, sparse_distance_);
    smooth_radius_ = std::max(0.01, smooth_radius_);
    min_endpoint_distance_ = std::max(0.0, min_endpoint_distance_);
    max_radius_fraction_ = std::max(0.10, std::min(0.90, max_radius_fraction_));

    raw_plan_pub_ = private_nh.advertise<nav_msgs::Path>("raw_plan", 1, true);
    smooth_plan_pub_ = private_nh.advertise<nav_msgs::Path>("smooth_plan", 1, true);
    rejected_points_pub_ =
        private_nh.advertise<geometry_msgs::PoseArray>("rejected_points", 1, true);

    // Keep the wrapped planner under the original namespace so existing
    // GlobalPlanner parameters keep working unchanged.
    inner_planner_->initialize("GlobalPlanner", costmap_ros_);

    initialized_ = true;
    ROS_INFO("[SmoothGlobalPlanner] initialized enabled=%s sample=%.3f sparse=%.3f "
             "threshold=%.1fdeg radius=%.3f endpoint_guard=%.3f unknown_blocked=%s",
             enabled_ ? "true" : "false", sample_step_, sparse_distance_,
             sharp_turn_threshold_deg, smooth_radius_, min_endpoint_distance_,
             collision_check_unknown_as_blocked_ ? "true" : "false");
  }

  bool makePlan(const geometry_msgs::PoseStamped &start,
                const geometry_msgs::PoseStamped &goal,
                std::vector<geometry_msgs::PoseStamped> &plan) {
    if (!initialized_) {
      ROS_ERROR("[SmoothGlobalPlanner] makePlan called before initialize");
      return false;
    }

    std::vector<geometry_msgs::PoseStamped> raw_plan;
    if (!inner_planner_->makePlan(start, goal, raw_plan)) {
      publishPlan(raw_plan, raw_plan_pub_);
      ROS_WARN("[SmoothGlobalPlanner] wrapped GlobalPlanner failed");
      return false;
    }

    publishPlan(raw_plan, raw_plan_pub_);

    if (!enabled_ || raw_plan.size() < 3) {
      plan = raw_plan;
      publishPlan(plan, smooth_plan_pub_);
      return true;
    }

    geometry_msgs::PoseArray rejected_points;
    rejected_points.header = raw_plan.front().header;

    bool changed = smoothPlan(raw_plan, plan, rejected_points);
    publishPlan(plan, smooth_plan_pub_);
    rejected_points_pub_.publish(rejected_points);

    ROS_INFO("[SmoothGlobalPlanner] plan raw=%zu smooth=%zu changed=%s rejected=%zu",
             raw_plan.size(), plan.size(), changed ? "true" : "false",
             rejected_points.poses.size());
    return true;
  }

  bool makePlan(const geometry_msgs::PoseStamped &start,
                const geometry_msgs::PoseStamped &goal,
                std::vector<geometry_msgs::PoseStamped> &plan, double &cost) {
    cost = 0.0;
    return makePlan(start, goal, plan);
  }

private:
  struct SparsePoint {
    size_t index;
    double x;
    double y;
    double path_s;
  };

  struct SharpTurn {
    size_t index;
    double angle;
    double prev_dist;
    double next_dist;
  };

  struct CutPoint {
    geometry_msgs::PoseStamped pose;
    size_t copy_until_index;
    size_t resume_index;
  };

  struct BezierSegment {
    size_t corner_index;
    CutPoint start;
    CutPoint end;
    std::vector<geometry_msgs::PoseStamped> samples;
  };

  static constexpr double kPi = 3.14159265358979323846;

  bool smoothPlan(const std::vector<geometry_msgs::PoseStamped> &raw_plan,
                  std::vector<geometry_msgs::PoseStamped> &smoothed_plan,
                  geometry_msgs::PoseArray &rejected_points) {
    smoothed_plan.clear();

    std::vector<SharpTurn> turns = findSharpTurns(raw_plan);
    if (turns.empty()) {
      smoothed_plan = raw_plan;
      return false;
    }

    std::vector<BezierSegment> accepted_segments;
    size_t last_resume_index = 0;
    for (size_t i = 0; i < turns.size(); ++i) {
      const SharpTurn &turn = turns[i];
      if (turn.index <= last_resume_index) {
        continue;
      }

      BezierSegment segment;
      std::string reject_reason;
      if (tryCreateBezierSegment(raw_plan, turn, segment, reject_reason)) {
        if (segment.start.copy_until_index < last_resume_index) {
          ROS_WARN("[SmoothGlobalPlanner][smooth_skip_overlap] index=%zu start=%zu last_resume=%zu",
                   turn.index, segment.start.copy_until_index, last_resume_index);
          continue;
        }
        accepted_segments.push_back(segment);
        last_resume_index = segment.end.resume_index;
        ROS_INFO("[SmoothGlobalPlanner][smooth_accept] index=%zu angle=%.1fdeg "
                 "samples=%zu start=(%.3f,%.3f) end=(%.3f,%.3f)",
                 turn.index, turn.angle * 180.0 / kPi, segment.samples.size(),
                 segment.start.pose.pose.position.x, segment.start.pose.pose.position.y,
                 segment.end.pose.pose.position.x, segment.end.pose.pose.position.y);
      } else {
        geometry_msgs::Pose pose = raw_plan[turn.index].pose;
        rejected_points.poses.push_back(pose);
        ROS_WARN("[SmoothGlobalPlanner][smooth_reject_collision] index=%zu "
                 "angle=%.1fdeg reason=%s point=(%.3f,%.3f)",
                 turn.index, turn.angle * 180.0 / kPi, reject_reason.c_str(),
                 pose.position.x, pose.position.y);
      }
    }

    if (accepted_segments.empty()) {
      smoothed_plan = raw_plan;
      return false;
    }

    size_t copy_from = 0;
    for (size_t s = 0; s < accepted_segments.size(); ++s) {
      const BezierSegment &segment = accepted_segments[s];
      appendRawRange(raw_plan, copy_from, segment.start.copy_until_index, smoothed_plan);
      appendPose(segment.start.pose, smoothed_plan);
      for (size_t i = 1; i + 1 < segment.samples.size(); ++i) {
        appendPose(segment.samples[i], smoothed_plan);
      }
      appendPose(segment.end.pose, smoothed_plan);
      copy_from = segment.end.resume_index;
    }
    appendRawRange(raw_plan, copy_from, raw_plan.size(), smoothed_plan);
    refreshOrientations(smoothed_plan);
    return true;
  }

  std::vector<SharpTurn>
  findSharpTurns(const std::vector<geometry_msgs::PoseStamped> &plan) const {
    std::vector<SparsePoint> sparse;
    buildSparsePlan(plan, sparse);

    std::vector<SharpTurn> turns;
    if (sparse.size() < 3) {
      return turns;
    }

    const double total_s = sparse.back().path_s;
    for (size_t i = 1; i + 1 < sparse.size(); ++i) {
      if (sparse[i].path_s < min_endpoint_distance_ ||
          total_s - sparse[i].path_s < min_endpoint_distance_) {
        continue;
      }

      double in_x = sparse[i].x - sparse[i - 1].x;
      double in_y = sparse[i].y - sparse[i - 1].y;
      double out_x = sparse[i + 1].x - sparse[i].x;
      double out_y = sparse[i + 1].y - sparse[i].y;
      const double in_len = std::hypot(in_x, in_y);
      const double out_len = std::hypot(out_x, out_y);
      if (in_len < 1e-6 || out_len < 1e-6) {
        continue;
      }

      double dot = (in_x * out_x + in_y * out_y) / (in_len * out_len);
      dot = std::max(-1.0, std::min(1.0, dot));
      const double angle = std::acos(dot);
      if (angle >= sharp_turn_threshold_rad_) {
        SharpTurn turn;
        turn.index = sparse[i].index;
        turn.angle = angle;
        turn.prev_dist = in_len;
        turn.next_dist = out_len;
        turns.push_back(turn);
        ROS_WARN("[SmoothGlobalPlanner][sharp_turn_detected] index=%zu "
                 "angle=%.1fdeg point=(%.3f,%.3f)",
                 turn.index, angle * 180.0 / kPi, sparse[i].x, sparse[i].y);
      }
    }
    return turns;
  }

  void buildSparsePlan(const std::vector<geometry_msgs::PoseStamped> &plan,
                       std::vector<SparsePoint> &sparse) const {
    sparse.clear();
    if (plan.empty()) {
      return;
    }

    double path_s = 0.0;
    double last_kept_x = plan.front().pose.position.x;
    double last_kept_y = plan.front().pose.position.y;
    SparsePoint first;
    first.index = 0;
    first.x = last_kept_x;
    first.y = last_kept_y;
    first.path_s = 0.0;
    sparse.push_back(first);

    for (size_t i = 1; i < plan.size(); ++i) {
      const double x = plan[i].pose.position.x;
      const double y = plan[i].pose.position.y;
      path_s += distance(plan[i - 1], plan[i]);
      if (std::hypot(x - last_kept_x, y - last_kept_y) >= sparse_distance_ ||
          i + 1 == plan.size()) {
        SparsePoint p;
        p.index = i;
        p.x = x;
        p.y = y;
        p.path_s = path_s;
        sparse.push_back(p);
        last_kept_x = x;
        last_kept_y = y;
      }
    }
  }

  bool tryCreateBezierSegment(
      const std::vector<geometry_msgs::PoseStamped> &plan, const SharpTurn &turn,
      BezierSegment &segment, std::string &reject_reason) {
    const double radius = std::min(
        smooth_radius_, max_radius_fraction_ * std::min(turn.prev_dist, turn.next_dist));
    if (radius < 0.04) {
      reject_reason = "radius_too_small";
      return false;
    }

    CutPoint start_cut;
    CutPoint end_cut;
    if (!findBackwardCut(plan, turn.index, radius, start_cut) ||
        !findForwardCut(plan, turn.index, radius, end_cut)) {
      reject_reason = "cut_unavailable";
      return false;
    }
    if (end_cut.resume_index <= start_cut.copy_until_index + 1) {
      reject_reason = "cut_overlap";
      return false;
    }

    std::vector<geometry_msgs::PoseStamped> samples;
    if (!sampleBezier(plan[turn.index], start_cut.pose, end_cut.pose, samples,
                      reject_reason)) {
      return false;
    }

    segment.corner_index = turn.index;
    segment.start = start_cut;
    segment.end = end_cut;
    segment.samples = samples;
    return true;
  }

  bool findBackwardCut(const std::vector<geometry_msgs::PoseStamped> &plan,
                       size_t corner_index, double radius, CutPoint &cut) const {
    double accum = 0.0;
    for (size_t j = corner_index; j > 0; --j) {
      const double seg_len = distance(plan[j], plan[j - 1]);
      if (seg_len < 1e-6) {
        continue;
      }
      if (accum + seg_len >= radius) {
        const double ratio = (radius - accum) / seg_len;
        cut.pose = interpolatePose(plan[j], plan[j - 1], ratio);
        cut.copy_until_index = j;
        cut.resume_index = j;
        return true;
      }
      accum += seg_len;
    }
    return false;
  }

  bool findForwardCut(const std::vector<geometry_msgs::PoseStamped> &plan,
                      size_t corner_index, double radius, CutPoint &cut) const {
    double accum = 0.0;
    for (size_t j = corner_index; j + 1 < plan.size(); ++j) {
      const double seg_len = distance(plan[j], plan[j + 1]);
      if (seg_len < 1e-6) {
        continue;
      }
      if (accum + seg_len >= radius) {
        const double ratio = (radius - accum) / seg_len;
        cut.pose = interpolatePose(plan[j], plan[j + 1], ratio);
        cut.copy_until_index = j;
        cut.resume_index = j + 1;
        return true;
      }
      accum += seg_len;
    }
    return false;
  }

  bool sampleBezier(const geometry_msgs::PoseStamped &corner,
                    const geometry_msgs::PoseStamped &start,
                    const geometry_msgs::PoseStamped &end,
                    std::vector<geometry_msgs::PoseStamped> &samples,
                    std::string &reject_reason) const {
    samples.clear();

    const double ax = start.pose.position.x;
    const double ay = start.pose.position.y;
    const double px = corner.pose.position.x;
    const double py = corner.pose.position.y;
    const double bx = end.pose.position.x;
    const double by = end.pose.position.y;

    double in_x = px - ax;
    double in_y = py - ay;
    double out_x = bx - px;
    double out_y = by - py;
    const double in_len = std::hypot(in_x, in_y);
    const double out_len = std::hypot(out_x, out_y);
    if (in_len < 1e-6 || out_len < 1e-6) {
      reject_reason = "degenerate_tangent";
      return false;
    }
    in_x /= in_len;
    in_y /= in_len;
    out_x /= out_len;
    out_y /= out_len;

    const double control = 0.55 * std::min(in_len, out_len);
    const double c1x = ax + in_x * control;
    const double c1y = ay + in_y * control;
    const double c2x = bx - out_x * control;
    const double c2y = by - out_y * control;
    const int sample_count =
        std::max(3, static_cast<int>(std::ceil((in_len + out_len) / sample_step_)) + 1);

    for (int i = 0; i < sample_count; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(sample_count - 1);
      const double omt = 1.0 - t;
      const double x = omt * omt * omt * ax + 3.0 * omt * omt * t * c1x +
                       3.0 * omt * t * t * c2x + t * t * t * bx;
      const double y = omt * omt * omt * ay + 3.0 * omt * omt * t * c1y +
                       3.0 * omt * t * t * c2y + t * t * t * by;

      const double dx = 3.0 * omt * omt * (c1x - ax) +
                        6.0 * omt * t * (c2x - c1x) +
                        3.0 * t * t * (bx - c2x);
      const double dy = 3.0 * omt * omt * (c1y - ay) +
                        6.0 * omt * t * (c2y - c1y) +
                        3.0 * t * t * (by - c2y);
      const double yaw = std::atan2(dy, dx);

      if (!isFootprintSafe(x, y, yaw, reject_reason)) {
        return false;
      }

      geometry_msgs::PoseStamped pose = start;
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
      samples.push_back(pose);
    }

    return true;
  }

  bool isFootprintSafe(double x, double y, double yaw,
                       std::string &reject_reason) const {
    if (!world_model_) {
      reject_reason = "world_model_unavailable";
      return false;
    }
    if (footprint_spec_.empty()) {
      reject_reason = "empty_footprint";
      return false;
    }

    const double cost = world_model_->footprintCost(x, y, yaw, footprint_spec_,
                                                    inscribed_radius_,
                                                    circumscribed_radius_);
    if (cost >= 0.0) {
      return true;
    }
    if (cost == -2.0 && !collision_check_unknown_as_blocked_) {
      return true;
    }

    if (cost == -1.0) {
      reject_reason = "lethal_obstacle";
    } else if (cost == -2.0) {
      reject_reason = "unknown_space";
    } else if (cost == -3.0) {
      reject_reason = "out_of_map";
    } else {
      reject_reason = "negative_cost";
    }
    return false;
  }

  static geometry_msgs::PoseStamped
  interpolatePose(const geometry_msgs::PoseStamped &from,
                  const geometry_msgs::PoseStamped &to, double ratio) {
    const double r = std::max(0.0, std::min(1.0, ratio));
    geometry_msgs::PoseStamped pose = from;
    pose.pose.position.x =
        from.pose.position.x + (to.pose.position.x - from.pose.position.x) * r;
    pose.pose.position.y =
        from.pose.position.y + (to.pose.position.y - from.pose.position.y) * r;
    pose.pose.position.z = 0.0;
    const double yaw =
        std::atan2(to.pose.position.y - from.pose.position.y,
                   to.pose.position.x - from.pose.position.x);
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    return pose;
  }

  static double distance(const geometry_msgs::PoseStamped &a,
                         const geometry_msgs::PoseStamped &b) {
    return std::hypot(a.pose.position.x - b.pose.position.x,
                      a.pose.position.y - b.pose.position.y);
  }

  void appendRawRange(const std::vector<geometry_msgs::PoseStamped> &raw,
                      size_t begin, size_t end,
                      std::vector<geometry_msgs::PoseStamped> &out) const {
    const size_t safe_end = std::min(end, raw.size());
    for (size_t i = begin; i < safe_end; ++i) {
      appendPose(raw[i], out);
    }
  }

  void appendPose(const geometry_msgs::PoseStamped &pose,
                  std::vector<geometry_msgs::PoseStamped> &out) const {
    if (!out.empty() && distance(out.back(), pose) < 1e-4) {
      return;
    }
    out.push_back(pose);
  }

  void refreshOrientations(std::vector<geometry_msgs::PoseStamped> &plan) const {
    if (plan.size() < 2) {
      return;
    }
    for (size_t i = 0; i + 1 < plan.size(); ++i) {
      const double yaw =
          std::atan2(plan[i + 1].pose.position.y - plan[i].pose.position.y,
                     plan[i + 1].pose.position.x - plan[i].pose.position.x);
      plan[i].pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    }
    plan.back().pose.orientation = plan[plan.size() - 2].pose.orientation;
  }

  void publishPlan(const std::vector<geometry_msgs::PoseStamped> &plan,
                   const ros::Publisher &publisher) const {
    if (!publisher) {
      return;
    }
    nav_msgs::Path path;
    if (!plan.empty()) {
      path.header = plan.front().header;
    } else {
      path.header.frame_id = costmap_ros_ ? costmap_ros_->getGlobalFrameID() : "map";
      path.header.stamp = ros::Time::now();
    }
    path.poses = plan;
    publisher.publish(path);
  }

  bool initialized_;
  std::string name_;
  costmap_2d::Costmap2DROS *costmap_ros_;
  costmap_2d::Costmap2D *costmap_;
  std::vector<geometry_msgs::Point> footprint_spec_;
  double inscribed_radius_;
  double circumscribed_radius_;
  std::unique_ptr<base_local_planner::CostmapModel> world_model_;
  std::unique_ptr<global_planner::GlobalPlanner> inner_planner_;

  bool enabled_;
  double sample_step_;
  double sparse_distance_;
  double sharp_turn_threshold_rad_;
  double smooth_radius_;
  double min_endpoint_distance_;
  double max_radius_fraction_;
  bool collision_check_unknown_as_blocked_;

  ros::Publisher raw_plan_pub_;
  ros::Publisher smooth_plan_pub_;
  ros::Publisher rejected_points_pub_;
};

} // namespace robot_slam

PLUGINLIB_EXPORT_CLASS(robot_slam::SmoothGlobalPlanner, nav_core::BaseGlobalPlanner)
