/**
 * @file: rpp_controller.cpp
 * @brief: RPP local planner implementation (merged controller base + RPP, ROS params)
 */
#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>
#include <glog/logging.h>

#include "abot_rpp/rpp_controller.h"

#define R_INFO  LOG(INFO)
#define R_WARN  LOG(WARNING)
#define R_ERROR LOG(ERROR)

PLUGINLIB_EXPORT_CLASS(abot_rpp::RPPController, nav_core::BaseLocalPlanner)

namespace abot_rpp {

static constexpr double kLargeAngleRad = M_PI_2;

// ======================== lifecycle ========================

RPPController::RPPController()
  : initialized_(false), tf_(nullptr), goal_reached_(false),
    costmap_ros_(nullptr), goal_x_(0), goal_y_(0), goal_theta_(0) {}

RPPController::~RPPController() {}

void RPPController::loadParams(ros::NodeHandle& nh) {
  // controller base
  nh.param("control_frequency",          control_frequency_, 20.0);
  nh.param("odom_frame",                 odom_frame_, std::string("odom"));
  nh.param("map_frame",                  map_frame_, std::string("map"));
  nh.param("goal_dist_tolerance",        goal_dist_tolerance_, 0.15);
  nh.param("rotate_tolerance",           rotate_tolerance_, 0.05);
  nh.param("max_linear_velocity",        max_linear_velocity_, 0.6);
  nh.param("min_linear_velocity",        min_linear_velocity_, 0.02);
  nh.param("max_linear_velocity_increment", max_linear_velocity_increment_, 0.3);
  nh.param("max_angular_velocity",       max_angular_velocity_, 1.0);
  nh.param("min_angular_velocity",       min_angular_velocity_, 0.01);
  nh.param("max_angular_velocity_increment", max_angular_velocity_increment_, 0.5);

  // RPP specific
  nh.param("lookahead_time",             lookahead_time_, 0.6);
  nh.param("min_lookahead_dist",         min_lookahead_dist_, 0.25);
  nh.param("max_lookahead_dist",         max_lookahead_dist_, 1.2);
  nh.param("regulated_min_radius",       regulated_min_radius_, 0.3);
  nh.param("inflation_cost_factor",      inflation_cost_factor_, 3.0);
  nh.param("scaling_dist",               scaling_dist_, 0.6);
  nh.param("scaling_gain",               scaling_gain_, 1.0);
  nh.param("approach_dist",              approach_dist_, 0.8);
  nh.param("approach_min_v",             approach_min_v_, 0.05);
}

void RPPController::initialize(std::string name, tf2_ros::Buffer* tf,
                                costmap_2d::Costmap2DROS* costmap_ros) {
  if (initialized_) {
    R_WARN << "RPP already initialized.";
    return;
  }
  initialized_ = true;
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  ros::NodeHandle nh("~/" + name);
  loadParams(nh);

  control_dt_ = 1.0 / control_frequency_;
  odom_helper_ = std::make_shared<base_local_planner::OdometryHelperRos>(odom_frame_);

  target_pt_pub_ = nh.advertise<visualization_msgs::Marker>("target_point", 1);
  current_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("current_pose", 10);

  R_INFO << "RPP Controller initialized (freq=" << control_frequency_
         << " lookahead=" << min_lookahead_dist_ << "~" << max_lookahead_dist_ << ")";
}

bool RPPController::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) {
  if (!initialized_) { R_ERROR << "Not initialized"; return false; }
  R_INFO << "Got new plan (" << plan.size() << " poses)";
  global_plan_ = plan;
  if (goal_x_ != plan.back().pose.position.x ||
      goal_y_ != plan.back().pose.position.y) {
    goal_x_ = plan.back().pose.position.x;
    goal_y_ = plan.back().pose.position.y;
    goal_theta_ = getYawAngle(global_plan_.back());
    goal_reached_ = false;
  }
  return true;
}

bool RPPController::isGoalReached() {
  if (!initialized_) return false;
  if (goal_reached_) { R_INFO << "GOAL Reached!"; return true; }
  return false;
}

// ======================== computeVelocityCommands ========================

bool RPPController::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
  if (!initialized_) { R_ERROR << "Not initialized"; return false; }

  nav_msgs::Odometry base_odom;
  odom_helper_->getOdom(base_odom);

  geometry_msgs::PoseStamped robot_pose_odom, robot_pose_map;
  costmap_ros_->getRobotPose(robot_pose_odom);
  transformPose(tf_, map_frame_, robot_pose_odom, robot_pose_map);

  std::vector<geometry_msgs::PoseStamped> prune_plan = prune(robot_pose_map);

  double vt = std::hypot(base_odom.twist.twist.linear.x, base_odom.twist.twist.linear.y);
  double wt = base_odom.twist.twist.angular.z;
  double L = clamp(std::abs(vt) * lookahead_time_,
                   min_lookahead_dist_, max_lookahead_dist_);

  double kappa;
  Point3d lookahead_pt;
  getLookAheadPoint(L, robot_pose_map, prune_plan, &lookahead_pt, &kappa);

  double dphi = std::atan2(lookahead_pt.y() - robot_pose_map.pose.position.y,
                           lookahead_pt.x() - robot_pose_map.pose.position.x) -
                tf2::getYaw(robot_pose_map.pose.orientation);

  double lookahead_k = 2 * std::sin(dphi) / L;

  if (shouldRotateToGoal(robot_pose_map, global_plan_.back())) {
    double e_theta = normalizeAngle(
        goal_theta_ - tf2::getYaw(robot_pose_map.pose.orientation));
    if (!shouldRotateToPath(std::abs(e_theta))) {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
      goal_reached_ = true;
    } else {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = angularRegularization(wt, e_theta / control_dt_);
    }
  } else {
    double e_theta = normalizeAngle(dphi);
    if (std::abs(e_theta) > kLargeAngleRad) {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = angularRegularization(wt, e_theta / control_dt_);
    } else {
      double curv_vel = applyCurvatureConstraint(max_linear_velocity_, lookahead_k);
      double cost_vel = applyObstacleConstraint(max_linear_velocity_);
      double v_d = std::min(curv_vel, cost_vel);
      v_d = applyApproachConstraint(v_d, robot_pose_map, prune_plan);
      cmd_vel.linear.x = linearRegularization(vt, v_d);
      cmd_vel.angular.z = angularRegularization(wt, v_d * lookahead_k);
    }
  }

  // visualization
  {
    visualization_msgs::Marker m;
    m.header.frame_id = map_frame_; m.header.stamp = ros::Time::now();
    m.ns = "lookahead"; m.id = 0; m.type = visualization_msgs::Marker::SPHERE;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = lookahead_pt.x();
    m.pose.position.y = lookahead_pt.y();
    m.scale.x = m.scale.y = m.scale.z = 0.15;
    m.color.r = 1.0; m.color.a = 1.0;
    target_pt_pub_.publish(m);
  }
  current_pose_pub_.publish(robot_pose_map);

  return true;
}

// ======================== base controller methods ========================

double RPPController::getYawAngle(const geometry_msgs::PoseStamped& ps) {
  return tf2::getYaw(ps.pose.orientation);
}

bool RPPController::shouldRotateToGoal(const geometry_msgs::PoseStamped& cur,
                                        const geometry_msgs::PoseStamped& goal) {
  return std::hypot(cur.pose.position.x - goal.pose.position.x,
                    cur.pose.position.y - goal.pose.position.y) < goal_dist_tolerance_;
}

bool RPPController::shouldRotateToPath(double a) { return a > rotate_tolerance_; }

double RPPController::linearRegularization(double v_in, double v_d) {
  double inc = v_d - v_in;
  if (std::abs(inc) > max_linear_velocity_increment_)
    inc = std::copysign(max_linear_velocity_increment_, inc);
  double v = v_in + inc;
  if (std::abs(v) > max_linear_velocity_)
    v = std::copysign(max_linear_velocity_, v);
  else if (std::abs(v) < min_linear_velocity_)
    v = std::copysign(min_linear_velocity_, v);
  return v;
}

double RPPController::angularRegularization(double w_in, double w_d) {
  if (std::abs(w_d) > max_angular_velocity_)
    w_d = std::copysign(max_angular_velocity_, w_d);
  double inc = w_d - w_in;
  if (std::abs(inc) > max_angular_velocity_increment_)
    inc = std::copysign(max_angular_velocity_increment_, inc);
  double w = w_in + inc;
  if (std::abs(w) > max_angular_velocity_)
    w = std::copysign(max_angular_velocity_, w);
  else if (std::abs(w) < min_angular_velocity_)
    w = std::copysign(min_angular_velocity_, w);
  return w;
}

void RPPController::transformPose(tf2_ros::Buffer* tf, const std::string& out_frame,
                                   const geometry_msgs::PoseStamped& in_pose,
                                   geometry_msgs::PoseStamped& out_pose) {
  if (in_pose.header.frame_id == out_frame) { out_pose = in_pose; return; }
  tf->transform(in_pose, out_pose, out_frame);
  out_pose.header.frame_id = out_frame;
}

std::vector<geometry_msgs::PoseStamped>
RPPController::prune(const geometry_msgs::PoseStamped& robot_pose_map) {
  auto dist = [](const geometry_msgs::PoseStamped& a,
                 const geometry_msgs::PoseStamped& b) {
    return std::hypot(a.pose.position.x - b.pose.position.x,
                      a.pose.position.y - b.pose.position.y);
  };

  // find closest point to robot on global plan
  double min_d = std::numeric_limits<double>::max();
  size_t closest = 0;
  for (size_t i = 0; i < global_plan_.size(); i++) {
    double d = dist(robot_pose_map, global_plan_[i]);
    if (d < min_d) { min_d = d; closest = i; }
  }

  std::vector<geometry_msgs::PoseStamped> result;
  for (size_t i = closest; i < global_plan_.size(); i++)
    result.push_back(global_plan_[i]);

  global_plan_.erase(global_plan_.begin(), global_plan_.begin() + closest);
  return result;
}

void RPPController::getLookAheadPoint(
    double L, const geometry_msgs::PoseStamped& robot_pose,
    const std::vector<geometry_msgs::PoseStamped>& prune_plan,
    Point3d* pt, double* kappa) {
  double rx = robot_pose.pose.position.x;
  double ry = robot_pose.pose.position.y;

  auto it = std::find_if(prune_plan.begin(), prune_plan.end(),
      [&](const geometry_msgs::PoseStamped& ps) {
        return std::hypot(ps.pose.position.x - rx, ps.pose.position.y - ry) >= L;
      });

  if (it == prune_plan.end()) {
    it = std::prev(prune_plan.end());
    pt->setX(it->pose.position.x);
    pt->setY(it->pose.position.y);
    pt->setTheta(std::atan2(pt->y() - ry, pt->x() - rx));
    *kappa = 0.0;
    return;
  }

  double px, py, gx = it->pose.position.x, gy = it->pose.position.y;
  if (it == prune_plan.begin()) { px = rx; py = ry; }
  else { auto p = std::prev(it); px = p->pose.position.x; py = p->pose.position.y; }

  Vec2d prev_p(px - rx, py - ry), goal_p(gx - rx, gy - ry);
  auto i_points = circleSegmentIntersection(prev_p, goal_p, L);

  double best_d = std::numeric_limits<double>::max();
  for (const auto& ip : i_points) {
    double d = std::hypot(ip.x() + rx - gx, ip.y() + ry - gy);
    if (d < best_d) { best_d = d; pt->setX(ip.x() + rx); pt->setY(ip.y() + ry); }
  }

  auto next_it = std::next(it);
  if (next_it != prune_plan.end()) {
    Vec2d p1(px, py), p2(gx, gy), p3(next_it->pose.position.x, next_it->pose.position.y);
    *kappa = arcCenter(p1, p2, p3, false);
  } else { *kappa = 0.0; }

  pt->setTheta(std::atan2(gy - py, gx - px));
}

// ======================== RPP constraints ========================

double RPPController::applyCurvatureConstraint(double raw_v, double curvature) {
  if (std::abs(curvature) < 1e-10) return raw_v;
  double radius = std::abs(1.0 / curvature);
  return radius < regulated_min_radius_ ? raw_v * (radius / regulated_min_radius_) : raw_v;
}

double RPPController::applyObstacleConstraint(double raw_v) {
  int cx = costmap_ros_->getCostmap()->getSizeInCellsX() / 2;
  int cy = costmap_ros_->getCostmap()->getSizeInCellsY() / 2;
  double robot_cost = static_cast<double>(costmap_ros_->getCostmap()->getCost(cx, cy));

  if (robot_cost == static_cast<double>(costmap_2d::FREE_SPACE) ||
      robot_cost == static_cast<double>(costmap_2d::NO_INFORMATION))
    return raw_v;

  double inscr_radius = costmap_ros_->getLayeredCostmap()->getInscribedRadius();
  double obs_dist = inscr_radius -
      (std::log(robot_cost) - std::log(static_cast<double>(costmap_2d::INSCRIBED_INFLATED_OBSTACLE)))
      / inflation_cost_factor_;

  if (obs_dist < scaling_dist_)
    return raw_v * scaling_gain_ * obs_dist / scaling_dist_;
  return raw_v;
}

double RPPController::applyApproachConstraint(
    double raw_v, const geometry_msgs::PoseStamped& robot_pose,
    const std::vector<geometry_msgs::PoseStamped>& prune_plan) {
  auto d = [](const geometry_msgs::PoseStamped& a, const geometry_msgs::PoseStamped& b) {
    return std::hypot(a.pose.position.x - b.pose.position.x,
                      a.pose.position.y - b.pose.position.y);
  };
  double remain = 0.0;
  for (size_t i = 0; i + 1 < prune_plan.size(); i++) remain += d(prune_plan[i], prune_plan[i + 1]);
  double s = remain < approach_dist_ ?
             d(prune_plan.back(), robot_pose) / approach_dist_ : 1.0;
  return std::min(raw_v, std::max(approach_min_v_, raw_v * s));
}

}  // namespace abot_rpp
