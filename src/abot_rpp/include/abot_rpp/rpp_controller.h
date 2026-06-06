/**
 * @file: rpp_controller.h
 * @brief: Regulated Pure Pursuit local planner (standalone, no external deps)
 *         Ported from ros_motion_planning, protobuf config → ROS params
 */
#ifndef ABOT_RPP_CONTROLLER_H_
#define ABOT_RPP_CONTROLLER_H_

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include "abot_rpp/rpp_math.h"

namespace abot_rpp {

class RPPController : public nav_core::BaseLocalPlanner {
public:
  RPPController();
  ~RPPController();

  void initialize(std::string name, tf2_ros::Buffer* tf,
                  costmap_2d::Costmap2DROS* costmap_ros) override;
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override;
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;
  bool isGoalReached() override;

private:
  // --- config from ROS params ---
  void loadParams(ros::NodeHandle& nh);

  // --- controller base methods (inlined from ros_motion_planning) ---
  double getYawAngle(const geometry_msgs::PoseStamped& ps);
  bool shouldRotateToGoal(const geometry_msgs::PoseStamped& cur,
                          const geometry_msgs::PoseStamped& goal);
  bool shouldRotateToPath(double angle_to_path);
  double linearRegularization(double v_in, double v_d);
  double angularRegularization(double w_in, double w_d);
  void transformPose(tf2_ros::Buffer* tf, const std::string& out_frame,
                     const geometry_msgs::PoseStamped& in_pose,
                     geometry_msgs::PoseStamped& out_pose);
  std::vector<geometry_msgs::PoseStamped>
  prune(const geometry_msgs::PoseStamped& robot_pose_map);
  void getLookAheadPoint(double L, const geometry_msgs::PoseStamped& robot_pose,
                         const std::vector<geometry_msgs::PoseStamped>& prune_plan,
                         Point3d* pt, double* kappa);

  // --- RPP constraint methods ---
  double applyCurvatureConstraint(double raw_v, double curvature);
  double applyObstacleConstraint(double raw_v);
  double applyApproachConstraint(double raw_v, const geometry_msgs::PoseStamped& robot_pose,
                                  const std::vector<geometry_msgs::PoseStamped>& prune_plan);

  // --- state ---
  bool initialized_;
  bool goal_reached_;
  tf2_ros::Buffer* tf_;
  costmap_2d::Costmap2DROS* costmap_ros_;
  std::shared_ptr<base_local_planner::OdometryHelperRos> odom_helper_;
  std::vector<geometry_msgs::PoseStamped> global_plan_;

  double goal_x_, goal_y_, goal_theta_;
  double control_dt_;

  // --- publishers ---
  ros::Publisher target_pt_pub_, current_pose_pub_;

  // ======== ROS params (replaces protobuf) ========

  // -- controller base params --
  double control_frequency_;
  std::string odom_frame_, map_frame_;
  double goal_dist_tolerance_;
  double rotate_tolerance_;
  double max_linear_velocity_, min_linear_velocity_;
  double max_linear_velocity_increment_;
  double max_angular_velocity_, min_angular_velocity_;
  double max_angular_velocity_increment_;

  // -- RPP-specific params --
  double lookahead_time_;
  double min_lookahead_dist_, max_lookahead_dist_;
  double regulated_min_radius_;
  double inflation_cost_factor_;
  double scaling_dist_, scaling_gain_;
  double approach_dist_, approach_min_v_;
};

}  // namespace abot_rpp
#endif
