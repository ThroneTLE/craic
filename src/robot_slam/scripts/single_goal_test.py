#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
import actionlib
import tf
from geometry_msgs.msg import Twist, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus


def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_to_quat(yaw):
    q = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
    return q


class SingleGoalTest:
    def __init__(self):
        rospy.init_node("single_goal_test")

        # ====== 目标点参数，先在 launch/命令行里传，也可以直接改默认值 ======
        self.goal_x = rospy.get_param("~goal_x", 0.0)
        self.goal_y = rospy.get_param("~goal_y", 0.0)
        self.goal_yaw = rospy.get_param("~goal_yaw", 0.0)

        # move_base 到点判定
        self.nav_timeout = rospy.get_param("~nav_timeout", 60.0)

        # 精修阶段参数
        self.enable_fine_adjust = rospy.get_param("~enable_fine_adjust", True)
        self.xy_tolerance = rospy.get_param("~xy_tolerance", 0.025)
        self.yaw_tolerance = rospy.get_param("~yaw_tolerance", 0.05)

        self.max_fine_vx = rospy.get_param("~max_fine_vx", 0.05)
        self.max_fine_vy = rospy.get_param("~max_fine_vy", 0.05)
        self.max_fine_wz = rospy.get_param("~max_fine_wz", 0.18)

        self.kp_xy = rospy.get_param("~kp_xy", 0.6)
        self.kp_yaw = rospy.get_param("~kp_yaw", 1.0)

        self.fine_timeout = rospy.get_param("~fine_timeout", 8.0)

        self.base_frame = rospy.get_param("~base_frame", "base_footprint")
        self.map_frame = rospy.get_param("~map_frame", "map")

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.tf_listener = tf.TransformListener()

        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")
        ok = self.client.wait_for_server(rospy.Duration(10.0))
        if not ok:
            rospy.logerr("move_base action server not available.")
            return

        rospy.sleep(1.0)

        rospy.loginfo("Single goal test target: x=%.3f y=%.3f yaw=%.3f rad",
                      self.goal_x, self.goal_y, self.goal_yaw)

        self.run()

    def run(self):
        success = self.send_move_base_goal()

        if not success:
            rospy.logwarn("move_base failed or timeout. Stop robot.")
            self.stop_robot()
            return

        rospy.loginfo("move_base reached goal region.")

        if self.enable_fine_adjust:
            rospy.loginfo("Start fine adjustment.")
            fine_ok = self.fine_adjust()
            if fine_ok:
                rospy.loginfo("Fine adjustment done.")
            else:
                rospy.logwarn("Fine adjustment timeout or failed.")

        self.stop_robot()
        rospy.loginfo("Single goal test finished.")

    def send_move_base_goal(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.map_frame
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = self.goal_x
        goal.target_pose.pose.position.y = self.goal_y
        goal.target_pose.pose.position.z = 0.0

        q = yaw_to_quat(self.goal_yaw)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        rospy.loginfo("Sending move_base goal...")
        self.client.send_goal(goal)

        finished = self.client.wait_for_result(rospy.Duration(self.nav_timeout))
        if not finished:
            rospy.logwarn("move_base timeout, cancel goal.")
            self.client.cancel_goal()
            return False

        state = self.client.get_state()
        rospy.loginfo("move_base result state: %s", str(state))

        return state == GoalStatus.SUCCEEDED

    def lookup_robot_pose(self):
        try:
            self.tf_listener.waitForTransform(
                self.map_frame,
                self.base_frame,
                rospy.Time(0),
                rospy.Duration(0.5)
            )
            trans, rot = self.tf_listener.lookupTransform(
                self.map_frame,
                self.base_frame,
                rospy.Time(0)
            )
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)
            return trans[0], trans[1], yaw
        except Exception as e:
            rospy.logwarn_throttle(1.0, "TF lookup failed: %s", str(e))
            return None

    def fine_adjust(self):
        start = rospy.Time.now()
        rate = rospy.Rate(20)

        stable_count = 0
        required_stable_count = 10

        while not rospy.is_shutdown():
            if (rospy.Time.now() - start).to_sec() > self.fine_timeout:
                self.stop_robot()
                return False

            pose = self.lookup_robot_pose()
            if pose is None:
                self.stop_robot()
                rate.sleep()
                continue

            rx, ry, ryaw = pose

            # map 坐标系下误差
            ex_map = self.goal_x - rx
            ey_map = self.goal_y - ry
            eyaw = normalize_angle(self.goal_yaw - ryaw)

            # 把 map 坐标误差转换到机器人自身坐标系
            # cmd_vel 是 base_footprint 坐标系下的速度
            cos_yaw = math.cos(ryaw)
            sin_yaw = math.sin(ryaw)

            ex_base = cos_yaw * ex_map + sin_yaw * ey_map
            ey_base = -sin_yaw * ex_map + cos_yaw * ey_map

            dist = math.sqrt(ex_map * ex_map + ey_map * ey_map)

            if dist < self.xy_tolerance and abs(eyaw) < self.yaw_tolerance:
                stable_count += 1
                self.stop_robot()
                if stable_count >= required_stable_count:
                    return True
            else:
                stable_count = 0

                cmd = Twist()
                cmd.linear.x = self.clamp(self.kp_xy * ex_base,
                                          -self.max_fine_vx,
                                          self.max_fine_vx)
                cmd.linear.y = self.clamp(self.kp_xy * ey_base,
                                          -self.max_fine_vy,
                                          self.max_fine_vy)
                cmd.angular.z = self.clamp(self.kp_yaw * eyaw,
                                           -self.max_fine_wz,
                                           self.max_fine_wz)

                # 死区，防止小误差抖动
                if abs(cmd.linear.x) < 0.006:
                    cmd.linear.x = 0.0
                if abs(cmd.linear.y) < 0.006:
                    cmd.linear.y = 0.0
                if abs(cmd.angular.z) < 0.015:
                    cmd.angular.z = 0.0

                self.cmd_pub.publish(cmd)

            rate.sleep()

        return False

    def stop_robot(self):
        cmd = Twist()
        for _ in range(5):
            self.cmd_pub.publish(cmd)
            rospy.sleep(0.03)

    @staticmethod
    def clamp(v, vmin, vmax):
        return max(vmin, min(vmax, v))


if __name__ == "__main__":
    try:
        SingleGoalTest()
    except rospy.ROSInterruptException:
        pass
