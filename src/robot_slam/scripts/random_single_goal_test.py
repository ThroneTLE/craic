#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import random
import rospy
import actionlib
import tf

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus


def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_to_quat(yaw):
    return tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)


class RandomSingleGoalTest:
    def __init__(self):
        rospy.init_node("random_single_goal_test")

        # =====================================================
        # 目标框中心
        # =====================================================
        self.target_x = rospy.get_param("~target_x", 0.0)
        self.target_y = rospy.get_param("~target_y", 0.0)
        self.target_yaw = rospy.get_param("~target_yaw", 0.0)

        # =====================================================
        # 候选入口点参数
        # =====================================================
        self.candidate_count = rospy.get_param("~candidate_count", 8)
        self.min_radius = rospy.get_param("~min_radius", 0.40)
        self.max_radius = rospy.get_param("~max_radius", 0.60)

        # 每个候选入口给 move_base 的最大时间
        self.nav_timeout = rospy.get_param("~nav_timeout", 8.0)

        # =====================================================
        # COMMIT 安全进入参数
        # =====================================================
        self.enable_safe_enter = rospy.get_param("~enable_safe_enter", True)
        self.safe_enter_timeout = rospy.get_param("~safe_enter_timeout", 18.0)

        self.xy_tolerance = rospy.get_param("~xy_tolerance", 0.04)
        self.yaw_tolerance = rospy.get_param("~yaw_tolerance", 0.12)

        self.kp_xy = rospy.get_param("~kp_xy", 0.35)
        self.kp_yaw = rospy.get_param("~kp_yaw", 0.60)

        self.max_enter_vx = rospy.get_param("~max_enter_vx", 0.018)
        self.max_enter_vy = rospy.get_param("~max_enter_vy", 0.018)
        self.max_enter_wz = rospy.get_param("~max_enter_wz", 0.06)

        # 雷达安全距离
        self.front_stop_dist = rospy.get_param("~front_stop_dist", 0.16)
        self.front_slow_dist = rospy.get_param("~front_slow_dist", 0.26)
        self.side_stop_dist = rospy.get_param("~side_stop_dist", 0.12)

        # 小幅后退参数
        self.backup_speed = rospy.get_param("~backup_speed", -0.025)
        self.backup_time = rospy.get_param("~backup_time", 0.25)

        # 坐标系和话题
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")
        self.scan_topic = rospy.get_param("~scan_topic", "/scan_filtered")

        # =====================================================
        # ROS 接口
        # =====================================================
        self.latest_scan = None
        self.scan_sub = rospy.Subscriber(
            self.scan_topic,
            LaserScan,
            self.scan_cb,
            queue_size=1
        )

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.tf_listener = tf.TransformListener()

        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")
        if not self.client.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr("move_base action server not available.")
            return

        rospy.sleep(1.0)

        rospy.loginfo(
            "Target center: x=%.3f y=%.3f yaw=%.3f",
            self.target_x,
            self.target_y,
            self.target_yaw
        )

        self.run()

    # =========================================================
    # 回调
    # =========================================================
    def scan_cb(self, msg):
        self.latest_scan = msg

    # =========================================================
    # 主流程
    # =========================================================
    def run(self):
        candidates = self.generate_candidates()

        for i, candidate in enumerate(candidates):
            x, y, yaw = candidate

            rospy.loginfo(
                "Try candidate %d/%d: x=%.3f y=%.3f yaw=%.3f",
                i + 1,
                len(candidates),
                x,
                y,
                yaw
            )

            ok = self.send_move_base_goal(x, y, yaw, self.nav_timeout)

            if not ok:
                rospy.logwarn("Candidate %d move_base failed. Try next.", i + 1)
                self.stop_robot()
                rospy.sleep(0.2)
                continue

            rospy.loginfo("Candidate %d reached. COMMIT to this entrance.", i + 1)

            # 到达入口点以后，必须取消 move_base
            # 后面不允许 move_base 重新规划绕圈
            self.client.cancel_all_goals()
            rospy.sleep(0.2)
            self.stop_robot()

            if not self.enable_safe_enter:
                rospy.loginfo("Safe enter disabled. Stop at approach point.")
                self.stop_robot()
                return

            enter_ok = self.safe_enter_commit_to_target(candidate)

            if enter_ok:
                rospy.loginfo("Target reached successfully.")
                self.stop_robot()
                return

            rospy.logwarn("Commit enter failed. Stop here, do not loop around.")
            self.stop_robot()
            return

        rospy.logerr("All candidates failed before entering.")
        self.stop_robot()

    # =========================================================
    # 候选入口生成
    # =========================================================
    def generate_candidates(self):
        candidates = []

        # 先加四个标准方向：右、上、左、下
        base_angles = [
            0.0,
            math.pi / 2.0,
            math.pi,
            -math.pi / 2.0
        ]

        r_mid = 0.5 * (self.min_radius + self.max_radius)

        for a in base_angles:
            x = self.target_x + r_mid * math.cos(a)
            y = self.target_y + r_mid * math.sin(a)

            # 入口点车头朝向目标中心
            yaw = math.atan2(self.target_y - y, self.target_x - x)

            candidates.append((x, y, yaw))

        # 再加随机入口点
        for _ in range(self.candidate_count):
            a = random.uniform(-math.pi, math.pi)
            r = random.uniform(self.min_radius, self.max_radius)

            x = self.target_x + r * math.cos(a)
            y = self.target_y + r * math.sin(a)
            yaw = math.atan2(self.target_y - y, self.target_x - x)

            candidates.append((x, y, yaw))

        random.shuffle(candidates)
        return candidates

    # =========================================================
    # move_base 发送候选入口
    # =========================================================
    def send_move_base_goal(self, x, y, yaw, timeout):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.map_frame
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0

        q = yaw_to_quat(yaw)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.client.send_goal(goal)

        finished = self.client.wait_for_result(rospy.Duration(timeout))

        if not finished:
            rospy.logwarn("move_base timeout.")
            self.client.cancel_goal()
            return False

        state = self.client.get_state()
        rospy.loginfo("move_base state: %s", str(state))

        return state == GoalStatus.SUCCEEDED

    # =========================================================
    # 获取机器人当前 map 坐标
    # =========================================================
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

            _, _, yaw = tf.transformations.euler_from_quaternion(rot)

            return trans[0], trans[1], yaw

        except Exception as e:
            rospy.logwarn_throttle(1.0, "TF lookup failed: %s", str(e))
            return None

    # =========================================================
    # COMMIT 安全进入目标
    # =========================================================
    def safe_enter_commit_to_target(self, selected_candidate):
        """
        提交入框模式：
        一旦到达入口点，就不再退出重新评估。
        如果前方太近，只做小幅停车/后退/微调。
        不再调用 move_base，不再绕圈。
        """

        start_time = rospy.Time.now()
        rate = rospy.Rate(20)

        stable_count = 0
        required_stable_count = 8

        stuck_count = 0
        max_stuck_count = 12

        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()

            if elapsed > self.safe_enter_timeout:
                rospy.logwarn("commit safe_enter timeout.")
                self.stop_robot()
                return False

            pose = self.lookup_robot_pose()

            if pose is None:
                self.stop_robot()
                rate.sleep()
                continue

            rx, ry, ryaw = pose

            ex_map = self.target_x - rx
            ey_map = self.target_y - ry
            eyaw = normalize_angle(self.target_yaw - ryaw)

            dist = math.sqrt(ex_map * ex_map + ey_map * ey_map)

            if dist < self.xy_tolerance and abs(eyaw) < self.yaw_tolerance:
                stable_count += 1
                self.stop_robot()

                if stable_count >= required_stable_count:
                    return True

                rate.sleep()
                continue

            stable_count = 0

            front = self.get_sector_min_range(-12, 12)
            left_front = self.get_sector_min_range(12, 45)
            right_front = self.get_sector_min_range(-45, -12)

            rospy.loginfo_throttle(
                0.5,
                "commit enter: dist=%.3f yaw_err=%.3f front=%.3f left_front=%.3f right_front=%.3f",
                dist,
                eyaw,
                front,
                left_front,
                right_front
            )

            # 正前方真的太近，不能继续顶
            if front < self.front_stop_dist:
                stuck_count += 1
                rospy.logwarn("Front blocked in commit mode: %.3f", front)
                self.stop_robot()

                # 小幅后退，不退出入口，不重新绕圈
                cmd = Twist()
                cmd.linear.x = self.backup_speed

                backup_steps = int(max(1, self.backup_time * 20.0))
                for _ in range(backup_steps):
                    self.cmd_pub.publish(cmd)
                    rospy.sleep(0.05)

                self.stop_robot()

                if stuck_count >= max_stuck_count:
                    rospy.logwarn("Too many front blocks in commit mode.")
                    return False

                rate.sleep()
                continue

            # 左前/右前太近时，不失败，只限制对应横移方向
            block_left = left_front < self.side_stop_dist
            block_right = right_front < self.side_stop_dist

            # map 坐标误差转换到 base_footprint
            # cmd_vel 是机器人自身坐标系
            cos_yaw = math.cos(ryaw)
            sin_yaw = math.sin(ryaw)

            ex_base = cos_yaw * ex_map + sin_yaw * ey_map
            ey_base = -sin_yaw * ex_map + cos_yaw * ey_map

            cmd = Twist()

            cmd.linear.x = self.clamp(
                self.kp_xy * ex_base,
                -self.max_enter_vx,
                self.max_enter_vx
            )

            cmd.linear.y = self.clamp(
                self.kp_xy * ey_base,
                -self.max_enter_vy,
                self.max_enter_vy
            )

            cmd.angular.z = self.clamp(
                self.kp_yaw * eyaw,
                -self.max_enter_wz,
                self.max_enter_wz
            )

            # 前方进入慢速区，限制前进速度
            if front < self.front_slow_dist:
                cmd.linear.x = min(cmd.linear.x, 0.008)

            # 左前被挡，禁止继续向左挤
            if block_left and cmd.linear.y > 0.0:
                cmd.linear.y = 0.0

            # 右前被挡，禁止继续向右挤
            if block_right and cmd.linear.y < 0.0:
                cmd.linear.y = 0.0

            # 入框阶段角速度要小，防止车角扫到挡板
            cmd.angular.z = self.clamp(cmd.angular.z, -0.05, 0.05)

            # 小死区，减少抖动
            if abs(cmd.linear.x) < 0.004:
                cmd.linear.x = 0.0
            if abs(cmd.linear.y) < 0.004:
                cmd.linear.y = 0.0
            if abs(cmd.angular.z) < 0.010:
                cmd.angular.z = 0.0

            self.cmd_pub.publish(cmd)
            rate.sleep()

        return False

    # =========================================================
    # 雷达扇区检测
    # =========================================================
    def get_sector_min_range(self, deg_min, deg_max):
        """
        获取雷达某个角度扇区的最小距离。
        角度单位：度。
        默认 laser_link 正前方是 0 度。
        """

        if self.latest_scan is None:
            return float("inf")

        msg = self.latest_scan

        if msg.angle_increment == 0.0:
            return float("inf")

        a0 = math.radians(deg_min)
        a1 = math.radians(deg_max)

        if a0 > a1:
            a0, a1 = a1, a0

        rmin = float("inf")

        for i, r in enumerate(msg.ranges):
            if math.isnan(r) or math.isinf(r):
                continue

            a = msg.angle_min + i * msg.angle_increment

            if a0 <= a <= a1:
                if r < rmin:
                    rmin = r

        return rmin

    # =========================================================
    # 停车和后退
    # =========================================================
    def stop_robot(self):
        cmd = Twist()

        for _ in range(6):
            self.cmd_pub.publish(cmd)
            rospy.sleep(0.03)

    @staticmethod
    def clamp(v, vmin, vmax):
        return max(vmin, min(vmax, v))


if __name__ == "__main__":
    try:
        RandomSingleGoalTest()
    except rospy.ROSInterruptException:
        pass
