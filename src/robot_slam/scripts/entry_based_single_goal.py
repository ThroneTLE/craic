#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
import tf
import actionlib

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus


def yaw_to_quat(yaw):
    return tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)


class EntryBasedSingleGoal:
    def __init__(self):
        rospy.init_node("entry_based_single_goal")

        # =====================================================
        # 目标框中心
        # =====================================================
        self.target_x = rospy.get_param("~target_x", 0.0)
        self.target_y = rospy.get_param("~target_y", 0.0)

        # yaw_mode 只给 move_base 到入口点时使用
        # 入框阶段不主动修 yaw，避免调头出来
        self.yaw_mode = rospy.get_param("~yaw_mode", 0)

        if int(self.yaw_mode) == 180:
            self.target_yaw = math.pi
        else:
            self.target_yaw = 0.0

        # =====================================================
        # 目标框参数
        # =====================================================
        self.box_length = rospy.get_param("~box_length", 0.36)
        self.box_width = rospy.get_param("~box_width", 0.32)

        self.entry_offset = rospy.get_param("~entry_offset", 0.35)
        self.inside_offset = rospy.get_param("~inside_offset", 0.10)

        # 入口顺序：right,left,up,down
        self.entry_order = rospy.get_param("~entry_order", "right,up,left,down")

        # =====================================================
        # move_base 参数
        # =====================================================
        self.nav_timeout = rospy.get_param("~nav_timeout", 6.0)
        self.entry_reached_tolerance = rospy.get_param("~entry_reached_tolerance", 0.20)

        # =====================================================
        # direct drive 到入口参数
        # =====================================================
        self.direct_entry_timeout = rospy.get_param("~direct_entry_timeout", 5.0)
        self.direct_entry_kp = rospy.get_param("~direct_entry_kp", 0.45)
        self.direct_entry_max_v = rospy.get_param("~direct_entry_max_v", 0.04)
        self.direct_entry_tolerance = rospy.get_param("~direct_entry_tolerance", 0.20)

        # 只有离入口比较近，才允许 direct_drive 补偿
        self.direct_entry_start_max_dist = rospy.get_param("~direct_entry_start_max_dist", 0.35)

        # =====================================================
        # 入框控制参数
        # =====================================================
        self.enter_timeout = rospy.get_param("~enter_timeout", 18.0)
        self.xy_tolerance = rospy.get_param("~xy_tolerance", 0.045)

        self.kp_x = rospy.get_param("~kp_x", 0.75)
        self.kp_y = rospy.get_param("~kp_y", 0.75)

        self.max_enter_vx = rospy.get_param("~max_enter_vx", 0.070)
        self.max_enter_vy = rospy.get_param("~max_enter_vy", 0.070)

        # 分段降速
        self.slow_dist = rospy.get_param("~slow_dist", 0.16)
        self.precise_dist = rospy.get_param("~precise_dist", 0.08)
        self.slow_v = rospy.get_param("~slow_v", 0.035)
        self.precise_v = rospy.get_param("~precise_v", 0.018)

        self.min_v = rospy.get_param("~min_v", 0.006)

        # =====================================================
        # 雷达安全参数
        # =====================================================
        self.scan_topic = rospy.get_param("~scan_topic", "/scan_filtered")

        self.front_stop_dist = rospy.get_param("~front_stop_dist", 0.16)
        self.front_slow_dist = rospy.get_param("~front_slow_dist", 0.26)

        # 左前、右前、正左、正右共同使用这个阈值
        self.side_stop_dist = rospy.get_param("~side_stop_dist", 0.16)

        self.any_stop_dist = rospy.get_param("~any_stop_dist", 0.09)

        self.max_block_count = rospy.get_param("~max_block_count", 8)

        # =====================================================
        # 坐标系
        # =====================================================
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")

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
            "Entry target: x=%.3f y=%.3f yaw_mode=%s entry_order=%s",
            self.target_x,
            self.target_y,
            str(self.yaw_mode),
            self.entry_order
        )

        self.run()

    def scan_cb(self, msg):
        self.latest_scan = msg

    # =========================================================
    # 主流程
    # =========================================================
    def run(self):
        entries = self.generate_entries()

        for i, entry in enumerate(entries):
            rospy.loginfo(
                "Try entry %d/%d: name=%s entry=(%.3f, %.3f, %.3f) inside=(%.3f, %.3f) axis=%s",
                i + 1,
                len(entries),
                entry["name"],
                entry["entry_x"],
                entry["entry_y"],
                entry["entry_yaw"],
                entry["inside_x"],
                entry["inside_y"],
                entry["enter_axis"]
            )

            nav_ok = self.send_move_base_goal(
                entry["entry_x"],
                entry["entry_y"],
                entry["entry_yaw"],
                self.nav_timeout
            )

            if not nav_ok:
                rospy.logwarn("Entry %s: navigation/direct entry failed, try next entry.", entry["name"])
                self.stop_robot()
                rospy.sleep(0.2)
                continue

            rospy.loginfo("Entry %s reached. Cancel move_base and enter straight.", entry["name"])

            self.client.cancel_all_goals()
            rospy.sleep(0.2)
            self.stop_robot()

            enter_ok = self.enter_from_entry(entry)

            if enter_ok:
                rospy.loginfo("Entry %s success. Robot is inside target box.", entry["name"])
                self.stop_robot()
                return

            rospy.logwarn("Entry %s failed during entering, try next entry.", entry["name"])
            self.stop_robot()
            rospy.sleep(0.3)

        rospy.logerr("All entries failed. Stop robot.")
        self.stop_robot()

    # =========================================================
    # 生成入口
    # =========================================================
    def generate_entries(self):
        order = [s.strip().lower() for s in self.entry_order.split(",") if s.strip()]
        entries = []

        for name in order:
            if name == "right":
                entry_x = self.target_x + self.entry_offset
                entry_y = self.target_y
                inside_x = self.target_x + self.inside_offset
                inside_y = self.target_y
                enter_axis = "x_minus"

            elif name == "left":
                entry_x = self.target_x - self.entry_offset
                entry_y = self.target_y
                inside_x = self.target_x - self.inside_offset
                inside_y = self.target_y
                enter_axis = "x_plus"

            elif name == "up":
                entry_x = self.target_x
                entry_y = self.target_y + self.entry_offset
                inside_x = self.target_x
                inside_y = self.target_y + self.inside_offset
                enter_axis = "y_minus"

            elif name == "down":
                entry_x = self.target_x
                entry_y = self.target_y - self.entry_offset
                inside_x = self.target_x
                inside_y = self.target_y - self.inside_offset
                enter_axis = "y_plus"

            else:
                rospy.logwarn("Unknown entry name: %s", name)
                continue

            entry_yaw = math.atan2(self.target_y - entry_y, self.target_x - entry_x)

            entries.append({
                "name": name,
                "entry_x": entry_x,
                "entry_y": entry_y,
                "entry_yaw": entry_yaw,
                "inside_x": inside_x,
                "inside_y": inside_y,
                "enter_axis": enter_axis,
            })

        return entries

    # =========================================================
    # move_base 到入口点；失败则近距离 direct drive
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

        if finished:
            state = self.client.get_state()
            rospy.loginfo("move_base state: %s", str(state))

            if state == GoalStatus.SUCCEEDED:
                return True

            if self.is_robot_near_point(x, y, self.entry_reached_tolerance):
                rospy.logwarn("move_base not SUCCEEDED, but robot is near entry. Accept entry.")
                self.client.cancel_goal()
                self.stop_robot()
                return True

            dist_to_entry = self.distance_to_point(x, y)
            if dist_to_entry is None:
                self.client.cancel_goal()
                self.stop_robot()
                return False

            if dist_to_entry > self.direct_entry_start_max_dist:
                rospy.logwarn(
                    "move_base failed and entry is still far %.3f > %.3f. Skip direct drive.",
                    dist_to_entry,
                    self.direct_entry_start_max_dist
                )
                self.client.cancel_goal()
                self.stop_robot()
                return False

            rospy.logwarn(
                "move_base failed but entry is close %.3f. Try direct drive to entry.",
                dist_to_entry
            )
            self.client.cancel_goal()
            self.stop_robot()
            return self.direct_drive_to_entry(x, y)

        rospy.logwarn("move_base timeout, checking distance to entry...")

        if self.is_robot_near_point(x, y, self.entry_reached_tolerance):
            rospy.logwarn("move_base timeout, but robot is near entry. Accept entry.")
            self.client.cancel_goal()
            self.stop_robot()
            return True

        dist_to_entry = self.distance_to_point(x, y)
        if dist_to_entry is None:
            self.client.cancel_goal()
            self.stop_robot()
            return False

        if dist_to_entry > self.direct_entry_start_max_dist:
            rospy.logwarn(
                "move_base timeout and entry is still far %.3f > %.3f. Skip direct drive.",
                dist_to_entry,
                self.direct_entry_start_max_dist
            )
            self.client.cancel_goal()
            self.stop_robot()
            return False

        rospy.logwarn(
            "move_base timeout but entry is close %.3f. Switch to direct drive to entry.",
            dist_to_entry
        )
        self.client.cancel_goal()
        self.stop_robot()
        return self.direct_drive_to_entry(x, y)

    def distance_to_point(self, x, y):
        pose = self.lookup_robot_pose()
        if pose is None:
            return None

        rx, ry, _ = pose
        dx = x - rx
        dy = y - ry
        return math.sqrt(dx * dx + dy * dy)

    def is_robot_near_point(self, x, y, tolerance):
        dist = self.distance_to_point(x, y)
        if dist is None:
            return False

        rospy.loginfo("distance to entry: %.3f, tolerance: %.3f", dist, tolerance)
        return dist < tolerance

    # =========================================================
    # direct drive 到入口点
    # =========================================================
    def direct_drive_to_entry(self, x, y):
        start_time = rospy.Time.now()
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            if (rospy.Time.now() - start_time).to_sec() > self.direct_entry_timeout:
                rospy.logwarn("direct_drive_to_entry timeout.")
                self.stop_robot()
                return self.is_robot_near_point(x, y, self.direct_entry_tolerance)

            pose = self.lookup_robot_pose()
            if pose is None:
                self.stop_robot()
                rate.sleep()
                continue

            rx, ry, ryaw = pose

            ex_map = x - rx
            ey_map = y - ry
            dist = math.sqrt(ex_map * ex_map + ey_map * ey_map)

            rospy.loginfo_throttle(
                0.5,
                "direct entry drive: dist=%.3f target=(%.3f, %.3f)",
                dist,
                x,
                y
            )

            if dist < self.direct_entry_tolerance:
                rospy.logwarn("direct_drive_to_entry reached entry.")
                self.stop_robot()
                return True

            if dist > self.direct_entry_start_max_dist + 0.08:
                rospy.logwarn("direct entry dist %.3f too far, abort direct drive.", dist)
                self.stop_robot()
                return False

            cos_yaw = math.cos(ryaw)
            sin_yaw = math.sin(ryaw)

            ex_base = cos_yaw * ex_map + sin_yaw * ey_map
            ey_base = -sin_yaw * ex_map + cos_yaw * ey_map

            cmd = Twist()
            cmd.linear.x = self.clamp(
                self.direct_entry_kp * ex_base,
                -self.direct_entry_max_v,
                self.direct_entry_max_v
            )
            cmd.linear.y = self.clamp(
                self.direct_entry_kp * ey_base,
                -self.direct_entry_max_v,
                self.direct_entry_max_v
            )
            cmd.angular.z = 0.0

            front = self.get_sector_min_range(-12, 12)
            left_front = self.get_sector_min_range(12, 50)
            right_front = self.get_sector_min_range(-50, -12)
            left_side = self.get_sector_min_range(55, 120)
            right_side = self.get_sector_min_range(-120, -55)

            rospy.loginfo_throttle(
                0.5,
                "direct scan front=%.3f lf=%.3f rf=%.3f left=%.3f right=%.3f",
                front,
                left_front,
                right_front,
                left_side,
                right_side
            )

            if front < self.front_stop_dist and cmd.linear.x > 0.0:
                cmd.linear.x = 0.0

            if front < self.front_slow_dist and cmd.linear.x > 0.0:
                cmd.linear.x = min(cmd.linear.x, 0.012)

            if (left_front < self.side_stop_dist or left_side < self.side_stop_dist) and cmd.linear.y > 0.0:
                rospy.logwarn_throttle(0.5, "direct left blocked, stop left strafe")
                cmd.linear.y = 0.0

            if (right_front < self.side_stop_dist or right_side < self.side_stop_dist) and cmd.linear.y < 0.0:
                rospy.logwarn_throttle(0.5, "direct right blocked, stop right strafe")
                cmd.linear.y = 0.0

            nearest = min(front, left_front, right_front, left_side, right_side)
            if nearest < self.any_stop_dist:
                rospy.logwarn_throttle(0.5, "direct entry too close %.3f, stop translation.", nearest)
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0

            if abs(cmd.linear.x) < 0.006:
                cmd.linear.x = 0.0
            if abs(cmd.linear.y) < 0.006:
                cmd.linear.y = 0.0

            self.cmd_pub.publish(cmd)
            rate.sleep()

        return False

    # =========================================================
    # 从入口沿入口轴进框
    # =========================================================
    def enter_from_entry(self, entry):
        start_time = rospy.Time.now()
        rate = rospy.Rate(20)

        stable_count = 0
        required_stable_count = 6
        block_count = 0

        inside_x = entry["inside_x"]
        inside_y = entry["inside_y"]
        axis = entry["enter_axis"]

        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()

            if elapsed > self.enter_timeout:
                rospy.logwarn("Entry %s enter timeout. Stop here.", entry["name"])
                self.stop_robot()
                return True

            pose = self.lookup_robot_pose()
            if pose is None:
                self.stop_robot()
                rate.sleep()
                continue

            rx, ry, ryaw = pose

            ex_map = inside_x - rx
            ey_map = inside_y - ry
            dist = math.sqrt(ex_map * ex_map + ey_map * ey_map)

            if dist < self.xy_tolerance:
                stable_count += 1
                self.stop_robot()

                if stable_count >= required_stable_count:
                    return True

                rate.sleep()
                continue

            stable_count = 0

            front = self.get_sector_min_range(-12, 12)
            left_front = self.get_sector_min_range(12, 50)
            right_front = self.get_sector_min_range(-50, -12)
            left_side = self.get_sector_min_range(55, 120)
            right_side = self.get_sector_min_range(-120, -55)

            rospy.loginfo_throttle(
                0.5,
                "entry=%s axis=%s dist=%.3f front=%.3f lf=%.3f rf=%.3f left=%.3f right=%.3f",
                entry["name"],
                axis,
                dist,
                front,
                left_front,
                right_front,
                left_side,
                right_side
            )

            if front < self.front_stop_dist:
                block_count += 1
                rospy.logwarn_throttle(
                    0.5,
                    "Entry %s front blocked: %.3f count=%d",
                    entry["name"],
                    front,
                    block_count
                )
                self.stop_robot()
                self.backup_small()

                if block_count >= self.max_block_count:
                    rospy.logwarn("Entry %s failed: too many front blocks.", entry["name"])
                    return False

                rate.sleep()
                continue

            if left_front < self.side_stop_dist or right_front < self.side_stop_dist or left_side < self.side_stop_dist or right_side < self.side_stop_dist:
                block_count += 1
                rospy.logwarn_throttle(
                    0.5,
                    "Entry %s side blocked: lf=%.3f rf=%.3f left=%.3f right=%.3f count=%d",
                    entry["name"],
                    left_front,
                    right_front,
                    left_side,
                    right_side,
                    block_count
                )

                if block_count >= self.max_block_count:
                    rospy.logwarn("Entry %s failed: side blocked too many times.", entry["name"])
                    return False

            cos_yaw = math.cos(ryaw)
            sin_yaw = math.sin(ryaw)

            ex_base = cos_yaw * ex_map + sin_yaw * ey_map
            ey_base = -sin_yaw * ex_map + cos_yaw * ey_map

            cmd = Twist()

            cur_max_vx = self.max_enter_vx
            cur_max_vy = self.max_enter_vy

            if dist < self.precise_dist:
                cur_max_vx = min(cur_max_vx, self.precise_v)
                cur_max_vy = min(cur_max_vy, self.precise_v)
            elif dist < self.slow_dist:
                cur_max_vx = min(cur_max_vx, self.slow_v)
                cur_max_vy = min(cur_max_vy, self.slow_v)

            if axis == "x_minus":
                cmd.linear.x = self.clamp(self.kp_x * ex_base, -cur_max_vx, 0.0)
                cmd.linear.y = 0.0

            elif axis == "x_plus":
                cmd.linear.x = self.clamp(self.kp_x * ex_base, 0.0, cur_max_vx)
                cmd.linear.y = 0.0

            elif axis == "y_minus":
                cmd.linear.x = 0.0
                cmd.linear.y = self.clamp(self.kp_y * ey_base, -cur_max_vy, 0.0)

            elif axis == "y_plus":
                cmd.linear.x = 0.0
                cmd.linear.y = self.clamp(self.kp_y * ey_base, 0.0, cur_max_vy)

            else:
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0

            cmd.angular.z = 0.0

            if front < self.front_slow_dist:
                if cmd.linear.x > 0.0:
                    cmd.linear.x = min(cmd.linear.x, 0.010)
                if cmd.linear.x < 0.0:
                    cmd.linear.x = max(cmd.linear.x, -0.010)

            if (left_front < self.side_stop_dist or left_side < self.side_stop_dist) and cmd.linear.y > 0.0:
                rospy.logwarn_throttle(0.5, "left blocked, stop left strafe")
                cmd.linear.y = 0.0

            if (right_front < self.side_stop_dist or right_side < self.side_stop_dist) and cmd.linear.y < 0.0:
                rospy.logwarn_throttle(0.5, "right blocked, stop right strafe")
                cmd.linear.y = 0.0

            if abs(cmd.linear.x) < self.min_v:
                cmd.linear.x = 0.0

            if abs(cmd.linear.y) < self.min_v:
                cmd.linear.y = 0.0

            nearest = min(front, left_front, right_front, left_side, right_side)
            if nearest < self.any_stop_dist:
                rospy.logwarn_throttle(0.5, "Too close %.3f, stop translation.", nearest)
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
                cmd.angular.z = 0.0

            self.cmd_pub.publish(cmd)
            rate.sleep()

        return False

    # =========================================================
    # TF
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
    # 雷达扇区
    # =========================================================
    def get_sector_min_range(self, deg_min, deg_max):
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

    def backup_small(self):
        cmd = Twist()
        cmd.linear.x = -0.020

        for _ in range(8):
            self.cmd_pub.publish(cmd)
            rospy.sleep(0.04)

        self.stop_robot()

    def stop_robot(self):
        cmd = Twist()

        for _ in range(8):
            self.cmd_pub.publish(cmd)
            rospy.sleep(0.03)

    @staticmethod
    def clamp(v, vmin, vmax):
        return max(vmin, min(vmax, v))


if __name__ == "__main__":
    try:
        EntryBasedSingleGoal()
    except rospy.ROSInterruptException:
        pass
