#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import math
import rospy
import tf
import actionlib

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


class AutoSinglePointTest:
    def __init__(self, target_x=None, target_y=None, target_yaw_deg=None):
        # =====================================================
        # 目标点（参数传入优先，否则从参数服务器读取）
        # =====================================================
        self.target_x = target_x if target_x is not None else rospy.get_param("target_x", 0.0)
        self.target_y = target_y if target_y is not None else rospy.get_param("target_y", 0.0)

        self.target_yaw_deg = target_yaw_deg if target_yaw_deg is not None else rospy.get_param("target_yaw", 0.0)
        self.target_yaw = math.radians(self.target_yaw_deg)

        # =====================================================
        # 无挡板直达
        # =====================================================
        self.enable_direct_center = rospy.get_param("enable_direct_center", True)
        self.direct_center_timeout = rospy.get_param("direct_center_timeout", 4.5)
        self.direct_center_tolerance = rospy.get_param("direct_center_tolerance", 0.07)

        self.skip_direct_if_path_blocked = rospy.get_param("skip_direct_if_path_blocked", True)
        self.current_path_width = rospy.get_param("current_path_width", 0.38)
        self.current_path_min_points = int(rospy.get_param("current_path_min_points", 4))

        # =====================================================
        # 入口参数
        # =====================================================
        self.entry_offset = rospy.get_param("entry_offset", 0.34)
        self.xy_tolerance = rospy.get_param("xy_tolerance", 0.06)

        # =====================================================
        # 入口识别评分参数
        # =====================================================
        self.enable_entry_recognition = rospy.get_param("enable_entry_recognition", True)

        # 目标点四面条带辅助检测
        self.target_box_half_size = rospy.get_param("target_box_half_size", 0.24)
        self.side_detect_width = rospy.get_param("side_detect_width", 0.12)
        self.side_detect_min_points = int(rospy.get_param("side_detect_min_points", 4))

        # 核心新增：目标点圆环开口检测
        self.enable_opening_circle_detect = rospy.get_param("enable_opening_circle_detect", True)
        self.opening_detect_radius = rospy.get_param("opening_detect_radius", 0.30)
        self.opening_ring_width = rospy.get_param("opening_ring_width", 0.08)
        self.opening_min_clear_diff = int(rospy.get_param("opening_min_clear_diff", 3))
        self.opening_best_bonus = rospy.get_param("opening_best_bonus", 45.0)
        self.opening_not_best_penalty = rospy.get_param("opening_not_best_penalty", 22.0)
        self.opening_count_weight = rospy.get_param("opening_count_weight", 10.0)
        self.opening_unknown_penalty = rospy.get_param("opening_unknown_penalty", 0.0)

        # 入口点到目标点通道检测
        self.path_corridor_width = rospy.get_param("path_corridor_width", 0.36)
        self.path_corridor_min_points = int(rospy.get_param("path_corridor_min_points", 4))
        self.path_corridor_ignore_near_start = rospy.get_param("path_corridor_ignore_near_start", 0.06)
        self.path_corridor_ignore_near_goal = rospy.get_param("path_corridor_ignore_near_goal", 0.08)

        # 旧 corridor 参数保留
        self.corridor_width = rospy.get_param("corridor_width", 0.32)
        self.corridor_min_points = int(rospy.get_param("corridor_min_points", 5))

        self.scan_memory_time = rospy.get_param("scan_memory_time", 0.8)
        self.recognition_max_range = rospy.get_param("recognition_max_range", 1.6)
        self.max_entries_to_try = int(rospy.get_param("max_entries_to_try", 3))

        self.scan_memory = []

        # =====================================================
        # L 型挡板接受停车条件
        # =====================================================
        self.obstacle_park_accept_dist = rospy.get_param("obstacle_park_accept_dist", 0.16)
        self.obstacle_park_front_min = rospy.get_param("obstacle_park_front_min", 0.17)
        self.obstacle_park_front_max = rospy.get_param("obstacle_park_front_max", 0.24)
        self.obstacle_park_yaw_tolerance = rospy.get_param("obstacle_park_yaw_tolerance", 0.10)
        self.obstacle_park_enter_travel_min = rospy.get_param("obstacle_park_enter_travel_min", 0.30)

        # =====================================================
        # 深入停车
        # =====================================================
        self.park_deep_offset = rospy.get_param("park_deep_offset", 0.10)

        # =====================================================
        # 成功后退出
        # =====================================================
        self.exit_after_success = rospy.get_param("exit_after_success", True)
        self.exit_distance = rospy.get_param("exit_distance", 0.24)
        self.exit_speed = rospy.get_param("exit_speed", 0.090)

        # =====================================================
        # yaw 对齐
        # =====================================================
        self.yaw_tolerance = rospy.get_param("yaw_tolerance", 0.09)
        self.align_timeout = rospy.get_param("align_timeout", 10.0)
        self.kp_yaw = rospy.get_param("kp_yaw", 2.00)
        self.max_align_wz = rospy.get_param("max_align_wz", 0.90)
        self.max_enter_wz = rospy.get_param("max_enter_wz", 0.12)
        self.align_accept_yaw_error = rospy.get_param("align_accept_yaw_error", 0.30)

        # =====================================================
        # move_base 到入口
        # =====================================================
        self.nav_timeout = rospy.get_param("nav_timeout", 11.0)
        self.entry_reached_tolerance = rospy.get_param("entry_reached_tolerance", 0.18)

        # =====================================================
        # direct 补偿到入口
        # =====================================================
        self.direct_entry_timeout = rospy.get_param("direct_entry_timeout", 8.0)
        self.direct_entry_kp = rospy.get_param("direct_entry_kp", 0.78)
        self.direct_entry_max_v = rospy.get_param("direct_entry_max_v", 0.115)
        self.direct_entry_tolerance = rospy.get_param("direct_entry_tolerance", 0.16)
        self.direct_entry_start_max_dist = rospy.get_param("direct_entry_start_max_dist", 1.45)

        self.direct_block_max_count = max(
            3,
            int(rospy.get_param("direct_block_max_count", 3))
        )

        self.direct_no_progress_max_count = max(
            32,
            int(rospy.get_param("direct_no_progress_max_count", 32))
        )

        # =====================================================
        # 脱困
        # =====================================================
        self.backoff_time = rospy.get_param("backoff_time", 0.7)
        self.backoff_speed = rospy.get_param("backoff_speed", 0.085)

        # =====================================================
        # 入框控制
        # =====================================================
        self.enter_timeout = rospy.get_param("enter_timeout", 20.0)

        self.kp_enter = rospy.get_param("kp_enter", 1.12)
        self.max_enter_v = rospy.get_param("max_enter_v", 0.120)

        self.slow_dist = rospy.get_param("slow_dist", 0.13)
        self.precise_dist = rospy.get_param("precise_dist", 0.06)
        self.slow_v = rospy.get_param("slow_v", 0.070)
        self.precise_v = rospy.get_param("precise_v", 0.028)

        self.min_v = rospy.get_param("min_v", 0.004)

        # =====================================================
        # 【优化1】进框前重定位收敛
        # 进框是精度最高的环节，先停车等 Cartographer/AMCL 收敛再进
        # =====================================================
        self.settle_before_enter = rospy.get_param("settle_before_enter", True)
        self.settle_max_wait = rospy.get_param("settle_max_wait", 2.5)
        self.settle_pos_var_thresh = rospy.get_param("settle_pos_var_thresh", 0.0008)
        self.settle_yaw_var_thresh = rospy.get_param("settle_yaw_var_thresh", 0.010)

        # =====================================================
        # 【优化2】稳定位姿采样：多帧平均抵消运动残余抖动
        # =====================================================
        self.stable_pose_samples = int(rospy.get_param("stable_pose_samples", 5))
        self.stable_pose_dt = rospy.get_param("stable_pose_dt", 0.04)

        # =====================================================
        # 【优化3】进框激光横向居中：用两侧挡板对称性微调，不依赖 map 定位
        # =====================================================
        self.enable_laser_centering = rospy.get_param("enable_laser_centering", True)
        self.center_kp = rospy.get_param("center_kp", 0.80)
        self.center_max_vy = rospy.get_param("center_max_vy", 0.050)
        self.center_corridor_half = rospy.get_param("center_corridor_half", 0.22)
        self.center_min_side_points = int(rospy.get_param("center_min_side_points", 3))
        self.center_forward_min = rospy.get_param("center_forward_min", 0.05)
        self.center_forward_max = rospy.get_param("center_forward_max", 0.45)
        # 只有进入精细/慢速段才开启横向居中，避免远处误判
        self.center_enable_dist = rospy.get_param("center_enable_dist", 0.22)

        # 进框最终判定改用稳定位姿（多帧平均）
        self.enter_use_stable_pose = rospy.get_param("enter_use_stable_pose", True)

        # =====================================================
        # 雷达安全
        # =====================================================
        self.scan_topic = rospy.get_param("scan_topic", "/scan_filtered")

        self.front_stop_dist = rospy.get_param("front_stop_dist", 0.17)
        self.front_slow_dist = rospy.get_param("front_slow_dist", 0.30)
        self.front_slow_v = rospy.get_param("front_slow_v", 0.034)
        self.side_stop_dist = rospy.get_param("side_stop_dist", 0.21)
        self.any_stop_dist = rospy.get_param("any_stop_dist", 0.085)

        self.max_block_count = max(
            2,
            int(rospy.get_param("max_block_count", 2))
        )

        self.last_block_front = False
        self.last_block_left = False
        self.last_block_right = False
        self.last_block_any = False

        # =====================================================
        # 坐标系
        # =====================================================
        self.map_frame = rospy.get_param("map_frame", "map")
        self.base_frame = rospy.get_param("base_frame", "base_footprint")

        # =====================================================
        # cmd_vel 平滑
        # =====================================================
        self.enable_cmd_smoothing = rospy.get_param("enable_cmd_smoothing", True)
        self.max_acc_x = rospy.get_param("max_acc_x", 0.90)
        self.max_acc_y = rospy.get_param("max_acc_y", 0.90)
        self.max_acc_wz = rospy.get_param("max_acc_wz", 2.20)

        self.last_cmd = Twist()
        self.last_cmd_time = rospy.Time.now()

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

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")

        if not self.move_base.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr("move_base action server not available.")
            return

        rospy.sleep(1.0)

        rospy.loginfo(
            "Auto single point target: x=%.3f y=%.3f yaw_deg=%.1f",
            self.target_x,
            self.target_y,
            self.target_yaw_deg
        )

    def scan_cb(self, msg):
        self.latest_scan = msg

    # =========================================================
    # 主流程
    # =========================================================
    def run(self):
        points_for_recognition = self.collect_scan_points_in_map()

        if self.enable_direct_center:
            direct_blocked = False

            if self.skip_direct_if_path_blocked:
                direct_blocked = self.is_current_path_to_target_blocked(points_for_recognition)

            if direct_blocked:
                rospy.logwarn("Skip direct center because current path to target is blocked.")
            else:
                if self.try_direct_goal_center():
                    rospy.loginfo("SUCCESS: direct center parking finished.")
                    self.stop_robot()
                    return

                rospy.logwarn("Direct center parking failed. Switch to entry-based parking.")

        entries = self.generate_entries()

        if self.enable_entry_recognition:
            entries = self.sort_entries_by_obstacle_score(entries)
        else:
            entries = self.sort_entries_by_robot_position(entries)

        if self.max_entries_to_try > 0 and self.max_entries_to_try < len(entries):
            entries = entries[:self.max_entries_to_try]

        rospy.loginfo("Final entry order: %s" % ",".join([e["name"] for e in entries]))

        for i, entry in enumerate(entries):
            rospy.loginfo(
                "Try entry %d/%d: %s entry=(%.3f, %.3f) yaw=%.3f target_yaw=%.3f axis=(%.1f, %.1f) score=%.3f opening_count=%d is_opening=%s path_count=%d",
                i + 1,
                len(entries),
                entry["name"],
                entry["entry_x"],
                entry["entry_y"],
                entry["entry_yaw"],
                entry["target_yaw"],
                entry["axis_x"],
                entry["axis_y"],
                entry.get("score", -1.0),
                entry.get("opening_count", -1),
                str(entry.get("is_best_opening", False)),
                entry.get("path_count", -1)
            )

            ok = self.goto_entry(entry)

            if not ok:
                rospy.logwarn("Entry %s failed before entering. Try next.", entry["name"])
                self.stop_robot()
                rospy.sleep(0.2)
                continue

            rospy.loginfo("Entry %s reached. Align yaw before entering.", entry["name"])

            self.move_base.cancel_all_goals()
            rospy.sleep(0.2)
            self.stop_robot()

            align_ok = self.align_to_yaw(entry["target_yaw"])

            if not align_ok:
                yaw_err_now = self.get_yaw_error(entry["target_yaw"])

                if yaw_err_now is not None and abs(yaw_err_now) < self.align_accept_yaw_error:
                    rospy.logwarn(
                        "Entry %s yaw align not perfect, err=%.3f. Continue entering and correct slowly.",
                        entry["name"],
                        yaw_err_now
                    )
                else:
                    rospy.logwarn("Entry %s yaw align failed. Try next entry.", entry["name"])
                    self.stop_robot()
                    rospy.sleep(0.2)
                    continue

            # 【优化1】进框前等定位收敛，利用"停下能拉回来"的特性
            self.wait_localization_settle()

            rospy.loginfo("Yaw aligned/enough and localization settled. Start entering target box.")

            enter_ok = self.enter_to_target_box(entry)

            if enter_ok:
                rospy.loginfo("SUCCESS: parked inside target box by entry %s.", entry["name"])
                self.stop_robot()
                rospy.sleep(0.3)

                if self.exit_after_success:
                    rospy.loginfo("Exit from target box for next task.")
                    self.exit_from_entry(entry)

                self.stop_robot()
                return

            rospy.logwarn("Entry %s blocked or failed during entering. Try next entry.", entry["name"])
            self.stop_robot()
            rospy.sleep(0.3)

        rospy.logerr("FAILED: all entries failed.")
        self.stop_robot()

    # =========================================================
    # 无挡板直达中心
    # =========================================================
    def try_direct_goal_center(self):
        rospy.loginfo(
            "Try direct center goal: x=%.3f y=%.3f yaw_deg=%.1f",
            self.target_x,
            self.target_y,
            self.target_yaw_deg
        )

        ok = self.send_move_base_pose(
            self.target_x,
            self.target_y,
            self.target_yaw,
            self.direct_center_timeout
        )

        if ok:
            rospy.loginfo("Direct center move_base SUCCEEDED.")
            return True

        if self.is_robot_near_point(self.target_x, self.target_y, self.direct_center_tolerance):
            rospy.logwarn("Direct center not SUCCEEDED, but robot is near target center. Accept.")
            self.move_base.cancel_all_goals()
            self.stop_robot()
            return True

        self.move_base.cancel_all_goals()
        self.stop_robot()
        return False

    def send_move_base_pose(self, x, y, yaw_rad, timeout):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.map_frame
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0

        q = yaw_to_quat(yaw_rad)

        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.move_base.send_goal(goal)

        finished = self.move_base.wait_for_result(rospy.Duration(timeout))

        if not finished:
            rospy.logwarn("send_move_base_pose timeout.")
            self.move_base.cancel_goal()
            return False

        state = self.move_base.get_state()
        rospy.loginfo("send_move_base_pose state: %s", str(state))

        return state == GoalStatus.SUCCEEDED

    # =========================================================
    # 四个入口
    # =========================================================
    def generate_entries(self):
        tx = self.target_x
        ty = self.target_y
        d = self.entry_offset

        raw_entries = [
            {
                "name": "right",
                "entry_x": tx + d,
                "entry_y": ty,
                "axis_x": -1.0,
                "axis_y": 0.0,
                "target_yaw": self.target_yaw,
            },
            {
                "name": "left",
                "entry_x": tx - d,
                "entry_y": ty,
                "axis_x": 1.0,
                "axis_y": 0.0,
                "target_yaw": self.target_yaw,
            },
            {
                "name": "up",
                "entry_x": tx,
                "entry_y": ty + d,
                "axis_x": 0.0,
                "axis_y": -1.0,
                "target_yaw": self.target_yaw,
            },
            {
                "name": "down",
                "entry_x": tx,
                "entry_y": ty - d,
                "axis_x": 0.0,
                "axis_y": 1.0,
                "target_yaw": self.target_yaw,
            },
        ]

        entries = []

        for e in raw_entries:
            yaw = math.atan2(ty - e["entry_y"], tx - e["entry_x"])
            e["entry_yaw"] = yaw
            entries.append(e)

        return entries

    def sort_entries_by_robot_position(self, entries):
        pose = self.lookup_robot_pose()

        if pose is None:
            rospy.logwarn("Cannot get robot pose. Use default entry order.")
            return entries

        rx, ry, _ = pose

        def score(e):
            dx = e["entry_x"] - rx
            dy = e["entry_y"] - ry
            return math.sqrt(dx * dx + dy * dy)

        return sorted(entries, key=score)

    # =========================================================
    # 路径通道检测
    # =========================================================
    def count_points_in_path_corridor(self, points, start_x, start_y, goal_x, goal_y, width):
        dx = goal_x - start_x
        dy = goal_y - start_y

        length = math.sqrt(dx * dx + dy * dy)

        if length < 1e-4:
            return 0

        half_width = width * 0.5
        count = 0

        for mx, my in points:
            vx = mx - start_x
            vy = my - start_y

            along = (vx * dx + vy * dy) / length
            side = abs((-vx * dy + vy * dx) / length)

            if along <= self.path_corridor_ignore_near_start:
                continue

            if along >= length - self.path_corridor_ignore_near_goal:
                continue

            if 0.0 < along < length and side <= half_width:
                count += 1

        return count

    def is_current_path_to_target_blocked(self, points):
        pose = self.lookup_robot_pose()

        if pose is None:
            rospy.logwarn("Cannot check current path blocked: no robot pose.")
            return False

        rx, ry, _ = pose

        count = self.count_points_in_path_corridor(
            points,
            rx,
            ry,
            self.target_x,
            self.target_y,
            self.current_path_width
        )

        rospy.logwarn(
            "Current path to target corridor: count=%d threshold=%d width=%.3f",
            count,
            self.current_path_min_points,
            self.current_path_width
        )

        return count >= self.current_path_min_points

    # =========================================================
    # 以目标点为圆心，30cm 圆环找开口方向
    # =========================================================
    def detect_opening_by_circle(self, points):
        counts = {
            "left": 0,
            "right": 0,
            "up": 0,
            "down": 0,
        }

        tx = self.target_x
        ty = self.target_y

        r_min = self.opening_detect_radius - self.opening_ring_width * 0.5
        r_max = self.opening_detect_radius + self.opening_ring_width * 0.5

        ring_count = 0

        for mx, my in points:
            dx = mx - tx
            dy = my - ty
            r = math.sqrt(dx * dx + dy * dy)

            if r < r_min or r > r_max:
                continue

            ring_count += 1

            ang = math.atan2(dy, dx)
            deg = math.degrees(ang)

            if -45.0 <= deg <= 45.0:
                counts["right"] += 1
            elif 45.0 < deg < 135.0:
                counts["up"] += 1
            elif -135.0 < deg < -45.0:
                counts["down"] += 1
            else:
                counts["left"] += 1

        min_count = min(counts.values())
        max_count = max(counts.values())

        best_names = []
        confident = False

        if ring_count > 0 and (max_count - min_count) >= self.opening_min_clear_diff:
            confident = True

            for k in ["left", "right", "up", "down"]:
                if counts[k] <= min_count + 1:
                    best_names.append(k)

        rospy.logwarn(
            "Opening circle: confident=%s ring=%d radius=%.2f width=%.2f left=%d right=%d up=%d down=%d best=%s",
            str(confident),
            ring_count,
            self.opening_detect_radius,
            self.opening_ring_width,
            counts["left"],
            counts["right"],
            counts["up"],
            counts["down"],
            ",".join(best_names)
        )

        return {
            "counts": counts,
            "ring_count": ring_count,
            "best_names": best_names,
            "confident": confident,
        }

    # =========================================================
    # 入口识别评分
    # =========================================================
    def sort_entries_by_obstacle_score(self, entries):
        points = self.collect_scan_points_in_map()
        sides = self.evaluate_target_sides(points)

        opening = {
            "counts": {
                "left": 0,
                "right": 0,
                "up": 0,
                "down": 0,
            },
            "ring_count": 0,
            "best_names": [],
            "confident": False,
        }

        if self.enable_opening_circle_detect:
            opening = self.detect_opening_by_circle(points)

        pose = self.lookup_robot_pose()

        if pose is None:
            rospy.logwarn("No robot pose for obstacle score. Use distance order.")
            return self.sort_entries_by_robot_position(entries)

        rx, ry, _ = pose

        rospy.logwarn(
            "Side detection: left=%s(%d) right=%s(%d) up=%s(%d) down=%s(%d), points=%d",
            str(sides["left"]["blocked"]),
            sides["left"]["count"],
            str(sides["right"]["blocked"]),
            sides["right"]["count"],
            str(sides["up"]["blocked"]),
            sides["up"]["count"],
            str(sides["down"]["blocked"]),
            sides["down"]["count"],
            len(points)
        )

        blocked_names = []
        open_names = []

        for k in ["left", "right", "up", "down"]:
            if sides[k]["blocked"]:
                blocked_names.append(k)
            else:
                open_names.append(k)

        rospy.logwarn(
            "Open sides by strip: %s, blocked sides: %s",
            ",".join(open_names),
            ",".join(blocked_names)
        )

        for e in entries:
            dist = math.sqrt((e["entry_x"] - rx) ** 2 + (e["entry_y"] - ry) ** 2)
            side_name = e["name"]

            side_blocked = sides.get(side_name, {"blocked": False})["blocked"]
            side_count = sides.get(side_name, {"count": 0})["count"]

            old_corridor_count = self.count_corridor_points(points, e)

            path_count = self.count_points_in_path_corridor(
                points,
                e["entry_x"],
                e["entry_y"],
                self.target_x,
                self.target_y,
                self.path_corridor_width
            )

            opening_count = opening["counts"].get(side_name, 0)
            is_best_opening = side_name in opening["best_names"]

            score = 0.0

            score += dist * 1.0

            if self.enable_opening_circle_detect:
                score += opening_count * self.opening_count_weight

                if opening["confident"]:
                    if is_best_opening:
                        score -= self.opening_best_bonus
                    else:
                        score += self.opening_not_best_penalty
                else:
                    score += self.opening_unknown_penalty

            if path_count >= self.path_corridor_min_points:
                score += 35.0 + path_count * 4.0
            else:
                score -= 10.0

            if side_blocked:
                score += 18.0 + side_count * 1.5

            if old_corridor_count >= self.corridor_min_points:
                score += 8.0 + old_corridor_count * 1.0

            if (not side_blocked) and len(open_names) <= 2:
                score -= 5.0

            e["score"] = score
            e["path_count"] = path_count
            e["corridor_count"] = old_corridor_count
            e["side_blocked"] = side_blocked
            e["opening_count"] = opening_count
            e["is_best_opening"] = is_best_opening
            e["opening_confident"] = opening["confident"]

            rospy.logwarn(
                "Entry score %s: score=%.3f dist=%.3f opening_count=%d is_best_opening=%s opening_confident=%s path_count=%d side_blocked=%s side_count=%d old_corridor=%d",
                e["name"],
                score,
                dist,
                opening_count,
                str(is_best_opening),
                str(opening["confident"]),
                path_count,
                str(side_blocked),
                side_count,
                old_corridor_count
            )

        return sorted(entries, key=lambda x: x.get("score", 999.0))

    def collect_scan_points_in_map(self):
        now = rospy.Time.now()
        points = []

        if self.latest_scan is None:
            return points

        scan = self.latest_scan

        try:
            self.tf_listener.waitForTransform(
                self.map_frame,
                scan.header.frame_id,
                rospy.Time(0),
                rospy.Duration(0.3)
            )

            trans, rot = self.tf_listener.lookupTransform(
                self.map_frame,
                scan.header.frame_id,
                rospy.Time(0)
            )

            _, _, yaw = tf.transformations.euler_from_quaternion(rot)

            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)

            for i, r in enumerate(scan.ranges):
                if math.isnan(r) or math.isinf(r):
                    continue

                if r <= scan.range_min or r >= scan.range_max:
                    continue

                a = scan.angle_min + i * scan.angle_increment

                lx = r * math.cos(a)
                ly = r * math.sin(a)

                mx = trans[0] + cos_yaw * lx - sin_yaw * ly
                my = trans[1] + sin_yaw * lx + cos_yaw * ly

                dx = mx - self.target_x
                dy = my - self.target_y

                if math.sqrt(dx * dx + dy * dy) <= self.recognition_max_range:
                    points.append((mx, my))

            self.scan_memory.append((now, points))

        except Exception as e:
            rospy.logwarn_throttle(1.0, "collect_scan_points_in_map failed: %s", str(e))

        new_memory = []
        combined = []

        for t, ps in self.scan_memory:
            age = (now - t).to_sec()

            if age <= self.scan_memory_time:
                new_memory.append((t, ps))
                combined.extend(ps)

        self.scan_memory = new_memory

        return combined

    def evaluate_target_sides(self, points):
        tx = self.target_x
        ty = self.target_y
        h = self.target_box_half_size
        w = self.side_detect_width

        counts = {
            "left": 0,
            "right": 0,
            "up": 0,
            "down": 0,
        }

        for mx, my in points:
            if (tx - h - w) <= mx <= (tx - h + w) and (ty - h) <= my <= (ty + h):
                counts["left"] += 1

            if (tx + h - w) <= mx <= (tx + h + w) and (ty - h) <= my <= (ty + h):
                counts["right"] += 1

            if (ty - h - w) <= my <= (ty - h + w) and (tx - h) <= mx <= (tx + h):
                counts["down"] += 1

            if (ty + h - w) <= my <= (ty + h + w) and (tx - h) <= mx <= (tx + h):
                counts["up"] += 1

        sides = {}

        for k in ["left", "right", "up", "down"]:
            sides[k] = {
                "count": counts[k],
                "blocked": counts[k] >= self.side_detect_min_points
            }

        return sides

    def count_corridor_points(self, points, entry):
        tx = self.target_x
        ty = self.target_y

        ex = entry["entry_x"]
        ey = entry["entry_y"]

        half_w = self.corridor_width * 0.5

        count = 0

        if entry["name"] in ["left", "right"]:
            xmin = min(ex, tx)
            xmax = max(ex, tx)
            ymin = ty - half_w
            ymax = ty + half_w

            for mx, my in points:
                if xmin <= mx <= xmax and ymin <= my <= ymax:
                    count += 1

        elif entry["name"] in ["up", "down"]:
            ymin = min(ey, ty)
            ymax = max(ey, ty)
            xmin = tx - half_w
            xmax = tx + half_w

            for mx, my in points:
                if xmin <= mx <= xmax and ymin <= my <= ymax:
                    count += 1

        return count

    # =========================================================
    # move_base 到入口
    # =========================================================
    def goto_entry(self, entry):
        x = entry["entry_x"]
        y = entry["entry_y"]
        yaw = entry["entry_yaw"]

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

        self.move_base.send_goal(goal)

        finished = self.move_base.wait_for_result(rospy.Duration(self.nav_timeout))

        if finished:
            state = self.move_base.get_state()
            rospy.loginfo("move_base state: %s", str(state))

            if state == GoalStatus.SUCCEEDED:
                return True

            if self.is_robot_near_point(x, y, self.entry_reached_tolerance):
                rospy.logwarn("move_base not succeeded, but robot is near entry. Accept.")
                self.move_base.cancel_goal()
                self.stop_robot()
                return True

            dist = self.distance_to_point(x, y)

            if dist is None:
                return False

            if dist > self.direct_entry_start_max_dist:
                rospy.logwarn(
                    "move_base failed and entry is far %.3f > %.3f. Skip direct.",
                    dist,
                    self.direct_entry_start_max_dist
                )
                self.move_base.cancel_goal()
                self.stop_robot()
                return False

            rospy.logwarn("move_base failed but entry close %.3f. Use direct drive.", dist)
            self.move_base.cancel_goal()
            self.stop_robot()
            return self.direct_drive_to_entry(x, y)

        rospy.logwarn("move_base timeout. Check entry distance.")
        self.move_base.cancel_goal()
        self.stop_robot()

        if self.is_robot_near_point(x, y, self.entry_reached_tolerance):
            rospy.logwarn("timeout, but near entry. Accept.")
            return True

        dist = self.distance_to_point(x, y)

        if dist is None:
            return False

        if dist > self.direct_entry_start_max_dist:
            rospy.logwarn(
                "timeout and entry is far %.3f > %.3f. Skip direct.",
                dist,
                self.direct_entry_start_max_dist
            )
            return False

        rospy.logwarn("timeout but entry close %.3f. Use direct drive.", dist)
        return self.direct_drive_to_entry(x, y)

    # =========================================================
    # direct 补偿
    # =========================================================
    def direct_drive_to_entry(self, x, y):
        start_time = rospy.Time.now()
        rate = rospy.Rate(20)

        block_count = 0
        last_dist = None
        no_progress_count = 0

        while not rospy.is_shutdown():
            pose = self.lookup_robot_pose()

            if pose is None:
                self.stop_robot()
                rate.sleep()
                continue

            rx, ry, ryaw = pose

            ex_map = x - rx
            ey_map = y - ry
            dist = math.sqrt(ex_map * ex_map + ey_map * ey_map)

            elapsed = (rospy.Time.now() - start_time).to_sec()

            rospy.loginfo_throttle(
                0.5,
                "direct entry: dist=%.3f target=(%.3f, %.3f) block=%d no_progress=%d",
                dist,
                x,
                y,
                block_count,
                no_progress_count
            )

            if dist < self.direct_entry_tolerance:
                rospy.logwarn(
                    "direct_drive_to_entry reached. dist=%.3f tolerance=%.3f",
                    dist,
                    self.direct_entry_tolerance
                )
                self.stop_robot()
                return True

            if elapsed > self.direct_entry_timeout:
                rospy.logwarn(
                    "direct_drive_to_entry timeout. dist=%.3f tolerance=%.3f",
                    dist,
                    self.direct_entry_tolerance
                )
                self.stop_robot()

                if dist < self.direct_entry_tolerance * 1.4:
                    rospy.logwarn("direct timeout but close enough. Accept.")
                    return True

                rospy.logwarn("direct timeout and still far. Reject.")
                return False

            if dist > self.direct_entry_start_max_dist + 0.25:
                rospy.logwarn(
                    "direct entry dist %.3f too far, abort. max_start=%.3f",
                    dist,
                    self.direct_entry_start_max_dist
                )
                self.stop_robot()
                return False

            vx_map = self.direct_entry_kp * ex_map
            vy_map = self.direct_entry_kp * ey_map

            speed = math.sqrt(vx_map * vx_map + vy_map * vy_map)

            if speed > self.direct_entry_max_v:
                scale = self.direct_entry_max_v / speed
                vx_map *= scale
                vy_map *= scale

            cmd_raw = self.map_velocity_to_base_cmd(vx_map, vy_map, ryaw)
            cmd_raw.angular.z = 0.0

            cmd = self.apply_laser_safety(cmd_raw)

            blocked_now = (
                abs(cmd.linear.x) < 1e-4 and
                abs(cmd.linear.y) < 1e-4 and
                (
                    self.last_block_front or
                    self.last_block_left or
                    self.last_block_right or
                    self.last_block_any
                )
            )

            if blocked_now:
                block_count += 1
                rospy.logwarn_throttle(
                    0.5,
                    "direct entry blocked count=%d/%d",
                    block_count,
                    self.direct_block_max_count
                )

                if block_count >= self.direct_block_max_count:
                    rospy.logwarn("direct entry blocked too many times. Escape and reject this entry.")
                    self.backoff_from_obstacle()
                    return False
            else:
                block_count = 0

            if elapsed > 2.5 and last_dist is not None:
                if abs(last_dist - dist) < 0.003:
                    no_progress_count += 1
                else:
                    no_progress_count = 0

                if no_progress_count >= self.direct_no_progress_max_count:
                    rospy.logwarn(
                        "direct entry no progress for long time. dist=%.3f. Escape and reject.",
                        dist
                    )
                    self.backoff_from_obstacle()
                    return False

            last_dist = dist

            self.publish_cmd(cmd)
            rate.sleep()

        return False

    # =========================================================
    # 脱困
    # =========================================================
    def backoff_from_obstacle(self):
        rospy.logwarn(
            "Escape from obstacle: front=%s left=%s right=%s any=%s time=%.2f speed=%.3f",
            str(self.last_block_front),
            str(self.last_block_left),
            str(self.last_block_right),
            str(self.last_block_any),
            self.backoff_time,
            self.backoff_speed
        )

        cmd = Twist()

        if self.last_block_front or self.last_block_any:
            cmd.linear.x = -abs(self.backoff_speed)

        if self.last_block_right:
            cmd.linear.y = abs(self.backoff_speed)

        if self.last_block_left:
            cmd.linear.y = -abs(self.backoff_speed)

        if abs(cmd.linear.x) < 1e-5 and abs(cmd.linear.y) < 1e-5:
            cmd.linear.x = -abs(self.backoff_speed)

        cmd.angular.z = 0.0

        start_time = rospy.Time.now()
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            if (rospy.Time.now() - start_time).to_sec() > self.backoff_time:
                break

            self.publish_cmd(cmd)
            rate.sleep()

        self.stop_robot()
        rospy.sleep(0.2)

    # =========================================================
    # yaw 对齐
    # =========================================================
    def align_to_yaw(self, target_yaw):
        start_time = rospy.Time.now()
        rate = rospy.Rate(20)

        stable_count = 0
        required_stable_count = 6

        while not rospy.is_shutdown():
            pose = self.lookup_robot_pose()

            if pose is None:
                self.stop_robot()
                rate.sleep()
                continue

            _, _, yaw = pose

            err = normalize_angle(target_yaw - yaw)

            elapsed = (rospy.Time.now() - start_time).to_sec()

            if elapsed > self.align_timeout:
                rospy.logwarn(
                    "align_to_yaw timeout. current_err=%.3f",
                    err
                )
                self.stop_robot()

                if abs(err) < self.align_accept_yaw_error:
                    rospy.logwarn("align timeout but yaw is close enough. Accept.")
                    return True

                return False

            rospy.loginfo_throttle(
                0.5,
                "align yaw: target=%.3f current=%.3f err=%.3f",
                target_yaw,
                yaw,
                err
            )

            if abs(err) < self.yaw_tolerance:
                stable_count += 1
                self.stop_robot()

                if stable_count >= required_stable_count:
                    return True

                rate.sleep()
                continue

            stable_count = 0

            cmd = Twist()
            cmd.angular.z = self.clamp(
                self.kp_yaw * err,
                -self.max_align_wz,
                self.max_align_wz
            )

            if abs(cmd.angular.z) < 0.015:
                cmd.angular.z = 0.0

            self.publish_cmd(cmd)
            rate.sleep()

        return False

    def get_yaw_error(self, target_yaw):
        pose = self.lookup_robot_pose()

        if pose is None:
            return None

        _, _, yaw = pose

        return normalize_angle(target_yaw - yaw)

    # =========================================================
    # L 型挡板停车接受
    # =========================================================
    def is_obstacle_parking_good_enough(self, dist, yaw_err, enter_travel=0.0):
        front = self.get_sector_min_range(-15, 15)
        left_front = self.get_sector_min_range(15, 55)
        right_front = self.get_sector_min_range(-55, -15)

        front_danger = min(front, left_front, right_front)

        yaw_ok = abs(yaw_err) <= self.obstacle_park_yaw_tolerance
        front_ok = self.obstacle_park_front_min <= front_danger <= self.obstacle_park_front_max

        if dist <= self.obstacle_park_accept_dist and yaw_ok and front_ok:
            rospy.logwarn(
                "obstacle parking good enough by dist: dist=%.3f yaw_err=%.3f front=%.3f travel=%.3f",
                dist,
                yaw_err,
                front_danger,
                enter_travel
            )
            return True

        if enter_travel >= self.obstacle_park_enter_travel_min and yaw_ok and front_ok:
            rospy.logwarn(
                "obstacle parking good enough by travel: dist=%.3f yaw_err=%.3f front=%.3f travel=%.3f",
                dist,
                yaw_err,
                front_danger,
                enter_travel
            )
            return True

        return False

    # =========================================================
    # 【优化2】稳定位姿：多帧采样 + 平均，并返回方差
    # 返回 (mx, my, myaw, var_pos, var_yaw)
    # =========================================================
    def get_stable_pose(self):
        xs = []
        ys = []
        yaws = []

        for _ in range(self.stable_pose_samples):
            p = self.lookup_robot_pose()
            if p is not None:
                xs.append(p[0])
                ys.append(p[1])
                yaws.append(p[2])
            rospy.sleep(self.stable_pose_dt)

        if not xs:
            return None

        n = len(xs)
        mx = sum(xs) / n
        my = sum(ys) / n

        sy = sum(math.sin(a) for a in yaws)
        cy = sum(math.cos(a) for a in yaws)
        myaw = math.atan2(sy, cy)

        var_pos = sum((x - mx) ** 2 + (y - my) ** 2 for x, y in zip(xs, ys)) / n
        var_yaw = sum(normalize_angle(a - myaw) ** 2 for a in yaws) / n

        return mx, my, myaw, var_pos, var_yaw

    # =========================================================
    # 【优化1】进框前等定位收敛
    # 利用"机器人停下后定位能拉回来"的特性
    # =========================================================
    def wait_localization_settle(self):
        if not self.settle_before_enter:
            return

        self.stop_robot()
        rospy.sleep(0.15)

        start = rospy.Time.now()

        while not rospy.is_shutdown():
            sp = self.get_stable_pose()

            if sp is not None:
                _, _, _, var_pos, var_yaw = sp

                if var_pos <= self.settle_pos_var_thresh and var_yaw <= self.settle_yaw_var_thresh:
                    rospy.loginfo(
                        "localization settled: var_pos=%.6f var_yaw=%.5f",
                        var_pos,
                        var_yaw
                    )
                    return

                rospy.loginfo_throttle(
                    0.5,
                    "waiting localization settle: var_pos=%.6f var_yaw=%.5f",
                    var_pos,
                    var_yaw
                )

            if (rospy.Time.now() - start).to_sec() > self.settle_max_wait:
                rospy.logwarn("settle wait timeout. Continue entering anyway.")
                return

            rospy.sleep(0.05)

    # =========================================================
    # 【优化3】激光横向居中误差（base 坐标系，完全不依赖 map 定位）
    # 沿进框方向 axis 取前方走廊两侧挡板，求对称中心偏差
    # 返回 (lateral_err, pbx, pby)；无法判定返回 None
    #   lateral_err > 0 表示走廊中心在机器人左侧，应向左微调
    #   (pbx, pby) 为 base 系下"左方向"单位向量，用于施加修正速度
    # =========================================================
    def measure_laser_lateral_error(self, axis_x, axis_y, robot_yaw):
        if self.latest_scan is None:
            return None

        msg = self.latest_scan

        if msg.angle_increment == 0.0:
            return None

        # map 系进框方向 -> base 系（map->base 旋转为 -robot_yaw）
        mbx = math.cos(robot_yaw) * axis_x + math.sin(robot_yaw) * axis_y
        mby = -math.sin(robot_yaw) * axis_x + math.cos(robot_yaw) * axis_y

        # base 系"左方向"（垂直于运动方向，左正）
        pbx = -mby
        pby = mbx

        left_s = []
        right_s = []

        half = self.center_corridor_half

        for i, r in enumerate(msg.ranges):
            if math.isnan(r) or math.isinf(r):
                continue

            if r <= msg.range_min or r >= msg.range_max:
                continue

            a = msg.angle_min + i * msg.angle_increment

            lx = r * math.cos(a)
            ly = r * math.sin(a)

            f = lx * mbx + ly * mby      # 前向分量
            s = lx * pbx + ly * pby      # 横向分量（左正）

            if f < self.center_forward_min or f > self.center_forward_max:
                continue

            if abs(s) > half:
                continue

            if s > 0:
                left_s.append(s)
            else:
                right_s.append(s)

        left_found = len(left_s) >= self.center_min_side_points
        right_found = len(right_s) >= self.center_min_side_points

        # 只有两侧都检测到挡板才做对称居中，否则不干预（避免单侧误拉）
        if left_found and right_found:
            left_lat = min(left_s)     # 左侧最近挡板（最小正值）
            right_lat = max(right_s)   # 右侧最近挡板（最接近0的负值）
            lateral_err = (left_lat + right_lat) / 2.0
            return lateral_err, pbx, pby

        return None

    # =========================================================
    # 入框（已优化：稳定位姿停车判定 + 激光横向居中）
    # =========================================================
    def enter_to_target_box(self, entry):
        start_time = rospy.Time.now()
        rate = rospy.Rate(20)

        stable_count = 0
        required_stable_count = 5
        block_count = 0

        axis_x = entry["axis_x"]
        axis_y = entry["axis_y"]
        target_yaw = entry["target_yaw"]

        park_x = self.target_x + axis_x * self.park_deep_offset
        park_y = self.target_y + axis_y * self.park_deep_offset

        start_pose = self.lookup_robot_pose()

        if start_pose is None:
            enter_start_x = None
            enter_start_y = None
        else:
            enter_start_x = start_pose[0]
            enter_start_y = start_pose[1]

        rospy.loginfo(
            "park point: center=(%.3f, %.3f), park=(%.3f, %.3f), deep_offset=%.3f",
            self.target_x,
            self.target_y,
            park_x,
            park_y,
            self.park_deep_offset
        )

        while not rospy.is_shutdown():
            pose = self.lookup_robot_pose()

            if pose is None:
                self.stop_robot()
                rate.sleep()
                continue

            rx, ry, ryaw = pose

            ex_map = park_x - rx
            ey_map = park_y - ry

            dist = math.sqrt(ex_map * ex_map + ey_map * ey_map)
            yaw_err = normalize_angle(target_yaw - ryaw)

            if enter_start_x is None:
                enter_travel = 0.0
            else:
                enter_travel = math.sqrt(
                    (rx - enter_start_x) * (rx - enter_start_x) +
                    (ry - enter_start_y) * (ry - enter_start_y)
                )

            elapsed = (rospy.Time.now() - start_time).to_sec()

            if elapsed > self.enter_timeout:
                rospy.logwarn(
                    "enter timeout. dist=%.3f yaw_err=%.3f travel=%.3f. Stop here.",
                    dist,
                    yaw_err,
                    enter_travel
                )
                self.stop_robot()

                if dist < self.xy_tolerance * 1.5 and abs(yaw_err) < self.yaw_tolerance:
                    rospy.logwarn("enter timeout but close enough. Accept.")
                    return True

                if self.is_obstacle_parking_good_enough(dist, yaw_err, enter_travel):
                    rospy.logwarn("enter timeout but obstacle parking is good enough. Accept.")
                    return True

                rospy.logwarn("enter timeout and still far. Reject this entry.")
                return False

            rospy.loginfo_throttle(
                0.5,
                "enter %s: dist=%.3f yaw_err=%.3f travel=%.3f",
                entry["name"],
                dist,
                yaw_err,
                enter_travel
            )

            # ---------- 到位候选：用稳定位姿二次确认，抵消瞬时抖动 ----------
            reached_now = (dist < self.xy_tolerance and abs(yaw_err) < self.yaw_tolerance)
            obstacle_ok = self.is_obstacle_parking_good_enough(dist, yaw_err, enter_travel)

            if reached_now or obstacle_ok:
                self.stop_robot()

                accept = True

                # 【优化2】对中心位置到位的情况，停车后用平均位姿复核一次
                if reached_now and self.enter_use_stable_pose:
                    rospy.sleep(0.05)
                    sp = self.get_stable_pose()
                    if sp is not None:
                        smx, smy, smyaw, vpos, vyaw = sp
                        sdist = math.sqrt((park_x - smx) ** 2 + (park_y - smy) ** 2)
                        syaw_err = normalize_angle(target_yaw - smyaw)
                        rospy.loginfo(
                            "stable check: sdist=%.3f syaw_err=%.3f var_pos=%.6f",
                            sdist, syaw_err, vpos
                        )
                        # 平均位姿仍满足容差才真正接受
                        accept = (sdist < self.xy_tolerance * 1.2 and
                                  abs(syaw_err) < self.yaw_tolerance * 1.2)

                if accept:
                    stable_count += 1
                    if stable_count >= required_stable_count:
                        if obstacle_ok and not reached_now:
                            rospy.logwarn("SUCCESS: obstacle based parking accepted.")
                        else:
                            rospy.logwarn("SUCCESS: stable-pose confirmed parking accepted.")
                        return True
                    rate.sleep()
                    continue
                else:
                    stable_count = 0
                    rate.sleep()
                    continue

            stable_count = 0

            progress = ex_map * axis_x + ey_map * axis_y

            if progress <= 0.0 and dist < self.xy_tolerance * 1.5:
                rospy.logwarn("passed park point along entry axis. Stop as success.")
                self.stop_robot()
                return True

            max_v = self.max_enter_v

            if dist < self.precise_dist:
                max_v = min(max_v, self.precise_v)
            elif dist < self.slow_dist:
                max_v = min(max_v, self.slow_v)

            speed = self.kp_enter * max(0.0, progress)
            speed = max(0.0, min(speed, max_v))

            if speed < self.min_v:
                speed = 0.0

            vx_map = axis_x * speed
            vy_map = axis_y * speed

            cmd = self.map_velocity_to_base_cmd(vx_map, vy_map, ryaw)

            cmd.angular.z = self.clamp(
                self.kp_yaw * yaw_err,
                -self.max_enter_wz,
                self.max_enter_wz
            )

            if abs(cmd.angular.z) < 0.012:
                cmd.angular.z = 0.0

            # 【优化3】进入近距离段后，用激光对两侧挡板做横向居中
            if self.enable_laser_centering and dist < self.center_enable_dist:
                res = self.measure_laser_lateral_error(axis_x, axis_y, ryaw)
                if res is not None:
                    lateral_err, pbx, pby = res
                    vcorr = self.clamp(
                        self.center_kp * lateral_err,
                        -self.center_max_vy,
                        self.center_max_vy
                    )
                    cmd.linear.x += pbx * vcorr
                    cmd.linear.y += pby * vcorr
                    rospy.loginfo_throttle(
                        0.5,
                        "laser centering: err=%.3f vcorr=%.3f",
                        lateral_err,
                        vcorr
                    )

            cmd = self.apply_laser_safety(cmd)

            blocked_now = (
                abs(cmd.linear.x) < 1e-4 and abs(cmd.linear.y) < 1e-4
            )

            if self.last_block_front:
                if self.is_obstacle_parking_good_enough(dist, yaw_err, enter_travel):
                    stable_count += 1
                    self.stop_robot()

                    if stable_count >= required_stable_count:
                        rospy.logwarn("SUCCESS: front blocked but parking condition accepted.")
                        return True

                    rate.sleep()
                    continue

                if dist > 0.25:
                    blocked_now = True

            if blocked_now:
                block_count += 1

                rospy.logwarn_throttle(
                    0.5,
                    "entry %s blocked count=%d dist=%.3f travel=%.3f",
                    entry["name"],
                    block_count,
                    dist,
                    enter_travel
                )

                if block_count >= self.max_block_count:
                    if self.is_obstacle_parking_good_enough(dist, yaw_err, enter_travel):
                        rospy.logwarn("blocked but obstacle parking is good enough. Accept.")
                        self.stop_robot()
                        return True

                    rospy.logwarn("entry %s blocked too many times. Escape and reject.", entry["name"])
                    self.backoff_from_obstacle()
                    return False
            else:
                block_count = 0

            self.publish_cmd(cmd)
            rate.sleep()

        return False

    # =========================================================
    # 退出
    # =========================================================
    def exit_from_entry(self, entry):
        axis_x = entry["axis_x"]
        axis_y = entry["axis_y"]

        start_pose = self.lookup_robot_pose()

        if start_pose is None:
            rospy.logwarn("exit_from_entry: no start pose.")
            return False

        sx, sy, _ = start_pose

        rate = rospy.Rate(20)
        start_time = rospy.Time.now()

        max_time = max(4.0, self.exit_distance / max(0.01, self.exit_speed) + 4.0)

        while not rospy.is_shutdown():
            pose = self.lookup_robot_pose()

            if pose is None:
                self.stop_robot()
                rate.sleep()
                continue

            rx, ry, ryaw = pose

            moved = math.sqrt((rx - sx) * (rx - sx) + (ry - sy) * (ry - sy))

            if moved >= self.exit_distance:
                rospy.loginfo("exit_from_entry finished. moved=%.3f", moved)
                self.stop_robot()
                return True

            if (rospy.Time.now() - start_time).to_sec() > max_time:
                rospy.logwarn("exit_from_entry timeout. moved=%.3f", moved)
                self.stop_robot()
                return False

            vx_map = -axis_x * self.exit_speed
            vy_map = -axis_y * self.exit_speed

            cmd = self.map_velocity_to_base_cmd(vx_map, vy_map, ryaw)
            cmd.angular.z = 0.0

            cmd = self.apply_exit_laser_safety(cmd)

            self.publish_cmd(cmd)
            rate.sleep()

        return False

    # =========================================================
    # 坐标速度转换
    # =========================================================
    def map_velocity_to_base_cmd(self, vx_map, vy_map, yaw):
        cmd = Twist()

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        cmd.linear.x = cos_yaw * vx_map + sin_yaw * vy_map
        cmd.linear.y = -sin_yaw * vx_map + cos_yaw * vy_map
        cmd.angular.z = 0.0

        return cmd

    # =========================================================
    # cmd_vel 平滑
    # =========================================================
    def smooth_cmd(self, cmd):
        if not self.enable_cmd_smoothing:
            return cmd

        now = rospy.Time.now()
        dt = (now - self.last_cmd_time).to_sec()

        if dt <= 0.0 or dt > 0.5:
            dt = 0.05

        out = Twist()

        out.linear.x = self.limit_delta(
            cmd.linear.x,
            self.last_cmd.linear.x,
            self.max_acc_x * dt
        )

        out.linear.y = self.limit_delta(
            cmd.linear.y,
            self.last_cmd.linear.y,
            self.max_acc_y * dt
        )

        out.angular.z = self.limit_delta(
            cmd.angular.z,
            self.last_cmd.angular.z,
            self.max_acc_wz * dt
        )

        self.last_cmd = out
        self.last_cmd_time = now

        return out

    def publish_cmd(self, cmd):
        cmd = self.smooth_cmd(cmd)
        self.cmd_pub.publish(cmd)

    # =========================================================
    # 雷达安全
    # =========================================================
    def apply_laser_safety(self, cmd):
        front = self.get_sector_min_range(-15, 15)
        left_front = self.get_sector_min_range(15, 55)
        right_front = self.get_sector_min_range(-55, -15)
        left_side = self.get_sector_min_range(55, 125)
        right_side = self.get_sector_min_range(-125, -55)

        front_danger = min(front, left_front, right_front)
        left_danger = min(left_front, left_side)
        right_danger = min(right_front, right_side)
        nearest = min(front_danger, left_side, right_side)

        self.last_block_front = False
        self.last_block_left = False
        self.last_block_right = False
        self.last_block_any = False

        rospy.loginfo_throttle(
            0.5,
            "scan front=%.3f lf=%.3f rf=%.3f left=%.3f right=%.3f nearest=%.3f cmd=(%.3f, %.3f, %.3f)",
            front,
            left_front,
            right_front,
            left_side,
            right_side,
            nearest,
            cmd.linear.x,
            cmd.linear.y,
            cmd.angular.z
        )

        if nearest < self.any_stop_dist:
            self.last_block_any = True
            rospy.logwarn_throttle(
                0.5,
                "too close nearest=%.3f < %.3f, stop translation",
                nearest,
                self.any_stop_dist
            )
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            return cmd

        if cmd.linear.x > 0.0:
            if front_danger < self.front_stop_dist:
                self.last_block_front = True
                rospy.logwarn_throttle(
                    0.5,
                    "front blocked %.3f < %.3f, stop forward",
                    front_danger,
                    self.front_stop_dist
                )
                cmd.linear.x = 0.0

            elif front_danger < self.front_slow_dist:
                rospy.logwarn_throttle(
                    0.5,
                    "front slow %.3f < %.3f",
                    front_danger,
                    self.front_slow_dist
                )
                cmd.linear.x = min(cmd.linear.x, self.front_slow_v)

        if cmd.linear.y > 0.0:
            if left_danger < self.side_stop_dist:
                self.last_block_left = True
                rospy.logwarn_throttle(
                    0.5,
                    "left blocked %.3f < %.3f, stop left strafe",
                    left_danger,
                    self.side_stop_dist
                )
                cmd.linear.y = 0.0

        if cmd.linear.y < 0.0:
            if right_danger < self.side_stop_dist:
                self.last_block_right = True
                rospy.logwarn_throttle(
                    0.5,
                    "right blocked %.3f < %.3f, stop right strafe",
                    right_danger,
                    self.side_stop_dist
                )
                cmd.linear.y = 0.0

        if front_danger < self.front_stop_dist and abs(cmd.linear.x) > 0.0:
            self.last_block_front = True
            cmd.linear.x = 0.0

        if abs(cmd.linear.x) < self.min_v:
            cmd.linear.x = 0.0

        if abs(cmd.linear.y) < self.min_v:
            cmd.linear.y = 0.0

        return cmd

    def apply_exit_laser_safety(self, cmd):
        front = self.get_sector_min_range(-15, 15)

        rear = min(
            self.get_sector_min_range(160, 180),
            self.get_sector_min_range(-180, -160)
        )

        left_side = self.get_sector_min_range(55, 125)
        right_side = self.get_sector_min_range(-125, -55)

        nearest = min(front, rear, left_side, right_side)

        rospy.loginfo_throttle(
            0.5,
            "exit scan front=%.3f rear=%.3f left=%.3f right=%.3f nearest=%.3f cmd=(%.3f, %.3f)",
            front,
            rear,
            left_side,
            right_side,
            nearest,
            cmd.linear.x,
            cmd.linear.y
        )

        if nearest < self.any_stop_dist:
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            return cmd

        if cmd.linear.x > 0.0 and front < self.front_stop_dist:
            cmd.linear.x = 0.0

        if cmd.linear.x < 0.0 and rear < self.front_stop_dist:
            cmd.linear.x = 0.0

        if cmd.linear.y > 0.0 and left_side < self.side_stop_dist:
            cmd.linear.y = 0.0

        if cmd.linear.y < 0.0 and right_side < self.side_stop_dist:
            cmd.linear.y = 0.0

        if abs(cmd.linear.x) < self.min_v:
            cmd.linear.x = 0.0

        if abs(cmd.linear.y) < self.min_v:
            cmd.linear.y = 0.0

        cmd.angular.z = 0.0

        return cmd

    # =========================================================
    # 位姿
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

        rospy.loginfo("distance to entry: %.3f tolerance=%.3f", dist, tolerance)

        return dist < tolerance

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

    def stop_robot(self):
        cmd = Twist()

        self.last_cmd = Twist()
        self.last_cmd_time = rospy.Time.now()

        for _ in range(8):
            self.cmd_pub.publish(cmd)
            rospy.sleep(0.03)

    @staticmethod
    def clamp(v, vmin, vmax):
        return max(vmin, min(vmax, v))

    @staticmethod
    def limit_delta(target, current, max_delta):
        if target > current + max_delta:
            return current + max_delta

        if target < current - max_delta:
            return current - max_delta

        return target


if __name__ == "__main__":
    rospy.init_node("auto_single_point_test")
    try:
        instance = AutoSinglePointTest()
        instance.run()
    except rospy.ROSInterruptException:
        pass