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


class PDController:
    """单轴PD控制器"""
    def __init__(self, kp, kd):
        self.kp = kp
        self.kd = kd
        self.last_error = 0.0
        self.last_time = None

    def update(self, error, now):
        if self.last_time is None:
            dt = 0.05
        else:
            dt = (now - self.last_time).to_sec()
            if dt <= 0.0:
                dt = 0.05
        de = (error - self.last_error) / dt
        self.last_error = error
        self.last_time = now
        return self.kp * error + self.kd * de

    def reset(self):
        self.last_error = 0.0
        self.last_time = None


class AutoSinglePointTest:
    def __init__(self, target_x=None, target_y=None, target_yaw_deg=None):
        # =====================================================
        # 目标点
        # =====================================================
        self.target_x = target_x if target_x is not None else rospy.get_param("target_x", 0.0)
        self.target_y = target_y if target_y is not None else rospy.get_param("target_y", 0.0)
        self.target_yaw_deg = target_yaw_deg if target_yaw_deg is not None else rospy.get_param("target_yaw", 0.0)
        self.target_yaw = math.radians(self.target_yaw_deg)

        # =====================================================
        # Phase 0: move_base 直达目标中心
        # =====================================================
        self.enable_direct_center = rospy.get_param("enable_direct_center", True)
        self.direct_center_timeout =5.0          # 硬编码，不读 param server
        self.direct_center_tolerance = rospy.get_param("direct_center_tolerance", 0.10)
        self.direct_center_oscillation_window = 3.0
        self.direct_center_oscillation_min_displacement = 0.2

        # =====================================================
        # 入口参数
        # =====================================================
        self.entry_offset = rospy.get_param("entry_offset", 0.34)

        # =====================================================
        # 入口识别评分参数
        # =====================================================
        self.enable_entry_recognition = rospy.get_param("enable_entry_recognition", True)
        self.target_box_half_size = rospy.get_param("target_box_half_size", 0.24)
        self.side_detect_width = rospy.get_param("side_detect_width", 0.12)
        self.side_detect_min_points = int(rospy.get_param("side_detect_min_points", 4))
        self.enable_opening_circle_detect = rospy.get_param("enable_opening_circle_detect", True)
        self.opening_detect_radius = rospy.get_param("opening_detect_radius", 0.30)
        self.opening_ring_width = rospy.get_param("opening_ring_width", 0.08)
        self.opening_min_clear_diff = int(rospy.get_param("opening_min_clear_diff", 3))
        self.opening_best_bonus = rospy.get_param("opening_best_bonus", 45.0)
        self.opening_not_best_penalty = rospy.get_param("opening_not_best_penalty", 22.0)
        self.opening_count_weight = rospy.get_param("opening_count_weight", 10.0)
        self.opening_unknown_penalty = rospy.get_param("opening_unknown_penalty", 0.0)
        self.path_corridor_width = rospy.get_param("path_corridor_width", 0.36)
        self.path_corridor_min_points = int(rospy.get_param("path_corridor_min_points", 4))
        self.path_corridor_ignore_near_start = rospy.get_param("path_corridor_ignore_near_start", 0.06)
        self.path_corridor_ignore_near_goal = rospy.get_param("path_corridor_ignore_near_goal", 0.08)
        self.corridor_width = rospy.get_param("corridor_width", 0.32)
        self.corridor_min_points = int(rospy.get_param("corridor_min_points", 5))
        self.scan_memory_time = rospy.get_param("scan_memory_time", 0.8)
        self.recognition_max_range = rospy.get_param("recognition_max_range", 1.6)
        self.scan_memory = []

        # =====================================================
        # Phase 1a: move_base 到入口
        # =====================================================
        self.entry_nav_timeout = 8.0          # 硬编码
        self.entry_nav_tolerance = rospy.get_param("entry_nav_tolerance", 0.15)

        # =====================================================
        # PD 控制参数
        # =====================================================
        self.pid_kp_xy = rospy.get_param("pid_kp_xy", 0.6)
        self.pid_kd_xy = rospy.get_param("pid_kd_xy", 0.2)
        self.pid_kp_yaw = rospy.get_param("pid_kp_yaw", 1.5)
        self.pid_kd_yaw = rospy.get_param("pid_kd_yaw", 0.3)
        self.pid_max_v = rospy.get_param("pid_max_v", 0.25)
        self.pid_max_wz = rospy.get_param("pid_max_wz", 1.6)
        self.pid_yaw_align_timeout = rospy.get_param("pid_yaw_align_timeout", 4.0)
        self.pid_translate_timeout = rospy.get_param("pid_translate_timeout", 9.0)
        self.pos_tolerance = rospy.get_param("pos_tolerance", 0.02)
        self.yaw_tolerance = rospy.get_param("yaw_tolerance", 0.05)

        # =====================================================
        # Phase 3: 激光挡板精调
        # =====================================================
        self.fine_tune_enabled = rospy.get_param("fine_tune_enabled", True)
        self.fine_tune_timeout = rospy.get_param("fine_tune_timeout", 6.0)
        self.fine_tune_front_back_target = rospy.get_param("fine_tune_front_back_target", 0.24)
        self.fine_tune_side_target = rospy.get_param("fine_tune_side_target", 0.20)
        self.fine_tune_tolerance = rospy.get_param("fine_tune_tolerance", 0.03)
        self.fine_tune_kp = rospy.get_param("fine_tune_kp", 0.3)
        self.fine_tune_kd = rospy.get_param("fine_tune_kd", 0.1)
        self.fine_tune_max_v = rospy.get_param("fine_tune_max_v", 0.03)

        # =====================================================
        # Phase 4: 逃逸
        # =====================================================
        self.escape_enabled = rospy.get_param("escape_enabled", True)
        self.escape_distance = rospy.get_param("escape_distance", 0.35)
        self.escape_speed = rospy.get_param("escape_speed", 0.20)
        self.escape_timeout = rospy.get_param("escape_timeout", 5.0)

        # 运行时状态
        self.best_entry = None
        self.parking_done = False
        self.parking_start_time = None
        self.current_phase = "INIT"

        # =====================================================
        # 雷达安全
        # =====================================================
        self.scan_topic = rospy.get_param("scan_topic", "/scan_filtered")
        self.front_stop_dist = rospy.get_param("front_stop_dist", 0.17)
        self.front_slow_dist = rospy.get_param("front_slow_dist", 0.30)
        self.front_slow_v = rospy.get_param("front_slow_v", 0.034)
        self.side_stop_dist = rospy.get_param("side_stop_dist", 0.16)
        self.any_stop_dist = rospy.get_param("any_stop_dist", 0.085)
        self.min_v = rospy.get_param("min_v", 0.004)

        self.last_block_front = False
        self.last_block_left = False
        self.last_block_right = False
        self.last_block_any = False

        # =====================================================
        # cmd_vel 平滑
        # =====================================================
        self.enable_cmd_smoothing = rospy.get_param("enable_cmd_smoothing", True)
        self.max_acc_x = rospy.get_param("max_acc_x", 1.0)
        self.max_acc_y = rospy.get_param("max_acc_y", 1.0)
        self.max_acc_wz = rospy.get_param("max_acc_wz", 2.3)
        self.last_cmd = Twist()
        self.last_cmd_time = rospy.Time.now()

        # =====================================================
        # 坐标系
        # =====================================================
        self.map_frame = rospy.get_param("map_frame", "map")
        self.base_frame = rospy.get_param("base_frame", "base_footprint")

        # =====================================================
        # ROS 接口
        # =====================================================
        self.latest_scan = None
        self.scan_sub = rospy.Subscriber(self.scan_topic, LaserScan, self.scan_cb, queue_size=1)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.tf_listener = tf.TransformListener()
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")
        if not self.move_base.wait_for_server(rospy.Duration(30.0)):
            rospy.logerr("move_base action server not available after 30s.")
            raise RuntimeError("move_base not available")

        rospy.sleep(1.0)
        rospy.loginfo("Auto single point target: x=%.3f y=%.3f yaw_deg=%.1f",
                       self.target_x, self.target_y, self.target_yaw_deg)

    # =========================================================
    # 回调
    # =========================================================
    def scan_cb(self, msg):
        self.latest_scan = msg

    # =========================================================
    # 阶段日志
    # =========================================================
    def parking_elapsed(self):
        if self.parking_start_time is None:
            return 0.0
        return (rospy.Time.now() - self.parking_start_time).to_sec()

    def phase_start(self, phase, detail=""):
        self.current_phase = phase
        start_time = rospy.Time.now()
        suffix = (" " + detail) if detail else ""
        rospy.loginfo("[PARK][%s][START][+%.2fs]%s",
                      phase, self.parking_elapsed(), suffix)
        return start_time

    def phase_end(self, phase, start_time, status, detail=""):
        dt = (rospy.Time.now() - start_time).to_sec() if start_time is not None else 0.0
        suffix = (" " + detail) if detail else ""
        rospy.loginfo("[PARK][%s][%s][dt=%.2fs][+%.2fs]%s",
                      phase, status, dt, self.parking_elapsed(), suffix)

    def phase_skip(self, phase, detail=""):
        suffix = (" " + detail) if detail else ""
        rospy.loginfo("[PARK][%s][SKIP][+%.2fs]%s",
                      phase, self.parking_elapsed(), suffix)

    # =========================================================
    # 主流程
    # =========================================================
    def run(self):
        self.parking_start_time = rospy.Time.now()
        run_start = self.phase_start(
            "RUN",
            "target=(%.3f, %.3f, %.1fdeg)" %
            (self.target_x, self.target_y, self.target_yaw_deg)
        )

        # ---- Phase 0: move_base 直达目标中心 ----
        if self.enable_direct_center:
            phase_start = self.phase_start(
                "PHASE0_DIRECT_CENTER",
                "timeout=%.1fs tolerance=%.3f" %
                (self.direct_center_timeout, self.direct_center_tolerance)
            )
            direct_ok = self.try_direct_goal_center()
            self.phase_end(
                "PHASE0_DIRECT_CENTER",
                phase_start,
                "OK" if direct_ok else "FAIL",
                "next=%s" % ("finish" if direct_ok else "entry_based")
            )
            if direct_ok:
                rospy.loginfo("SUCCESS: Phase 0 direct center parking finished.")
                self.stop_robot()
                self.parking_done = True
                self.phase_end("RUN", run_start, "OK", "finished_by=direct_center")
                return
            rospy.logwarn("Phase 0 failed. Switch to entry-based parking.")
        else:
            self.phase_skip("PHASE0_DIRECT_CENTER", "enable_direct_center=false")

        # ---- 生成入口 + 评分 + 取最佳 ----
        phase_start = self.phase_start(
            "ENTRY_SELECT",
            "recognition=%s" % str(self.enable_entry_recognition)
        )
        entries = self.generate_entries()
        if self.enable_entry_recognition:
            entries = self.sort_entries_by_obstacle_score(entries)
        else:
            entries = self.sort_entries_by_robot_position(entries)
        best_entry = entries[0]
        self.best_entry = best_entry  # 保存，供 escape() 使用

        rospy.loginfo("Best entry: %s score=%.3f entry=(%.3f, %.3f)",
                       best_entry["name"], best_entry.get("score", -1),
                       best_entry["entry_x"], best_entry["entry_y"])
        self.phase_end(
            "ENTRY_SELECT",
            phase_start,
            "OK",
            "best=%s score=%.3f entry=(%.3f, %.3f)" %
            (best_entry["name"], best_entry.get("score", -1),
             best_entry["entry_x"], best_entry["entry_y"])
        )

        # ---- Phase 1a: move_base 到入口 ----
        phase_start = self.phase_start(
            "PHASE1A_MOVE_BASE_ENTRY",
            "entry=%s timeout=%.1fs tolerance=%.3f" %
            (best_entry["name"], self.entry_nav_timeout, self.entry_nav_tolerance)
        )
        entry_ok = self.goto_entry_polling(best_entry)
        self.phase_end(
            "PHASE1A_MOVE_BASE_ENTRY",
            phase_start,
            "OK" if entry_ok else "FAIL",
            "entry=%s" % best_entry["name"]
        )

        # ---- Phase 1b: PID 到入口 (move_base 失败时) ----
        if not entry_ok:
            rospy.logwarn("Phase 1a move_base failed. Switch to Phase 1b PID to entry.")
            phase_start = self.phase_start(
                "PHASE1B_PID_ENTRY",
                "entry=%s timeout=%.1fs max_v=%.3f max_wz=%.3f" %
                (best_entry["name"], self.pid_translate_timeout,
                 self.pid_max_v, self.pid_max_wz)
            )
            entry_ok = self.pid_goto_point(
                best_entry["entry_x"], best_entry["entry_y"], best_entry["target_yaw"],
                "entry")
            self.phase_end(
                "PHASE1B_PID_ENTRY",
                phase_start,
                "OK" if entry_ok else "FAIL",
                "entry=%s" % best_entry["name"]
            )
        else:
            self.phase_skip("PHASE1B_PID_ENTRY", "phase1a_ok=true")

        if not entry_ok:
            rospy.logwarn("Phase 1b PID to entry failed. Stop here.")
            self.stop_robot()
            self.phase_end("RUN", run_start, "FAIL", "failed_at=entry")
            return

        # ---- Phase 2: PID 入口 → 目标中心 ----
        phase_start = self.phase_start(
            "PHASE2_PID_CENTER",
            "timeout=%.1fs max_v=%.3f max_wz=%.3f" %
            (self.pid_translate_timeout, self.pid_max_v, self.pid_max_wz)
        )
        center_ok = self.pid_goto_point(
            self.target_x, self.target_y, self.target_yaw, "center")
        self.phase_end(
            "PHASE2_PID_CENTER",
            phase_start,
            "OK" if center_ok else "TIMEOUT_ACCEPT",
            "target=(%.3f, %.3f)" % (self.target_x, self.target_y)
        )

        if center_ok:
            rospy.loginfo("SUCCESS: parked at target center via entry %s.", best_entry["name"])
        else:
            rospy.logwarn("Phase 2 PID to center timeout. Accept current position.")

        # ---- Phase 3: 激光挡板精调 ----
        if self.fine_tune_enabled:
            phase_start = self.phase_start(
                "PHASE3_FINE_TUNE",
                "timeout=%.1fs tolerance=%.3f max_v=%.3f" %
                (self.fine_tune_timeout, self.fine_tune_tolerance, self.fine_tune_max_v)
            )
            fine_status, fine_detail = self.precision_fine_tune()
            self.phase_end("PHASE3_FINE_TUNE", phase_start, fine_status, fine_detail)
        else:
            self.phase_skip("PHASE3_FINE_TUNE", "fine_tune_enabled=false")

        self.stop_robot()
        self.parking_done = True
        self.phase_end("RUN", run_start, "OK", "finished_by=entry_based")

    # =========================================================
    # Phase 0: move_base 直达目标中心 (轮询 + 抽搐检测)
    # =========================================================
    def try_direct_goal_center(self):
        rospy.loginfo("Phase 0: direct center x=%.3f y=%.3f yaw_deg=%.1f",
                       self.target_x, self.target_y, self.target_yaw_deg)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.map_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.target_x
        goal.target_pose.pose.position.y = self.target_y
        goal.target_pose.pose.position.z = 0.0
        q = yaw_to_quat(self.target_yaw)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.move_base.send_goal(goal)

        # Timer 硬中断：3s 到立刻 cancel，不受主循环影响
        cancelled = [False]

        def hard_cancel(event):
            rospy.logwarn("Phase 0: TIMER CANCEL fired")
            self.move_base.cancel_goal()
            cancelled[0] = True

        timer = rospy.Timer(rospy.Duration(self.direct_center_timeout), hard_cancel, oneshot=True)

        poll_interval = 0.1
        start_time = rospy.Time.now()
        pose_history = []
        dist = None

        while not rospy.is_shutdown():
            if cancelled[0]:
                rospy.logwarn("Phase 0: cancelled by timer. dist=%.3f", dist if dist else -1)
                self.stop_robot()
                rospy.sleep(0.2)
                timer.shutdown()
                return False

            elapsed = (rospy.Time.now() - start_time).to_sec()

            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Phase 0: move_base SUCCEEDED in %.1fs", elapsed)
                timer.shutdown()
                return True

            pose = self.lookup_robot_pose()
            if pose is not None:
                now = rospy.Time.now()
                pose_history.append((now, pose[0], pose[1]))
                pose_history = [(t, px, py) for t, px, py in pose_history
                                if (now - t).to_sec() <= self.direct_center_oscillation_window]

                dist = math.sqrt((self.target_x - pose[0]) ** 2 + (self.target_y - pose[1]) ** 2)
                yaw_err = abs(normalize_angle(self.target_yaw - pose[2]))
                if dist < self.direct_center_tolerance and yaw_err < self.yaw_tolerance:
                    rospy.logwarn("Phase 0: close enough (dist=%.3f yaw_err=%.3f). Accept.", dist, yaw_err)
                    self.move_base.cancel_goal()
                    self.stop_robot()
                    rospy.sleep(0.2)
                    timer.shutdown()
                    return True

                if len(pose_history) >= 2:
                    window_dt = (pose_history[-1][0] - pose_history[0][0]).to_sec()
                    if window_dt >= self.direct_center_oscillation_window:
                        dx = pose_history[-1][1] - pose_history[0][1]
                        dy = pose_history[-1][2] - pose_history[0][2]
                        displacement = math.sqrt(dx * dx + dy * dy)
                        if displacement < self.direct_center_oscillation_min_displacement:
                            rospy.logwarn("Phase 0: oscillation (disp=%.3fm < %.2fm). Cancel.",
                                          displacement, self.direct_center_oscillation_min_displacement)
                            self.move_base.cancel_goal()
                            self.stop_robot()
                            rospy.sleep(0.2)
                            timer.shutdown()
                            return False

            if elapsed > self.direct_center_timeout:
                rospy.logwarn("Phase 0: timeout %.1fs", elapsed)
                self.move_base.cancel_goal()
                self.stop_robot()
                rospy.sleep(0.2)
                timer.shutdown()
                return False

            rospy.sleep(poll_interval)

        return False

    # =========================================================
    # Phase 1a: move_base 到入口 (轮询)
    # =========================================================
    def goto_entry_polling(self, entry):
        x = entry["entry_x"]
        y = entry["entry_y"]
        yaw = entry["entry_yaw"]

        rospy.loginfo("Phase 1a: move_base to entry %s (%.3f, %.3f)",
                       entry["name"], x, y)

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

        # Timer 硬中断
        cancelled = [False]

        def hard_cancel(event):
            rospy.logwarn("Phase 1a: TIMER CANCEL fired")
            self.move_base.cancel_goal()
            cancelled[0] = True

        timer = rospy.Timer(rospy.Duration(self.entry_nav_timeout), hard_cancel, oneshot=True)

        poll_interval = 0.1
        start_time = rospy.Time.now()
        pose_history = []
        dist = None

        while not rospy.is_shutdown():
            if cancelled[0]:
                rospy.logwarn("Phase 1a: cancelled by timer. dist=%.3f", dist if dist else -1)
                self.stop_robot()
                rospy.sleep(0.2)
                timer.shutdown()
                return False

            elapsed = (rospy.Time.now() - start_time).to_sec()

            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Phase 1a: move_base reached entry SUCCEEDED in %.1fs", elapsed)
                timer.shutdown()
                return True

            dist = self.distance_to_point(x, y)
            if dist is not None and dist < self.entry_nav_tolerance:
                rospy.logwarn("Phase 1a: near entry (dist=%.3f) after %.1fs. Accept.", dist, elapsed)
                self.move_base.cancel_goal()
                self.stop_robot()
                rospy.sleep(0.2)
                timer.shutdown()
                return True

            pose = self.lookup_robot_pose()
            if pose is not None:
                now = rospy.Time.now()
                pose_history.append((now, pose[0], pose[1]))
                pose_history = [(t, px, py) for t, px, py in pose_history
                                if (now - t).to_sec() <= self.direct_center_oscillation_window]

                if len(pose_history) >= 2:
                    window_dt = (pose_history[-1][0] - pose_history[0][0]).to_sec()
                    if window_dt >= self.direct_center_oscillation_window:
                        dx = pose_history[-1][1] - pose_history[0][1]
                        dy = pose_history[-1][2] - pose_history[0][2]
                        displacement = math.sqrt(dx * dx + dy * dy)
                        if displacement < self.direct_center_oscillation_min_displacement:
                            rospy.logwarn("Phase 1a: oscillation (disp=%.3fm < %.2fm). Cancel.",
                                          displacement, self.direct_center_oscillation_min_displacement)
                            self.move_base.cancel_goal()
                            self.stop_robot()
                            rospy.sleep(0.2)
                            timer.shutdown()
                            return False

            if elapsed > self.entry_nav_timeout:
                rospy.logwarn("Phase 1a: timeout %.1fs", elapsed)
                self.move_base.cancel_goal()
                self.stop_robot()
                rospy.sleep(0.2)
                timer.shutdown()
                return False

            rospy.sleep(poll_interval)

        return False

    # =========================================================
    # PID 航向对齐 (纯旋转)
    # =========================================================
    def pid_align_yaw(self, target_yaw, timeout=None):
        if timeout is None:
            timeout = self.pid_yaw_align_timeout

        rospy.loginfo("pid_align_yaw: target=%.3f", target_yaw)
        rate = rospy.Rate(20)
        start_time = rospy.Time.now()
        pd_yaw = PDController(self.pid_kp_yaw, self.pid_kd_yaw)
        stable_count = 0

        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed > timeout:
                rospy.logwarn("pid_align_yaw timeout. Accept current yaw.")
                self.stop_robot()
                return False

            pose = self.lookup_robot_pose()
            if pose is None:
                self.stop_robot()
                rate.sleep()
                continue

            _, _, yaw = pose
            err = normalize_angle(target_yaw - yaw)

            if abs(err) < self.yaw_tolerance:
                stable_count += 1
                self.stop_robot()
                if stable_count >= 3:
                    rospy.loginfo("pid_align_yaw done: err=%.4f", err)
                    return True
                rate.sleep()
                continue

            stable_count = 0
            now = rospy.Time.now()
            wz = pd_yaw.update(err, now)
            wz = self.clamp(wz, -self.pid_max_wz, self.pid_max_wz)

            cmd = Twist()
            cmd.angular.z = wz
            self.publish_cmd(cmd)
            rate.sleep()

        return False

    # =========================================================
    # PID 平移 + 航向锁
    # =========================================================
    def pid_translate(self, tx, ty, target_yaw, timeout=None):
        if timeout is None:
            timeout = self.pid_translate_timeout

        rospy.loginfo("pid_translate: to (%.3f, %.3f) yaw=%.3f", tx, ty, target_yaw)
        rate = rospy.Rate(20)
        start_time = rospy.Time.now()
        pd_x = PDController(self.pid_kp_xy, self.pid_kd_xy)
        pd_y = PDController(self.pid_kp_xy, self.pid_kd_xy)
        pd_yaw = PDController(self.pid_kp_yaw, self.pid_kd_yaw)
        stable_count = 0

        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed > timeout:
                rospy.logwarn("pid_translate timeout.")
                self.stop_robot()
                return False

            pose = self.lookup_robot_pose()
            if pose is None:
                self.stop_robot()
                rate.sleep()
                continue

            rx, ry, ryaw = pose
            ex = tx - rx
            ey = ty - ry
            dist = math.sqrt(ex * ex + ey * ey)
            yaw_err = normalize_angle(target_yaw - ryaw)

            if dist < self.pos_tolerance:
                stable_count += 1
                self.stop_robot()
                if stable_count >= 3:
                    rospy.loginfo("pid_translate done: dist=%.4f", dist)
                    return True
                rate.sleep()
                continue

            stable_count = 0
            now = rospy.Time.now()

            vx_map = pd_x.update(ex, now)
            vy_map = pd_y.update(ey, now)
            wz = pd_yaw.update(yaw_err, now)

            vx_map = self.clamp(vx_map, -self.pid_max_v, self.pid_max_v)
            vy_map = self.clamp(vy_map, -self.pid_max_v, self.pid_max_v)
            wz = self.clamp(wz, -self.pid_max_wz, self.pid_max_wz)

            cmd = self.map_velocity_to_base_cmd(vx_map, vy_map, ryaw)
            cmd.angular.z = wz
            cmd = self.apply_laser_safety(cmd)
            self.publish_cmd(cmd)
            rate.sleep()

        return False

    # =========================================================
    # PID 到目标点 (航向对齐 + 平移)
    # =========================================================
    def pid_goto_point(self, tx, ty, target_yaw, label="point"):
        rospy.loginfo("=== pid_goto_point [%s]: (%.3f, %.3f) yaw=%.3f ===",
                       label, tx, ty, target_yaw)

        # 子阶段A: 航向对齐
        rospy.loginfo("[%s] Sub-phase A: align yaw", label)
        phase_name = "PID_%s_ALIGN_YAW" % label.upper()
        phase_start = self.phase_start(
            phase_name,
            "target_yaw=%.3f timeout=%.1fs" %
            (target_yaw, self.pid_yaw_align_timeout)
        )
        align_ok = self.pid_align_yaw(target_yaw)
        self.phase_end(
            phase_name,
            phase_start,
            "OK" if align_ok else "TIMEOUT_ACCEPT"
        )

        # 子阶段B: 平移 + 航向锁
        rospy.loginfo("[%s] Sub-phase B: translate", label)
        phase_name = "PID_%s_TRANSLATE" % label.upper()
        phase_start = self.phase_start(
            phase_name,
            "target=(%.3f, %.3f) timeout=%.1fs pos_tol=%.3f" %
            (tx, ty, self.pid_translate_timeout, self.pos_tolerance)
        )
        ok = self.pid_translate(tx, ty, target_yaw)
        self.phase_end(
            phase_name,
            phase_start,
            "OK" if ok else "FAIL"
        )

        self.stop_robot()
        return ok

    # =========================================================
    # Phase 3: 激光挡板精调
    # =========================================================
    def get_laser_at_angle(self, angle_deg):
        """读取激光 scan 中指定角度(度)的单点距离，读到 inf/NaN 返回 None"""
        if self.latest_scan is None:
            return None
        msg = self.latest_scan
        if msg.angle_increment == 0.0:
            return None
        ang = math.radians(angle_deg)
        idx = int(round((ang - msg.angle_min) / msg.angle_increment))
        if idx < 0 or idx >= len(msg.ranges):
            return None
        r = msg.ranges[idx]
        if math.isnan(r) or math.isinf(r):
            return None
        return r

    def precision_fine_tune(self):
        """Phase 3: 只用入口对面那个真实挡板做单方向激光精调"""
        rospy.loginfo("=== Phase 3: laser precision fine-tune ===")

        if self.best_entry is None:
            rospy.loginfo("Phase 3: no entry info, skip.")
            return "SKIP", "reason=no_entry"

        # 入口对面 = 真挡板方向
        opposite = {
            "right": "left",
            "left": "right",
            "up": "down",
            "down": "up",
        }
        entry_name = self.best_entry["name"]
        baffle_side = opposite[entry_name]

        # box 系下该挡板的方向角
        box_angles = {"right": 0.0, "up": math.pi / 2, "left": math.pi, "down": -math.pi / 2}
        box_ang = box_angles[baffle_side]

        # 映射到 base 系 (机器人局部坐标系)
        base_ang = normalize_angle(box_ang - self.target_yaw)

        # 归类到 front/back/left/right + 确定激光角度和目标距离
        if abs(base_ang) < math.pi / 4:
            laser_ang = 0
            target_dist = self.fine_tune_front_back_target
            axis = "x"
            sign = 1  # front: error>0 → 往前 (x+)
        elif abs(base_ang) > 3 * math.pi / 4:
            laser_ang = 180
            target_dist = self.fine_tune_front_back_target
            axis = "x"
            sign = -1  # back: error>0 → 后退 (x-)
        elif base_ang > 0:
            laser_ang = 90
            target_dist = self.fine_tune_side_target
            axis = "y"
            sign = 1  # left: error>0 → 左移 (y+)
        else:
            laser_ang = -90
            target_dist = self.fine_tune_side_target
            axis = "y"
            sign = -1  # right: error>0 → 右移 (y-)

        rospy.loginfo("Phase 3: entry=%s baffle=%s box_ang=%.2f base_ang=%.2f laser=%d axis=%s sign=%d target=%.2f",
                       entry_name, baffle_side, box_ang, base_ang, laser_ang, axis, sign, target_dist)

        pd = PDController(self.fine_tune_kp, self.fine_tune_kd)
        rate = rospy.Rate(20)
        start_time = rospy.Time.now()
        stable_cnt = 0
        status = "TIMEOUT"
        detail = ""

        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed > self.fine_tune_timeout:
                rospy.logwarn("Phase 3: timeout %.1fs", elapsed)
                detail = "elapsed=%.2f" % elapsed
                break

            now = rospy.Time.now()
            d = self.get_laser_at_angle(laser_ang)

            if d is not None and d < 1.5:
                err = d - target_dist
                if abs(err) < self.fine_tune_tolerance:
                    stable_cnt += 1
                    if stable_cnt >= 3:
                        rospy.loginfo("Phase 3: done. laser=%.3f target=%.2f err=%.3f",
                                       d, target_dist, err)
                        status = "OK"
                        detail = "laser=%.3f target=%.2f err=%.3f" % (d, target_dist, err)
                        break
                else:
                    stable_cnt = 0
                    v = self.clamp(sign * pd.update(err, now), -self.fine_tune_max_v, self.fine_tune_max_v)

                    cmd = Twist()
                    if axis == "x":
                        cmd.linear.x = v
                    else:
                        cmd.linear.y = v
                    cmd.angular.z = 0.0
                    cmd = self.apply_laser_safety(cmd)
                    self.publish_cmd(cmd)
            else:
                stable_cnt = 0

            rate.sleep()

        self.stop_robot()
        rospy.loginfo("Phase 3: finished.")
        return status, detail

    # =========================================================
    # Phase 4: 逃逸 (泊车后沿入口轴反向退出)
    # =========================================================
    def escape(self):
        """泊车后沿入口轴反向退出，离开挡板区域"""
        if not self.escape_enabled or self.best_entry is None:
            self.phase_skip(
                "PHASE4_ESCAPE",
                "escape_enabled=%s best_entry=%s" %
                (str(self.escape_enabled), str(self.best_entry is not None))
            )
            return

        entry = self.best_entry
        phase_start = self.phase_start(
            "PHASE4_ESCAPE",
            "entry=%s distance=%.3f speed=%.3f timeout=%.1fs" %
            (entry["name"], self.escape_distance, self.escape_speed, self.escape_timeout)
        )
        rospy.loginfo("=== Phase 4: escape via entry %s ===", entry["name"])

        # 逃逸方向 = 入口轴反方向
        escape_axis_x = -entry["axis_x"]
        escape_axis_y = -entry["axis_y"]

        start_pose = self.lookup_robot_pose()
        if start_pose is None:
            rospy.logwarn("escape: no start pose, skip.")
            self.phase_end("PHASE4_ESCAPE", phase_start, "SKIP", "reason=no_start_pose")
            return
        sx, sy, _ = start_pose

        rate = rospy.Rate(20)
        start_time = rospy.Time.now()
        escape_status = "TIMEOUT"
        escape_detail = ""

        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed > self.escape_timeout:
                rospy.logwarn("escape: timeout %.1fs", elapsed)
                escape_detail = "elapsed=%.2f" % elapsed
                break

            pose = self.lookup_robot_pose()
            if pose is None:
                self.stop_robot()
                rate.sleep()
                continue

            rx, ry, ryaw = pose
            moved = math.sqrt((rx - sx) ** 2 + (ry - sy) ** 2)

            if moved >= self.escape_distance:
                rospy.loginfo("escape: done, moved=%.3f", moved)
                escape_status = "OK"
                escape_detail = "moved=%.3f" % moved
                break

            vx_map = escape_axis_x * self.escape_speed
            vy_map = escape_axis_y * self.escape_speed

            cmd = self.map_velocity_to_base_cmd(vx_map, vy_map, ryaw)
            cmd.angular.z = 0.0
            cmd = self.apply_laser_safety(cmd)
            self.publish_cmd(cmd)
            rate.sleep()

        self.stop_robot()
        rospy.loginfo("Phase 4: escape finished.")
        self.phase_end("PHASE4_ESCAPE", phase_start, escape_status, escape_detail)

    # =========================================================
    # 4个入口生成 (复用现有逻辑)
    # =========================================================
    def generate_entries(self):
        tx = self.target_x
        ty = self.target_y
        d = self.entry_offset

        raw_entries = [
            {"name": "right", "entry_x": tx + d, "entry_y": ty,
             "axis_x": -1.0, "axis_y": 0.0, "target_yaw": self.target_yaw},
            {"name": "left", "entry_x": tx - d, "entry_y": ty,
             "axis_x": 1.0, "axis_y": 0.0, "target_yaw": self.target_yaw},
            {"name": "up", "entry_x": tx, "entry_y": ty + d,
             "axis_x": 0.0, "axis_y": -1.0, "target_yaw": self.target_yaw},
            {"name": "down", "entry_x": tx, "entry_y": ty - d,
             "axis_x": 0.0, "axis_y": 1.0, "target_yaw": self.target_yaw},
        ]

        entries = []
        for e in raw_entries:
            e["entry_yaw"] = math.atan2(ty - e["entry_y"], tx - e["entry_x"])
            entries.append(e)
        return entries

    def sort_entries_by_robot_position(self, entries):
        pose = self.lookup_robot_pose()
        if pose is None:
            return entries
        rx, ry, _ = pose
        return sorted(entries, key=lambda e: math.sqrt((e["entry_x"] - rx) ** 2 + (e["entry_y"] - ry) ** 2))

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

    # =========================================================
    # 目标点圆环开口检测
    # =========================================================
    def detect_opening_by_circle(self, points):
        counts = {"left": 0, "right": 0, "up": 0, "down": 0}
        tx, ty = self.target_x, self.target_y
        r_min = self.opening_detect_radius - self.opening_ring_width * 0.5
        r_max = self.opening_detect_radius + self.opening_ring_width * 0.5
        ring_count = 0

        for mx, my in points:
            dx, dy = mx - tx, my - ty
            r = math.sqrt(dx * dx + dy * dy)
            if r < r_min or r > r_max:
                continue
            ring_count += 1
            deg = math.degrees(math.atan2(dy, dx))
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

        rospy.logwarn("Opening circle: confident=%s ring=%d left=%d right=%d up=%d down=%d best=%s",
                       str(confident), ring_count, counts["left"], counts["right"],
                       counts["up"], counts["down"], ",".join(best_names))
        return {"counts": counts, "ring_count": ring_count, "best_names": best_names, "confident": confident}

    # =========================================================
    # 入口识别评分
    # =========================================================
    def sort_entries_by_obstacle_score(self, entries):
        points = self.collect_scan_points_in_map()
        sides = self.evaluate_target_sides(points)
        opening = {"counts": {"left": 0, "right": 0, "up": 0, "down": 0},
                    "ring_count": 0, "best_names": [], "confident": False}
        if self.enable_opening_circle_detect:
            opening = self.detect_opening_by_circle(points)

        pose = self.lookup_robot_pose()
        if pose is None:
            return self.sort_entries_by_robot_position(entries)
        rx, ry, _ = pose

        blocked_names = []
        open_names = []
        for k in ["left", "right", "up", "down"]:
            if sides[k]["blocked"]:
                blocked_names.append(k)
            else:
                open_names.append(k)

        rospy.logwarn("Open sides: %s, blocked: %s", ",".join(open_names), ",".join(blocked_names))

        for e in entries:
            dist = math.sqrt((e["entry_x"] - rx) ** 2 + (e["entry_y"] - ry) ** 2)
            side_name = e["name"]
            side_blocked = sides.get(side_name, {"blocked": False})["blocked"]
            side_count = sides.get(side_name, {"count": 0})["count"]
            old_corridor_count = self.count_corridor_points(points, e)
            path_count = self.count_points_in_path_corridor(
                points, e["entry_x"], e["entry_y"], self.target_x, self.target_y, self.path_corridor_width)
            opening_count = opening["counts"].get(side_name, 0)
            is_best_opening = side_name in opening["best_names"]

            score = dist * 1.0
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
            e["side_blocked"] = side_blocked
            e["opening_count"] = opening_count
            e["is_best_opening"] = is_best_opening
            e["opening_confident"] = opening["confident"]

        return sorted(entries, key=lambda x: x.get("score", 999.0))

    def collect_scan_points_in_map(self):
        now = rospy.Time.now()
        points = []
        if self.latest_scan is None:
            return points
        scan = self.latest_scan
        try:
            self.tf_listener.waitForTransform(self.map_frame, scan.header.frame_id,
                                              rospy.Time(0), rospy.Duration(0.3))
            trans, rot = self.tf_listener.lookupTransform(self.map_frame, scan.header.frame_id, rospy.Time(0))
            _, _, yaw = tf.transformations.euler_from_quaternion(rot)
            cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
            for i, r in enumerate(scan.ranges):
                if math.isnan(r) or math.isinf(r):
                    continue
                if r <= scan.range_min or r >= scan.range_max:
                    continue
                a = scan.angle_min + i * scan.angle_increment
                lx, ly = r * math.cos(a), r * math.sin(a)
                mx = trans[0] + cos_yaw * lx - sin_yaw * ly
                my = trans[1] + sin_yaw * lx + cos_yaw * ly
                if math.sqrt((mx - self.target_x) ** 2 + (my - self.target_y) ** 2) <= self.recognition_max_range:
                    points.append((mx, my))
            self.scan_memory.append((now, points))
        except Exception as e:
            rospy.logwarn_throttle(1.0, "collect_scan_points_in_map failed: %s", str(e))

        new_memory, combined = [], []
        for t, ps in self.scan_memory:
            if (now - t).to_sec() <= self.scan_memory_time:
                new_memory.append((t, ps))
                combined.extend(ps)
        self.scan_memory = new_memory
        return combined

    def evaluate_target_sides(self, points):
        tx, ty = self.target_x, self.target_y
        h, w = self.target_box_half_size, self.side_detect_width
        counts = {"left": 0, "right": 0, "up": 0, "down": 0}
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
            sides[k] = {"count": counts[k], "blocked": counts[k] >= self.side_detect_min_points}
        return sides

    def count_corridor_points(self, points, entry):
        tx, ty = self.target_x, self.target_y
        ex, ey = entry["entry_x"], entry["entry_y"]
        half_w = self.corridor_width * 0.5
        count = 0
        if entry["name"] in ["left", "right"]:
            xmin, xmax = min(ex, tx), max(ex, tx)
            ymin, ymax = ty - half_w, ty + half_w
            for mx, my in points:
                if xmin <= mx <= xmax and ymin <= my <= ymax:
                    count += 1
        elif entry["name"] in ["up", "down"]:
            ymin, ymax = min(ey, ty), max(ey, ty)
            xmin, xmax = tx - half_w, tx + half_w
            for mx, my in points:
                if xmin <= mx <= xmax and ymin <= my <= ymax:
                    count += 1
        return count

    # =========================================================
    # 工具函数
    # =========================================================
    def map_velocity_to_base_cmd(self, vx_map, vy_map, yaw):
        cmd = Twist()
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        cmd.linear.x = cos_yaw * vx_map + sin_yaw * vy_map
        cmd.linear.y = -sin_yaw * vx_map + cos_yaw * vy_map
        cmd.angular.z = 0.0
        return cmd

    def lookup_robot_pose(self):
        try:
            self.tf_listener.waitForTransform(self.map_frame, self.base_frame,
                                              rospy.Time(0), rospy.Duration(0.5))
            trans, rot = self.tf_listener.lookupTransform(self.map_frame, self.base_frame, rospy.Time(0))
            _, _, yaw = tf.transformations.euler_from_quaternion(rot)
            return trans[0], trans[1], yaw
        except Exception as e:
            rospy.logwarn_throttle(1.0, "TF lookup failed: %s", str(e))
            return None

    def distance_to_point(self, x, y):
        pose = self.lookup_robot_pose()
        if pose is None:
            return None
        return math.sqrt((x - pose[0]) ** 2 + (y - pose[1]) ** 2)

    def get_sector_min_range(self, deg_min, deg_max):
        if self.latest_scan is None:
            return float("inf")
        msg = self.latest_scan
        if msg.angle_increment == 0.0:
            return float("inf")
        a0, a1 = math.radians(deg_min), math.radians(deg_max)
        if a0 > a1:
            a0, a1 = a1, a0
        rmin = float("inf")
        for i, r in enumerate(msg.ranges):
            if math.isnan(r) or math.isinf(r):
                continue
            a = msg.angle_min + i * msg.angle_increment
            if a0 <= a <= a1 and r < rmin:
                rmin = r
        return rmin

    def smooth_cmd(self, cmd):
        if not self.enable_cmd_smoothing:
            return cmd
        now = rospy.Time.now()
        dt = (now - self.last_cmd_time).to_sec()
        if dt <= 0.0 or dt > 0.5:
            dt = 0.05
        out = Twist()
        out.linear.x = self.limit_delta(cmd.linear.x, self.last_cmd.linear.x, self.max_acc_x * dt)
        out.linear.y = self.limit_delta(cmd.linear.y, self.last_cmd.linear.y, self.max_acc_y * dt)
        out.angular.z = self.limit_delta(cmd.angular.z, self.last_cmd.angular.z, self.max_acc_wz * dt)
        self.last_cmd = out
        self.last_cmd_time = now
        return out

    def publish_cmd(self, cmd):
        cmd = self.smooth_cmd(cmd)
        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        cmd = Twist()
        self.last_cmd = Twist()
        self.last_cmd_time = rospy.Time.now()
        for _ in range(8):
            self.cmd_pub.publish(cmd)
            rospy.sleep(0.03)

    # =========================================================
    # 激光安全 (含后方保护)
    # =========================================================
    def apply_laser_safety(self, cmd):
        front = self.get_sector_min_range(-15, 15)
        left_front = self.get_sector_min_range(15, 55)
        right_front = self.get_sector_min_range(-55, -15)
        left_side = self.get_sector_min_range(55, 125)
        right_side = self.get_sector_min_range(-125, -55)
        rear = min(self.get_sector_min_range(165, 180), self.get_sector_min_range(-180, -165))
        rear_left = self.get_sector_min_range(125, 165)
        rear_right = self.get_sector_min_range(-165, -125)

        front_danger = min(front, left_front, right_front)
        left_danger = min(left_front, left_side)
        right_danger = min(right_front, right_side)
        rear_danger = min(rear, rear_left, rear_right)
        nearest = min(front_danger, left_side, right_side, rear_danger)

        self.last_block_front = False
        self.last_block_left = False
        self.last_block_right = False
        self.last_block_any = False

        if nearest < self.any_stop_dist:
            self.last_block_any = True
            rospy.logwarn_throttle(0.5, "too close nearest=%.3f < %.3f", nearest, self.any_stop_dist)
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            return cmd

        if cmd.linear.x > 0.0:
            if front_danger < self.front_stop_dist:
                self.last_block_front = True
                rospy.logwarn_throttle(0.5, "front blocked %.3f < %.3f", front_danger, self.front_stop_dist)
                cmd.linear.x = 0.0
            elif front_danger < self.front_slow_dist:
                cmd.linear.x = min(cmd.linear.x, self.front_slow_v)

        if cmd.linear.x < 0.0:
            if rear_danger < self.front_stop_dist:
                self.last_block_front = True
                rospy.logwarn_throttle(0.5, "rear blocked %.3f < %.3f", rear_danger, self.front_stop_dist)
                cmd.linear.x = 0.0

        if cmd.linear.y > 0.0:
            if left_danger < self.side_stop_dist:
                self.last_block_left = True
                cmd.linear.y = 0.0
        if cmd.linear.y < 0.0:
            if right_danger < self.side_stop_dist:
                self.last_block_right = True
                cmd.linear.y = 0.0

        if front_danger < self.front_stop_dist and abs(cmd.linear.x) > 0.0:
            self.last_block_front = True
            cmd.linear.x = 0.0

        if abs(cmd.linear.x) < self.min_v:
            cmd.linear.x = 0.0
        if abs(cmd.linear.y) < self.min_v:
            cmd.linear.y = 0.0

        return cmd

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
