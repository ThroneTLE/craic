#!/usr/bin/env python2
# -*- coding: utf-8 -*-


# =================== 导入依赖库/ROS消息 ===================
import rospy
import actionlib
import numpy as np
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import pi
from std_msgs.msg import String, Int32
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan, Imu
from rosgraph_msgs.msg import Log
import sys, os, time
import dynamic_reconfigure.client
from std_srvs.srv import Trigger, TriggerRequest
# 自定义TTS语音播报服务接口
from TTS_audio.srv import StringService, StringServiceRequest
# 精密停车模块
from auto_parking_pd import AutoSinglePointTest

# =================== 全局变量定义 ===================
# VLM 视觉检测结果 → 任务点索引映射
# 31,32,33 → 1,2,3  |  40,41,42 → 4,5,6  |  49,50,51 → 7,8,9
VLM_TO_TASK = {
    31: 1, 32: 2, 33: 3,
    40: 4, 41: 5, 42: 6,
    49: 7, 50: 8, 51: 9,
}
# 反向映射：映射后索引 → 原始VLM识别编号（用于语音播报）
TASK_TO_VLM = {v: k for k, v in VLM_TO_TASK.items()}
time_val = 1        # 机器人终点动作计时变量
clue = 1            # 线索计数(第1条、第2条线索...)
# 预设的【检测点】索引列表(对应launch文件中的导航点位)
points=[10, 11, 12, 13]
# 存储视觉识别到的【任务编号】(1-9)
task_numbers = []
# 点位语音文件映射(未使用，代码中用的是文本播报)
point_audio = {
    12: "/home/abot/EIU0US/src/robot_slam/mp3/01.mp3",
    13: "/home/abot/EIU0US/src/robot_slam/mp3/02.mp3",
    14: "/home/abot/EIU0US/src/robot_slam/mp3/03.mp3",
    15: "/home/abot/EIU0US/src/robot_slam/mp3/04.mp3"
}

# =================== 核心导航类定义 ===================
class navigation_demo:
    # 构造函数：初始化节点、发布者、订阅者、服务客户端
    def __init__(self):
        # 1. 发布者：设置机器人初始位姿(地图坐标系)
        self.set_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=5)
        # 2. 发布者：播报到达消息(未使用)
        self.arrive_pub = rospy.Publisher('/voiceWords', String, queue_size=10)
        # 3. 导航动作客户端：连接move_base(ROS官方导航模块)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        # 等待导航服务启动(超时60秒)
        self.move_base.wait_for_server(rospy.Duration(60))

        # 4. 发布者：控制机器人底盘速度(前进/旋转/平移)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1000)

        # 5. 连接【视觉大模型检测服务】
        rospy.loginfo("等待视觉大模型检测服务 /fruit_detection 可用...")
        rospy.wait_for_service('/fruit_detection', timeout=20)
        self.fruit_detection_service = rospy.ServiceProxy('/fruit_detection', Trigger)
        rospy.loginfo("视觉大模型检测服务连接成功！")

        # 6. 连接【TTS语音播报服务】
        rospy.loginfo("等待TTS服务 /tts_service 可用...")
        rospy.wait_for_service('tts_service', timeout=20)
        self.tts_service = rospy.ServiceProxy('tts_service', StringService)
        rospy.loginfo("TTS服务连接成功！")

        # 7. 订阅激光雷达数据（供 adjust_position 使用）
        self.scan_data = None
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        # 8. 订阅里程计数据（获取当前航向角）
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # 9. PID校准参数
        self.kp_linear = 0.5        # 线速度比例系数
        self.kp_angular = 0.5       # 角速度比例系数
        self.position_tolerance = 0.02   # 位置容差 (米)
        self.yaw_tolerance = 0.05        # 航向角容差 (弧度, ~3°)
        self.target_yaw = 0.0            # 目标航向角
        self.current_yaw = 0.0           # 当前航向角
        self.odom_received = False
        self.is_adjusting = False

        # 10. 检测点拍照前预对准参数
        self.detect_prealign_enabled = rospy.get_param("~detect_prealign_enabled", True)
        self.detect_prealign_mode = rospy.get_param("~detect_prealign_mode", "back")
        self.detect_prealign_distance = rospy.get_param("~detect_prealign_distance", 0.35)
        self.detect_prealign_timeout = rospy.get_param("~detect_prealign_timeout", 25)
        self.detect_yaw_align_at_prealign = rospy.get_param("~detect_yaw_align_at_prealign", True)
        self.detect_final_timeout = rospy.get_param("~detect_final_timeout", 35)
        self.detect_locked_final_approach = rospy.get_param("~detect_locked_final_approach", True)
        self.detect_locked_approach_speed = rospy.get_param("~detect_locked_approach_speed", 0.15)
        self.detect_locked_approach_yaw_hold = rospy.get_param("~detect_locked_approach_yaw_hold", True)
        self.detect_locked_approach_yaw_kp = rospy.get_param("~detect_locked_approach_yaw_kp", 0.8)
        self.detect_locked_approach_max_yaw_vel = rospy.get_param("~detect_locked_approach_max_yaw_vel", 0.20)
        self.detect_locked_approach_timeout_margin = rospy.get_param("~detect_locked_approach_timeout_margin", 1.0)
        self.detect_yaw_align_at_photo = rospy.get_param("~detect_yaw_align_at_photo", False)
        self.detect_yaw_align_enabled = rospy.get_param("~detect_yaw_align_enabled", True)
        self.detect_yaw_tolerance = rospy.get_param("~detect_yaw_tolerance", 0.06)
        self.detect_yaw_align_timeout = rospy.get_param("~detect_yaw_align_timeout", 3.0)
        self.detect_yaw_kp = rospy.get_param("~detect_yaw_kp", 1.2)
        self.detect_yaw_min_vel = rospy.get_param("~detect_yaw_min_vel", 0.08)
        self.detect_yaw_max_vel = rospy.get_param("~detect_yaw_max_vel", 0.45)
        self.detect_yaw_stable_count = int(rospy.get_param("~detect_yaw_stable_count", 4))
        self.detect_photo_settle_time = rospy.get_param("~detect_photo_settle_time", 0.25)
        self.detect_capture_wait = rospy.get_param("~detect_capture_wait", 0.5)

        # 11. 调试/比赛固定任务点：跳过前置视觉扫描，直接进入任务点泊车
        self.use_fixed_task_positions = rospy.get_param("~use_fixed_task_positions", False)
        self.fixed_task_ids = rospy.get_param("~fixed_task_ids", "")
        self.final_nav_timeout = rospy.get_param("~final_nav_timeout", 10.0)
        self.final_yaw_align_timeout = rospy.get_param("~final_yaw_align_timeout", 3.0)
        self.final_yaw_tolerance = rospy.get_param("~final_yaw_tolerance", 0.05)
        self.final_yaw_kp = rospy.get_param("~final_yaw_kp", 1.2)
        self.final_yaw_min_vel = rospy.get_param("~final_yaw_min_vel", 0.08)
        self.final_yaw_max_vel = rospy.get_param("~final_yaw_max_vel", 0.45)
        self.final_yaw_stable_count = int(rospy.get_param("~final_yaw_stable_count", 3))
        self.task_nav_timeout = rospy.get_param("~task_nav_timeout", 8.0)
        self.task_nav_retry_timeout = rospy.get_param("~task_nav_retry_timeout", 5.0)
        self.task_nav_accept_dist = rospy.get_param("~task_nav_accept_dist", 0.45)
        self.task_nav_use_approach_goal = rospy.get_param("~task_nav_use_approach_goal", True)
        self.task_nav_approach_offset = rospy.get_param("~task_nav_approach_offset", 0.30)
        self.task_nav_approach_modes = rospy.get_param(
            "~task_nav_approach_modes",
            "back,back_left,left,front_left,front,front_right,right,back_right")
        self.task_nav_approach_filter_costmap = rospy.get_param("~task_nav_approach_filter_costmap", True)
        self.task_nav_approach_costmap_topic = rospy.get_param(
            "~task_nav_approach_costmap_topic", "/move_base/global_costmap/costmap")
        self.task_nav_approach_cost_threshold = int(rospy.get_param("~task_nav_approach_cost_threshold", 98))
        self.task_nav_approach_reject_unknown = rospy.get_param("~task_nav_approach_reject_unknown", True)
        self.task_nav_approach_costmap_wait = rospy.get_param("~task_nav_approach_costmap_wait", 0.5)
        self.task_nav_approach_fallback_to_target = rospy.get_param("~task_nav_approach_fallback_to_target", False)
        self.task_nav_target_accept_yaw = rospy.get_param("~task_nav_target_accept_yaw", 0.5)
        self.task_nav_approach_score_radius = rospy.get_param("~task_nav_approach_score_radius", 0.20)
        self.task_nav_no_progress_enabled = rospy.get_param("~task_nav_no_progress_enabled", True)
        self.task_nav_no_progress_timeout = rospy.get_param("~task_nav_no_progress_timeout", 3.0)
        self.task_nav_no_progress_min_delta = rospy.get_param("~task_nav_no_progress_min_delta", 0.05)
        self.task_nav_plan_fail_cancel_enabled = rospy.get_param("~task_nav_plan_fail_cancel_enabled", True)
        self.task_nav_plan_fail_count = int(rospy.get_param("~task_nav_plan_fail_count", 5))
        self.task_nav_plan_fail_window = rospy.get_param("~task_nav_plan_fail_window", 1.0)
        self.force_escape_after_approach_nav = rospy.get_param("~force_escape_after_approach_nav", True)
        self.parking_phase_global_inflation_enabled = rospy.get_param(
            "~parking_phase_global_inflation_enabled", True)
        self.parking_phase_global_inflation_radius = rospy.get_param(
            "~parking_phase_global_inflation_radius", 0.06)
        self.cruise_global_inflation_radius = rospy.get_param(
            "~cruise_global_inflation_radius", 0.15)
        self.global_inflation_layer_name = rospy.get_param(
            "~global_inflation_layer_name", "move_base/global_costmap/inflation_layer")
        self.global_inflation_client = None
        self.start_escape_turn_enabled = rospy.get_param("~start_escape_turn_enabled", True)
        self.start_escape_turn_speed = rospy.get_param("~start_escape_turn_speed", 0.18)
        self.start_escape_turn_duration = rospy.get_param("~start_escape_turn_duration", 1.0)
        self.global_costmap = None
        rospy.Subscriber(self.task_nav_approach_costmap_topic, OccupancyGrid, self.global_costmap_callback)
        self.task_nav_goal_active = False
        self.task_nav_plan_fail_cancel_requested = False
        self.task_nav_plan_fail_seen = 0
        self.task_nav_plan_fail_window_start = rospy.Time(0)
        self.task_nav_plan_fail_label = ""
        self.task_nav_plan_fail_mode = ""
        rospy.Subscriber("/rosout", Log, self.rosout_callback)
        self.last_move_base_state = None
        self.last_move_base_feedback = None
    
    def scan_callback(self, msg):
        """存储最新的激光雷达数据"""
        self.scan_data = msg

    def global_costmap_callback(self, msg):
        self.global_costmap = msg

    def rosout_callback(self, msg):
        if (not self.task_nav_plan_fail_cancel_enabled
                or not self.task_nav_goal_active):
            return
        if msg.name != "/move_base":
            return
        if "Failed to get a plan" not in msg.msg:
            return

        now = rospy.Time.now()
        if (self.task_nav_plan_fail_window_start == rospy.Time(0)
                or (now - self.task_nav_plan_fail_window_start).to_sec() > self.task_nav_plan_fail_window):
            self.task_nav_plan_fail_window_start = now
            self.task_nav_plan_fail_seen = 0

        self.task_nav_plan_fail_seen += 1
        if self.task_nav_plan_fail_seen >= self.task_nav_plan_fail_count:
            self.task_nav_plan_fail_cancel_requested = True
            rospy.logwarn(
                "[TASK_NAV][PLAN_FAIL_CANCEL_REQUEST] label=%s mode=%s count=%d window=%.2fs msg=%s",
                self.task_nav_plan_fail_label,
                self.task_nav_plan_fail_mode,
                self.task_nav_plan_fail_seen,
                self.task_nav_plan_fail_window,
                msg.msg
            )

    def get_global_inflation_client(self):
        if self.global_inflation_client is not None:
            return self.global_inflation_client
        try:
            self.global_inflation_client = dynamic_reconfigure.client.Client(
                self.global_inflation_layer_name,
                timeout=2.0
            )
            return self.global_inflation_client
        except Exception as e:
            rospy.logwarn(
                "[COSTMAP_PHASE][CLIENT_FAILED] name=%s error=%s",
                self.global_inflation_layer_name,
                str(e)
            )
            return None

    def set_global_inflation_radius(self, radius, reason):
        if not self.parking_phase_global_inflation_enabled:
            rospy.loginfo("[COSTMAP_PHASE][SKIP] reason=%s enabled=false", reason)
            return False
        client = self.get_global_inflation_client()
        if client is None:
            return False
        try:
            client.update_configuration({"inflation_radius": float(radius)})
            self.global_costmap = None
            rospy.loginfo(
                "[COSTMAP_PHASE][SET] reason=%s global_inflation_radius=%.3f",
                reason,
                radius
            )
            return True
        except Exception as e:
            rospy.logwarn(
                "[COSTMAP_PHASE][SET_FAILED] reason=%s radius=%.3f error=%s",
                reason,
                radius,
                str(e)
            )
            return False

    def set_parking_phase_costmap(self):
        return self.set_global_inflation_radius(
            self.parking_phase_global_inflation_radius,
            "parking_phase"
        )

    def restore_cruise_costmap(self):
        return self.set_global_inflation_radius(
            self.cruise_global_inflation_radius,
            "return_to_final"
        )

    def odom_callback(self, msg):
        """从里程计提取当前航向角"""
        orientation_q = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.current_yaw = yaw
        self.odom_received = True

    def normalize_angle(self, angle):
        """将角度归一化到[-π, π]范围内"""
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle

    def log_nav_state(self, label, target=None):
        if target is None:
            rospy.loginfo("[NAV_STATE][%s] yaw=%.3f odom_received=%s",
                          label, self.current_yaw, str(self.odom_received))
        else:
            rospy.loginfo("[NAV_STATE][%s] target=(%.3f,%.3f,%.1f) yaw=%.3f odom_received=%s",
                          label, target[0], target[1], target[2],
                          self.current_yaw, str(self.odom_received))

    def distance_to_goal_xy(self, target):
        if self.last_move_base_feedback is not None:
            pose = self.last_move_base_feedback.base_position.pose
            dx = target[0] - pose.position.x
            dy = target[1] - pose.position.y
            return np.sqrt(dx * dx + dy * dy)
        rospy.logwarn("无move_base反馈位姿，无法计算map目标距离")
        return None

    def reset_nav_feedback(self):
        self.last_move_base_feedback = None
        self.last_move_base_state = None

    def nav_reached_by_state_and_distance(self, nav_ok, target):
        nav_dist = self.distance_to_goal_xy(target)
        if nav_dist is None:
            if nav_ok:
                rospy.logwarn("[TASK_TIME][NAV_NO_DISTANCE_ACCEPT_STATE] state_ok=true")
                return True, None
            return False, None
        nav_reached = (
            nav_ok and nav_dist is not None and nav_dist <= self.task_nav_accept_dist
        ) or (
            nav_dist is not None and nav_dist <= self.task_nav_accept_dist
        )
        if nav_ok and not nav_reached:
            rospy.logwarn(
                "[TASK_TIME][NAV_STATE_DISTANCE_MISMATCH] state_ok=true dist=%s accept=%.3f",
                "%.3f" % nav_dist if nav_dist is not None else "None",
                self.task_nav_accept_dist
            )
        return nav_reached, nav_dist

    def yaw_error_to_goal(self, target):
        target_yaw = target[2] / 180.0 * pi
        return self.normalize_angle(target_yaw - self.current_yaw)

    def make_task_approach_goals(self, target):
        """生成给 move_base 使用的多个墙外预到达点，泊车仍使用原目标点。"""
        if self.task_nav_approach_offset <= 0.0:
            return [("target", list(target))]

        yaw_rad = target[2] / 180.0 * pi
        forward_x = np.cos(yaw_rad)
        forward_y = np.sin(yaw_rad)
        left_x = -np.sin(yaw_rad)
        left_y = np.cos(yaw_rad)
        diag = 1.0 / np.sqrt(2.0)
        mode_vectors = {
            "back": (-forward_x, -forward_y),
            "front": (forward_x, forward_y),
            "left": (left_x, left_y),
            "right": (-left_x, -left_y),
            "back_left": ((-forward_x + left_x) * diag, (-forward_y + left_y) * diag),
            "back_right": ((-forward_x - left_x) * diag, (-forward_y - left_y) * diag),
            "front_left": ((forward_x + left_x) * diag, (forward_y + left_y) * diag),
            "front_right": ((forward_x - left_x) * diag, (forward_y - left_y) * diag),
            "left_back": ((-forward_x + left_x) * diag, (-forward_y + left_y) * diag),
            "right_back": ((-forward_x - left_x) * diag, (-forward_y - left_y) * diag),
            "left_front": ((forward_x + left_x) * diag, (forward_y + left_y) * diag),
            "right_front": ((forward_x - left_x) * diag, (forward_y - left_y) * diag),
        }

        goals_out = []
        seen = set()
        modes = [m.strip().lower() for m in str(self.task_nav_approach_modes).split(",") if m.strip()]
        for mode in modes:
            if mode not in mode_vectors:
                rospy.logwarn("[TASK_NAV][APPROACH_MODE_UNKNOWN] mode=%s", mode)
                continue
            vx, vy = mode_vectors[mode]
            approach = [
                target[0] + vx * self.task_nav_approach_offset,
                target[1] + vy * self.task_nav_approach_offset,
                target[2]
            ]
            key = (round(approach[0], 3), round(approach[1], 3), round(approach[2], 1))
            if key in seen:
                continue
            seen.add(key)
            goals_out.append((mode, approach))
            rospy.loginfo(
                "[TASK_NAV][APPROACH_CANDIDATE] mode=%s target=(%.3f,%.3f,%.1f) approach=(%.3f,%.3f,%.1f) offset=%.3f",
                mode, target[0], target[1], target[2],
                approach[0], approach[1], approach[2],
                self.task_nav_approach_offset
            )

        goals_out.append(("target", list(target)))
        return goals_out

    def get_global_costmap_for_approach(self):
        if self.global_costmap is not None:
            return self.global_costmap
        try:
            self.global_costmap = rospy.wait_for_message(
                self.task_nav_approach_costmap_topic,
                OccupancyGrid,
                timeout=self.task_nav_approach_costmap_wait)
        except Exception as e:
            rospy.logwarn(
                "[TASK_NAV][APPROACH_COSTMAP_WAIT_FAILED] topic=%s timeout=%.2f error=%s",
                self.task_nav_approach_costmap_topic,
                self.task_nav_approach_costmap_wait,
                str(e)
            )
        return self.global_costmap

    def costmap_cost_at(self, costmap, x, y):
        info = costmap.info
        if info.resolution <= 0.0 or info.width <= 0 or info.height <= 0:
            return None, "bad_costmap_info"
        mx = int((x - info.origin.position.x) / info.resolution)
        my = int((y - info.origin.position.y) / info.resolution)
        if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
            return None, "out_of_map"
        idx = my * info.width + mx
        if idx < 0 or idx >= len(costmap.data):
            return None, "bad_index"
        return int(costmap.data[idx]), "ok"

    def costmap_score_near(self, costmap, x, y, radius):
        info = costmap.info
        if info.resolution <= 0.0 or info.width <= 0 or info.height <= 0:
            return None, "bad_costmap_info"
        mx = int((x - info.origin.position.x) / info.resolution)
        my = int((y - info.origin.position.y) / info.resolution)
        if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
            return None, "out_of_map"

        radius_cells = int(np.ceil(max(0.0, radius) / info.resolution))
        max_cost = 0
        total_cost = 0
        count = 0
        unknown_count = 0
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy > radius_cells * radius_cells:
                    continue
                sx = mx + dx
                sy = my + dy
                if sx < 0 or sy < 0 or sx >= info.width or sy >= info.height:
                    unknown_count += 1
                    continue
                idx = sy * info.width + sx
                if idx < 0 or idx >= len(costmap.data):
                    unknown_count += 1
                    continue
                cost = int(costmap.data[idx])
                if cost < 0:
                    unknown_count += 1
                    cost = 100
                max_cost = max(max_cost, cost)
                total_cost += cost
                count += 1

        if count <= 0:
            return None, "no_score_cells"
        avg_cost = float(total_cost) / float(count)
        return (max_cost, avg_cost, unknown_count), "ok"

    def evaluate_task_approach_goal(self, mode, nav_target, costmap=None, costmap_checked=False):
        if not self.task_nav_approach_filter_costmap or mode == "target":
            return True, "filter_disabled_or_target", (0, 0.0, 0)

        if not costmap_checked:
            costmap = self.get_global_costmap_for_approach()
        if costmap is None:
            return True, "no_costmap_allow", (0, 0.0, 0)

        cost, detail = self.costmap_cost_at(costmap, nav_target[0], nav_target[1])
        if cost is None:
            return False, detail, (999, 999.0, 999)
        if cost < 0:
            if self.task_nav_approach_reject_unknown:
                return False, "unknown", (999, 999.0, 999)
            return True, "unknown_allowed", (100, 100.0, 1)
        if cost > self.task_nav_approach_cost_threshold:
            return False, "cost=%d>threshold=%d" % (
                cost, self.task_nav_approach_cost_threshold), (cost, float(cost), 0)

        score, detail = self.costmap_score_near(
            costmap, nav_target[0], nav_target[1], self.task_nav_approach_score_radius)
        if score is None:
            score = (cost, float(cost), 0)
            score_text = "score_unavailable=%s" % detail
        else:
            score_text = "score=max:%d avg:%.1f unk:%d radius:%.2f" % (
                score[0], score[1], score[2], self.task_nav_approach_score_radius)
        return True, "cost=%d %s" % (cost, score_text), score

    def is_task_approach_goal_clear(self, mode, nav_target):
        clear, reason, _ = self.evaluate_task_approach_goal(mode, nav_target)
        return clear, reason

    def select_task_approach_goal(self, target, skipped_modes=None):
        if skipped_modes is None:
            skipped_modes = set()
        candidates = self.make_task_approach_goals(target)
        if not self.task_nav_use_approach_goal and "target" not in skipped_modes:
            return "target", list(target)

        target_fallback = None
        clear_candidates = []
        costmap = None
        costmap_checked = False
        if self.task_nav_approach_filter_costmap:
            costmap = self.get_global_costmap_for_approach()
            costmap_checked = True
        for mode, nav_target in candidates:
            if mode == "target":
                if mode not in skipped_modes:
                    target_fallback = (mode, nav_target)
                continue
            if mode in skipped_modes:
                rospy.loginfo("[TASK_NAV][APPROACH_SKIP_FAILED] mode=%s", mode)
                continue
            clear, reason, score = self.evaluate_task_approach_goal(
                mode, nav_target, costmap=costmap, costmap_checked=costmap_checked)
            rospy.loginfo(
                "[TASK_NAV][APPROACH_CHECK] mode=%s nav_target=(%.3f,%.3f,%.1f) clear=%s reason=%s",
                mode, nav_target[0], nav_target[1], nav_target[2],
                str(clear), reason
            )
            if clear:
                clear_candidates.append((score, mode, nav_target, reason))

        if clear_candidates:
            clear_candidates.sort(key=lambda item: (item[0][2], item[0][0], item[0][1]))
            score, mode, nav_target, reason = clear_candidates[0]
            rospy.loginfo(
                "[TASK_NAV][APPROACH_SELECTED] mode=%s nav_target=(%.3f,%.3f,%.1f) score=max:%d avg:%.1f unk:%d reason=%s",
                mode, nav_target[0], nav_target[1], nav_target[2],
                score[0], score[1], score[2], reason
            )
            return mode, nav_target

        rospy.logwarn("[TASK_NAV][APPROACH_NO_CLEAR] target=%s", str(target))
        if self.task_nav_approach_fallback_to_target and target_fallback is not None:
            rospy.logwarn("[TASK_NAV][APPROACH_FALLBACK_TARGET] nav_target=%s", str(target_fallback[1]))
            return target_fallback
        return None, None

    def goto_task_approach(self, target, timeout, label, skipped_modes=None):
        mode, nav_target = self.select_task_approach_goal(target, skipped_modes)
        if nav_target is None:
            return False, False, None, None
        rospy.loginfo("[TASK_NAV][TRY_%s] mode=%s nav_target=%s", label, mode, nav_target)
        nav_ok = self.goto_task_nav_goal(nav_target, timeout=timeout, label=label, mode=mode)
        nav_reached, approach_dist = self.nav_reached_by_state_and_distance(nav_ok, nav_target)
        nav_dist = self.distance_to_goal_xy(target)
        if nav_reached:
            yaw_err = self.yaw_error_to_goal(target)
            if mode == "target" and not nav_ok and abs(yaw_err) > self.task_nav_target_accept_yaw:
                rospy.logwarn(
                    "[TASK_NAV][TARGET_DISTANCE_ACCEPT_REJECT_YAW] dist=%s yaw_err=%.3f accept_yaw=%.3f",
                    "%.3f" % approach_dist if approach_dist is not None else "None",
                    yaw_err,
                    self.task_nav_target_accept_yaw
                )
                nav_reached = False
            elif mode != "target" and abs(yaw_err) > self.task_nav_target_accept_yaw:
                rospy.logwarn(
                    "[TASK_NAV][APPROACH_REACHED_REJECT_YAW] mode=%s approach_dist=%s target_dist=%s yaw_err=%.3f accept_yaw=%.3f",
                    mode,
                    "%.3f" % approach_dist if approach_dist is not None else "None",
                    "%.3f" % nav_dist if nav_dist is not None else "None",
                    yaw_err,
                    self.task_nav_target_accept_yaw
                )
                nav_reached = False
        if mode != "target" and nav_ok and not nav_reached:
            rospy.logwarn(
                "[TASK_NAV][APPROACH_STATE_DISTANCE_MISMATCH] mode=%s approach_dist=%s accept=%.3f",
                mode,
                "%.3f" % approach_dist if approach_dist is not None else "None",
                self.task_nav_accept_dist
            )
        rospy.loginfo(
            "[TASK_NAV][TRY_%s_DONE] mode=%s ok=%s target_dist=%s approach_dist=%s reached=%s state=%s",
            label, mode, str(nav_ok),
            "%.3f" % nav_dist if nav_dist is not None else "None",
            "%.3f" % approach_dist if approach_dist is not None else "None",
            str(nav_reached), str(self.last_move_base_state)
        )
        return nav_ok, nav_reached, nav_dist, mode

    def mark_failed_approach_mode(self, idx, task_id, mode, failed_approach_modes):
        if mode is None:
            return
        failed_approach_modes.add(mode)
        rospy.logwarn(
            "[TASK_NAV][APPROACH_MARK_FAILED] idx=%d task_id=%d mode=%s failed_modes=%s",
            idx + 1, task_id, str(mode),
            ",".join(sorted(failed_approach_modes))
        )

    def navigate_task_with_all_approaches(self, idx, task_id, target, last_parking, last_task_id):
        failed_approach_modes = set()
        nav_ok = False
        nav_reached = False
        nav_dist = None
        nav_mode = None
        escaped_after_abort = False
        attempt = 0

        while not rospy.is_shutdown():
            attempt += 1
            label = "MAIN" if attempt == 1 else "RETRY_%d" % (attempt - 1)
            timeout = self.task_nav_timeout if attempt == 1 else self.task_nav_retry_timeout
            nav_start_time = rospy.Time.now()
            nav_ok, nav_reached, nav_dist, nav_mode = self.goto_task_approach(
                target, timeout, label, failed_approach_modes)
            rospy.loginfo(
                "[TASK_TIME][NAV_ATTEMPT] idx=%d task_id=%d label=%s dt=%.2fs ok=%s target_dist=%s reached=%s state=%s mode=%s",
                idx + 1, task_id, label,
                (rospy.Time.now() - nav_start_time).to_sec(),
                str(nav_ok),
                "%.3f" % nav_dist if nav_dist is not None else "None",
                str(nav_reached), str(self.last_move_base_state), str(nav_mode)
            )

            if nav_reached:
                break

            self.mark_failed_approach_mode(idx, task_id, nav_mode, failed_approach_modes)

            if (not escaped_after_abort and not nav_ok
                    and self.last_move_base_state == GoalStatus.ABORTED
                    and last_parking is not None):
                rospy.logwarn(
                    "[TASK_TIME][NAV_ABORTED_ESCAPE] idx=%d task_id=%d prev_task_id=%s state=%s",
                    idx + 1, task_id, str(last_task_id), str(self.last_move_base_state)
                )
                escape_retry_start = rospy.Time.now()
                force_escape = self.should_force_escape_after_approach(last_parking, True)
                if force_escape:
                    last_parking.escape(force=True, reason="next_nav_aborted")
                else:
                    last_parking.escape()
                rospy.loginfo(
                    "[TASK_TIME][NAV_ABORTED_ESCAPE_DONE] idx=%d task_id=%d dt=%.2fs forced=%s",
                    idx + 1, task_id,
                    (rospy.Time.now() - escape_retry_start).to_sec(),
                    str(force_escape)
                )
                escaped_after_abort = True

            if nav_mode is None:
                break

            rospy.logwarn(
                "[TASK_TIME][NAV_NOT_REACHED_RETRY] idx=%d task_id=%d target_dist=%s accept=%.3f next_failed_modes=%s",
                idx + 1, task_id,
                "%.3f" % nav_dist if nav_dist is not None else "None",
                self.task_nav_accept_dist,
                ",".join(sorted(failed_approach_modes))
            )

        return nav_ok, nav_reached, nav_dist, nav_mode

    def should_force_escape_after_approach(self, parking, approach_nav_used):
        if not self.force_escape_after_approach_nav or not approach_nav_used:
            return False
        if parking is None:
            return False
        blocked_names = getattr(parking, "relative_blocked_names", [])
        return len(blocked_names) > 0


    def fallback_odom_distance_to_goal_xy(self, target):
        try:
            odom = rospy.wait_for_message('/odom', Odometry, timeout=0.2)
            dx = target[0] - odom.pose.pose.position.x
            dy = target[1] - odom.pose.pose.position.y
            return np.sqrt(dx * dx + dy * dy)
        except Exception:
            return None

    def clamp(self, value, min_value, max_value):
        """限制数值范围"""
        return max(min_value, min(max_value, value))
    
    def get_range_at_angle(self, angle):
        """获取指定角度的激光距离"""
        # 确保角度在雷达扫描范围内
        if angle > np.pi:
            angle -= 2 * np.pi
        elif angle < -np.pi:
            angle += 2 * np.pi
        
        # 计算激光数据索引
        index = int((angle - self.scan_data.angle_min) / self.scan_data.angle_increment)
        
        # 确保索引在有效范围内
        if 0 <= index < len(self.scan_data.ranges):
            distance = self.scan_data.ranges[index]
            # 检查距离是否在有效范围内
            if self.scan_data.range_min <= distance <= self.scan_data.range_max:
                return distance
        return float('nan')  # 返回NaN表示无效值
    def adjust_position(self, side_target, back_target):
        """
        执行位置校准
        :param side_target: 侧方(+90°)目标距离 (米)
        :param back_target: 后方(-180°)目标距离 (米)
        :return: 是否完成校准
        """
        if self.scan_data is None:
            rospy.logwarn("无激光数据，无法校准!")
            return False
        
        #rospy.loginfo(f"开始位置校准: 侧方目标={side_target:.2f}m, 后方目标={back_target:.2f}m, 航向角目标=0°")
        
        self.is_adjusting = True
        rate = rospy.Rate(10)  # 10Hz控制频率
        complete = False
        start_time = rospy.Time.now()
        while not rospy.is_shutdown() and self.is_adjusting and not complete:
                    # 检查超时
            if (rospy.Time.now() - start_time).to_sec() > 9:
                rospy.logwarn("位置校准超时!")
                self.stop_movement()
                return False
            # 获取关键角度距离
            left_dist = self.get_range_at_angle(np.pi/2)   # +90度 (左侧)
            back_dist = self.get_range_at_angle(np.pi)     # -180度 (后方)
            
            
            # 计算位置误差
            left_error = left_dist - side_target
            back_error = back_dist - back_target
            
            # 计算航向角误差
            yaw_error = self.normalize_angle(self.target_yaw - self.current_yaw)
            
            # 创建速度指令
            cmd = Twist()
            
            # 横向移动调整 (Y方向)
            if abs(left_error) > self.position_tolerance:
                cmd.linear.y = self.kp_linear * left_error   # 左侧：正误差→左移
            
            # 前后移动调整 (X方向)
            if abs(back_error) > self.position_tolerance:
                cmd.linear.x = -self.kp_linear * back_error
            
            # 航向角调整 (Z轴旋转)
            if abs(yaw_error) > self.yaw_tolerance:
                cmd.angular.z = self.kp_angular * yaw_error
            
            # 发布控制指令
            self.pub.publish(cmd)
            
            # 检查是否完成校准
            position_ok = (abs(left_error) < self.position_tolerance and 
                          abs(back_error) < self.position_tolerance)
            yaw_ok = abs(yaw_error) < self.yaw_tolerance
            
            if position_ok and yaw_ok:
                rospy.loginfo("位置和航向角校准完成!")
                complete = True
                self.stop_movement()
                break
            elif position_ok and not yaw_ok:
                rospy.loginfo_throttle(1, "位置已校准，正在调整航向角...")
            elif not position_ok and yaw_ok:
                rospy.loginfo_throttle(1, "航向角已校准，正在调整位置...")
            
            # 调试信息
            #rospy.loginfo_throttle(0.5, 
            #    f"校准中: 左侧={left_dist:.2f}m (目标:{side_target:.2f}), "
             #   f"后方={back_dist:.2f}m (目标:{back_target:.2f}), "
             #   f"航向角={np.degrees(self.current_yaw):.2f}° (目标:0.0°)")
            
            rate.sleep()
        return complete
    def stop_movement(self):
        """停止机器人运动"""
        cmd = Twist()
        self.pub.publish(cmd)
        self.is_adjusting = False

    def make_detection_prealign_goal(self, target):
        """按配置方向生成检测点预对准位姿"""
        yaw_rad = target[2] / 180.0 * pi
        forward_x = np.cos(yaw_rad)
        forward_y = np.sin(yaw_rad)
        left_x = -np.sin(yaw_rad)
        left_y = np.cos(yaw_rad)
        mode = str(self.detect_prealign_mode).strip().lower()
        distance = self.detect_prealign_distance

        if mode == "front":
            offset_x = forward_x * distance
            offset_y = forward_y * distance
        elif mode == "left":
            offset_x = left_x * distance
            offset_y = left_y * distance
        elif mode == "right":
            offset_x = -left_x * distance
            offset_y = -left_y * distance
        else:
            offset_x = -forward_x * distance
            offset_y = -forward_y * distance

        return [
            target[0] + offset_x,
            target[1] + offset_y,
            target[2]
        ]

    def get_locked_approach_velocity(self, speed):
        """根据预对准方向生成保持当前yaw时的base_link速度"""
        mode = str(self.detect_prealign_mode).strip().lower()
        if mode == "front":
            return -speed, 0.0
        if mode == "left":
            return 0.0, -speed
        if mode == "right":
            return 0.0, speed
        return speed, 0.0

    def wait_for_odom_yaw(self, timeout=1.0):
        """等待里程计航向角可用"""
        start_time = rospy.Time.now()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and not self.odom_received:
            if (rospy.Time.now() - start_time).to_sec() > timeout:
                return False
            rate.sleep()
        return True

    def align_detection_yaw(self, yaw_deg):
        """
        拍照前低速闭环修正 yaw，避免 move_base 到点后最后一刻大幅旋转。
        :param yaw_deg: 目标航向角，单位为度
        """
        if not self.detect_yaw_align_enabled:
            return True
        if not self.wait_for_odom_yaw(timeout=1.0):
            rospy.logwarn("未收到里程计yaw，跳过检测点yaw闭环")
            return False

        target_yaw = yaw_deg / 180.0 * pi
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)
        stable_count = 0

        rospy.loginfo("检测点yaw闭环开始: target=%.1fdeg tolerance=%.3frad" %
                      (yaw_deg, self.detect_yaw_tolerance))
        while not rospy.is_shutdown():
            yaw_error = self.normalize_angle(target_yaw - self.current_yaw)
            if abs(yaw_error) <= self.detect_yaw_tolerance:
                stable_count += 1
                self.stop_movement()
                if stable_count >= self.detect_yaw_stable_count:
                    rospy.loginfo("检测点yaw闭环完成: err=%.3frad" % yaw_error)
                    rospy.sleep(self.detect_photo_settle_time)
                    return True
            else:
                stable_count = 0
                cmd = Twist()
                omega = self.clamp(
                    self.detect_yaw_kp * yaw_error,
                    -self.detect_yaw_max_vel,
                    self.detect_yaw_max_vel
                )
                if abs(omega) < self.detect_yaw_min_vel:
                    omega = self.detect_yaw_min_vel if omega >= 0 else -self.detect_yaw_min_vel
                cmd.angular.z = omega
                self.pub.publish(cmd)

            if (rospy.Time.now() - start_time).to_sec() > self.detect_yaw_align_timeout:
                self.stop_movement()
                rospy.logwarn("检测点yaw闭环超时: err=%.3frad" % yaw_error)
                rospy.sleep(self.detect_photo_settle_time)
                return False

            rate.sleep()

        self.stop_movement()
        return False

    def align_final_yaw(self, yaw_deg):
        """终点贴边前先对齐终点 yaw，避免按错误车体方向做激光校准。"""
        if not self.wait_for_odom_yaw(timeout=1.0):
            rospy.logwarn("未收到里程计yaw，跳过终点yaw闭环")
            return False

        target_yaw = yaw_deg / 180.0 * pi
        self.target_yaw = target_yaw
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)
        stable_count = 0

        rospy.loginfo("[FINAL][YAW_ALIGN][START] target=%.1fdeg tolerance=%.3frad",
                      yaw_deg, self.final_yaw_tolerance)
        while not rospy.is_shutdown():
            yaw_error = self.normalize_angle(target_yaw - self.current_yaw)
            if abs(yaw_error) <= self.final_yaw_tolerance:
                stable_count += 1
                self.stop_movement()
                if stable_count >= self.final_yaw_stable_count:
                    rospy.loginfo("[FINAL][YAW_ALIGN][OK] err=%.3frad", yaw_error)
                    return True
            else:
                stable_count = 0
                cmd = Twist()
                omega = self.clamp(
                    self.final_yaw_kp * yaw_error,
                    -self.final_yaw_max_vel,
                    self.final_yaw_max_vel
                )
                if abs(omega) < self.final_yaw_min_vel:
                    omega = self.final_yaw_min_vel if omega >= 0 else -self.final_yaw_min_vel
                cmd.angular.z = omega
                self.pub.publish(cmd)

            if (rospy.Time.now() - start_time).to_sec() > self.final_yaw_align_timeout:
                self.stop_movement()
                rospy.logwarn("[FINAL][YAW_ALIGN][TIMEOUT] err=%.3frad", yaw_error)
                return False

            rate.sleep()

        self.stop_movement()
        return False

    def locked_approach_detection_point(self, yaw_deg):
        """
        从预对准点到拍照点的短距离直行段。
        不再交给move_base，避免TEB在最后0.6m重新优化yaw。
        """
        distance = self.detect_prealign_distance
        speed = abs(self.detect_locked_approach_speed)
        if distance <= 0.0 or speed <= 0.0:
            rospy.logwarn("锁yaw靠近参数无效: distance=%.3f speed=%.3f" %
                          (distance, speed))
            return False
        if not self.wait_for_odom_yaw(timeout=1.0):
            rospy.logwarn("未收到里程计yaw，无法锁yaw靠近拍照点")
            return False

        target_yaw = yaw_deg / 180.0 * pi
        travel_time = distance / speed
        timeout = travel_time + self.detect_locked_approach_timeout_margin
        cmd_x, cmd_y = self.get_locked_approach_velocity(speed)
        start_time = rospy.Time.now()
        rate = rospy.Rate(20)

        rospy.loginfo("锁yaw靠近拍照点: mode=%s distance=%.3fm speed=%.3fm/s vx=%.3f vy=%.3f time=%.2fs target=%.1fdeg" %
                      (self.detect_prealign_mode, distance, speed, cmd_x, cmd_y, travel_time, yaw_deg))
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed >= travel_time:
                self.stop_movement()
                rospy.sleep(self.detect_photo_settle_time)
                return True
            if elapsed > timeout:
                self.stop_movement()
                rospy.logwarn("锁yaw靠近拍照点超时")
                return False

            cmd = Twist()
            cmd.linear.x = cmd_x
            cmd.linear.y = cmd_y

            if self.detect_locked_approach_yaw_hold:
                yaw_error = self.normalize_angle(target_yaw - self.current_yaw)
                if abs(yaw_error) > self.detect_yaw_tolerance:
                    cmd.angular.z = self.clamp(
                        self.detect_locked_approach_yaw_kp * yaw_error,
                        -self.detect_locked_approach_max_yaw_vel,
                        self.detect_locked_approach_max_yaw_vel
                    )

            self.pub.publish(cmd)
            rate.sleep()

        self.stop_movement()
        return False

    # ---------------- TTS语音播报客户端 ----------------
    def tts_client(self, text):
        """
        功能：调用语音服务播报文本
        适配：自定义TTS服务，参数为data字段
        :param text: 要播报的中文文本
        """
        # Python2中文编码兼容
        if isinstance(text, unicode):
            text = text.encode('utf-8')
        try:
            # 构造语音服务请求
            request = StringServiceRequest()
            request.data = text  # 服务接收的关键字段
            response = self.tts_service(request)
            rospy.loginfo("TTS播报成功: %s | 响应: %s" % (text, response.result))
            return True
        except rospy.ServiceException as e:
            rospy.logerr("TTS服务调用失败: %s" % str(e))
            return False

    # ---------------- 调用视觉检测服务 ----------------
    def call_fruit_detection_service(self):
        """
        功能：调用视觉服务识别线索(返回数字1-9)
        """
        try:
            # 设置参数：启动检测
            rospy.set_param('/detect', 1)
            rospy.sleep(self.detect_capture_wait)
            # 调用服务并获取识别结果
            response = self.fruit_detection_service()
            rospy.loginfo("视觉大模型识别结果: %s" % response.message)
            return response.message
        except rospy.ServiceException as e:
            rospy.logerr("视觉大模型服务调用失败: %s" % e)
            return "无"

    # ---------------- 机器人终点动作(2/4) ----------------
    def start24(self):
        """起点动作，冲出障碍区：左上方斜移，可叠加慢速左转"""
        global time_val
        msg = Twist()
        msg.linear.x = 0.25    # X轴：前进
        msg.linear.y = 0.1     # Y轴：左移
        msg.angular.z = 0.0
        # 持续发布速度指令1.3秒
        while time_val <= 13:
            elapsed = (time_val - 1) * 0.1
            if self.start_escape_turn_enabled and elapsed < self.start_escape_turn_duration:
                msg.angular.z = abs(self.start_escape_turn_speed)
            else:
                msg.angular.z = 0.0
            self.pub.publish(msg)
            rospy.sleep(0.1)
            time_val += 1
        self.pub.publish(Twist())

    # ---------------- 机器人终点动作(1/3) ----------------
    def end13(self):
        """终点动作：快速后退+右移"""
        global time_val
        msg = Twist()
        msg.linear.x = -0.3
        msg.linear.y = 0.3
        msg.angular.z = 0.0
        while time_val <= 13:
            self.pub.publish(msg)
            rospy.sleep(0.1)
            time_val += 1

    # ---------------- 机器人旋转动作 ----------------
    def rotate(self):
        """原地旋转(用于环视检测)"""
        time1 = 0
        msg = Twist()
        msg.angular.z = 1.0    # 角速度：左转
        # 旋转0.8秒
        while time1 <= 8:
            self.pub.publish(msg)
            rospy.sleep(0.1)
            time1 += 1

    # ---------------- 机器人右移动作 ----------------
    def right(self):
        """右侧平移"""
        time1 = 0
        msg = Twist()
        msg.linear.y = -0.5
        # 平移2秒
        while time1 <= 20:
            self.pub.publish(msg)
            rospy.sleep(0.1)
            time1 += 1

    # ---------------- 设置机器人初始位姿 ----------------
    def set_pose(self, p):
        """
        功能：告诉机器人在地图中的初始坐标
        :param p: [x坐标, y坐标, 朝向角度]
        """
        if self.move_base is None:
            return False
        x, y, th = p
        # 构造位姿消息
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        # 欧拉角(角度) → 四元数(ROS姿态标准格式)
        q = quaternion_from_euler(0.0, 0.0, th / 180.0 * pi)
        pose.pose.pose.orientation.x = q[0]
        pose.pose.pose.orientation.y = q[1]
        pose.pose.pose.orientation.z = q[2]
        pose.pose.pose.orientation.w = q[3]
        # 发布初始位姿
        self.set_pose_pub.publish(pose)
        return True

    # ---------------- 导航回调函数 ----------------
    def _done_cb(self, status, result):
        """导航完成后自动调用"""
        self.last_move_base_state = status
        rospy.loginfo("导航完成! status=%s result=%s" % (status, result))
        self.arrive_pub.publish("arrived to target point")

    def _active_cb(self):
        """导航开始时自动调用"""
        rospy.loginfo("[Navi] 导航已激活")

    def _feedback_cb(self, feedback):
        """导航过程中实时反馈(无需处理)"""
        self.last_move_base_feedback = feedback

    # ---------------- 核心：导航到目标点 ----------------
    def goto(self, p, timeout=60):
        """
        功能：导航到指定坐标
        :param p: [x, y, 朝向角度]
        :param timeout: 超时秒数，默认60
        """
        rospy.loginfo("[Navi] 前往目标点: %s (timeout=%.1fs)" % (p, timeout))
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = p[0]
        goal.target_pose.pose.position.y = p[1]
        q = quaternion_from_euler(0.0, 0.0, p[2] / 180.0 * pi)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.reset_nav_feedback()
        self.move_base.send_goal(goal, self._done_cb, self._active_cb, self._feedback_cb)
        result = self.move_base.wait_for_result(rospy.Duration(timeout))
        if not result:
            self.move_base.cancel_goal()
            self.last_move_base_state = GoalStatus.PREEMPTED
            rospy.loginfo("导航超时，取消目标")
            return False
        else:
            state = self.move_base.get_state()
            self.last_move_base_state = state
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("到达目标点 %s 成功! " % p)
                return True
            rospy.logwarn("导航未成功到达目标点 %s，state=%s" %
                          (p, state))
            return False

    def goto_task_nav_goal(self, p, timeout=60, label="", mode=""):
        """
        任务点导航专用：在常规 timeout 外，检测规划失败和目标距离长时间没有变近。
        这样目标点在墙里/局部规划卡住时，可以更快切换到 approach 或下一个 approach。
        """
        rospy.loginfo(
            "[TASK_NAV][GOTO_START] label=%s mode=%s target=%s timeout=%.1fs no_progress=%s no_progress_timeout=%.1fs min_delta=%.3f",
            label, mode, str(p), timeout,
            str(self.task_nav_no_progress_enabled),
            self.task_nav_no_progress_timeout,
            self.task_nav_no_progress_min_delta
        )
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = p[0]
        goal.target_pose.pose.position.y = p[1]
        q = quaternion_from_euler(0.0, 0.0, p[2] / 180.0 * pi)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.reset_nav_feedback()
        self.task_nav_goal_active = True
        self.task_nav_plan_fail_cancel_requested = False
        self.task_nav_plan_fail_seen = 0
        self.task_nav_plan_fail_window_start = rospy.Time(0)
        self.task_nav_plan_fail_label = label
        self.task_nav_plan_fail_mode = mode
        self.move_base.send_goal(goal, self._done_cb, self._active_cb, self._feedback_cb)

        start_time = rospy.Time.now()
        best_dist = None
        last_progress_time = start_time
        rate = rospy.Rate(5)
        try:
            while not rospy.is_shutdown():
                if self.task_nav_plan_fail_cancel_requested:
                    self.move_base.cancel_goal()
                    self.last_move_base_state = GoalStatus.PREEMPTED
                    rospy.logwarn(
                        "[TASK_NAV][PLAN_FAIL_CANCEL] label=%s mode=%s count=%d window=%.2fs best_dist=%s",
                        label, mode,
                        self.task_nav_plan_fail_seen,
                        self.task_nav_plan_fail_window,
                        "%.3f" % best_dist if best_dist is not None else "None"
                    )
                    return False

                elapsed = (rospy.Time.now() - start_time).to_sec()
                if elapsed > timeout:
                    self.move_base.cancel_goal()
                    self.last_move_base_state = GoalStatus.PREEMPTED
                    rospy.logwarn(
                        "[TASK_NAV][TIMEOUT_CANCEL] label=%s mode=%s elapsed=%.2fs timeout=%.2fs best_dist=%s",
                        label, mode, elapsed, timeout,
                        "%.3f" % best_dist if best_dist is not None else "None"
                    )
                    return False

                state = self.move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    self.last_move_base_state = state
                    rospy.loginfo("[TASK_NAV][GOTO_DONE] label=%s mode=%s state=SUCCEEDED", label, mode)
                    return True
                if state in [GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.PREEMPTED, GoalStatus.RECALLED]:
                    self.last_move_base_state = state
                    rospy.logwarn("[TASK_NAV][GOTO_FAILED_STATE] label=%s mode=%s state=%s", label, mode, state)
                    return False

                if self.task_nav_no_progress_enabled:
                    dist = self.distance_to_goal_xy(p)
                    if dist is not None:
                        if best_dist is None or dist < best_dist - self.task_nav_no_progress_min_delta:
                            best_dist = dist
                            last_progress_time = rospy.Time.now()
                        elif (rospy.Time.now() - last_progress_time).to_sec() > self.task_nav_no_progress_timeout:
                            self.move_base.cancel_goal()
                            self.last_move_base_state = GoalStatus.PREEMPTED
                            rospy.logwarn(
                                "[TASK_NAV][NO_PROGRESS_CANCEL] label=%s mode=%s dist=%.3f best_dist=%.3f idle=%.2fs timeout=%.2fs",
                                label, mode, dist, best_dist,
                                (rospy.Time.now() - last_progress_time).to_sec(),
                                self.task_nav_no_progress_timeout
                            )
                            return False

                rate.sleep()
        finally:
            self.task_nav_goal_active = False

        self.move_base.cancel_goal()
        self.last_move_base_state = GoalStatus.PREEMPTED
        return False

    def goto_detection_point(self, point):
        """检测点导航：先用同yaw预对准，再进入原拍照点并短闭环修正yaw"""
        target = goals[point]
        prealign_ok = True
        if self.detect_prealign_enabled and self.detect_prealign_distance > 0.0:
            prealign_goal = self.make_detection_prealign_goal(target)
            rospy.loginfo("检测点%s预对准目标: %s" % (point, prealign_goal))
            prealign_ok = self.goto(prealign_goal, timeout=self.detect_prealign_timeout)
            if self.detect_yaw_align_at_prealign:
                self.align_detection_yaw(target[2])

        if self.detect_locked_final_approach:
            if not prealign_ok:
                rospy.logwarn("检测点%s预对准未确认成功，仍使用锁yaw靠近，避免再次发送最终点move_base" % point)
            self.locked_approach_detection_point(target[2])
        else:
            rospy.loginfo("检测点%s原始拍照目标: %s" % (point, target))
            self.goto(target, timeout=self.detect_final_timeout)

        if self.detect_yaw_align_at_photo:
            self.align_detection_yaw(target[2])
        if self.detect_photo_settle_time > 0:
            rospy.sleep(self.detect_photo_settle_time)
        return True

    # ---------------- 取消导航 ----------------
    def cancel(self):
        self.move_base.cancel_all_goals()
        return True

    # ---------------- 单个检测点完整任务逻辑 ----------------
    def mission(self, point):
        """
        单个检测点执行流程：
        1. 导航到检测点
        2. 视觉识别线索
        3. 语音播报线索
        4. 保存线索编号
        """
        global clue, id, find_id
        id = 0
        find_id = 0
        rospy.sleep(0.1)

        rospy.loginfo("导航到检测点 → 目标点索引%s" % point)
        # 步骤1：导航到预设检测点
        self.goto_detection_point(point)

        # 步骤2：调用视觉检测
        detect_result = self.call_fruit_detection_service()
        rospy.loginfo("当前检测点%s结果: %s" % (point, detect_result))

        # 步骤3: 处理识别结果
        if detect_result != "无":
            try:
                # 转换为数字
                task_id = int(detect_result)
                if task_id in VLM_TO_TASK:
                    # 映射 VLM 返回值到任务编号：31→1, 32→2, ..., 51→9
                    mapped_id = VLM_TO_TASK[task_id]
                    task_numbers.append(mapped_id)
                    rospy.loginfo("收集到任务编号: %s (原始VLM: %s)" % (mapped_id, task_id))
                    # 语音播报：已检测第X条线索为X号（用原始VLM识别编号）
                    tts_text = u"已检测第%d条线索为%d号" % (clue, task_id)
                    self.tts_client(tts_text)
                    clue += 1  # 线索计数+1
                else:
                    rospy.logwarn("任务编号超出范围: %s" % task_id)
            except ValueError:
                rospy.logwarn("检测结果不是有效数字: %s" % detect_result)
        # 重置标记
        id = 0
        find_id = 0

    # ---------------- 执行识别 ----------------
    def recognize(self, p):
        self.mission(p)
        return True

    def parse_fixed_task_ids(self):
        """
        解析固定任务点列表。
        支持内部任务编号1-9，也兼容VLM原始编号31/32/.../51。
        """
        parsed_tasks = []
        raw_text = str(self.fixed_task_ids).strip()
        if not raw_text:
            rospy.logwarn("use_fixed_task_positions=true，但 fixed_task_ids 为空")
            return parsed_tasks

        for item in raw_text.replace(";", ",").split(","):
            item = item.strip()
            if not item:
                continue
            try:
                raw_id = int(item)
            except ValueError:
                rospy.logwarn("固定任务点编号无效: %s" % item)
                continue

            if 1 <= raw_id <= 9:
                task_id = raw_id
            elif raw_id in VLM_TO_TASK:
                task_id = VLM_TO_TASK[raw_id]
            else:
                rospy.logwarn("固定任务点编号超出范围: %s" % raw_id)
                continue

            parsed_tasks.append(task_id)

        return parsed_tasks

    # ---------------- 按线索导航到任务点 ----------------
    def go_to_task_positions(self):
        """按识别到的线索，依次导航到对应任务点"""
        rospy.loginfo("开始按顺序前往任务位置: %s" % task_numbers)
        # 先导航到中转点14
        # self.goto(goals[14])
        # 遍历所有线索
        last_parking = None
        last_task_id = None
        for idx, task_id in enumerate(task_numbers):
            task_start_time = rospy.Time.now()
            rospy.loginfo("[TASK_TIME][START] idx=%d/%d task_id=%s",
                          idx + 1, len(task_numbers), str(task_id))
            if 1 <= task_id <= 9:
                # 导航到线索对应的任务点 (5s 超时)
                target = goals[task_id]
                self.log_nav_state("TASK_NAV_START_%d" % task_id, target)
                nav_ok, nav_reached, nav_dist, nav_mode = self.navigate_task_with_all_approaches(
                    idx, task_id, target, last_parking, last_task_id)
                if not nav_reached:
                    rospy.logwarn(
                        "[TASK_TIME][PARK_SKIP_NAV_TOO_FAR] idx=%d task_id=%d target_dist=%s accept=%.3f",
                        idx + 1, task_id,
                        "%.3f" % nav_dist if nav_dist is not None else "None",
                        self.task_nav_accept_dist
                    )
                    rospy.loginfo("[TASK_TIME][END] idx=%d task_id=%d total_dt=%.2fs skipped=true reason=nav_too_far",
                                  idx + 1, task_id,
                                  (rospy.Time.now() - task_start_time).to_sec())
                    continue
                approach_nav_used = nav_mode not in [None, "target"]
                self.log_nav_state("TASK_NAV_DONE_%d" % task_id, target)

                rospy.loginfo("[TASK_TIME][PRE_PARK_WAIT][SKIP] idx=%d task_id=%d",
                              idx + 1, task_id)

                # 启动精密停车
                rospy.loginfo("move_base 到达任务点 %d，启动精密停车 (x=%.3f y=%.3f yaw=%.1f)..." % (task_id, target[0], target[1], target[2]))
                rospy.loginfo("[PARK_TASK][START] idx=%d task_id=%d target=(%.3f, %.3f, %.1f)",
                              idx + 1, task_id, target[0], target[1], target[2])

                parking_init_start_time = rospy.Time.now()
                parking = AutoSinglePointTest(target_x=target[0], target_y=target[1], target_yaw_deg=target[2])
                rospy.loginfo("[TASK_TIME][PARK_INIT] idx=%d task_id=%d dt=%.2fs",
                              idx + 1, task_id,
                              (rospy.Time.now() - parking_init_start_time).to_sec())

                parking_run_start_time = rospy.Time.now()
                parking.run()
                rospy.loginfo("[TASK_TIME][PARK_RUN] idx=%d task_id=%d dt=%.2fs parking_done=%s best_entry=%s",
                              idx + 1, task_id,
                              (rospy.Time.now() - parking_run_start_time).to_sec(),
                              str(parking.parking_done),
                              parking.best_entry["name"] if parking.best_entry is not None else "None")

                post_wait_start_time = rospy.Time.now()
                rospy.sleep(0.5)
                rospy.loginfo("[TASK_TIME][POST_PARK_WAIT] idx=%d task_id=%d dt=%.2fs",
                              idx + 1, task_id,
                              (rospy.Time.now() - post_wait_start_time).to_sec())

                # 语音播报到达任务点（用原始VLM识别编号）
                raw_id = TASK_TO_VLM.get(task_id, task_id)
                tts_text = u"已到达任务点%d号" % raw_id
                tts_start_time = rospy.Time.now()
                tts_ok = self.tts_client(tts_text)
                rospy.loginfo("[TASK_TIME][TTS] idx=%d task_id=%d dt=%.2fs ok=%s",
                              idx + 1, task_id,
                              (rospy.Time.now() - tts_start_time).to_sec(),
                              str(tts_ok))

                # 播报完毕，逃逸离开挡板区域
                escape_start_time = rospy.Time.now()
                force_escape = self.should_force_escape_after_approach(parking, approach_nav_used)
                if force_escape:
                    parking.escape(force=True, reason="approach_nav_%s" % str(nav_mode))
                else:
                    parking.escape()
                rospy.loginfo("[TASK_TIME][ESCAPE] idx=%d task_id=%d dt=%.2fs mode=%s forced=%s",
                              idx + 1, task_id,
                              (rospy.Time.now() - escape_start_time).to_sec(),
                              str(nav_mode), str(force_escape))
                last_parking = parking
                last_task_id = task_id
                rospy.loginfo("[TASK_TIME][END] idx=%d task_id=%d total_dt=%.2fs",
                              idx + 1, task_id,
                              (rospy.Time.now() - task_start_time).to_sec())
            else:
                rospy.logwarn("任务编号%s无效，跳过" % task_id)
                rospy.loginfo("[TASK_TIME][END] idx=%d task_id=%s total_dt=%.2fs skipped=true",
                              idx + 1, str(task_id),
                              (rospy.Time.now() - task_start_time).to_sec())

    # ---------------- 执行完整任务流程 ----------------
    def execute_mission(self):
        """
        完整任务流程：
        1. 遍历所有检测点(10/11/12/13)识别线索
        2. 按线索导航到任务点
        3. 导航到终点并执行动作
        """
        global task_numbers, clue
        task_numbers = []
        clue = 1

        rospy.loginfo("开始执行任务！")
        if self.use_fixed_task_positions:
            task_numbers = self.parse_fixed_task_ids()
            rospy.loginfo("使用固定任务点，跳过检测点扫描: raw=%s parsed=%s" %
                          (self.fixed_task_ids, task_numbers))
        else:
            # 执行所有检测点任务
            for p in points:
                rospy.loginfo("\n=== 开始处理第%s个检测点 ===" % (1))
                self.recognize(p)

            rospy.loginfo("\n=== 所有检测点处理完成 ===")
            rospy.loginfo("收集到的任务编号: %s" % task_numbers)

        # 按线索导航
        self.set_parking_phase_costmap()
        try:
            self.go_to_task_positions()
        finally:
            self.restore_cruise_costmap()

        # 终点只让 move_base 粗到位，最后贴边交给激光闭环校准
        final_nav_start = rospy.Time.now()
        final_nav_ok = self.goto(goals[16], timeout=self.final_nav_timeout)
        rospy.loginfo("[FINAL][NAV_TO_FINAL] dt=%.2fs ok=%s timeout=%.1fs",
                      (rospy.Time.now() - final_nav_start).to_sec(),
                      str(final_nav_ok), self.final_nav_timeout)
        self.target_yaw = goals[16][2] / 180.0 * pi
        rospy.loginfo("[FINAL][ADJUST_POSITION][START] target_yaw=%.1fdeg side=0.220 back=0.240",
                      goals[16][2])
        final_adjust_ok = self.adjust_position(side_target=0.220, back_target=0.240)
        rospy.loginfo("[FINAL][ADJUST_POSITION][DONE] ok=%s", str(final_adjust_ok))
        # 语音播报到达终点
        tts_text = u"已到达终点"
        self.tts_client(tts_text)

    # ---------------- 任务启动回调(空挂，不使用) ----------------
    def start_mission_callback(self, msg):
        pass


# =================== 主函数 ===================
if __name__ == "__main__":
    # 1. 初始化ROS节点
    rospy.init_node('navigation_demo', anonymous=True)
    rospy.loginfo("导航节点初始化成功! 等待语音唤醒信号...")
    try:
        # 2. 从launch文件读取导航点位参数
        goalListX = rospy.get_param('~goalListX')     # X坐标列表
        goalListY = rospy.get_param('~goalListY')     # Y坐标列表
        goalListYaw = rospy.get_param('~goalListYaw') # 朝向角度列表

        # 字符串转浮点点列表
        x_list = [float(x.strip()) for x in goalListX.split(",") if x.strip()]
        y_list = [float(y.strip()) for y in goalListY.split(",") if y.strip()]
        yaw_list = [float(yaw.strip()) for yaw in goalListYaw.split(",") if yaw.strip()]

        # 组合成[x,y,yaw]格式的目标点列表
        goals = []
        for x, y, yaw in zip(x_list, y_list, yaw_list):
            goals.append([x, y, yaw])

    except KeyError as e:
        rospy.logerr("未找到点位参数 %s，请检查launch文件！" % e)
        sys.exit(1)
    except Exception as e:
        rospy.logerr("解析点位失败: %s" % e)
        sys.exit(1)

    # 3 创建导航对象
    navi = navigation_demo()
    # 4. 订阅启动话题(空挂，不使用)
    rospy.Subscriber('/start_mission', String, navi.start_mission_callback)

    # 5. 等待IMU初始化完成
    rospy.loginfo("等待IMU传感器激活...")
    imu_msg = rospy.wait_for_message('/imu/data', Imu, timeout=None)
    rospy.loginfo("IMU传感器已激活，5秒后开始任务...")

    # 6. 延时5秒，等待系统稳定
    rospy.sleep(5)

    # 7. 播报离线音频并开始任务
    os.system('ffplay -nodisp -autoexit -loglevel quiet /home/abot/EIU0US/src/robot_slam/resources/startGame.wav')
    # navi.adjust_position(side_target=2.352, back_target=0.600) 
    navi.start24()
    navi.execute_mission()

    # 8. 保持节点运行
    rospy.spin()
