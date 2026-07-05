#!/usr/bin/env python2
# -*- coding: utf-8 -*-


# =================== 导入依赖库/ROS消息 ===================
import rospy
import actionlib
import numpy as np
import tf
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from nav_msgs.srv import GetPlan, GetPlanRequest
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
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
from std_srvs.srv import Trigger, TriggerRequest, SetBool, SetBoolRequest
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
        self.tf_listener = tf.TransformListener()

        # 9. PID校准参数
        self.kp_linear = 0.5        # 线速度比例系数
        self.kp_angular = 0.5       # 角速度比例系数
        self.position_tolerance = 0.02   # 位置容差 (米)
        self.yaw_tolerance = 0.05        # 航向角容差 (弧度, ~3°)
        self.target_yaw = 0.0            # 目标航向角
        self.current_yaw = 0.0           # 当前航向角
        self.current_odom_pose = None
        self.odom_received = False
        self.is_adjusting = False

        # 10. 检测点拍照前预对准参数
        self.detect_prealign_enabled = rospy.get_param("~detect_prealign_enabled", True)
        self.detect_prealign_mode = rospy.get_param("~detect_prealign_mode", "back")
        self.detect_prealign_distance = rospy.get_param("~detect_prealign_distance", 0.35)
        self.detect_prealign_timeout = rospy.get_param("~detect_prealign_timeout", 25)
        self.detect_nav_retry_enabled = rospy.get_param("~detect_nav_retry_enabled", True)
        self.detect_nav_retry_modes = rospy.get_param(
            "~detect_nav_retry_modes",
            "back,back_left,left,front_left,front,front_right,right,back_right")
        self.detect_nav_retry_distance = rospy.get_param("~detect_nav_retry_distance", 0.35)
        self.detect_nav_retry_timeout = rospy.get_param(
            "~detect_nav_retry_timeout", self.detect_prealign_timeout)
        self.detect_nav_accept_dist = rospy.get_param("~detect_nav_accept_dist", 0.18)
        self.detect_skip_capture_on_nav_fail = rospy.get_param(
            "~detect_skip_capture_on_nav_fail", True)
        self.detect_require_all_points = rospy.get_param("~detect_require_all_points", True)
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
        self.detect_dynamic_yaw_enabled = rospy.get_param("~detect_dynamic_yaw_enabled", True)
        self.detect_dynamic_yaw_fallback_on_none = rospy.get_param(
            "~detect_dynamic_yaw_fallback_on_none", True)
        self.detect_dynamic_yaw_min_distance = rospy.get_param("~detect_dynamic_yaw_min_distance", 0.05)
        self.detect_dynamic_yaw_capture_at_prealign = rospy.get_param(
            "~detect_dynamic_yaw_capture_at_prealign", True)
        self.detect_photo_target_points_param = rospy.get_param("~detect_photo_target_points", "")
        self.detect_photo_target_x_param = rospy.get_param(
            "~detect_photo_target_x", rospy.get_param("~detectPhotoTargetX", ""))
        self.detect_photo_target_y_param = rospy.get_param(
            "~detect_photo_target_y", rospy.get_param("~detectPhotoTargetY", ""))
        self.detect_photo_target_map = self.parse_detection_photo_targets()
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
        self.final_prealign_enabled = rospy.get_param("~final_prealign_enabled", True)
        self.final_prealign_mode = rospy.get_param("~final_prealign_mode", "back")
        self.final_prealign_distance = rospy.get_param("~final_prealign_distance", 0.35)
        self.final_prealign_timeout = rospy.get_param("~final_prealign_timeout", self.final_nav_timeout)
        self.final_align_yaw_before_laser = rospy.get_param("~final_align_yaw_before_laser", True)
        self.final_yaw_align_timeout = rospy.get_param("~final_yaw_align_timeout", 3.0)
        self.final_yaw_tolerance = rospy.get_param("~final_yaw_tolerance", 0.05)
        self.final_yaw_kp = rospy.get_param("~final_yaw_kp", 1.2)
        self.final_yaw_min_vel = rospy.get_param("~final_yaw_min_vel", 0.08)
        self.final_yaw_max_vel = rospy.get_param("~final_yaw_max_vel", 0.45)
        self.final_yaw_stable_count = int(rospy.get_param("~final_yaw_stable_count", 3))
        self.final_side_laser_direction = rospy.get_param("~final_side_laser_direction", "left")
        self.final_depth_laser_direction = rospy.get_param("~final_depth_laser_direction", "back")
        self.final_adjust_timeout = rospy.get_param("~final_adjust_timeout", 9.0)
        self.final_adjust_force_side_on_fail = rospy.get_param("~final_adjust_force_side_on_fail", True)
        self.final_force_side_duration = rospy.get_param("~final_force_side_duration", 1.5)
        self.final_force_side_speed = rospy.get_param("~final_force_side_speed", 0.04)
        self.final_force_depth_after_side = rospy.get_param("~final_force_depth_after_side", True)
        self.final_force_depth_duration = rospy.get_param("~final_force_depth_duration", 1.0)
        self.final_force_depth_speed = rospy.get_param("~final_force_depth_speed", 0.04)
        self.task_nav_timeout = rospy.get_param("~task_nav_timeout", 8.0)
        self.task_nav_retry_timeout = rospy.get_param("~task_nav_retry_timeout", 5.0)
        self.task_nav_accept_dist = rospy.get_param("~task_nav_accept_dist", 0.45)
        self.task_nav_approach_accept_dist = rospy.get_param("~task_nav_approach_accept_dist", 0.45)
        self.task_nav_direct_done_dist = rospy.get_param("~task_nav_direct_done_dist", 0.05)
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
        self.task_nav_flexible_yaw_enabled = rospy.get_param("~task_nav_flexible_yaw_enabled", True)
        self.task_nav_flexible_yaw_candidates_param = rospy.get_param(
            "~task_nav_flexible_yaw_candidates", "0,90,180,-90")
        self.task_nav_flexible_yaw_candidates = self.parse_yaw_candidates(
            self.task_nav_flexible_yaw_candidates_param)
        self.task_nav_approach_score_radius = rospy.get_param("~task_nav_approach_score_radius", 0.20)
        self.task_nav_path_filter_enabled = rospy.get_param("~task_nav_path_filter_enabled", True)
        self.task_nav_path_make_plan_service = rospy.get_param(
            "~task_nav_path_make_plan_service", "/move_base/make_plan")
        self.task_nav_path_make_plan_wait = rospy.get_param("~task_nav_path_make_plan_wait", 0.5)
        self.move_base_cancel_wait = rospy.get_param("~move_base_cancel_wait", 1.0)
        self.move_base_make_plan_idle_wait = rospy.get_param("~move_base_make_plan_idle_wait", 1.0)
        self.task_nav_path_sparse_distance = rospy.get_param("~task_nav_path_sparse_distance", 0.06)
        self.task_nav_path_sharp_turn_threshold_deg = rospy.get_param(
            "~task_nav_path_sharp_turn_threshold_deg", 65.0)
        self.task_nav_path_min_endpoint_distance = rospy.get_param(
            "~task_nav_path_min_endpoint_distance", 0.20)
        self.task_nav_transition_enabled = rospy.get_param("~task_nav_transition_enabled", True)
        self.task_nav_transition_distance_before_corner = rospy.get_param(
            "~task_nav_transition_distance_before_corner", 0.25)
        self.task_nav_transition_accept_dist = rospy.get_param(
            "~task_nav_transition_accept_dist", self.task_nav_approach_accept_dist)
        self.task_nav_teb_slow_fallback_enabled = rospy.get_param(
            "~task_nav_teb_slow_fallback_enabled", True)
        self.task_nav_teb_reconfigure_name = rospy.get_param(
            "~task_nav_teb_reconfigure_name", "move_base/TebLocalPlannerROS")
        self.task_nav_teb_slow_max_vel_x = rospy.get_param("~task_nav_teb_slow_max_vel_x", 0.12)
        self.task_nav_teb_slow_max_vel_y = rospy.get_param("~task_nav_teb_slow_max_vel_y", 0.12)
        self.task_nav_teb_slow_max_vel_theta = rospy.get_param(
            "~task_nav_teb_slow_max_vel_theta", 0.8)
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
        self.obstacle_memory_control_enabled = rospy.get_param("~obstacle_memory_control_enabled", True)
        self.obstacle_memory_clear_service = rospy.get_param(
            "~obstacle_memory_clear_service",
            "/move_base/global_costmap/obstacle_memory_layer/clear")
        self.obstacle_memory_set_enabled_service = rospy.get_param(
            "~obstacle_memory_set_enabled_service",
            "/move_base/global_costmap/obstacle_memory_layer/set_enabled")
        self.obstacle_memory_service_wait = rospy.get_param("~obstacle_memory_service_wait", 0.5)
        self.obstacle_memory_clear_client = None
        self.obstacle_memory_set_enabled_client = None
        self.start_escape_turn_enabled = rospy.get_param("~start_escape_turn_enabled", True)
        self.start_escape_turn_speed = rospy.get_param("~start_escape_turn_speed", 0.18)
        self.start_escape_turn_duration = rospy.get_param("~start_escape_turn_duration", 1.0)
        self.global_costmap = None
        self.task_nav_make_plan_client = None
        self.task_nav_teb_client = None
        self.task_nav_teb_nominal_config = None
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

    def get_teb_reconfigure_client(self):
        if self.task_nav_teb_client is not None:
            return self.task_nav_teb_client
        try:
            self.task_nav_teb_client = dynamic_reconfigure.client.Client(
                self.task_nav_teb_reconfigure_name,
                timeout=1.0
            )
            return self.task_nav_teb_client
        except Exception as e:
            rospy.logwarn(
                "[TASK_NAV][TEB_SLOW_CLIENT_FAILED] name=%s error=%s",
                self.task_nav_teb_reconfigure_name,
                str(e)
            )
            return None

    def set_teb_slow_mode(self, enabled, reason):
        if not self.task_nav_teb_slow_fallback_enabled:
            return False
        client = self.get_teb_reconfigure_client()
        if client is None:
            return False
        try:
            if enabled:
                if self.task_nav_teb_nominal_config is None:
                    config = client.get_configuration(timeout=1.0)
                    self.task_nav_teb_nominal_config = {
                        "max_vel_x": config.get("max_vel_x", 0.3),
                        "max_vel_y": config.get("max_vel_y", 0.3),
                        "max_vel_theta": config.get("max_vel_theta", 2.0)
                    }
                client.update_configuration({
                    "max_vel_x": float(self.task_nav_teb_slow_max_vel_x),
                    "max_vel_y": float(self.task_nav_teb_slow_max_vel_y),
                    "max_vel_theta": float(self.task_nav_teb_slow_max_vel_theta)
                })
                rospy.logwarn(
                    "[TASK_NAV][TEB_SLOW_SET] reason=%s vx=%.3f vy=%.3f wz=%.3f",
                    reason,
                    self.task_nav_teb_slow_max_vel_x,
                    self.task_nav_teb_slow_max_vel_y,
                    self.task_nav_teb_slow_max_vel_theta
                )
                return True

            if self.task_nav_teb_nominal_config is not None:
                client.update_configuration(self.task_nav_teb_nominal_config)
                rospy.loginfo(
                    "[TASK_NAV][TEB_SLOW_RESTORE] reason=%s vx=%.3f vy=%.3f wz=%.3f",
                    reason,
                    self.task_nav_teb_nominal_config.get("max_vel_x", 0.0),
                    self.task_nav_teb_nominal_config.get("max_vel_y", 0.0),
                    self.task_nav_teb_nominal_config.get("max_vel_theta", 0.0)
                )
                return True
        except Exception as e:
            self.task_nav_teb_client = None
            rospy.logwarn(
                "[TASK_NAV][TEB_SLOW_SET_FAILED] enabled=%s reason=%s error=%s",
                str(enabled), reason, str(e)
            )
        return False

    def wait_for_service_short(self, service_name, reason):
        try:
            rospy.wait_for_service(service_name, timeout=self.obstacle_memory_service_wait)
            return True
        except Exception as e:
            rospy.logwarn(
                "[OBSTACLE_MEMORY][SERVICE_UNAVAILABLE] reason=%s service=%s wait=%.2fs error=%s",
                reason,
                service_name,
                self.obstacle_memory_service_wait,
                str(e)
            )
            return False

    def clear_obstacle_memory(self, reason):
        if not self.obstacle_memory_control_enabled:
            rospy.loginfo("[OBSTACLE_MEMORY][CLEAR_SKIP] reason=%s enabled=false", reason)
            return False
        if not self.wait_for_service_short(self.obstacle_memory_clear_service, reason):
            return False
        try:
            if self.obstacle_memory_clear_client is None:
                self.obstacle_memory_clear_client = rospy.ServiceProxy(
                    self.obstacle_memory_clear_service, Trigger)
            response = self.obstacle_memory_clear_client(TriggerRequest())
            self.global_costmap = None
            rospy.loginfo(
                "[OBSTACLE_MEMORY][CLEAR] reason=%s ok=%s msg=%s",
                reason,
                str(response.success),
                response.message
            )
            return response.success
        except Exception as e:
            self.obstacle_memory_clear_client = None
            rospy.logwarn(
                "[OBSTACLE_MEMORY][CLEAR_FAILED] reason=%s service=%s error=%s",
                reason,
                self.obstacle_memory_clear_service,
                str(e)
            )
            return False

    def set_obstacle_memory_enabled(self, enabled, reason):
        if not self.obstacle_memory_control_enabled:
            rospy.loginfo(
                "[OBSTACLE_MEMORY][SET_SKIP] reason=%s target_enabled=%s control_enabled=false",
                reason,
                str(enabled)
            )
            return False
        if not self.wait_for_service_short(self.obstacle_memory_set_enabled_service, reason):
            return False
        try:
            if self.obstacle_memory_set_enabled_client is None:
                self.obstacle_memory_set_enabled_client = rospy.ServiceProxy(
                    self.obstacle_memory_set_enabled_service, SetBool)
            request = SetBoolRequest()
            request.data = bool(enabled)
            response = self.obstacle_memory_set_enabled_client(request)
            self.global_costmap = None
            rospy.loginfo(
                "[OBSTACLE_MEMORY][SET] reason=%s target_enabled=%s ok=%s msg=%s",
                reason,
                str(enabled),
                str(response.success),
                response.message
            )
            return response.success
        except Exception as e:
            self.obstacle_memory_set_enabled_client = None
            rospy.logwarn(
                "[OBSTACLE_MEMORY][SET_FAILED] reason=%s target_enabled=%s service=%s error=%s",
                reason,
                str(enabled),
                self.obstacle_memory_set_enabled_service,
                str(e)
            )
            return False

    def disable_obstacle_memory_for_parking(self, reason):
        self.set_obstacle_memory_enabled(False, reason)
        self.clear_obstacle_memory(reason + "_clear")

    def enable_obstacle_memory_after_parking(self, reason):
        self.clear_obstacle_memory(reason + "_clear")
        self.set_obstacle_memory_enabled(True, reason)

    def odom_callback(self, msg):
        """从里程计提取当前航向角"""
        orientation_q = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.current_yaw = yaw
        self.current_odom_pose = msg.pose.pose
        self.odom_received = True

    def normalize_angle(self, angle):
        """将角度归一化到[-π, π]范围内"""
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle

    def normalize_angle_deg(self, angle):
        """将角度归一化到(-180, 180]，让270度等价为-90度。"""
        while angle > 180.0:
            angle -= 360.0
        while angle <= -180.0:
            angle += 360.0
        return angle

    def yaw_diff_deg(self, a, b):
        return abs(self.normalize_angle((a - b) / 180.0 * pi) * 180.0 / pi)

    def parse_yaw_candidates(self, value):
        candidates = []
        seen = set()
        for item in str(value).split(","):
            item = item.strip()
            if not item:
                continue
            try:
                yaw = self.normalize_angle_deg(float(item))
            except ValueError:
                rospy.logwarn("[TASK_NAV][FLEX_YAW_BAD_CANDIDATE] value=%s", item)
                continue
            key = round(yaw, 3)
            if key in seen:
                continue
            seen.add(key)
            candidates.append(yaw)
        if not candidates:
            rospy.logwarn(
                "[TASK_NAV][FLEX_YAW_NO_CANDIDATES] raw=%s fallback=0,90,180,-90",
                str(value)
            )
            candidates = [0.0, 90.0, 180.0, -90.0]
        return candidates

    def parse_float_list(self, value):
        values = []
        for item in str(value).replace(";", ",").split(","):
            item = item.strip()
            if not item:
                continue
            try:
                values.append(float(item))
            except ValueError:
                rospy.logwarn("[DETECT_YAW][BAD_FLOAT] value=%s", item)
        return values

    def parse_detection_photo_targets(self):
        """
        解析检测图片真实坐标。
        推荐格式: 10:x:y;11:x:y;12:x:y;13:x:y
        也兼容 detectPhotoTargetX/Y，长度为4时按 points 顺序映射，长度等于 goals 时按索引映射。
        """
        target_map = {}

        raw = str(self.detect_photo_target_points_param).strip()
        if raw:
            for item in raw.replace("\n", ";").split(";"):
                item = item.strip()
                if not item:
                    continue
                parts = [part.strip() for part in item.replace(":", ",").split(",")
                         if part.strip()]
                if len(parts) < 3:
                    rospy.logwarn("[DETECT_YAW][BAD_TARGET_ITEM] item=%s", item)
                    continue
                try:
                    point = int(parts[0])
                    target_map[point] = (float(parts[1]), float(parts[2]))
                except ValueError:
                    rospy.logwarn("[DETECT_YAW][BAD_TARGET_ITEM] item=%s", item)

        xs = self.parse_float_list(self.detect_photo_target_x_param)
        ys = self.parse_float_list(self.detect_photo_target_y_param)
        if xs or ys:
            if len(xs) != len(ys):
                rospy.logwarn(
                    "[DETECT_YAW][BAD_TARGET_LIST] x_count=%d y_count=%d",
                    len(xs), len(ys))
            count = min(len(xs), len(ys))
            if count == len(points):
                for i in range(count):
                    target_map[points[i]] = (xs[i], ys[i])
            elif "goals" in globals() and count == len(goals):
                for i in range(count):
                    target_map[i] = (xs[i], ys[i])
            elif count > 0:
                rospy.logwarn(
                    "[DETECT_YAW][BAD_TARGET_LIST] count=%d should_be_detect_points=%d or goals=%d",
                    count, len(points), len(goals) if "goals" in globals() else -1)

        if target_map:
            summary = ",".join([
                "%s:(%.3f,%.3f)" % (str(point), xy[0], xy[1])
                for point, xy in sorted(target_map.items())
            ])
            rospy.loginfo("[DETECT_YAW][TARGETS] %s", summary)
        else:
            rospy.loginfo("[DETECT_YAW][TARGETS] none, use fixed detection yaw")
        return target_map

    def has_detection_photo_target(self, point):
        return int(point) in self.detect_photo_target_map

    def detection_yaw_from_xy(self, point, from_x, from_y, fallback_yaw_deg, source):
        if (not self.detect_dynamic_yaw_enabled
                or not self.has_detection_photo_target(point)):
            return fallback_yaw_deg, False

        photo_x, photo_y = self.detect_photo_target_map[int(point)]
        dx = photo_x - from_x
        dy = photo_y - from_y
        dist = np.sqrt(dx * dx + dy * dy)
        if dist < self.detect_dynamic_yaw_min_distance:
            rospy.logwarn(
                "[DETECT_YAW][DYNAMIC_REJECT] point=%s source=%s reason=too_close from=(%.3f,%.3f) photo=(%.3f,%.3f) dist=%.3f fallback=%.1f",
                str(point), source, from_x, from_y, photo_x, photo_y, dist,
                fallback_yaw_deg)
            return fallback_yaw_deg, False

        yaw_deg = self.normalize_angle_deg(np.arctan2(dy, dx) * 180.0 / pi)
        rospy.loginfo(
            "[DETECT_YAW][DYNAMIC] point=%s source=%s from=(%.3f,%.3f) photo=(%.3f,%.3f) yaw=%.1f dist=%.3f fallback=%.1f",
            str(point), source, from_x, from_y, photo_x, photo_y, yaw_deg,
            dist, fallback_yaw_deg)
        return yaw_deg, True

    def detection_yaw_from_current_pose(self, point, fallback_yaw_deg, source):
        if (not self.detect_dynamic_yaw_enabled
                or not self.has_detection_photo_target(point)):
            return fallback_yaw_deg, False

        pose = self.current_map_pose_for_plan()
        if pose is None:
            rospy.logwarn(
                "[DETECT_YAW][CURRENT_POSE_FAILED] point=%s source=%s fallback=%.1f",
                str(point), source, fallback_yaw_deg)
            return fallback_yaw_deg, False
        return self.detection_yaw_from_xy(
            point,
            pose.pose.position.x,
            pose.pose.position.y,
            fallback_yaw_deg,
            source)

    def should_capture_detection_at_prealign(self, point, use_dynamic_yaw=True):
        return (use_dynamic_yaw
                and self.detect_dynamic_yaw_enabled
                and self.detect_dynamic_yaw_capture_at_prealign
                and self.has_detection_photo_target(point))

    def should_run_dynamic_yaw_fallback(self, point):
        return (self.detect_dynamic_yaw_fallback_on_none
                and self.detect_dynamic_yaw_enabled
                and self.has_detection_photo_target(point))

    def select_task_flexible_yaw(self, target):
        original_yaw = target[2]
        if (not self.task_nav_flexible_yaw_enabled
                or len(self.task_nav_flexible_yaw_candidates) <= 0):
            rospy.loginfo(
                "[TASK_NAV][FLEX_YAW_DISABLED] original_yaw=%.1f selected_yaw=%.1f",
                original_yaw, original_yaw
            )
            return original_yaw

        if self.odom_received:
            current_yaw_deg = self.current_yaw * 180.0 / pi
            candidates = sorted(
                self.task_nav_flexible_yaw_candidates,
                key=lambda yaw: (
                    self.yaw_diff_deg(yaw, current_yaw_deg),
                    self.yaw_diff_deg(yaw, original_yaw)
                )
            )
            selected_yaw = candidates[0]
            rospy.loginfo(
                "[TASK_NAV][FLEX_YAW_SELECTED] reason=current_odom current_yaw=%.1f original_yaw=%.1f selected_yaw=%.1f candidates=%s diff_current=%.1f diff_original=%.1f",
                current_yaw_deg,
                original_yaw,
                selected_yaw,
                ",".join(["%.1f" % yaw for yaw in self.task_nav_flexible_yaw_candidates]),
                self.yaw_diff_deg(selected_yaw, current_yaw_deg),
                self.yaw_diff_deg(selected_yaw, original_yaw)
            )
            return selected_yaw

        candidates = sorted(
            self.task_nav_flexible_yaw_candidates,
            key=lambda yaw: self.yaw_diff_deg(yaw, original_yaw)
        )
        selected_yaw = candidates[0]
        rospy.logwarn(
            "[TASK_NAV][FLEX_YAW_SELECTED] reason=odom_unavailable original_yaw=%.1f selected_yaw=%.1f candidates=%s diff_original=%.1f",
            original_yaw,
            selected_yaw,
            ",".join(["%.1f" % yaw for yaw in self.task_nav_flexible_yaw_candidates]),
            self.yaw_diff_deg(selected_yaw, original_yaw)
        )
        return selected_yaw

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

    def nav_reached_by_state_and_distance(self, nav_ok, target, accept_dist=None):
        if accept_dist is None:
            accept_dist = self.task_nav_accept_dist
        nav_dist = self.distance_to_goal_xy(target)
        if nav_dist is None:
            if nav_ok:
                rospy.logwarn("[TASK_TIME][NAV_NO_DISTANCE_ACCEPT_STATE] state_ok=true")
                return True, None
            return False, None
        nav_reached = (
            nav_ok and nav_dist is not None and nav_dist <= accept_dist
        ) or (
            nav_dist is not None and nav_dist <= accept_dist
        )
        if nav_ok and not nav_reached:
            rospy.logwarn(
                "[TASK_TIME][NAV_STATE_DISTANCE_MISMATCH] state_ok=true dist=%s accept=%.3f",
                "%.3f" % nav_dist if nav_dist is not None else "None",
                accept_dist
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

    def get_task_make_plan_client(self):
        if self.task_nav_make_plan_client is not None:
            return self.task_nav_make_plan_client
        try:
            rospy.wait_for_service(
                self.task_nav_path_make_plan_service,
                timeout=self.task_nav_path_make_plan_wait)
            self.task_nav_make_plan_client = rospy.ServiceProxy(
                self.task_nav_path_make_plan_service, GetPlan)
            return self.task_nav_make_plan_client
        except Exception as e:
            rospy.logwarn(
                "[TASK_NAV][PATH_FILTER_SERVICE_UNAVAILABLE] service=%s wait=%.2fs error=%s",
                self.task_nav_path_make_plan_service,
                self.task_nav_path_make_plan_wait,
                str(e)
            )
            return None

    def current_map_pose_for_plan(self):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        try:
            self.tf_listener.waitForTransform(
                "map", "base_footprint", rospy.Time(0), rospy.Duration(0.1))
            trans, rot = self.tf_listener.lookupTransform(
                "map", "base_footprint", rospy.Time(0))
            pose.pose.position.x = trans[0]
            pose.pose.position.y = trans[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = rot[0]
            pose.pose.orientation.y = rot[1]
            pose.pose.orientation.z = rot[2]
            pose.pose.orientation.w = rot[3]
            return pose
        except Exception as e:
            rospy.logwarn_throttle(
                2.0,
                "[TASK_NAV][PATH_FILTER_TF_FALLBACK] map->base_footprint unavailable: %s",
                str(e)
            )

        if self.last_move_base_feedback is not None:
            pose.pose = self.last_move_base_feedback.base_position.pose
            return pose

        if self.current_odom_pose is not None:
            pose.pose = self.current_odom_pose
            return pose

        return None

    def task_goal_pose_for_plan(self, nav_target):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = nav_target[0]
        pose.pose.position.y = nav_target[1]
        pose.pose.position.z = 0.0
        q = quaternion_from_euler(0.0, 0.0, nav_target[2] / 180.0 * pi)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose

    def evaluate_task_plan_quality(self, mode, nav_target):
        if not self.task_nav_path_filter_enabled:
            return True, "path_filter_disabled", {
                "max_angle_deg": 0.0,
                "transition_goal": None,
                "plan_points": 0
            }

        if not self.wait_for_make_plan_idle("task:%s" % mode):
            return True, "path_filter_move_base_active_allow", {
                "max_angle_deg": 0.0,
                "transition_goal": None,
                "plan_points": 0
            }

        client = self.get_task_make_plan_client()
        if client is None:
            return True, "path_filter_no_service_allow", {
                "max_angle_deg": 0.0,
                "transition_goal": None,
                "plan_points": 0
            }

        start = self.current_map_pose_for_plan()
        if start is None:
            return True, "path_filter_no_start_pose_allow", {
                "max_angle_deg": 0.0,
                "transition_goal": None,
                "plan_points": 0
            }

        request = GetPlanRequest()
        request.start = start
        request.goal = self.task_goal_pose_for_plan(nav_target)
        request.tolerance = 0.0
        try:
            response = client(request)
        except Exception as e:
            self.task_nav_make_plan_client = None
            rospy.logwarn(
                "[TASK_NAV][PATH_FILTER_MAKE_PLAN_FAILED] mode=%s target=(%.3f,%.3f,%.1f) error=%s",
                mode, nav_target[0], nav_target[1], nav_target[2], str(e)
            )
            return True, "path_filter_call_failed_allow", {
                "max_angle_deg": 0.0,
                "transition_goal": None,
                "plan_points": 0
            }

        poses = response.plan.poses
        if len(poses) < 2:
            return False, "path_filter_no_plan", {
                "max_angle_deg": 999.0,
                "transition_goal": None,
                "plan_points": len(poses)
            }

        ok, info = self.analyze_task_plan_sharp_turns(poses)
        info["plan_points"] = len(poses)
        if ok:
            return True, "path_ok max_angle=%.1f points=%d" % (
                info.get("max_angle_deg", 0.0), len(poses)), info
        transition_goal = info.get("transition_goal")
        transition_text = "none"
        if transition_goal is not None:
            transition_text = "(%.3f,%.3f,%.1f)" % (
                transition_goal[0], transition_goal[1], transition_goal[2])
        return False, "path_sharp_turn angle=%.1f index=%s transition=%s points=%d" % (
            info.get("max_angle_deg", 0.0),
            str(info.get("sharp_index", None)),
            transition_text,
            len(poses)
        ), info

    def analyze_task_plan_sharp_turns(self, poses):
        sparse = []
        path_s = 0.0
        last_x = poses[0].pose.position.x
        last_y = poses[0].pose.position.y
        sparse.append((0, last_x, last_y, 0.0))
        for i in range(1, len(poses)):
            x = poses[i].pose.position.x
            y = poses[i].pose.position.y
            prev_x = poses[i - 1].pose.position.x
            prev_y = poses[i - 1].pose.position.y
            path_s += np.sqrt((x - prev_x) ** 2 + (y - prev_y) ** 2)
            if (np.sqrt((x - last_x) ** 2 + (y - last_y) ** 2)
                    >= self.task_nav_path_sparse_distance
                    or i == len(poses) - 1):
                sparse.append((i, x, y, path_s))
                last_x = x
                last_y = y

        if len(sparse) < 3:
            return True, {
                "max_angle_deg": 0.0,
                "transition_goal": None,
                "sharp_index": None
            }

        total_s = sparse[-1][3]
        threshold_rad = self.task_nav_path_sharp_turn_threshold_deg / 180.0 * pi
        max_angle = 0.0
        max_index = None
        for i in range(1, len(sparse) - 1):
            if (sparse[i][3] < self.task_nav_path_min_endpoint_distance
                    or total_s - sparse[i][3] < self.task_nav_path_min_endpoint_distance):
                continue
            in_x = sparse[i][1] - sparse[i - 1][1]
            in_y = sparse[i][2] - sparse[i - 1][2]
            out_x = sparse[i + 1][1] - sparse[i][1]
            out_y = sparse[i + 1][2] - sparse[i][2]
            in_len = np.sqrt(in_x * in_x + in_y * in_y)
            out_len = np.sqrt(out_x * out_x + out_y * out_y)
            if in_len < 1e-6 or out_len < 1e-6:
                continue
            dot = (in_x * out_x + in_y * out_y) / (in_len * out_len)
            dot = max(-1.0, min(1.0, dot))
            angle = np.arccos(dot)
            if angle > max_angle:
                max_angle = angle
                max_index = sparse[i][0]

        max_angle_deg = max_angle * 180.0 / pi
        if max_index is None or max_angle < threshold_rad:
            return True, {
                "max_angle_deg": max_angle_deg,
                "transition_goal": None,
                "sharp_index": max_index
            }

        transition_goal = self.transition_goal_before_path_index(
            poses, max_index, self.task_nav_transition_distance_before_corner)
        rospy.logwarn(
            "[TASK_NAV][PATH_SHARP_TURN] angle=%.1fdeg index=%s transition=%s threshold=%.1fdeg",
            max_angle_deg,
            str(max_index),
            str(transition_goal),
            self.task_nav_path_sharp_turn_threshold_deg
        )
        return False, {
            "max_angle_deg": max_angle_deg,
            "transition_goal": transition_goal,
            "sharp_index": max_index
        }

    def transition_goal_before_path_index(self, poses, sharp_index, distance_before):
        if sharp_index is None or sharp_index <= 0:
            return None
        remaining = max(0.0, distance_before)
        for i in range(sharp_index, 0, -1):
            x0 = poses[i].pose.position.x
            y0 = poses[i].pose.position.y
            x1 = poses[i - 1].pose.position.x
            y1 = poses[i - 1].pose.position.y
            seg_len = np.sqrt((x0 - x1) ** 2 + (y0 - y1) ** 2)
            if seg_len < 1e-6:
                continue
            if remaining <= seg_len:
                ratio = remaining / seg_len
                x = x0 + (x1 - x0) * ratio
                y = y0 + (y1 - y0) * ratio
                yaw = np.arctan2(y0 - y1, x0 - x1) * 180.0 / pi
                return [x, y, yaw]
            remaining -= seg_len
        yaw = self.current_yaw * 180.0 / pi
        return [poses[0].pose.position.x, poses[0].pose.position.y, yaw]

    def is_transition_goal_clear(self, transition_goal, costmap):
        if transition_goal is None:
            return False, "no_transition_goal", (999, 999.0, 999)
        if costmap is None:
            return True, "no_costmap_allow", (0, 0.0, 0)
        cost, detail = self.costmap_cost_at(costmap, transition_goal[0], transition_goal[1])
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
            costmap, transition_goal[0], transition_goal[1],
            self.task_nav_approach_score_radius)
        if score is None:
            score = (cost, float(cost), 0)
        return True, "cost=%d score=max:%d avg:%.1f unk:%d" % (
            cost, score[0], score[1], score[2]), score

    def evaluate_detection_prealign_goal(self, mode, nav_target, costmap=None):
        if costmap is not None:
            cost, detail = self.costmap_cost_at(costmap, nav_target[0], nav_target[1])
            if cost is None:
                return False, detail
            if cost < 0:
                if self.task_nav_approach_reject_unknown:
                    return False, "unknown"
            elif cost > self.task_nav_approach_cost_threshold:
                return False, "cost=%d>threshold=%d" % (
                    cost, self.task_nav_approach_cost_threshold)

        if not self.wait_for_make_plan_idle("detect:%s" % mode):
            return False, "move_base_active_for_make_plan"

        client = self.get_task_make_plan_client()
        if client is None:
            return False, "make_plan_service_unavailable"

        start = self.current_map_pose_for_plan()
        if start is None:
            return False, "no_start_pose"

        request = GetPlanRequest()
        request.start = start
        request.goal = self.task_goal_pose_for_plan(nav_target)
        request.tolerance = 0.0
        try:
            response = client(request)
        except Exception as e:
            self.task_nav_make_plan_client = None
            return False, "make_plan_failed:%s" % str(e)

        poses = response.plan.poses
        if len(poses) < 2:
            return False, "no_plan"

        path_ok, path_info = self.analyze_task_plan_sharp_turns(poses)
        if not path_ok:
            return False, "sharp_turn angle=%.1f index=%s" % (
                path_info.get("max_angle_deg", 0.0),
                str(path_info.get("sharp_index", None))
            )
        return True, "path_ok max_angle=%.1f points=%d" % (
            path_info.get("max_angle_deg", 0.0), len(poses))

    def evaluate_task_approach_goal(self, mode, nav_target, costmap=None, costmap_checked=False):
        if not self.task_nav_approach_filter_costmap or mode == "target":
            path_clear, path_reason, path_info = self.evaluate_task_plan_quality(mode, nav_target)
            return path_clear, "filter_disabled_or_target %s" % path_reason, (0, 0.0, 0), path_info

        if not costmap_checked:
            costmap = self.get_global_costmap_for_approach()
        if costmap is None:
            path_clear, path_reason, path_info = self.evaluate_task_plan_quality(mode, nav_target)
            return path_clear, "no_costmap_allow %s" % path_reason, (0, 0.0, 0), path_info

        cost, detail = self.costmap_cost_at(costmap, nav_target[0], nav_target[1])
        if cost is None:
            return False, detail, (999, 999.0, 999), None
        if cost < 0:
            if self.task_nav_approach_reject_unknown:
                return False, "unknown", (999, 999.0, 999), None
            path_clear, path_reason, path_info = self.evaluate_task_plan_quality(mode, nav_target)
            return path_clear, "unknown_allowed %s" % path_reason, (100, 100.0, 1), path_info
        if cost > self.task_nav_approach_cost_threshold:
            return False, "cost=%d>threshold=%d" % (
                cost, self.task_nav_approach_cost_threshold), (cost, float(cost), 0), None

        score, detail = self.costmap_score_near(
            costmap, nav_target[0], nav_target[1], self.task_nav_approach_score_radius)
        if score is None:
            score = (cost, float(cost), 0)
            score_text = "score_unavailable=%s" % detail
        else:
            score_text = "score=max:%d avg:%.1f unk:%d radius:%.2f" % (
                score[0], score[1], score[2], self.task_nav_approach_score_radius)
        path_clear, path_reason, path_info = self.evaluate_task_plan_quality(mode, nav_target)
        return path_clear, "cost=%d %s %s" % (cost, score_text, path_reason), score, path_info

    def is_task_approach_goal_clear(self, mode, nav_target):
        clear, reason, _, _ = self.evaluate_task_approach_goal(mode, nav_target)
        return clear, reason

    def select_task_approach_goal(self, target, skipped_modes=None):
        if skipped_modes is None:
            skipped_modes = set()
        candidates = self.make_task_approach_goals(target)
        if not self.task_nav_use_approach_goal and "target" not in skipped_modes:
            return "target", list(target), None

        target_fallback = None
        clear_candidates = []
        transition_candidates = []
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
            clear, reason, score, path_info = self.evaluate_task_approach_goal(
                mode, nav_target, costmap=costmap, costmap_checked=costmap_checked)
            rospy.loginfo(
                "[TASK_NAV][APPROACH_CHECK] mode=%s nav_target=(%.3f,%.3f,%.1f) clear=%s reason=%s",
                mode, nav_target[0], nav_target[1], nav_target[2],
                str(clear), reason
            )
            if clear:
                clear_candidates.append((score, mode, nav_target, reason))
            elif (self.task_nav_transition_enabled
                  and path_info is not None
                  and path_info.get("transition_goal") is not None):
                transition_goal = path_info.get("transition_goal")
                transition_clear, transition_reason, transition_score = self.is_transition_goal_clear(
                    transition_goal, costmap)
                rospy.logwarn(
                    "[TASK_NAV][APPROACH_TRANSITION_CHECK] mode=%s transition=(%.3f,%.3f,%.1f) clear=%s reason=%s original_reason=%s",
                    mode,
                    transition_goal[0], transition_goal[1], transition_goal[2],
                    str(transition_clear), transition_reason, reason
                )
                if transition_clear:
                    transition_candidates.append((
                        transition_score,
                        path_info.get("max_angle_deg", 0.0),
                        mode,
                        nav_target,
                        transition_goal,
                        reason
                    ))

        if clear_candidates:
            clear_candidates.sort(key=lambda item: (item[0][2], item[0][0], item[0][1]))
            score, mode, nav_target, reason = clear_candidates[0]
            rospy.loginfo(
                "[TASK_NAV][APPROACH_SELECTED] mode=%s nav_target=(%.3f,%.3f,%.1f) score=max:%d avg:%.1f unk:%d reason=%s",
                mode, nav_target[0], nav_target[1], nav_target[2],
                score[0], score[1], score[2], reason
            )
            return mode, nav_target, None

        if transition_candidates:
            transition_candidates.sort(key=lambda item: (item[0][2], item[0][0], item[0][1], item[1]))
            score, angle_deg, mode, nav_target, transition_goal, reason = transition_candidates[0]
            rospy.logwarn(
                "[TASK_NAV][APPROACH_TRANSITION_SELECTED] mode=%s transition=(%.3f,%.3f,%.1f) followup=(%.3f,%.3f,%.1f) angle=%.1f score=max:%d avg:%.1f unk:%d reason=%s",
                mode,
                transition_goal[0], transition_goal[1], transition_goal[2],
                nav_target[0], nav_target[1], nav_target[2],
                angle_deg,
                score[0], score[1], score[2], reason
            )
            return "transition:%s" % mode, transition_goal, (mode, nav_target)

        rospy.logwarn("[TASK_NAV][APPROACH_NO_CLEAR] target=%s", str(target))
        if self.task_nav_approach_fallback_to_target and target_fallback is not None:
            rospy.logwarn("[TASK_NAV][APPROACH_FALLBACK_TARGET] nav_target=%s", str(target_fallback[1]))
            return target_fallback[0], target_fallback[1], None
        return None, None, None

    def goto_task_approach(self, target, timeout, label, skipped_modes=None):
        mode, nav_target, followup = self.select_task_approach_goal(target, skipped_modes)
        if nav_target is None:
            return False, False, None, None, None
        rospy.loginfo("[TASK_NAV][TRY_%s] mode=%s nav_target=%s", label, mode, nav_target)
        if mode.startswith("transition:"):
            accept_dist = self.task_nav_transition_accept_dist
        else:
            accept_dist = self.task_nav_accept_dist if mode == "target" else self.task_nav_approach_accept_dist
        nav_ok = self.goto_task_nav_goal(
            nav_target,
            timeout=timeout,
            label=label,
            mode=mode,
            position_accept_dist=accept_dist
        )
        nav_reached, approach_dist = self.nav_reached_by_state_and_distance(nav_ok, nav_target, accept_dist)

        if followup is not None:
            followup_mode, followup_target = followup
            followup_accept = self.task_nav_approach_accept_dist
            if nav_reached:
                rospy.logwarn(
                    "[TASK_NAV][FALLBACK_TRANSITION_GOAL] transition_mode=%s followup_mode=%s followup_target=(%.3f,%.3f,%.1f)",
                    mode, followup_mode,
                    followup_target[0], followup_target[1], followup_target[2]
                )
                nav_ok = self.goto_task_nav_goal(
                    followup_target,
                    timeout=self.task_nav_retry_timeout,
                    label=label + "_FOLLOWUP",
                    mode=followup_mode,
                    position_accept_dist=followup_accept,
                    slow_mode=self.task_nav_teb_slow_fallback_enabled
                )
                nav_reached, approach_dist = self.nav_reached_by_state_and_distance(
                    nav_ok, followup_target, followup_accept)
                mode = followup_mode
                nav_target = followup_target
                accept_dist = followup_accept
            elif self.task_nav_teb_slow_fallback_enabled:
                rospy.logwarn(
                    "[TASK_NAV][FALLBACK_TRANSITION_FAILED_SLOW_TEB] transition_mode=%s followup_mode=%s followup_target=(%.3f,%.3f,%.1f)",
                    mode, followup_mode,
                    followup_target[0], followup_target[1], followup_target[2]
                )
                nav_ok = self.goto_task_nav_goal(
                    followup_target,
                    timeout=self.task_nav_retry_timeout,
                    label=label + "_SLOW",
                    mode="slow:%s" % followup_mode,
                    position_accept_dist=followup_accept,
                    slow_mode=True
                )
                nav_reached, approach_dist = self.nav_reached_by_state_and_distance(
                    nav_ok, followup_target, followup_accept)
                mode = followup_mode
                nav_target = followup_target
                accept_dist = followup_accept

        nav_dist = self.distance_to_goal_xy(target)
        selected_yaw_deg = None
        if nav_reached:
            selected_yaw_deg = self.select_task_flexible_yaw(target)
            yaw_err = self.yaw_error_to_goal(target)
            rospy.loginfo(
                "[TASK_NAV][REACHED_BY_POSITION] mode=%s approach_dist=%s target_dist=%s accept=%.3f yaw_err=%.3f original_yaw=%.1f selected_yaw=%.1f",
                mode,
                "%.3f" % approach_dist if approach_dist is not None else "None",
                "%.3f" % nav_dist if nav_dist is not None else "None",
                accept_dist,
                yaw_err,
                target[2],
                selected_yaw_deg
            )
        if mode != "target" and nav_ok and not nav_reached:
            rospy.logwarn(
                "[TASK_NAV][APPROACH_STATE_DISTANCE_MISMATCH] mode=%s approach_dist=%s accept=%.3f",
                mode,
                "%.3f" % approach_dist if approach_dist is not None else "None",
                self.task_nav_approach_accept_dist
            )
        rospy.loginfo(
            "[TASK_NAV][TRY_%s_DONE] mode=%s ok=%s target_dist=%s approach_dist=%s reached=%s state=%s selected_yaw=%s",
            label, mode, str(nav_ok),
            "%.3f" % nav_dist if nav_dist is not None else "None",
            "%.3f" % approach_dist if approach_dist is not None else "None",
            str(nav_reached), str(self.last_move_base_state),
            "%.1f" % selected_yaw_deg if selected_yaw_deg is not None else "None"
        )
        return nav_ok, nav_reached, nav_dist, mode, selected_yaw_deg

    def mark_failed_approach_mode(self, idx, task_id, mode, failed_approach_modes):
        if mode is None:
            return
        failed_mode = str(mode)
        if failed_mode.startswith("transition:"):
            failed_mode = failed_mode.split(":", 1)[1]
        elif failed_mode.startswith("slow:"):
            failed_mode = failed_mode.split(":", 1)[1]
        failed_approach_modes.add(failed_mode)
        rospy.logwarn(
            "[TASK_NAV][APPROACH_MARK_FAILED] idx=%d task_id=%d mode=%s failed_modes=%s",
            idx + 1, task_id, failed_mode,
            ",".join(sorted(failed_approach_modes))
        )

    def navigate_task_with_all_approaches(self, idx, task_id, target, last_parking, last_task_id):
        failed_approach_modes = set()
        nav_ok = False
        nav_reached = False
        nav_dist = None
        nav_mode = None
        selected_yaw_deg = None
        escaped_after_abort = False
        attempt = 0

        while not rospy.is_shutdown():
            attempt += 1
            label = "MAIN" if attempt == 1 else "RETRY_%d" % (attempt - 1)
            timeout = self.task_nav_timeout if attempt == 1 else self.task_nav_retry_timeout
            nav_start_time = rospy.Time.now()
            nav_ok, nav_reached, nav_dist, nav_mode, selected_yaw_deg = self.goto_task_approach(
                target, timeout, label, failed_approach_modes)
            rospy.loginfo(
                "[TASK_TIME][NAV_ATTEMPT] idx=%d task_id=%d label=%s dt=%.2fs ok=%s target_dist=%s reached=%s state=%s mode=%s selected_yaw=%s",
                idx + 1, task_id, label,
                (rospy.Time.now() - nav_start_time).to_sec(),
                str(nav_ok),
                "%.3f" % nav_dist if nav_dist is not None else "None",
                str(nav_reached), str(self.last_move_base_state), str(nav_mode),
                "%.1f" % selected_yaw_deg if selected_yaw_deg is not None else "None"
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

        return nav_ok, nav_reached, nav_dist, nav_mode, selected_yaw_deg

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
    def final_laser_direction_config(self, direction):
        direction = str(direction).strip().lower()
        configs = {
            "left": (np.pi / 2.0, "linear.y", 1.0),
            "right": (-np.pi / 2.0, "linear.y", -1.0),
            "front": (0.0, "linear.x", 1.0),
            "back": (np.pi, "linear.x", -1.0),
        }
        return configs.get(direction)

    def apply_final_axis_cmd(self, cmd, axis, value):
        if axis == "linear.x":
            cmd.linear.x += value
        elif axis == "linear.y":
            cmd.linear.y += value

    def run_final_axis_motion(self, label, axis, sign, speed, duration):
        if duration <= 0.0 or speed <= 0.0:
            rospy.loginfo(
                "[FINAL][FORCE_%s][SKIP] duration=%.2f speed=%.3f",
                label, duration, speed)
            return False

        rospy.logwarn(
            "[FINAL][FORCE_%s][START] axis=%s sign=%.1f speed=%.3f duration=%.2fs",
            label, axis, sign, speed, duration)
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed >= duration:
                break
            cmd = Twist()
            self.apply_final_axis_cmd(cmd, axis, sign * abs(speed))
            self.pub.publish(cmd)
            rate.sleep()
        self.pub.publish(Twist())
        rospy.logwarn("[FINAL][FORCE_%s][DONE]", label)
        return True

    def adjust_final_axis(self, label, angle, axis, sign, target, timeout, adjust_yaw):
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        self.is_adjusting = True
        rospy.loginfo(
            "[FINAL][ADJUST_%s][START] axis=%s target=%.3f timeout=%.1fs yaw=%s",
            label, axis, target, timeout, str(adjust_yaw))

        while not rospy.is_shutdown() and self.is_adjusting:
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed > timeout:
                rospy.logwarn("[FINAL][ADJUST_%s][TIMEOUT] elapsed=%.2f", label, elapsed)
                self.stop_movement()
                return False

            distance = self.get_range_at_angle(angle)
            if not np.isfinite(distance):
                rospy.logwarn_throttle(
                    1.0,
                    "[FINAL][ADJUST_%s][WAIT_LASER] invalid_distance",
                    label)
                self.pub.publish(Twist())
                rate.sleep()
                continue

            distance_error = distance - target
            yaw_error = self.normalize_angle(self.target_yaw - self.current_yaw)
            distance_ok = abs(distance_error) < self.position_tolerance
            yaw_ok = (not adjust_yaw) or abs(yaw_error) < self.yaw_tolerance

            if distance_ok and yaw_ok:
                rospy.loginfo(
                    "[FINAL][ADJUST_%s][OK] distance=%.3f target=%.3f err=%.3f yaw_err=%.3f",
                    label, distance, target, distance_error, yaw_error)
                self.stop_movement()
                return True

            cmd = Twist()
            if not distance_ok:
                self.apply_final_axis_cmd(
                    cmd,
                    axis,
                    self.kp_linear * sign * distance_error)
            if adjust_yaw and not yaw_ok:
                cmd.angular.z = self.kp_angular * yaw_error
            self.pub.publish(cmd)
            rospy.loginfo_throttle(
                0.5,
                "[FINAL][ADJUST_%s] distance=%.3f target=%.3f err=%.3f yaw_err=%.3f cmd=(%.3f,%.3f,%.3f)",
                label, distance, target, distance_error, yaw_error,
                cmd.linear.x, cmd.linear.y, cmd.angular.z)
            rate.sleep()

        self.stop_movement()
        return False

    def adjust_position(self, side_target, back_target):
        """
        执行位置校准
        :param side_target: 侧方目标距离 (米)
        :param back_target: 深度方向目标距离 (米)
        :return: 是否完成校准
        """
        if self.scan_data is None:
            rospy.logwarn("无激光数据，无法校准!")
            return False

        side_config = self.final_laser_direction_config(self.final_side_laser_direction)
        depth_config = self.final_laser_direction_config(self.final_depth_laser_direction)
        if side_config is None or depth_config is None:
            rospy.logerr(
                "终点激光方向配置无效: side=%s depth=%s，应为 left/right/front/back",
                str(self.final_side_laser_direction),
                str(self.final_depth_laser_direction)
            )
            return False
        side_angle, side_axis, side_sign = side_config
        depth_angle, depth_axis, depth_sign = depth_config
        rospy.loginfo(
            "[FINAL][ADJUST_POSITION][CONFIG] side=%s target=%.3f depth=%s target=%.3f",
            str(self.final_side_laser_direction), side_target,
            str(self.final_depth_laser_direction), back_target
        )

        side_ok = self.adjust_final_axis(
            "SIDE",
            side_angle,
            side_axis,
            side_sign,
            side_target,
            self.final_adjust_timeout,
            True)

        if not side_ok:
            rospy.logwarn(
                "[FINAL][ADJUST_POSITION][SIDE_FAILED] force_side_on_fail=%s",
                str(self.final_adjust_force_side_on_fail))
            if self.final_adjust_force_side_on_fail:
                self.run_final_axis_motion(
                    "SIDE",
                    side_axis,
                    side_sign,
                    self.final_force_side_speed,
                    self.final_force_side_duration)
                if self.final_force_depth_after_side:
                    self.run_final_axis_motion(
                        "DEPTH",
                        depth_axis,
                        depth_sign,
                        self.final_force_depth_speed,
                        self.final_force_depth_duration)
            return False

        depth_ok = self.adjust_final_axis(
            "DEPTH",
            depth_angle,
            depth_axis,
            depth_sign,
            back_target,
            self.final_adjust_timeout,
            False)
        return side_ok and depth_ok

    def stop_movement(self):
        """停止机器人运动"""
        cmd = Twist()
        self.pub.publish(cmd)
        self.is_adjusting = False

    def make_offset_goal(self, target, mode, distance):
        """按目标yaw的相对方向生成偏移位姿。"""
        yaw_rad = target[2] / 180.0 * pi
        forward_x = np.cos(yaw_rad)
        forward_y = np.sin(yaw_rad)
        left_x = -np.sin(yaw_rad)
        left_y = np.cos(yaw_rad)
        mode = str(mode).strip().lower()
        diag = 1.0 / np.sqrt(2.0)
        mode_vectors = {
            "front": (forward_x, forward_y),
            "back": (-forward_x, -forward_y),
            "left": (left_x, left_y),
            "right": (-left_x, -left_y),
            "front_left": ((forward_x + left_x) * diag, (forward_y + left_y) * diag),
            "front_right": ((forward_x - left_x) * diag, (forward_y - left_y) * diag),
            "back_left": ((-forward_x + left_x) * diag, (-forward_y + left_y) * diag),
            "back_right": ((-forward_x - left_x) * diag, (-forward_y - left_y) * diag),
            "left_front": ((forward_x + left_x) * diag, (forward_y + left_y) * diag),
            "right_front": ((forward_x - left_x) * diag, (forward_y - left_y) * diag),
            "left_back": ((-forward_x + left_x) * diag, (-forward_y + left_y) * diag),
            "right_back": ((-forward_x - left_x) * diag, (-forward_y - left_y) * diag),
        }
        offset_x, offset_y = mode_vectors.get(mode, mode_vectors["back"])

        return [
            target[0] + offset_x * distance,
            target[1] + offset_y * distance,
            target[2]
        ]

    def make_detection_prealign_goal(self, target, point=None, use_dynamic_yaw=False):
        """按配置方向生成检测点预对准位姿"""
        goal = self.make_offset_goal(
            target,
            self.detect_prealign_mode,
            self.detect_prealign_distance
        )
        if point is not None and self.should_capture_detection_at_prealign(point, use_dynamic_yaw):
            goal[2], _ = self.detection_yaw_from_xy(
                point, goal[0], goal[1], goal[2], "prealign_goal")
        return goal

    def make_detection_prealign_candidates(self, point, target, use_dynamic_yaw=False):
        candidates = []
        seen = set()

        def add_candidate(mode, distance):
            mode = str(mode).strip().lower()
            try:
                distance = float(distance)
            except Exception:
                return
            if distance <= 0.0:
                return
            goal = self.make_offset_goal(target, mode, distance)
            if self.should_capture_detection_at_prealign(point, use_dynamic_yaw):
                goal[2], _ = self.detection_yaw_from_xy(
                    point, goal[0], goal[1], goal[2], "candidate:%s" % mode)
            key = (round(goal[0], 3), round(goal[1], 3), round(goal[2], 1))
            if key in seen:
                return
            seen.add(key)
            candidates.append((mode, goal, distance))

        add_candidate(self.detect_prealign_mode, self.detect_prealign_distance)
        if self.detect_nav_retry_enabled:
            for mode in str(self.detect_nav_retry_modes).split(","):
                if mode.strip():
                    add_candidate(mode, self.detect_nav_retry_distance)
        return candidates

    def make_final_prealign_goal(self, target):
        """按配置方向生成终点预对准位姿"""
        return self.make_offset_goal(
            target,
            self.final_prealign_mode,
            self.final_prealign_distance
        )

    def get_locked_approach_velocity(self, speed, mode=None):
        """根据预对准方向生成保持当前yaw时的base_link速度"""
        mode = str(mode if mode is not None else self.detect_prealign_mode).strip().lower()
        diag = 1.0 / np.sqrt(2.0)
        if mode == "front":
            return -speed, 0.0
        if mode == "left":
            return 0.0, -speed
        if mode == "right":
            return 0.0, speed
        if mode in ["front_left", "left_front"]:
            return -speed * diag, -speed * diag
        if mode in ["front_right", "right_front"]:
            return -speed * diag, speed * diag
        if mode in ["back_left", "left_back"]:
            return speed * diag, -speed * diag
        if mode in ["back_right", "right_back"]:
            return speed * diag, speed * diag
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

    def locked_approach_detection_point(self, yaw_deg, mode=None, distance=None):
        """
        从预对准点到拍照点的短距离直行段。
        不再交给move_base，避免TEB在最后0.6m重新优化yaw。
        """
        mode = str(mode if mode is not None else self.detect_prealign_mode).strip().lower()
        if distance is None:
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
        cmd_x, cmd_y = self.get_locked_approach_velocity(speed, mode=mode)
        start_time = rospy.Time.now()
        rate = rospy.Rate(20)

        rospy.loginfo("锁yaw靠近拍照点: mode=%s distance=%.3fm speed=%.3fm/s vx=%.3f vy=%.3f time=%.2fs target=%.1fdeg" %
                      (mode, distance, speed, cmd_x, cmd_y, travel_time, yaw_deg))
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

    def is_move_base_active_state(self, state):
        return state in [
            GoalStatus.PENDING,
            GoalStatus.ACTIVE,
            GoalStatus.PREEMPTING,
            GoalStatus.RECALLING
        ]

    def wait_for_move_base_inactive(self, reason="", timeout=None):
        if timeout is None:
            timeout = self.move_base_cancel_wait
        timeout = max(0.0, float(timeout))
        start_time = rospy.Time.now()
        rate = rospy.Rate(20)
        last_state = None
        while not rospy.is_shutdown():
            state = self.move_base.get_state()
            last_state = state
            if not self.is_move_base_active_state(state):
                self.last_move_base_state = state
                rospy.loginfo(
                    "[MOVE_BASE][WAIT_INACTIVE] reason=%s ok=True state=%s elapsed=%.2fs",
                    reason, str(state), (rospy.Time.now() - start_time).to_sec()
                )
                return True
            if (rospy.Time.now() - start_time).to_sec() >= timeout:
                rospy.logwarn(
                    "[MOVE_BASE][WAIT_INACTIVE] reason=%s ok=False state=%s timeout=%.2fs",
                    reason, str(last_state), timeout
                )
                return False
            rate.sleep()
        return False

    def cancel_move_base_goal(self, reason="", final_state=GoalStatus.PREEMPTED,
                              wait_timeout=None):
        self.move_base.cancel_goal()
        self.wait_for_move_base_inactive(reason, wait_timeout)
        self.last_move_base_state = final_state

    def wait_for_make_plan_idle(self, reason):
        state = self.move_base.get_state()
        if not self.is_move_base_active_state(state):
            return True
        return self.wait_for_move_base_inactive(
            "make_plan:%s" % reason,
            self.move_base_make_plan_idle_wait)

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
            self.cancel_move_base_goal("goto_timeout", GoalStatus.PREEMPTED)
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

    def goto_task_nav_goal(self, p, timeout=60, label="", mode="", position_accept_dist=None,
                           slow_mode=False):
        """
        任务点导航专用：在常规 timeout 外，检测规划失败和目标距离长时间没有变近。
        这样目标点在墙里/局部规划卡住时，可以更快切换到 approach 或下一个 approach。
        """
        rospy.loginfo(
            "[TASK_NAV][GOTO_START] label=%s mode=%s target=%s timeout=%.1fs position_accept=%s slow_mode=%s no_progress=%s no_progress_timeout=%.1fs min_delta=%.3f",
            label, mode, str(p), timeout,
            "%.3f" % position_accept_dist if position_accept_dist is not None else "None",
            str(slow_mode),
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

        slow_applied = False
        if slow_mode:
            slow_applied = self.set_teb_slow_mode(True, "%s:%s" % (label, mode))

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
                    self.cancel_move_base_goal(
                        "plan_fail_cancel:%s:%s" % (label, mode),
                        GoalStatus.PREEMPTED)
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
                    self.cancel_move_base_goal(
                        "timeout_cancel:%s:%s" % (label, mode),
                        GoalStatus.PREEMPTED)
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

                dist = self.distance_to_goal_xy(p)
                if (position_accept_dist is not None
                        and dist is not None
                        and dist <= position_accept_dist):
                    self.cancel_move_base_goal(
                        "position_accept:%s:%s" % (label, mode),
                        GoalStatus.SUCCEEDED)
                    rospy.loginfo(
                        "[TASK_NAV][POSITION_ACCEPT] label=%s mode=%s dist=%.3f accept=%.3f target=%s",
                        label, mode, dist, position_accept_dist, str(p)
                    )
                    return True

                if state in [GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.PREEMPTED, GoalStatus.RECALLED]:
                    self.last_move_base_state = state
                    rospy.logwarn("[TASK_NAV][GOTO_FAILED_STATE] label=%s mode=%s state=%s dist=%s accept=%s",
                                  label, mode, state,
                                  "%.3f" % dist if dist is not None else "None",
                                  "%.3f" % position_accept_dist if position_accept_dist is not None else "None")
                    return False

                if self.task_nav_no_progress_enabled and dist is not None:
                    if best_dist is None or dist < best_dist - self.task_nav_no_progress_min_delta:
                        best_dist = dist
                        last_progress_time = rospy.Time.now()
                    elif (rospy.Time.now() - last_progress_time).to_sec() > self.task_nav_no_progress_timeout:
                        self.cancel_move_base_goal(
                            "no_progress_cancel:%s:%s" % (label, mode),
                            GoalStatus.PREEMPTED)
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
            if slow_applied:
                self.set_teb_slow_mode(False, "%s:%s" % (label, mode))

        self.cancel_move_base_goal("goto_task_nav_goal_exit:%s:%s" % (label, mode),
                                   GoalStatus.PREEMPTED)
        return False

    def goto_detection_point(self, point, use_dynamic_yaw=False):
        """检测点导航：先用同yaw预对准，再进入原拍照点并短闭环修正yaw"""
        target = goals[point]
        prealign_ok = True
        selected_mode = self.detect_prealign_mode
        selected_distance = self.detect_prealign_distance
        capture_at_current_pose = False
        rospy.loginfo(
            "[DETECT_NAV][STRATEGY] point=%s dynamic_yaw=%s fixed_yaw=%.1f",
            str(point), str(use_dynamic_yaw), target[2]
        )
        if self.detect_prealign_enabled and self.detect_prealign_distance > 0.0:
            prealign_ok = False
            candidates = self.make_detection_prealign_candidates(
                point, target, use_dynamic_yaw=use_dynamic_yaw)
            costmap = self.get_global_costmap_for_approach()
            for attempt, (mode, prealign_goal, distance) in enumerate(candidates):
                clear, reason = self.evaluate_detection_prealign_goal(
                    mode, prealign_goal, costmap=costmap)
                rospy.loginfo(
                    "[DETECT_NAV][CANDIDATE] point=%s attempt=%d mode=%s distance=%.3f goal=(%.3f,%.3f,%.1f) clear=%s reason=%s",
                    str(point), attempt + 1, mode, distance,
                    prealign_goal[0], prealign_goal[1], prealign_goal[2],
                    str(clear), reason
                )
                if not clear:
                    continue

                timeout = self.detect_prealign_timeout if attempt == 0 else self.detect_nav_retry_timeout
                nav_ok = self.goto_task_nav_goal(
                    prealign_goal,
                    timeout=timeout,
                    label="DETECT_%s_%d" % (str(point), attempt + 1),
                    mode="detect:%s" % mode,
                    position_accept_dist=self.detect_nav_accept_dist
                )
                nav_reached, nav_dist = self.nav_reached_by_state_and_distance(
                    nav_ok, prealign_goal, self.detect_nav_accept_dist)
                rospy.loginfo(
                    "[DETECT_NAV][ATTEMPT_DONE] point=%s mode=%s ok=%s reached=%s dist=%s accept=%.3f",
                    str(point), mode, str(nav_ok), str(nav_reached),
                    "%.3f" % nav_dist if nav_dist is not None else "None",
                    self.detect_nav_accept_dist
                )
                if nav_reached:
                    prealign_ok = True
                    selected_mode = mode
                    selected_distance = distance
                    capture_at_current_pose = self.should_capture_detection_at_prealign(
                        point, use_dynamic_yaw)
                    break

            if self.detect_yaw_align_at_prealign and prealign_ok:
                prealign_yaw = target[2]
                if use_dynamic_yaw and capture_at_current_pose:
                    prealign_yaw, _ = self.detection_yaw_from_current_pose(
                        point, target[2], "prealign_current")
                self.align_detection_yaw(prealign_yaw)

        if capture_at_current_pose:
            rospy.loginfo(
                "[DETECT_NAV][CAPTURE_AT_PREALIGN] point=%s mode=%s distance=%.3f reason=dynamic_photo_target",
                str(point), selected_mode, selected_distance)
        elif self.detect_locked_final_approach:
            if not prealign_ok:
                rospy.logwarn("检测点%s预对准未确认成功" % point)
                if self.detect_skip_capture_on_nav_fail:
                    rospy.logwarn("[DETECT_NAV][SKIP_CAPTURE] point=%s reason=prealign_failed", str(point))
                    return False
            if not self.locked_approach_detection_point(
                    target[2], mode=selected_mode, distance=selected_distance):
                if self.detect_skip_capture_on_nav_fail:
                    rospy.logwarn("[DETECT_NAV][SKIP_CAPTURE] point=%s reason=locked_approach_failed", str(point))
                    return False
        else:
            final_target = list(target)
            if use_dynamic_yaw:
                final_target[2], _ = self.detection_yaw_from_xy(
                    point, final_target[0], final_target[1], final_target[2],
                    "final_goal")
            rospy.loginfo("检测点%s原始拍照目标: %s" % (point, final_target))
            nav_ok = self.goto(final_target, timeout=self.detect_final_timeout)
            if not nav_ok and self.detect_skip_capture_on_nav_fail:
                rospy.logwarn("[DETECT_NAV][SKIP_CAPTURE] point=%s reason=final_nav_failed", str(point))
                return False

        if self.detect_yaw_align_at_photo:
            photo_yaw = target[2]
            if use_dynamic_yaw:
                photo_yaw, _ = self.detection_yaw_from_current_pose(
                    point, target[2], "photo_current")
            self.align_detection_yaw(photo_yaw)
        if self.detect_photo_settle_time > 0:
            rospy.sleep(self.detect_photo_settle_time)
        return True

    # ---------------- 取消导航 ----------------
    def cancel(self):
        self.move_base.cancel_all_goals()
        return True

    def normalize_detection_result(self, detect_result):
        if detect_result is None:
            return u""
        if isinstance(detect_result, unicode):
            return detect_result.strip()
        return str(detect_result).strip().decode("utf-8", "ignore")

    def is_no_detection_result(self, detect_result):
        return self.normalize_detection_result(detect_result) == u"无"

    def handle_detection_result(self, point, detect_result, source):
        """处理一次VLM结果；只有有效任务编号才入队和播报。"""
        global clue
        normalized = self.normalize_detection_result(detect_result)
        if normalized == u"无":
            rospy.loginfo("[DETECT_RESULT][NONE] point=%s source=%s", str(point), source)
            return False
        if not normalized:
            rospy.logwarn("[DETECT_RESULT][EMPTY] point=%s source=%s raw=%s",
                          str(point), source, str(detect_result))
            return False

        try:
            task_id = int(normalized)
        except ValueError:
            rospy.logwarn("检测结果不是有效数字: %s" % detect_result)
            return False

        if task_id not in VLM_TO_TASK:
            rospy.logwarn("任务编号超出范围: %s" % task_id)
            return False

        mapped_id = VLM_TO_TASK[task_id]
        task_numbers.append(mapped_id)
        rospy.loginfo("收集到任务编号: %s (原始VLM: %s)" % (mapped_id, task_id))
        tts_text = u"已检测第%d条线索为%d号" % (clue, task_id)
        self.tts_client(tts_text)
        clue += 1
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
        try:
            rospy.loginfo("导航到检测点 → 目标点索引%s" % point)
            # 首扫固定使用 goalListYaw，动态YAW只作为“无”后的保底。
            detect_nav_ok = self.goto_detection_point(point, use_dynamic_yaw=False)
            if not detect_nav_ok:
                rospy.logwarn("[DETECT_NAV][MISSION_SKIP] point=%s reason=navigation_failed", str(point))
                return False

            detect_result = self.call_fruit_detection_service()
            rospy.loginfo("当前检测点%s固定YAW扫描结果: %s" % (point, detect_result))
            if self.handle_detection_result(point, detect_result, "fixed_yaw"):
                return True

            if not self.is_no_detection_result(detect_result):
                return True

            if not self.should_run_dynamic_yaw_fallback(point):
                rospy.loginfo(
                    "[DETECT_YAW][FALLBACK_SKIP] point=%s reason=disabled_or_no_photo_target",
                    str(point)
                )
                return True

            rospy.logwarn(
                "[DETECT_YAW][FALLBACK_START] point=%s reason=first_scan_none",
                str(point)
            )
            fallback_nav_ok = self.goto_detection_point(point, use_dynamic_yaw=True)
            if not fallback_nav_ok:
                rospy.logwarn(
                    "[DETECT_YAW][FALLBACK_SKIP_CAPTURE] point=%s reason=navigation_failed",
                    str(point)
                )
                return True

            fallback_result = self.call_fruit_detection_service()
            rospy.loginfo("当前检测点%s动态YAW保底扫描结果: %s" % (point, fallback_result))
            self.handle_detection_result(point, fallback_result, "dynamic_yaw_fallback")
            return True
        finally:
            id = 0
            find_id = 0

    # ---------------- 执行识别 ----------------
    def recognize(self, p):
        return self.mission(p)

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

    def announce_task_arrival(self, idx, task_id):
        raw_id = TASK_TO_VLM.get(task_id, task_id)
        tts_text = u"已到达任务点%d号" % raw_id
        tts_start_time = rospy.Time.now()
        tts_ok = self.tts_client(tts_text)
        rospy.loginfo("[TASK_TIME][TTS] idx=%d task_id=%d dt=%.2fs ok=%s",
                      idx + 1, task_id,
                      (rospy.Time.now() - tts_start_time).to_sec(),
                      str(tts_ok))
        return tts_ok

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
                nav_ok, nav_reached, nav_dist, nav_mode, selected_yaw_deg = self.navigate_task_with_all_approaches(
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
                if selected_yaw_deg is None:
                    selected_yaw_deg = target[2]
                    rospy.logwarn(
                        "[TASK_NAV][FLEX_YAW_FALLBACK_ORIGINAL] idx=%d task_id=%d original_yaw=%.1f",
                        idx + 1, task_id, target[2]
                    )

                if (nav_mode == "target" and nav_dist is not None
                        and nav_dist <= self.task_nav_direct_done_dist):
                    rospy.loginfo(
                        "[TASK_TIME][DIRECT_DONE_SKIP_PARK] idx=%d task_id=%d target_dist=%.3f accept=%.3f",
                        idx + 1, task_id, nav_dist, self.task_nav_direct_done_dist
                    )
                    self.announce_task_arrival(idx, task_id)
                    last_parking = None
                    last_task_id = task_id
                    rospy.loginfo("[TASK_TIME][END] idx=%d task_id=%d total_dt=%.2fs direct_done=true",
                                  idx + 1, task_id,
                                  (rospy.Time.now() - task_start_time).to_sec())
                    continue

                rospy.loginfo("[TASK_TIME][PRE_PARK_WAIT][SKIP] idx=%d task_id=%d",
                              idx + 1, task_id)

                # 启动精密停车
                rospy.loginfo("move_base 到达任务点 %d，启动精密停车 (x=%.3f y=%.3f original_yaw=%.1f selected_yaw=%.1f)..." %
                              (task_id, target[0], target[1], target[2], selected_yaw_deg))
                rospy.loginfo("[PARK_TASK][START] idx=%d task_id=%d target=(%.3f, %.3f, %.1f) selected_yaw=%.1f",
                              idx + 1, task_id, target[0], target[1], target[2], selected_yaw_deg)

                parking_init_start_time = rospy.Time.now()
                parking = AutoSinglePointTest(
                    target_x=target[0],
                    target_y=target[1],
                    target_yaw_deg=selected_yaw_deg
                )
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
                self.announce_task_arrival(idx, task_id)

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
            for detect_idx, p in enumerate(points):
                rospy.loginfo("\n=== 开始处理第%s个检测点 ===" % (detect_idx + 1))
                detect_ok = self.recognize(p)
                if not detect_ok and self.detect_require_all_points:
                    rospy.logerr(
                        "[DETECT_NAV][ABORT_TASK_PHASE] detect_idx=%d point=%s collected=%s",
                        detect_idx + 1, str(p), task_numbers)
                    return False

            rospy.loginfo("\n=== 所有检测点处理完成 ===")
            rospy.loginfo("收集到的任务编号: %s" % task_numbers)

        # 按线索导航
        self.set_parking_phase_costmap()
        try:
            self.enable_obstacle_memory_after_parking("task_nav_start")
            self.go_to_task_positions()
        finally:
            self.disable_obstacle_memory_for_parking("task_phase_end")
            self.restore_cruise_costmap()

        # 终点按检测点思路处理：先到安全预对准点，再对齐yaw，最后交给激光闭环贴边。
        final_target = goals[16]
        final_nav_goal = final_target
        final_nav_timeout = self.final_nav_timeout
        if self.final_prealign_enabled and self.final_prealign_distance > 0.0:
            final_nav_goal = self.make_final_prealign_goal(final_target)
            final_nav_timeout = self.final_prealign_timeout
            rospy.loginfo(
                "[FINAL][PREALIGN_GOAL] mode=%s distance=%.3f target=(%.3f, %.3f, %.1f) goal=(%.3f, %.3f, %.1f)",
                self.final_prealign_mode,
                self.final_prealign_distance,
                final_target[0], final_target[1], final_target[2],
                final_nav_goal[0], final_nav_goal[1], final_nav_goal[2]
            )

        final_nav_start = rospy.Time.now()
        final_nav_ok = self.goto(final_nav_goal, timeout=final_nav_timeout)
        rospy.loginfo("[FINAL][NAV_TO_PREALIGN] dt=%.2fs ok=%s timeout=%.1fs",
                      (rospy.Time.now() - final_nav_start).to_sec(),
                      str(final_nav_ok), final_nav_timeout)

        final_yaw_ok = True
        if self.final_align_yaw_before_laser:
            final_yaw_ok = self.align_final_yaw(final_target[2])
            rospy.loginfo("[FINAL][YAW_ALIGN][DONE] ok=%s", str(final_yaw_ok))
        else:
            self.target_yaw = final_target[2] / 180.0 * pi

        rospy.loginfo("[FINAL][ADJUST_POSITION][START] target_yaw=%.1fdeg side=0.150 depth=0.240",
                      final_target[2])
        final_adjust_ok = self.adjust_position(side_target=0.17, back_target=0.20)
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
