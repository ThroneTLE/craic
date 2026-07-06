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
from sensor_msgs.msg import LaserScan, Imu, PointCloud2
import sensor_msgs.point_cloud2 as point_cloud2
from rosgraph_msgs.msg import Log
import sys, os, time, json, shutil, subprocess, threading, itertools
try:
    import Queue as queue_module
except ImportError:
    import queue as queue_module
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
        self.tts_service_lock = threading.RLock()
        self.tts_async_enabled = rospy.get_param("~tts_async_enabled", True)
        self.tts_async_queue_size = int(rospy.get_param("~tts_async_queue_size", 32))
        self.task_arrival_nav_delay_after_tts_start = rospy.get_param(
            "~task_arrival_nav_delay_after_tts_start", 0.8)
        self.task_arrival_tts_start_wait_timeout = rospy.get_param(
            "~task_arrival_tts_start_wait_timeout", 5.0)
        self.tts_async_queue = queue_module.Queue(max(1, self.tts_async_queue_size))
        self.tts_async_worker_thread = None
        if self.tts_async_enabled:
            self.start_tts_async_worker()

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
        self.detect_capture_stable_wait = rospy.get_param("~detect_capture_stable_wait", 0.08)
        self.detect_image_path = rospy.get_param(
            "~detect_image_path", "/home/abot/EIU0US/src/abot_vlm/temp2/vl_now.jpg")

        # OCR优先识别：固定yaw先走本地OCR+题库匹配，失败后再动态yaw+大模型兜底。
        self.detect_ocr_enabled = rospy.get_param("~detect_ocr_enabled", True)
        self.detect_ocr_python = rospy.get_param(
            "~detect_ocr_python", "/home/abot/anaconda3/envs/robot_com/bin/python3")
        self.detect_ocr_matcher_script = rospy.get_param(
            "~detect_ocr_matcher_script",
            "/home/abot/EIU0US/src/robot_slam/scripts/ocr_question_matcher.py")
        self.detect_ocr_min_score = rospy.get_param("~detect_ocr_min_score", 0.50)
        self.detect_ocr_timeout = rospy.get_param("~detect_ocr_timeout", 4.0)
        self.detect_ocr_capture_timeout = rospy.get_param("~detect_ocr_capture_timeout", 1.0)
        self.detect_ocr_snapshot_dir = rospy.get_param(
            "~detect_ocr_snapshot_dir", "/tmp/robot_slam_ocr")
        self.detect_ocr_early_enabled = rospy.get_param("~detect_ocr_early_enabled", True)
        self.detect_ocr_early_count = int(rospy.get_param("~detect_ocr_early_count", 2))
        self.detect_ocr_early_interval = rospy.get_param("~detect_ocr_early_interval", 0.20)
        self.detect_ocr_async_join_timeout = rospy.get_param(
            "~detect_ocr_async_join_timeout", 0.05)
        self.detect_ocr_capture_lock = threading.RLock()
        self.detect_ocr_process_lock = threading.RLock()
        self.detect_ocr_async_lock = threading.RLock()
        self.detect_ocr_async_threads = []
        self.detect_ocr_async_results = []
        self.detect_ocr_async_point = None
        self.detect_ocr_interrupt_result = None

        # 11. 调试/比赛固定任务点：跳过前置视觉扫描，直接进入任务点泊车
        self.use_fixed_task_positions = rospy.get_param("~use_fixed_task_positions", False)
        self.fixed_task_ids = rospy.get_param("~fixed_task_ids", "")
        self.task_nav_optimize_order = rospy.get_param("~task_nav_optimize_order", True)
        self.task_nav_order_pin_nearest_first = rospy.get_param(
            "~task_nav_order_pin_nearest_first", False)
        self.task_nav_order_pin_nearest_final_last = rospy.get_param(
            "~task_nav_order_pin_nearest_final_last", False)
        self.task_nav_order_use_final_prealign = rospy.get_param(
            "~task_nav_order_use_final_prealign", True)
        self.task_nav_order_final_goal_index = int(rospy.get_param(
            "~task_nav_order_final_goal_index", 16))
        self.task_nav_order_max_bruteforce = int(rospy.get_param(
            "~task_nav_order_max_bruteforce", 7))
        self.task_nav_order_turn_penalty_weight = rospy.get_param(
            "~task_nav_order_turn_penalty_weight", 0.0)
        self.task_nav_order_path_quality_enabled = rospy.get_param(
            "~task_nav_order_path_quality_enabled", True)
        self.task_nav_order_use_approach_candidates = rospy.get_param(
            "~task_nav_order_use_approach_candidates", True)
        self.task_nav_order_reject_no_plan = rospy.get_param(
            "~task_nav_order_reject_no_plan", True)
        self.task_nav_order_reject_sharp_turns = rospy.get_param(
            "~task_nav_order_reject_sharp_turns", True)
        self.task_nav_order_path_cost_weight = rospy.get_param(
            "~task_nav_order_path_cost_weight", 0.004)
        self.task_nav_order_path_max_cost_weight = rospy.get_param(
            "~task_nav_order_path_max_cost_weight", 0.002)
        self.task_nav_order_unknown_penalty = rospy.get_param(
            "~task_nav_order_unknown_penalty", 0.08)
        self.task_nav_order_no_plan_penalty = rospy.get_param(
            "~task_nav_order_no_plan_penalty", 1000.0)
        self.task_nav_order_sharp_turn_penalty = rospy.get_param(
            "~task_nav_order_sharp_turn_penalty", 20.0)
        self.final_nav_timeout = rospy.get_param("~final_nav_timeout", 10.0)
        self.final_prealign_enabled = rospy.get_param("~final_prealign_enabled", True)
        self.final_prealign_mode = rospy.get_param("~final_prealign_mode", "back")
        self.final_prealign_distance = rospy.get_param("~final_prealign_distance", 0.35)
        self.final_prealign_timeout = rospy.get_param("~final_prealign_timeout", self.final_nav_timeout)
        self.final_doudi_enabled = rospy.get_param("~final_doudi_enabled", True)
        self.final_doudi_plan_check_enabled = rospy.get_param("~final_doudi_plan_check_enabled", True)
        self.final_doudi_goal_x = rospy.get_param("~final_doudi_goal_x", 0.25)
        self.final_doudi_goal_y = rospy.get_param("~final_doudi_goal_y", 2.25)
        self.final_doudi_goal_yaw = rospy.get_param("~final_doudi_goal_yaw", 90.0)
        self.final_doudi_timeout = rospy.get_param("~final_doudi_timeout", self.final_prealign_timeout)
        self.final_doudi_side_laser_direction = rospy.get_param(
            "~final_doudi_side_laser_direction", "left")
        self.final_align_yaw_before_laser = rospy.get_param("~final_align_yaw_before_laser", True)
        self.final_yaw_align_timeout = rospy.get_param("~final_yaw_align_timeout", 3.0)
        self.final_yaw_tolerance = rospy.get_param("~final_yaw_tolerance", 0.05)
        self.final_yaw_kp = rospy.get_param("~final_yaw_kp", 1.2)
        self.final_yaw_min_vel = rospy.get_param("~final_yaw_min_vel", 0.08)
        self.final_yaw_max_vel = rospy.get_param("~final_yaw_max_vel", 0.45)
        self.final_yaw_stable_count = int(rospy.get_param("~final_yaw_stable_count", 3))
        self.final_side_laser_direction = rospy.get_param("~final_side_laser_direction", "left")
        self.final_depth_laser_direction = rospy.get_param("~final_depth_laser_direction", "back")
        self.final_doudi_depth_laser_direction = rospy.get_param(
            "~final_doudi_depth_laser_direction", self.final_depth_laser_direction)
        self.final_side_target = rospy.get_param("~final_side_target", 0.17)
        self.final_depth_target = rospy.get_param("~final_depth_target", 0.20)
        self.final_adjust_timeout = rospy.get_param("~final_adjust_timeout", 9.0)
        self.final_side_prealign_tolerance = rospy.get_param(
            "~final_side_prealign_tolerance", 0.04)
        self.final_side_prealign_yaw_tolerance = rospy.get_param(
            "~final_side_prealign_yaw_tolerance", self.yaw_tolerance)
        self.final_side_prealign_continue_depth_hold = rospy.get_param(
            "~final_side_prealign_continue_depth_hold", True)
        self.final_depth_hold_side_yaw = rospy.get_param("~final_depth_hold_side_yaw", True)
        self.final_depth_tolerance = rospy.get_param(
            "~final_depth_tolerance", self.position_tolerance)
        self.final_depth_hold_side_tolerance = rospy.get_param(
            "~final_depth_hold_side_tolerance", self.final_side_prealign_tolerance)
        self.final_depth_hold_yaw_tolerance = rospy.get_param(
            "~final_depth_hold_yaw_tolerance", self.yaw_tolerance)
        self.final_depth_max_v = rospy.get_param("~final_depth_max_v", 0.06)
        self.final_depth_max_side_v = rospy.get_param("~final_depth_max_side_v", 0.04)
        self.final_depth_max_wz = rospy.get_param("~final_depth_max_wz", 0.35)
        self.final_depth_side_kp = rospy.get_param("~final_depth_side_kp", self.kp_linear)
        self.final_depth_yaw_kp = rospy.get_param("~final_depth_yaw_kp", self.kp_angular)
        self.final_arrival_tts_on_depth_timeout = rospy.get_param(
            "~final_arrival_tts_on_depth_timeout", True)
        self.final_depth_hold_timed_out_after_cmd = False
        self.final_depth_hold_last_state = None
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
        self.task_nav_retry_approach_accept_dist = rospy.get_param(
            "~task_nav_retry_approach_accept_dist",
            self.task_nav_approach_accept_dist)
        self.task_nav_direct_done_dist = rospy.get_param("~task_nav_direct_done_dist", 0.05)
        self.task_nav_use_approach_goal = rospy.get_param("~task_nav_use_approach_goal", True)
        self.task_nav_approach_offset = rospy.get_param("~task_nav_approach_offset", 0.30)
        self.task_nav_approach_modes = rospy.get_param(
            "~task_nav_approach_modes",
            "back,back_left,left,front_left,front,front_right,right,back_right")
        self.task_nav_l_corner_pretry_enabled = rospy.get_param(
            "~task_nav_l_corner_pretry_enabled", True)
        self.task_nav_l_corner_pretry_task_ids_param = rospy.get_param(
            "~task_nav_l_corner_pretry_task_ids", "all")
        self.task_nav_l_corner_pretry_task_ids = self.parse_task_id_list_param(
            self.task_nav_l_corner_pretry_task_ids_param,
            "task_nav_l_corner_pretry_task_ids")
        self.task_nav_l_corner_pretry_mode = rospy.get_param(
            "~task_nav_l_corner_pretry_mode", "auto_l_opening")
        self.task_nav_l_corner_pretry_map_dx = rospy.get_param(
            "~task_nav_l_corner_pretry_map_dx", 0.20)
        self.task_nav_l_corner_pretry_map_dy = rospy.get_param(
            "~task_nav_l_corner_pretry_map_dy", 0.20)
        self.task_nav_l_corner_pretry_corner_offsets_param = rospy.get_param(
            "~task_nav_l_corner_pretry_corner_offsets", "0.20,0.25,0.30")
        self.task_nav_l_corner_pretry_corner_offsets = self.parse_float_list_param(
            self.task_nav_l_corner_pretry_corner_offsets_param,
            "task_nav_l_corner_pretry_corner_offsets",
            [0.20, 0.25, 0.30])
        self.task_nav_l_corner_pretry_offset = rospy.get_param(
            "~task_nav_l_corner_pretry_offset", 0.35)
        self.task_nav_l_corner_pretry_timeout = rospy.get_param(
            "~task_nav_l_corner_pretry_timeout", 4.0)
        self.task_nav_l_corner_pretry_accept_dist = rospy.get_param(
            "~task_nav_l_corner_pretry_accept_dist",
            0.15)
        self.task_nav_l_corner_pretry_score_radius = rospy.get_param(
            "~task_nav_l_corner_pretry_score_radius", 0.0)
        self.task_nav_l_corner_pretry_require_clear = rospy.get_param(
            "~task_nav_l_corner_pretry_require_clear", True)
        self.task_nav_l_corner_pretry_require_plan = rospy.get_param(
            "~task_nav_l_corner_pretry_require_plan", True)
        self.task_nav_l_corner_pretry_require_costmap = rospy.get_param(
            "~task_nav_l_corner_pretry_require_costmap", True)
        self.task_nav_l_corner_detect_side_offset = rospy.get_param(
            "~task_nav_l_corner_detect_side_offset", 0.24)
        self.task_nav_l_corner_detect_side_span = rospy.get_param(
            "~task_nav_l_corner_detect_side_span", 0.26)
        self.task_nav_l_corner_detect_open_span = rospy.get_param(
            "~task_nav_l_corner_detect_open_span", 0.10)
        self.task_nav_l_corner_detect_sample_step = rospy.get_param(
            "~task_nav_l_corner_detect_sample_step", 0.04)
        self.task_nav_l_corner_detect_min_blocked_points = int(rospy.get_param(
            "~task_nav_l_corner_detect_min_blocked_points", 2))
        self.task_nav_l_corner_detect_open_max_blocked_points = int(rospy.get_param(
            "~task_nav_l_corner_detect_open_max_blocked_points", 1))
        self.task_nav_l_corner_detect_open_relaxed_max_blocked_points = int(rospy.get_param(
            "~task_nav_l_corner_detect_open_relaxed_max_blocked_points", 3))
        self.task_nav_l_corner_detect_open_blocked_margin = int(rospy.get_param(
            "~task_nav_l_corner_detect_open_blocked_margin", 2))
        self.task_nav_l_corner_detect_unknown_as_blocked = rospy.get_param(
            "~task_nav_l_corner_detect_unknown_as_blocked", True)
        self.task_nav_l_corner_use_obstacle_memory_cloud = rospy.get_param(
            "~task_nav_l_corner_use_obstacle_memory_cloud", True)
        self.task_nav_l_corner_obstacle_memory_topic = rospy.get_param(
            "~task_nav_l_corner_obstacle_memory_topic", "/scan_obstacle_memory")
        self.task_nav_l_corner_obstacle_memory_max_age = rospy.get_param(
            "~task_nav_l_corner_obstacle_memory_max_age", 2.0)
        self.task_nav_l_corner_memory_side_half_width = rospy.get_param(
            "~task_nav_l_corner_memory_side_half_width", 0.06)
        self.task_nav_l_corner_memory_min_blocked_points = int(rospy.get_param(
            "~task_nav_l_corner_memory_min_blocked_points", 3))
        self.task_nav_l_corner_memory_open_max_points = int(rospy.get_param(
            "~task_nav_l_corner_memory_open_max_points", 2))
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
            "~task_nav_flexible_yaw_candidates", "90,-90")
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
        self.localization_guard_enabled = rospy.get_param("~localization_guard_enabled", True)
        self.localization_guard_start_x = rospy.get_param("~localization_guard_start_x", 0.50)
        self.localization_guard_start_y = rospy.get_param("~localization_guard_start_y", -0.30)
        self.localization_guard_start_radius = rospy.get_param("~localization_guard_start_radius", 0.75)
        self.localization_guard_start_wait = rospy.get_param("~localization_guard_start_wait", 3.0)
        self.localization_guard_max_jump = rospy.get_param("~localization_guard_max_jump", 0.60)
        self.localization_guard_jump_window = rospy.get_param("~localization_guard_jump_window", 1.0)
        self.localization_guard_protect_enabled = rospy.get_param("~localization_guard_protect_enabled", True)
        self.localization_guard_recover_timeout = rospy.get_param("~localization_guard_recover_timeout", 2.0)
        self.localization_guard_stable_time = rospy.get_param("~localization_guard_stable_time", 0.6)
        self.localization_guard_zero_cmd_count = int(rospy.get_param("~localization_guard_zero_cmd_count", 4))
        self.localization_guard_last_pose = None
        self.localization_guard_last_time = None
        self.global_costmap = None
        self.task_nav_make_plan_client = None
        self.task_nav_teb_client = None
        self.task_nav_teb_nominal_config = None
        self.l_corner_obstacle_memory_points = []
        self.l_corner_obstacle_memory_stamp = rospy.Time(0)
        rospy.Subscriber(self.task_nav_approach_costmap_topic, OccupancyGrid, self.global_costmap_callback)
        if self.task_nav_l_corner_use_obstacle_memory_cloud:
            rospy.Subscriber(
                self.task_nav_l_corner_obstacle_memory_topic,
                PointCloud2,
                self.l_corner_obstacle_memory_callback,
                queue_size=1)
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

    def l_corner_obstacle_memory_callback(self, msg):
        points = []
        try:
            for point in point_cloud2.read_points(
                    msg, field_names=("x", "y", "z"), skip_nans=True):
                x = float(point[0])
                y = float(point[1])
                if np.isfinite(x) and np.isfinite(y):
                    points.append((x, y))
        except Exception as e:
            rospy.logwarn_throttle(
                1.0,
                "[TASK_NAV][L_CORNER_MEMORY_CLOUD_FAILED] topic=%s error=%s",
                self.task_nav_l_corner_obstacle_memory_topic,
                str(e))
            return

        self.l_corner_obstacle_memory_points = points
        self.l_corner_obstacle_memory_stamp = (
            msg.header.stamp if msg.header.stamp != rospy.Time(0) else rospy.Time.now())

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
                "[TASK_NAV][FLEX_YAW_NO_CANDIDATES] raw=%s fallback=90,-90",
                str(value)
            )
            candidates = [90.0, -90.0]
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

    def parse_float_list_param(self, value, label, default_values):
        values = []
        for item in str(value).replace(";", ",").split(","):
            item = item.strip()
            if not item:
                continue
            try:
                values.append(float(item))
            except ValueError:
                rospy.logwarn("[%s] bad float value=%s", label, item)
        if not values:
            return list(default_values)
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

    def make_task_approach_goals(self, target, log_candidates=True):
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
            if log_candidates:
                rospy.loginfo(
                    "[TASK_NAV][APPROACH_CANDIDATE] mode=%s target=(%.3f,%.3f,%.1f) approach=(%.3f,%.3f,%.1f) offset=%.3f",
                    mode, target[0], target[1], target[2],
                    approach[0], approach[1], approach[2],
                    self.task_nav_approach_offset
                )

        goals_out.append(("target", list(target)))
        return goals_out

    def make_task_approach_goal_by_mode(self, target, mode, offset):
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
        }
        normalized_mode = str(mode).strip().lower()
        l_corner_vectors = {
            "l_opening_upper_right": (1.0, 1.0),
            "l_opening_ur": (1.0, 1.0),
            "map_upper_right": (1.0, 1.0),
            "l_opening_upper_left": (-1.0, 1.0),
            "l_opening_ul": (-1.0, 1.0),
            "l_opening_lower_right": (1.0, -1.0),
            "l_opening_lr": (1.0, -1.0),
            "l_opening_lower_left": (-1.0, -1.0),
            "l_opening_ll": (-1.0, -1.0),
        }
        if normalized_mode in l_corner_vectors:
            sx, sy = l_corner_vectors[normalized_mode]
            return [
                target[0] + sx * abs(self.task_nav_l_corner_pretry_map_dx),
                target[1] + sy * abs(self.task_nav_l_corner_pretry_map_dy),
                target[2]
            ]
        if normalized_mode not in mode_vectors:
            return None
        vx, vy = mode_vectors[normalized_mode]
        return [
            target[0] + vx * offset,
            target[1] + vy * offset,
            target[2]
        ]

    def is_auto_l_corner_mode(self, mode):
        return str(mode).strip().lower() in [
            "auto",
            "auto_l",
            "auto_l_opening",
            "detect_l_opening"
        ]

    def task_l_corner_pretry_enabled_for_task(self, task_id):
        if not self.task_nav_l_corner_pretry_enabled:
            return False
        if self.task_nav_l_corner_pretry_task_ids is None:
            return True
        return int(task_id) in self.task_nav_l_corner_pretry_task_ids

    def l_corner_count_side(self, costmap, target, side, span):
        offset = max(0.01, float(self.task_nav_l_corner_detect_side_offset))
        span = max(0.01, float(span))
        step = max(0.01, float(self.task_nav_l_corner_detect_sample_step))
        sample_count = max(3, int(np.ceil((span * 2.0) / step)) + 1)
        blocked_count = 0
        unknown_count = 0
        valid_count = 0
        max_cost = 0

        for i in range(sample_count):
            if sample_count <= 1:
                t = 0.0
            else:
                t = -span + (2.0 * span * float(i) / float(sample_count - 1))
            if side == "left":
                x = target[0] - offset
                y = target[1] + t
            elif side == "right":
                x = target[0] + offset
                y = target[1] + t
            elif side == "up":
                x = target[0] + t
                y = target[1] + offset
            elif side == "down":
                x = target[0] + t
                y = target[1] - offset
            else:
                return {
                    "blocked_count": 0,
                    "unknown_count": 0,
                    "valid_count": 0,
                    "sample_count": 0,
                    "max_cost": 0,
                    "reason": "unknown_side"
                }

            cost, detail = self.costmap_cost_at(costmap, x, y)
            if cost is None or cost < 0:
                unknown_count += 1
                if self.task_nav_l_corner_detect_unknown_as_blocked:
                    blocked_count += 1
                    max_cost = max(max_cost, 100)
                continue

            valid_count += 1
            max_cost = max(max_cost, cost)
            if cost > self.task_nav_approach_cost_threshold:
                blocked_count += 1

        return {
            "blocked_count": blocked_count,
            "unknown_count": unknown_count,
            "valid_count": valid_count,
            "sample_count": sample_count,
            "max_cost": max_cost,
        }

    def l_corner_memory_points_available(self):
        if not self.task_nav_l_corner_use_obstacle_memory_cloud:
            return []
        if not self.l_corner_obstacle_memory_points:
            return []
        if self.task_nav_l_corner_obstacle_memory_max_age > 0.0:
            age = (rospy.Time.now() - self.l_corner_obstacle_memory_stamp).to_sec()
            if age > self.task_nav_l_corner_obstacle_memory_max_age:
                return []
        return self.l_corner_obstacle_memory_points

    def l_corner_count_obstacle_memory_side(self, target, side, span):
        points = self.l_corner_memory_points_available()
        if not points:
            return 0

        offset = max(0.01, float(self.task_nav_l_corner_detect_side_offset))
        span = max(0.01, float(span))
        half_width = max(0.01, float(self.task_nav_l_corner_memory_side_half_width))
        count = 0
        if side == "left":
            line_x = target[0] - offset
            for x, y in points:
                if abs(x - line_x) <= half_width and abs(y - target[1]) <= span:
                    count += 1
        elif side == "right":
            line_x = target[0] + offset
            for x, y in points:
                if abs(x - line_x) <= half_width and abs(y - target[1]) <= span:
                    count += 1
        elif side == "up":
            line_y = target[1] + offset
            for x, y in points:
                if abs(y - line_y) <= half_width and abs(x - target[0]) <= span:
                    count += 1
        elif side == "down":
            line_y = target[1] - offset
            for x, y in points:
                if abs(y - line_y) <= half_width and abs(x - target[0]) <= span:
                    count += 1
        return count

    def l_corner_sample_side(self, costmap, target, side):
        broad = self.l_corner_count_side(
            costmap,
            target,
            side,
            self.task_nav_l_corner_detect_side_span)
        core = self.l_corner_count_side(
            costmap,
            target,
            side,
            self.task_nav_l_corner_detect_open_span)

        memory_broad_count = self.l_corner_count_obstacle_memory_side(
            target,
            side,
            self.task_nav_l_corner_detect_side_span)
        memory_core_count = self.l_corner_count_obstacle_memory_side(
            target,
            side,
            self.task_nav_l_corner_detect_open_span)
        memory_blocked = (
            memory_broad_count >= self.task_nav_l_corner_memory_min_blocked_points)
        memory_open = (
            memory_core_count <= self.task_nav_l_corner_memory_open_max_points)

        blocked = (
            broad["blocked_count"] >= self.task_nav_l_corner_detect_min_blocked_points
            or memory_blocked)
        open_side = (
            core["blocked_count"] <= self.task_nav_l_corner_detect_open_max_blocked_points
            and memory_open
            and (not self.task_nav_l_corner_detect_unknown_as_blocked
                 or core["unknown_count"] == 0))
        return {
            "blocked": blocked,
            "open": open_side,
            "blocked_count": broad["blocked_count"],
            "unknown_count": broad["unknown_count"],
            "valid_count": broad["valid_count"],
            "sample_count": broad["sample_count"],
            "max_cost": broad["max_cost"],
            "core_blocked_count": core["blocked_count"],
            "core_unknown_count": core["unknown_count"],
            "core_valid_count": core["valid_count"],
            "core_sample_count": core["sample_count"],
            "core_max_cost": core["max_cost"],
            "memory_blocked_count": memory_broad_count,
            "memory_core_count": memory_core_count,
            "memory_blocked": memory_blocked,
            "memory_open": memory_open,
            "reason": "b=%d u=%d v=%d n=%d max=%d core_b=%d core_u=%d core_n=%d core_max=%d mem_b=%d mem_core=%d blocked=%s open=%s" % (
                broad["blocked_count"], broad["unknown_count"],
                broad["valid_count"], broad["sample_count"], broad["max_cost"],
                core["blocked_count"], core["unknown_count"], core["sample_count"],
                core["max_cost"], memory_broad_count, memory_core_count,
                str(blocked), str(open_side))
        }

    def format_l_corner_side_info(self, sides):
        parts = []
        for name in ["left", "right", "up", "down"]:
            info = sides.get(name, {})
            parts.append("%s:%s" % (name, info.get("reason", "none")))
        return "; ".join(parts)

    def l_corner_open_against_opposite(self, sides, open_side, opposite_side):
        info = sides[open_side]
        opposite = sides[opposite_side]
        if info["open"]:
            return True, "strict"
        if (self.task_nav_l_corner_detect_unknown_as_blocked
                and info["core_unknown_count"] > 0):
            return False, "unknown_core=%d" % info["core_unknown_count"]

        core_blocked = max(info["core_blocked_count"], info["memory_core_count"])
        opposite_core_blocked = max(
            opposite["core_blocked_count"],
            opposite["memory_core_count"])
        broad_blocked = max(info["blocked_count"], info["memory_blocked_count"])
        opposite_broad_blocked = max(
            opposite["blocked_count"],
            opposite["memory_blocked_count"])
        core_margin = opposite_core_blocked - core_blocked
        broad_margin = opposite_broad_blocked - broad_blocked

        relaxed_max = self.task_nav_l_corner_detect_open_relaxed_max_blocked_points
        required_margin = self.task_nav_l_corner_detect_open_blocked_margin
        if (core_blocked <= relaxed_max
                and core_margin >= required_margin
                and broad_margin >= required_margin):
            return True, "relative core=%d opp_core=%d broad=%d opp_broad=%d" % (
                core_blocked, opposite_core_blocked,
                broad_blocked, opposite_broad_blocked)

        return False, "core=%d opp_core=%d broad=%d opp_broad=%d" % (
            core_blocked, opposite_core_blocked,
            broad_blocked, opposite_broad_blocked)

    def l_corner_side_blocked_evidence(self, side_info):
        return max(
            side_info["blocked_count"],
            side_info["memory_blocked_count"])

    def detect_l_corner_pretry_candidates(self, idx, task_id, target):
        costmap = self.get_global_costmap_for_approach()
        if costmap is None:
            rospy.logwarn(
                "[TASK_NAV][L_CORNER_DETECT_SKIP] idx=%d task_id=%d reason=costmap_unavailable",
                idx + 1, task_id)
            return []

        sides = {}
        for side in ["left", "right", "up", "down"]:
            sides[side] = self.l_corner_sample_side(costmap, target, side)

        corner_defs = [
            ("upper_right", 1.0, 1.0, ("right", "up"), ("left", "down")),
            ("upper_left", -1.0, 1.0, ("left", "up"), ("right", "down")),
            ("lower_right", 1.0, -1.0, ("right", "down"), ("left", "up")),
            ("lower_left", -1.0, -1.0, ("left", "down"), ("right", "up")),
        ]
        opposite_sides = {
            "left": "right",
            "right": "left",
            "up": "down",
            "down": "up",
        }

        candidates = []
        rejected = []
        for name, sx, sy, open_sides, blocked_sides in corner_defs:
            open_checks = []
            for side in open_sides:
                ok, reason = self.l_corner_open_against_opposite(
                    sides, side, opposite_sides[side])
                open_checks.append((side, ok, reason))
            open_ok = all(item[1] for item in open_checks)
            blocked_ok = all(sides[s]["blocked"] for s in blocked_sides)
            score = sum(self.l_corner_side_blocked_evidence(sides[s]) for s in blocked_sides) \
                - sum(self.l_corner_side_blocked_evidence(sides[s]) for s in open_sides)
            if not (open_ok and blocked_ok):
                rejected.append("%s:open=%s blocked=%s score=%d open_detail=%s" % (
                    name, str(open_ok), str(blocked_ok), score,
                    ",".join(["%s:%s" % (item[0], item[2]) for item in open_checks])))
                continue

            for corner_offset in self.task_nav_l_corner_pretry_corner_offsets:
                nav_target = [
                    target[0] + sx * corner_offset,
                    target[1] + sy * corner_offset,
                    target[2]
                ]
                mode = "l_opening_%s" % name
                reason = "auto open=%s blocked=%s offset=%.2f score=%d" % (
                    ",".join(open_sides), ",".join(blocked_sides),
                    corner_offset, score)
                candidates.append({
                    "mode": mode,
                    "nav_target": nav_target,
                    "score": score,
                    "offset": corner_offset,
                    "reason": reason
                })

        candidates.sort(
            key=lambda item: (item["score"], item.get("offset", 0.0)),
            reverse=True)
        rospy.logwarn(
            "[TASK_NAV][L_CORNER_DETECT] idx=%d task_id=%d sides={%s} candidates=%s rejected=%s",
            idx + 1, task_id,
            self.format_l_corner_side_info(sides),
            ",".join(["%s@%.2f" % (c["mode"], c.get("offset", 0.0))
                      for c in candidates]) if candidates else "none",
            "; ".join(rejected) if rejected else "none")
        return candidates

    def make_l_corner_pretry_candidates(self, idx, task_id, target):
        mode = str(self.task_nav_l_corner_pretry_mode).strip().lower()
        if self.is_auto_l_corner_mode(mode):
            return self.detect_l_corner_pretry_candidates(idx, task_id, target)

        nav_target = self.make_task_approach_goal_by_mode(
            target,
            mode,
            self.task_nav_l_corner_pretry_offset)
        if nav_target is None:
            rospy.logwarn(
                "[TASK_NAV][L_CORNER_PRETRY_SKIP] idx=%d task_id=%d reason=unknown_mode mode=%s",
                idx + 1, task_id, mode)
            return []
        return [{
            "mode": mode,
            "nav_target": nav_target,
            "score": 0,
            "reason": "manual"
        }]

    def evaluate_l_corner_pretry_goal(self, task_id, nav_target, mode):
        reasons = []
        costmap = self.global_costmap
        if costmap is None:
            if self.task_nav_l_corner_pretry_require_costmap:
                return False, "costmap_unavailable_required"
            reasons.append("costmap_unavailable_allow")
        else:
            cost, detail = self.costmap_cost_at(costmap, nav_target[0], nav_target[1])
            if cost is None:
                if self.task_nav_l_corner_pretry_require_clear:
                    return False, detail
                reasons.append("%s allow_probe" % detail)
            elif cost < 0:
                if self.task_nav_approach_reject_unknown:
                    if self.task_nav_l_corner_pretry_require_clear:
                        return False, "unknown"
                    reasons.append("unknown allow_probe")
                else:
                    reasons.append("unknown_allowed")
            elif cost > self.task_nav_approach_cost_threshold:
                reason = "cost=%d>threshold=%d" % (
                    cost, self.task_nav_approach_cost_threshold)
                if self.task_nav_l_corner_pretry_require_clear:
                    return False, reason
                reasons.append("%s allow_probe" % reason)
            else:
                score, score_detail = self.costmap_score_near(
                    costmap,
                    nav_target[0],
                    nav_target[1],
                    self.task_nav_l_corner_pretry_score_radius)
                if score is None:
                    reasons.append("cost=%d score_unavailable=%s" % (cost, score_detail))
                elif score[0] > self.task_nav_approach_cost_threshold:
                    reason = "corner_score=max:%d>threshold:%d avg:%.1f unk:%d radius:%.2f" % (
                        score[0], self.task_nav_approach_cost_threshold,
                        score[1], score[2],
                        self.task_nav_l_corner_pretry_score_radius)
                    if self.task_nav_l_corner_pretry_require_clear:
                        return False, reason
                    reasons.append("%s allow_probe" % reason)
                else:
                    reasons.append("cost=%d score=max:%d avg:%.1f unk:%d radius:%.2f" % (
                        cost, score[0], score[1], score[2],
                        self.task_nav_l_corner_pretry_score_radius))

        plan_mode = "l_corner:%s" % mode
        if self.task_nav_l_corner_pretry_require_plan:
            path_clear, path_reason = self.evaluate_detection_prealign_goal(
                plan_mode, nav_target)
            if not path_clear:
                return False, "%s %s" % (" ".join(reasons), path_reason)
            reasons.append(path_reason)
        else:
            reasons.append("plan_check_skipped")
        return True, " ".join(reasons)

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

    def raw_current_map_pose(self, timeout=0.1, log_warn=True):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        try:
            self.tf_listener.waitForTransform(
                "map", "base_footprint", rospy.Time(0), rospy.Duration(timeout))
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
            if log_warn:
                rospy.logwarn_throttle(
                    2.0,
                    "[TASK_NAV][PATH_FILTER_TF_FALLBACK] map->base_footprint unavailable: %s",
                    str(e)
                )
            return None

    def map_pose_xy_yaw(self, pose):
        q = pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return pose.pose.position.x, pose.pose.position.y, yaw

    def set_localization_guard_reference(self, pose):
        if pose is None:
            return
        x, y, yaw = self.map_pose_xy_yaw(pose)
        if not (np.isfinite(x) and np.isfinite(y) and np.isfinite(yaw)):
            return
        self.localization_guard_last_pose = (float(x), float(y), float(yaw))
        self.localization_guard_last_time = time.time()

    def fail_localization_guard(self, label, reason):
        rospy.logerr("[LOCALIZATION_GUARD][WARN_CONTINUE] label=%s reason=%s",
                     str(label), str(reason))

    def publish_guard_stop(self):
        repeat = max(1, int(self.localization_guard_zero_cmd_count))
        for _ in range(repeat):
            self.pub.publish(Twist())
            rospy.sleep(0.02)

    def wait_localization_guard_stable(self, label):
        timeout = max(0.0, float(self.localization_guard_recover_timeout))
        stable_time = max(0.0, float(self.localization_guard_stable_time))
        window = max(0.01, float(self.localization_guard_jump_window))
        start = time.time()
        stable_since = None
        last_pose = None
        last_time = None
        latest_pose = None
        rate = rospy.Rate(10)

        while not rospy.is_shutdown() and time.time() - start <= timeout:
            pose = self.raw_current_map_pose(timeout=0.02, log_warn=False)
            now = time.time()
            if pose is None:
                stable_since = None
                self.publish_guard_stop()
                rate.sleep()
                continue

            x, y, yaw = self.map_pose_xy_yaw(pose)
            if not (np.isfinite(x) and np.isfinite(y) and np.isfinite(yaw)):
                stable_since = None
                self.publish_guard_stop()
                rate.sleep()
                continue

            latest_pose = pose
            if last_pose is not None and last_time is not None:
                last_x, last_y, last_yaw = last_pose
                dt = now - last_time
                jump = np.sqrt((x - last_x) ** 2 + (y - last_y) ** 2)
                if dt <= window and jump > self.localization_guard_max_jump:
                    stable_since = None
                    rospy.logerr_throttle(
                        0.5,
                        "[LOCALIZATION_GUARD][RECOVER_STILL_JUMPING] label=%s jump=%.3fm dt=%.2fs max=%.3f",
                        str(label), jump, dt, self.localization_guard_max_jump)
                elif stable_since is None:
                    stable_since = now
            else:
                stable_since = now

            last_pose = (float(x), float(y), float(yaw))
            last_time = now
            if stable_since is not None and now - stable_since >= stable_time:
                self.set_localization_guard_reference(pose)
                rospy.logwarn(
                    "[LOCALIZATION_GUARD][RECOVER_STABLE] label=%s stable=%.2fs elapsed=%.2fs pose=(%.3f,%.3f,%.1fdeg)",
                    str(label), now - stable_since, now - start,
                    x, y, yaw * 180.0 / pi)
                return True

            self.publish_guard_stop()
            rate.sleep()

        if latest_pose is not None:
            self.set_localization_guard_reference(latest_pose)
        rospy.logerr(
            "[LOCALIZATION_GUARD][RECOVER_TIMEOUT_CONTINUE] label=%s timeout=%.2fs stable_required=%.2fs",
            str(label), timeout, stable_time)
        return False

    def handle_localization_jump_protection(self, label, cancel_active_goal=False):
        if not self.localization_guard_protect_enabled:
            return 0.0

        start = time.time()
        rospy.logerr(
            "[LOCALIZATION_GUARD][PROTECT_BEGIN] label=%s cancel_active_goal=%s no_shutdown=true",
            str(label), str(cancel_active_goal))
        self.publish_guard_stop()
        if cancel_active_goal:
            self.cancel_move_base_goal(
                "localization_guard:%s" % str(label),
                GoalStatus.PREEMPTED,
                wait_timeout=min(0.5, float(self.move_base_cancel_wait)))
        self.publish_guard_stop()
        stable = self.wait_localization_guard_stable(label)
        self.publish_guard_stop()
        elapsed = time.time() - start
        rospy.logwarn(
            "[LOCALIZATION_GUARD][PROTECT_RESUME] label=%s stable=%s elapsed=%.2fs action=reissue_goal_continue",
            str(label), str(stable), elapsed)
        return elapsed

    def check_start_localization(self):
        if not self.localization_guard_enabled:
            return True

        pose = None
        start_time = time.time()
        wait_time = max(0.0, float(self.localization_guard_start_wait))
        while not rospy.is_shutdown() and time.time() - start_time <= wait_time:
            pose = self.raw_current_map_pose(timeout=0.2, log_warn=False)
            if pose is not None:
                break
            rospy.sleep(0.05)

        if pose is None:
            self.fail_localization_guard("start", "no_map_pose")
            return True

        x, y, yaw = self.map_pose_xy_yaw(pose)
        dist = np.sqrt((x - self.localization_guard_start_x) ** 2
                       + (y - self.localization_guard_start_y) ** 2)
        if (not np.isfinite(dist)
                or dist > self.localization_guard_start_radius):
            self.fail_localization_guard(
                "start",
                "pose=(%.3f,%.3f,%.1fdeg) expected=(%.3f,%.3f) dist=%.3f radius=%.3f" % (
                    x, y, yaw * 180.0 / pi,
                    self.localization_guard_start_x,
                    self.localization_guard_start_y,
                    dist,
                    self.localization_guard_start_radius))
            self.set_localization_guard_reference(pose)
            return True

        self.set_localization_guard_reference(pose)
        rospy.loginfo(
            "[LOCALIZATION_GUARD][START_OK] pose=(%.3f,%.3f,%.1fdeg) expected=(%.3f,%.3f) dist=%.3f radius=%.3f",
            x, y, yaw * 180.0 / pi,
            self.localization_guard_start_x,
            self.localization_guard_start_y,
            dist,
            self.localization_guard_start_radius)
        return True

    def check_localization_jump(self, label):
        if not self.localization_guard_enabled:
            return True

        pose = self.raw_current_map_pose(timeout=0.02, log_warn=False)
        if pose is None:
            return True

        x, y, yaw = self.map_pose_xy_yaw(pose)
        if not (np.isfinite(x) and np.isfinite(y) and np.isfinite(yaw)):
            self.fail_localization_guard(label, "non_finite_pose")
            return True

        now = time.time()
        if self.localization_guard_last_pose is not None and self.localization_guard_last_time is not None:
            last_x, last_y, last_yaw = self.localization_guard_last_pose
            dt = now - self.localization_guard_last_time
            jump = np.sqrt((x - last_x) ** 2 + (y - last_y) ** 2)
            if (dt <= max(0.01, float(self.localization_guard_jump_window))
                    and jump > self.localization_guard_max_jump):
                rospy.logerr_throttle(
                    0.5,
                    "[LOCALIZATION_GUARD][JUMP_PROTECT_REQUEST] label=%s jump=%.3fm dt=%.2fs from=(%.3f,%.3f) to=(%.3f,%.3f) max=%.3f",
                    str(label), jump, dt, last_x, last_y, x, y,
                    self.localization_guard_max_jump)
                self.localization_guard_last_pose = (float(x), float(y), float(yaw))
                self.localization_guard_last_time = now
                return False

        self.localization_guard_last_pose = (float(x), float(y), float(yaw))
        self.localization_guard_last_time = now
        return True

    def current_map_pose_for_plan(self):
        pose = self.raw_current_map_pose()
        if pose is not None:
            return pose
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()

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

    def analyze_task_plan_sharp_turns(self, poses, log_warning=True):
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
        if log_warning:
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
        is_retry = str(label).startswith("RETRY")
        if mode.startswith("transition:"):
            accept_dist = self.task_nav_transition_accept_dist
        else:
            if mode == "target":
                accept_dist = self.task_nav_accept_dist
            else:
                accept_dist = (self.task_nav_retry_approach_accept_dist
                               if is_retry else self.task_nav_approach_accept_dist)
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
            followup_accept = (self.task_nav_retry_approach_accept_dist
                               if is_retry else self.task_nav_approach_accept_dist)
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
                accept_dist
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

    def try_l_corner_pretry(self, idx, task_id, target):
        if not self.task_l_corner_pretry_enabled_for_task(task_id):
            return False, False, False, None, None, None

        candidates = self.make_l_corner_pretry_candidates(idx, task_id, target)
        if not candidates:
            rospy.logwarn(
                "[TASK_NAV][L_CORNER_PRETRY_SKIP] idx=%d task_id=%d reason=no_l_corner_candidate",
                idx + 1, task_id)
            return False, False, False, None, None, None

        selected = None
        for candidate in candidates:
            mode = candidate["mode"]
            nav_target = candidate["nav_target"]
            clear, reason = self.evaluate_l_corner_pretry_goal(
                task_id, nav_target, mode)
            rospy.logwarn(
                "[TASK_NAV][L_CORNER_PRETRY_CHECK] idx=%d task_id=%d mode=%s nav_target=(%.3f,%.3f,%.1f) clear=%s detect=%s reason=%s",
                idx + 1, task_id, mode,
                nav_target[0], nav_target[1], nav_target[2],
                str(clear), candidate.get("reason", ""), reason)
            if clear:
                selected = candidate
                break

        if selected is None:
            rospy.logwarn(
                "[TASK_NAV][L_CORNER_PRETRY_SKIP] idx=%d task_id=%d reason=no_clear_planned_l_corner_candidate",
                idx + 1, task_id)
            return False, False, False, None, None, None

        mode = selected["mode"]
        nav_target = selected["nav_target"]

        label = "L_CORNER_PRETRY"
        nav_mode = "l_corner:%s" % mode
        nav_start_time = rospy.Time.now()
        nav_ok = self.goto_task_nav_goal(
            nav_target,
            timeout=self.task_nav_l_corner_pretry_timeout,
            label=label,
            mode=nav_mode,
            position_accept_dist=self.task_nav_l_corner_pretry_accept_dist)
        nav_reached, approach_dist = self.nav_reached_by_state_and_distance(
            nav_ok,
            nav_target,
            self.task_nav_l_corner_pretry_accept_dist)
        nav_dist = self.distance_to_goal_xy(target)
        selected_yaw_deg = None
        if nav_reached:
            selected_yaw_deg = self.select_task_flexible_yaw(target)
            yaw_err = self.yaw_error_to_goal(target)
            rospy.logwarn(
                "[TASK_NAV][L_CORNER_PRETRY_REACHED] idx=%d task_id=%d mode=%s approach_dist=%s target_dist=%s accept=%.3f yaw_err=%.3f selected_yaw=%.1f",
                idx + 1, task_id, mode,
                "%.3f" % approach_dist if approach_dist is not None else "None",
                "%.3f" % nav_dist if nav_dist is not None else "None",
                self.task_nav_l_corner_pretry_accept_dist,
                yaw_err,
                selected_yaw_deg)

        rospy.loginfo(
            "[TASK_TIME][NAV_ATTEMPT] idx=%d task_id=%d label=%s dt=%.2fs ok=%s target_dist=%s reached=%s state=%s mode=%s selected_yaw=%s",
            idx + 1, task_id, label,
            (rospy.Time.now() - nav_start_time).to_sec(),
            str(nav_ok),
            "%.3f" % nav_dist if nav_dist is not None else "None",
            str(nav_reached), str(self.last_move_base_state), nav_mode,
            "%.1f" % selected_yaw_deg if selected_yaw_deg is not None else "None")
        if not nav_reached:
            rospy.logwarn(
                "[TASK_NAV][L_CORNER_PRETRY_FALLBACK] idx=%d task_id=%d mode=%s target_dist=%s approach_dist=%s",
                idx + 1, task_id, mode,
                "%.3f" % nav_dist if nav_dist is not None else "None",
                "%.3f" % approach_dist if approach_dist is not None else "None")
        return True, nav_ok, nav_reached, nav_dist, nav_mode, selected_yaw_deg

    def navigate_task_with_all_approaches(self, idx, task_id, target, last_parking, last_task_id):
        failed_approach_modes = set()
        nav_ok = False
        nav_reached = False
        nav_dist = None
        nav_mode = None
        selected_yaw_deg = None
        escaped_after_abort = False
        l_corner_pretry_done = False
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

            if not l_corner_pretry_done:
                l_corner_pretry_done = True
                pretry_used, pretry_ok, pretry_reached, pretry_dist, pretry_mode, pretry_yaw = (
                    self.try_l_corner_pretry(idx, task_id, target))
                if pretry_used and pretry_reached:
                    return pretry_ok, pretry_reached, pretry_dist, pretry_mode, pretry_yaw

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

    def adjust_final_axis(self, label, angle, axis, sign, target, timeout, adjust_yaw,
                          distance_tolerance=None, yaw_tolerance=None):
        if distance_tolerance is None:
            distance_tolerance = self.position_tolerance
        if yaw_tolerance is None:
            yaw_tolerance = self.yaw_tolerance
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        self.is_adjusting = True
        rospy.loginfo(
            "[FINAL][ADJUST_%s][START] axis=%s target=%.3f timeout=%.1fs yaw=%s dist_tol=%.3f yaw_tol=%.3f",
            label, axis, target, timeout, str(adjust_yaw),
            distance_tolerance, yaw_tolerance)

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
            distance_ok = abs(distance_error) <= distance_tolerance
            yaw_ok = (not adjust_yaw) or abs(yaw_error) <= yaw_tolerance

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

    def adjust_final_depth_with_side_yaw(
            self, side_angle, side_axis, side_sign, side_target,
            depth_angle, depth_axis, depth_sign, depth_target, timeout):
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        self.is_adjusting = True
        depth_cmd_sent = False
        self.final_depth_hold_timed_out_after_cmd = False
        self.final_depth_hold_last_state = None
        rospy.loginfo(
            "[FINAL][ADJUST_DEPTH_HOLD][START] depth_axis=%s depth_target=%.3f side_axis=%s side_target=%.3f timeout=%.1fs",
            depth_axis, depth_target, side_axis, side_target, timeout)

        while not rospy.is_shutdown() and self.is_adjusting:
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed > timeout:
                self.final_depth_hold_timed_out_after_cmd = depth_cmd_sent
                if self.final_depth_hold_last_state is not None:
                    state = self.final_depth_hold_last_state
                    rospy.logwarn(
                        "[FINAL][ADJUST_DEPTH_HOLD][TIMEOUT] elapsed=%.2f depth_cmd_sent=%s depth=%.3f target=%.3f err=%.3f side=%.3f target=%.3f err=%.3f yaw_err=%.3f",
                        elapsed, str(depth_cmd_sent),
                        state["depth"], depth_target, state["depth_error"],
                        state["side"], side_target, state["side_error"],
                        state["yaw_error"])
                else:
                    rospy.logwarn(
                        "[FINAL][ADJUST_DEPTH_HOLD][TIMEOUT] elapsed=%.2f depth_cmd_sent=%s no_valid_laser_state",
                        elapsed, str(depth_cmd_sent))
                self.stop_movement()
                return False

            side_distance = self.get_range_at_angle(side_angle)
            depth_distance = self.get_range_at_angle(depth_angle)
            if not np.isfinite(side_distance) or not np.isfinite(depth_distance):
                rospy.logwarn_throttle(
                    1.0,
                    "[FINAL][ADJUST_DEPTH_HOLD][WAIT_LASER] side_valid=%s depth_valid=%s",
                    str(np.isfinite(side_distance)), str(np.isfinite(depth_distance)))
                self.pub.publish(Twist())
                rate.sleep()
                continue

            side_error = side_distance - side_target
            depth_error = depth_distance - depth_target
            yaw_error = self.normalize_angle(self.target_yaw - self.current_yaw)
            self.final_depth_hold_last_state = {
                "depth": depth_distance,
                "depth_error": depth_error,
                "side": side_distance,
                "side_error": side_error,
                "yaw_error": yaw_error,
            }
            side_ok = abs(side_error) <= self.final_depth_hold_side_tolerance
            depth_ok = abs(depth_error) <= self.final_depth_tolerance
            yaw_ok = abs(yaw_error) <= self.final_depth_hold_yaw_tolerance

            if side_ok and depth_ok and yaw_ok:
                rospy.loginfo(
                    "[FINAL][ADJUST_DEPTH_HOLD][OK] depth=%.3f target=%.3f err=%.3f side=%.3f target=%.3f err=%.3f yaw_err=%.3f",
                    depth_distance, depth_target, depth_error,
                    side_distance, side_target, side_error, yaw_error)
                self.stop_movement()
                return True

            cmd = Twist()
            if not depth_ok:
                depth_cmd = self.kp_linear * depth_sign * depth_error
                depth_cmd = self.clamp(
                    depth_cmd,
                    -abs(self.final_depth_max_v),
                    abs(self.final_depth_max_v))
                if abs(depth_cmd) > 1e-4:
                    depth_cmd_sent = True
                self.apply_final_axis_cmd(cmd, depth_axis, depth_cmd)
            if not side_ok:
                side_cmd = self.final_depth_side_kp * side_sign * side_error
                side_cmd = self.clamp(
                    side_cmd,
                    -abs(self.final_depth_max_side_v),
                    abs(self.final_depth_max_side_v))
                self.apply_final_axis_cmd(cmd, side_axis, side_cmd)
            if not yaw_ok:
                cmd.angular.z = self.clamp(
                    self.final_depth_yaw_kp * yaw_error,
                    -abs(self.final_depth_max_wz),
                    abs(self.final_depth_max_wz))

            self.pub.publish(cmd)
            rospy.loginfo_throttle(
                0.5,
                "[FINAL][ADJUST_DEPTH_HOLD] depth=%.3f target=%.3f err=%.3f side=%.3f target=%.3f err=%.3f yaw_err=%.3f ok=(%s,%s,%s) cmd=(%.3f,%.3f,%.3f)",
                depth_distance, depth_target, depth_error,
                side_distance, side_target, side_error, yaw_error,
                str(depth_ok), str(side_ok), str(yaw_ok),
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
        self.final_depth_hold_timed_out_after_cmd = False
        self.final_depth_hold_last_state = None
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
            True,
            self.final_side_prealign_tolerance,
            self.final_side_prealign_yaw_tolerance)

        if not side_ok:
            rospy.logwarn(
                "[FINAL][ADJUST_POSITION][SIDE_FAILED] continue_depth_hold=%s force_side_on_fail=%s",
                str(self.final_side_prealign_continue_depth_hold
                    and self.final_depth_hold_side_yaw),
                str(self.final_adjust_force_side_on_fail))
            if self.final_depth_hold_side_yaw and self.final_side_prealign_continue_depth_hold:
                rospy.logwarn(
                    "[FINAL][ADJUST_POSITION][SIDE_FAILED_CONTINUE_DEPTH_HOLD] depth stage will keep correcting side+yaw")
            elif self.final_adjust_force_side_on_fail:
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
            else:
                return False

        if self.final_depth_hold_side_yaw:
            depth_ok = self.adjust_final_depth_with_side_yaw(
                side_angle,
                side_axis,
                side_sign,
                side_target,
                depth_angle,
                depth_axis,
                depth_sign,
                back_target,
                self.final_adjust_timeout)
        else:
            depth_ok = self.adjust_final_axis(
                "DEPTH",
                depth_angle,
                depth_axis,
                depth_sign,
                back_target,
                self.final_adjust_timeout,
                False)
        if self.final_depth_hold_side_yaw:
            return depth_ok
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

    def make_final_doudi_goal(self):
        """主终点预对齐点不可规划时使用的终点兜底位姿。"""
        return [
            float(self.final_doudi_goal_x),
            float(self.final_doudi_goal_y),
            float(self.final_doudi_goal_yaw)
        ]

    def final_nav_make_plan_ok(self, nav_target, label):
        if not self.final_doudi_plan_check_enabled:
            return None, "plan_check_disabled"
        if not self.wait_for_make_plan_idle("final:%s" % label):
            return None, "move_base_active_for_make_plan"

        client = self.get_task_make_plan_client()
        if client is None:
            return None, "make_plan_service_unavailable"

        start = self.current_map_pose_for_plan()
        if start is None:
            return None, "no_start_pose"

        request = GetPlanRequest()
        request.start = start
        request.start.header.stamp = rospy.Time.now()
        request.goal = self.task_goal_pose_for_plan(nav_target)
        request.tolerance = 0.0
        try:
            response = client(request)
        except Exception as e:
            self.task_nav_make_plan_client = None
            return None, "make_plan_failed:%s" % str(e)

        poses = response.plan.poses
        if len(poses) < 2:
            dx = nav_target[0] - start.pose.position.x
            dy = nav_target[1] - start.pose.position.y
            dist = np.sqrt(dx * dx + dy * dy)
            if dist <= 0.12:
                reason = "already_close points=%d dist=%.3f" % (len(poses), dist)
                rospy.loginfo(
                    "[FINAL][PLAN_CHECK][OK] label=%s target=(%.3f,%.3f,%.1f) %s",
                    label, nav_target[0], nav_target[1], nav_target[2], reason)
                return True, reason
            reason = "no_plan points=%d dist=%.3f" % (len(poses), dist)
            rospy.logwarn(
                "[FINAL][PLAN_CHECK][NO_PLAN] label=%s target=(%.3f,%.3f,%.1f) %s",
                label, nav_target[0], nav_target[1], nav_target[2], reason)
            return False, reason

        reason = "path_ok points=%d" % len(poses)
        rospy.loginfo(
            "[FINAL][PLAN_CHECK][OK] label=%s target=(%.3f,%.3f,%.1f) %s",
            label, nav_target[0], nav_target[1], nav_target[2], reason)
        return True, reason

    def select_final_doudi_goal(self, reason):
        goal = self.make_final_doudi_goal()
        timeout = float(self.final_doudi_timeout)
        side_direction = str(self.final_doudi_side_laser_direction)
        depth_direction = str(self.final_doudi_depth_laser_direction)
        rospy.logwarn(
            "[FINAL][DOUDI_SELECTED] reason=%s goal=(%.3f, %.3f, %.1f) side_laser=%s depth_laser=%s timeout=%.1fs",
            reason, goal[0], goal[1], goal[2],
            side_direction, depth_direction, timeout)
        return goal, timeout, side_direction, depth_direction

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

    def align_detection_yaw(self, yaw_deg, interrupt_check=None):
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
            if interrupt_check is not None and interrupt_check():
                self.stop_movement()
                rospy.logwarn("[DETECT_NAV][YAW_ALIGN_INTERRUPTED_BY_OCR]")
                return False
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

    def locked_approach_detection_point(self, yaw_deg, mode=None, distance=None,
                                        interrupt_check=None):
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
            if interrupt_check is not None and interrupt_check():
                self.stop_movement()
                rospy.logwarn("[DETECT_NAV][LOCKED_APPROACH_INTERRUPTED_BY_OCR]")
                return False
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
    def start_tts_async_worker(self):
        if self.tts_async_worker_thread is not None:
            return
        thread = threading.Thread(target=self.tts_async_worker)
        thread.daemon = True
        self.tts_async_worker_thread = thread
        thread.start()
        rospy.loginfo("[TTS_ASYNC][WORKER_START] queue_size=%d",
                      max(1, self.tts_async_queue_size))

    def tts_async_worker(self):
        while not rospy.is_shutdown():
            try:
                item = self.tts_async_queue.get(True, 0.2)
            except queue_module.Empty:
                continue
            if item is None:
                self.tts_async_queue.task_done()
                return

            if len(item) == 3:
                text, label, enqueue_time = item
                start_event = None
                start_time_holder = None
            else:
                text, label, enqueue_time, start_event, start_time_holder = item
            rospy.loginfo("[TTS_ASYNC][PLAY_START] label=%s queue_wait=%.2fs",
                          label, time.time() - enqueue_time)
            if start_time_holder is not None:
                start_time_holder["time"] = time.time()
            if start_event is not None:
                start_event.set()
            start_time = rospy.Time.now()
            ok = self.tts_client(text)
            rospy.loginfo("[TTS_ASYNC][PLAY_DONE] label=%s dt=%.2fs ok=%s",
                          label,
                          (rospy.Time.now() - start_time).to_sec(),
                          str(ok))
            self.tts_async_queue.task_done()

    def tts_client_async(self, text, label="", start_event=None, start_time_holder=None):
        if not self.tts_async_enabled:
            if start_time_holder is not None:
                start_time_holder["time"] = time.time()
            if start_event is not None:
                start_event.set()
            return self.tts_client(text)
        try:
            self.tts_async_queue.put_nowait(
                (text, str(label), time.time(), start_event, start_time_holder))
            rospy.loginfo("[TTS_ASYNC][ENQUEUE] label=%s queue=%d text=%s",
                          str(label), self.tts_async_queue.qsize(),
                          self.log_text(text))
            return True
        except queue_module.Full:
            rospy.logwarn("[TTS_ASYNC][DROP] label=%s reason=queue_full text=%s",
                          str(label), self.log_text(text))
            return False

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
            with self.tts_service_lock:
                response = self.tts_service(request)
            rospy.loginfo("TTS播报成功: %s | 响应: %s" %
                          (self.log_text(text), self.log_text(response.result)))
            return True
        except rospy.ServiceException as e:
            rospy.logerr("TTS服务调用失败: %s" % str(e))
            return False

    # ---------------- OCR优先识别客户端 ----------------
    def ensure_ocr_snapshot_dir(self):
        try:
            if not os.path.isdir(self.detect_ocr_snapshot_dir):
                os.makedirs(self.detect_ocr_snapshot_dir)
            return True
        except Exception as e:
            rospy.logwarn("[DETECT_OCR][SNAPSHOT_DIR_FAILED] dir=%s err=%s",
                          self.detect_ocr_snapshot_dir, str(e))
            return False

    def safe_label_text(self, text):
        safe = []
        for ch in str(text):
            if ch.isalnum() or ch in ["_", "-"]:
                safe.append(ch)
            else:
                safe.append("_")
        return "".join(safe).strip("_") or "capture"

    def image_mtime(self, path):
        try:
            if os.path.isfile(path):
                return os.path.getmtime(path)
        except Exception:
            pass
        return 0.0

    def image_stat(self, path):
        try:
            if os.path.isfile(path):
                stat = os.stat(path)
                return stat.st_mtime, stat.st_size
        except Exception:
            pass
        return 0.0, 0

    def image_jpeg_complete(self, path):
        try:
            if not path or not os.path.isfile(path):
                return False
            size = os.path.getsize(path)
            if size < 4:
                return False
            with open(path, "rb") as fp:
                head = fp.read(2)
                fp.seek(-2, os.SEEK_END)
                tail = fp.read(2)
            return head == "\xff\xd8" and tail == "\xff\xd9"
        except Exception:
            return False

    def wait_detection_image_ready(self, reason, before_mtime, start_time):
        last_mtime = 0.0
        last_size = -1
        stable_since = None
        while not rospy.is_shutdown():
            now = time.time()
            current_mtime, current_size = self.image_stat(self.detect_image_path)
            complete = self.image_jpeg_complete(self.detect_image_path)
            changed = current_mtime > before_mtime and current_size > 0
            same_as_last = (
                current_mtime == last_mtime
                and current_size == last_size
            )
            if changed and same_as_last and complete:
                if stable_since is None:
                    stable_since = now
                if now - stable_since >= self.detect_capture_stable_wait:
                    return True
            else:
                stable_since = None

            last_mtime = current_mtime
            last_size = current_size
            if now - start_time > self.detect_ocr_capture_timeout:
                rospy.logwarn(
                    "[DETECT_CAPTURE][TIMEOUT] reason=%s path=%s timeout=%.2fs before_mtime=%.6f current_mtime=%.6f size=%d complete=%s",
                    reason, self.detect_image_path, self.detect_ocr_capture_timeout,
                    before_mtime, current_mtime, current_size, str(complete))
                return False
            rospy.sleep(0.02)

        return False

    def copy_detection_snapshot(self, reason, snapshot_path, start_time):
        tmp_path = "%s.tmp.%d" % (snapshot_path, int(time.time() * 1000))
        while not rospy.is_shutdown():
            try:
                if os.path.isfile(tmp_path):
                    os.remove(tmp_path)
                shutil.copy2(self.detect_image_path, tmp_path)
                if self.image_jpeg_complete(tmp_path):
                    if os.path.isfile(snapshot_path):
                        os.remove(snapshot_path)
                    os.rename(tmp_path, snapshot_path)
                    rospy.loginfo("[DETECT_CAPTURE][SNAPSHOT] reason=%s path=%s",
                                  reason, snapshot_path)
                    return snapshot_path
                rospy.logwarn(
                    "[DETECT_CAPTURE][SNAPSHOT_INCOMPLETE] reason=%s src=%s tmp=%s size=%d",
                    reason, self.detect_image_path, tmp_path,
                    os.path.getsize(tmp_path) if os.path.isfile(tmp_path) else 0)
            except Exception as e:
                rospy.logwarn("[DETECT_CAPTURE][SNAPSHOT_FAILED] reason=%s src=%s err=%s",
                              reason, self.detect_image_path, str(e))
            try:
                if os.path.isfile(tmp_path):
                    os.remove(tmp_path)
            except Exception:
                pass
            if time.time() - start_time > self.detect_ocr_capture_timeout:
                return None
            rospy.sleep(0.03)

        return None

    def capture_detection_image(self, reason, snapshot=True):
        """
        只触发相机保存图片，不触发大模型。返回可供OCR/VLM使用的图片路径。
        identify_service.py 中 /detect=1 会保存 detect_image_path。
        """
        with self.detect_ocr_capture_lock:
            before_mtime = self.image_mtime(self.detect_image_path)
            try:
                rospy.set_param('/detect_run_vlm_on_capture', False)
                rospy.set_param('/detect', 1)
            except Exception as e:
                rospy.logwarn("[DETECT_CAPTURE][PARAM_FAILED] reason=%s err=%s",
                              reason, str(e))
                return None

            start_time = time.time()
            if not self.wait_detection_image_ready(reason, before_mtime, start_time):
                return None

            if not snapshot:
                rospy.loginfo("[DETECT_CAPTURE][OK] reason=%s path=%s",
                              reason, self.detect_image_path)
                return self.detect_image_path

            if not self.ensure_ocr_snapshot_dir():
                return self.detect_image_path

            snapshot_name = "%s_%d.jpg" % (
                self.safe_label_text(reason),
                int(time.time() * 1000)
            )
            snapshot_path = os.path.join(self.detect_ocr_snapshot_dir, snapshot_name)
            snapshot_path = self.copy_detection_snapshot(reason, snapshot_path, start_time)
            if snapshot_path is None:
                rospy.logwarn("[DETECT_CAPTURE][SNAPSHOT_TIMEOUT] reason=%s src=%s",
                              reason, self.detect_image_path)
            return snapshot_path

    def run_process_with_timeout(self, cmd, timeout):
        start_time = time.time()
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        while proc.poll() is None:
            if time.time() - start_time > timeout:
                try:
                    proc.kill()
                except Exception:
                    pass
                stdout, stderr = proc.communicate()
                return proc.returncode, stdout, stderr, time.time() - start_time, True
            time.sleep(0.03)
        stdout, stderr = proc.communicate()
        return proc.returncode, stdout, stderr, time.time() - start_time, False

    def decode_process_text(self, value):
        if value is None:
            return u""
        if isinstance(value, unicode):
            return value
        return str(value).decode("utf-8", "ignore")

    def log_text(self, value, max_len=None):
        text = self.decode_process_text(value)
        if max_len is not None:
            text = text[-max_len:]
        return text.encode("utf-8", "replace")

    def run_ocr_matcher(self, image_path, reason):
        if not self.detect_ocr_enabled:
            return {
                "valid": False,
                "empty": True,
                "answer": None,
                "score": 0.0,
                "raw_text": u"",
                "reason": "disabled",
                "source": reason
            }
        if not image_path or not os.path.isfile(image_path):
            rospy.logwarn("[DETECT_OCR][IMAGE_MISSING] reason=%s path=%s",
                          reason, str(image_path))
            return {
                "valid": False,
                "empty": True,
                "answer": None,
                "score": 0.0,
                "raw_text": u"",
                "reason": "image_missing",
                "source": reason
            }
        if not self.image_jpeg_complete(image_path):
            rospy.logwarn("[DETECT_OCR][IMAGE_INCOMPLETE] reason=%s path=%s size=%d",
                          reason, image_path,
                          os.path.getsize(image_path) if os.path.isfile(image_path) else 0)
            return {
                "valid": False,
                "empty": True,
                "answer": None,
                "score": 0.0,
                "raw_text": u"",
                "reason": "image_incomplete",
                "source": reason
            }

        cmd = [
            self.detect_ocr_python,
            self.detect_ocr_matcher_script,
            "--image", image_path,
            "--top-k", "3",
            "--json"
        ]
        rospy.loginfo("[DETECT_OCR][START] reason=%s path=%s timeout=%.2fs",
                      reason, image_path, self.detect_ocr_timeout)
        with self.detect_ocr_process_lock:
            return_code, stdout, stderr, elapsed, timed_out = self.run_process_with_timeout(
                cmd, self.detect_ocr_timeout)
        stdout_text = self.decode_process_text(stdout).strip()
        stderr_text = self.decode_process_text(stderr).strip()
        if timed_out:
            rospy.logwarn("[DETECT_OCR][TIMEOUT] reason=%s timeout=%.2fs path=%s",
                          reason, self.detect_ocr_timeout, image_path)
            return {
                "valid": False,
                "empty": True,
                "answer": None,
                "score": 0.0,
                "raw_text": u"",
                "reason": "timeout",
                "source": reason,
                "elapsed": elapsed
            }
        if return_code != 0:
            rospy.logwarn("[DETECT_OCR][FAILED] reason=%s code=%s elapsed=%.2fs stderr=%s stdout=%s",
                          reason, str(return_code), elapsed,
                          self.log_text(stderr_text, 300),
                          self.log_text(stdout_text, 300))
            return {
                "valid": False,
                "empty": True,
                "answer": None,
                "score": 0.0,
                "raw_text": u"",
                "reason": "process_failed",
                "source": reason,
                "elapsed": elapsed
            }

        try:
            payload = json.loads(stdout_text.splitlines()[-1])
        except Exception as e:
            rospy.logwarn("[DETECT_OCR][BAD_JSON] reason=%s err=%s stdout=%s",
                          reason, str(e), self.log_text(stdout_text, 500))
            return {
                "valid": False,
                "empty": True,
                "answer": None,
                "score": 0.0,
                "raw_text": u"",
                "reason": "bad_json",
                "source": reason,
                "elapsed": elapsed
            }

        raw_text = payload.get("raw_text", u"")
        normalized_text = payload.get("normalized_text", u"")
        try:
            answer = int(payload.get("best_answer"))
        except Exception:
            answer = None
        try:
            score = float(payload.get("best_score", 0.0))
        except Exception:
            score = 0.0

        empty = not bool(normalized_text)
        valid = (not empty
                 and answer in VLM_TO_TASK
                 and score >= self.detect_ocr_min_score)
        rospy.loginfo(
            "[DETECT_OCR][RESULT] reason=%s valid=%s empty=%s answer=%s score=%.4f threshold=%.2f elapsed=%.2fs raw=%s best=%s",
            reason, str(valid), str(empty), str(answer), score,
            self.detect_ocr_min_score, elapsed,
            self.log_text(raw_text),
            self.log_text(payload.get("best_question", u"")))
        return {
            "valid": valid,
            "empty": empty,
            "answer": answer,
            "score": score,
            "raw_text": raw_text,
            "normalized_text": normalized_text,
            "reason": "ok" if valid else "no_valid_match",
            "source": reason,
            "elapsed": elapsed,
            "image_path": image_path,
            "payload": payload
        }

    def run_detection_ocr_capture(self, point, reason):
        image_path = self.capture_detection_image(
            "ocr_point_%s_%s" % (str(point), reason),
            snapshot=True)
        if image_path is None:
            return {
                "valid": False,
                "empty": True,
                "answer": None,
                "score": 0.0,
                "raw_text": u"",
                "reason": "capture_failed",
                "source": reason,
                "point": point
            }
        result = self.run_ocr_matcher(image_path, reason)
        result["point"] = point
        return result

    def reset_detection_ocr_async(self, point):
        with self.detect_ocr_async_lock:
            self.detect_ocr_async_results = []
            self.detect_ocr_async_threads = []
            self.detect_ocr_async_point = point
            self.detect_ocr_interrupt_result = None
        rospy.loginfo("[DETECT_OCR][EARLY_CAPTURE_RESET] point=%s", str(point))

    def is_detection_ocr_async_current(self, point):
        with self.detect_ocr_async_lock:
            return self.detect_ocr_async_point == point

    def store_detection_ocr_async_result(self, result):
        with self.detect_ocr_async_lock:
            if result.get("point") != self.detect_ocr_async_point:
                rospy.loginfo(
                    "[DETECT_OCR][EARLY_STALE_RESULT] point=%s current=%s source=%s",
                    str(result.get("point")), str(self.detect_ocr_async_point),
                    str(result.get("source")))
                return
            self.detect_ocr_async_results.append(result)
            if result.get("valid"):
                self.detect_ocr_interrupt_result = result
                rospy.logwarn(
                    "[DETECT_OCR][EARLY_VALID_INTERRUPT_READY] point=%s answer=%s score=%.4f source=%s",
                    str(result.get("point")), str(result.get("answer")),
                    result.get("score", 0.0), str(result.get("source")))

    def get_detection_ocr_interrupt_result(self, point):
        with self.detect_ocr_async_lock:
            result = self.detect_ocr_interrupt_result
            if result is not None and result.get("point") == point and result.get("valid"):
                return result
            for item in self.detect_ocr_async_results:
                if item.get("point") == point and item.get("valid"):
                    self.detect_ocr_interrupt_result = item
                    return item
        return None

    def detection_ocr_interrupt_requested(self, point):
        result = self.get_detection_ocr_interrupt_result(point)
        if result is None:
            return False
        rospy.logwarn(
            "[DETECT_OCR][EARLY_VALID_INTERRUPT] point=%s answer=%s score=%.4f source=%s",
            str(point), str(result.get("answer")),
            result.get("score", 0.0), str(result.get("source")))
        return True

    def sleep_with_detection_ocr_interrupt(self, duration, point, reason):
        if duration <= 0.0:
            return not self.detection_ocr_interrupt_requested(point)
        deadline = time.time() + float(duration)
        while not rospy.is_shutdown():
            if self.detection_ocr_interrupt_requested(point):
                return False
            remaining = deadline - time.time()
            if remaining <= 0.0:
                return True
            rospy.sleep(min(0.05, remaining))
        return False


    def run_detection_ocr_burst(self, point, reason):
        try:
            count = max(0, int(self.detect_ocr_early_count))
            for index in range(count):
                if rospy.is_shutdown():
                    return
                if not self.is_detection_ocr_async_current(point):
                    rospy.loginfo(
                        "[DETECT_OCR][EARLY_STOP_STALE] point=%s reason=%s",
                        str(point), reason)
                    return
                if self.get_detection_ocr_interrupt_result(point) is not None:
                    return
                capture_reason = "%s_%d" % (reason, index + 1)
                snapshot = self.capture_detection_image(
                    "ocr_point_%s_%s" % (str(point), capture_reason),
                    snapshot=True)
                if snapshot is not None:
                    rospy.loginfo(
                        "[DETECT_OCR][EARLY_CAPTURE] point=%s reason=%s seq=%d path=%s",
                        str(point), capture_reason, index + 1, snapshot)
                    result = self.run_ocr_matcher(snapshot, capture_reason)
                    result["point"] = point
                    result["pending_ocr"] = False
                    result["sequence"] = index
                    self.store_detection_ocr_async_result(result)
                    if result.get("valid"):
                        rospy.logwarn(
                            "[DETECT_OCR][EARLY_BURST_STOP_VALID] point=%s reason=%s answer=%s score=%.4f",
                            str(point), capture_reason, str(result.get("answer")),
                            result.get("score", 0.0))
                        return
                else:
                    self.store_detection_ocr_async_result({
                        "valid": False,
                        "empty": True,
                        "answer": None,
                        "score": 0.0,
                        "raw_text": u"",
                        "reason": "early_capture_failed",
                        "source": capture_reason,
                        "point": point,
                        "pending_ocr": False,
                        "sequence": index
                    })
                if index + 1 < count and self.detect_ocr_early_interval > 0.0:
                    if not self.sleep_with_detection_ocr_interrupt(
                            self.detect_ocr_early_interval, point, "early_interval"):
                        return
        except Exception as e:
            rospy.logerr("[DETECT_OCR][EARLY_CAPTURE_EXCEPTION] point=%s reason=%s err=%s",
                         str(point), reason, str(e))

    def start_detection_ocr_burst_async(self, point, reason):
        if (not self.detect_ocr_enabled
                or not self.detect_ocr_early_enabled
                or self.detect_ocr_early_count <= 0):
            return False
        thread = threading.Thread(
            target=self.run_detection_ocr_burst,
            args=(point, reason))
        thread.daemon = True
        with self.detect_ocr_async_lock:
            self.detect_ocr_async_threads.append(thread)
        thread.start()
        rospy.loginfo("[DETECT_OCR][EARLY_CAPTURE_START] point=%s reason=%s count=%d",
                      str(point), reason, self.detect_ocr_early_count)
        return True

    def thread_is_alive(self, thread):
        if hasattr(thread, "is_alive"):
            return thread.is_alive()
        return thread.isAlive()

    def collect_detection_ocr_async_results(self, point, wait_timeout=None):
        if wait_timeout is None:
            wait_timeout = self.detect_ocr_async_join_timeout
        deadline = time.time() + max(0.0, float(wait_timeout))
        with self.detect_ocr_async_lock:
            threads = list(self.detect_ocr_async_threads)
        for thread in threads:
            if not self.thread_is_alive(thread):
                continue
            remaining = deadline - time.time()
            if remaining <= 0.0:
                break
            thread.join(remaining)
        with self.detect_ocr_async_lock:
            return [
                result for result in self.detect_ocr_async_results
                if result.get("point") == point
            ]

    def wait_detection_ocr_early_captures(self, point):
        count = max(0, int(self.detect_ocr_early_count))
        wait_timeout = max(0.0, float(self.detect_ocr_async_join_timeout))
        if count > 0:
            wait_timeout = max(
                wait_timeout,
                self.detect_ocr_capture_timeout * count
                + self.detect_ocr_early_interval * max(0, count - 1)
                + 0.2)
        results = self.collect_detection_ocr_async_results(
            point, wait_timeout=wait_timeout)
        rospy.loginfo(
            "[DETECT_OCR][EARLY_CAPTURE_READY] point=%s cached=%d wait=%.2fs",
            str(point), len(results), wait_timeout)
        return results

    def run_pending_early_ocr_result(self, point, early_result):
        if not early_result.get("pending_ocr"):
            return early_result
        image_path = early_result.get("image_path")
        source = early_result.get("source", "early_capture")
        rospy.loginfo(
            "[DETECT_OCR][EARLY_CHECK_START] point=%s source=%s path=%s",
            str(point), str(source), str(image_path))
        result = self.run_ocr_matcher(image_path, source)
        result["point"] = point
        result["pending_ocr"] = False
        result["sequence"] = early_result.get("sequence", 0)
        early_result.clear()
        early_result.update(result)
        rospy.loginfo(
            "[DETECT_OCR][EARLY_CHECK_DONE] point=%s source=%s valid=%s empty=%s answer=%s score=%.4f",
            str(point), str(source), str(result.get("valid")),
            str(result.get("empty")), str(result.get("answer")),
            result.get("score", 0.0))
        return early_result

    def select_detection_ocr_result(self, point, final_result):
        async_results = self.collect_detection_ocr_async_results(point)
        if final_result is not None and final_result.get("valid"):
            rospy.loginfo("[DETECT_OCR][USE_FINAL] point=%s answer=%s score=%.4f",
                          str(point), str(final_result.get("answer")),
                          final_result.get("score", 0.0))
            return final_result
        async_results.sort(key=lambda item: item.get("sequence", 0))
        for early_result in async_results:
            checked = self.run_pending_early_ocr_result(point, early_result)
            if checked.get("valid"):
                rospy.loginfo(
                    "[DETECT_OCR][USE_EARLY] point=%s answer=%s score=%.4f source=%s",
                    str(point), str(checked.get("answer")),
                    checked.get("score", 0.0), str(checked.get("source")))
                return checked
        if final_result is not None:
            return final_result
        if async_results:
            async_results.sort(key=lambda item: item.get("score", 0.0), reverse=True)
            return async_results[0]
        return {
            "valid": False,
            "empty": True,
            "answer": None,
            "score": 0.0,
            "raw_text": u"",
            "reason": "no_ocr_result",
            "source": "none",
            "point": point
        }

    def call_detection_ocr_priority(self, point, reason):
        self.wait_detection_ocr_early_captures(point)
        early_interrupt = self.get_detection_ocr_interrupt_result(point)
        if early_interrupt is not None:
            rospy.logwarn(
                "[DETECT_OCR][SKIP_FINAL_CAPTURE_EARLY_VALID] point=%s answer=%s score=%.4f source=%s",
                str(point), str(early_interrupt.get("answer")),
                early_interrupt.get("score", 0.0),
                str(early_interrupt.get("source")))
            return early_interrupt
        final_result = self.run_detection_ocr_capture(point, "%s_final" % reason)
        selected = self.select_detection_ocr_result(point, final_result)
        rospy.loginfo(
            "[DETECT_OCR][SELECT] point=%s valid=%s empty=%s answer=%s score=%.4f source=%s reason=%s",
            str(point), str(selected.get("valid")), str(selected.get("empty")),
            str(selected.get("answer")), selected.get("score", 0.0),
            str(selected.get("source")), str(selected.get("reason")))
        return selected

    # ---------------- 调用视觉检测服务 ----------------
    def call_fruit_detection_service(self):
        """
        功能：调用视觉服务识别线索(返回数字1-9)
        """
        try:
            # 先触发相机保存当前画面，再让大模型读取最新图片。
            image_path = self.capture_detection_image("vlm_capture", snapshot=False)
            if image_path is None:
                rospy.logwarn("[DETECT_VLM][CAPTURE_FAILED]")
                return "无"
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
            if not self.check_localization_jump("start24_escape"):
                self.handle_localization_jump_protection(
                    "start24_escape", cancel_active_goal=False)
                continue
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
        if not self.check_localization_jump("goto_precheck"):
            self.handle_localization_jump_protection(
                "goto_precheck", cancel_active_goal=False)
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
        start_time = rospy.Time.now()
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if not self.check_localization_jump("goto:%s" % str(p)):
                protect_start = rospy.Time.now()
                self.handle_localization_jump_protection(
                    "goto:%s" % str(p), cancel_active_goal=True)
                pause = (rospy.Time.now() - protect_start).to_sec()
                start_time = start_time + rospy.Duration.from_sec(pause)
                self.reset_nav_feedback()
                self.move_base.send_goal(goal, self._done_cb, self._active_cb, self._feedback_cb)
                continue

            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed > timeout:
                self.cancel_move_base_goal("goto_timeout", GoalStatus.PREEMPTED)
                rospy.loginfo("导航超时，取消目标")
                return False

            state = self.move_base.get_state()
            self.last_move_base_state = state
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("到达目标点 %s 成功! " % p)
                return True
            if state in [GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.PREEMPTED, GoalStatus.RECALLED]:
                rospy.logwarn("导航未成功到达目标点 %s，state=%s" %
                              (p, state))
                return False
            rate.sleep()

        self.cancel_move_base_goal("goto_shutdown", GoalStatus.PREEMPTED)
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
        if not self.check_localization_jump("task_nav_precheck:%s:%s" % (label, mode)):
            self.handle_localization_jump_protection(
                "task_nav_precheck:%s:%s" % (label, mode),
                cancel_active_goal=False)
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
                if not self.check_localization_jump("task_nav:%s:%s" % (label, mode)):
                    protect_start = rospy.Time.now()
                    self.handle_localization_jump_protection(
                        "task_nav:%s:%s" % (label, mode),
                        cancel_active_goal=True)
                    pause = (rospy.Time.now() - protect_start).to_sec()
                    start_time = start_time + rospy.Duration.from_sec(pause)
                    last_progress_time = rospy.Time.now()
                    best_dist = None
                    self.reset_nav_feedback()
                    self.task_nav_plan_fail_cancel_requested = False
                    self.task_nav_plan_fail_seen = 0
                    self.task_nav_plan_fail_window_start = rospy.Time(0)
                    self.task_nav_goal_active = True
                    self.move_base.send_goal(goal, self._done_cb, self._active_cb, self._feedback_cb)
                    continue

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
        def early_ocr_interrupt():
            return self.detection_ocr_interrupt_requested(point)
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
                    if not use_dynamic_yaw:
                        self.start_detection_ocr_burst_async(
                            point, "fixed_yaw_prealign")
                    break

            if self.detect_yaw_align_at_prealign and prealign_ok:
                if early_ocr_interrupt():
                    return True
                prealign_yaw = target[2]
                if use_dynamic_yaw and capture_at_current_pose:
                    prealign_yaw, _ = self.detection_yaw_from_current_pose(
                        point, target[2], "prealign_current")
                self.align_detection_yaw(prealign_yaw, interrupt_check=early_ocr_interrupt)
                if early_ocr_interrupt():
                    return True

        if capture_at_current_pose:
            rospy.loginfo(
                "[DETECT_NAV][CAPTURE_AT_PREALIGN] point=%s mode=%s distance=%.3f reason=dynamic_photo_target",
                str(point), selected_mode, selected_distance)
        elif self.detect_locked_final_approach:
            if early_ocr_interrupt():
                return True
            if not prealign_ok:
                rospy.logwarn("检测点%s预对准未确认成功" % point)
                if self.detect_skip_capture_on_nav_fail:
                    rospy.logwarn("[DETECT_NAV][SKIP_CAPTURE] point=%s reason=prealign_failed", str(point))
                    return False
            if not self.locked_approach_detection_point(
                    target[2], mode=selected_mode, distance=selected_distance,
                    interrupt_check=early_ocr_interrupt):
                if early_ocr_interrupt():
                    return True
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

        if early_ocr_interrupt():
            return True
        if self.detect_yaw_align_at_photo:
            photo_yaw = target[2]
            if use_dynamic_yaw:
                photo_yaw, _ = self.detection_yaw_from_current_pose(
                    point, target[2], "photo_current")
            self.align_detection_yaw(photo_yaw, interrupt_check=early_ocr_interrupt)
            if early_ocr_interrupt():
                return True
        if self.detect_photo_settle_time > 0:
            if not self.sleep_with_detection_ocr_interrupt(
                    self.detect_photo_settle_time, point, "photo_settle"):
                return True
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
                          str(point), source, self.log_text(detect_result))
            return False

        try:
            task_id = int(normalized)
        except ValueError:
            rospy.logwarn("检测结果不是有效数字: %s" %
                          self.log_text(detect_result))
            return False

        if task_id not in VLM_TO_TASK:
            rospy.logwarn("任务编号超出范围: %s" % task_id)
            return False

        mapped_id = VLM_TO_TASK[task_id]
        task_numbers.append(mapped_id)
        rospy.loginfo("收集到任务编号: %s (原始VLM: %s)" % (mapped_id, task_id))
        tts_text = u"已检测第%d条线索为%d号" % (clue, task_id)
        self.tts_client_async(
            tts_text,
            "detect_clue_%d_%d" % (clue, task_id)
        )
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
            self.reset_detection_ocr_async(point)
            # 首扫固定使用 goalListYaw + OCR题库匹配；动态YAW只作为OCR失败后的大模型保底。
            detect_nav_ok = self.goto_detection_point(point, use_dynamic_yaw=False)
            early_ocr_result = self.get_detection_ocr_interrupt_result(point)
            if early_ocr_result is not None:
                if self.handle_detection_result(
                        point, str(early_ocr_result.get("answer")),
                        "fixed_yaw_prealign_ocr"):
                    rospy.logwarn(
                        "[DETECT_OCR][EARLY_SUCCESS_SKIP_FINAL] point=%s answer=%s score=%.4f source=%s",
                        str(point), str(early_ocr_result.get("answer")),
                        early_ocr_result.get("score", 0.0),
                        str(early_ocr_result.get("source")))
                    return True
            if not detect_nav_ok:
                rospy.logwarn("[DETECT_NAV][MISSION_SKIP] point=%s reason=navigation_failed", str(point))
                return False

            if self.detect_ocr_enabled:
                ocr_result = self.call_detection_ocr_priority(point, "fixed_yaw_ocr")
                if ocr_result.get("valid"):
                    if self.handle_detection_result(
                            point, str(ocr_result.get("answer")), "fixed_yaw_ocr"):
                        return True
                rospy.logwarn(
                    "[DETECT_OCR][FALLBACK_TO_VLM] point=%s empty=%s score=%.4f reason=%s raw=%s",
                    str(point), str(ocr_result.get("empty")),
                    ocr_result.get("score", 0.0),
                    str(ocr_result.get("reason")),
                    self.log_text(ocr_result.get("raw_text", u"")))
            else:
                detect_result = self.call_fruit_detection_service()
                rospy.loginfo("当前检测点%s固定YAW大模型扫描结果: %s" % (point, detect_result))
                if self.handle_detection_result(point, detect_result, "fixed_yaw_vlm"):
                    return True
                if not self.is_no_detection_result(detect_result):
                    return True

            if not self.should_run_dynamic_yaw_fallback(point):
                rospy.loginfo(
                    "[DETECT_YAW][FALLBACK_SKIP] point=%s reason=disabled_or_no_photo_target",
                    str(point)
                )
                if self.detect_ocr_enabled:
                    rospy.logwarn(
                        "[DETECT_VLM][FIXED_FALLBACK_START] point=%s reason=no_dynamic_yaw_target",
                        str(point))
                    fixed_vlm_result = self.call_fruit_detection_service()
                    rospy.loginfo("当前检测点%s固定YAW大模型兜底结果: %s" %
                                  (point, fixed_vlm_result))
                    self.handle_detection_result(point, fixed_vlm_result, "fixed_yaw_vlm_fallback")
                return True

            rospy.logwarn(
                "[DETECT_YAW][FALLBACK_START] point=%s reason=ocr_no_valid_match",
                str(point)
            )
            fallback_nav_ok = self.goto_detection_point(point, use_dynamic_yaw=True)
            if not fallback_nav_ok:
                rospy.logwarn(
                    "[DETECT_YAW][FALLBACK_SKIP_CAPTURE] point=%s reason=navigation_failed",
                    str(point)
                )
                return True
            early_ocr_result = self.get_detection_ocr_interrupt_result(point)
            if early_ocr_result is not None:
                if self.handle_detection_result(
                        point, str(early_ocr_result.get("answer")),
                        "fixed_yaw_prealign_ocr_late"):
                    rospy.logwarn(
                        "[DETECT_OCR][EARLY_SUCCESS_SKIP_DYNAMIC_VLM] point=%s answer=%s score=%.4f source=%s",
                        str(point), str(early_ocr_result.get("answer")),
                        early_ocr_result.get("score", 0.0),
                        str(early_ocr_result.get("source")))
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

    def parse_task_id_list_param(self, raw_text, label):
        parsed_tasks = set()
        raw_text = str(raw_text).strip()
        if not raw_text:
            return parsed_tasks
        if raw_text.lower() in ["all", "*", "any"]:
            return None

        for item in raw_text.replace(";", ",").split(","):
            item = item.strip()
            if not item:
                continue
            if item.lower() in ["all", "*", "any"]:
                return None
            try:
                raw_id = int(item)
            except ValueError:
                rospy.logwarn("[%s] task id invalid: %s", label, item)
                continue

            if 1 <= raw_id <= 9:
                task_id = raw_id
            elif raw_id in VLM_TO_TASK:
                task_id = VLM_TO_TASK[raw_id]
            else:
                rospy.logwarn("[%s] task id out of range: %s", label, raw_id)
                continue
            parsed_tasks.add(task_id)

        return parsed_tasks

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

    def task_id_to_xy(self, task_id):
        if not (1 <= int(task_id) <= 9):
            return None
        if "goals" not in globals() or int(task_id) >= len(goals):
            return None
        target = goals[int(task_id)]
        return float(target[0]), float(target[1])

    def current_xy_for_task_order(self):
        pose = self.current_map_pose_for_plan()
        if pose is None:
            return None
        return float(pose.pose.position.x), float(pose.pose.position.y)

    def final_xy_for_task_order(self):
        final_target = self.final_target_for_task_order()
        if final_target is None:
            return None
        return float(final_target[0]), float(final_target[1])

    def final_target_for_task_order(self):
        if "goals" not in globals() or self.task_nav_order_final_goal_index >= len(goals):
            return None
        final_target = goals[self.task_nav_order_final_goal_index]
        if (self.task_nav_order_use_final_prealign
                and self.final_prealign_enabled
                and self.final_prealign_distance > 0.0):
            final_target = self.make_final_prealign_goal(final_target)
        return list(final_target)

    def xy_distance(self, a, b):
        return np.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def pose_stamped_from_nav_target(self, nav_target):
        return self.task_goal_pose_for_plan(nav_target)

    def pose_key_for_order(self, pose):
        yaw = 0.0
        try:
            quat = [
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w
            ]
            yaw = euler_from_quaternion(quat)[2] * 180.0 / pi
        except Exception:
            yaw = 0.0
        return (
            round(float(pose.pose.position.x), 3),
            round(float(pose.pose.position.y), 3),
            round(yaw, 1)
        )

    def plan_path_length(self, poses):
        if len(poses) < 2:
            return 0.0
        length = 0.0
        for i in range(1, len(poses)):
            x0 = poses[i - 1].pose.position.x
            y0 = poses[i - 1].pose.position.y
            x1 = poses[i].pose.position.x
            y1 = poses[i].pose.position.y
            length += np.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)
        return length

    def costmap_path_score(self, costmap, poses):
        if costmap is None or len(poses) <= 0:
            return {
                "avg_cost": 0.0,
                "max_cost": 0,
                "unknown_count": 0,
                "blocked_count": 0,
                "blocked": False
            }

        total_cost = 0.0
        count = 0
        max_cost = 0
        unknown_count = 0
        blocked_count = 0
        for pose in poses:
            cost, detail = self.costmap_cost_at(
                costmap,
                pose.pose.position.x,
                pose.pose.position.y)
            if cost is None or cost < 0:
                unknown_count += 1
                if self.task_nav_approach_reject_unknown:
                    blocked_count += 1
                cost = 100
            elif cost > self.task_nav_approach_cost_threshold:
                blocked_count += 1
            max_cost = max(max_cost, int(cost))
            total_cost += float(max(0, int(cost)))
            count += 1

        avg_cost = total_cost / float(count) if count > 0 else 0.0
        return {
            "avg_cost": avg_cost,
            "max_cost": max_cost,
            "unknown_count": unknown_count,
            "blocked_count": blocked_count,
            "blocked": blocked_count > 0
        }

    def make_plan_for_order(self, start_pose, nav_target, cache, label):
        key = (self.pose_key_for_order(start_pose),
               round(float(nav_target[0]), 3),
               round(float(nav_target[1]), 3),
               round(float(nav_target[2]), 1))
        if key in cache:
            return cache[key]

        client = self.get_task_make_plan_client()
        if client is None:
            result = None, "make_plan_service_unavailable"
            cache[key] = result
            return result

        request = GetPlanRequest()
        request.start = start_pose
        request.start.header.stamp = rospy.Time.now()
        request.goal = self.task_goal_pose_for_plan(nav_target)
        request.tolerance = 0.0
        try:
            response = client(request)
        except Exception as e:
            self.task_nav_make_plan_client = None
            result = None, "make_plan_failed:%s" % str(e)
            cache[key] = result
            return result

        poses = response.plan.poses
        if len(poses) < 2:
            result = None, "no_plan"
            cache[key] = result
            return result

        result = poses, "ok"
        cache[key] = result
        return result

    def task_order_nav_target_clear(self, costmap, nav_target):
        if costmap is None:
            return True, "no_costmap"
        cost, detail = self.costmap_cost_at(costmap, nav_target[0], nav_target[1])
        if cost is None:
            return False, detail
        if cost < 0:
            if self.task_nav_approach_reject_unknown:
                return False, "unknown"
            return True, "unknown_allowed"
        if cost > self.task_nav_approach_cost_threshold:
            return False, "cost=%d>threshold=%d" % (
                cost, self.task_nav_approach_cost_threshold)
        return True, "cost=%d" % cost

    def evaluate_task_order_nav_target(self, start_pose, nav_target, costmap, cache, label):
        end_pose = self.pose_stamped_from_nav_target(nav_target)
        target_clear, target_reason = self.task_order_nav_target_clear(costmap, nav_target)
        if not target_clear:
            return {
                "valid": False,
                "end_pose": end_pose,
                "score": self.task_nav_order_no_plan_penalty,
                "length": 0.0,
                "max_angle_deg": 999.0,
                "sharp_count": 1,
                "no_plan_count": 1,
                "blocked_count": 1,
                "avg_cost": 100.0,
                "max_cost": 100,
                "unknown_count": 0,
                "reason": "target_blocked:%s" % target_reason,
                "mode": label,
                "points": 0
            }

        poses, plan_reason = self.make_plan_for_order(start_pose, nav_target, cache, label)
        if poses is None:
            return {
                "valid": not self.task_nav_order_reject_no_plan,
                "end_pose": end_pose,
                "score": self.task_nav_order_no_plan_penalty,
                "length": 0.0,
                "max_angle_deg": 999.0,
                "sharp_count": 0,
                "no_plan_count": 1,
                "blocked_count": 0,
                "avg_cost": 0.0,
                "max_cost": 0,
                "unknown_count": 0,
                "reason": plan_reason,
                "mode": label,
                "points": 0
            }

        path_ok, path_info = self.analyze_task_plan_sharp_turns(
            poses, log_warning=False)
        length = self.plan_path_length(poses)
        cost_info = self.costmap_path_score(costmap, poses)
        sharp_count = 0 if path_ok else 1
        blocked_count = cost_info.get("blocked_count", 0)
        score = length
        score += cost_info.get("avg_cost", 0.0) * self.task_nav_order_path_cost_weight
        score += cost_info.get("max_cost", 0) * self.task_nav_order_path_max_cost_weight
        score += cost_info.get("unknown_count", 0) * self.task_nav_order_unknown_penalty
        if not path_ok:
            score += self.task_nav_order_sharp_turn_penalty
        if cost_info.get("blocked", False):
            score += self.task_nav_order_no_plan_penalty * 0.5

        valid = True
        if not path_ok and self.task_nav_order_reject_sharp_turns:
            valid = False
        if cost_info.get("blocked", False):
            valid = False

        return {
            "valid": valid,
            "end_pose": end_pose,
            "score": score,
            "length": length,
            "max_angle_deg": path_info.get("max_angle_deg", 0.0),
            "sharp_count": sharp_count,
            "no_plan_count": 0,
            "blocked_count": blocked_count,
            "avg_cost": cost_info.get("avg_cost", 0.0),
            "max_cost": cost_info.get("max_cost", 0),
            "unknown_count": cost_info.get("unknown_count", 0),
            "reason": "ok" if path_ok else "sharp_turn",
            "mode": label,
            "points": len(poses)
        }

    def evaluate_task_order_leg_to_task(self, start_pose, task_entry, costmap, cache):
        target = goals[task_entry["task_id"]]
        if self.task_nav_order_use_approach_candidates:
            candidates = self.make_task_approach_goals(target, log_candidates=False)
        else:
            candidates = [("target", list(target))]

        best_valid = None
        best_fallback = None
        for mode, nav_target in candidates:
            leg = self.evaluate_task_order_nav_target(
                start_pose,
                nav_target,
                costmap,
                cache,
                "%s:%d" % (mode, task_entry["task_id"]))
            leg["task_id"] = task_entry["task_id"]
            leg["nav_target"] = list(nav_target)
            if leg["valid"]:
                if best_valid is None or leg["score"] < best_valid["score"]:
                    best_valid = leg
            else:
                if best_fallback is None or leg["score"] < best_fallback["score"]:
                    best_fallback = leg

        if best_valid is not None:
            return best_valid
        return best_fallback

    def evaluate_task_order_candidate(self, entries, order_indices, start_pose,
                                      final_target, costmap, cache):
        current_pose = start_pose
        order_tasks = [entries[idx]["task_id"] for idx in order_indices]
        total_score = 0.0
        total_length = 0.0
        max_angle = 0.0
        sharp_count = 0
        no_plan_count = 0
        blocked_count = 0
        unknown_count = 0
        valid = True
        leg_summaries = []

        for entry_index in order_indices:
            leg = self.evaluate_task_order_leg_to_task(
                current_pose, entries[entry_index], costmap, cache)
            if leg is None:
                valid = False
                total_score += self.task_nav_order_no_plan_penalty
                no_plan_count += 1
                leg_summaries.append("task%d:no_candidate" % entries[entry_index]["task_id"])
                current_pose = self.pose_stamped_from_nav_target(
                    goals[entries[entry_index]["task_id"]])
                continue

            total_score += leg["score"]
            total_length += leg["length"]
            max_angle = max(max_angle, leg["max_angle_deg"])
            sharp_count += leg["sharp_count"]
            no_plan_count += leg["no_plan_count"]
            blocked_count += leg["blocked_count"]
            unknown_count += leg["unknown_count"]
            valid = valid and leg["valid"]
            leg_summaries.append(
                "task%d/%s len=%.2f angle=%.1f cost=%.1f max=%d reason=%s" % (
                    leg["task_id"], leg["mode"], leg["length"],
                    leg["max_angle_deg"], leg["avg_cost"],
                    leg["max_cost"], leg["reason"]))
            current_pose = leg["end_pose"]

        final_leg = self.evaluate_task_order_nav_target(
            current_pose,
            final_target,
            costmap,
            cache,
            "final")
        total_score += final_leg["score"]
        total_length += final_leg["length"]
        max_angle = max(max_angle, final_leg["max_angle_deg"])
        sharp_count += final_leg["sharp_count"]
        no_plan_count += final_leg["no_plan_count"]
        blocked_count += final_leg["blocked_count"]
        unknown_count += final_leg["unknown_count"]
        valid = valid and final_leg["valid"]
        leg_summaries.append(
            "final len=%.2f angle=%.1f cost=%.1f max=%d reason=%s" % (
                final_leg["length"], final_leg["max_angle_deg"],
                final_leg["avg_cost"], final_leg["max_cost"],
                final_leg["reason"]))

        return {
            "order_indices": list(order_indices),
            "order_tasks": order_tasks,
            "valid": valid,
            "score": total_score,
            "length": total_length,
            "max_angle_deg": max_angle,
            "sharp_count": sharp_count,
            "no_plan_count": no_plan_count,
            "blocked_count": blocked_count,
            "unknown_count": unknown_count,
            "legs": "; ".join(leg_summaries)
        }

    def order_index_permutations(self, valid_entries, start_xy, final_xy):
        all_indices = list(range(len(valid_entries)))
        first_index = None
        last_index = None
        if self.task_nav_order_pin_nearest_first:
            first_index = min(
                all_indices,
                key=lambda idx: self.xy_distance(start_xy, valid_entries[idx]["xy"]))
        if self.task_nav_order_pin_nearest_final_last and len(valid_entries) > 1:
            last_candidates = [
                idx for idx in all_indices
                if idx != first_index or len(valid_entries) == 1
            ]
            if not last_candidates:
                last_candidates = list(all_indices)
            last_index = min(
                last_candidates,
                key=lambda idx: self.xy_distance(final_xy, valid_entries[idx]["xy"]))

        middle_indices = [
            idx for idx in all_indices
            if idx not in [first_index, last_index]
        ]
        if len(middle_indices) > self.task_nav_order_max_bruteforce:
            return [self.greedy_task_order_indices(
                valid_entries, all_indices, first_index, last_index, start_xy)], "greedy"

        candidates = []
        for middle_perm in itertools.permutations(middle_indices):
            candidate = []
            if first_index is not None:
                candidate.append(first_index)
            candidate.extend(list(middle_perm))
            if last_index is not None:
                candidate.append(last_index)
            for idx in all_indices:
                if idx not in candidate:
                    candidate.append(idx)
            candidates.append(candidate)
        return candidates, "bruteforce_path"

    def optimize_task_order_by_plan(self, tasks, valid_entries, invalid_tasks,
                                    start_pose, final_target, start_xy, final_xy):
        if not self.wait_for_make_plan_idle("task_order"):
            rospy.logwarn("[TASK_ORDER][PATH_QUALITY_SKIP] reason=move_base_active")
            return None

        costmap = self.get_global_costmap_for_approach()
        if costmap is None:
            rospy.logwarn(
                "[TASK_ORDER][PATH_QUALITY_COSTMAP_MISSING] use_make_plan_only=true")

        candidates, method = self.order_index_permutations(
            valid_entries, start_xy, final_xy)
        cache = {}
        scored = []
        for candidate in candidates:
            result = self.evaluate_task_order_candidate(
                valid_entries,
                candidate,
                start_pose,
                final_target,
                costmap,
                cache)
            scored.append(result)
            raw_order = [TASK_TO_VLM.get(task_id, task_id)
                         for task_id in result["order_tasks"]]
            rospy.loginfo(
                "[TASK_ORDER][CANDIDATE] method=%s order=%s raw=%s valid=%s score=%.3f length=%.3f max_angle=%.1f sharp=%d no_plan=%d blocked=%d unknown=%d legs=%s",
                method,
                str(result["order_tasks"]),
                str(raw_order),
                str(result["valid"]),
                result["score"],
                result["length"],
                result["max_angle_deg"],
                result["sharp_count"],
                result["no_plan_count"],
                result["blocked_count"],
                result["unknown_count"],
                result["legs"]
            )

        if not scored:
            return None

        scored.sort(key=lambda item: (
            0 if item["valid"] else 1,
            item["sharp_count"],
            item["no_plan_count"],
            item["blocked_count"],
            item["score"],
            item["length"]
        ))
        best = scored[0]
        optimized_tasks = list(best["order_tasks"])
        optimized_tasks.extend(invalid_tasks)
        original_raw = [TASK_TO_VLM.get(task_id, task_id) for task_id in tasks]
        optimized_raw = [TASK_TO_VLM.get(task_id, task_id) for task_id in optimized_tasks]
        if best["valid"]:
            rospy.loginfo(
                "[TASK_ORDER][OPTIMIZED_PATH] original=%s raw=%s optimized=%s raw=%s score=%.3f length=%.3f max_angle=%.1f sharp=%d no_plan=%d blocked=%d start=(%.3f,%.3f) final=(%.3f,%.3f)",
                str(tasks), str(original_raw),
                str(optimized_tasks), str(optimized_raw),
                best["score"], best["length"], best["max_angle_deg"],
                best["sharp_count"], best["no_plan_count"], best["blocked_count"],
                start_xy[0], start_xy[1], final_xy[0], final_xy[1])
        else:
            rospy.logwarn(
                "[TASK_ORDER][NO_FULLY_SAFE_ORDER] original=%s raw=%s fallback=%s raw=%s score=%.3f length=%.3f max_angle=%.1f sharp=%d no_plan=%d blocked=%d start=(%.3f,%.3f) final=(%.3f,%.3f)",
                str(tasks), str(original_raw),
                str(optimized_tasks), str(optimized_raw),
                best["score"], best["length"], best["max_angle_deg"],
                best["sharp_count"], best["no_plan_count"], best["blocked_count"],
                start_xy[0], start_xy[1], final_xy[0], final_xy[1])
        return optimized_tasks

    def score_task_order_indices(self, entries, order_indices, start_xy, final_xy):
        points_xy = [start_xy]
        for entry_index in order_indices:
            points_xy.append(entries[entry_index]["xy"])
        points_xy.append(final_xy)

        distance_score = 0.0
        for i in range(len(points_xy) - 1):
            distance_score += self.xy_distance(points_xy[i], points_xy[i + 1])

        turn_penalty = 0.0
        if self.task_nav_order_turn_penalty_weight > 0.0:
            for i in range(1, len(points_xy) - 1):
                ax = points_xy[i][0] - points_xy[i - 1][0]
                ay = points_xy[i][1] - points_xy[i - 1][1]
                bx = points_xy[i + 1][0] - points_xy[i][0]
                by = points_xy[i + 1][1] - points_xy[i][1]
                a_len = np.sqrt(ax * ax + ay * ay)
                b_len = np.sqrt(bx * bx + by * by)
                if a_len < 1e-6 or b_len < 1e-6:
                    continue
                dot = max(-1.0, min(1.0, (ax * bx + ay * by) / (a_len * b_len)))
                turn_penalty += np.arccos(dot) * self.task_nav_order_turn_penalty_weight

        return distance_score + turn_penalty, distance_score, turn_penalty

    def greedy_task_order_indices(self, entries, all_indices, first_index, last_index, start_xy):
        remaining = [idx for idx in all_indices if idx not in [first_index, last_index]]
        ordered = []
        current_xy = start_xy
        if first_index is not None:
            ordered.append(first_index)
            current_xy = entries[first_index]["xy"]

        while remaining:
            next_index = min(
                remaining,
                key=lambda idx: self.xy_distance(current_xy, entries[idx]["xy"]))
            ordered.append(next_index)
            current_xy = entries[next_index]["xy"]
            remaining.remove(next_index)

        if last_index is not None:
            ordered.append(last_index)
        return ordered

    def optimize_task_order(self, tasks):
        if not self.task_nav_optimize_order or len(tasks) <= 1:
            return list(tasks)

        start_pose = self.current_map_pose_for_plan()
        final_target = self.final_target_for_task_order()
        if start_pose is None or final_target is None:
            rospy.logwarn(
                "[TASK_ORDER][SKIP] reason=no_anchor_pose original=%s",
                str(tasks)
            )
            return list(tasks)
        start_xy = (
            float(start_pose.pose.position.x),
            float(start_pose.pose.position.y)
        )
        final_xy = (float(final_target[0]), float(final_target[1]))

        valid_entries = []
        invalid_tasks = []
        for original_index, task_id in enumerate(tasks):
            xy = self.task_id_to_xy(task_id)
            if xy is None:
                invalid_tasks.append(task_id)
                continue
            valid_entries.append({
                "original_index": original_index,
                "task_id": int(task_id),
                "xy": xy
            })

        if len(valid_entries) <= 1:
            return list(tasks)

        if self.task_nav_order_path_quality_enabled:
            optimized_by_plan = self.optimize_task_order_by_plan(
                tasks,
                valid_entries,
                invalid_tasks,
                start_pose,
                final_target,
                start_xy,
                final_xy)
            if optimized_by_plan is not None:
                return optimized_by_plan
            rospy.logwarn(
                "[TASK_ORDER][PATH_QUALITY_FALLBACK_GEOMETRY] original=%s",
                str(tasks)
            )

        all_indices = range(len(valid_entries))
        first_index = None
        last_index = None
        if self.task_nav_order_pin_nearest_first:
            first_index = min(
                all_indices,
                key=lambda idx: self.xy_distance(start_xy, valid_entries[idx]["xy"]))
        if self.task_nav_order_pin_nearest_final_last and len(valid_entries) > 1:
            last_candidates = [
                idx for idx in all_indices
                if idx != first_index or len(valid_entries) == 1
            ]
            if not last_candidates:
                last_candidates = list(all_indices)
            last_index = min(
                last_candidates,
                key=lambda idx: self.xy_distance(final_xy, valid_entries[idx]["xy"]))

        middle_indices = [
            idx for idx in all_indices
            if idx not in [first_index, last_index]
        ]
        if len(middle_indices) > self.task_nav_order_max_bruteforce:
            best_order = self.greedy_task_order_indices(
                valid_entries, list(all_indices), first_index, last_index, start_xy)
            best_score, best_distance, best_turn = self.score_task_order_indices(
                valid_entries, best_order, start_xy, final_xy)
            method = "greedy"
        else:
            best_order = None
            best_score = None
            best_distance = None
            best_turn = None
            for middle_perm in itertools.permutations(middle_indices):
                candidate = []
                if first_index is not None:
                    candidate.append(first_index)
                candidate.extend(list(middle_perm))
                if last_index is not None:
                    candidate.append(last_index)
                for idx in all_indices:
                    if idx not in candidate:
                        candidate.append(idx)

                score, distance_score, turn_score = self.score_task_order_indices(
                    valid_entries, candidate, start_xy, final_xy)
                if best_score is None or score < best_score:
                    best_score = score
                    best_distance = distance_score
                    best_turn = turn_score
                    best_order = candidate
            method = "bruteforce"

        optimized_tasks = [valid_entries[idx]["task_id"] for idx in best_order]
        optimized_tasks.extend(invalid_tasks)
        original_raw = [TASK_TO_VLM.get(task_id, task_id) for task_id in tasks]
        optimized_raw = [TASK_TO_VLM.get(task_id, task_id) for task_id in optimized_tasks]
        rospy.loginfo(
            "[TASK_ORDER][OPTIMIZED] method=%s original=%s raw=%s optimized=%s raw=%s start=(%.3f,%.3f) final=(%.3f,%.3f) first=%s last=%s score=%.3f distance=%.3f turn_penalty=%.3f",
            method,
            str(tasks),
            str(original_raw),
            str(optimized_tasks),
            str(optimized_raw),
            start_xy[0], start_xy[1],
            final_xy[0], final_xy[1],
            str(valid_entries[first_index]["task_id"]) if first_index is not None else "None",
            str(valid_entries[last_index]["task_id"]) if last_index is not None else "None",
            best_score if best_score is not None else 0.0,
            best_distance if best_distance is not None else 0.0,
            best_turn if best_turn is not None else 0.0
        )
        return optimized_tasks

    def announce_task_arrival(self, idx, task_id, return_gate=False):
        raw_id = TASK_TO_VLM.get(task_id, task_id)
        tts_text = u"已到达任务点%d号" % raw_id
        tts_start_time = rospy.Time.now()
        start_event = threading.Event()
        start_time_holder = {}
        tts_ok = self.tts_client_async(
            tts_text,
            "task_arrival_%d_%d" % (idx + 1, raw_id),
            start_event=start_event,
            start_time_holder=start_time_holder
        )
        rospy.loginfo("[TASK_TIME][TTS_QUEUE] idx=%d task_id=%d dt=%.2fs queued=%s",
                      idx + 1, task_id,
                      (rospy.Time.now() - tts_start_time).to_sec(),
                      str(tts_ok))
        if return_gate:
            return {
                "queued": tts_ok,
                "start_event": start_event,
                "start_time_holder": start_time_holder,
                "label": "task_arrival_%d_%d" % (idx + 1, raw_id)
            }
        return tts_ok

    def wait_task_arrival_tts_delay_before_next_nav(self, tts_gate, idx, task_id, reason):
        delay = max(0.0, float(self.task_arrival_nav_delay_after_tts_start))
        if delay <= 0.0:
            return True
        if not tts_gate or not tts_gate.get("queued"):
            rospy.logwarn(
                "[TASK_TIME][TTS_NAV_DELAY_SKIP] idx=%d task_id=%d reason=%s queued=false",
                idx + 1, task_id, reason)
            return False

        start_event = tts_gate.get("start_event")
        start_time_holder = tts_gate.get("start_time_holder", {})
        wait_timeout = max(0.0, float(self.task_arrival_tts_start_wait_timeout))
        wait_start = time.time()
        while (start_event is not None
               and not start_event.is_set()
               and not rospy.is_shutdown()):
            if time.time() - wait_start >= wait_timeout:
                rospy.logwarn(
                    "[TASK_TIME][TTS_NAV_DELAY_START_TIMEOUT] idx=%d task_id=%d reason=%s wait=%.2fs label=%s",
                    idx + 1, task_id, reason, wait_timeout,
                    str(tts_gate.get("label")))
                return False
            rospy.sleep(0.02)

        tts_start_wall = start_time_holder.get("time", time.time())
        elapsed_after_start = time.time() - tts_start_wall
        remaining = delay - elapsed_after_start
        if remaining > 0.0:
            rospy.loginfo(
                "[TASK_TIME][TTS_NAV_DELAY_WAIT] idx=%d task_id=%d reason=%s remaining=%.2fs delay=%.2fs label=%s",
                idx + 1, task_id, reason, remaining, delay,
                str(tts_gate.get("label")))
            rospy.sleep(remaining)
        rospy.loginfo(
            "[TASK_TIME][TTS_NAV_DELAY_DONE] idx=%d task_id=%d reason=%s elapsed_after_start=%.2fs delay=%.2fs",
            idx + 1, task_id, reason, time.time() - tts_start_wall, delay)
        return True

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
                    tts_gate = self.announce_task_arrival(
                        idx, task_id, return_gate=True)
                    last_parking = None
                    last_task_id = task_id
                    self.wait_task_arrival_tts_delay_before_next_nav(
                        tts_gate, idx, task_id, "direct_done")
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
                tts_gate = self.announce_task_arrival(
                    idx, task_id, return_gate=True)

                # 播报开始后先留出固定时间，再逃逸离开挡板区域。
                self.wait_task_arrival_tts_delay_before_next_nav(
                    tts_gate, idx, task_id, "before_escape")
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
        self.check_start_localization()

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
                if not detect_ok:
                    rospy.logwarn(
                        "[DETECT_NAV][SKIP_CURRENT_CONTINUE] detect_idx=%d point=%s collected=%s",
                        detect_idx + 1, str(p), task_numbers)

            rospy.loginfo("\n=== 所有检测点处理完成 ===")
            rospy.loginfo("收集到的任务编号: %s" % task_numbers)

        # 按线索导航
        self.set_parking_phase_costmap()
        try:
            self.enable_obstacle_memory_after_parking("task_nav_start")
            if self.task_nav_optimize_order:
                task_numbers = self.optimize_task_order(task_numbers)
            else:
                rospy.loginfo("[TASK_ORDER][DISABLED] order=%s", str(task_numbers))
            self.go_to_task_positions()
        finally:
            self.disable_obstacle_memory_for_parking("task_phase_end")
            self.restore_cruise_costmap()

        # 终点按检测点思路处理：先到安全预对准点，再对齐yaw，最后交给激光闭环贴边。
        final_target = goals[16]
        final_adjust_target = final_target
        final_nav_goal = final_target
        final_nav_timeout = self.final_nav_timeout
        final_nav_stage = "TARGET"
        final_used_doudi = False
        final_adjust_side_laser_direction = self.final_side_laser_direction
        final_adjust_depth_laser_direction = self.final_depth_laser_direction
        if self.final_prealign_enabled and self.final_prealign_distance > 0.0:
            final_nav_goal = self.make_final_prealign_goal(final_target)
            final_nav_timeout = self.final_prealign_timeout
            final_nav_stage = "PREALIGN"
            rospy.loginfo(
                "[FINAL][PREALIGN_GOAL] mode=%s distance=%.3f target=(%.3f, %.3f, %.1f) goal=(%.3f, %.3f, %.1f)",
                self.final_prealign_mode,
                self.final_prealign_distance,
                final_target[0], final_target[1], final_target[2],
                final_nav_goal[0], final_nav_goal[1], final_nav_goal[2]
            )

        primary_final_nav_goal = list(final_nav_goal)
        if self.final_doudi_enabled:
            primary_plan_ok, primary_plan_reason = self.final_nav_make_plan_ok(
                primary_final_nav_goal,
                "primary")
            if primary_plan_ok is False:
                final_nav_goal, final_nav_timeout, final_adjust_side_laser_direction, \
                    final_adjust_depth_laser_direction = self.select_final_doudi_goal(
                        "primary_plan_failed:%s" % primary_plan_reason)
                final_adjust_target = final_nav_goal
                final_nav_stage = "DOUDI"
                final_used_doudi = True
            elif primary_plan_ok is None:
                rospy.logwarn(
                    "[FINAL][DOUDI_CHECK][SKIP] label=primary reason=%s, keep primary goal",
                    primary_plan_reason)

        final_nav_start = rospy.Time.now()
        final_nav_ok = self.goto(final_nav_goal, timeout=final_nav_timeout)
        rospy.loginfo("[FINAL][NAV_TO_%s] dt=%.2fs ok=%s timeout=%.1fs goal=(%.3f, %.3f, %.1f)",
                      final_nav_stage,
                      (rospy.Time.now() - final_nav_start).to_sec(),
                      str(final_nav_ok), final_nav_timeout,
                      final_nav_goal[0], final_nav_goal[1], final_nav_goal[2])

        if (not final_nav_ok) and (not final_used_doudi) and self.final_doudi_enabled:
            primary_plan_ok, primary_plan_reason = self.final_nav_make_plan_ok(
                primary_final_nav_goal,
                "primary_after_nav_fail")
            if primary_plan_ok is False:
                final_nav_goal, final_nav_timeout, final_adjust_side_laser_direction, \
                    final_adjust_depth_laser_direction = self.select_final_doudi_goal(
                        "primary_after_nav_fail:%s" % primary_plan_reason)
                final_adjust_target = final_nav_goal
                final_nav_stage = "DOUDI"
                final_used_doudi = True
                final_nav_start = rospy.Time.now()
                final_nav_ok = self.goto(final_nav_goal, timeout=final_nav_timeout)
                rospy.loginfo(
                    "[FINAL][NAV_TO_%s] dt=%.2fs ok=%s timeout=%.1fs goal=(%.3f, %.3f, %.1f)",
                    final_nav_stage,
                    (rospy.Time.now() - final_nav_start).to_sec(),
                    str(final_nav_ok), final_nav_timeout,
                    final_nav_goal[0], final_nav_goal[1], final_nav_goal[2])
            elif primary_plan_ok is None:
                rospy.logwarn(
                    "[FINAL][DOUDI_CHECK][SKIP] label=primary_after_nav_fail reason=%s, not a confirmed no-plan",
                    primary_plan_reason)
            else:
                rospy.loginfo(
                    "[FINAL][DOUDI_CHECK][KEEP_PRIMARY] primary_after_nav_fail reason=%s",
                    primary_plan_reason)

        final_yaw_ok = True
        if self.final_align_yaw_before_laser:
            final_yaw_ok = self.align_final_yaw(final_adjust_target[2])
            rospy.loginfo("[FINAL][YAW_ALIGN][DONE] ok=%s", str(final_yaw_ok))
        else:
            self.target_yaw = final_adjust_target[2] / 180.0 * pi

        rospy.loginfo(
            "[FINAL][ADJUST_POSITION][START] target_yaw=%.1fdeg side=%.3f depth=%.3f side_laser=%s depth_laser=%s doudi=%s",
            final_adjust_target[2], self.final_side_target, self.final_depth_target,
            final_adjust_side_laser_direction, final_adjust_depth_laser_direction,
            str(final_used_doudi))
        old_side_laser_direction = self.final_side_laser_direction
        old_depth_laser_direction = self.final_depth_laser_direction
        try:
            self.final_side_laser_direction = final_adjust_side_laser_direction
            self.final_depth_laser_direction = final_adjust_depth_laser_direction
            final_adjust_ok = self.adjust_position(
                side_target=self.final_side_target,
                back_target=self.final_depth_target)
        finally:
            self.final_side_laser_direction = old_side_laser_direction
            self.final_depth_laser_direction = old_depth_laser_direction
        rospy.loginfo("[FINAL][ADJUST_POSITION][DONE] ok=%s", str(final_adjust_ok))
        final_allow_depth_timeout_tts = (
            self.final_arrival_tts_on_depth_timeout
            and self.final_depth_hold_timed_out_after_cmd)
        if final_nav_ok and final_yaw_ok and final_allow_depth_timeout_tts and not final_adjust_ok:
            state = self.final_depth_hold_last_state
            if state is not None:
                rospy.logwarn(
                    "[FINAL][ARRIVAL_TTS_DEPTH_TIMEOUT_ALLOW] depth=%.3f target=%.3f err=%.3f side=%.3f target=%.3f err=%.3f yaw_err=%.3f",
                    state["depth"], self.final_depth_target, state["depth_error"],
                    state["side"], self.final_side_target, state["side_error"],
                    state["yaw_error"])
            else:
                rospy.logwarn("[FINAL][ARRIVAL_TTS_DEPTH_TIMEOUT_ALLOW] no_valid_laser_state")
        final_arrival_ok = final_yaw_ok and (
            final_adjust_ok
            or (final_nav_ok and final_allow_depth_timeout_tts)
        )
        if final_yaw_ok and final_adjust_ok and not final_nav_ok:
            rospy.logwarn(
                "[FINAL][ARRIVAL_NAV_TIMEOUT_ALLOWED] nav_ok=false yaw_ok=true adjust_ok=true, allow arrival TTS after laser final adjust"
            )
        if final_arrival_ok:
            tts_text = u"已到达终点"
            self.tts_client(tts_text)
        else:
            rospy.logwarn(
                "[FINAL][ARRIVAL_SUPPRESSED] nav_ok=%s yaw_ok=%s adjust_ok=%s depth_timeout_allow=%s, skip arrival TTS",
                str(final_nav_ok), str(final_yaw_ok), str(final_adjust_ok),
                str(final_allow_depth_timeout_tts))

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
    startup_audio_stabilize_wait = float(rospy.get_param("~startup_audio_stabilize_wait", 5.0))
    startup_audio_notice_before = float(rospy.get_param("~startup_audio_notice_before", 3.0))
    rospy.loginfo("IMU传感器已激活，%.1f秒后开始任务..." % startup_audio_stabilize_wait)

    # 6. 延时等待系统稳定；在启动播报前3秒给终端提示。
    notice_wait = max(0.0, startup_audio_stabilize_wait - startup_audio_notice_before)
    if notice_wait > 0.0:
        rospy.sleep(notice_wait)
    terminal_notice = "终端提示：%.1f秒后播报控制核心加载完毕" % startup_audio_notice_before
    print(terminal_notice)
    sys.stdout.flush()
    rospy.logwarn("[STARTUP_NOTICE] %s", terminal_notice)
    remaining_wait = max(0.0, min(startup_audio_notice_before, startup_audio_stabilize_wait))
    if remaining_wait > 0.0:
        rospy.sleep(remaining_wait)

    # 7. 播报离线音频并开始任务
    os.system('ffplay -nodisp -autoexit -loglevel quiet /home/abot/EIU0US/src/robot_slam/resources/startGame.wav')
    # navi.adjust_position(side_target=2.352, back_target=0.600) 
    navi.check_start_localization()
    navi.start24()
    navi.check_start_localization()
    navi.execute_mission()

    # 8. 保持节点运行
    rospy.spin()
