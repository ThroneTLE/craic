#!/usr/bin/env python2
# -*- coding: utf-8 -*-


# =================== 导入依赖库/ROS消息 ===================
import rospy
import actionlib
import numpy as np
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import math
import tf as tf_mod
from math import pi
from std_msgs.msg import String, Int32
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan, Imu
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
    12: "/home/abot/demo/src/robot_slam/mp3/01.mp3",
    13: "/home/abot/demo/src/robot_slam/mp3/02.mp3",
    14: "/home/abot/demo/src/robot_slam/mp3/03.mp3",
    15: "/home/abot/demo/src/robot_slam/mp3/04.mp3"
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
        self.tf_listener = tf_mod.TransformListener()

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
        self.is_adjusting = False
    
    def scan_callback(self, msg):
        """存储最新的激光雷达数据"""
        self.scan_data = msg

    def odom_callback(self, msg):
        """从里程计提取当前航向角"""
        orientation_q = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.current_yaw = yaw

    def normalize_angle(self, angle):
        """将角度归一化到[-π, π]范围内"""
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle
    
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
            rospy.sleep(0.5)
            # 调用服务并获取识别结果
            response = self.fruit_detection_service()
            rospy.loginfo("视觉大模型识别结果: %s" % response.message)
            return response.message
        except rospy.ServiceException as e:
            rospy.logerr("视觉大模型服务调用失败: %s" % e)
            return "无"

    # ---------------- 机器人终点动作(2/4) ----------------
    def start24(self):
        """起点动作，冲出障碍区：左上方直线斜移"""
        global time_val
        msg = Twist()
        msg.linear.x = 0.25    # X轴：前进
        msg.linear.y = 0.1     # Y轴：左移
        msg.angular.z = 0.0    # 无旋转
        # 持续发布速度指令1.3秒
        while time_val <= 13:
            self.pub.publish(msg)
            rospy.sleep(0.1)
            time_val += 1

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
        rospy.loginfo("导航完成! status=%s result=%s" % (status, result))
        self.arrive_pub.publish("arrived to target point")

    def _active_cb(self):
        """导航开始时自动调用"""
        rospy.loginfo("[Navi] 导航已激活")

    def _feedback_cb(self, feedback):
        """导航过程中实时反馈(无需处理)"""
        pass

    # ---------------- 核心：导航到目标点 ----------------
<<<<<<< HEAD
    def goto(self, p, timeout=60):
=======
    def goto(self, p, timeout=60, close_dist=None):
>>>>>>> d99393dcc1bc4b13118fcda0280ed972cd35cdff
        """
        功能：导航到指定坐标
        :param p: [x, y, 朝向角度]
        :param timeout: 超时秒数，默认60
<<<<<<< HEAD
=======
        :param close_dist: 靠近即收距离，None则必须等SUCCEEDED
>>>>>>> d99393dcc1bc4b13118fcda0280ed972cd35cdff
        """
        rospy.loginfo("[Navi] 前往目标点: %s (timeout=%ds)" % (p, timeout))
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

        self.move_base.send_goal(goal, self._done_cb, self._active_cb, self._feedback_cb)
<<<<<<< HEAD
        result = self.move_base.wait_for_result(rospy.Duration(timeout))
        if not result:
            self.move_base.cancel_goal()
            rospy.loginfo("导航超时，取消目标")
        else:
            if self.move_base.get_state() == GoalStatus.SUCCEEDED:
                rospy.loginfo("到达目标点 %s 成功! " % p)
=======

        if close_dist is not None:
            # 轮询模式：靠近即收
            start_time = rospy.Time.now()
            while not rospy.is_shutdown():
                elapsed = (rospy.Time.now() - start_time).to_sec()
                if elapsed > timeout:
                    rospy.loginfo("导航超时，取消目标")
                    self.move_base.cancel_goal()
                    return True
                if self.move_base.get_state() == GoalStatus.SUCCEEDED:
                    rospy.loginfo("到达目标点 %s 成功! (%.1fs)" % (p, elapsed))
                    return True
                try:
                    self.tf_listener.waitForTransform("map", "base_footprint", rospy.Time(0), rospy.Duration(0.2))
                    trans, _ = self.tf_listener.lookupTransform("map", "base_footprint", rospy.Time(0))
                    dx = p[0] - trans[0]
                    dy = p[1] - trans[1]
                    if math.sqrt(dx*dx + dy*dy) < close_dist:
                        rospy.logwarn("导航靠近目标点 %s (dist=%.3f < %.3f), 取消导航." % (p, math.sqrt(dx*dx+dy*dy), close_dist))
                        self.move_base.cancel_goal()
                        return True
                except:
                    pass
                rospy.sleep(0.1)
        else:
            result = self.move_base.wait_for_result(rospy.Duration(timeout))
            if not result:
                self.move_base.cancel_goal()
                rospy.loginfo("导航超时，取消目标")
            else:
                if self.move_base.get_state() == GoalStatus.SUCCEEDED:
                    rospy.loginfo("到达目标点 %s 成功! " % p)
>>>>>>> d99393dcc1bc4b13118fcda0280ed972cd35cdff
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
        self.goto(goals[point])

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

    # ---------------- 按线索导航到任务点 ----------------
    def go_to_task_positions(self):
        """按识别到的线索，依次导航到对应任务点"""
        rospy.loginfo("开始按顺序前往任务位置: %s" % task_numbers)
        # 先导航到中转点14
        # self.goto(goals[14])
        # 遍历所有线索
        for idx, task_id in enumerate(task_numbers):
            if 1 <= task_id <= 9:
<<<<<<< HEAD
                # 导航到线索对应的任务点 (5s 超时)
                self.goto(goals[task_id], timeout=5)
                rospy.sleep(1)
                # 启动精密停车
=======
                # 直接启动精密停车 (Phase 0 内部会用 move_base 尝试)
>>>>>>> d99393dcc1bc4b13118fcda0280ed972cd35cdff
                target = goals[task_id]
                rospy.loginfo("启动精密停车, 任务点 %d (x=%.3f y=%.3f yaw=%.1f)..." % (task_id, target[0], target[1], target[2]))
                parking = AutoSinglePointTest(target_x=target[0], target_y=target[1], target_yaw_deg=target[2])
                parking.run()
                rospy.sleep(0.5)
                # 语音播报到达任务点（用原始VLM识别编号）
                raw_id = TASK_TO_VLM.get(task_id, task_id)
                tts_text = u"已到达任务点%d号" % raw_id
                self.tts_client(tts_text)
                # 播报完毕，逃逸离开挡板区域
                parking.escape()
            else:
                rospy.logwarn("任务编号%s无效，跳过" % task_id)

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
        # 执行所有检测点任务
        for p in points:
            rospy.loginfo("\n=== 开始处理第%s个检测点 ===" % (1))
            self.recognize(p)

        rospy.loginfo("\n=== 所有检测点处理完成 ===")
        rospy.loginfo("收集到的任务编号: %s" % task_numbers)

        # 按线索导航
        self.go_to_task_positions()

        # 导航到终点
        self.goto(goals[16])
        self.adjust_position(side_target=0.220, back_target=0.240)  
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
    os.system('ffplay -nodisp -autoexit -loglevel quiet /home/abot/craic/src/robot_slam/resources/startGame.wav')
    # navi.adjust_position(side_target=2.352, back_target=0.600) 
    navi.start24()
    navi.execute_mission()

    # 8. 保持节点运行
    rospy.spin()