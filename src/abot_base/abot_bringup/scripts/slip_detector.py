#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
轮速里程计退化检测：对比 IMU，打滑时膨胀协方差，重新发布 odom。
不打滑 = 完全信任轮速计；打滑越严重 = 协方差越大 = EKF 越倾向 IMU。
"""
import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class SlipDetector:
    def __init__(self):
        # ---- 退化阈值 ----
        self.ang_threshold = rospy.get_param("~ang_threshold", 0.15)   # rad/s 差异阈值
        self.lin_threshold = rospy.get_param("~lin_threshold", 0.30)   # m/s^2 差异阈值

        # ---- 延时平滑（避免噪声触发误检测） ----
        self.smooth_factor = rospy.get_param("~smooth_factor", 0.7)    # 低通系数
        self.smoothed_score = 0.0

        # ---- 状态 ----
        self.last_wheel_vx = 0.0
        self.last_wheel_vyaw = 0.0
        self.last_wheel_time = None
        self.wheel_msg = None

        # ---- 订阅 ----
        rospy.Subscriber("/wheel_odom", Odometry, self.wheel_cb, queue_size=1)
        rospy.Subscriber("/imu/data", Imu, self.imu_cb, queue_size=1)

        # ---- 发布 ----
        self.pub = rospy.Publisher("/wheel_odom_filtered", Odometry, queue_size=10)

        rospy.loginfo("SlipDetector: 退化检测已启动")
        rospy.loginfo("  ang_threshold=%.2f rad/s, lin_threshold=%.2f m/s^2",
                      self.ang_threshold, self.lin_threshold)

    def wheel_cb(self, msg):
        self.wheel_msg = msg
        self.last_wheel_vx = msg.twist.twist.linear.x
        self.last_wheel_vyaw = msg.twist.twist.angular.z
        self.last_wheel_time = msg.header.stamp

    def imu_cb(self, msg):
        if self.wheel_msg is None or self.last_wheel_time is None:
            return

        dt = (msg.header.stamp - self.last_wheel_time).to_sec()
        if dt <= 0.0 or dt > 0.5:
            return

        # ---- 1. 角速度差异评分 ----
        imu_yaw_rate = msg.angular_velocity.z
        wheel_yaw_rate = self.last_wheel_vyaw
        diff_ang = abs(wheel_yaw_rate - imu_yaw_rate)
        score_ang = min(diff_ang / self.ang_threshold, 1.0)

        # ---- 2. 线加速度差异评分 ----
        wheel_acc = (self.wheel_msg.twist.twist.linear.x - self.last_wheel_vx) / dt
        imu_acc = msg.linear_acceleration.x
        diff_lin = abs(wheel_acc - imu_acc)
        score_lin = min(diff_lin / self.lin_threshold, 1.0)

        # ---- 3. 综合评分 + 低通平滑 ----
        raw_score = max(score_ang, score_lin)
        self.smoothed_score = (self.smooth_factor * self.smoothed_score +
                               (1.0 - self.smooth_factor) * raw_score)

        # ---- 4. 指数膨胀系数 ----
        s = self.smoothed_score
        factor = 1.0 + 99.0 * s * s          # 平滑曲线：1 ~ 100

        # ---- 5. 深拷贝并改写协方差 ----
        out = Odometry()
        out.header = self.wheel_msg.header
        out.header.stamp = msg.header.stamp
        out.child_frame_id = self.wheel_msg.child_frame_id
        out.pose = self.wheel_msg.pose
        out.twist = self.wheel_msg.twist

        # 协方差是 tuple，转成 list 修改再转回
        twist_cov = list(out.twist.covariance)
        twist_cov[0] *= factor          # vx
        twist_cov[35] *= factor         # vyaw
        out.twist.covariance = tuple(twist_cov)

        pose_cov = list(out.pose.covariance)
        pose_cov[0] *= factor           # x
        pose_cov[7] *= factor           # y
        pose_cov[35] *= factor          # yaw
        out.pose.covariance = tuple(pose_cov)

        self.pub.publish(out)

        rospy.loginfo_throttle(
            1.0,
            "slip: score=%.2f factor=%.1f | ang_diff=%.3f(%.2f) lin_diff=%.3f(%.2f)",
            self.smoothed_score, factor,
            diff_ang, score_ang, diff_lin, score_lin
        )


if __name__ == "__main__":
    rospy.init_node("slip_detector")
    SlipDetector()
    rospy.spin()
