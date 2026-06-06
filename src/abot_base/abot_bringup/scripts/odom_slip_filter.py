#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
打滑防护：检测到轮速计打滑时冻结 odom，Cartographer 纯靠激光撑着
"""
import rospy
import math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class OdomSlipFilter:
    def __init__(self):
        self.last_imu_wz = 0.0
        self.last_odom_wz = 0.0
        self.frozen_odom = None
        self.is_slipping = False
        self.slip_score = 0.0

        # 阈值
        self.ang_threshold = rospy.get_param("~ang_threshold", 0.20)   # rad/s
        self.smooth = rospy.get_param("~smooth", 0.8)
        self.trigger = rospy.get_param("~trigger", 0.4)                # 得分超此值触发冻结

        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=1)
        self.imu_sub  = rospy.Subscriber("/imu/data", Imu, self.imu_cb, queue_size=20)
        self.odom_pub = rospy.Publisher("/odom_filtered", Odometry, queue_size=10)

        rospy.loginfo("OdomSlipFilter: ang_threshold=%.2f trigger=%.2f -> /odom_filtered",
                      self.ang_threshold, self.trigger)

    def imu_cb(self, msg):
        self.last_imu_wz = msg.angular_velocity.z

    def odom_cb(self, msg):
        wheel_wz = msg.twist.twist.angular.z
        imu_wz   = self.last_imu_wz

        diff = abs(wheel_wz - imu_wz)
        raw_score = min(diff / self.ang_threshold, 1.0)
        self.slip_score = self.smooth * self.slip_score + (1 - self.smooth) * raw_score

        if self.slip_score > self.trigger and not self.is_slipping:
            self.is_slipping = True
            self.frozen_odom = msg
            self.frozen_odom.twist.twist.linear.x = 0.0
            self.frozen_odom.twist.twist.linear.y = 0.0
            self.frozen_odom.twist.twist.angular.z = 0.0
            rospy.logwarn("SLIP DETECTED score=%.2f — freezing odom", self.slip_score)
        elif self.slip_score < self.trigger * 0.5 and self.is_slipping:
            self.is_slipping = False
            self.frozen_odom = None
            rospy.loginfo("Slip ended score=%.2f — resuming odom", self.slip_score)

        if self.is_slipping:
            out = self.frozen_odom
            out.header.stamp = msg.header.stamp
        else:
            out = msg

        self.odom_pub.publish(out)
        self.last_odom_wz = wheel_wz


if __name__ == "__main__":
    rospy.init_node("odom_slip_filter")
    OdomSlipFilter()
    rospy.spin()
