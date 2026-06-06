#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""简单固定系数膨胀轮速计协方差，让 EKF 降低轮速计权重，更多依赖 IMU"""
import rospy
from nav_msgs.msg import Odometry


class OdomCovInflate:
    def __init__(self):
        self.factor = rospy.get_param("~factor", 5.0)  # 默认膨胀 5 倍
        self.pub = rospy.Publisher("/wheel_odom_inflated", Odometry, queue_size=10)
        rospy.Subscriber("/wheel_odom", Odometry, self.cb, queue_size=1)
        rospy.loginfo("OdomCovInflate: factor=%.1f", self.factor)

    def cb(self, msg):
        # pose 协方差: x[0], y[7], yaw[35]
        pc = list(msg.pose.covariance)
        pc[0] *= self.factor
        pc[7] *= self.factor
        pc[35] *= self.factor
        msg.pose.covariance = tuple(pc)

        # twist 协方差: vx[0], vyaw[35]
        tc = list(msg.twist.covariance)
        tc[0] *= self.factor
        tc[35] *= self.factor
        msg.twist.covariance = tuple(tc)

        self.pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("odom_cov_inflate")
    OdomCovInflate()
    rospy.spin()
