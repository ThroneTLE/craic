#!/home/abot/anaconda3/envs/py39/bin/python3.9
# -*- coding: UTF-8 -*-
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan

class LaserDistanceMonitor:
    def __init__(self):
        rospy.init_node('laser_distance_monitor')

        # 监测角度 (弧度): 前后左右
        self.front_angle = 0.0         # 0度 (前方)
        self.back_angle  = np.pi       # 180度 (后方)
        self.left_angle  = np.pi / 2   # 90度 (左侧)
        self.right_angle = -np.pi / 2  # -90度 (右侧)

        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        self.front_distance = 0.0
        self.back_distance  = 0.0
        self.left_distance  = 0.0
        self.right_distance = 0.0

        rospy.loginfo("激光距离监测节点已启动 (前后左右四向)")

    def scan_callback(self, msg):
        self.front_distance = self.get_range_at_angle(msg, self.front_angle)
        self.back_distance  = self.get_range_at_angle(msg, self.back_angle)
        self.left_distance  = self.get_range_at_angle(msg, self.left_angle)
        self.right_distance = self.get_range_at_angle(msg, self.right_angle)

        rospy.loginfo_throttle(0.5,
            "前: %.3fm  后: %.3fm  左: %.3fm  右: %.3fm" %
            (self.front_distance, self.back_distance,
             self.left_distance, self.right_distance))

    def get_range_at_angle(self, scan_msg, angle):
        if angle > np.pi:
            angle -= 2 * np.pi
        elif angle < -np.pi:
            angle += 2 * np.pi

        index = int((angle - scan_msg.angle_min) / scan_msg.angle_increment)

        if 0 <= index < len(scan_msg.ranges):
            distance = scan_msg.ranges[index]
            if scan_msg.range_min <= distance <= scan_msg.range_max:
                return distance
        return float('nan')

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        monitor = LaserDistanceMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        pass
