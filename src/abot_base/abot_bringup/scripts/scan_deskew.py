#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
激光扫描去畸变节点：用 IMU 角速度逐点修正旋转畸变
订阅 /scan_filtered + /imu/data，发布 /scan_deskewed
"""
import rospy
import numpy as np
from collections import deque
from sensor_msgs.msg import LaserScan, Imu


class ScanDeskew:
    def __init__(self):
        self.imu_queue = deque(maxlen=300)  # 缓存 ~0.3s 的 IMU 数据

        self.scan_sub  = rospy.Subscriber("/scan_filtered", LaserScan, self.scan_cb, queue_size=1)
        self.imu_sub   = rospy.Subscriber("/imu/data", Imu, self.imu_cb, queue_size=20)
        self.scan_pub  = rospy.Publisher("/scan_deskewed", LaserScan, queue_size=10)

        rospy.loginfo("ScanDeskew: 去畸变节点已启动 (imu=/imu/data → /scan_deskewed)")

    def imu_cb(self, msg):
        self.imu_queue.append(msg)

    def scan_cb(self, scan):
        t0 = scan.header.stamp.to_sec()
        scan_time = scan.scan_time if scan.scan_time > 0 else (
            len(scan.ranges) * scan.time_increment)

        if scan_time <= 0 or scan.time_increment <= 0:
            self.scan_pub.publish(scan)
            return

        # 找覆盖扫描时间范围的 IMU 数据
        imu_vals = [(m.header.stamp.to_sec(), m.angular_velocity.z)
                    for m in self.imu_queue
                    if abs(m.header.stamp.to_sec() - t0) < 0.5]

        if len(imu_vals) < 2:
            self.scan_pub.publish(scan)  # IMU 不足，原样转发
            return

        imu_vals.sort()

        # 计算累积转角：cum[i] = 从 t0 到 imu_vals[i] 的旋转角
        cum_angle = [0.0]
        for i in range(1, len(imu_vals)):
            dt = imu_vals[i][0] - imu_vals[i-1][0]
            avg_wz = (imu_vals[i][1] + imu_vals[i-1][1]) * 0.5
            cum_angle.append(cum_angle[-1] + avg_wz * dt)

        # 对每个激光点做去畸变
        ranges = list(scan.ranges)
        for i, r in enumerate(ranges):
            if np.isnan(r) or np.isinf(r) or r < scan.range_min or r > scan.range_max:
                continue

            pt_time = t0 + i * scan.time_increment     # 该点的采集时刻
            angle_offset = self._interp_angle(pt_time, imu_vals, cum_angle)

            if abs(angle_offset) < 1e-6:
                continue

            # 计算该点在当前帧中的原始角度
            pt_angle = scan.angle_min + i * scan.angle_increment
            # 修正角度 = 原始角度 - 偏转角（把点转回 t0 位置）
            corrected_angle = pt_angle - angle_offset

            # 找最近的激光点索引
            idx = int((corrected_angle - scan.angle_min) / scan.angle_increment + 0.5)
            if 0 <= idx < len(ranges):
                ranges[idx] = r
                if idx != i:
                    ranges[i] = float('nan')

        out = LaserScan()
        out.header = scan.header
        out.angle_min = scan.angle_min
        out.angle_max = scan.angle_max
        out.angle_increment = scan.angle_increment
        out.time_increment = scan.time_increment
        out.scan_time = scan_time
        out.range_min = scan.range_min
        out.range_max = scan.range_max
        out.ranges = ranges
        out.intensities = scan.intensities
        self.scan_pub.publish(out)

    def _interp_angle(self, t, imu_vals, cum_angle):
        """二分查找 t 时刻的累积转角"""
        if t <= imu_vals[0][0]:
            return cum_angle[0]
        if t >= imu_vals[-1][0]:
            return cum_angle[-1]

        # 二分找到最接近的时间戳
        lo, hi = 0, len(imu_vals) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if imu_vals[mid][0] <= t:
                lo = mid
            else:
                hi = mid

        # 线性插值
        frac = (t - imu_vals[lo][0]) / (imu_vals[hi][0] - imu_vals[lo][0])
        return cum_angle[lo] + frac * (cum_angle[hi] - cum_angle[lo])


if __name__ == "__main__":
    rospy.init_node("scan_deskew")
    ScanDeskew()
    rospy.spin()
