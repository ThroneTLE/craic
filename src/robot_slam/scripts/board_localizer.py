#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from __future__ import division

import math

import rospy
import tf

from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
from visualization_msgs.msg import Marker, MarkerArray

from board_localizer_core import (
    FieldGrid,
    estimate_translation_correction,
    snap_points_to_grid,
)


def yaw_to_quat(yaw):
    return tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)


class BoardLocalizer(object):
    def __init__(self):
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")
        self.scan_topic = rospy.get_param("~scan_topic", "/scan_filtered")
        self.enabled = rospy.get_param("~enabled", True)

        self.grid = FieldGrid(
            rospy.get_param("~grid_origin_x", -1.0),
            rospy.get_param("~grid_origin_y", -1.0),
            rospy.get_param("~grid_cell_size", 0.39),
            rospy.get_param("~grid_cols", 9),
            rospy.get_param("~grid_rows", 9),
            rospy.get_param("~grid_internal_only", True),
        )

        self.max_scan_range = rospy.get_param("~max_scan_range", 2.2)
        self.min_scan_range = rospy.get_param("~min_scan_range", 0.10)
        self.max_snap_dist = rospy.get_param("~max_snap_dist", 0.07)
        self.min_points_per_line = int(rospy.get_param("~min_points_per_line", 8))
        self.memory_time = rospy.get_param("~memory_time", 18.0)
        self.max_correction = rospy.get_param("~max_correction", 0.12)
        self.max_match_dist = rospy.get_param("~max_match_dist", 0.10)
        self.min_match_points = int(rospy.get_param("~min_match_points", 8))
        self.publish_rate = rospy.get_param("~publish_rate", 10.0)

        self.tf_listener = tf.TransformListener()
        self.scan_memory = []
        self.boards = {"vertical": [], "horizontal": []}
        self.latest_scan_points_map = []
        self.latest_pose = None
        self.latest_correction = None

        self.pose_pub = rospy.Publisher("~corrected_pose", PoseWithCovarianceStamped, queue_size=5)
        self.valid_pub = rospy.Publisher("~valid", Bool, queue_size=5)
        self.conf_pub = rospy.Publisher("~confidence", Float32, queue_size=5)
        self.marker_pub = rospy.Publisher("~markers", MarkerArray, queue_size=1)
        self.scan_sub = rospy.Subscriber(self.scan_topic, LaserScan, self.scan_cb, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.publish_timer)
        rospy.loginfo("board_localizer started: scan=%s grid_origin=(%.3f, %.3f) cell=%.3f",
                      self.scan_topic, self.grid.origin_x, self.grid.origin_y, self.grid.cell_size)

    def lookup_pose(self, target_frame, source_frame, timeout=0.2):
        try:
            self.tf_listener.waitForTransform(target_frame, source_frame,
                                              rospy.Time(0), rospy.Duration(timeout))
            trans, rot = self.tf_listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
            _, _, yaw = tf.transformations.euler_from_quaternion(rot)
            return trans[0], trans[1], yaw
        except Exception as e:
            rospy.logwarn_throttle(1.0, "board_localizer TF lookup failed: %s", str(e))
            return None

    def scan_cb(self, msg):
        if not self.enabled:
            return

        pose = self.lookup_pose(self.map_frame, msg.header.frame_id, 0.2)
        base_pose = self.lookup_pose(self.map_frame, self.base_frame, 0.2)
        if pose is None or base_pose is None:
            return

        sx, sy, syaw = pose
        cos_yaw, sin_yaw = math.cos(syaw), math.sin(syaw)
        points = []
        for i, r in enumerate(msg.ranges):
            if math.isnan(r) or math.isinf(r):
                continue
            if r < max(msg.range_min, self.min_scan_range) or r > min(msg.range_max, self.max_scan_range):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            lx = r * math.cos(angle)
            ly = r * math.sin(angle)
            mx = sx + cos_yaw * lx - sin_yaw * ly
            my = sy + sin_yaw * lx + cos_yaw * ly
            points.append((mx, my))

        now = rospy.Time.now()
        self.scan_memory.append((now, points))
        kept = []
        combined = []
        for stamp, ps in self.scan_memory:
            if (now - stamp).to_sec() <= self.memory_time:
                kept.append((stamp, ps))
                combined.extend(ps)
        self.scan_memory = kept

        self.boards = snap_points_to_grid(
            combined, self.grid,
            max_snap_dist=self.max_snap_dist,
            min_points_per_line=self.min_points_per_line,
        )
        self.latest_scan_points_map = points
        self.latest_pose = base_pose

        correction = estimate_translation_correction(
            points, self.boards,
            max_match_dist=self.max_match_dist,
            min_points=self.min_match_points,
        )

        if correction.valid:
            dx = max(-self.max_correction, min(self.max_correction, correction.dx))
            dy = max(-self.max_correction, min(self.max_correction, correction.dy))
            correction = correction._replace(dx=dx, dy=dy)
        self.latest_correction = correction

    def publish_timer(self, _event):
        valid = bool(self.latest_correction and self.latest_correction.valid and self.latest_pose)
        self.valid_pub.publish(Bool(valid))
        self.conf_pub.publish(Float32(self.latest_correction.confidence if valid else 0.0))

        if valid:
            x, y, yaw = self.latest_pose
            corr = self.latest_correction
            msg = PoseWithCovarianceStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.map_frame
            msg.pose.pose.position.x = x + corr.dx
            msg.pose.pose.position.y = y + corr.dy
            q = yaw_to_quat(yaw + corr.yaw)
            msg.pose.pose.orientation.x = q[0]
            msg.pose.pose.orientation.y = q[1]
            msg.pose.pose.orientation.z = q[2]
            msg.pose.pose.orientation.w = q[3]
            cov = [0.0] * 36
            scale = max(0.02, 0.10 * (1.0 - corr.confidence))
            cov[0] = scale * scale
            cov[7] = scale * scale
            cov[35] = 0.10
            msg.pose.covariance = cov
            self.pose_pub.publish(msg)

        self.marker_pub.publish(self.build_markers())

    def build_markers(self):
        arr = MarkerArray()
        marker_id = 0
        now = rospy.Time.now()
        for orientation in ["vertical", "horizontal"]:
            for board in self.boards.get(orientation, []):
                m = Marker()
                m.header.stamp = now
                m.header.frame_id = self.map_frame
                m.ns = "snapped_boards"
                m.id = marker_id
                marker_id += 1
                m.type = Marker.CUBE
                m.action = Marker.ADD
                if orientation == "vertical":
                    m.pose.position.x = board.position
                    m.pose.position.y = 0.5 * (board.spread_min + board.spread_max)
                    m.scale.x = 0.035
                    m.scale.y = max(0.08, board.spread_max - board.spread_min)
                else:
                    m.pose.position.x = 0.5 * (board.spread_min + board.spread_max)
                    m.pose.position.y = board.position
                    m.scale.x = max(0.08, board.spread_max - board.spread_min)
                    m.scale.y = 0.035
                m.pose.position.z = 0.05
                m.pose.orientation.w = 1.0
                m.scale.z = 0.10
                m.color.r = 0.0
                m.color.g = 0.85
                m.color.b = 1.0
                m.color.a = min(1.0, 0.25 + 0.04 * board.count)
                m.lifetime = rospy.Duration(0.5)
                arr.markers.append(m)
        return arr


if __name__ == "__main__":
    rospy.init_node("board_localizer")
    BoardLocalizer()
    rospy.spin()
