#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy
import tf
import math


def main():
    rospy.init_node('localization_printer')

    map_frame = rospy.get_param('~map_frame', 'map')
    base_frame = rospy.get_param('~base_frame', 'base_footprint')

    listener = tf.TransformListener()
    rate = rospy.Rate(10)  # 10 Hz

    rospy.loginfo("Waiting for tf %s -> %s ..." % (map_frame, base_frame))
    listener.waitForTransform(map_frame, base_frame, rospy.Time(0), rospy.Duration(10.0))

    rospy.loginfo("Start printing robot pose (x, y, yaw_deg) ...")
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform(map_frame, base_frame, rospy.Time(0))
            x = trans[0]
            y = trans[1]
            _, _, yaw_rad = tf.transformations.euler_from_quaternion(rot)
            yaw_deg = yaw_rad * 180.0 / math.pi
            rospy.loginfo("pose: x=%.4f  y=%.4f  yaw=%.2f deg" % (x, y, yaw_deg))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        rate.sleep()


if __name__ == '__main__':
    main()
