#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
纯导航跑点脚本：按顺序跑 launch 文件中的 1~13 号点，不做识别，不走逻辑
"""

import rospy
import actionlib
import sys
from math import pi
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler


class PointRunner:
    def __init__(self):
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("等待 move_base 服务...")
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.loginfo("move_base 连接成功！")

    def goto(self, p, index=0):
        """导航到目标点 p: [x, y, yaw_deg]"""
        rospy.loginfo("前往第%d个点: x=%.3f y=%.3f yaw=%.1f" % (index, p[0], p[1], p[2]))
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

        self.move_base.send_goal(goal)
        result = self.move_base.wait_for_result(rospy.Duration(60))
        if not result:
            self.move_base.cancel_goal()
            rospy.logwarn("导航超时！")
            return False
        if self.move_base.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo("第%d个点到达成功！" % index)
            return True
        else:
            rospy.logwarn("第%d个点导航失败，状态: %d" % (index, self.move_base.get_state()))
            return False


if __name__ == "__main__":
    rospy.init_node('point_runner', anonymous=True)
    rospy.loginfo("纯跑点节点启动，跑 1~13 号点")

    # 读取导航点位
    try:
        goalListX = rospy.get_param('~goalListX')
        goalListY = rospy.get_param('~goalListY')
        goalListYaw = rospy.get_param('~goalListYaw')
        x_list = [float(x.strip()) for x in goalListX.split(",") if x.strip()]
        y_list = [float(y.strip()) for y in goalListY.split(",") if y.strip()]
        yaw_list = [float(yaw.strip()) for yaw in goalListYaw.split(",") if yaw.strip()]
        goals = []
        for x, y, yaw in zip(x_list, y_list, yaw_list):
            goals.append([x, y, yaw])
    except Exception as e:
        rospy.logerr("解析点位失败: %s" % e)
        sys.exit(1)

    runner = PointRunner()

    # 顺序跑 1~13 号点（索引 1 到 13）
    for i in range(1, 14):
        if rospy.is_shutdown():
            break
        runner.goto(goals[i], index=i)
        rospy.sleep(3)

    rospy.loginfo("1~13 号点全部跑完！")
