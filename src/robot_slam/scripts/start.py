#!/usr/bin/env python
'''
Copyright (c) [Zachary]
本代码受版权法保护，未经授权禁止任何形式的复制、分发、修改等使用行为。
Author:Zachary
'''
import rospy
from std_msgs.msg import String
import decoder
import sys
import signal

interrupted = False

def signal_handler(signal, frame):
    global interrupted
    interrupted = True

def interrupt_callback():
    global interrupted
    return interrupted

def detected_callback():
    """语音唤醒成功后的回调函数"""
    # 播放音频文件（已移到main.py中播报，此处注释掉）
    # decoder.play_audio_file()
    # 设置启动参数
    rospy.set_param('start', True)
    # 发布启动任务的话题
    pub = rospy.Publisher('/start_mission', String, queue_size=10)
    rospy.sleep(0.1)
    pub.publish("start")
    rospy.loginfo("语音唤醒成功，已发布启动任务信号！")
    
    # ========== 新增核心代码 ==========
    # 1. 终止热词检测器
    global detector
    detector.terminate()
    
    # 2. 关闭ROS节点（核心操作）
    rospy.signal_shutdown("已成功发布启动信号，关闭节点")
    
    # 3. 退出程序
    sys.exit(0)

if __name__ == '__main__':
    # 初始化ROS节点
    rospy.init_node('game_node', anonymous=True)
    
    # 设置模型路径
    model = '/home/abot/craic/src/robot_slam/resources/models/start.pmdl'
    
    # 捕获SIGINT信号，例如Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)
    
    # 创建热词检测器（声明为全局变量，方便在回调中终止）
    global detector
    detector = decoder.HotwordDetector(model, sensitivity=0.62)
    print('Listening... Press Ctrl+C to exit')
    
    # 主循环
    detector.start(detected_callback=detected_callback,
                   interrupt_check=interrupt_callback,
                   sleep_time=0.03)
    
    # 保持ROS节点运行（实际执行到这里时节点已被关闭）
    rospy.spin()
