#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
Copyright (c) [Zachary]
本代码受版权法保护，未经授权禁止任何形式的复制、分发、修改等使用行为。
Author:Zachary
'''
import rospy
import cv2
import numpy as np
from PIL import Image
import os
from sensor_msgs.msg import Image as ROSImage
from sensor_msgs.msg import CompressedImage
from std_srvs.srv import Trigger, TriggerResponse
from cnocr import CnOcr
import time
import sys
from cv_bridge import CvBridge

# 确保目录存在
def ensure_dir_exists(dir_path):
    if not os.path.exists(dir_path):
        os.makedirs(dir_path)

# 检查图像文件是否完整
def is_image_complete(img_path):
    try:
        # 检查文件大小是否大于 0
        if os.path.getsize(img_path) <= 0:
            rospy.logerr(f"图像文件为空: {img_path}")
            return False
        
        # 验证图像文件是否完整
        with Image.open(img_path) as img:
            img.verify()
        return True
    except Exception as e:
        rospy.logerr(f"图像文件不完整或损坏: {e}")
        return False

# 将 ROS 压缩图像消息转换为 OpenCV 格式
def compressed_imgmsg_to_cv2(img_msg):
    try:
        # 将压缩图像数据转换为 numpy 数组
        np_arr = np.frombuffer(img_msg.data, np.uint8)
        # 使用 OpenCV 解码图像
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        return cv_image
    except Exception as e:
        rospy.logerr(f"转换压缩图像失败: {e}")
        return None

# 保存图像并重试直到成功
def save_image_with_retry(img_bgr, img_path, max_retries=5, retry_delay=1):
    retries = 0
    while retries < max_retries:
        try:
            success = cv2.imwrite(img_path, img_bgr)
            if success:
                rospy.sleep(1)
                # 强制将文件写入磁盘
                with open(img_path, 'a') as f:
                    os.fsync(f.fileno())
                
                # 检查文件大小是否大于 0
                if os.path.getsize(img_path) > 0:
                    rospy.loginfo(f"图像保存成功: {img_path}")
                    return True
                else:
                    rospy.logwarn(f"图像文件为空，重试中... (尝试次数: {retries + 1})")
            else:
                rospy.logwarn(f"图像保存失败，重试中... (尝试次数: {retries + 1})")
        except Exception as e:
            rospy.logerr(f"图像保存失败: {e}")
        
        retries += 1
        time.sleep(retry_delay)  # 等待一段时间后重试

    rospy.logerr(f"图像保存失败，已达到最大重试次数: {max_retries}")
    return False

# 处理图像并保存
def top_view_shot(image_msg):
    global ocr_det
    img_bgr = compressed_imgmsg_to_cv2(image_msg)
    ocr_det = rospy.get_param('/ocr_det', 255)
    
    if ocr_det == 1:
        img_path = '/home/abot/new_vision/src/ocr_detect/temp/ocr_now.jpg'
        ensure_dir_exists(os.path.dirname(img_path))
        rospy.loginfo('尝试保存图像至 %s', img_path)
        
        # 保存图像并重试直到成功
        if save_image_with_retry(img_bgr, img_path):
            while True:
                if is_image_complete(img_path):
                    break
            rospy.set_param('/ocr_det', 255)
        else:
            rospy.logerr("图像保存失败，无法继续 OCR 处理")

# OCR 检测
def ocr_detection(img_path='/home/abot/new_vision/src/ocr_detect/temp/ocr_now.jpg'):
    global ocr
    if not os.path.exists(img_path):
        rospy.logerr('文件不存在: %s', img_path)
        return ""
    
    result = ocr.ocr(img_path)
    combined_text = ""
    count = 0
    for temp in result:
        if count >= 2:
            break
        combined_text += temp["text"] + " "
        count += 1
    rospy.loginfo(combined_text.strip())
    os.remove(img_path)
    return combined_text.strip()

# 处理 OCR 请求
def handle_ocr_detection(req):
    result = ocr_detection()
    return TriggerResponse(success=True, message=result)

# 主函数
def main():
    global ocr_det
    rospy.init_node('ocr_node', anonymous=True)
    rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, top_view_shot)
    rospy.loginfo('OCR 模块导入成功！')
    rospy.loginfo('准备识别...')
    ocr_det = rospy.set_param('/ocr_det', 255)
    
    # 创建服务服务器
    s = rospy.Service('ocr_detection', Trigger, handle_ocr_detection)
    
    rospy.spin()

if __name__ == '__main__':
    ocr = CnOcr()
    main()