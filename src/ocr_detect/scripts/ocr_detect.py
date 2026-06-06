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
from PIL import Image, ImageFont, ImageDraw
import time
from sensor_msgs.msg import Image as ROSImage
from std_srvs.srv import Trigger, TriggerResponse
import json
import base64
import sys
import numpy as np
from cnocr import CnOcr

from PIL import Image
import os
class ocr:
    def __init__(self):
        self.sub=rospy.Subscriber('/usb_cam/image_raw', ROSImage, self.top_view_shot)
    def is_image_complete(self,img_path):
        try:
            with Image.open(img_path) as img:
                img.verify()  # 验证图像文件是否完整
            return True
        except Exception as e:
            rospy.logerr(f"图像文件不完整或损坏: {e}")
            return False
        
    def imgmsg_to_cv2(self,img_msg):
        dtype = np.dtype("uint8")  # Hardcode to 8 bits...
        dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
        image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), dtype=dtype, buffer=img_msg.data)

        # If the byte order is different between the message and the system.
        if img_msg.is_bigendian == (sys.byteorder == 'little'):
            image_opencv = image_opencv.byteswap().newbyteorder()

        # Convert to BGR if the encoding is not already BGR
        if img_msg.encoding == "rgb8":
            image_opencv = cv2.cvtColor(image_opencv, cv2.COLOR_RGB2BGR)
        elif img_msg.encoding == "mono8":
            image_opencv = cv2.cvtColor(image_opencv, cv2.COLOR_GRAY2BGR)
        elif img_msg.encoding != "bgr8":
            rospy.logerr("Unsupported encoding: %s", img_msg.encoding)
            return None

        return image_opencv

    def cv2_to_imgmsg(self,cv_image):
        img_msg = ROSImage()
        img_msg.height = cv_image.shape[0]
        img_msg.width = cv_image.shape[1]
        img_msg.encoding = "bgr8"
        img_msg.is_bigendian = 0
        img_msg.data = cv_image.tobytes()
        img_msg.step = len(img_msg.data) // img_msg.height  # That double line is actually integer division, not a comment
        return img_msg

    def top_view_shot(self,image_msg):
        global ocr_det
        # 将ROS图像消息转换为OpenCV格式
        self.img_bgr = self.imgmsg_to_cv2(image_msg)
        # 从参数服务器获取ocr_det的值
        ocr_det = rospy.get_param('/ocr_det', 255)
        
        if ocr_det == 1:
            # 保存图像
            rospy.loginfo('保存至temp/ocr_now.jpg')
            cv2.imwrite('/home/abot/demo/src/ocr_detect/temp/ocr_now.jpg', self.img_bgr)
            # 将ocr_det重置为255
            rospy.set_param('/ocr_det', 255)
            # # 屏幕上展示图像
            # cv2.imshow('vlm', img_bgr)
            cv2.waitKey(1)
            # 调用ocr模型识别
            #ocr_detection()
    def ocr_detection( self,img_path='/home/abot/demo/src/ocr_detect/temp/ocr_now.jpg'):
            global ocr1
            while True:
                if self.is_image_complete('/home/abot/demo/src/ocr_detect/temp/ocr_now.jpg'):
                    break
                else:
                    cv2.imwrite('/home/abot/demo/src/ocr_detect/temp/ocr_now.jpg', self.img_bgr)
            result = ocr1.ocr(img_path)
            combined_text = ""  # 创建一个空字符串来存储所有句子
            count = 0  # 初始化计数器
            for temp in result:
                if count >= 2:  # 如果计数器达到 2，跳出循环
                    break
                combined_text += temp["text"] + " "  # 将每个句子添加到 combined_text 中，并在每个句子后添加一个空格
                count += 1  # 计数器加 1
            rospy.loginfo(combined_text.strip())  # 输出合并后的句子，并去掉末尾多余的空格
            os.remove(img_path)
            return combined_text.strip()

def handle_ocr_detection(req):

    result = ocr2.ocr_detection()
    return TriggerResponse(success=True, message=result)

def main():
    global ocr_det
    rospy.init_node('ocr_node', anonymous=True)
    
    rospy.loginfo('ocr模块导入成功！')
    rospy.loginfo('准备识别...')
    # 从参数服务器获取ocr_det的值
    ocr_det = rospy.set_param('/ocr_det', 255)
    # 创建服务服务器
    s = rospy.Service('ocr_detection', Trigger, handle_ocr_detection)
    
    rospy.spin()

if __name__ == '__main__':
    ocr1 = CnOcr()
    ocr2=ocr()
    main()
