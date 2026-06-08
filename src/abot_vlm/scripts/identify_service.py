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
from API_KEY import *
import json
import openai
from openai import OpenAI
import base64
import sys

def imgmsg_to_cv2(img_msg):
    dtype = np.dtype("uint8")  # Hardcode to 8 bits...
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), dtype=dtype, buffer=img_msg.data)

    # 如果消息和系统的字节序不同
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()

    # 转换编码格式
    if img_msg.encoding == "rgb8":
        image_opencv = cv2.cvtColor(image_opencv, cv2.COLOR_RGB2BGR)
    elif img_msg.encoding == "mono8":
        image_opencv = cv2.cvtColor(image_opencv, cv2.COLOR_GRAY2BGR)
    elif img_msg.encoding != "bgr8":
        rospy.logerr("Unsupported encoding: %s", img_msg.encoding)
        return None

    return image_opencv

def cv2_to_imgmsg(cv_image):
    img_msg = ROSImage()
    img_msg.height = cv_image.shape[0]
    img_msg.width = cv_image.shape[1]
    img_msg.encoding = "bgr8"
    img_msg.is_bigendian = 0
    img_msg.data = cv_image.tobytes()
    img_msg.step = len(img_msg.data) // img_msg.height
    return img_msg

def top_view_shot(image_msg):
    global detect
    # 将ROS图像消息转换为OpenCV格式
    img_bgr = imgmsg_to_cv2(image_msg)
    # 从参数服务器获取detect的值
    detect = rospy.get_param('/detect', 255)
    
    if detect == 1:
        # 保存图像
        save_path = '/home/abot/EIU0US/src/abot_vlm/temp2/vl_now.jpg'
        rospy.loginfo(f'保存至{save_path}')
        cv2.imwrite(save_path, img_bgr)
        # 重置detect
        rospy.set_param('/detect', 255)
        cv2.waitKey(1)

        # 调用视觉大模型API
        result = yi_vision_api()
        rospy.loginfo(f'最终识别结果：{result}')

def yi_vision_api(
    # 强化Prompt：要求最后一行只输出答案数字
    PROMPT='图中有一道数学题，答案一个阿拉伯数字。请先完整解题，最后一行只输出这个答案数字（仅数字，无任何其他字符）；若没有符合条件的数字，最后一行只输出“无”。',
    # PROMPT='你看到了色什么',
    img_path='/home/abot/EIU0US/src/abot_vlm/temp2/vl_now.jpg',
    max_retry=3  # 最大重试次数，避免无限循环
):
    '''
    零一万物大模型开放平台，yi-vision视觉语言多模态大模型API
    '''
    # 初始化OpenAI客户端
    client = OpenAI(
        api_key=YI_KEY,
        base_url="https://ark.cn-beijing.volces.com/api/v3"
    )
    
    # 编码图片为base64
    try:
        with open(img_path, 'rb') as image_file:
            image_base64 = base64.b64encode(image_file.read()).decode('utf-8')
        image_url = f'data:image/jpeg;base64,{image_base64}'
    except Exception as e:
        rospy.logerr(f'读取图片失败：{e}')
        return "无"
    
    # 有效结果（字符串类型）
    valid_results = ["31", "32", "33", "40", "41", "42", "49", "50", "51", "无"]
    retry_count = 0  # 重试计数器
    
    while retry_count < max_retry:
        try:
            # 调用大模型API
            response = client.chat.completions.create(
                model="doubao-1-5-vision-pro-32k-250115",
                messages=[
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": PROMPT},
                            {"type": "image_url", "image_url": {"url": image_url}}
                        ]
                    }
                ],
                timeout=30  # 设置超时时间
            )
            
            # 解析结果
            result_str = response.choices[0].message.content.strip()
            clean_result = "无"  # 默认值
            
            # ========== 修复后的核心数据清洗逻辑 ==========
            # 步骤1：按换行分割，过滤空行后取最后一行
            lines = [line.strip() for line in result_str.split('\n') if line.strip()]
            last_line = lines[-1] if lines else ""
            
            # 步骤2：优先匹配最后一行是否为有效结果
            if last_line in valid_results:
                clean_result = last_line
            else:
                # 步骤3：兜底逻辑 - 从整个文本中提取所有有效数字串，取最后一个匹配项
                matched = []
                # 遍历有效结果（排除"无"），检查是否存在于返回文本中
                for valid_num in valid_results:
                    if valid_num != "无" and valid_num in result_str:
                        matched.append(valid_num)
                # 取最后一个匹配的有效数字，无则保留"无"
                if matched:
                    clean_result = matched[-1]
            # ========== 数据清洗逻辑结束 ==========
            
            rospy.loginfo(f'大模型原始返回：\n{result_str}\n--- 清理后结果：{clean_result}')
            
            # 检查结果是否有效
            if clean_result in valid_results:
                return clean_result
            else:
                retry_count += 1
                rospy.logwarn(f'结果不符合要求（{clean_result}），第{retry_count}次重试...')
                time.sleep(1)
        
        except Exception as e:
            retry_count += 1
            rospy.logerr(f'调用大模型失败：{e}，第{retry_count}次重试...')
            time.sleep(1)
    
    # 超过最大重试次数
    rospy.logerr(f'超过最大重试次数（{max_retry}次），返回“无”')
    return "无"

def handle_fruit_detection(req):
    # 调用视觉大模型API
    result = yi_vision_api()
    return TriggerResponse(success=True, message=result)

def main():
    global detect
    rospy.init_node('identify_node', anonymous=True)
    # 订阅图像话题
    rospy.Subscriber('/usb_cam/image_raw', ROSImage, top_view_shot)
    # 初始化参数服务器
    rospy.set_param('/detect', 255)
    # 创建服务
    s = rospy.Service('fruit_detection', Trigger, handle_fruit_detection)
    
    rospy.loginfo('视觉大模型模块启动成功！')
    rospy.loginfo('准备识别...')
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
