#!/usr/bin/env python3
'''
Copyright (c) [Zachary]
本代码受版权法保护，未经授权禁止任何形式的复制、分发、修改等使用行为。
Author:Zachary
company:WCXC

基于火山引擎 TTS REST API (HTTP POST)
'''
import rospy
import uuid
import json
import os
import base64
import urllib.request
from TTS_audio.srv import StringService, StringServiceResponse

# ==================== API 配置 ====================
API_KEY = "821dba5e-8f6f-476f-9ead-bcd1098633d1"
API_URL = "https://openspeech.bytedance.com/api/v1/tts"
CLUSTER = "volcano_tts"
VOICE_TYPE = "BV001"


def send_tts_request(text):
    """
    通过 HTTP POST 调用火山引擎 TTS REST API，保存音频文件
    :param text: 待合成文本
    :return: 音频文件路径，失败返回 None
    """
    reqid = str(uuid.uuid4())

    body = {
        "app": {"cluster": CLUSTER},
        "user": {"uid": "abot_robot"},
        "audio": {
            "voice_type": VOICE_TYPE,
            "encoding": "mp3",
            "speed_ratio": 0.9,
            "volume_ratio": 2.0,
            "pitch_ratio": 1.0
        },
        "request": {
            "reqid": reqid,
            "text": text,
            "operation": "query"
        }
    }

    data = json.dumps(body).encode("utf-8")
    req = urllib.request.Request(API_URL, data=data, method="POST")
    req.add_header("x-api-key", API_KEY)
    req.add_header("Content-Type", "application/json")

    try:
        with urllib.request.urlopen(req, timeout=30) as resp:
            content_type = resp.headers.get("Content-Type", "")
            raw = resp.read()

            if resp.status != 200:
                rospy.logerr("TTS API HTTP %s: %s", resp.status, raw[:500])
                return None

            # JSON 响应（含 base64 音频数据）
            if "json" in content_type:
                result = json.loads(raw.decode("utf-8"))
                code = result.get("code", -1)
                if code != 3000:
                    rospy.logerr("TTS API 错误: code=%s, message=%s",
                                 code, result.get("message", ""))
                    return None
                audio_data = base64.b64decode(result["data"])
                with open("output.mp3", "wb") as f:
                    f.write(audio_data)
            else:
                # 直接返回音频二进制
                with open("output.mp3", "wb") as f:
                    f.write(raw)

            return "output.mp3"

    except urllib.error.HTTPError as e:
        body_text = e.read().decode("utf-8", errors="replace")[:500]
        rospy.logerr("TTS API HTTP错误: %s %s - %s", e.code, e.reason, body_text)
        return None
    except Exception as e:
        rospy.logerr("TTS API 请求异常: %s", str(e))
        return None


def handle_tts_request(req):
    """处理 TTS 服务请求"""
    text = req.data
    rospy.loginfo("收到TTS请求: %s", text)

    try:
        audio_path = send_tts_request(text)
        if audio_path is None:
            return StringServiceResponse("TTS合成失败")

        file_size = os.path.getsize(audio_path)
        rospy.loginfo("音频保存至: %s, 文件大小: %d 字节", audio_path, file_size)

        if file_size < 100:
            rospy.logerr("音频文件过小(%d字节)，跳过播放", file_size)
            return StringServiceResponse("TTS合成失败: 文件过小")

        os.system("ffplay -nodisp -autoexit -loglevel quiet %s" % audio_path)
        return StringServiceResponse("TTS处理完成")

    except Exception as e:
        rospy.logerr("TTS处理出错: %s", str(e))
        return StringServiceResponse("错误: %s" % str(e))


def tts_server():
    rospy.init_node("tts_server")
    rospy.Service("tts_service", StringService, handle_tts_request)
    rospy.loginfo("TTS服务已启动 (REST API)，等待请求...")
    rospy.spin()


if __name__ == "__main__":
    tts_server()
