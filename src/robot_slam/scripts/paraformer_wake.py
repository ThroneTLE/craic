#!/home/abot/anaconda3/envs/robot_com/bin/python
# -*- coding: utf-8 -*-
"""
Paraformer 中文语音唤醒节点
循环录音 → 语音识别 → 检测"比赛开始" → 触发任务启动
"""
import rospy
import pyaudio
import wave
import os
import sys
import time
from funasr import AutoModel
from std_msgs.msg import String

# 模型路径
MODEL_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "paraformer-zh")

# 音频参数
SAMPLE_RATE = 16000      # 采样率 16kHz（Paraformer 要求）
CHANNELS = 1             # 单声道
CHUNK = 1024             # 每次读取帧数
FORMAT = pyaudio.paInt16 # 16位采样
RECORD_SECONDS = 3       # 每次录音时长（秒）
GAP_SECONDS = 0.5        # 两次录音间隔（秒）

# 唤醒词
WAKE_WORD = "比赛开始"

# 提示音
START_GAME_WAV = "/home/abot/throne_craic/src/robot_slam/resources/startGame.wav"

os.environ['KMP_DUPLICATE_LIB_OK'] = 'True'


def play_audio(filepath):
    """播放提示音"""
    if os.path.exists(filepath):
        os.system('mplayer -really-quiet "%s" 2>/dev/null' % filepath)


def record_audio(duration, save_file):
    """录制一段音频并保存为 WAV 文件"""
    p = pyaudio.PyAudio()

    if os.path.exists(save_file):
        os.remove(save_file)

    stream = p.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=SAMPLE_RATE,
                    input=True,
                    frames_per_buffer=CHUNK)

    frames = []
    for _ in range(0, int(SAMPLE_RATE / CHUNK * duration)):
        data = stream.read(CHUNK, exception_on_overflow=False)
        frames.append(data)

    stream.stop_stream()
    stream.close()
    p.terminate()

    with wave.open(save_file, 'wb') as wf:
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(p.get_sample_size(FORMAT))
        wf.setframerate(SAMPLE_RATE)
        wf.writeframes(b''.join(frames))


def main():
    rospy.init_node('paraformer_wake_node')

    # 加载 Paraformer 模型（首次加载需要 10-30 秒）
    rospy.loginfo("Loading Paraformer model from %s ..." % MODEL_DIR)
    model = AutoModel(model=MODEL_DIR, disable_update=True)
    rospy.loginfo("Paraformer model loaded. Listening for wake word '%s' ..." % WAKE_WORD)

    temp_wav = "/tmp/paraformer_wake.wav"
    pub = rospy.Publisher('/start_mission', String, queue_size=10, latch=True)

    rate = rospy.Rate(1.0 / (RECORD_SECONDS + GAP_SECONDS))

    while not rospy.is_shutdown():
        try:
            # 录音
            rospy.loginfo("Recording %d seconds ..." % RECORD_SECONDS)
            record_audio(RECORD_SECONDS, temp_wav)

            # 识别
            rospy.loginfo("Recognizing ...")
            res = model.generate(input=temp_wav)
            text = res[0].get('text', '').strip() if res else ''
            rospy.loginfo("Recognized: '%s'" % text)

            # 检测唤醒词
            if WAKE_WORD in text:
                rospy.loginfo("=== Wake word '%s' detected! ===" % WAKE_WORD)

                # 播放提示音
                play_audio(START_GAME_WAV)

                # 设置启动参数
                rospy.set_param('start', True)
                rospy.sleep(0.1)

                # 发布启动消息
                pub.publish("start")
                rospy.loginfo("Published /start_mission, mission starting ...")

                rospy.sleep(0.5)
                break

        except Exception as e:
            rospy.logerr("Error in wake loop: %s" % str(e))

        rate.sleep()

    # 清理临时文件
    if os.path.exists(temp_wav):
        os.remove(temp_wav)

    rospy.loginfo("Wake node finished.")


if __name__ == '__main__':
    main()
