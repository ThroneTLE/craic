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
import hashlib
import subprocess
import threading
import time
import urllib.error
import urllib.request
from TTS_audio.srv import StringService, StringServiceResponse

# ==================== API 配置 ====================
API_KEY = "821dba5e-8f6f-476f-9ead-bcd1098633d1"
API_URL = "https://openspeech.bytedance.com/api/v1/tts"
CLUSTER = "volcano_tts"
VOICE_TYPE = "BV001"
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CACHE_DIR = os.path.join(SCRIPT_DIR, "cache")
MIN_AUDIO_BYTES = 100

cache_enabled = True
cache_dir = DEFAULT_CACHE_DIR
cache_index_path = os.path.join(DEFAULT_CACHE_DIR, "index.json")
cache_index = {"version": 1, "items": {}}
cache_lock = threading.RLock()


def normalize_tts_text(text):
    if isinstance(text, bytes):
        text = text.decode("utf-8", errors="replace")
    text = str(text).strip()
    return " ".join(text.split())


def classify_tts_text(text):
    if text.startswith("已检测"):
        return "detect_clue"
    if text.startswith("已到达任务点"):
        return "task_arrival"
    if text.startswith("已到达终点"):
        return "final_arrival"
    return "general"


def now_text():
    return time.strftime("%Y-%m-%d %H:%M:%S")


def tts_cache_key(text):
    payload = {
        "text": text,
        "cluster": CLUSTER,
        "voice_type": VOICE_TYPE,
        "encoding": "mp3",
        "speed_ratio": 0.9,
        "volume_ratio": 2.0,
        "pitch_ratio": 1.0,
    }
    raw = json.dumps(payload, ensure_ascii=False, sort_keys=True).encode("utf-8")
    return hashlib.sha1(raw).hexdigest()


def ensure_dir(path):
    if not path:
        return
    if not os.path.isdir(path):
        os.makedirs(path)


def load_cache_index():
    if not os.path.exists(cache_index_path):
        return {"version": 1, "items": {}}
    try:
        with open(cache_index_path, "r", encoding="utf-8") as f:
            data = json.load(f)
        if not isinstance(data, dict):
            return {"version": 1, "items": {}}
        data.setdefault("version", 1)
        data.setdefault("items", {})
        return data
    except Exception as e:
        rospy.logwarn("[TTS_CACHE][INDEX_LOAD_FAILED] path=%s err=%s",
                      cache_index_path, str(e))
        return {"version": 1, "items": {}}


def save_cache_index():
    ensure_dir(os.path.dirname(cache_index_path))
    tmp_path = cache_index_path + ".tmp"
    with open(tmp_path, "w", encoding="utf-8") as f:
        json.dump(cache_index, f, ensure_ascii=False, indent=2, sort_keys=True)
    os.replace(tmp_path, cache_index_path)


def relative_cache_path(path):
    try:
        return os.path.relpath(path, cache_dir)
    except Exception:
        return path


def absolute_cache_path(path):
    if os.path.isabs(path):
        return path
    return os.path.join(cache_dir, path)


def cache_audio_path(key, category):
    return os.path.join(cache_dir, category, "%s.mp3" % key)


def valid_audio_file(path):
    return os.path.isfile(path) and os.path.getsize(path) >= MIN_AUDIO_BYTES


def cache_lookup(text):
    key = tts_cache_key(text)
    category = classify_tts_text(text)
    expected_path = cache_audio_path(key, category)
    with cache_lock:
        item = cache_index.get("items", {}).get(key)
        if item is not None:
            cached_path = absolute_cache_path(item.get("file", ""))
            if valid_audio_file(cached_path):
                item["hit_count"] = int(item.get("hit_count", 0)) + 1
                item["last_used_at"] = now_text()
                save_cache_index()
                return key, cached_path, True, category
            rospy.logwarn("[TTS_CACHE][STALE] key=%s path=%s", key, cached_path)

        if valid_audio_file(expected_path):
            cache_index.setdefault("items", {})[key] = {
                "text": text,
                "category": category,
                "file": relative_cache_path(expected_path),
                "voice_type": VOICE_TYPE,
                "cluster": CLUSTER,
                "created_at": now_text(),
                "last_used_at": now_text(),
                "hit_count": 1,
            }
            save_cache_index()
            return key, expected_path, True, category

    return key, expected_path, False, category


def cache_store(key, text, category, audio_path):
    with cache_lock:
        cache_index.setdefault("items", {})[key] = {
            "text": text,
            "category": category,
            "file": relative_cache_path(audio_path),
            "voice_type": VOICE_TYPE,
            "cluster": CLUSTER,
            "created_at": now_text(),
            "last_used_at": now_text(),
            "hit_count": 0,
        }
        save_cache_index()


def configure_cache():
    global cache_enabled, cache_dir, cache_index_path, cache_index
    cache_enabled = rospy.get_param("~cache_enabled", True)
    cache_dir = rospy.get_param("~cache_dir", DEFAULT_CACHE_DIR)
    cache_index_path = rospy.get_param(
        "~cache_index_file", os.path.join(cache_dir, "index.json"))
    if cache_enabled:
        ensure_dir(cache_dir)
        cache_index = load_cache_index()
        rospy.loginfo("[TTS_CACHE][READY] dir=%s index=%s items=%d",
                      cache_dir, cache_index_path,
                      len(cache_index.get("items", {})))
    else:
        rospy.logwarn("[TTS_CACHE][DISABLED]")


def send_tts_request(text, output_path):
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
                ensure_dir(os.path.dirname(output_path))
                with open(output_path, "wb") as f:
                    f.write(audio_data)
            else:
                # 直接返回音频二进制
                ensure_dir(os.path.dirname(output_path))
                with open(output_path, "wb") as f:
                    f.write(raw)

            return output_path

    except urllib.error.HTTPError as e:
        body_text = e.read().decode("utf-8", errors="replace")[:500]
        rospy.logerr("TTS API HTTP错误: %s %s - %s", e.code, e.reason, body_text)
        return None
    except Exception as e:
        rospy.logerr("TTS API 请求异常: %s", str(e))
        return None


def play_audio(audio_path):
    return subprocess.call([
        "ffplay", "-nodisp", "-autoexit", "-loglevel", "quiet", audio_path
    ])


def handle_tts_request(req):
    """处理 TTS 服务请求"""
    text = normalize_tts_text(req.data)
    rospy.loginfo("收到TTS请求: %s", text)

    try:
        cache_hit = False
        if cache_enabled:
            key, audio_path, cache_hit, category = cache_lookup(text)
            if cache_hit:
                rospy.loginfo("[TTS_CACHE][HIT] category=%s key=%s path=%s",
                              category, key, audio_path)
            else:
                rospy.loginfo("[TTS_CACHE][MISS] category=%s key=%s text=%s",
                              category, key, text)
                audio_path = send_tts_request(text, audio_path)
                if audio_path is None:
                    return StringServiceResponse("TTS合成失败")
                if valid_audio_file(audio_path):
                    cache_store(key, text, category, audio_path)
                    rospy.loginfo("[TTS_CACHE][STORE] category=%s key=%s path=%s",
                                  category, key, audio_path)
        else:
            audio_path = send_tts_request(text, os.path.join(SCRIPT_DIR, "output.mp3"))
            if audio_path is None:
                return StringServiceResponse("TTS合成失败")

        file_size = os.path.getsize(audio_path)
        rospy.loginfo("音频保存至: %s, 文件大小: %d 字节", audio_path, file_size)

        if file_size < 100:
            rospy.logerr("音频文件过小(%d字节)，跳过播放", file_size)
            return StringServiceResponse("TTS合成失败: 文件过小")

        play_code = play_audio(audio_path)
        if play_code != 0:
            rospy.logerr("TTS播放失败: ffplay exit=%s path=%s", play_code, audio_path)
            return StringServiceResponse("TTS播放失败")
        if cache_hit:
            return StringServiceResponse("TTS缓存播放完成")
        return StringServiceResponse("TTS处理完成")

    except Exception as e:
        rospy.logerr("TTS处理出错: %s", str(e))
        return StringServiceResponse("错误: %s" % str(e))


def tts_server():
    rospy.init_node("tts_server")
    configure_cache()
    rospy.Service("tts_service", StringService, handle_tts_request)
    rospy.loginfo("TTS服务已启动 (REST API + 本地缓存)，等待请求...")
    rospy.spin()


if __name__ == "__main__":
    tts_server()
