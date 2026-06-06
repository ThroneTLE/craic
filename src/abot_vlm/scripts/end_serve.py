#!/usr/bin/env python
'''
Copyright (c) [Zachary]
本代码受版权法保护，未经授权禁止任何形式的复制、分发、修改等使用行为。
Author:Zachary
'''
import rospy
from abot_vlm.srv import LLMQuery, LLMQueryResponse
import openai
from openai import OpenAI
from API_KEY import *

pre_PROMPT = '这是一段话，代表一个数字，需要你根据这段话来解谜出这个相关数字，只输出最终的答案，例如答案为1'

def handle_llm_query(req):
    '''
    处理LLM查询请求
    '''
    last_PROMPT = req.query
    
    API_BASE = "https://ark.cn-beijing.volces.com/api/v3"
    API_KEY = YI_KEY

    MODEL = 'doubao-1-5-vision-pro-32k-250115'
    while True:
        # 访问大模型API
        client = OpenAI(api_key=API_KEY, base_url=API_BASE)
        PROMPT = pre_PROMPT + last_PROMPT
        # 正确的变量名是completion
        completion = client.chat.completions.create(
            model=MODEL, 
            messages=[{"role": "user", "content": PROMPT}]
        )
        # 将response改为completion
        result = completion.choices[0].message.content.strip()
        rospy.loginfo(f"LLM response: {result}")
        
        # 检查结果是否为数字且是个位数
        if result.isdigit() and len(result) == 1:
            return LLMQueryResponse(result)

def llm_server():
    '''
    LLM服务端
    '''
    rospy.init_node('llm_server')
    s = rospy.Service('llm_query', LLMQuery, handle_llm_query)
    rospy.loginfo("LLM server is ready to handle queries.")
    rospy.spin()

if __name__ == "__main__":
    llm_server()

