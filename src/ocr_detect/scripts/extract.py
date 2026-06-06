#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
Copyright (c) [Zachary]
本代码受版权法保护，未经授权禁止任何形式的复制、分发、修改等使用行为。
Author:Zachary
'''
from cnocr import CnOcr

img_path = '/home/abot/demo/src/ocr_detect/image/11.jpg'
ocr = CnOcr() 
result = ocr.ocr(img_path)
#print(result)
combined_text = ""  # 创建一个空字符串来存储所有句子

for temp in result:
    combined_text += temp["text"] + " "  # 将每个句子添加到 combined_text 中，并在每个句子后添加一个空格

print(combined_text.strip())  # 输出合并后的句子，并去掉末尾多余的空格
