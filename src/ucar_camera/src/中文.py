import cv2
from PIL import Image, ImageDraw, ImageFont
import numpy as np
# encoding:utf-8
import base64
import requests

def cv2AddChineseText(img, text, position, textColor=(0, 0, 255), textSize=15):
    if (isinstance(img, np.ndarray)):  # 判断是否OpenCV图片类型
        img = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    # 创建一个可以在给定图像上绘图的对象
    draw = ImageDraw.Draw(img)
    # 字体的格式
    fontStyle = ImageFont.truetype("simhei.ttf", textSize, encoding="utf-8")
    # 绘制文本                                                                                                                                                                      
    draw.text(position, text, textColor, font=fontStyle)
    # 转换回OpenCV格式                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
    return cv2.cvtColor(np.asarray(img), cv2.COLOR_RGB2BGR)


frame = cv2.imread('image_2023-04-05_13-19-41.jpg')

cv2.rectangle(frame, (100,100), (200,200), (255, 0, 0), -1)
frame=cv2AddChineseText(frame,"彭锁群", (100, 100),(225, 255, 255), 15)

cv2.imshow('dsads', frame)

cv2.waitKey(0)
