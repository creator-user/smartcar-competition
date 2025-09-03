# -*- coding: utf-8 -*-
import cv2
import numpy as np

def nothing(x):
    pass

# 创建一个窗口
cv2.namedWindow('Adjustments')

# 创建HSV范围的滑动条
cv2.createTrackbar('LH', 'Adjustments', 0, 180, nothing)
cv2.createTrackbar('LS', 'Adjustments', 0, 255, nothing)
cv2.createTrackbar('LV', 'Adjustments', 0, 255, nothing)
cv2.createTrackbar('UH', 'Adjustments', 180, 180, nothing)
cv2.createTrackbar('US', 'Adjustments', 255, 255, nothing)
cv2.createTrackbar('UV', 'Adjustments', 255, 255, nothing)

# 创建Canny边缘检测的阈值滑动条
cv2.createTrackbar('Canny Lower', 'Adjustments', 50, 255, nothing)
cv2.createTrackbar('Canny Upper', 'Adjustments', 150, 255, nothing)

# 打开摄像头
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open video stream")
    exit()

# 设置较低的分辨率
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

while True:
    # 读取帧
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame")
        break

    # 转换为HSV颜色空间
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 获取滑动条的当前值
    lh = cv2.getTrackbarPos('LH', 'Adjustments')
    ls = cv2.getTrackbarPos('LS', 'Adjustments')
    lv = cv2.getTrackbarPos('LV', 'Adjustments')
    uh = cv2.getTrackbarPos('UH', 'Adjustments')
    us = cv2.getTrackbarPos('US', 'Adjustments')
    uv = cv2.getTrackbarPos('UV', 'Adjustments')

    # 获取Canny边缘检测的阈值
    canny_lower = cv2.getTrackbarPos('Canny Lower', 'Adjustments')
    canny_upper = cv2.getTrackbarPos('Canny Upper', 'Adjustments')

    # 创建HSV范围
    lower_white = np.array([lh, ls, lv])
    upper_white = np.array([uh, us, uv])

    # 创建掩膜
    mask = cv2.inRange(hsv, lower_white, upper_white)
    result = cv2.bitwise_and(frame, frame, mask=mask)

    # 转换为灰度图像
    gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)

    # 使用高斯模糊去噪
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # 使用Canny边缘检测
    edges = cv2.Canny(blurred, canny_lower, canny_upper)

    # 查找轮廓，兼容OpenCV 3和4
    contours = None
    hierarchy = None
    if int(cv2.__version__.split('.')[0]) == 3:
        _, contours, hierarchy = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    else:
        contours, hierarchy = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 初始化最大矩形框
    max_area = 0
    best_rect = None

    # 遍历所有轮廓找到最大的矩形框
    for contour in contours:
        # 计算轮廓的边界矩形
        rect = cv2.boundingRect(contour)
        area = rect[2] * rect[3]  # 计算矩形框面积
        if area > max_area:
            max_area = area
            best_rect = rect

    # 绘制最大的矩形框，并扩大范围
    if best_rect is not None:
        x, y, w, h = best_rect
        
        # 扩大框的范围
        margin = 20
        x = max(x - margin, 0)
        y = max(y - margin, 0)
        w = min(w + 2 * margin, frame.shape[1] - x)
        h = min(h + 2 * margin, frame.shape[0] - y)

        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
        # 提取白框内的区域
        roi = gray[y:y+h, x:x+w]
        roi_edges = cv2.Canny(roi, canny_lower, canny_upper)

        # 使用Hough变换查找直线
        lines = cv2.HoughLinesP(roi_edges, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=10)

        # 初始化最长的边缘
        max_length = 0
        best_line = None
        
        # 遍历找到的所有直线，找到最长的非竖直方向的边缘
        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                    if length > max_length and abs(x1 - x2) > abs(y1 - y2):  # 非竖直方向的边缘
                        max_length = length
                        best_line = (x1, y1, x2, y2)
                        # 计算并打印斜率
                        slope = (y2 - y1) / (x2 - x1) if (x2 - x1) != 0 else float('inf')
                        print(f"Detected line with slope: {slope:.2f}")
        
        # 绘制最长的边缘
        if best_line is not None:
            x1, y1, x2, y2 = best_line
            cv2.line(frame, (x1 + x, y1 + y), (x2 + x, y2 + y), (255, 0, 0), 2)

    # 显示结果
    cv2.imshow('Frame', frame)
    cv2.imshow('Mask', mask)
    cv2.imshow('Edges', edges)

    # 按下 'q' 键退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放摄像头并关闭所有窗口
cap.release()
cv2.destroyAllWindows()
