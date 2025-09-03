# -*- coding: utf-8 -*-
import cv2
import numpy as np

def nothing(x):
    pass

# ����һ������
cv2.namedWindow('Adjustments')

# ����HSV��Χ�Ļ�����
cv2.createTrackbar('LH', 'Adjustments', 0, 180, nothing)
cv2.createTrackbar('LS', 'Adjustments', 0, 255, nothing)
cv2.createTrackbar('LV', 'Adjustments', 0, 255, nothing)
cv2.createTrackbar('UH', 'Adjustments', 180, 180, nothing)
cv2.createTrackbar('US', 'Adjustments', 255, 255, nothing)
cv2.createTrackbar('UV', 'Adjustments', 255, 255, nothing)

# ����Canny��Ե������ֵ������
cv2.createTrackbar('Canny Lower', 'Adjustments', 50, 255, nothing)
cv2.createTrackbar('Canny Upper', 'Adjustments', 150, 255, nothing)

# ������ͷ
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open video stream")
    exit()

# ���ýϵ͵ķֱ���
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

while True:
    # ��ȡ֡
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame")
        break

    # ת��ΪHSV��ɫ�ռ�
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # ��ȡ�������ĵ�ǰֵ
    lh = cv2.getTrackbarPos('LH', 'Adjustments')
    ls = cv2.getTrackbarPos('LS', 'Adjustments')
    lv = cv2.getTrackbarPos('LV', 'Adjustments')
    uh = cv2.getTrackbarPos('UH', 'Adjustments')
    us = cv2.getTrackbarPos('US', 'Adjustments')
    uv = cv2.getTrackbarPos('UV', 'Adjustments')

    # ��ȡCanny��Ե������ֵ
    canny_lower = cv2.getTrackbarPos('Canny Lower', 'Adjustments')
    canny_upper = cv2.getTrackbarPos('Canny Upper', 'Adjustments')

    # ����HSV��Χ
    lower_white = np.array([lh, ls, lv])
    upper_white = np.array([uh, us, uv])

    # ������Ĥ
    mask = cv2.inRange(hsv, lower_white, upper_white)
    result = cv2.bitwise_and(frame, frame, mask=mask)

    # ת��Ϊ�Ҷ�ͼ��
    gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)

    # ʹ�ø�˹ģ��ȥ��
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # ʹ��Canny��Ե���
    edges = cv2.Canny(blurred, canny_lower, canny_upper)

    # ��������������OpenCV 3��4
    contours = None
    hierarchy = None
    if int(cv2.__version__.split('.')[0]) == 3:
        _, contours, hierarchy = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    else:
        contours, hierarchy = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # ��ʼ�������ο�
    max_area = 0
    best_rect = None

    # �������������ҵ����ľ��ο�
    for contour in contours:
        # ���������ı߽����
        rect = cv2.boundingRect(contour)
        area = rect[2] * rect[3]  # ������ο����
        if area > max_area:
            max_area = area
            best_rect = rect

    # �������ľ��ο򣬲�����Χ
    if best_rect is not None:
        x, y, w, h = best_rect
        
        # �����ķ�Χ
        margin = 20
        x = max(x - margin, 0)
        y = max(y - margin, 0)
        w = min(w + 2 * margin, frame.shape[1] - x)
        h = min(h + 2 * margin, frame.shape[0] - y)

        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
        # ��ȡ�׿��ڵ�����
        roi = gray[y:y+h, x:x+w]
        roi_edges = cv2.Canny(roi, canny_lower, canny_upper)

        # ʹ��Hough�任����ֱ��
        lines = cv2.HoughLinesP(roi_edges, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=10)

        # ��ʼ����ı�Ե
        max_length = 0
        best_line = None
        
        # �����ҵ�������ֱ�ߣ��ҵ���ķ���ֱ����ı�Ե
        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                    if length > max_length and abs(x1 - x2) > abs(y1 - y2):  # ����ֱ����ı�Ե
                        max_length = length
                        best_line = (x1, y1, x2, y2)
                        # ���㲢��ӡб��
                        slope = (y2 - y1) / (x2 - x1) if (x2 - x1) != 0 else float('inf')
                        print(f"Detected line with slope: {slope:.2f}")
        
        # ������ı�Ե
        if best_line is not None:
            x1, y1, x2, y2 = best_line
            cv2.line(frame, (x1 + x, y1 + y), (x2 + x, y2 + y), (255, 0, 0), 2)

    # ��ʾ���
    cv2.imshow('Frame', frame)
    cv2.imshow('Mask', mask)
    cv2.imshow('Edges', edges)

    # ���� 'q' ���˳�
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# �ͷ�����ͷ���ر����д���
cap.release()
cv2.destroyAllWindows()
