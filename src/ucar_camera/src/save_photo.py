import cv2
import os
import time
# 设置保存帧的文件夹路径
output_folder = 'photos'

# 创建输出文件夹（如果不存在）
if not os.path.exists(output_folder):
    os.makedirs(output_folder)

# 打开摄像头
cap = cv2.VideoCapture(0)  # 参数0表示第一个摄像头，如果有多个摄像头可以尝试不同的索引
count=0
while True:
    # 读取摄像头帧
    ret, frame = cap.read()

    if not ret:
        # 读取帧失败
        print("无法读取摄像头帧")
        break


    # 保存帧到文件夹中
    frame_path = os.path.join(output_folder, f'frame{str(int(time.time()*100)%1000000000)}.jpg')
    cv2.imwrite(frame_path, frame)
    # 显示帧
    #cv2.imshow('Frame', frame)
    time.sleep(0.2)
    # 按下 'q' 键退出循环
    print(count)
    count+=1
    if count>70:
        break
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放摄像头和关闭窗口
cap.release()
cv2.destroyAllWindows()
