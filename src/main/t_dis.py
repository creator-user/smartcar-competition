# -*- coding: utf-8 -*-
# 判断前方距离
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int8
import numpy as np
import time

# 声明全局变量
distance_flag_pub = None
task_flag_pub = None
threshold_distance = 1.0 # 设置距离阈值
task_flag = 0

def filter_outliers(data, m=2):
    mean = np.mean(data)
    std_dev = np.std(data)
    return [x for x in data if (mean - m * std_dev < x < mean + m * std_dev)]

def measure_distance(scan_msg):
    num = int((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment)
    expect_angle = 180  # 正前方的角度为180度
    range_val = 3       # 范围值为3，表示取正前方角度附近的7个点（包括目标角度点）
    test_direction = int((expect_angle / 360.0) * num)
    
    distances = []
    
    for i in range(-range_val, range_val + 1):
        index = test_direction + i
        
        # 处理边界情况
        if index < 0:
            index += num
        elif index >= num:
            index -= num
        
        distances.append(scan_msg.ranges[index])
    
    filtered_distances = filter_outliers(distances)
    if filtered_distances:
        return np.mean(filtered_distances)
    return None

def laser_callback(scan_msg):
    global distance_flag_pub, threshold_distance, task_flag, task_flag_pub
    if task_flag != 5:
        return
    
    num_measurements = 5
    measurements = []
    
    for _ in range(num_measurements):
        avg_distance = measure_distance(scan_msg)
        if avg_distance is not None:
            measurements.append(avg_distance)
        time.sleep(0.2)
    
    if measurements:
        stable_avg_distance = measurements[3] #np.mean(measurements)
        #rospy.loginfo("多次测量后的稳定平均距离在180度: %f", stable_avg_distance)

        distance_flag = Int8()
        if stable_avg_distance > threshold_distance:
            distance_flag.data = 1  # 大于阈值
        else:
            distance_flag.data = 0  # 小于等于阈值

        distance_flag_pub.publish(distance_flag)
        task_flag = 1
        task_flag_msg = Int8()
        task_flag_msg.data = 1
        task_flag_pub.publish(task_flag_msg)
        rospy.loginfo("前方距离判断标志: %d", distance_flag.data)

def Task_callback(msg):
    global task_flag
    task_flag = msg.data
    rospy.loginfo("I heard task %d", task_flag)

def main():
    global distance_flag_pub, task_flag_pub
    rospy.init_node('distance_checker', anonymous=True)
    distance_flag_pub = rospy.Publisher('/distance_flag', Int8, queue_size=1)
    rospy.Subscriber('/scan', LaserScan, laser_callback)
    rospy.Subscriber("task_flag", Int8, Task_callback)
    task_flag_pub = rospy.Publisher('task_flag', Int8, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    main()
