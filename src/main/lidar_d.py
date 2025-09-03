# -*- coding: utf-8 -*-
# 测距
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int8, Float32
import numpy as np

# 声明全局变量
go_m_pub = None
measure_pub = None
is_measuring = 0
measurements = []

def filter_outliers(data, m=2):
    mean = np.mean(data)
    std_dev = np.std(data)
    return [x for x in data if (mean - m * std_dev < x < mean + m * std_dev)]

def start_measurement_callback(msg):
    global is_measuring, measurements
    is_measuring = msg.data
    if is_measuring == 1:
        measurements = []
    rospy.loginfo("%d", is_measuring)
    #rospy.loginfo("开始测距: %s", "是" if is_measuring else "否")

def laser_callback(scan_msg):
    global is_measuring, measurements, go_m_pub, measure_pub

    if is_measuring == 0 or is_measuring == 2: # 距离没有测完 或者 走向面板时
        return
    
    num = int((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment)
    
    expect_angle = 180  # 正前方的角度为180度
    range_val = 3       # 范围值为3，表示取正后方角度附近的7个点（包括目标角度点）
    
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
        average_distance = np.mean(filtered_distances)
        measurements.append(average_distance)
        #rospy.loginfo("过滤后的平均距离在180度: %f", average_distance)

    # 假设测距时间为10秒
    if len(measurements) >= 10 * 3:  # 10 Hz * 10 seconds
        final_average_distance = filter_outliers(measurements)
        final_average_distance = np.mean(measurements)
        go_m_msg = Float32()
        go_m_msg.data = final_average_distance-0.85  #0.35
        if go_m_msg.data < 0: 
            go_m_msg.data = 0
        go_m_pub.publish(go_m_msg)
        rospy.loginfo("最终的平均距离在180度: %f", go_m_msg.data)
        measurement = Int8()
        measurement.data = 2
        measure_pub.publish(measurement)
        is_measuring = 0
        #rospy.loginfo("测距结束")

def main():
    global go_m_pub, measure_pub
    rospy.init_node('laser_scan_processor', anonymous=True)
    go_m_pub = rospy.Publisher('/go_m', Float32, queue_size=1)
    rospy.Subscriber('/scan', LaserScan, laser_callback)
    rospy.Subscriber('/start_measurement', Int8, start_measurement_callback)
    measure_pub = rospy.Publisher('/start_measurement', Int8, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    main()
