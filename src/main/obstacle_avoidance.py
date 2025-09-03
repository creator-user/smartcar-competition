# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
import numpy as np
import subprocess
# 全局变量
min_distance = float('inf')
MAX_DIFF = 0.2  # 最大允许的距离差值
prev_distance = None
is_stopped = False  # 小车是否停止的状态
end_confirmed = False  # 是否确认到达终点
END_DISTANCE_THRESHOLD = 0.45  # 终点距离阈值
CONFIRMATION_COUNT = 20  # 需要连续几次检测到终点距离范围内
confirmation_counter = 0  # 连续检测计数器
task_flag_pub = None
measure_pub = None
is_measuring = 0
task_flag = 0
num_per = 0
def filter_outliers(data, m=2):
    mean = np.mean(data)
    std_dev = np.std(data)
    return [x for x in data if (mean - m * std_dev < x < mean + m * std_dev)]

def laser_callback(scan_msg):
    global min_distance, prev_distance, is_stopped, end_confirmed, confirmation_counter, is_measuring

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
        average_distance = np.mean(filtered_distances)
        # 忽略距离变化过大的数据
        if prev_distance is None and is_measuring:
            prev_distance = average_distance
        elif prev_distance != None and abs(average_distance - prev_distance) < MAX_DIFF:
            min_distance = average_distance
            prev_distance = average_distance
            is_stopped = False
        else:
            is_stopped = True
        #rospy.loginfo("过滤后的平均距离在180度: %f", average_distance)

        # 检查是否到达终点
        if average_distance < END_DISTANCE_THRESHOLD:
            confirmation_counter += 1
        else:
            confirmation_counter = 0
        
        if confirmation_counter >= CONFIRMATION_COUNT:
            end_confirmed = True
            #rospy.loginfo("确认到达终点")
        else:
            end_confirmed = False

def start_measurement_callback(msg):
    global is_measuring, measurements
    is_measuring = msg.data
    rospy.loginfo("is_measuring:%d", is_measuring)

def Task_callback(msg):
    global task_flag
    task_flag = msg.data
    rospy.loginfo("I heard task %d", task_flag)

def person_num_callback(msg):
    global num_per
    num_per = msg.data
    rospy.loginfo("person_number: %d", num_per)

def main():
    global min_distance, is_stopped, end_confirmed, is_measuring, measure_pub, task_flag, num_per, prev_distance

    # 初始化ROS节点
    rospy.init_node('obstacle_avoidance', anonymous=True)

    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    task_flag_pub = rospy.Publisher('task_flag', Int8, queue_size=1)
    rospy.Subscriber('/start_measurement', Int8, start_measurement_callback)
    measure_pub = rospy.Publisher('/start_measurement', Int8, queue_size=1)
    rospy.Subscriber("task_flag", Int8, Task_callback)
    # 创建订阅者，订阅激光雷达数据
    rospy.Subscriber('/scan', LaserScan, laser_callback)
    rospy.Subscriber('person_num', Int8, person_num_callback)

    rate = rospy.Rate(10)  # 设置循环频率为10Hz

    while not rospy.is_shutdown():
        if is_measuring and task_flag == 3:
            cmd = Twist()
            if end_confirmed:
                if num_per == 1:
                    subprocess.call("mpg123 /home/ucar/Desktop/ucar/src/mp3/w1.mp3", shell=True)
                elif num_per == 2:
                    subprocess.call("mpg123 /home/ucar/Desktop/ucar/src/mp3/w2.mp3", shell=True)
                elif num_per == 3:
                    subprocess.call("mpg123 /home/ucar/Desktop/ucar/src/mp3/w3.mp3", shell=True)
                task_flag_msg = Int8()
                task_flag_msg.data = 10
                task_flag_pub.publish(task_flag_msg)
                task_flag = 10
                msg = Int8()
                msg.data = 0
                measure_pub.publish(msg)
                is_measuring = 0
                end_confirmed = False
                is_stopped = False
                prev_distance = None

            elif is_stopped or min_distance < END_DISTANCE_THRESHOLD:
                cmd.linear.x = 0.0
                cmd_pub.publish(cmd)
            else:
                cmd.linear.x = 0.3  # 设置前进速度
                cmd_pub.publish(cmd)
        

        # 休眠以保持循环频率
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
