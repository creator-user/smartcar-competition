# ps -ef | grep vino
# 
# 为了让小车经过多个导航点时不停留花时间来原地调整方向，可以在设置目标点时，将朝向角度设置为与前一个目标点相同。
# 这样小车就可以直接从前一个目标点到达下一个目标点，而不需要在目标点处调整方向。
# 具体来说，可以在每个目标点的朝向角度中设置与前一个目标点相同的值，例如在代码中的第1个目标点后，可以将第2个目标点的朝向角度设置为-0.009
# 与第1个目标点相同。同样的，可以在后续的目标点中重复这个步骤，以便小车可以顺利地到达每个目标点。

import rospy
import time
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from move_base_msgs.msg import MoveBaseActionResult
from std_msgs.msg import Int8
from actionlib_msgs.msg import GoalID
import subprocess

task_flag = 0
wheel_stop = 0
distance_flag = 0
num_per = 0

# 任务计时，避免卡死
accumulated_time = 0
threshold_time = 10  # 设定的阈值时间，单位：秒
# 设置多个目标点
goal_list = [
    #[0.608, 1.22, 0.000, 0.000, 0.000, 0.691, 0.723],
    [0.5, 0.651, 0.000, 0.000, 0.000, 0.691, 0.723],
    [1.1, 1.20, 0.000, 0.000, 0.000, 0.001, 0.999],  # 识别区0

    [0.379, 0.004, 0.000, 0.000, 0.000, -0.710, 0.704],
    [0.350, -0.065, 0.000, 0.000, 0.000, 0.999, 0.001],

    #[-1.85, -0.18, 0.000, 0.000, 0.000, 0.999, -0.008],
    [-2.0, 0.100, 0.000, 0.000, 0.000, 0.999, 0.001], # 中朝上
    [-2.75, -1.74, 0.000, 0.000, 0.000, -0.738, 0.675], # 急救包4
    [-2.7, -1.74, 0.000, 0.000, 0.000, 0.001, 0.999],# 急救包下

    [-2.0, 0.200, 0.000, 0.000, 0.000, 0.719, 0.695],#中朝右
    [-2.0, 0.9, 0.000, 0.000, 0.000, 0.999, 0.001],#右朝上
    [-2.0, 0.200, 0.000, 0.000, 0.000, -0.678, 0.735],#中朝左
    [-2.0, -0.6, 0.000, 0.000, 0.000, 0.001, 0.999],#左朝下  -0.678, 0.735
    #[-2.0, 0.100, 0.000, 0.000, 0.000, 0.001, 0.7],#中  

    [-2.3, 0.25, 0.000, 0.000, 0.000, 0.001, 0.9],

    [-0.1, -0.1, 0.000, 0.000, 0.000, 0.001, 0.999],
    [0.350, -0.065, 0.000, 0.000, 0.000, 0.001, 0.999],
    [0.414, -0.370, 0.000, 0.000, 0.000, -0.678, 0.735],
    # [1.447, -0.151, 0.000, 0.000, 0.000, -0.234, 0.999],
    [1.447, -0.151, 0.000, 0.000, 0.000, 0.269, 0.963],
    [1.90, -0.20, 0.000, 0.000, 0.000, -0.1, 0.995]
]

goal_index = 0
def rotate_and_move(twist, target_angle_deg, rotation_time, pub):
    global wheel_stop
    if wheel_stop:
        return
    # 设置旋转角度，将目标旋转角度转换为弧度
    target_angle_rad = target_angle_deg * 3.14159 / 180.0
    # 计算旋转速度
    angular_speed = target_angle_rad / rotation_time

    # 设置机器人的旋转速度
    twist.angular.z = -angular_speed  # 顺时针

    # 计算旋转的起始时间和发布频率
    t0 = rospy.Time.now().to_sec()
    rate = rospy.Rate(10)

    # 循环发布旋转命令，直到达到旋转时间
    while rospy.Time.now().to_sec() - t0 < rotation_time:
        if wheel_stop:
            break
        pub.publish(twist)
        rate.sleep()
    # 发布停止命令，使机器人停止旋转
    twist.angular.z = 0.0
    pub.publish(twist)
    time.sleep(0.3)


def set_goal(index):
    global goal_list
    goal = goal_list[index]
    mypose = PoseStamped()
    mypose.header.frame_id = 'map'
    mypose.pose.position.x = goal[0]
    mypose.pose.position.y = goal[1]
    mypose.pose.position.z = goal[2]
    mypose.pose.orientation.x = goal[3]
    mypose.pose.orientation.y = goal[4]
    mypose.pose.orientation.z = goal[5]
    mypose.pose.orientation.w = goal[6]
    return mypose


def callback(data):
    global goal_index, task_flag, distance_flag, accumulated_time, threshold_time, num_per, wheel_stop
    if data.status.status == 3:
        if goal_index < len(goal_list):
            wheel_stop = 0
            accumulated_time = 0
            mypose = set_goal(goal_index)
            # 根据目标索引（goal_index）处理任务
            if goal_index == 2:
                task_flag = 2
                task_flag_msg = Int8()
                task_flag_msg.data = task_flag
                task_flag_pub.publish(task_flag_msg)
                while (task_flag == 2):
                    accumulated_time += 0.1
                    time.sleep(0.1)
                    if accumulated_time >= threshold_time:
                        accumulated_time = 0
                        break
                goal_index += 1
            elif goal_index == 6:
                with open("/home/ucar/Desktop/ucar/src/fame_data.txt", "w+") as fp:
                    fp.write("我已取到急救包")

                subprocess.call("mpg123 /home/ucar/Desktop/ucar/src/mp3/jj.mp3", shell=True)
                if task_flag == 10:
                    mypose = set_goal(11)
                    goal_index = 12
                else:
                    goal_index += 1

            elif goal_index == 7  or goal_index == 5:  # 中朝上 急救包处识别
                time.sleep(0.3)
                # 打开识别
                task_flag = 3
                task_flag_msg = Int8()
                task_flag_msg.data = task_flag
                task_flag_pub.publish(task_flag_msg)
                while (task_flag == 3): # 等待追踪任务完成
                    time.sleep(0.5)
                    accumulated_time += 0.5
                    if not wheel_stop or accumulated_time >= threshold_time:
                        task_flag = 6  #没识别到,关闭识别程序
                        task_flag_msg = Int8()
                        task_flag_msg.data = task_flag
                        task_flag_pub.publish(task_flag_msg)
                        goal_index += 1
                        accumulated_time = 0
                        break
                if task_flag == 10:
                    if goal_index == 5:
                        mypose = set_goal(5)
                        goal_index = 6
                    else:
                        mypose = set_goal(11)
                        goal_index = 12

            elif goal_index == 8:
                time.sleep(0.3)
                # 打开识别
                task_flag = 3
                task_flag_msg = Int8()
                task_flag_msg.data = task_flag
                task_flag_pub.publish(task_flag_msg)
                while (task_flag == 3): # 等待追踪任务完成
                    time.sleep(0.5)
                    accumulated_time += 0.5
                    if not wheel_stop or accumulated_time >= threshold_time:
                        task_flag = 6  #没识别到,关闭识别程序
                        task_flag_msg = Int8()
                        task_flag_msg.data = task_flag
                        task_flag_pub.publish(task_flag_msg)
                        accumulated_time = 0
                        break
                if task_flag == 10:
                    mypose = set_goal(11)
                    goal_index = 12
                else: # 测距
                    task_flag = 5
                    task_flag_msg = Int8()
                    task_flag_msg.data = 5
                    task_flag_pub.publish(task_flag_msg)
                    while (task_flag == 5):
                        time.sleep(0.1)
                    if distance_flag == 1:
                        distance_flag = 0
                        goal_index += 1
                    elif distance_flag == 0:
                        distance_flag = 0
                        mypose = set_goal(goal_index + 1)
                        goal_index += 2
            elif goal_index == 9:#右朝上
                time.sleep(0.3)
                task_flag = 3
                task_flag_msg = Int8()
                task_flag_msg.data = task_flag
                task_flag_pub.publish(task_flag_msg)
                rotate_and_move(twist, 215, 8, pub)
                if wheel_stop:
                    twist.angular.z = 0.0
                    pub.publish(twist)
                while (task_flag == 3): # 等待追踪任务完成
                    time.sleep(0.3)
                    accumulated_time += 0.3
                    if not wheel_stop or accumulated_time >= threshold_time:
                        task_flag = 6  #没识别到,关闭识别程序
                        task_flag_msg = Int8()
                        task_flag_msg.data = task_flag
                        task_flag_pub.publish(task_flag_msg)
                        goal_index += 1
                        accumulated_time = 0
                        break
                    
                if task_flag == 10:
                    mypose = set_goal(11)
                    goal_index = 12
            elif goal_index == 10:#中朝左
                # 打开识别
                task_flag = 3
                task_flag_msg = Int8()
                task_flag_msg.data = task_flag
                task_flag_pub.publish(task_flag_msg)
                while (task_flag == 3): # 等待追踪任务完成
                    time.sleep(0.5)
                    accumulated_time += 0.5
                    if not wheel_stop or accumulated_time >= threshold_time:
                        task_flag = 6  #没识别到,关闭识别程序
                        task_flag_msg = Int8()
                        task_flag_msg.data = task_flag
                        task_flag_pub.publish(task_flag_msg)
                        accumulated_time = 0
                        break
                if task_flag == 10:
                    mypose = set_goal(11)
                    goal_index = 12
                else: # 测距
                    task_flag = 5
                    task_flag_msg = Int8()
                    task_flag_msg.data = 5
                    task_flag_pub.publish(task_flag_msg)
                    while (task_flag == 5):
                        time.sleep(0.1)
                    if distance_flag == 1:
                        distance_flag = 0
                        goal_index += 1
                    elif distance_flag == 0:
                        distance_flag = 0
                        mypose = set_goal(goal_index + 1)
                        goal_index += 2
            elif goal_index == 11:#左朝下
                time.sleep(0.3)
                task_flag = 3
                task_flag_msg = Int8()
                task_flag_msg.data = task_flag
                task_flag_pub.publish(task_flag_msg)
                rotate_and_move(twist, 215, 8, pub)
                if wheel_stop:
                    twist.angular.z = 0.0
                    pub.publish(twist)
                while (task_flag == 3): # 等待追踪任务完成
                    time.sleep(0.3)
                    accumulated_time += 0.3
                    if not wheel_stop or accumulated_time >= threshold_time:
                        task_flag = 6  #没识别到,关闭识别程序
                        task_flag_msg = Int8()
                        task_flag_msg.data = task_flag
                        task_flag_pub.publish(task_flag_msg)
                        goal_index += 1
                        accumulated_time = 0
                        break
                if task_flag == 10:
                    mypose = set_goal(11)
                    goal_index = 12
            elif goal_index == 12:
                if task_flag != 10:
                    if num_per == 1:
                        subprocess.call("mpg123 /home/ucar/Desktop/ucar/src/mp3/w1.mp3", shell=True)
                    elif num_per == 2:
                        subprocess.call("mpg123 /home/ucar/Desktop/ucar/src/mp3/w2.mp3", shell=True)
                    elif num_per == 3:
                        subprocess.call("mpg123 /home/ucar/Desktop/ucar/src/mp3/w3.mp3", shell=True)
                goal_index += 1
            else:
                goal_index += 1

            # 发布下一个目标点
            turtle_vel_pub.publish(mypose)
        else:
            subprocess.call("mpg123 /home/ucar/Desktop/ucar/src/mp3/ff.mp3", shell=True)
            rospy.loginfo("所有目标点均已到达！")
            return

def Task_callback(msg):
    global task_flag
    task_flag = msg.data
    rospy.loginfo("I heard task %d", task_flag)

def wheel_stop_callback(msg):
    global wheel_stop
    wheel_stop = msg.data
    rospy.loginfo("I heard wheel %d", wheel_stop)

def distance_flag_callback(msg):
    global distance_flag
    distance_flag = msg.data
    rospy.loginfo("I heard distance_flag %d", distance_flag)

def person_num_callback(msg):
    global num_per
    num_per = msg.data
    rospy.loginfo("person_number: %d", num_per)

if __name__ == '__main__':
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    twist = Twist()

    rospy.init_node('pubpose')
    turtle_vel_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.Subscriber("/move_base/result", MoveBaseActionResult, callback)
    rospy.wait_for_service('/move_base/make_plan')
    rospy.Subscriber("task_flag", Int8, Task_callback)
    task_flag_pub = rospy.Publisher('task_flag', Int8, queue_size=1)
    rospy.Subscriber("wheel_stop", Int8, wheel_stop_callback)
    rospy.Subscriber("/distance_flag", Int8, distance_flag_callback)
    rospy.Subscriber('person_num', Int8, person_num_callback)

    # 等待唤醒
    while(task_flag == 0):
        time.sleep(0.1)
    rospy.loginfo("I heard %d", task_flag)
    mypose=PoseStamped()
    turtle_vel_pub.publish(mypose) #先发送一个空位置，试探一下，否则第一个包容易丢
    time.sleep(1)


    mypose = set_goal(0)
    turtle_vel_pub.publish(mypose)
    goal_index += 1

    rospy.spin()    





