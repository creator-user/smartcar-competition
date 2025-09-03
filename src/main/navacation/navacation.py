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

# 设置多个目标点
goal_list = [
    [3.079, -0.709, 0.000, 0.000, 0.000, 0.443, 0.897],
    [2.703, -5.321, 0.000, 0.000, 0.000, -0.405, 0.914],
    [4.689, -0.725, 0.000, 0.000, 0.000, 0.951, 0.309],
    [4.448, -5.476, 0.000, 0.000, 0.000, -0.403, 0.915],
    [0.945, -0.376, 0.000, 0.000, 0.000, -0.718, 0.696],
    [0.558, -5.256, 0.000, 0.000, 0.000, 0.977, -0.212],
    [0.756, -2.413, 0.000, 0.000, 0.000, 0.345, 0.939],
    [0.860, -0.577, 0.000, 0.000, 0.000, 0.943, 0.333],
    [-0.080, 0.080, 0.000, 0.000, 0.000,  1.000, 0.000]
]

goal_index = 0
def rotate_and_move(twist, target_angle_deg, rotation_time, pub):
    global goal_index
    # 设置旋转角度，将目标旋转角度转换为弧度
    target_angle_rad = target_angle_deg * 3.14159 / 180.0
    # 计算旋转速度
    angular_speed = target_angle_rad / rotation_time

    # 设置机器人的旋转速度
    twist.angular.z = angular_speed

    # 计算旋转的起始时间和发布频率
    t0 = rospy.Time.now().to_sec()
    rate = rospy.Rate(10)

    # 循环发布旋转命令，直到达到旋转时间
    while rospy.Time.now().to_sec() - t0 < rotation_time:
        pub.publish(twist)
        rate.sleep()

    # 发布停止命令，使机器人停止旋转
    twist.angular.z = 0.0
    pub.publish(twist)
    #if goal_index==7:
    #    time.sleep(1)
    #else:
    time.sleep(0.3)

def callback(data):
    global goal_index

    if data.status.status == 3:
        if goal_index < len(goal_list):

            # 根据目标索引（goal_index）处理旋转和移动
            if goal_index == 1:
                time.sleep(0.3)
                rotate_and_move(twist, -100, 1.2, pub)
                rotate_and_move(twist, -100, 1.2, pub)
            elif goal_index == 2:
                time.sleep(0.3)
                rotate_and_move(twist, -100, 1.2, pub)
                rotate_and_move(twist, -100, 1.2, pub)
                rotate_and_move(twist, -100, 1.2, pub)
            elif goal_index == 3:
                time.sleep(0.3)
                rotate_and_move(twist, -120, 1.4, pub)
                rotate_and_move(twist, -100, 1.2, pub)
                rotate_and_move(twist, -100, 1.2, pub)
            elif goal_index == 4:
                time.sleep(0.3)
                rotate_and_move(twist, -100, 1.2, pub)
                #rotate_and_move(twist, -100, 1.2, pub)
                #rotate_and_move(twist, -100, 1.2, pub)
            elif goal_index == 6:
                #time.sleep(0.5)
                time.sleep(0.3)
                rotate_and_move(twist, 140, 1, pub)
            elif goal_index == 7:
                #time.sleep(0.5)
                time.sleep(0.3)
                #rotate_and_move(twist, 20, 0.5, pub)
                #rotate_and_move(twist, -20, 0.5, pub)
                rotate_and_move(twist, 100, 0.8, pub)
                rotate_and_move(twist, 120, 1.2, pub)
                time.sleep(1)
                rotate_and_move(twist, 60, 0.6, pub)
                time.sleep(1)
                rotate_and_move(twist, -180,1.8, pub)
                #rotate_and_move(twist, 20, 0.5, pub)
                #rotate_and_move(twist, -20, 0.5, pub)
                rotate_and_move(twist, -50, 0.5, pub)

            goal = goal_list[goal_index]
            mypose = PoseStamped()

            # 设置下一个目标点
            mypose.header.frame_id = 'map'
            mypose.pose.position.x = goal[0]
            mypose.pose.position.y = goal[1]
            mypose.pose.position.z = goal[2]
            mypose.pose.orientation.x = goal[3]
            mypose.pose.orientation.y = goal[4]
            mypose.pose.orientation.z = goal[5]
            mypose.pose.orientation.w = goal[6]

            # 发布下一个目标点
            turtle_vel_pub.publish(mypose)
            goal_index += 1
        else:
            rospy.loginfo("所有目标点均已到达！")
            return

# ... 余下的代码保持不变 ...


if __name__ == '__main__':
    # 创建一个发布器，用于发布机器人的移动命令
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    # 创建一个Twist消息实例
    twist = Twist()

    rospy.init_node('pubpose')
    turtle_vel_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.Subscriber("/move_base/result", MoveBaseActionResult, callback)
    rospy.wait_for_service('/move_base/make_plan')

    mypose=PoseStamped()
    turtle_vel_pub.publish(mypose) #先发送一个空位置，试探一下，否则第一个包容易丢
    time.sleep(1)

    mypose = PoseStamped()
    # 设置下一个目标点
    mypose.header.frame_id = 'map'
    mypose.pose.position.x = goal_list[goal_index][0]
    mypose.pose.position.y = goal_list[goal_index][1]
    mypose.pose.position.z = goal_list[goal_index][2]
    mypose.pose.orientation.x = goal_list[goal_index][3]
    mypose.pose.orientation.y = goal_list[goal_index][4]
    mypose.pose.orientation.z = goal_list[goal_index][5]
    mypose.pose.orientation.w = goal_list[goal_index][6]
    turtle_vel_pub.publish(mypose)
    goal_index += 1

    rospy.spin()    





