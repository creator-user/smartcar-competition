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
from std_msgs.msg import String

# 设置多个目标点
goal_list = [
    [2.845, -0.362, 0.000, 0.000, 0.000, 0.201, 0.979],

    [2.790, -3.688, 0.000, 0.000, 0.000, -0.712, 0.703],
    [2.785, -5.307, 0.000, 0.000, 0.000, -0.323, 0.946],

    [4.756, -0.633, 0.000, 0.000, 0.000, 0.952, 0.306],

    [4.385, -5.701, 0.000, 0.000, 0.000, -0.117, 0.993],

    [0.839, -0.420, 0.000, 0.000, 0.000, -0.710, 0.704],

    [0.732, -2.681, 0.000, 0.000, 0.000, -0.714, 0.700],

    [0.614, -5.088, 0.000, 0.000, 0.000, -0.239, 0.971],

    [0.756, -2.681, 0.000, 0.000, 0.000, 0.910, 0.414],

    [0.495, -0.369, 0.000, 0.000, 0.000, 0.960, 0.281],
    [-0.116, 0.038, 0.000, 0.000, 0.000, 1.000, 0.024]

]
goal_num=[0,1,0,2,2,2,0,0,1,2,0,0]
goal_num_index=0
goal_index = 0

ratate_data = [[-100, 1],# E
               [-110, 1], [-140, 1],#D2
               [-130, 1], [-120, 1],  #C
               [-140, 1], [-140, 1],  # B
               [-120, 1], [-100, 1],  # F
               [60,1]
               ]


def callback(data):
    global goal_index
    global goal_num_index
    if data.status.status == 3:
        if goal_index > 10:
            rospy.loginfo("所有的目标点都到了!")
            return 0
        if goal_index!=6:
            time.sleep(1)
        while goal_num[goal_index]!=0:
            goal_num[goal_index]-=1
            # 设置旋转角度，将目标旋转角度转换为弧度
            target_angle = ratate_data[goal_num_index][0]  # 目标旋转角度 (度)
            # 设置旋转时间，计算旋转速度
            rotation_time = ratate_data[goal_num_index][1]  # 旋转时间 (秒)


            target_angle_rad = target_angle * 3.14159 / 180.0
            angular_speed = target_angle_rad / rotation_time  # 旋转速度 (弧度/秒)
            # 设置机器人的旋转速度
            twist.angular.z = angular_speed
            # 计算旋转的时间和发布频率
            t0 = rospy.Time.now().to_sec()
            rate = rospy.Rate(10)  # 发布频率为10Hz
            # 循环发布旋转命令，直到达到旋转时间
            while rospy.Time.now().to_sec() - t0 < rotation_time:
                pub.publish(twist)
                rate.sleep()

            # 发布停止命令，使机器人停止旋转
            twist.angular.z = 0.0
            pub.publish(twist)
            # 创建消息对象
            msg = String()
            msg.data = "A"
            # 发布消息
            rotate_photos.publish(msg)
            time.sleep(1)
            goal_num_index+=1
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
        # 创建消息对象
        msg = String()
        msg.data = "S"
        # 发布消息
        rotate_photos.publish(msg)


if __name__ == '__main__':
    # 创建一个发布器，用于发布机器人的移动命令
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    # 创建一个Twist消息实例
    twist = Twist()

    rospy.init_node('pubpose')
    turtle_vel_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.Subscriber("/move_base/result", MoveBaseActionResult, callback)
    rospy.wait_for_service('/move_base/make_plan')
    # 创建一个发布者，指定要发布的话题和消息类型
    rotate_photos = rospy.Publisher('/rotate', String, queue_size=10)

    mypose = PoseStamped()
    turtle_vel_pub.publish(mypose)  # 先发送一个空位置，试探一下，否则第一个包容易丢
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

