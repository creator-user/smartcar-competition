# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Int8
import subprocess

class FixedDistancePublisher:
    def __init__(self):
        self.px = 0.0  # 机器人的当前位置x
        self.ix = 0.0  # 初始位置x
        self.dis_long = 0.0  # 需要前进的距离
        self.robot_v = 0.3  # 机器人的线速度
        self.is_start = False  # 标志位，表示是否开始移动
        self.num_per = 0
        self.arrive = 0
        self.is_measuring = 0

        rospy.init_node('Fixed_Distance_publisher')

        self.task_flag_pub = rospy.Publisher('task_flag', Int8, queue_size=1)
        self.measure_pub = rospy.Publisher('/start_measurement', Int8, queue_size=1)
        self.command_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        rospy.Subscriber('/start_measurement', Int8, self.start_measurement_callback)
        rospy.Subscriber('/odom', Odometry, self.pose_callback)
        rospy.Subscriber('/go_m', Float32, self.distance_callback)
        rospy.Subscriber('person_num', Int8, self.person_num_callback)

        self.rate = rospy.Rate(10)  # 10 Hz

    def pose_callback(self, msg):
        self.px = msg.pose.pose.position.x
        rospy.loginfo("当前位置: %.2f", self.px)

    def distance_callback(self, msg):
        if msg.data != 0:
            self.dis_long = msg.data
            self.is_start = True
            rospy.loginfo("target_distance: %.2f m", self.dis_long)

    def start_measurement_callback(self, msg):
        self.is_measuring = msg.data
        rospy.loginfo("is_measuring: %d", self.is_measuring)

    def person_num_callback(self, msg):
        self.num_per = msg.data
        rospy.loginfo("person_number: %d", self.num_per)

    def main(self):
        while not rospy.is_shutdown():
            if self.is_start:
                self.ix = self.px
                self.is_start = False

            if not self.is_start and self.is_measuring == 2 and self.arrive == 0:
                count = abs(self.px - self.ix)
                if count < self.dis_long:  # 还没有到达
                    com_msg = Twist()
                    if count < self.dis_long / 5 or count > self.dis_long * 4 / 5:
                        com_msg.linear.x = self.robot_v / 3
                    else:
                        com_msg.linear.x = self.robot_v

                    self.command_pub.publish(com_msg)
                else:
                    if self.is_measuring == 2:
                        # 没有测量状态
                        msg = Int8()
                        msg.data = 0
                        self.measure_pub.publish(msg)
                        # 车停止
                        stop_msg = Twist()
                        self.command_pub.publish(stop_msg)
                        rospy.loginfo("arrive at target: %.2f m", count)
                        # 文件写入
                        with open("/home/ucar/Desktop/ucar/src/fame_data.txt", "w+") as fp:
                            if self.num_per == 3:
                                fp.write("我已取到催泪瓦斯")
                            elif self.num_per == 2:
                                fp.write("我已取到防弹衣")
                            elif self.num_per == 1:
                                fp.write("我已取到警棍")
                        # 播放
                        subprocess.call("~/Desktop/ucar/src/Aisound/bin/result.sh", shell=True)

                        # 回到小车导航状态
                        task_flag_msg = Int8()
                        task_flag_msg.data = 1
                        self.task_flag_pub.publish(task_flag_msg)
                        self.arrive = 1
                        self.is_start = False

            self.rate.sleep()


if __name__ == '__main__':
    try:
        node = FixedDistancePublisher()
        node.main()
    except rospy.ROSInterruptException:
        pass
