import rospy
import time
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped

rospy.init_node('sss')
# 创建一个发布器，用于发布机器人的移动命令
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
# 创建一个Twist消息实例
twist = Twist()

# 设置旋转角度，将目标旋转角度转换为弧度
target_angle = 200  # 目标旋转角度 (度)
target_angle_rad = target_angle * 3.14159 / 180.0
# 设置旋转时间，计算旋转速度
rotation_time = 1  # 旋转时间 (秒)
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