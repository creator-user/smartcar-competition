import rospy
import time
import sys
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from move_base_msgs.msg import MoveBaseActionResult

# 设置多个目标点
goal_list = [
    [-2.0, 0.100, 0.000, 0.000, 0.000, -0.678, 0.735],
    [-2.0, 0.100, 0.000, 0.000, 0.000, 0.001, 0.7],
    [0.379, 0.004, 0.000, 0.000, 0.000, -0.710, 0.704],
    [0.350, -0.065, 0.000, 0.000, 0.000, 0.999, 0.001],
    [1.0, 1.25, 0.000, 0.000, 0.000, 0.001, 0.999],  # 识别区
    [0.475, 0.1, 0.000, 0.000, 0.000, -0.710, 0.704],
    [0.441, -0.065, 0.000, 0.000, 0.000, 0.94, -0.2],
    [-2.65, -2.01, 0.000, 0.000, 0.000, -0.738, 0.675], # 急救包
    [-2.0, 0.005, 0.000, 0.000, 0.000, 0.999, 0.001],
    [-2.65, -2.01, 0.000, 0.000, 0.000, -0.738, 0.675], # 急救包
    [-2.0, 0.100, 0.000, 0.000, 0.000, 0.001, 0.7],
    [0.441, -0.065, 0.000, 0.000, 0.000, 0.999, 0.001],

    
]

rospy.init_node('pubposase')
turtle_vel_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)

mypose=PoseStamped()
turtle_vel_pub.publish(mypose) #先发送一个空位置，试探一下，否则第一个包容易丢
time.sleep(1)

if __name__ == "__main__":
  if len(sys.argv) != 2:
    print("Usage: python3 one_goal.py <goal_index>")
    sys.exit(1)

  goal_index = int(sys.argv[1])
    
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

	