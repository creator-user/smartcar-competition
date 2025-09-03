import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

# 识别E区作物，当 x>2.226 时，开始识别，当 y<-2.376 时停止识别
# 识别D区作物，当 y<-3.418 时，开始识别，当 y>-3.418 时停止识别
# 识别C区作物，当 y>-2.498 时，开始识别，当 y<-2.498 时停止识别
# 识别B区作物，当 y<-3.514 时，开始识别，当 y>-3.514 时停止识别
# 识别F区果实，当 y<-1.357 and x<2.226 时，开始识别，当 y>-1.357 时停止识别

def amcl_pose_callback(msg):
    # 处理接收到的位置信息
    # 在这个例子中，我们只打印位置坐标
    position = msg.pose.pose.position
    print("Position: x={}, y={}, z={}".format(position.x, position.y, position.z))

def main():
    # 初始化ROS节点
    rospy.init_node('amcl_pose_listener')

    # 创建订阅者，订阅/amcl_pose话题
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, amcl_pose_callback)

    # 每隔300毫秒获取一次位置信息
    rospy.Rate(1 / 0.3).sleep()

    # 循环等待节点关闭
    rospy.spin()

if __name__ == '__main__':
    main()
