# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Polygon, Point32, Twist

class CarController:
    def __init__(self):
        rospy.init_node('car_controller', anonymous=True)
        rospy.Subscriber('/detected_rect', Polygon, self.rect_callback)
        rospy.Subscriber('/detected_slope', Float32, self.slope_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.area_threshold = 59000  # 根据需要调整阈值
        self.car_speed = 0.2  # 设置小车速度
        self.rect_area = 0
        self.car_stopped = False  # 标志变量，初始值为 False

    def rect_callback(self, msg):
        if len(msg.points) == 4:
            width = abs(msg.points[1].x - msg.points[0].x)
            height = abs(msg.points[2].y - msg.points[1].y)
            self.rect_area = width * height
            rospy.loginfo(f"Detected rectangle area: {self.rect_area}")

            if self.rect_area < self.area_threshold and not self.car_stopped:
                self.move_car_forward()
            else:
                self.stop_car()

    def slope_callback(self, msg):
        rospy.loginfo(f"Detected line slope: {msg.data}")

    def move_car_forward(self):
        rospy.loginfo(f"Moving car forward with speed: {self.car_speed}")
        twist = Twist()
        twist.linear.x = self.car_speed
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        self.car_stopped = False  # 设置标志变量为 False

    def stop_car(self):
        if not self.car_stopped:
            rospy.loginfo("Stopping car")
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            self.car_stopped = True  # 设置标志变量为 True

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = CarController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
