import rospy
import numpy as np
from sensor_msgs.msg import Image, LaserScan
from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import tf2_ros
import tf.transformations

class PanelDetection:
    def __init__(self):
        rospy.init_node('panel_detection')
        self.bridge = CvBridge()

        # 订阅ros_darknet发布的识别结果
        self.bbox_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.bbox_callback)

        # 订阅激光雷达数据
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)

        # 发布导航目标点
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        # TF Buffer和Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        # 相机内参矩阵参数
        self.fx = 823.24
        self.fy = 821.61
        self.cx = 640
        self.cy = 480

        # 初始化属性
        self.panel_detected = False
        self.panel_position = (0, 0)
        self.panel_distance = None
        self.angle_increment = None
        self.angle_min = None

        # 地图范围参数（根据地图文件计算得出）
        self.map_x_min = -10.0
        self.map_x_max = 10.0
        self.map_y_min = -10.0
        self.map_y_max = 10.0

    def bbox_callback(self, msg):
        rospy.loginfo("Received BoundingBoxes message")
        # 处理识别结果，获取目标的中心像素坐标
        detected, u, v = self.process_bboxes(msg)
        if detected:
            self.panel_detected = True
            self.panel_position = (u, v)
            rospy.loginfo(f"Panel detected at position: {self.panel_position}")
        else:
            self.panel_detected = False
            rospy.loginfo("Panel not detected")

    def process_bboxes(self, msg):
        for bbox in msg.bounding_boxes:
            if bbox.Class == "smoke":  # 替换为实际目标类
                x1 = bbox.xmin
                y1 = bbox.ymin
                x2 = bbox.xmax
                y2 = bbox.ymax
                u = (x1 + x2) / 2
                v = (y1 + y2) / 2
                return True, u, v
        return False, None, None

    def lidar_callback(self, msg):
        rospy.loginfo("Received LaserScan message")
        if self.panel_detected:
            self.angle_min = msg.angle_min
            self.angle_increment = msg.angle_increment
            ranges = msg.ranges

            # 计算相机坐标
            u, v = self.panel_position
            Xc, Yc, Zc = self.pixel_to_camera_coords(u, v, self.fx, self.fy, self.cx, self.cy)

            # 使用激光雷达数据确定面板距离和角度
            r, theta = self.calculate_distance_and_angle(u, self.cx, self.fx, self.angle_min, self.angle_increment, ranges)
            if r is not None and theta is not None:
                Xr, Yr = self.calculate_panel_normal(Xc, Yc, r, theta)
                target_angle = self.calculate_target_orientation(Xr, Yr)

                # 转换为地图坐标并导航
                map_x, map_y = self.transform_to_map_frame(Xr, Yr, "base_link")
                if self.is_within_map(map_x, map_y):  # 检查目标点是否在地图范围内
                    self.navigate_to_target(map_x, map_y, target_angle)
                else:
                    rospy.logwarn("目标点超出地图范围")
        else:
            rospy.loginfo("Panel not detected, skipping lidar processing")

    def pixel_to_camera_coords(self, u, v, fx, fy, cx, cy):
        Xc = (u - cx) / fx
        Yc = (v - cy) / fy
        Zc = 1  # 假设深度为1米，实际距离后面计算得出
        return Xc, Yc, Zc

    def calculate_distance_and_angle(self, u, cx, fx, angle_min, angle_increment, ranges):
        theta = np.arctan2(u - cx, fx)
        lidar_index = int((theta - angle_min) / angle_increment)
        if 0 <= lidar_index < len(ranges):
            r = ranges[lidar_index]
            return r, theta
        return None, None

    def calculate_panel_normal(self, Xc, Yc, r, theta):
        Xr = r * np.cos(theta)
        Yr = r * np.sin(theta)
        return Xr, Yr

    def calculate_target_orientation(self, Xr, Yr):
        target_angle = np.arctan2(Yr, Xr)
        return target_angle

    def transform_to_map_frame(self, x, y, frame_id="base_link"):
        try:
            trans = self.tf_buffer.lookup_transform("map", frame_id, rospy.Time(0), rospy.Duration(1.0))
            point_in_robot_frame = np.array([x, y, 0, 1])
            transform_matrix = tf.transformations.quaternion_matrix([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
            transform_matrix[0, 3] = trans.transform.translation.x
            transform_matrix[1, 3] = trans.transform.translation.y
            transform_matrix[2, 3] = trans.transform.translation.z
            point_in_map_frame = np.dot(transform_matrix, point_in_robot_frame)
            return point_in_map_frame[0], point_in_map_frame[1]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("TF transformation failed")
            return None, None

    def is_within_map(self, x, y):
        # 检查目标点是否在已知地图范围内
        return (self.map_x_min <= x <= self.map_x_max) and (self.map_y_min <= y <= self.map_y_max)

    def navigate_to_target(self, map_x, map_y, target_angle):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = map_x
        goal.pose.position.y = map_y
        goal.pose.orientation.z = np.sin(target_angle / 2)
        goal.pose.orientation.w = np.cos(target_angle / 2)
        self.goal_pub.publish(goal)

if __name__ == '__main__':
    panel_detection = PanelDetection()
    rospy.spin()
