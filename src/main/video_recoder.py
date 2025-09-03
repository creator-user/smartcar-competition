# -*- coding: utf-8 -*-
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class VideoRecorder:
    def __init__(self):
        rospy.init_node('video_recorder', anonymous=True)
        self.image_sub = rospy.Subscriber('/ucar_camera/image_raw', Image, self.image_callback)
        self.bridge = CvBridge()
        self.video_writer = None
        self.recording = True  # 自动开始录制

        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("VideoRecorder node started, waiting for images...")

    def image_callback(self, data):
        rospy.loginfo("Received image")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return

        if self.recording:
            if self.video_writer is None:
                height, width, _ = cv_image.shape
                # 修改保存路径
                self.video_writer = cv2.VideoWriter('/home/ucar/Desktop/ucar/src/output.avi', cv2.VideoWriter_fourcc(*'XVID'), 20, (width, height))
            self.video_writer.write(cv_image)

        cv2.imshow('Image Window', cv_image)
        cv2.waitKey(1)  # 必须的，否则窗口不刷新

    def shutdown(self):
        rospy.loginfo("Shutting down...")
        if self.video_writer is not None:
            self.video_writer.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        VideoRecorder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
