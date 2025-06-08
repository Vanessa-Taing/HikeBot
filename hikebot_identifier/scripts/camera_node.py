#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraNode:
    def __init__(self):
        rospy.init_node('camera_node')

        # Subscriber to voice command trigger
        rospy.Subscriber('/voice_commands/trigger_camera', Bool, self.voice_callback)

        # Publisher for camera images
        self.image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)

        self.bridge = CvBridge()
        self.camera = cv2.VideoCapture(2)
        self.capture_enabled = False

        rospy.loginfo("Camera Node initialized. Waiting for voice trigger...")
        self.run()

    def voice_callback(self, msg):
        if msg.data:
            rospy.loginfo("Voice command received. Capturing image...")
            self.capture_enabled = True

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if self.capture_enabled:
                ret, frame = self.camera.read()
                if ret:
                    ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                    self.image_pub.publish(ros_image)
                    rospy.loginfo("Image published to /camera/image_raw.")
                else:
                    rospy.logwarn("Failed to capture frame from camera.")
                self.capture_enabled = False  # Reset until next voice command
            rate.sleep()

        self.camera.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        CameraNode()
    except rospy.ROSInterruptException:
        pass
