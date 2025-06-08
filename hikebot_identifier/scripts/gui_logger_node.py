#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import os
from datetime import datetime

class GUILoggerNode:
    def __init__(self):
        rospy.init_node('gui_logger_node', anonymous=True)

        # Setup log file path in home directory
        home_dir = os.path.expanduser("~/catkin_ws/src/HikeBot/hikebot_identifier")
        self.log_file = os.path.join(home_dir, "plant_log.txt")

        # Subscribe to combined info topic
        rospy.Subscriber('/plant/info', String, self.info_callback)

        rospy.loginfo("GUI Logger Node started. Listening for plant info...")
        self.spin()

    def info_callback(self, msg):
        self.latest_info = msg.data
        self.display()
        self.log_to_file(self.latest_info)

    def display(self):
        print("\n" + "=" * 40)
        print(f"🌿 Plant Info: {self.latest_info}")
        print("=" * 40 + "\n")

    def log_to_file(self, text):
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        entry = f"[{timestamp}] {text}\n"
        try:
            with open(self.log_file, "a") as f:
                f.write(entry)
        except Exception as e:
            rospy.logerr(f"Failed to write to log file: {e}")

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        GUILoggerNode()
    except rospy.ROSInterruptException:
        pass
