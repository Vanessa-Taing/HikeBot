#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import json
import os

class InfoRetrievalNode:
    def __init__(self):
        rospy.init_node('info_retrieval_node', anonymous=True)

        # Load plant info JSON
        info_path = os.path.expanduser('~/catkin_ws/src/HikeBot/hikebot_identifier/config/plant_descriptions.json')
        try:
            with open(info_path, 'r') as f:
                self.plant_info = json.load(f)
        except Exception as e:
            rospy.logerr(f"Failed to load plant info file: {e}")
            self.plant_info = {}

        # Subscribers and Publishers
        rospy.Subscriber('/plant/name', String, self.plant_name_callback)
        self.info_pub = rospy.Publisher('/plant/info', String, queue_size=10)

        rospy.loginfo("Info Retrieval Node started.")

    def plant_name_callback(self, msg):
        name_conf = msg.data.strip()
        plant_name = name_conf.split('(')[0].strip()  # Remove confidence info
        rospy.loginfo(f"Received plant name: {plant_name}")

        description = self.plant_info.get(plant_name, "No information available for this plant.")
        rospy.loginfo(f"Description: {description}")
        final_message = f"{plant_name}: {description}"
        self.info_pub.publish(final_message)

if __name__ == '__main__':
    try:
        node = InfoRetrievalNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
