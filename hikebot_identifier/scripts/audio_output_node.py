#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import pyttsx3

class AudioOutputNode:
    def __init__(self):
        rospy.init_node('audio_output_node', anonymous=True)
        rospy.Subscriber('/plant/info', String, self.speak_callback)

        self.engine = pyttsx3.init()
        self.engine.setProperty('rate', 150)  # You can tweak rate (default ~200)
        rospy.loginfo("Audio Output Node ready and listening to /plant/info")

    def speak_callback(self, msg):
        text = msg.data
        rospy.loginfo(f"Speaking: {text}")
        self.engine.say(text)
        self.engine.runAndWait()

if __name__ == '__main__':
    try:
        node = AudioOutputNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass