#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
import pvporcupine
import pyaudio
import struct
import os

def main():
    rospy.init_node('audio_detection_node')
    pub = rospy.Publisher('/voice_commands/trigger_camera', Bool, queue_size=1)
    rospy.loginfo("Audio detection node started.")

    access_key = "VfJn83w3V+upeXs7yWg9IyL/FOqYa3v1Qz9QkgTqM5BowpaNl+VfMg=="  # <-- Replace this!

    keyword_path = os.path.expanduser('~/catkin_ws/src/HikeBot/hikebot_identifier/model/Hey-Hike-Bot_en_linux_v3_0_0.ppn')

    porcupine = pvporcupine.create(
        access_key=access_key,
        keyword_paths=[keyword_path]
    )

    pa = pyaudio.PyAudio()
    audio_stream = pa.open(
        rate=porcupine.sample_rate,
        channels=1,
        format=pyaudio.paInt16,
        input=True,
        frames_per_buffer=porcupine.frame_length
    )

    try:
        while not rospy.is_shutdown():
            pcm = audio_stream.read(porcupine.frame_length, exception_on_overflow=False)
            pcm = struct.unpack_from("h" * porcupine.frame_length, pcm)

            keyword_index = porcupine.process(pcm)
            if keyword_index >= 0:
                rospy.loginfo("Wake word detected!")
                pub.publish(True)

    except rospy.ROSInterruptException:
        pass
    finally:
        audio_stream.stop_stream()
        audio_stream.close()
        pa.terminate()
        porcupine.delete()

if __name__ == '__main__':
    main()