#!/usr/bin/python3

# Written by Adam Fong
# January 31, 2022
# Node which listens to the /recording topic and classifies the files that it is told to
# will eventually output a voltage to some pin to connect to Jetson Nano

# NOT TESTED
import rospy
from std_msgs.msg import String
import os
import rospkg
import sys

rospack = rospkg.RosPack()
pck_path = rospack.get_path('bike_sentry')

path = ('module.name', pck_path + '/scripts')

sys.path.append(path) # adding /scripts directory of ROS package to path to import AudioManager

import AudioManager 

AM = AudioManager.AudioManager()

def listener_rec():
    rospy.Subscriber("/recording", String, classifier_callback)
    
def classifier_callback(filename):
    f = filename.data
    full_file = "../recordings/{}.wav".format(f)
    sample_rate, y = AM.read_wav(full_file)
    y_scaled = AM.process_recording(y, sample_rate)
    
    result = AM.classify_audio(y_scaled)
    rospy.loginfo("{f} is {r}".format(f=f, r=result))

def main():
    rospy.init_node("audio_classifier")
    listener_rec()
    rospy.spin()

if __name__ == "__main__":
    main()