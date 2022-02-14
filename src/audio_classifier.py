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
pck_path = rospack.get_path('bike_sentry_raspi')

# path = ('module.name', pck_path + '/scripts')
# print(path)
# sys.path.append(path) # adding /scripts directory of ROS package to path to import AudioManager

import AudioManager 

AM = AudioManager.AudioManager()

def classifier_callback(filename):
    f = filename.data
    rospy.loginfo("WORK!!! {f}".format(f=f))
    full_file = "../recordings/{}.wav".format(f)
    sample_rate, audio = AM.read_wav(full_file)
    audio_scaled = AM.process_recording(audio, sample_rate)
    
    result = AM.classify_audio(audio_scaled)
    rospy.loginfo("{f} is {r}".format(f=f, r=result))

def testing_tf(input):
    rospy.loginfo("Filename to classify: {}".format(input.data))

def main():
    rospy.init_node("audio_classifier")
    #rospy.Subscriber("/recording", String, classifier_callback)
    rospy.Subscriber("/recording", String, testing_tf)
    rospy.spin()

if __name__ == "__main__":
    main()