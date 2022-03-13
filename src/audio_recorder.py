#!/usr/bin/python3

# Written by Adam Fong
# January 31, 2022
# Node which records audio from usb microphone on RPi

import rospy
from std_msgs.msg import String
import os
import wave
import sounddevice as sd
import time
import math
import rospkg
import scipy.io.wavfile as wav


rospack = rospkg.RosPack()
pck_path = rospack.get_path('bike_sentry_raspi')

def record_audio(duration, filename):
    '''
    Record a .wav file for a defined amount of duration. Output to a user defined filename.
    Return: string that has filename without file extension 
    '''
    full_path = pck_path + "/recordings/" + filename + ".wav"
    fs = 44100
    sd.default.samplerate = fs
    sd.default.channels = 1
    sd.default.dtype = "int16"

    rec = sd.rec(int(duration * fs))
    sd.wait()

    # save .wav
    wav.write(full_path, fs, rec)
    
    rospy.loginfo("Saved recording: {}".format(full_path))
    
    return filename

def main():
    rospy.init_node("audio_recorder")
    
    audio_pub = rospy.Publisher("/recording", String, queue_size = 100)
    # how long to record audio and what to name the recording
    SECONDS = 2
    base_file_name = "recording_{t}"
    
    # set sampling rate
    # if this runs at 2/3 Hz, there should be a slight gap between recordings but that's fine
    rate = rospy.Rate(2/3)
    
    while not rospy.is_shutdown():
        t = time.time()
        file = base_file_name.format(t = math.floor(t))
        n = record_audio(SECONDS, file)
        audio_pub.publish(n) # remember this doesn't have the relative directory or .wav file extension
        rate.sleep()

if __name__ == "__main__":
    main()
