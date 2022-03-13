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
import requests
import RPi.GPIO as GPIO

rospack = rospkg.RosPack()
pck_path = rospack.get_path('bike_sentry_raspi')

# path = ('module.name', pck_path + '/scripts')
# print(path)
# sys.path.append(path) # adding /scripts directory of ROS package to path to import AudioManager

import AudioManager 

AM = AudioManager.AudioManager(model = pck_path + "/models/binary_classifier.joblib",
                               scaler = pck_path + "/models/mfcc_scaler.pkl")

def classifier_callback(filename):
    f = filename.data
    
    rospy.loginfo("File to classify {f}".format(f=f))
    full_file = pck_path + "/recordings/{}.wav".format(f)
    sample_rate, audio = AM.read_wav(full_file)
    audio_scaled = AM.process_recording(audio, sample_rate)
    
    result = AM.classify_audio(audio_scaled)
    
    # send to topic 
    class_pub = rospy.Publisher("/recording_class", String, queue_size = 100)
    class_pub.publish("{f} is {r}".format(f=f, r=result))
    
    if result == 1:
        theft_alert()
        pin_output(1)
    else:
        pin_output(0)
    
def theft_alert():
    url = r"https://bike-sentry-api-2vgam74tba-uc.a.run.app/theft_alert/T0"
    _ = requests.post(url)

def setup():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    pin = 8
    
    GPIO.setup(pin, GPIO.OUT, initial = GPIO.LOW)

def pin_output(logic):
    if logic == 0:
        GPIO.output(8, GPIO.LOW)
    else:
        GPIO.output(8, GPIO.HIGH)

def testing_tf(input):
    rospy.loginfo("Filename to classify: TEST DAMNit")

def main():
    rospy.init_node("audio_classifier")
    
    rospy.Subscriber("/recording", String, classifier_callback)
    #rospy.Subscriber("/recording", String, testing_tf)
    rospy.spin()

if __name__ == "__main__":
    setup()
    main()
