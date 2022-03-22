#!/usr/bin/python3

# Written by Adam Fong
# January 31, 2022
# Node which listens to the /recording topic and classifies the files that it is told to
# will eventually output a voltage to some pin to connect to Jetson Nano


import rospy
from std_msgs.msg import String
import rospkg

rospack = rospkg.RosPack()
pck_path = rospack.get_path('bike_sentry_raspi')

# path = ('module.name', pck_path + '/scripts')
# print(path)
# sys.path.append(path) # adding /scripts directory of ROS package to path to import AudioManager

import AudioManager 

HAS_TRIGGERED = False

AM = AudioManager.AudioManager(model = pck_path + "/models/binary_classifier.joblib",
                               scaler = pck_path + "/models/mfcc_scaler.pkl")

def classifier_callback(filename):
    global HAS_TRIGGERED
    
    f = filename.data
    
    rospy.loginfo("File to classify {f}".format(f=f))
    full_file = pck_path + "/recordings/{}.wav".format(f)
    sample_rate, audio = AM.read_wav(full_file)
    audio_scaled = AM.process_recording(audio, sample_rate)
    
    result = AM.classify_audio(audio_scaled)
    
    # send to topic 
    class_pub = rospy.Publisher("/recording_class", String, queue_size = 100)
    class_pub.publish("{r}".format(r = result))
    

def main():
    rospy.init_node("audio_classifier")
    
    rospy.Subscriber("/recording", String, classifier_callback)
    rospy.spin()

if __name__ == "__main__":
    main()
