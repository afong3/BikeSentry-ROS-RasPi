#!/usr/bin/python3

# Written by Adam Fong
# January 31, 2022
# Node which records audio from usb microphone on RPi

import rospy
from std_msgs.msg import String
import os
import wave
import pyaudio
import time
import math
import rospkg


rospack = rospkg.RosPack()
pck_path = rospack.get_path('bike_sentry_raspi')

def record_audio(seconds, filename):
    '''
    Record a .wav file for a defined amount of seconds. Output to a user defined filename.
    Return: string that has filename without file extension 
    '''
    form_1 = pyaudio.paInt16 # 16-bit resolution
    chans = 1 # 1 channel
    samp_rate = 44100 # 44.1kHz sampling rate
    chunk = 4096 # 2^12 samples for buffer
    record_secs = seconds # seconds to record
    dev_index = 2 # device index found by p.get_device_info_by_index(ii)
    recordings_directory = pck_path + '/recordings/'
    wav_output_filename = recordings_directory + filename + ".wav" # name of .wav file

    audio = pyaudio.PyAudio() # create pyaudio instantiation

    # create pyaudio stream
    stream = audio.open(format = form_1,rate = samp_rate,channels = chans, \
                        input_device_index = dev_index,input = True, \
                        frames_per_buffer=chunk)
    frames = []

    # loop through stream and append audio chunks to frame array
    for ii in range(0,int((samp_rate/chunk)*record_secs)):
        data = stream.read(chunk)
        frames.append(data)

    # stop the stream, close it, and terminate the pyaudio instantiation
    stream.stop_stream()
    stream.close()
    audio.terminate()

    # save the audio frames as .wav file
    wavefile = wave.open(wav_output_filename,'wb')
    wavefile.setnchannels(chans)
    wavefile.setsampwidth(audio.get_sample_size(form_1))
    wavefile.setframerate(samp_rate)
    wavefile.writeframes(b''.join(frames))
    wavefile.close()

    rospy.loginfo(("{} saved".format(filename)))
    
    return filename

def main():
    rospy.init_node("audio_recorder")
    
    audio_pub = rospy.Publisher("/recording", String, queue_size = 100)
    # how long to record audio and what to name the recording
    SECONDS = 0.5
    base_file_name = "recording_{t}"
    
    # set sampling rate
    # if this runs at 2/3 Hz, there should be a slight gap between recordings but that's fine
    rate = rospy.Rate(1)
    
    while not rospy.is_shutdown():
        t = time.time()
        file = base_file_name.format(t = math.floor(t))
        n = record_audio(SECONDS, file)
        audio_pub.publish(n) # remember this doesn't have the relative directory or .wav file extension
        audio_pub.publish("test")
        rate.sleep()

if __name__ == "__main__":
    main()
