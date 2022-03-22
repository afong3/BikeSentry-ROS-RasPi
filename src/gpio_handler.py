#!/usr/bin/python3

import rospy
from std_msgs.msg import String
import RPi.GPIO as GPIO
import requests

HAS_TRIGGERED = 0
INPUT_GPIO = 26
OUTPUT_GPIO = 17

def setup():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    
    GPIO.setup(OUTPUT_GPIO, GPIO.OUT)
    GPIO.setup(INPUT_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

def pin_output(logic):
    if logic == 0:
        GPIO.output(17, GPIO.LOW)
    else:
        GPIO.output(17, GPIO.HIGH)

def theft_alert():
    url = r"https://bike-sentry-api-2vgam74tba-uc.a.run.app/theft_alert/T0"
    _ = requests.post(url)

def class_callback(c):
    global HAS_TRIGGERED

    result = int(c.data)
    override_switch = GPIO.input(INPUT_GPIO)
    
    if override_switch == 0: # if override pin is not given power, STATE is forced to be zero
        HAS_TRIGGERED = 0 # reset HAS_TRIGGERED with button press / switch
        pin_output(0)
    elif result == 1 and HAS_TRIGGERED == 0:
        HAS_TRIGGERED = 1
        theft_alert() # don't spam cyclists 
        pin_output(1)
    elif HAS_TRIGGERED == 1:
        pin_output(1)
    else:
        pin_output(0)
        

def main():
    rospy.init_node("gpio_handler")
    rospy.Subscriber("/recording_class", String, class_callback)

if __name__ == "__main__":
    setup()
    main()
    