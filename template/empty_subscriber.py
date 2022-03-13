#!/usr/bin/python2.7

import rospy
from std_msgs.msg import Float32


def sub_callback(pixels_x):
    pass


def main():
    rospy.init_node("example_node_name")
    rospy.Subscriber("example_topic", Float32, sub_callback)
    rospy.spin()


if __name__ == "__main__":
    main()
