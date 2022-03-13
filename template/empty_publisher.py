#!/usr/bin/python2.7

import rospy
from std_msgs.msg import Float32


def publisher():
    pub = rospy.Publisher("example_topic", Float32, queue_size=10)
    rospy.init_node("example_name", anonymous=True)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        example_msg = 0.01
        rospy.loginfo(example_msg)
        pub.publish(example_msg)
        rate.sleep()


if __name__ == "__main__":
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
