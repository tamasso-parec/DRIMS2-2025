#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def callback(data):
    # Log the received message
    rospy.loginfo("I heard: %s", data.data)

def listener():
    # Initialize the ROS node
    rospy.init_node('listener', anonymous=True)

    # Create a subscriber to the 'chatter' topic
    rospy.Subscriber('chatter', String, callback)

    # Keep the node alive and receiving messages
    rospy.spin()

if __name__ == '__main__':
    listener()

