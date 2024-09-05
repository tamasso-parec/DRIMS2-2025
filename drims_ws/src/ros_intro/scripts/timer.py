#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

# Timer callback function
def timer_callback(event):
    rospy.loginfo(f"Callback called at time: {rospy.Time.now()}")

def timed_talker():
    # Initialize the ROS node
    rospy.init_node('timed_talker')

    # Create a timer that triggers every 0.1 seconds
    timer = rospy.Timer(rospy.Duration(0.1), timer_callback)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        timed_talker()
    except rospy.ROSInterruptException:
        pass
