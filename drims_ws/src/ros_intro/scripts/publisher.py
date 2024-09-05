#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def talker():
    # Initialize the ROS node
    rospy.init_node('talker', anonymous=True)

    # Create a publisher that publishes to the 'chatter' topic
    pub = rospy.Publisher('chatter', String, queue_size=10)

    # Set the loop rate (1 Hz)
    rate = rospy.Rate(10)  # 1 message per second

    while not rospy.is_shutdown():
        # The message to be published
        hello_str = "Hello World %s" % rospy.get_time()

        # Log the message
        rospy.loginfo(hello_str)

        # Publish the message
        pub.publish(hello_str)

        # Sleep for the required time to maintain the loop rate
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

