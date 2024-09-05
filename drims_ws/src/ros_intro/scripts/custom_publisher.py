#!/usr/bin/env python3

import rospy
from ros_intro.msg import Num  # Import your custom message

def talker():
    # Initialize the ROS node named 'talker'
    rospy.init_node('talker', anonymous=True)

    # Create a publisher that publishes Num messages to the 'num_topic' topic
    pub = rospy.Publisher('custom_topic', Num, queue_size=10)

    # Set the loop rate (1 Hz)
    rate = rospy.Rate(1)  # 1 message per second

    # Counter to publish in the message
    counter = 0

    while not rospy.is_shutdown():
        # Create an instance of the Num message
        num_msg = Num()

        # Set the 'num' field of the message
        num_msg.num = counter

        # Log the message data
        rospy.loginfo(f"Publishing: {num_msg.num}")

        # Publish the message
        pub.publish(num_msg)

        # Increment the counter
        counter += 1

        # Sleep for the required time to maintain the loop rate
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
