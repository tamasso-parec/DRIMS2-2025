#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def main():
    # Initialize the ROS node
    rospy.init_node('static_param', anonymous=True)

    # Create a publisher for the 'parameter' topic
    chatter_pub = rospy.Publisher('parameter', String, queue_size=1000)

    # Retrieve the '/name' parameter
    name = rospy.get_param('/name', 'default_name')  # Use 'default_name' if the parameter is not set

    # Set the rate to 10 Hz
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # Create the message
        msg = String()
        msg.data = name

        # Log the message
        rospy.loginfo(msg.data)

        # Publish the message
        chatter_pub.publish(msg)

        # Sleep to maintain the loop rate
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
