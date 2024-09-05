#!/usr/bin/env python3

import rospy
from ros_intro.srv import AddTwoInts, AddTwoIntsResponse  # Import your custom service message

# Callback function for handling the service request
def handle_add_two_ints(req):
    # Sum the two integers from the request
    result = req.a + req.b

    # Log the request and the response
    rospy.loginfo(f"Request: a={req.a}, b={req.b}")
    rospy.loginfo(f"Sending back response: {result}")

    # Return the result as part of the response
    return AddTwoIntsResponse(result)

# Main function
def add_two_ints_server():
    # Initialize the ROS node
    rospy.init_node('add_two_ints_server')

    # Create a service named 'add_two_ints' and register the callback
    service = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)

    # Log that the service is ready
    rospy.loginfo("Ready to add two ints.")

    # Keep the service running
    rospy.spin()

if __name__ == "__main__":
    try:
        add_two_ints_server()
    except rospy.ROSInterruptException:
        pass
