#!/usr/bin/env python3

import sys
import rospy
from ros_intro.srv import AddTwoInts, AddTwoIntsRequest  # Import your custom service

def add_two_ints_client(x, y):
    # Wait for the service to become available
    rospy.wait_for_service('add_two_ints')

    try:
        # Create a service client for 'add_two_ints'
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)

        # Create a request object
        req = AddTwoIntsRequest(a=x, b=y)

        # Call the service and get the response
        resp = add_two_ints(req)

        # Log and return the result
        rospy.loginfo(f"Sum: {resp.sum}")
        return resp.sum

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return None

if __name__ == "__main__":
    # Initialize the client node
    rospy.init_node('add_two_ints_client')

    # Check if the correct number of arguments is provided
    if len(sys.argv) != 3:
        rospy.loginfo("Usage: add_two_ints_client X Y")
        sys.exit(1)

    # Parse command-line arguments for the two integers
    x = int(sys.argv[1])
    y = int(sys.argv[2])

    # Call the service
    add_two_ints_client(x, y)
