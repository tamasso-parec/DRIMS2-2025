#!/usr/bin/env python3

import rospy
from dynamic_reconfigure.server import Server
from ros_intro.cfg import parametersConfig

def callback(config, level):
    rospy.loginfo("Reconfigure Request for logger level {level}: {int_param} {double_param:.2f} {str_param} {bool_param} {size}".format(level=level, **config))
    return config

def main():
    # Initialize the node
    rospy.init_node('dynamic_param', anonymous=False)

    # Setup dynamic reconfigure server
    srv = Server(parametersConfig, callback)

    # Keep the node running
    rospy.loginfo("Spinning node")
    rospy.spin()

if __name__ == '__main__':
    main()
