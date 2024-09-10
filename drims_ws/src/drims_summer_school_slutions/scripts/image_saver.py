#!/usr/bin/env python

import rospy
import cv2
import os
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class ImageSaver:
    def __init__(self):
        # CvBridge
        self.bridge = CvBridge()

        # Counter for image filenames
        self.counter = 0

        # Make sure the directory exists
        self.directory = '/home/simone/calib_temp'
        if not os.path.exists(self.directory):
            os.makedirs(self.directory)

        # Subscribe to the image topic
        rospy.Subscriber('/oak/rgb/image_raw', Image, self.callback)

    def callback(self, image_msg):
        # Convert the ROS Image message to a CV Image
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

        # Save the image
        filename = os.path.join(self.directory, 'image_{}.png'.format(self.counter))
        cv2.imwrite(filename, cv_image)
        rospy.loginfo("Saved image: " + filename)

        self.counter += 1

        # Sleep for 1 second
        rospy.sleep(1)

if __name__ == '__main__':
    rospy.init_node('image_saver', anonymous=True)
    img_saver = ImageSaver()
    rospy.spin()
