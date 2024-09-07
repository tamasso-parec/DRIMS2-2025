#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class ExampleImgNode:
    def __init__(self):
        # Initialize the node
        rospy.init_node('example_img_node', anonymous=True)
        
        # Subscriber to the input image topic
        self.image_sub = rospy.Subscriber("/rgb_stereo_publisher/color/image", Image, self.image_callback)
        
        # Publisher for the output image with the cross
        self.image_pub = rospy.Publisher("/camera/cross_image", Image, queue_size=1)
        
        # Bridge to convert ROS Image messages to OpenCV images
        self.bridge = CvBridge()

    def image_callback(self, rgb_image_msg):
        try:
            # Convert the RGB image message to a CV image
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_image_msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("cv_bridge exception: %s", e)
            return
        
        # Draw a cross in the middle of the image
        img_center_x = rgb_image.shape[1] // 2
        img_center_y = rgb_image.shape[0] // 2
        cv2.drawMarker(rgb_image, (img_center_x, img_center_y), (255, 0, 0), markerType=cv2.MARKER_CROSS, markerSize=40, thickness=2)

        # Show the image with the cross (optional for debugging)
        #cv2.imshow("Cross Image", rgb_image)
        #cv2.waitKey(30)

        # Publish the image with the cross
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(rgb_image, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr("cv_bridge exception: %s", e)

if __name__ == '__main__':
    try:
        node = ExampleImgNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

