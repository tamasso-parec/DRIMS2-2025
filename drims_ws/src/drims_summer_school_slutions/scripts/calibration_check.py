#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class ImageToWorld:
    def __init__(self):
        self.intrinsic_matrix = np.array([[1557.024165, 0.000000, 934.265062],[0.000000, 1560.411810, 542.964541],[0.000000, 0.000000, 1.000000]]) # Example intrinsic parameters
        self.extrinsic_matrix = np.array([[0.9987901506204047, 0.022050872545017967, -0.04395445419605139,-0.2910210008461363],
                                            [-0.020573301329190746, 0.9992171970957201, 0.033789529452053754,0.14372418853767385],
                                            [0.04466513512895405, -0.03284436097987389, 0.998461954035173,0.8664238583818229],
                                             [0,0,0,1]]) # Example extrinsic parameters
                                                   
     
        

        # CvBridge
        self.bridge = CvBridge()

        # Subscribe to the image topic
        rospy.Subscriber('/rgb_stereo_publisher/color/image', Image, self.callback)

    def callback(self, image_msg):
        # Convert the ROS Image message to a CV Image
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        #print (cv_image.shape)

        # Show the image and set the mouse click callback
        cv2.imshow('Click to Get Position', cv_image)
        cv2.setMouseCallback('Click to Get Position', self.mouse_click, param=cv_image)
        cv2.waitKey(1)

    def mouse_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            image_point = np.array([[x, y]])
            world_coords = self.image_to_world(image_point, self.intrinsic_matrix, self.extrinsic_matrix)
            print("World coordinates:", world_coords[0], world_coords[1], world_coords[2])

    def image_to_world(self,image_points, intrinsic_matrix, extrinsic_matrix):
      # Invert the intrinsic matrix
      inv_intrinsic_matrix = np.linalg.inv(intrinsic_matrix)

      # Extract the rotation and translation from the extrinsic matrix
      R = extrinsic_matrix[:3, :3]
      t = extrinsic_matrix[:3, 3]
      camera_height = t[2]

      # Invert the rotation and translation (since we're going from camera to world)
      inv_R = np.linalg.inv(R)
      inv_t = -np.dot(inv_R, t)

      # Create homogeneous image points (with z=1)
      homogeneous_image_points = np.column_stack([image_points, np.ones(image_points.shape[0])])

      # Apply the inverse intrinsic matrix to get normalized camera coordinates
      normalized_camera_coords = np.dot(inv_intrinsic_matrix, homogeneous_image_points.T)

      # Since the camera is perpendicular to the ground, the Z component in the camera coordinate system is -camera_height
      scale = camera_height / normalized_camera_coords[2, :]

      # Apply the scale and convert to 3D camera coordinates
      camera_coords = normalized_camera_coords * scale

      # Transform to world coordinates
      world_coords = np.dot(inv_R, camera_coords[:, :3] - t.reshape(-1, 1))

      return world_coords.T[0]
if __name__ == '__main__':
    rospy.init_node('image_to_world', anonymous=True)
    itw = ImageToWorld()
    rospy.spin()

