#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import tf
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R

# Define the size of the checkerboard
CHECKERBOARD_SIZE = (6, 8)  # (height, width)
SQUARE_SIZE = 0.018  # Size of each square in meters

class CheckerboardFinder:
    def __init__(self):
        # Initialize the node
        rospy.init_node('checkerboard_finder')

        # Image topic subscriber with queue_size set to 1 for faster updates
        self.image_sub = rospy.Subscriber('/rgb_stereo_publisher/color/image', Image, self.image_callback, queue_size=1)

        # Publisher for processed image (checkerboard with corners)
        self.image_pub = rospy.Publisher('/checkerboard_image', Image, queue_size=1)

        # CV Bridge to convert ROS Image messages to OpenCV
        self.bridge = CvBridge()

        # TF Broadcaster to publish the transform
        self.br = tf.TransformBroadcaster()

        # Define the 3D points of the checkerboard in the checkerboard frame
        self.object_points = np.zeros((CHECKERBOARD_SIZE[0] * CHECKERBOARD_SIZE[1], 3), np.float32)
        self.object_points[:, :2] = np.mgrid[0:CHECKERBOARD_SIZE[1], 0:CHECKERBOARD_SIZE[0]].T.reshape(-1, 2)
        self.object_points *= SQUARE_SIZE

        # Placeholder for camera matrix and distortion coefficients
        self.camera_matrix = np.array([[600, 0, 320], [0, 600, 240], [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.zeros((4, 1))  # Assuming no distortion

    def image_callback(self, data):
        try:
            # Convert the ROS image message to a format OpenCV can use (BGR)
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge error: %s", e)
            return

        # Resize the image for faster processing (reducing resolution)
        resized_image = cv2.resize(cv_image, (0, 0), fx=0.5, fy=0.5)  # 50% of the original size

        # Convert the image to grayscale for corner detection
        gray_image = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)

        # Find the checkerboard pattern (lower accuracy for faster performance)
        found, corners = cv2.findChessboardCorners(gray_image, CHECKERBOARD_SIZE, cv2.CALIB_CB_FAST_CHECK)

        if found:
            # Optionally refine the corner positions using the grayscale image, or skip for speed
            # This can be further tuned for speed vs. accuracy balance
            cv2.cornerSubPix(gray_image, corners, (5, 5), (-1, -1),
                             criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 0.01))

            # Draw the corners on the resized image (for visualization)
            cv2.drawChessboardCorners(resized_image, CHECKERBOARD_SIZE, corners, found)

            # SolvePnP to find the relative position of the checkerboard to the camera
            success, rvec, tvec = cv2.solvePnP(self.object_points, corners, self.camera_matrix, self.dist_coeffs)

            if success:
                # Convert rotation vector (rvec) to a rotation matrix
                rmat, _ = cv2.Rodrigues(rvec)

                 # Directly convert the 3x3 rotation matrix to a quaternion using scipy
                rotation = R.from_matrix(rmat)
                quat = rotation.as_quat()  # Quaternion (x, y, z, w)

                # Publish the transform (from camera to checkerboard)
                self.publish_tf(tvec, quat)

            # Convert the processed image back to a ROS Image message and publish it
            try:
                checkerboard_img_msg = self.bridge.cv2_to_imgmsg(resized_image, "bgr8")
                self.image_pub.publish(checkerboard_img_msg)
            except CvBridgeError as e:
                rospy.logerr("CvBridge error when publishing image: %s", e)

    def publish_tf(self, tvec, quat):
        # Broadcast the transform from 'oak_rgb_camera_optical_frame' to 'checkerboard'
        self.br.sendTransform((tvec[0], tvec[1], tvec[2]),
                              quat,
                              rospy.Time.now(),
                              "checkerboard",
                              "oak_rgb_camera_optical_frame")


if __name__ == '__main__':
    try:
        checkerboard_finder = CheckerboardFinder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
