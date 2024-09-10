#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class CameraCalibration:
    def __init__(self):
        # CvBridge
        self.bridge = CvBridge()

        # Checkerboard dimensions
        self.checkerboard_size = (8, 6)
        self.square_size = 0.018

        # Intrinsic camera matrix
        #self.camera_matrix = np.array([[1031.260207, 0, 654.662578],[0, 1038.279897, 373.823873],[0, 0, 1]]) # Example intrinsic parameters
        #self.distortion = np.array([0.108085, -0.185623, -0.000974, 0.001274, 0.000000])
        self.camera_matrix = np.array([[1557.024165, 0.000000, 934.265062],[0.000000, 1560.411810, 542.964541],[0.000000, 0.000000, 1.000000]]) # Example intrinsic parameters
        self.distortion = np.array([0.109678, -0.196869, 0.001645, -0.000817, 0.000000])
        

        self.dist_coeffs = np.zeros((4,1)) # Assuming no lens distortion
        self.dist_coeffs = self.distortion 
                               

        # Create object points like (0,0,0), (1,0,0), (2,0,0), ..., (5,3,0)
        self.objp = np.zeros((self.checkerboard_size[0] * self.checkerboard_size[1], 3), np.float32)
        self.objp[:,:2] = np.mgrid[0:self.checkerboard_size[0], 0:self.checkerboard_size[1]].T.reshape(-1,2) * self.square_size

        # Vectors to store object points and image points
        self.obj_points = []
        self.img_points = []

        # Subscribe to the image topic
        self.image_sub = rospy.Subscriber('/rgb_stereo_publisher/color/image', Image, self.callback)

    def callback(self, image_msg):
        # Convert the ROS Image message to a CV Image
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        #cv_image = cv2.undistort (cv_image, self.camera_matrix,self.distortion)

        # Convert to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Find the chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, self.checkerboard_size, None)

        if ret:
            # Refine the detected corners
            corners = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01))

            # Add the object points and image points to the vectors
            
            self.object_points = np.zeros((self.checkerboard_size[0]*self.checkerboard_size[1],3), np.float32)
            self.object_points[:,:2] = self.square_size * np.mgrid[0:self.checkerboard_size[0],0:self.checkerboard_size[1]].T.reshape(-1,2)

            
            self.img_points.append(corners)
            

            # Draw the corners on the image
            cv2.drawChessboardCorners(cv_image, self.checkerboard_size, corners, ret)
            

            # Calibrate the camera
            #ret, _, _, rvecs, tvecs = cv2.calibrateCamera(self.obj_points, self.img_points, gray.shape[::-1], self.camera_matrix, None)
            retval, rvecs, tvecs, _ = cv2.solvePnPRansac(self.object_points, corners, self.camera_matrix, self.dist_coeffs, cv2.SOLVEPNP_IPPE_SQUARE)

            
            # Convert the rotation vector to a rotation matrix
            R, _ = cv2.Rodrigues(rvecs)

            # Combine the rotation matrix and translation vector into the extrinsic matrix
            extrinsic_matrix = np.hstack((R, tvecs))

            rospy.loginfo("-----------------------------------")
            rospy.loginfo("Rotation Vector: [" + str(rvecs[0]) +", "+ str(rvecs[1]) +", "+ str(rvecs[2]) +"]")
            rospy.loginfo("Translation Vector:  [" + str(tvecs[0]) +", "+ str(tvecs[1]) +", "+ str(tvecs[2]) +"]")
            rospy.loginfo("Matrix Vector:  [[" + str(extrinsic_matrix[0][0]) +", "+ str(extrinsic_matrix[0][1]) +", "+ str(extrinsic_matrix[0][2]) +","+ str(extrinsic_matrix[0][3]) +"],\n"+
                          "[" + str(extrinsic_matrix[1][0]) +", "+ str(extrinsic_matrix[1][1]) +", "+ str(extrinsic_matrix[1][2]) +","+ str(extrinsic_matrix[1][3]) +"],\n"+
                          "[" + str(extrinsic_matrix[2][0]) +", "+ str(extrinsic_matrix[2][1]) +", "+ str(extrinsic_matrix[2][2]) +","+ str(extrinsic_matrix[2][3]) +"],\n")
            #rospy.loginfo("Matrix: " + str(extrinsic_matrix))

        # Optionally, display the image with the corners drawn
        cv2.imshow('calibration', cv_image)
        cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('camera_calibration', anonymous=True)
    calibrator = CameraCalibration()
    rospy.spin()

