#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import dynamic_reconfigure.server
from drims_summer_school_solution.cfg import ColorDetectorConfig
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int16

def matrix_to_euler_angles(r):
    # Assuming r is a 3x3 rotation matrix
    assert r.shape == (3, 3)

    sy = np.sqrt(r[0, 0] ** 2 + r[1, 0] ** 2)

    singular = sy < 1e-6  # If singular, it's gimbal lock

    if not singular:
        x = np.arctan2(r[2, 1], r[2, 2])
        y = np.arctan2(-r[2, 0], sy)
        z = np.arctan2(r[1, 0], r[0, 0])
    else:
        x = np.arctan2(-r[1, 2], r[1, 1])
        y = np.arctan2(-r[2, 0], sy)
        z = 0

    return np.array([x, y, z])

class YellowObjectDetection:

    def __init__(self):
        rospy.init_node('color_object_detection', anonymous=False)

        self.bridge = CvBridge()

        # Subscribe to the RGB image topic
        rospy.Subscriber('/rgb_stereo_publisher/color/image', Image, self.callback)

        # Publishers for color bounding box and final image with red crosses
        self.yellow_bbox_pub = rospy.Publisher('/oak/yellow_bbox', Image, queue_size=10)
        self.final_image_pub = rospy.Publisher('/oak/final_image', Image, queue_size=10)
        self.axis_pub = rospy.Publisher('/oak/axis', Image, queue_size=10)
        self.yellow_mask_pub = rospy.Publisher('/oak/yellow_mask', Image, queue_size=10)
        self.dot_mask_pub = rospy.Publisher('/oak/dot_mask', Image, queue_size=10)
        self.final_pose_pub = rospy.Publisher('/dice_pose', PoseStamped, queue_size=10)
        self.dice_value_pub = rospy.Publisher('/dice_value', Int16, queue_size=10)

        # History of detected black points
        self.history = []
        
        # Min and max size in pixel of the dice
        self.min_dice_size = 1500
        self.max_dice_size = 5000

        # Number of recent frames to consider
        self.history_length = 20

        # Maximum allowed distance for a point to be considered the same between frames
        self.max_distance = 5
        
        # circle filtering parameters
        self.min_contours = 5
        self.max_contours = 45
        self.roundness = 0.75

        #threshold
        self.min_yellow_h = 20
        self.min_yellow_s = 150
        self.min_yellow_v = 150
        self.max_yellow_h = 40
        self.max_yellow_s = 255
        self.max_yellow_v = 255
        
        self.min_black_h = 0
        self.min_black_s = 0
        self.min_black_v = 0
        self.max_black_h = 100
        self.max_black_s = 100
        self.max_black_v = 10
        
        
        #camera data
        self.intrinsic_matrix = np.array([[1557.303030, 0.000000, 957.549517],[0.000000, 1588.170044, 532.507686],[0.000000, 0.000000, 1.000000]]) # Example intrinsic parameters
        self.distortion = np.array([38.68403244018555, -284.3472900390625, -0.0023334568832069635, 0.0014240274904295802, 617.3338623046875, 38.054351806640625, -280.6847839355469, 609.9551391601562])

        # self.extrinsic_matrix = np.array([[0.9987901506204047, 0.022050872545017967, -0.04395445419605139,-0.2910210008461363],
        #                                     [-0.020573301329190746, 0.9992171970957201, 0.033789529452053754,0.14372418853767385],
        #                                     [0.04466513512895405, -0.03284436097987389, 0.998461954035173,0.8664238583818229],
        #                                   [0,0,0,1]]) # Example extrinsic parameters GOFA
        self.extrinsic_matrix = np.array( [[0.9977190937185407, -0.032988770060911055, 0.058892708201626856,-0.24301328240316153],
                                            [0.03202503405352446, 0.9993384737554155, 0.017234038008177723,0.1226216843681515],
                                            [-0.0594222788466096, -0.015308687796968569, 0.9981155428379075,0.7716245862262272],
                                             [0,0,0,1]]) # Example extrinsic parameters YUMI
        self.camera_height = self.extrinsic_matrix[2][3] #Camera height above the ground in meters
        
        
        # self.robot_check = np.array([[-1.0,  0.0,   0.0,  0.45],
        #                              [0.0,  1.0,   0.0,   -0.637],
        #                              [0.0,  0.0,   -1.0,  0.0],
        #                              [0,0,0,1]]) # Example extrinsic parameters GOFA

        self.robot_check = np.array([[-1.0,  0.0,   0.0,  0.327],
                                     [0.0,  1.0,   0.0,   -0.299],
                                     [0.0,  0.0,   -1.0,   0.0850],
                                     [0,0,0,1]]) # Example extrinsic parameters YUMI
        
                                      
        
        # Set up dynamic reconfigure server
        self.dyn_reconf_server = dynamic_reconfigure.server.Server(ColorDetectorConfig, self.dynamic_reconfigure_callback)
        

        rospy.spin()
        
       

    def dynamic_reconfigure_callback(self, config, level):
        self.min_contours = config["min_contours"]
        self.max_contours = config["max_contours"]
        self.roundness = config["roundness"]
        
        self.min_dice_size = config["min_dice_size"]
        self.max_dice_size = config["max_dice_size"]
        
        self.min_yellow_h = config["min_yellow_h"]
        self.min_yellow_s = config["min_yellow_s"]
        self.min_yellow_v = config["min_yellow_v"]
        self.max_yellow_h = config["max_yellow_h"]
        self.max_yellow_s = config["max_yellow_s"]
        self.max_yellow_v = config["max_yellow_v"]
        
        self.min_black_h = config["min_black_h"]
        self.min_black_s = config["min_black_s"]
        self.min_black_v = config["min_black_v"]
        self.max_black_h = config["max_black_h"]
        self.max_black_s = config["max_black_s"]
        self.max_black_v = config["max_black_v"]
        
        return config
        
        
    def find_consistent_points(self):
        # Counts the number of occurrences of each point
        point_counts = {}

        # Iterate through the history and count the occurrences
        for frame_points in self.history:
            for point in frame_points:
                # Check if the point is close to any existing points
                found_existing_point = False
                for existing_point in point_counts.keys():
                    if self.distance(point, existing_point) < self.max_distance:
                        # Increment the count for the existing point
                        point_counts[existing_point] += 1
                        found_existing_point = True
                        break

                if not found_existing_point:
                    # Add the point as a new point
                    point_counts[point] = 1

        # Filter the points that have been consistently detected in the recent frames
        consistent_points = [point for point, count in point_counts.items() if count >= self.history_length - 1]

        return consistent_points

    def distance(self, point1, point2):
        return np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)
        
   
    def pose_to_homogeneous_matrix(self,x, y, theta):
        return np.array([
            [np.cos(theta), -np.sin(theta), 0,x],
            [np.sin(theta), np.cos(theta),0, y],
            [0, 0, 1, -0.03],
            [0, 0, 0, 1]
        ])        
        
    def image_to_world(self,image_points, intrinsic_matrix, extrinsic_matrix, camera_height):
    # Invert the intrinsic matrix
      inv_intrinsic_matrix = np.linalg.inv(intrinsic_matrix)

      # Extract the rotation and translation from the extrinsic matrix
      R = extrinsic_matrix[:3, :3]
      t = extrinsic_matrix[:3, 3]

      # Invert the rotation and translation (since we're going from camera to world)
      inv_R = np.linalg.inv(R)
      inv_t = -np.dot(inv_R, t)

      # Create homogeneous image points (with z=1)
      homogeneous_image_points = np.column_stack([image_points, np.ones(image_points.shape[0])])

      # Apply the inverse intrinsic matrix to get normalized camera coordinates
      normalized_camera_coords = np.dot(inv_intrinsic_matrix, homogeneous_image_points.T)
      
      # Compute scale based on height
      scale = camera_height / normalized_camera_coords[2, :]

      # Apply the scale and convert to 3D camera coordinates
      camera_coords = normalized_camera_coords * scale

      # Transform to world coordinates
      world_coords = np.dot(inv_R, camera_coords[:, :3] - t.reshape(-1, 1))
      
      return world_coords.T

    def callback(self, rgb_image_msg):
        try:
            # Convert the RGB image message to a CV image
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_image_msg, "bgr8")
            #print (rgb_image.shape)
            rgb_image = cv2.undistort (rgb_image, self.intrinsic_matrix,self.distortion)

            # Threshold to isolate yellow color
            hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
            lower_yellow = np.array([self.min_yellow_h, self.min_yellow_s, self.min_yellow_v])
            upper_yellow = np.array([self.max_yellow_h, self.max_yellow_s, self.max_yellow_v])
            yellow_mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
            self.yellow_mask_pub.publish(self.bridge.cv2_to_imgmsg(yellow_mask, "8UC1"))

            # Find contours in the yellow mask
            yellow_contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Find the bounding rectangle of the yellow area
            if yellow_contours:
                
            
                filtered_contours = []
            
                for contour in yellow_contours:
                  # Calculate the bounding rectangle
                  x, y, w, h = cv2.boundingRect(contour)
                  
                  # Calculate the aspect ratio
                  aspect_ratio = float(w) / h
                  size = w*h
                  #print (size)
                  # Check if the aspect ratio is between 2:1 and 1:2
                  #print (size) 
                  if 0.5 <= aspect_ratio <= 2.0 and self.min_dice_size <= size <= self.max_dice_size:
                       filtered_contours.append(contour)
                       
                       
                 
                #print (filtered_contours)
                if filtered_contours:
                  largest_yellow_contour = max(filtered_contours, key=cv2.contourArea)
                else:
                  rospy.loginfo("No dice detected")
                  return
                x, y, w, h = cv2.boundingRect(largest_yellow_contour)

                # Draw the bounding rectangle on the original image
                yellow_bbox_image = np.copy(rgb_image)
                #cv2.rectangle(yellow_bbox_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                #print(largest_yellow_contour)
                
                rect = cv2.minAreaRect(largest_yellow_contour)
                center, size, angle = rect
                if angle < 45:
                  angle = angle + 90
                elif angle > 135:
                  angle = angle - 90
                box = cv2.boxPoints(rect)
                #print(box)
                box = np.int0(box)
                #print(box)
                
                # Draw the rectangle
                cv2.drawContours(yellow_bbox_image, [box], 0, (255, 0, 0), 2)

                # Publish the image with the yellow bounding box
                self.yellow_bbox_pub.publish(self.bridge.cv2_to_imgmsg(yellow_bbox_image, "bgr8"))

                # Extract the region of interest (ROI) from the yellow area
                roi = rgb_image[y:y+h, x:x+w]

                # Threshold to isolate black color in the ROI
                lower_black = np.array([self.min_black_h, self.min_black_s, self.min_black_v])
                upper_black = np.array([self.max_black_h, self.max_black_s, self.max_black_v])
                black_mask = cv2.inRange(roi, lower_black, upper_black)
                self.dot_mask_pub.publish(self.bridge.cv2_to_imgmsg(black_mask, "8UC1"))

                # Find contours in the black mask
                contours, _ = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                # Detected black points in the current frame
                current_points = []

                # Iterate through each contour and find the black points
                for contour in contours:
                    # Ignore small contours that are likely noise
                    if cv2.contourArea(contour) < self.min_contours or cv2.contourArea(contour) > self.max_contours:
                        continue
                        
                    # Area and perimeter
                    area = cv2.contourArea(contour)
                    perimeter = cv2.arcLength(contour, True)

                    # Roundness metric
                    if perimeter > 0:
                        point_roundness = 4 * np.pi * area / (perimeter ** 2)

                        # check if is almost circular
                        if point_roundness < self.roundness: 
                            continue
                            
                    # Find the center of the contour
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        center_x = int(M["m10"] / M["m00"]) + x
                        center_y = int(M["m01"] / M["m00"]) + y
                    else:
                        continue

                    current_points.append((center_x, center_y))

                # Add the current points to the history and keep only the recent frames
                self.history.append(current_points)
                self.history = self.history[-self.history_length:]

                # Find points that have been consistently detected
                consistent_points = self.find_consistent_points()

                # Draw the red "X" cross for consistent points
                for point in consistent_points:
                    center_x, center_y = point
                    cv2.line(rgb_image, (center_x - 5, center_y - 5), (center_x + 5, center_y + 5), (0, 0, 255), 2)
                    cv2.line(rgb_image, (center_x - 5, center_y + 5), (center_x + 5, center_y - 5), (0, 0, 255), 2)

                # Publish the final image with red crosses
                self.final_image_pub.publish(self.bridge.cv2_to_imgmsg(rgb_image, "bgr8"))
                
                # Baricenter in camera coordinates
                if (len(consistent_points)> 0):
                  baricenter_camera = np.mean(consistent_points, axis=0)
                  world_point = self.image_to_world(np.array([[baricenter_camera[0], baricenter_camera[1]]]), self.intrinsic_matrix, self.extrinsic_matrix, self.camera_height)
                  dice_value = len (consistent_points)
                  
                  # Draw RG Axes
                  axis_length = 50
                  # X axis (Red)
                  x_axis = (int(baricenter_camera[0] + axis_length * np.cos(np.radians(angle))), 
                            int(baricenter_camera[1] + axis_length * np.sin(np.radians(angle))))
                  
                  cv2.line(yellow_bbox_image, (int(baricenter_camera[0]),int(baricenter_camera[1])), x_axis, (0, 0, 255), 2)
                  
                  # Y axis (Green)
                  y_axis = (int(baricenter_camera[0] + axis_length * np.cos(np.radians(angle-90))), 
                            int(baricenter_camera[1] + axis_length * np.sin(np.radians(angle-90))))
                  cv2.line(yellow_bbox_image, (int(baricenter_camera[0]),int(baricenter_camera[1])), y_axis, (0, 255, 0), 2)
                  
                  
                  # Annotate the center values
                  text_offset = 20
                  center_text = f"({dice_value}: {world_point[0][0]:.4f}, {world_point[0][1]:.4f})"
                  text_position = (int(baricenter_camera[0])  + text_offset, int(baricenter_camera[1])  - text_offset)
                  cv2.putText(yellow_bbox_image, center_text, text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                  
                  self.axis_pub.publish(self.bridge.cv2_to_imgmsg(yellow_bbox_image, "bgr8"))
                  
                  rospy.loginfo(f"Dice value: {dice_value}, position: ({world_point[0][0]} m,{world_point[0][1]} m, angle:{angle} deg )")
                  
                  #convert to robot coodinates
                  T_aruco_obj = self.pose_to_homogeneous_matrix (world_point[0][0], world_point[0][1], angle*0.0174533)
                  
                  
                  
                  final_pose = self.robot_check @ T_aruco_obj
                  
                  #print (final_pose)
                  
                  r = final_pose [0:3,0:3]
                  #print (matrix)
                  
                  rot = Rotation.from_matrix(r)
                  roll, pitch, yaw = rot.as_euler('xyz', degrees=False)
                  quaternion = rot.as_quat()
                  print ("final pose")
                  print (f"Dice value: {dice_value}, position: ({final_pose[0][3]:.4f},{final_pose[1][3]:.4f},{final_pose[2][3]:.4f}), orientation: ({roll:.4f},{pitch:.4f},{yaw:.4f})  ")                    
                  
                  #print ("final pose leggibile")
                  #rospy.loginfo(f"Dice value: {dice_value}, position: ({final_pose[0][0]} m,{world_point[0][1]} m, angle:{angle} deg )")
                  
                  #publish the final pose
                  pose_message = PoseStamped()
                  pose_message.header.frame_id = "robot" 
                  pose_message.header.stamp = rospy.get_rostime()
                  pose_message.pose.position.x = final_pose[0][3]
                  pose_message.pose.position.y = final_pose[1][3]
                  pose_message.pose.position.z = final_pose[2][3]
                  pose_message.pose.orientation.x = quaternion[0]
                  pose_message.pose.orientation.y = quaternion[1]
                  pose_message.pose.orientation.z = quaternion[2]
                  pose_message.pose.orientation.w = quaternion[3]
                  
                  self.final_pose_pub.publish(pose_message)
                  
                  value_msg = Int16()
                  value_msg.data = dice_value
                  self.dice_value_pub.publish (value_msg)
                  
                  
                  
              
                else:
                  rospy.loginfo("Not enough points")


    


        except CvBridgeError as e:
            rospy.logerr(e)

if __name__ == '__main__':
    YellowObjectDetection()

