import numpy as np
import cv2
import glob

# Termination criteria for the iterative algorithm
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Define the checkerboard dimensions
# Suppose you are using a 8x6 checkerboard, then (w, h) = (8, 6)
w, h = 8, 6

# Suppose each square has a size of 18mm (for example)
square_size = 18e-3 # in meters, or 18mm
objp = np.zeros((w*h,3), np.float32)
objp[:,:2] = np.mgrid[0:w,0:h].T.reshape(-1,2) * square_size

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

# Read images
images = glob.glob('/home/simone/calib_temp/*.png') # or whatever your file extension is

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, (w,h), None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        
        corners_refined = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners_refined)

        # Draw and display the corners
        cv2.drawChessboardCorners(img, (w,h), corners_refined, ret)
        cv2.imshow('img', img)
        cv2.waitKey(500)

cv2.destroyAllWindows()

# Calibrate the camera
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

print("Camera Matrix:", mtx)
print("Distortion Coefficients:", dist)

