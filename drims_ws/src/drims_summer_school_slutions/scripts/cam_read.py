#!/usr/bin/env python3

import cv2
import numpy as np
import depthai as dai

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


# Weights to use when blending depth/rgb image (should equal 1.0)
rgbWeight = 0.4
depthWeight = 0.6


def updateBlendWeights(percent_rgb):
    """
    Update the rgb and depth weights used to blend depth/rgb image

    @param[in] percent_rgb The rgb weight expressed as a percentage (0..100)
    """
    global depthWeight
    global rgbWeight
    rgbWeight = float(percent_rgb)/100.0
    depthWeight = 1.0 - rgbWeight


# Optional. If set (True), the ColorCamera is downscaled from 1080p to 720p.
# Otherwise (False), the aligned depth is automatically upscaled to 1080p
downscaleColor = True
fps = 30
# The disparity is computed at this resolution, then upscaled to RGB resolution
monoResolution = dai.MonoCameraProperties.SensorResolution.THE_720_P

# Create pipeline
pipeline = dai.Pipeline()
device = dai.Device()
device.setIrLaserDotProjectorBrightness(1300)
queueNames = []

# Define sources and outputs
camRgb = pipeline.create(dai.node.ColorCamera)
left = pipeline.create(dai.node.MonoCamera)
right = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
stereo.setExtendedDisparity(True)

## camera config
config = stereo.initialConfig.get()
config.postProcessing.speckleFilter.enable = True
config.postProcessing.speckleFilter.speckleRange = 50
config.postProcessing.temporalFilter.enable = True
config.postProcessing.spatialFilter.enable = True
config.postProcessing.spatialFilter.holeFillingRadius = 2
config.postProcessing.spatialFilter.numIterations = 1
config.postProcessing.thresholdFilter.minRange = 400
config.postProcessing.thresholdFilter.maxRange = 15000
stereo.initialConfig.set(config)

rgbOut = pipeline.create(dai.node.XLinkOut)
disparityOut = pipeline.create(dai.node.XLinkOut)

rgbOut.setStreamName("rgb")
queueNames.append("rgb")
disparityOut.setStreamName("disp")
queueNames.append("disp")

#Properties
camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setFps(fps)
if downscaleColor: camRgb.setIspScale(2, 3)
try:
    calibData = device.readCalibration2()
    lensPosition = calibData.getLensPosition(dai.CameraBoardSocket.CAM_A)
    if lensPosition:
        camRgb.initialControl.setManualFocus(lensPosition)
except:
    raise
left.setResolution(monoResolution)
left.setFps(fps)
right.setResolution(monoResolution)
right.setFps(fps)

stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)
# LR-check is required for depth alignment
stereo.setLeftRightCheck(True)
stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
#stereo.setSubpixel(True)
#stereo.setSubpixelFractionalBits(5)
stereo.setExtendedDisparity(False)

# Linking
camRgb.isp.link(rgbOut.input)
left.out.link(stereo.left)
right.out.link(stereo.right)
stereo.disparity.link(disparityOut.input)


rospy.init_node('oak_publisher')

rgb_pub = rospy.Publisher('/oak/rgb/image_raw', Image, queue_size=10)
disparity_pub = rospy.Publisher('/oak/stereo/image_raw', Image, queue_size=10)

bridge = CvBridge()

# Connect to device and start pipeline
with device:
    device.startPipeline(pipeline)

    frameRgb = None
    frameDisp = None


    while not rospy.is_shutdown():
        latestPacket = {}
        latestPacket["rgb"] = None
        latestPacket["disp"] = None

        queueEvents = device.getQueueEvents(("rgb", "disp"))
        for queueName in queueEvents:
            packets = device.getOutputQueue(queueName).tryGetAll()
            if len(packets) > 0:
                latestPacket[queueName] = packets[-1]

        if latestPacket["rgb"] is not None:
            frameRgb = latestPacket["rgb"].getCvFrame()
            ros_image = bridge.cv2_to_imgmsg(frameRgb, encoding="bgr8")
            rgb_pub.publish(ros_image)


        if latestPacket["disp"] is not None:
            frameDisp = latestPacket["disp"].getFrame()
        
            frameDisp = np.ascontiguousarray(frameDisp)
            ros_image = bridge.cv2_to_imgmsg(frameDisp, encoding="8UC1")
            disparity_pub.publish(ros_image)

