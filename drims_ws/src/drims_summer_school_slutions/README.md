DRIM Summer School repo

## Scripts
### Calibration
- intrinsic.py: given a set of images compute intrinisc calibration of the camera. Important to set the size of the checkerboard (i.e., number of squares and size in m of each square)
- calibrate.py: perform extrinsic calibration. Compute the relative position of the camera to the checkerboard. It subscribe to an image topic, find the checkerboard, via pnp compute the roto-translation, and print it on the terminal.

### Dice detector
- color_detector.py: main node to perform the dice detection, using rgb images, and compute the position and value of the dice

### Utils
-calibration_check.py: test node to validate the camera claibration. It subscribe to the topic from the camera, and allow the user to click on a live streamed image to get the 3D position of the point
-image_saver.py: utils node to subscribe to a topic and save images
-foxglove.json foxglove config to visualize all the topic published by color_detector.py

