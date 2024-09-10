cmake_minimum_required(VERSION 2.8.3)
project(drims_summer_school_solution)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  dynamic_reconfigure
  sensor_msgs
  cv_bridge
)
# Generate dynamic reconfigure parameters
generate_dynamic_reconfigure_options(
  cfg/ColorDetectorConfig.cfg
)



## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp rospy sensor_msgs dynamic_reconfigure cv_bridge
)


install(PROGRAMS
  scripts/filter_haight.py
  scripts/cam_read.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



