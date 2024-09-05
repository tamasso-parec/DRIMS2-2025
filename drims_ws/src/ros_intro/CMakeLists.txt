cmake_minimum_required(VERSION 2.8.3)
project(ros_intro)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  dynamic_reconfigure
  sensor_msgs
  cv_bridge
  message_generation
)


## Added for custom messages
add_message_files(
 FILES
 Num.msg

 )

# Added for services
add_service_files(
  FILES
  AddTwoInts.srv
)

## Added for both custom messages and services 
generate_messages(
   DEPENDENCIES
  std_msgs
 )

## Added for dynamic reconfigure
generate_dynamic_reconfigure_options(
  cfg/parameters.cfg
  
) 


## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp rospy sensor_msgs cv_bridge message_runtime
)



install(PROGRAMS
  scripts/publisher.py
  scripts/subscriber.py
  scripts/custom_subscriber.py
  scripts/service_server.py
  scritps/service_client.py
  scripts/timer.py
  scripts/tf_demo.py
  scripts/static_parameter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



