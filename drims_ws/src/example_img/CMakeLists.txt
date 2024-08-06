cmake_minimum_required(VERSION 3.0.2)
project(example_img)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(example_img src/example_img.cpp)
target_link_libraries(example_img ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
