cmake_minimum_required(VERSION 3.0.2)
project(marker_update)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  fiducial_msgs
  tf
 roslib
indoor_2d_nav
)

#add_message_files(
#  FILES
#  Int16Array.msg
#  FiducialTransform.msg
#  FiducialTransformArray_i2n.msg
#)


## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)



catkin_package(
  INCLUDE_DIRS include
  LIBRARIES marker_update
  CATKIN_DEPENDS cv_bridge roscpp rospy sensor_msgs std_msgs tf roslib
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(marker_update src/marker_update.cpp src/marker_update_node.cpp)
target_link_libraries(marker_update ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(tf_changer src/tf_changer.cpp)
target_link_libraries(tf_changer ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(opencv_sub src/sub.cpp)
#target_link_libraries(opencv_sub ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
