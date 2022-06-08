cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
project(ros_sharp_driver)

# Catkin dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
)

# Defining the package
catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)