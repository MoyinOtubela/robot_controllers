cmake_minimum_required(VERSION 2.8.3)
project(robbie_control)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg controller_interface pluginlib)

## Declare ROS messages and services
#add_message_files(FILES Num.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

#add_executable(robot_control src/robot_control.cpp)
#target_link_libraries(robot_control ${catkin_LIBRARIES})
