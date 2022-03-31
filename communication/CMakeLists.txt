cmake_minimum_required(VERSION 3.0.2)
project(communication)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
)

include_directories(${sensor_msgs_INCLUDE_DIRS})


add_message_files(
	FILES
  Image.msg
  Liste_points.msg
  Points.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   identification.srv
   capture.srv
   localisation.srv
)



generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)


catkin_package(

  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs
  message_runtime
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



install(FILES
   launch/roslaunch_cognex.launch
   # myfile2
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} 
)


