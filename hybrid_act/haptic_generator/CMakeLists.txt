cmake_minimum_required(VERSION 2.8.3)
project(haptic_generator)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
)


## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   IntArray.msg
   WSArray.msg
#   Message2.msg
 )


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

