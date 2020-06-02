cmake_minimum_required(VERSION 3.0.2)
project(two_ints_msg)

## use C++11
set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
  FILES
  TwoInts.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

## Tell other catkin packages that they need the message_runtime package
## in order to use the message defined in this package
catkin_package(
  CATKIN_DEPENDS message_runtime
)
