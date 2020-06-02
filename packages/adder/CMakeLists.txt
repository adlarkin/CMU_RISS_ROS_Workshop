cmake_minimum_required(VERSION 3.0.2)
project(adder)

## use C++11
set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  two_ints_msg
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_std_msgs_pub src/std_msgs_publisher.cpp src/rand_int_generator.cpp)
add_executable(${PROJECT_NAME}_std_msgs_sub src/std_msgs_subscriber.cpp)
add_executable(${PROJECT_NAME}_two_ints_msg_pub src/two_ints_msg_publisher.cpp src/rand_int_generator.cpp)
add_executable(${PROJECT_NAME}_two_ints_msg_sub src/two_ints_msg_subscriber.cpp)

target_link_libraries(${PROJECT_NAME}_std_msgs_pub
  ${catkin_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_std_msgs_sub
  ${catkin_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_two_ints_msg_pub
  ${catkin_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_two_ints_msg_sub
  ${catkin_LIBRARIES}
)

install(
  TARGETS
  ${PROJECT_NAME}_std_msgs_pub
  ${PROJECT_NAME}_std_msgs_sub
  ${PROJECT_NAME}_two_ints_msg_pub
  ${PROJECT_NAME}_two_ints_msg_sub
  RUNTIME DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
