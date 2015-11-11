cmake_minimum_required(VERSION 2.8.3)
project(subscription)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg)

## Declare ROS messages and services
add_message_files(DIRECTORY msg FILES Num.msg)
add_service_files(DIRECTORY srv FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)


catkin_package()
## Declare a catkin package
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(subscriber src/subscriber2.cpp)
target_link_libraries(subscriber ${catkin_LIBRARIES})
add_dependencies(subscriber Subsription)
