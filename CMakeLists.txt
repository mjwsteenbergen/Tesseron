cmake_minimum_required(VERSION 3.3)
project(Tesseron)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES
        src/beginner_tutorials/src/listener.cpp
        src/beginner_tutorials/src/talker.cpp src/beginner_tutorials/src/test.cpp src/beginner_tutorials/src/test.h)

add_executable(Tesseron ${SOURCE_FILES})