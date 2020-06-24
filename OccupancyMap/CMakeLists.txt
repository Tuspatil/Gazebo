# 2.8.8 required to use PROTOBUF_IMPORT_DIRS
cmake_minimum_required(VERSION 2.8.8 FATAL_ERROR)
FIND_PACKAGE( Boost 1.40 COMPONENTS system REQUIRED)
set (CMAKE_CXX_FLAGS "-g -Wall -std=c++11")

find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
add_library(collision_map_creator SHARED collision_map_creator.cc)
target_link_libraries(collision_map_creator ${GAZEBO_LIBRARIES})
Hi this is VS typing