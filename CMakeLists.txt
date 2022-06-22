cmake_minimum_required(VERSION 3.1)
project(common)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(Poco REQUIRED Foundation Net)

include_directories(
  include
)

link_directories(
  lib/noetic/x86_64
)

add_library(DRFL_robot SHARED
  src/doosan_control.cpp
)
target_link_libraries(DRFL_robot DRFL Poco::Foundation Poco::Net)

add_executable(example src/example.cpp src/doosan_control.cpp)
target_link_libraries(example DRFL_robot)