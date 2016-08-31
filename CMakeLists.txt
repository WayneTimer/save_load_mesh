cmake_minimum_required(VERSION 2.8.3)
project(save_load_mesh)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  chisel_msgs
)

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(save src/save_mesh.cpp)
target_link_libraries(save ${catkin_LIBRARIES})
add_dependencies(save save_generate_messages_cpp)

add_executable(load src/load_mesh.cpp)
target_link_libraries(load ${catkin_LIBRARIES})
add_dependencies(load save_generate_messages_cpp)
