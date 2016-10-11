cmake_minimum_required(VERSION 2.8.3)
project(save_load_mesh)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(ADDITIONAL_CXX_FLAG "-Wall -O3 -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ADDITIONAL_CXX_FLAG}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  chisel_msgs
)
find_package(OpenGL REQUIRED)

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

add_executable(gl_viewer src/gl_viewer.cpp src/utils.cpp)
target_link_libraries(gl_viewer GL glut GLU X11 ${catkin_LIBRARIES})
