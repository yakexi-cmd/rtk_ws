cmake_minimum_required(VERSION 3.0.2)
project(return_mode)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED
    roscpp
    roslib
    std_msgs
    geometry_msgs
    visualization_msgs
    nav_msgs
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES return_mode
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

add_executable(return_node 
    src/return_node.cpp
    src/MotionGoalPub.cpp
)
target_link_libraries(return_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)