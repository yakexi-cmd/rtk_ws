cmake_minimum_required(VERSION 2.8.3)
project(traj_utils)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(my_cv_bridge_DIR "~/disk/thirdlib/cv_bridge_ws/devel/share/my_cv_bridge/cmake")

find_package(catkin REQUIRED COMPONENTS
  bspline_opt
  path_searching
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  my_cv_bridge
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES traj_utils
 CATKIN_DEPENDS path_searching bspline_opt
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library( traj_utils
    src/planning_visualization.cpp 
    src/polynomial_traj.cpp 
    )
target_link_libraries( traj_utils
    ${catkin_LIBRARIES} 
    )  
