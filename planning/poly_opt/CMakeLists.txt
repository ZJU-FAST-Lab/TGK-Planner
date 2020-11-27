cmake_minimum_required(VERSION 2.8.3)
project(poly_opt)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  occ_grid
  poly_traj_utils
  visualization_utils
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES poly_opt
  CATKIN_DEPENDS occ_grid poly_traj_utils
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library( poly_opt 
  src/traj_optimizer.cpp
)
target_link_libraries( poly_opt
  ${catkin_LIBRARIES} 
)
