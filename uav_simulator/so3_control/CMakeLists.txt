cmake_minimum_required(VERSION 2.8.3)
project(so3_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  quadrotor_msgs
  tf
  nodelet
  cmake_utils
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES irobot_msgs
#  CATKIN_DEPENDS geometry_msgs nav_msgs
#  DEPENDS system_lib
)


find_package(Eigen3 REQUIRED)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(SO3Control src/SO3Control.cpp)
target_link_libraries(SO3Control 
    ${catkin_LIBRARIES}
)


add_library(so3_control_nodelet src/so3_control_nodelet.cpp)

target_link_libraries(so3_control_nodelet 
    ${catkin_LIBRARIES}
    SO3Control
)
