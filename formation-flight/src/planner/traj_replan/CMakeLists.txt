cmake_minimum_required(VERSION 2.8.3)
project(traj_replan)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  traj_utils
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES traj_replan
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
  SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}
)

add_executable(traj_replan_node
  src/traj_replan.cpp 
  )
target_link_libraries(traj_replan_node 
  ${catkin_LIBRARIES}
  )
