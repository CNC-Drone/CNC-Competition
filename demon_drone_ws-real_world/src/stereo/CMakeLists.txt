cmake_minimum_required(VERSION 3.0.2)
project(stereo)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  sensor_msgs
  image_transport
  pcl_conversions
  roscpp
)
find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/stereo_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  stereo_match 
)