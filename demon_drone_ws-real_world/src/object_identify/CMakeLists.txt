cmake_minimum_required(VERSION 3.0.2)
project(object_identify)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  image_transport
  object_detection_msgs
)
find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/object_identify_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    people_detect
)
