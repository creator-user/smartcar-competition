cmake_minimum_required(VERSION 3.0.2)
project(ang_line_follow)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/HSVThreshold.cfg
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ang_line_follow
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs cv_bridge image_transport
  DEPENDS OpenCV
  CATKIN_DEPENDS dynamic_reconfigure
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(ang_line_follow src/ang_line_follow.cpp)
target_link_libraries(ang_line_follow
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(ang_image_hsv src/image_hsv.cpp)

add_dependencies(ang_image_hsv ${PROJECT_NAME}_gencfg)

target_link_libraries(ang_image_hsv
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)