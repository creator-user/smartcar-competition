cmake_minimum_required(VERSION 3.0.2)
project(line_follower2)

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
  LIBRARIES line_follower2
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs cv_bridge image_transport
  DEPENDS OpenCV
  CATKIN_DEPENDS dynamic_reconfigure
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(line_follower2_node src/line_follower2_node.cpp)
target_link_libraries(line_follower2_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(image_hsv src/image_hsv.cpp)

add_dependencies(image_hsv ${PROJECT_NAME}_gencfg)

target_link_libraries(image_hsv
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)