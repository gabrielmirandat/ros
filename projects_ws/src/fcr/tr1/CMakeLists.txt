cmake_minimum_required(VERSION 2.8.3)
project(tr1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  p2os_msgs
  tf
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs tf p2os_msgs
)

include_directories(
  include
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(obstacle_avoidance_node src/obstacle_avoidance_node.cpp src/obstacle_avoidance.cpp)
target_link_libraries(obstacle_avoidance_node ${catkin_LIBRARIES})
