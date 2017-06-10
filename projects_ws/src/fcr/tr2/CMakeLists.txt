cmake_minimum_required(VERSION 2.8.3)
project(tr2)

find_package( OpenCV REQUIRED )
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

add_executable( tr2_node src/node.cpp src/trabalho.cpp src/topological_map.cpp src/graph.cpp src/point_kinematics.cpp src/obstacle_avoidance.cpp src/occupancy_grid.cpp)

target_link_libraries(tr2_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
