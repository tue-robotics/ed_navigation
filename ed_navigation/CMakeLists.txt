cmake_minimum_required(VERSION 2.8.3)
project(ed_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cb_base_navigation_msgs
  ed
  ed_navigation_msgs
  ed_sensor_integration
  nav_msgs
  sensor_msgs
  roscpp
  rgbd
)

catkin_package(
  CATKIN_DEPENDS ed ed_navigation_msgs roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(ed_navigation_plugin
  src/navigation_plugin.cpp
  src/map.cpp
  src/occupancy_grid_publisher.cpp
)
add_library(ed_kinect_navigation_plugin
  src/kinect_navigation_plugin.cpp
  src/map.cpp
  src/depth_sensor_integrator.cpp
)

target_link_libraries(ed_navigation_plugin ${catkin_LIBRARIES})
add_dependencies(ed_navigation_plugin ${catkin_EXPORTED_TARGETS})

target_link_libraries(ed_kinect_navigation_plugin ${catkin_LIBRARIES})
add_dependencies(ed_kinect_navigation_plugin ${catkin_EXPORTED_TARGETS})
