cmake_minimum_required(VERSION 2.8.3)
project(ed_navigation)

find_package(catkin REQUIRED COMPONENTS
  cb_planner_msgs_srvs
  ed
  ed_sensor_integration
  message_generation
  nav_msgs
  sensor_msgs
  roscpp
  rgbd
)

add_service_files(
  FILES
    GetGoalConstraint.srv
)

generate_messages(
  DEPENDENCIES
)

catkin_package(
  CATKIN_DEPENDS
    cb_planner_msgs_srvs ed message_runtime nav_msgs rgbd
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
  src/depth_sensor_integrator.cpp
  src/map.cpp
  src/kinect_navigation_plugin.cpp
)

target_link_libraries(ed_navigation_plugin ${catkin_LIBRARIES})
add_dependencies(ed_navigation_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(ed_kinect_navigation_plugin ${catkin_LIBRARIES})
add_dependencies(ed_kinect_navigation_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS})
