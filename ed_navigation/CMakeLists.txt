cmake_minimum_required(VERSION 3.0.2)
project(ed_navigation)

add_compile_options(-Wall -Werror=all)
add_compile_options(-Wextra -Werror=extra)

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  ed
  ${PROJECT_NAME}_msgs
  geolib2
  nav_msgs
  roscpp
  tue_config
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_map ${PROJECT_NAME}_occupancy_grid_publisher ${PROJECT_NAME}_plugin
  CATKIN_DEPENDS ed geolib2 roscpp tue_config
  DEPENDS OpenCV
)

include_directories(
  include
  SYSTEM
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_map
  include/ed/navigation/map.h
  src/map.cpp
)
target_link_libraries(${PROJECT_NAME}_map ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_occupancy_grid_publisher
  include/ed/navigation/occupancy_grid_publisher.h
  src/occupancy_grid_publisher.cpp
)
target_link_libraries(ed_navigation_occupancy_grid_publisher ed_navigation_map ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_occupancy_grid_publisher ${catkin_EXPORTED_TARGETS})

add_library(${PROJECT_NAME}_plugin
  src/navigation_plugin.h
  src/navigation_plugin.cpp
)
target_link_libraries(${PROJECT_NAME}_plugin ${PROJECT_NAME}_map ${PROJECT_NAME}_occupancy_grid_publisher ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_plugin ${catkin_EXPORTED_TARGETS})
