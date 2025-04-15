cmake_minimum_required(VERSION 3.5)
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
target_link_libraries(${PROJECT_NAME}_occupancy_grid_publisher ${PROJECT_NAME}_map ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_occupancy_grid_publisher ${catkin_EXPORTED_TARGETS})

add_library(${PROJECT_NAME}_plugin
  src/navigation_plugin.cpp
  src/navigation_plugin.h
)
target_link_libraries(${PROJECT_NAME}_plugin ${PROJECT_NAME}_map ${PROJECT_NAME}_occupancy_grid_publisher ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_plugin ${catkin_EXPORTED_TARGETS})

catkin_install_python(
  PROGRAMS scripts/visualize-symbolic-navigation-goal
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  FILES plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

install(
  TARGETS
    ${PROJECT_NAME}_map
    ${PROJECT_NAME}_occupancy_grid_publisher
    ${PROJECT_NAME}_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  find_package(catkin_lint_cmake REQUIRED)
  catkin_add_catkin_lint_test("-W2 --ignore HEADER_OUTSIDE_PACKAGE_INCLUDE_PATH")
endif()
