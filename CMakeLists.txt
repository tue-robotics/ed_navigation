cmake_minimum_required(VERSION 2.8.3)
project(ed_navigation)

find_package(catkin REQUIRED COMPONENTS
    ed
    ed_sensor_integration
    message_generation
)

# ------------------------------------------------------------------------------------------------
#                                     ROS MESSAGES AND SERVICES
# ------------------------------------------------------------------------------------------------

# Generate services
add_service_files(
    FILES
    GetGoalConstraint.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
)

# ------------------------------------------------------------------------------------------------
#                                          CATKIN EXPORT
# ------------------------------------------------------------------------------------------------

catkin_package(
)

# ------------------------------------------------------------------------------------------------
#                                              BUILD
# ------------------------------------------------------------------------------------------------

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(ed_navigation_plugin
    src/map.cpp
    src/navigation_plugin.cpp
    src/occupancy_grid_publisher.cpp
    src/depth_sensor_integrator.cpp
)
target_link_libraries(ed_navigation_plugin ${catkin_LIBRARIES})
