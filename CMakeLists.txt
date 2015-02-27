cmake_minimum_required(VERSION 2.8.3)
project(ed_navigation)

find_package(catkin REQUIRED COMPONENTS
    ed
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
     src/navigation_plugin.cpp
     src/occupancy_grid_publisher.cpp
)
target_link_libraries(ed_navigation_plugin ${catkin_LIBRARIES})
