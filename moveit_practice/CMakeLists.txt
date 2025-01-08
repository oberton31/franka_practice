cmake_minimum_required(VERSION 3.0.2)
project(moveit_practice)

# Load catkin and all dependencies required for your package
find_package(catkin REQUIRED
  COMPONENTS
  moveit_core
  moveit_ros_planning_interface
  moveit_commander
  roscpp
  geometry_msgs
  moveit_visual_tools
  shape_msgs
)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES move_group_practice
  CATKIN_DEPENDS moveit_core moveit_ros_planning_interface moveit_commander roscpp geometry_msgs moveit_visual_tools shape_msgs
)

# Move script
add_executable(move_practice_node src/move.cpp)

# Add dependencies to ensure everything is properly built
target_link_libraries(move_practice_node
  ${catkin_LIBRARIES}
)

# Ensure the target is linked with necessary MoveIt! and ROS libraries
add_dependencies(move_practice_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Pick and Place script
add_executable(pick_and_place src/pick_and_place.cpp)

target_link_libraries(pick_and_place
  ${catkin_LIBRARIES}
)

add_dependencies(pick_and_place ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

