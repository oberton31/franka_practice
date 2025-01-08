cmake_minimum_required(VERSION 3.0.2)
project(fci_practice)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  actionlib
  franka_gripper
)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs actionlib franka_gripper
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(move_franka src/move_franka.cpp)
target_link_libraries(move_franka ${catkin_LIBRARIES})
