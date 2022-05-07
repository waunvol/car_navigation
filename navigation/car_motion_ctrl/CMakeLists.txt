cmake_minimum_required(VERSION 3.0.2)
project(car_motion_ctrl)
SET(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES car_motion_ctrl
 CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy tf2 tf2_ros
#  DEPENDS system_lib
)

include_directories(
# include
  include/car_motionCtrl
  include/local_planner
  ${catkin_INCLUDE_DIRS}
)

add_executable(ctrlNode src/ctrl_node.cpp)
target_link_libraries(ctrlNode ${catkin_LIBRARIES})
