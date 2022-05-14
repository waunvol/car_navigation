cmake_minimum_required(VERSION 2.8.3)

project(car)

find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
tf
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(map_br map/src/map_tf.cpp)
target_link_libraries(map_br ${catkin_LIBRARIES})

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
