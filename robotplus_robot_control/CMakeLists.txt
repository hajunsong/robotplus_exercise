cmake_minimum_required(VERSION 3.0.2)
project(robotplus_robot_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	moveit_core
	moveit_ros_planning_interface
	geometry_msgs
	tf2_geometry_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
	# include
	${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})