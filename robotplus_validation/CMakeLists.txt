cmake_minimum_required(VERSION 3.0.2)
project(robotplus_validation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
)

catkin_package()

include_directories(
	# include
	${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})