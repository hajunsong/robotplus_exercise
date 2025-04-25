cmake_minimum_required(VERSION 3.10)
project(GazeboOpenCVCamera)

# Gazebo 패키지 찾기
find_package(gazebo REQUIRED)
# OpenCV 패키지 찾기
find_package(OpenCV REQUIRED)

# Gazebo 관련 include, 라이브러리 디렉토리, 컴파일 플래그 추가
include_directories(${GAZEBO_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
add_definitions(${GAZEBO_CXX_FLAGS})

# 실행 파일 생성
add_executable(gazebo_opencv_camera main.cpp)

# Gazebo와 OpenCV 라이브러리 링크
target_link_libraries(gazebo_opencv_camera ${GAZEBO_LIBRARIES} ${OpenCV_LIBS})
