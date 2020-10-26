#cmake_version
cmake_minimum_required(VERSION 2.8)

#project name
PROJECT(marker)

#head file path
include_directories("/usr/include/eigen3")
include_directories("./include")

#set environment variable
set(CMAKE_BUILD_TYPE Release)#compile mode
set(URLAR_LIB ../include/Utils.cpp)
set (CMAKE_CXX_FLAGS "-g -std=c++11 -Wformat=0 -Wall  ${CMAKE_CXX_FLAGS} ") 

# 寻找OpenCV库
find_package(OpenCV 4.2 REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Test REQUIRED)
find_package(Qt5Concurrent REQUIRED)
find_package(Qt5OpenGL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(VTK REQUIRED)

add_executable(marker ./src/detect_multi_markers_2.cpp ${URLAR_LIB} )#only one


# 链接OpenCV库
target_link_libraries( marker ${OpenCV_LIBS} )#-lNX_C_Share
#usr/lib/libNX_C_Share.so
