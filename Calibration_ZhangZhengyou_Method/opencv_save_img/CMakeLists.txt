cmake_minimum_required(VERSION 2.8)
project(saveimg)
set(CMAKE_BUILD_TYPE Debug)#compile mode
#编译选项
set (CMAKE_CXX_FLAGS "-g -std=c++11 -Wformat=0 ${CMAKE_CXX_FLAGS} ") 
# 寻找OpenCV库
find_package(OpenCV 4.2 REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Test REQUIRED)
find_package(Qt5Concurrent REQUIRED)
find_package(Qt5OpenGL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(VTK REQUIRED)

add_executable(SaveImg ./src/opencv_save_img.cpp ${URLAR_LIB} )#only one main.cpp 
target_link_libraries( SaveImg ${OpenCV_LIBS})
