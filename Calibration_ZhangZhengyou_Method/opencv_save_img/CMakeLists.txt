cmake_minimum_required(VERSION 2.8)
project(saveimg)
set(CMAKE_BUILD_TYPE Debug)#compile mode
#编译选项
set (CMAKE_CXX_FLAGS "-g -std=c++11 -Wformat=0 ${CMAKE_CXX_FLAGS} ") 
# 寻找OpenCV库
find_package(OpenCV REQUIRED)#打开opencv安装目录下的opencv.pc就明白为啥这样写了

add_executable(SaveImg ./opencv_save_img.cpp ${URLAR_LIB} )#only one main.cpp 
target_link_libraries( SaveImg ${OpenCV_LIBS})