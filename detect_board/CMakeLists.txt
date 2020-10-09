cmake_minimum_required(VERSION 2.8)
project(marker)
set(CMAKE_BUILD_TYPE Debug)#compile mode
set(PROJECT_SOURCE_DIR ${/home/zq/zq/git-space/drone_about/aruco_board/detect_board})

#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)#set path of executable file
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)#set outpath of produced lib
#set(CMAKE_VERBOSE_MAKEFILE TRUE)
 
set(URLAR_LIB
    /home/zq/zq/git-space/drone_about/aruco_board/detect_board/include/Utils.cpp
)
 
#编译选项
set (CMAKE_CXX_FLAGS "-g -std=c++11 -Wformat=0 ${CMAKE_CXX_FLAGS} ") 
# 寻找OpenCV库
find_package(OpenCV REQUIRED)#打开opencv安装目录下的opencv.pc就明白为啥这样写了
# 添加头文件
include_directories("/usr/include/eigen3")
include_directories(/home/zq/zq/git-space/drone_about/aruco_board/detect_board/include)
add_executable(marker ../detect_aruco_marker.cpp ${URLAR_LIB} )#only one main.cpp 
# 链接OpenCV库
target_link_libraries( marker ${OpenCV_LIBS})
