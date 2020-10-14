
cmake_minimum_required(VERSION 2.8)

project(marker)

set(CMAKE_BUILD_TYPE Release)#compile mode

set(PROJECT_SOURCE_DIR ${/home/qiang/zq/git-space/drone_about/aruco_board/detect_board})
 
set(URLAR_LIB
    /home/qiang/zq/git-space/drone_about/aruco_board/detect_board/include/Utils.cpp
)
 
#编译选项
set (CMAKE_CXX_FLAGS "-g -std=c++11 -Wformat=0 -Wall ${CMAKE_CXX_FLAGS} ") 

# 寻找OpenCV库
find_package(OpenCV 4.2 REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Test REQUIRED)
find_package(Qt5Concurrent REQUIRED)
find_package(Qt5OpenGL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(VTK REQUIRED)
# 添加头文件
include_directories("/usr/include/eigen3")
include_directories(/home/qiang/zq/git-space/drone_about/aruco_board/detect_board/include)

add_executable(marker ../detect_multi_markers.cpp ${URLAR_LIB} )#only one main.cpp ${URLAR_LIB}

# 链接OpenCV库
target_link_libraries( marker ${OpenCV_LIBS})
