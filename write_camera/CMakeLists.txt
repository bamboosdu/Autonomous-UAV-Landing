cmake_minimum_required(VERSION 2.8)
project(ModelWash)
set(CMAKE_BUILD_TYPE Release)#compile mode
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)#set path of executable file
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)#set outpath of produced lib
#set(CMAKE_VERBOSE_MAKEFILE TRUE)
 
 
#编译选项
set (CMAKE_CXX_FLAGS "-g -std=c++11 -Wformat=0 ${CMAKE_CXX_FLAGS} ") 
# 寻找OpenCV库
find_package(OpenCV REQUIRED)#打开opencv安装目录下的opencv.pc就明白为啥这样写了
# 添加头文件
include_directories(
${PROJECT_SOURCE_DIR}/include 
)
add_executable(ModelWash ./write_ci.cpp)#only one main.cpp 
# 链接OpenCV库
target_link_libraries( ModelWash ${OpenCV_LIBS})
