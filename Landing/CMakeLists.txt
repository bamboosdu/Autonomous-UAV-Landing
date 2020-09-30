cmake_minimum_required(VERSION 2.8)
project(landing)
set(CMAKE_BUILD_TYPE Release)#compile mode

 
#编译选项
set (CMAKE_CXX_FLAGS "-g -std=c++11 -Wformat=0   ${CMAKE_CXX_FLAGS}  ") 
# 寻找OpenCV库
find_package(OpenCV REQUIRED)#打开opencv安装目录下的opencv.pc就明白为啥这样写了
find_package(yaml-cpp)


set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost  COMPONENTS filesystem regex)
# 添加头文件

if(Boost_FOUND)
	message("-- Boost found")
	include_directories(
    		"/usr/include/eigen3"
   		"./include"
    		${Boost_INUCLUDE_DIRS}
    	)
	add_executable(landing ./src/landing.cpp  )#only one main.cpp 
	# 链接OpenCV库
	target_link_libraries( landing ${OpenCV_LIBS} ${Boost_LIBRARIES})
endif()
