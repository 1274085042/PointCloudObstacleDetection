cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_definitions(-std=c++17)

set(CMAKE_BUILD_TYPE "Debug")
set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")


project(point_cloud_detection)

#find_package(PCL 1.7.2 EXACT)
find_package(PCL 1.2  REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

add_executable (point_cloud_detection src/point_cloud_detection.cpp src/render/render.cpp src/processPointClouds.cpp)
target_link_libraries (point_cloud_detection ${PCL_LIBRARIES})
