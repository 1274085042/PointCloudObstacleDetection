cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++17)

set(CMAKE_BUILD_TYPE "Debug")
set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(traditionalpointcloud)

#find_package(PCL 1.2 REQUIRED)
find_package(PCL 1.2 REQUIRED)
find_package(pybind11 REQUIRED)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

#add_subdirectory(/home/lfq/pybind11/ pybind_binary_dir)
#add_library(traditionalpointcloud MODULE src/traditionalpointcloud.cpp src/render/render.cpp src/processPointClouds.cpp)

pybind11_add_module(traditionalpointcloud src/traditionalpointcloud.cpp src/render/render.cpp src/processPointClouds.cpp )
target_link_libraries (traditionalpointcloud  PUBLIC pybind11::module ${PCL_LIBRARIES})