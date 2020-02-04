cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(playback)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


add_library(
    udacity
        lib/udacity/euclidean_clustering.cpp
        lib/udacity/euclidean_clustering.h
        lib/udacity/kdtree.cpp
        lib/udacity/kdtree.h
        lib/udacity/ransac.h
        lib/udacity/ransac.cpp
)

target_link_libraries(udacity ${PCL_INCLUDE_DIRS})

add_executable (environment src/environment.cpp src/render/render.cpp src/processPointClouds.cpp)
target_link_libraries (environment udacity ${PCL_LIBRARIES})




