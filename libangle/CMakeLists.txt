cmake_minimum_required(VERSION 2.8)
project(AngleLib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(Eigen3 REQUIRED)
include_directories(
        include
        ${EIGEN3_INCLUDE_DIR})

set(SOURCE_FILES include/Angle.h src/Angle.cpp)
add_library(angle ${SOURCE_FILES})
