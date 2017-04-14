cmake_minimum_required(VERSION 2.8)
project(TransformationLib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(Eigen3 REQUIRED)
include_directories(
        include
        ${EIGEN3_INCLUDE_DIR})

include_directories(../include/libangle)

set(SOURCE_FILES include/RotationConversions.h src/RotationConversions.cpp)
add_library(transformation ${SOURCE_FILES})
