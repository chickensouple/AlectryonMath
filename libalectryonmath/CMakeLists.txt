cmake_minimum_required(VERSION 2.8)
project(AlectryonMathLib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(Eigen3 REQUIRED)
include_directories(
        include
        ${EIGEN3_INCLUDE_DIR})

set(SOURCE_FILES include/alectryonmath/Angle.h src/Angle.cpp include/alectryonmath/RotationConversions.h src/RotationConversions.cpp include/alectryonmath/AlectryonMath.h)
add_library(alectryonmath ${SOURCE_FILES})
