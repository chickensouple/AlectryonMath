cmake_minimum_required(VERSION 2.8)
project(AlectyonMath)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(GTEST_ROOT ~/Documents/googletest/build/googlemock/gtest)

enable_testing()
find_package (Threads)
find_package(GTest REQUIRED)
set(GTEST_INCLUDE_DIRS ~/Documents/googletest/googletest/include/)
find_package(Eigen3 REQUIRED)
include_directories(
        libtransformation/include
        ${EIGEN3_INCLUDE_DIR}
        ${GTEST_INCLUDE_DIRS})

add_subdirectory(libangle)
add_subdirectory(libtransformation)

include_directories(libtransformation/include libangle/include)

add_executable(TransformationUnitTests tests/TransformationUnitTests.cpp)
target_link_libraries(TransformationUnitTests transformation ${GTEST_BOTH_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})


add_executable(AngleUnitTests tests/AngleUnitTests.cpp)
target_link_libraries(AngleUnitTests angle ${GTEST_BOTH_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

