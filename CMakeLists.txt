cmake_minimum_required(VERSION 2.8)
project(AlectyonMath)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(GTEST_ROOT ~/Documents/googletest/build/googlemock/gtest)

enable_testing()
find_package (Threads)
find_package(GTest REQUIRED)
set(GTEST_INCLUDE_DIRS ~/Documents/googletest/googletest/include/)
find_package(Eigen3 REQUIRED)

add_subdirectory(libalectryonmath)


include_directories(
        libalectryonmath/include
        ${EIGEN3_INCLUDE_DIR}
        ${GTEST_INCLUDE_DIRS})

add_executable(TransformUnitTests tests/TransformUnitTests.cpp)
target_link_libraries(TransformUnitTests alectryonmath ${GTEST_BOTH_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

add_executable(AngleUnitTests tests/AngleUnitTests.cpp)
target_link_libraries(AngleUnitTests alectryonmath ${GTEST_BOTH_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

