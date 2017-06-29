cmake_minimum_required(VERSION 2.8)
project(AlectyonMath)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(GTEST_ROOT ~/Documents/googletest/build/googlemock/gtest)
set(GTEST_INCLUDE_DIR ~/Documents/googletest/googletest/include/)

enable_testing()
find_package (Threads)
find_package(GTest REQUIRED)
find_package(Eigen3 REQUIRED)

add_subdirectory(libalectryonmath)


include_directories(
        libalectryonmath/include
        ${EIGEN3_INCLUDE_DIR}
        ${GTEST_INCLUDE_DIR})

add_executable(TransformUnitTests tests/TransformUnitTests.cpp)
target_link_libraries(TransformUnitTests alectryonmath ${GTEST_BOTH_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

add_executable(AngleUnitTests tests/AngleUnitTests.cpp tests/TestHelpers.h)
target_link_libraries(AngleUnitTests alectryonmath ${GTEST_BOTH_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

add_executable(QuaternionTests tests/QuaternionUnitTests.cpp tests/TestHelpers.h)
target_link_libraries(QuaternionTests alectryonmath ${GTEST_BOTH_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})