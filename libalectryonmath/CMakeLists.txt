cmake_minimum_required(VERSION 2.8)
project(AlectryonMathLib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(GTEST_ROOT ~/Documents/googletest/build/googlemock/gtest)
set(GTEST_INCLUDE_DIR ~/Documents/googletest/googletest/include/)

enable_testing()
find_package (Threads)
find_package(GTest REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(
        .
        include
        ${EIGEN3_INCLUDE_DIR}
        ${GTEST_INCLUDE_DIR})

#set(SOURCE_FILES src/alectryon_math.cpp)
#add_library(alectryonmath ${SOURCE_FILES})


add_executable(RotationConversionsTests tests/RotationConversionsUnitTests.cpp)
target_link_libraries(RotationConversionsTests ${GTEST_BOTH_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

add_executable(RotationConversionsIntensiveTest tests/RotationConversionsIntensiveTest.cpp)
target_link_libraries(RotationConversionsIntensiveTest ${GTEST_BOTH_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

add_executable(AngleUnitTests tests/AngleUnitTests.cpp tests/TestHelpers.h)
target_link_libraries(AngleUnitTests ${GTEST_BOTH_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

add_executable(RotationOperationsTests tests/RotationOperationsUnitTests.cpp tests/TestHelpers.h)
target_link_libraries(RotationOperationsTests ${GTEST_BOTH_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

add_executable(AlectryonMathTests tests/AlectryonMathUnitTests.cpp tests/TestHelpers.h)
target_link_libraries(AlectryonMathTests ${GTEST_BOTH_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
