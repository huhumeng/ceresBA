cmake_minimum_required(VERSION 2.8)

project(CeresBA)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

find_package(Ceres REQUIRED)
include_directories(${Ceres_INCLUDE_DIRS})

include_directories(
    /usr/include/eigen3
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/include/Util
    ${CMAKE_SOURCE_DIR}/include/CeresCost
    ${CMAKE_SOURCE_DIR}/include/Problem
)

set(
    Third_LIBS
    ${OpenCV_LIBS}
    ${CERES_LIBRARIES}
    ${GLOG_LIBRARIES}
)


add_subdirectory(src)
add_subdirectory(test)