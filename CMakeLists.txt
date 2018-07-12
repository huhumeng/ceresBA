cmake_minimum_required(VERSION 2.8)

project(CeresBA)

set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -g -ftest-coverage -fprofile-arcs -Wno-deprecated -pthread")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Ceres REQUIRED)
include_directories(${Ceres_INCLUDE_DIRS})

include_directories(
    /usr/include/eigen3
    ${CMAKE_SOURCE_DIR}/include
)

set(
    Third_LIBS
    ${OpenCV_LIBS}
    ${Ceres_LIBS}
)

add_subdirectory(src)
add_subdirectory(test)