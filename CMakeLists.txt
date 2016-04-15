cmake_minimum_required(VERSION 2.8)

project(EdgeBoxes)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic -Wno-long-long -std=gnu++11 -O2 -fopenmp -DUSEOMP -lpthread -msse2") ##-g -fno-omit-frame-pointer")

FIND_PACKAGE( OpenCV REQUIRED )
link_directories ( ${OpenCV_LIB_DIR} )
MESSAGE(STATUS "OpenCV_LIB_DIR: ${OpenCV_LIB_DIR}")

include_directories ( ${OpenCV_INCLUDE_DIRS} )
MESSAGE(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")

include_directories(${CMAKE_SOURCE_DIR}/src/)

add_subdirectory(src)

add_executable(detect_demo demo/detect_demo.cpp)
target_link_libraries(detect_demo ${OpenCV_LIBS} grad image conv channel acf)

add_executable(edge_demo demo/edge_demo.cpp)
target_link_libraries(edge_demo ${OpenCV_LIBS} grad image conv edge)
