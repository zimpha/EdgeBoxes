cmake_minimum_required(VERSION 2.8)

project(EdgeBoxes)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic -Wno-long-long -std=c++0x -O2") ##-g -fno-omit-frame-pointer")

FIND_PACKAGE( OpenCV REQUIRED )
link_directories ( ${OpenCV_LIB_DIR} )
MESSAGE(STATUS "OpenCV_LIB_DIR: ${OpenCV_LIB_DIR}")

include_directories ( ${OpenCV_INCLUDE_DIRS} )
MESSAGE(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")

include_directories(${CMAKE_SOURCE_DIR}/src/)

add_subdirectory(src)

add_executable(image_demo demo/image_demo.cpp)
target_link_libraries(image_demo ${OpenCV_LIBS} grad image conv)

add_executable(detect_demo demo/detect_demo.cpp)
target_link_libraries(detect_demo ${OpenCV_LIBS} grad image conv edge)
