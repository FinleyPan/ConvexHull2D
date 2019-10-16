cmake_minimum_required(VERSION 2.8)

project(convex_hull_2D)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++11 -O0")

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} test.cc ConvexHull.h)
target_link_libraries(${PROJECT_NAME} opencv_core opencv_highgui opencv_imgproc)

