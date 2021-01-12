project(MR)

cmake_minimum_required(VERSION 2.8)

set(OpenCV_DIR ~/opencv_build/opencv/build/)

find_package(Eigen3 REQUIRED)

INCLUDE_DIRECTORIES ( "${EIGEN3_INCLUDE_DIR}" )
MESSAGE("Eigen Version: ${Eigen3_VERSION}")



find_package(OpenCV 4.5.1 REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    )
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
add_subdirectory(path_planning)
add_subdirectory(roadmaps)
add_subdirectory(graph_search)
