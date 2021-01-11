cmake_minimum_required(VERSION 2.8)
project(prm)

set(OpenCV_DIR ~/opencv_build/opencv/build/)

find_package(Eigen3 REQUIRED)

INCLUDE_DIRECTORIES ( "${EIGEN3_INCLUDE_DIR}" )
MESSAGE("Eigen Version: ${Eigen3_VERSION}")



find_package(OpenCV 4.5.1 REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    )

add_executable(prm prm.cpp map.cpp dijkstra.cpp main.cpp)
target_link_libraries(prm ${OpenCV_LIBS})