cmake_minimum_required(VERSION 2.8)
project(path_planning)

set(OpenCV_DIR ~/opencv_build/opencv/build/)

find_package(Eigen3 REQUIRED)

INCLUDE_DIRECTORIES ( "${EIGEN3_INCLUDE_DIR}" )
MESSAGE("Eigen Version: ${Eigen3_VERSION}")



find_package(OpenCV 4.5.1 REQUIRED)

MESSAGE("OpenCV Version: ${OpenCV_VERSION}")

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    )

# The following folder will be included
include_directories("${PROJECT_SOURCE_DIR}")

add_executable(path_planning prm.cpp rrt.cpp map.cpp dijkstra.cpp astar.cpp bfs.cpp dfs.cpp main.cpp)

target_link_libraries(path_planning ${OpenCV_LIBS})