cmake_minimum_required(VERSION 3.0.2)
project(robot_exploration)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE Debug)

set(OpenCV_DIR ~/opencv4.5/opencv/build/)
find_package(OpenCV 4.5 REQUIRED)

find_package(Eigen3 REQUIRED)

MESSAGE("Include: ${PROJECT_SOURCE_DIR}")

set(${PROJECT_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
)

include_directories(
    include
    ${PROJECT_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}
    src/astar.cpp
    src/bfs.cpp 
    src/dfs.cpp 
    src/dijkstra.cpp 
    src/map.cpp 
    src/prm.cpp 
    src/rrt.cpp
    src/main.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
)
