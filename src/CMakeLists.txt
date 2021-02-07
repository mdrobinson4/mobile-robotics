# Notice name prefix of this variable, set by CMake according
# to value given with "project()" in the root CMakeLists.txt.

set(OpenCV_DIR ~/opencv_build/opencv/build/)
find_package(OpenCV 4.5.1 REQUIRED)
find_package(Eigen3 REQUIRED)


include_directories(
    ${MR_SOURCE_DIR}/graph_search/include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

include_directories(${MR_SOURCE_DIR}/graph_search/include)
add_library(graph_search dfs.cpp bfs.cpp dijkstra.cpp astar.cpp)

target_link_libraries(graph_search ${OpenCV_LIBS})
