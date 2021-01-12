include_directories(${PROJECT_SOURCE_DIR}/path_planning/include)
include_directories(${MR_SOURCE_DIR}/graph_search/include)
include_directories(${MR_SOURCE_DIR}/roadmaps/include)

link_directories(${MR_SOURCE_DIR}/graph_search/src)
link_directories(${MR_SOURCE_DIR}/roadmaps/src)

add_executable(mr main.cpp)
target_link_libraries(mr roadmaps graph_search)

find_program(GENERATOR mr)
message(${GENERATOR})