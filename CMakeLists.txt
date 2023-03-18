cmake_minimum_required(VERSION 3.10)
project(planning_algorithms)

set(ASTAR
    common/map.cc
    a_star/a_star.cc
)

add_executable(a_star ${ASTAR})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
target_link_libraries(a_star ${OpenCV_LIBS})

target_include_directories(a_star
    PRIVATE 
        ${PROJECT_SOURCE_DIR}/common
)

add_executable(rrt rrt/rrt.cc)
target_link_libraries(rrt ${OpenCV_LIBS})