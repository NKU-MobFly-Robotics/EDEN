cmake_minimum_required(VERSION 3.0.2)
project(map_visualization)

add_definitions(-std=c++14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

cs_add_library(${PROJECT_NAME} src/map_visualization.cpp)


cs_add_executable(map_pcl_node src/map_pcl_node.cpp)

target_link_libraries(map_pcl_node
    ${catkin_LIBRARIES} ${PROJECT_NAME})
##########
# EXPORT #
##########
cs_install()
cs_export()