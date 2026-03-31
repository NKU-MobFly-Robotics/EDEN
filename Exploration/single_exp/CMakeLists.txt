cmake_minimum_required(VERSION 2.8.3)
project(single_exp)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin_simple REQUIRED)
add_definitions(-std=c++14)

catkin_simple()


include_directories(
  ${catkin_INCLUDE_DIRS}
)
cs_add_library(${PROJECT_NAME}_lib src/single_exp.cpp src/single_exp_fsm.cpp)

cs_add_executable(${PROJECT_NAME}_node src/single_exp_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES} ${PROJECT_NAME}_lib)
cs_install() 
cs_export()