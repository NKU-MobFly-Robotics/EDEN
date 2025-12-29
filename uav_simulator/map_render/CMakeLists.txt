cmake_minimum_required(VERSION 2.8.3)
project(map_render)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  cv_bridge
  pointcloud_render
  mesh_render
)

catkin_package(
  LIBRARIES map_render
  CATKIN_DEPENDS roscpp std_msgs pointcloud_render mesh_render
  # DEPENDS system_lib
)

include_directories(
  SYSTEM
  include
  ${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_executable(map_render_node
  src/map_render_node.cpp 
)
target_link_libraries(map_render_node 
  ${catkin_LIBRARIES}
  -ldw
  -lglog
)

add_executable (pointcloud_modifier 
  src/pointcloud_modifier.cpp 
)
target_link_libraries(pointcloud_modifier
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable (click_map 
  src/click_map.cpp
)
target_link_libraries(click_map
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}) 