project(mesh_render)
cmake_minimum_required(VERSION 3.0.2)

set(CMAKE_BUILD_TYPE "Release") # Release, RelWithDebInfo
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Open3D REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  cv_bridge
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mesh_render
)

include_directories("/usr/include/eigen3")
include_directories(
  SYSTEM 
  include 
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS} 
  ${OpenCV_INCLUDE_DIRS}
)

add_library(mesh_render 
  src/mesh_render.cpp
)
target_link_libraries(mesh_render 
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES}    
  ${OpenCV_LIBS}
  # http://www.open3d.org/docs/release/cpp_project.html
  # https://github.com/isl-org/open3d-cmake-find-package 
  Open3D::Open3D
)

add_executable(mesh_render_node 
  src/mesh_render_node.cpp
  src/mesh_render.cpp
)
target_link_libraries(mesh_render_node 
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES}    
  ${OpenCV_LIBS}
  # http://www.open3d.org/docs/release/cpp_project.html
  # https://github.com/isl-org/open3d-cmake-find-package 
  Open3D::Open3D
)