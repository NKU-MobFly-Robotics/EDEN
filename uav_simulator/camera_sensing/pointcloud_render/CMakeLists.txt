cmake_minimum_required(VERSION 2.8.12)
project(pointcloud_render)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

# set(ENABLE_CUDA true)

# if(ENABLE_CUDA)
# GPU accelerated

find_package(CUDA REQUIRED)
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-O3 -use_fast_math)
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS}" "--ptxas-options=-v")
set(CUDA_NVCC_FLAGS -gencode arch=compute_86,code=sm_86;) # Titan Xp
# set(CUDA_NVCC_FLAGS -gencode arch=compute_90,code=sm_90;) # 4090, sm_86 or sm_89 (CUDA 11.8+)
set(CUDA_PROPAGATE_HOST_FLAGS OFF)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)

find_package(
  catkin REQUIRED COMPONENTS 
  roscpp
  roslib
  cmake_modules
  cv_bridge
  image_transport
  pcl_ros
  sensor_msgs
  geometry_msgs
  nav_msgs
  quadrotor_msgs
  dynamic_reconfigure
)

catkin_package(
  INCLUDE_DIRS include ${CUDA_INCLUDE_DIRS}
  DEPENDS OpenCV Eigen Boost
  LIBRARIES depth_render_cuda pointcloud_render
  CATKIN_DEPENDS roscpp roslib image_transport pcl_ros
)

include_directories(
  SYSTEM
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

cuda_add_library(depth_render_cuda src/depth_render.cu)

link_directories(${PCL_LIBRARY_DIRS})

add_library(pointcloud_render 
  src/pointcloud_render.cpp
)
target_link_libraries(pointcloud_render
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(pcl_render_node 
  # src/pcl_render_node.cpp
  src/pointcloud_render.cpp
  src/pointcloud_render_node.cpp
)
target_link_libraries(pcl_render_node 
  depth_render_cuda 
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)
# else(ENABLE_CUDA)
#   # CPU only
#   find_package(Eigen3 REQUIRED)
#   find_package(OpenCV REQUIRED)
#   find_package(
#     catkin REQUIRED
#     COMPONENTS roscpp
#                roslib
#                cv_bridge
#                cmake_modules
#                pcl_ros
#                sensor_msgs
#                geometry_msgs
#                nav_msgs
#                quadrotor_msgs
#   )

#   catkin_package(
#     DEPENDS
#     Eigen
#     OpenCV
#     CATKIN_DEPENDS
#     roscpp
#     roslib
#     pcl_ros
#   )

#   include_directories(
#     SYSTEM 
#     include 
#     ${catkin_INCLUDE_DIRS}          
#     ${Eigen_INCLUDE_DIRS} 
#     ${OpenCV_INCLUDE_DIRS}
#   )

#   add_executable(pcl_render_node 
#     src/depth_render_node.cpp
#   )
#   target_link_libraries(pcl_render_node 
#     ${catkin_LIBRARIES} 
#     ${PCL_LIBRARIES}
#     ${OpenCV_LIBS}
#   )
# endif(ENABLE_CUDA)
