cmake_minimum_required(VERSION 2.8.3)
project(ato_test)

find_package(catkin_simple REQUIRED)
add_definitions(-std=c++14)

catkin_simple()

# cs_add_executable(fov_ray_test src/fov_ray_test.cpp)

# target_link_libraries(fov_ray_test
#     ${catkin_LIBRARIES})


# cs_add_executable(swarm_data_test src/swarm_data_test.cpp)

# target_link_libraries(swarm_data_test
#     ${catkin_LIBRARIES})

# cs_add_executable(traj_swarm src/traj_swarm.cpp)

# target_link_libraries(traj_swarm
#         ${catkin_LIBRARIES})

# cs_add_executable(cmd_test src/cmd_test.cpp)

# target_link_libraries(cmd_test
#         ${catkin_LIBRARIES})

# cs_add_executable(cmd_repub src/cmd_repub.cpp)

# target_link_libraries(cmd_repub
#         ${catkin_LIBRARIES})

# cs_add_executable(GMM_test src/GMM_test.cpp)

# target_link_libraries(cmd_repub
#         ${catkin_LIBRARIES})
# include_directories(/usr/local/include/scip)
# link_directories(/usr/local/lib)
# target_link_libraries(map_test_node
#         ${catkin_LIBRARIES} scip)
# cs_add_executable(traj_branch_test src/traj_branch_test.cpp)

# target_link_libraries(traj_branch_test
#         ${catkin_LIBRARIES})

# cs_add_executable(map_test_node src/map_test_node.cpp)

# target_link_libraries(map_test_node
#         ${catkin_LIBRARIES})

# cs_add_executable(lrm_test_node src/lrm_test_node.cpp)

# target_link_libraries(lrm_test_node
#         ${catkin_LIBRARIES})

# cs_add_executable(eroi_test_node src/eroi_test_node.cpp)

# target_link_libraries(eroi_test_node
#         ${catkin_LIBRARIES})

cs_add_executable(dtgplus_test_node src/dtgplus_test_node.cpp)

target_link_libraries(dtgplus_test_node
        ${catkin_LIBRARIES})

cs_add_executable(traj_debug src/traj_debug.cpp)

target_link_libraries(traj_debug
        ${catkin_LIBRARIES})        

cs_add_executable(debug_puber src/debug_puber.cpp)

target_link_libraries(debug_puber
        ${catkin_LIBRARIES})        

cs_add_executable(traj_vel_vis src/traj_vel_vis.cpp)

target_link_libraries(traj_vel_vis
        ${catkin_LIBRARIES})        

# cs_add_executable(route_test_node src/route_test_node.cpp)

# target_link_libraries(route_test_node
#         ${catkin_LIBRARIES})
cs_install()
cs_export()