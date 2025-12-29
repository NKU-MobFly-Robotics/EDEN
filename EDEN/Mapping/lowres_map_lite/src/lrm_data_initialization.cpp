#include <lowres_map_lite/lowres_map_lite.h>
using namespace std;
using namespace lowres_lite;


void LowResMap::AlignInit(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private,
    Eigen::Vector3d &origin, Eigen::Vector3i &block_size, 
    Eigen::Vector3i &block_num, Eigen::Vector3i &local_block_num){
        nh_ = nh;
        nh_private_ = nh_private;
        std::string ns = ros::this_node::getName();
        // Eigen::Vector3d localgraph_scale;
        // nh_private_.param(ns + "/LowResMap/localgraph_sizex", 
        //     localgraph_scale.x(), 14.0);
        // nh_private_.param(ns + "/LowResMap/localgraph_sizey", 
        //     localgraph_scale.y(), 14.0);
        // nh_private_.param(ns + "/LowResMap/localgraph_sizez", 
        //     localgraph_scale.z(), 8.0);
        nh_private_.param(ns + "/LowResMap/node_x", 
            node_scale_.x(), 0.5);
        nh_private_.param(ns + "/LowResMap/node_y", 
            node_scale_.y(), 0.5);
        nh_private_.param(ns + "/LowResMap/node_z", 
            node_scale_.z(), 0.3);
        nh_private_.param(ns + "/Exp/robot_sizeX", 
            Robot_size_.x(), 0.5);
        nh_private_.param(ns + "/Exp/robot_sizeY", 
            Robot_size_.y(), 0.5);
        nh_private_.param(ns + "/Exp/robot_sizeZ", 
            Robot_size_.z(), 0.3);

        nh_private_.param(ns + "/LowResMap/corridor_expX", 
            corridor_exp_r_.x(), 1);
        nh_private_.param(ns + "/LowResMap/corridor_expY", 
            corridor_exp_r_.y(), 1);
        nh_private_.param(ns + "/LowResMap/corridor_expZ", 
            corridor_exp_r_.z(), 1);
        nh_private_.param(ns + "/block_map/maxX", 
            mapscale_.x(), 10.0);
        nh_private_.param(ns + "/block_map/maxY", 
            mapscale_.y(), 10.0);
        nh_private_.param(ns + "/block_map/maxZ", 
            mapscale_.z(), 0.0);
        nh_private_.param(ns + "/block_map/minX", 
            origin_.x(), -10.0);
        nh_private_.param(ns + "/block_map/minY", 
            origin_.y(), -10.0);
        nh_private_.param(ns + "/block_map/minZ", 
            origin_.z(), 0.0);
        // nh_private_.param(ns + "/LowResMap/blockX", 
        //     block_size_.x(), 5);
        // nh_private_.param(ns + "/LowResMap/blockY", 
        //     block_size_.y(), 5);
        // nh_private_.param(ns + "/LowResMap/blockZ", 
        //     block_size_.z(), 3);
        nh_private_.param(ns + "/block_map/resolution", 
            resolution_, 0.2);
        nh_private_.param(ns + "/LowResMap/showmap", 
            showmap_, false);
        nh_private_.param(ns + "/LowResMap/debug", 
            debug_, false);
        nh_private_.param(ns + "/LowResMap/seg_length", 
            seg_length_, 3.0);
        nh_private_.param(ns + "/LowResMap/prune_seg_length", 
            prune_seg_length_, 3.0);

    if(showmap_){
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(ns + "/LowResMap/Nodes", 10);
        show_timer_ = nh_private_.createTimer(ros::Duration(0.15), &LowResMap::ShowGridLocal, this);
    }
    node_size_(0) = ceil(node_scale_(0) / resolution_) + 1;
    node_size_(1) = ceil(node_scale_(1) / resolution_) + 1;
    node_size_(2) = ceil(node_scale_(2) / resolution_) + 1;
    node_scale_ = node_size_.cast<double>() * resolution_;
    expand_r_ = Robot_size_*4.0;
    nv_num_ = node_size_(0) * node_size_(1) * node_size_(2);
    Eigen::Vector3i BM_Block_v_size;
    Eigen::Vector3d BM_Block_scale;
    nh_private_.param(ns + "/block_map/blockX", 
        BM_Block_v_size.x(), 50);
    nh_private_.param(ns + "/block_map/blockY", 
        BM_Block_v_size.y(), 50);
    nh_private_.param(ns + "/block_map/blockZ", 
        BM_Block_v_size.z(), 50);
    nh_private_.param(ns + "/block_map/LocalBlockNumX", 
        local_block_num_.x(), 7);
    nh_private_.param(ns + "/block_map/LocalBlockNumY", 
        local_block_num_.y(), 7);
    nh_private_.param(ns + "/block_map/LocalBlockNumZ", 
        local_block_num_.z(), 3);

    for(int dim = 0; dim < 3; dim++){
        BM_Block_scale(dim) = BM_Block_v_size(dim) * resolution_;
        block_size_(dim) = ceil(BM_Block_scale(dim) / node_scale_(dim));
        blockscale_(dim) = block_size_(dim) * node_scale_(dim);
        block_num_(dim) = ceil((mapscale_(dim) - origin_(dim))/blockscale_(dim));
        mapscale_(dim) = ceil((mapscale_(dim) - origin_(dim))/blockscale_(dim)) * blockscale_(dim);
        voxel_num_(dim) = block_size_(dim) * block_num_(dim);
        map_upbd_(dim) = origin_(dim) + mapscale_(dim) - 1e-4;
        map_lowbd_(dim) = origin_(dim) + 1e-4;
        local_origin_(dim) = map_lowbd_(dim);
        local_block_num_(dim) = min(local_block_num_(dim), block_num_(dim));
        local_mapscale_(dim) = local_block_num_(dim) * blockscale_(dim);
        local_map_upbd_(dim) = origin_(dim) + local_mapscale_(dim) - 1e-3;
        local_map_lowbd_(dim) = origin_(dim) + 1e-3;
        local_origin_idx_(dim) = 0;
        local_up_idx_(dim) = local_block_num_(dim) - 1;
        local_origin_node_idx_(dim) = 0;
        local_up_node_idx_(dim) = block_size_(dim) * local_block_num_(dim) - 1;
        local_node_num_(dim) = block_size_(dim) * local_block_num_(dim);

        if(local_block_num_(dim) > block_num_(dim)){
            ROS_ERROR("error lcoal num, too large");
        }
    }
    ROS_WARN("lrm");
    cout<<"node_size_:"<<node_size_.transpose()<<endl;
    cout<<"blockscale_:"<<blockscale_.transpose()<<endl;
    cout<<"block_num_:"<<block_num_.transpose()<<endl;
    cout<<"block_size_:"<<block_size_.transpose()<<endl;
    cout<<"local_block_num_:"<<local_block_num_.transpose()<<endl;
    cout<<"map_upbd_:"<<map_upbd_.transpose()<<endl;
    cout<<"mapscale_:"<<mapscale_.transpose()<<endl;
    cout<<"map_lowbd_:"<<map_lowbd_.transpose()<<endl;
    for(int z = 0; z < local_block_num_(2); z++){
        for(int y = 0; y < local_block_num_(1); y++){
            for(int x = 0; x < local_block_num_(0); x++){
                LG_.emplace_back(make_shared<LR_block>());
                LG_.back()->origin_ = Eigen::Vector3i(x, y, z).cwiseProduct(block_size_);
                LG_.back()->local_grid_.resize(block_size_(0) * block_size_(1) * block_size_(2));
            }
        }
    }

    v_n_(0) = voxel_num_(0);
    v_n_(1) = voxel_num_(0) * voxel_num_(1);
    v_n_(2) = voxel_num_(0) * voxel_num_(1) * voxel_num_(2);
    b_n_(0) = block_num_(0);
    b_n_(1) = block_num_(0) * block_num_(1);
    b_n_(2) = block_num_(0) * block_num_(1) * block_num_(2);
    l_b_n_(0) = local_block_num_(0);
    l_b_n_(1) = local_block_num_(0) * local_block_num_(1);
    l_b_n_(2) = local_block_num_(0) * local_block_num_(1)  * local_block_num_(2);
    InitializeQuickCompute();

    origin = origin_;
    block_size = block_size_.cwiseProduct(node_size_);
    block_num = block_num_;
    local_block_num = local_block_num_;
    Outnode_ = make_shared<LR_node>();
}

void LowResMap::InitializeQuickCompute(){
    Eigen::Vector3i pi, pi2;
    int i = 0, c, dc;
    for(pi(2) = -1; pi(2) < 2; pi(2)++){
        for(pi(1) = -1; pi(1) < 2; pi(1)++){
            for(pi(0) = -1; pi(0) < 2; pi(0)++){

                if(!pi.norm() == 0) {
                    connect_diff_.emplace_back(pi);
                    occdiff_2_ch_dir_.emplace_back(4294967295 - (1<<i));
                }
                else{
                    connect_diff_.emplace_back(pi);
                    occdiff_2_ch_dir_.emplace_back(4294967295);
                }
                i++;
                int j = 0;
                for(pi2(2) = -1; pi2(2) < 2; pi2(2)++){
                    for(pi2(1) = -1; pi2(1) < 2; pi2(1)++){
                        for(pi2(0) = -1; pi2(0) < 2; pi2(0)++){
                            c = abs(pi2(0)) + abs(pi2(1)) + abs(pi2(2));
                            dc = abs(pi(0) - pi2(0)) + abs(pi(1) - pi2(1)) + abs(pi(2) - pi2(2));

                            if(c == 0){

                            }
                            else if(c == 1){
                                
                            }
                            else if(c == 2){
                                if(dc == 1 && abs(pi(0)) + abs(pi(1)) + abs(pi(2)) == 1){
                                    occdiff_2_ch_dir_.back() &= (4294967295 - (1<<j));
                                }
                            }
                            else if(c == 3){
                                if(abs(pi(0) - pi2(0)) <= 1 && abs(pi(1) - pi2(1)) <= 1 && abs(pi(2) - pi2(2)) <= 1){
                                    occdiff_2_ch_dir_.back() &= (4294967295 - (1<<j));
                                }
                            }

                            j++;
                            // c = abs(pi2(0) + pi2(1) + pi2(2));
                            // if(c == 0){ // root node 
                            //     connection_check_.emplace_back(true);
                            // }
                            // else if(c == 1){
                            //     if((pi + pi2).norm() != 0) connection_check_.emplace_back(true);
                            //     else connection_check_.emplace_back(false);
                            // }
                            // else if(c == 2){
                            //     if(pi.norm() == 1 && (pi+pi2).norm() == 1) connection_check_.emplace_back(false);
                            //     else connection_check_.emplace_back(true);
                            // }
                            // else if(c == 3){
                            //     if(abs((pi+pi2)(0)) == 2 || abs((pi+pi2)(1)) == 2 || abs((pi+pi2)(2)) == 2) connection_check_.emplace_back(true);
                            //     else connection_check_.emplace_back(false);

                            // }
                        }
                    }
                }

            }
        }
    }
    // vector<pair<Eigen::Vector3i, uint8_t>> occlude_check_list_; // [iter], size = 18, <diff, id0 = 0-17>

}
