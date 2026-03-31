#include <RipNeon/ripneon_struct.h>
#include <RipNeon/ripneon.h>


void RipNeon::Init(ros::NodeHandle &nh, ros::NodeHandle &nh_private){
    nh_ = nh;
    nh_private_ = nh_private;
    std::string ns = ros::this_node::getName();
    
	Eigen::Vector3f robots_scale;

	/**** map parameters ****/
    nh_private_.param(ns + "/block_map/minX", 
        origin_.x(), -10.0f);
    nh_private_.param(ns + "/block_map/minY", 
        origin_.y(), -10.0f);
    nh_private_.param(ns + "/block_map/minZ", 
        origin_.z(), 0.0f);
    nh_private_.param(ns + "/block_map/maxX", 
        map_upbd_.x(), 10.0f);
    nh_private_.param(ns + "/block_map/maxY", 
        map_upbd_.y(), 10.0f);
    nh_private_.param(ns + "/block_map/maxZ", 
        map_upbd_.z(), 0.0f);
    nh_private_.param(ns + "/block_map/blockX", 
        block_size_.x(), 5);
    nh_private_.param(ns + "/block_map/blockY", 
        block_size_.y(), 5);
    nh_private_.param(ns + "/block_map/blockZ", 
        block_size_.z(), 3);
    nh_private_.param(ns + "/block_map/LocalBlockNumX", 
        local_block_num_.x(), 7);
    nh_private_.param(ns + "/block_map/LocalBlockNumY", 
        local_block_num_.y(), 7);
    nh_private_.param(ns + "/block_map/LocalBlockNumZ", 
        local_block_num_.z(), 3);
    nh_private_.param(ns + "/block_map/resolution", 
        resolution_, 0.2f);
    nh_private_.param(ns + "/block_map/sensor_max_range", 
        sensor_range_, 20.5f);
    nh_private_.param(ns + "/block_map/rot_x", 
        sen_odom_rot_.x(), 0.0f);
    nh_private_.param(ns + "/block_map/rot_y", 
        sen_odom_rot_.y(), 0.0f);
    nh_private_.param(ns + "/block_map/rot_z", 
        sen_odom_rot_.z(), 0.0f);
    nh_private_.param(ns + "/block_map/rot_w", 
        sen_odom_rot_.w(), 1.0f);
    nh_private_.param(ns + "/block_map/world_frame_pts", 
        world_frame_pts_, false);
    nh_private_.param(ns + "/block_map/occ_max", 
        thr_max_, 0.9f);
    nh_private_.param(ns + "/block_map/occ_min", 
        thr_min_, 0.1f);
    nh_private_.param(ns + "/block_map/pro_miss_free", 
        pro_miss_, 0.8f);    
    nh_private_.param(ns + "/block_map/inflate_scale", 
        inflate_r_scale_, 0.5f);
    nh_private_.param(ns + "/block_map/void_frame_thresh", 
        void_frame_thresh_, 25);
    nh_private_.param(ns + "/block_map/void_pts_thresh", 
        void_pts_thresh_, 4);
    nh_private_.param(ns + "/block_map/sensor_void_range", 
        sensor_range_void_, 10.0f);

    /* create read write path */
    nh_private.param(ns + "/block_map/Path", 
        map_path_, map_path_);
    int tn;
    nh_private.param(ns + "/block_map/thread_num", 
        tn, 5);
    thread_num_ = tn;
    string command;
    time_t now = time(0);
    tm* t=localtime(&now);
    map_path_ = map_path_+"/"+to_string(t->tm_year+1900)+"_"+to_string(t->tm_mon+1)+"_"+to_string(t->tm_mday)
    +"_"+to_string(t->tm_hour)+"_"+to_string(t->tm_min)+"_"+to_string(t->tm_sec);//+"_"+to_string(SDM_->self_id_);
    command = "mkdir "+map_path_;
    cout<<"map command:"<<command<<endl;
    cout<<"map path_:"<<map_path_<<endl;
    system(command.c_str());

    /* set map data */
    sensor_range_2_ = sensor_range_ * sensor_range_;
    sensor_range_void_2_ = sensor_range_void_ * sensor_range_void_;
    // for(int x = -1; x < 2; x += 2){
    //     for(int y = -1; y < 2; y += 2){
    //         for(int z = -1; z < 2; z += 2){
    //             corners_.emplace_back(Eigen::Vector3d(resolution_*x, resolution_*y, resolution_*z));
    //         }
    //     }
    // }


    bs_x_(0) = block_size_(0);
    bs_x_(1) = block_size_(1) * block_size_(0);
    bs_x_(2) = block_size_(0) * block_size_(1) * block_size_(2);

    if(bs_x_(2) >= 65536){
        ROS_ERROR("block_size_(0) * block_size_(1) * block_size_(2) >= 2^16 too many voxels in one block");
        ros::shutdown();
        return;
    }
    void_frame_num_ = 0;

    res_inv_ = 1.0 / resolution_;

    map_upbd_.x() = ceil((map_upbd_.x() - origin_.x())/resolution_) * resolution_;
    map_upbd_.y() = ceil((map_upbd_.y() - origin_.y())/resolution_) * resolution_;
    map_upbd_.z() = ceil((map_upbd_.z() - origin_.z())/resolution_) * resolution_;

    float dx = origin_.x() - (floor((origin_.x())/resolution_)) * resolution_;
    float dy = origin_.y() - (floor((origin_.y())/resolution_)) * resolution_;
    float dz = origin_.z() - (floor((origin_.z())/resolution_)) * resolution_;

    origin_.x() -= dx;
    origin_.y() -= dy;
    origin_.z() -= dz;

    map_upbd_.x() += resolution_;
    map_upbd_.y() += resolution_;
    map_upbd_.z() += resolution_;

    voxel_num_.x() = ceil((map_upbd_.x())/resolution_);
    voxel_num_.y() = ceil((map_upbd_.y())/resolution_);
    voxel_num_.z() = ceil((map_upbd_.z())/resolution_);

    // map_upbd_ = origin_ + map_upbd_ - Eigen::Vector3f(1e-4, 1e-4, 1e-4);
    map_lowbd_ = origin_ + Eigen::Vector3f(1e-4, 1e-4, 1e-4);

    block_num_.x() = ceil(float(voxel_num_.x()) / block_size_.x());
    block_num_.y() = ceil(float(voxel_num_.y()) / block_size_.y());
    block_num_.z() = ceil(float(voxel_num_.z()) / block_size_.z());

    bn_x_.x() = block_num_.x();
    bn_x_.y() = block_num_.x() * block_num_.y();
    bn_x_.z() = block_num_.x() * block_num_.y() * block_num_.z();

    local_block_num_.x() = min(local_block_num_.x(), block_num_.x());
    local_block_num_.y() = min(local_block_num_.y(), block_num_.y());
    local_block_num_.z() = min(local_block_num_.z(), block_num_.z());

    lbn_x_.x() = local_block_num_.x();
    lbn_x_.y() = local_block_num_.x() * local_block_num_.y();
    lbn_x_.z() = local_block_num_.x() * local_block_num_.y() * local_block_num_.z();

    blockscale_.x() = resolution_*block_size_.x();
    blockscale_.y() = resolution_*block_size_.y();
    blockscale_.z() = resolution_*block_size_.z();

    map_upbd_ = origin_ + block_num_.cast<float>().cwiseProduct(blockscale_) - Eigen::Vector3f(1e-4f, 1e-4f, 1e-4f);

    bs_inv_.x() = 1.0 / blockscale_.x();
    bs_inv_.y() = 1.0 / blockscale_.y();
    bs_inv_.z() = 1.0 / blockscale_.z();

    local_scale_.x() = blockscale_.x() * local_block_num_.x();
    local_scale_.y() = blockscale_.y() * local_block_num_.y();
    local_scale_.z() = blockscale_.z() * local_block_num_.z();

    for(int dim = 0; dim < 3; dim++){
        inflate_r_size_ = ceil(max(inflate_r_scale_ * 0.5f - resolution_ * 0.5f + 1e-3f, 0.0f) * res_inv_) + 0.001f;
        if(block_size_(dim) < inflate_r_size_){
            cout<<block_size_.transpose()<<endl;
            cout<<inflate_r_size_<<endl;
            ROS_ERROR("error: inflate_size >= block_size(%d)", dim);
            ros::shutdown();
        }
        if(block_size_(dim) < 2){
            cout<<block_size_.transpose()<<endl;
            ROS_ERROR("error: inflate_size >= 2");
            ros::shutdown();
        }
        if(inflate_r_size_ == 0){
            cout<<block_size_.transpose()<<endl;
            cout<<inflate_r_size_<<endl;
            ROS_ERROR("error: inflate_size = 0");
            ros::shutdown();
        }
        bline_margin_(dim) = resolution_ * max(inflate_r_size_, 2);
    }

    /* for quick iter */
    six_connect_vec_.emplace_back(Eigen::Vector3i(-1, 0, 0));
    six_iter_idx_.push_back({0, -1});
    six_cross_iter_idx_.push_back({-1, block_size_(0) - 1});

    six_connect_vec_.emplace_back(Eigen::Vector3i(1, 0, 0));
    six_iter_idx_.push_back({0, 1});
    six_cross_iter_idx_.push_back({1, 1 - block_size_(0)});

    six_connect_vec_.emplace_back(Eigen::Vector3i(0, -1, 0));
    six_iter_idx_.push_back({0, -block_size_(0)});
    six_cross_iter_idx_.push_back({-local_block_num_(0), block_size_(0) * (block_size_(1) - 1)});

    six_connect_vec_.emplace_back(Eigen::Vector3i(0, 1, 0));
    six_iter_idx_.push_back({0, block_size_(0)});
    six_cross_iter_idx_.push_back({local_block_num_(0), -block_size_(0) * (block_size_(1) - 1)});

    six_connect_vec_.emplace_back(Eigen::Vector3i(0, 0, -1));
    six_iter_idx_.push_back({0, -block_size_(0)*block_size_(1)});
    six_cross_iter_idx_.push_back({-local_block_num_(0) * local_block_num_(1), block_size_(0) * block_size_(1) * (block_size_(2) - 1)});

    six_connect_vec_.emplace_back(Eigen::Vector3i(0, 0, 1));
    six_iter_idx_.push_back({0, block_size_(0)*block_size_(1)});
    six_cross_iter_idx_.push_back({local_block_num_(0) * local_block_num_(1), -block_size_(0) * block_size_(1) * (block_size_(2) - 1)});

    for(int x = -inflate_r_size_; x <= inflate_r_size_; x++){
        for(int y = -inflate_r_size_; y <= inflate_r_size_; y++){
            for(int z = -inflate_r_size_; z <= inflate_r_size_; z++){
                float dx = max(0.0f, abs(x) * resolution_ - 0.5f * resolution_);
                float dy = max(0.0f, abs(y) * resolution_ - 0.5f * resolution_);
                float dz = max(0.0f, abs(z) * resolution_ - 0.5f * resolution_);
                if(dx * dx + dy * dy + dz * dz < inflate_r_scale_ * inflate_r_scale_){
                    inflate_vec_.emplace_back(Eigen::Vector3i(x, y, z));
                }
            }
        }
    }

    local_origin_ = origin_;
    local_upbd_ = local_origin_ + local_scale_ - Eigen::Vector3f::Ones() * 1e-3f;
    local_lowbd_ = local_origin_ + Eigen::Vector3f::Ones() * 1e-3f;
    update_upbd_ = local_upbd_ - bline_margin_;
    update_lowbd_ = local_lowbd_ + bline_margin_;

    local_origin_idx_.setZero();
    local_up_idx_ = local_origin_idx_ + local_block_num_ - Eigen::Vector3i::Ones();
    local_origin_v_idx_ = local_origin_idx_.cwiseProduct(block_size_);
    local_up_v_idx_ = (local_up_idx_ + Eigen::Vector3i::Ones()).cwiseProduct(block_size_) - Eigen::Vector3i::Ones();
    PVBs_.resize(local_block_num_.x()*local_block_num_.y()*local_block_num_.z());
    Eigen::Vector3i it;
    for(int z = 0; z < local_block_num_.z(); z++){
        for(int y = 0; y < local_block_num_.y(); y++){
            for(int x = 0; x < local_block_num_.x(); x++){
                it = Eigen::Vector3i(x, y, z);
                int idx = x + y * local_block_num_.x() + z * local_block_num_.x() * local_block_num_.y();
                PVBs_[idx] = make_shared<PV_Block>();
                float loggodds_init = thr_min_ - 100.0;
                PVBs_[idx]->Reset(block_size_(0) * block_size_(1) * block_size_(2), loggodds_init);
                PVBs_[idx]->PvbFlags_ |= PVB_EMPTY;
                PVBs_[idx]->origin_ = Eigen::Vector3i(x * block_size_(0), y * block_size_(1), z * block_size_(2));
                Eigen::Vector3f p = PVBs_[idx]->origin_.cast<float>() * resolution_ + origin_ + Eigen::Vector3f::Ones() * 0.5;
                if(!InsideLocalMap(p)){
                    cout<<"p:"<<p.transpose()<<endl;
                    cout<<"x:"<<x<<" y:"<<y<<" z:"<<z<<endl;
                    cout<<"local_upbd_:"<<local_upbd_.transpose()<<endl;
                    cout<<"local_lowbd_:"<<local_lowbd_.transpose()<<endl;
                    cout<<"origin_:"<<origin_.transpose()<<endl;
                    cout<<"origin_:"<<origin_.transpose()<<endl;
                    StopageDebug("change failed");
                }

                PVBs_[idx]->id_ = GetBlockId(it);
            }
        }
    }

    cout<<"origin_:"<<origin_.transpose()<<endl;
    cout<<"map_lowbd_:"<<map_lowbd_.transpose()<<endl;
    cout<<"map_upbd_:"<<map_upbd_.transpose()<<endl;
    cout<<"block_num_:"<<block_num_.transpose()<<endl;
    cout<<"local_origin_:"<<local_origin_.transpose()<<endl;
    cout<<"local_upbd_:"<<local_upbd_.transpose()<<endl;
    cout<<"local_lowbd_:"<<local_lowbd_.transpose()<<endl;
    cout<<"update_upbd_:"<<update_upbd_.transpose()<<endl;
    cout<<"bline_margin_:"<<bline_margin_.transpose()<<endl;
    cout<<"update_lowbd_:"<<update_lowbd_.transpose()<<endl;
    cout<<"local_up_idx_:"<<local_up_idx_.transpose()<<endl;
    cout<<"local_origin_idx_:"<<local_origin_idx_.transpose()<<endl;
    cout<<"inflate_r_size_:"<<inflate_r_size_<<endl;
    cout<<"resolution_:"<<resolution_<<endl;
    cout<<"sensor_range_:"<<sensor_range_<<endl;

    thr_max_ = log(thr_max_ / (1 - thr_max_));
    thr_min_ = log(thr_min_ / (1 - thr_min_));
    pro_miss_ = log((1 - pro_miss_) / pro_miss_);
    free_thr_ = thr_min_ - 1e-3f;

    cout<<"thr_max_:"<<thr_max_<<endl;
    cout<<"pro_miss_:"<<pro_miss_<<endl;
    cout<<"thr_min_:"<<thr_min_<<endl;
    have_pose_ = false;

	/**** points lidar image ****/
    nh_private_.param(ns + "/block_map/Dtheta", 
        dtheta_, 0.1f);
    nh_private_.param(ns + "/block_map/Dpsi", 
        dpsi_, 0.1f);
    uint16_t h_size, v_size;
    dtheta_inv_ = 1.0 / dtheta_;
    dpsi_inv_ = 1.0 / dpsi_;
    h_size = ceil(M_PI * 2 * dtheta_inv_);
    v_size = ceil(M_PI * dpsi_inv_);
    dtheta_ = M_PI * 2 / h_size; 
    dpsi_ = M_PI / v_size; 
    dtheta_inv_ = 1.0 / dtheta_;
    dpsi_inv_ = 1.0 / dpsi_;
    cout<<"h_size:"<<h_size<<endl;
    cout<<"v_size:"<<v_size<<endl;
    cout<<"dtheta_:"<<dtheta_<<endl;
    cout<<"dpsi_:"<<dpsi_<<endl;

    InitRdImg(ray_depth_img_, h_size, v_size);


	/**** init color ****/
    vector<float> CR, CB, CG;
	nh_private_.param(ns + "/block_map/HeightcolorR", 
        CR, {});
    nh_private_.param(ns + "/block_map/HeightcolorG", 
        CG, {});
    nh_private_.param(ns + "/block_map/HeightcolorB", 
        CB, {});
	nh_private_.param(ns + "/block_map/show_freq", 
        show_freq_, 2.0f);
	nh_private_.param(ns + "/block_map/show_voxel", 
        show_vox_, true);
	nh_private_.param(ns + "/block_map/show_pts", 
        show_pts_, false);
	nh_private_.param(ns + "/block_map/show_frontier", 
        show_fontier_, false);


	std_msgs::ColorRGBA color;
    colorhsize_ = (map_upbd_(2) - origin_(2)) / (CG.size() - 1);
    for(int i = 0; i < CG.size(); i++){
        color.a = 1.0;
        color.r = CR[i]/255;
        color.g = CG[i]/255;
        color.b = CB[i]/255;
        color_list_.push_back(color);
    }

	vox_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(ns + "/block_map/voxvis", 10);
	pts_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(ns + "/block_map/points", 10);
	vis_timer_ = nh_.createTimer(ros::Duration(1.0 / show_freq_), &RipNeon::ShowMapCallback, this);
    sen_ready_pub_ = nh_.advertise<std_msgs::Empty>(ns + "/sensor/ready", 1);

    cout<<"color_list_:"<<color_list_.size()<<endl;

	/**** statistic ****/
	nh_private_.param(ns + "/block_map/statistic_v", 
		stat_, false);
    nh_private_.param(ns + "/Exp/maxX", 
        stat_upbd_.x(), -10.0f);
    nh_private_.param(ns + "/Exp/maxY", 
        stat_upbd_.y(), -10.0f);
    nh_private_.param(ns + "/Exp/maxZ", 
        stat_upbd_.z(), 0.0f);
    nh_private_.param(ns + "/Exp/minX", 
        stat_lowbd_.x(), 10.0f);
    nh_private_.param(ns + "/Exp/minY", 
        stat_lowbd_.y(), 10.0f);
    nh_private_.param(ns + "/Exp/minZ", 
        stat_lowbd_.z(), 0.0f);
    stat_n_ = 0;
    if(stat_){
        // stat_pub_ = nh.advertise<std_msgs::Float32>(ns + "/block_map/stat_v", 1);
        // stat_timer_ = nh.createTimer(ros::Duration(0.5), &RipNeon::StatisticV, this);
        // CS_.init(nh, nh_private);
    }

    debug_pub_ = nh_.advertise<visualization_msgs::Marker>(ns + "/block_map/debug", 10);

}

void RipNeon::AlignInit(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private,     
    const Eigen::Vector3d &origin, const Eigen::Vector3i &block_size, 
    const Eigen::Vector3i &block_num, const Eigen::Vector3i &local_block_num){
    nh_ = nh;
    nh_private_ = nh_private;
    std::string ns = ros::this_node::getName();
    nh_private_.param(ns + "/block_map/resolution", 
        resolution_, 0.2f);
    nh_private_.param(ns + "/block_map/sensor_max_range", 
        sensor_range_, 20.5f);
    nh_private_.param(ns + "/block_map/rot_x", 
        sen_odom_rot_.x(), 0.0f);
    nh_private_.param(ns + "/block_map/rot_y", 
        sen_odom_rot_.y(), 0.0f);
    nh_private_.param(ns + "/block_map/rot_z", 
        sen_odom_rot_.z(), 0.0f);
    nh_private_.param(ns + "/block_map/rot_w", 
        sen_odom_rot_.w(), 1.0f);
    nh_private_.param(ns + "/block_map/world_frame_pts", 
        world_frame_pts_, false);
    nh_private_.param(ns + "/block_map/occ_max", 
        thr_max_, 0.9f);
    nh_private_.param(ns + "/block_map/occ_min", 
        thr_min_, 0.1f);
    nh_private_.param(ns + "/block_map/pro_miss_free", 
        pro_miss_, 0.8f);    
    nh_private_.param(ns + "/block_map/inflate_scale", 
        inflate_r_scale_, 0.5f);
    nh_private_.param(ns + "/block_map/void_frame_thresh", 
        void_frame_thresh_, 25);
    nh_private_.param(ns + "/block_map/void_pts_thresh", 
        void_pts_thresh_, 4);
    nh_private_.param(ns + "/block_map/sensor_void_range", 
        sensor_range_void_, 10.0f);
    origin_ = origin.cast<float>();
    block_size_ = block_size;
    blockscale_.x() = resolution_*block_size_.x();
    blockscale_.y() = resolution_*block_size_.y();
    blockscale_.z() = resolution_*block_size_.z();
    block_num_ = block_num;
    map_upbd_ = origin_ + blockscale_.cwiseProduct(block_num_.cast<float>()) - Eigen::Vector3f(1e-4, 1e-4, 1e-4);
    map_lowbd_ = origin_ + Eigen::Vector3f(1e-4, 1e-4, 1e-4);
    voxel_num_ = block_size.cwiseProduct(block_num_);
    local_origin_ = origin_;
    local_scale_ = blockscale_.cwiseProduct(local_block_num.cast<float>());
    local_block_num_ = local_block_num;
    local_upbd_ = local_origin_ + local_scale_ - Eigen::Vector3f::Ones() * 1e-3;
    local_lowbd_ = local_origin_ + Eigen::Vector3f::Ones() * 1e-3;
    local_origin_idx_.setZero();
    local_up_idx_ = local_origin_idx_ + local_block_num_ - Eigen::Vector3i::Ones();
    local_origin_v_idx_ = local_origin_idx_.cwiseProduct(block_size_);
    local_up_v_idx_ = (local_origin_idx_ + Eigen::Vector3i::Ones()).cwiseProduct(block_size_) - Eigen::Vector3i::Ones();
    res_inv_ = 1.0 / resolution_;

    bn_x_.x() = block_num_.x();
    bn_x_.y() = block_num_.x() * block_num_.y();
    bn_x_.z() = block_num_.x() * block_num_.y() * block_num_.z();

    sensor_range_2_ = sensor_range_ * sensor_range_;
    sensor_range_void_2_ = sensor_range_void_ * sensor_range_void_;

    // for(int x = -1; x < 2; x += 2){
    //     for(int y = -1; y < 2; y += 2){
    //         for(int z = -1; z < 2; z += 2){
    //             corners_.emplace_back(Eigen::Vector3d(resolution_*x, resolution_*y, resolution_*z));
    //         }
    //     }
    // }

    void_frame_num_ = 0;

    bs_x_(0) = block_size_(0);
    bs_x_(1) = block_size_(1) * block_size_(0);
    bs_x_(2) = block_size_(0) * block_size_(1) * block_size_(2);
    if(bs_x_(2) >= 65536){
        ROS_ERROR("block_size_(0) * block_size_(1) * block_size_(2) >= 2^16 too many voxels in one block");
        ros::shutdown();
        return;
    }

    lbn_x_.x() = local_block_num_.x();
    lbn_x_.y() = local_block_num_.x() * local_block_num_.y();
    lbn_x_.z() = local_block_num_.x() * local_block_num_.y() * local_block_num_.z();

    blockscale_.x() = resolution_*block_size_.x();
    blockscale_.y() = resolution_*block_size_.y();
    blockscale_.z() = resolution_*block_size_.z();

    bs_inv_.x() = 1.0 / blockscale_.x();
    bs_inv_.y() = 1.0 / blockscale_.y();
    bs_inv_.z() = 1.0 / blockscale_.z();

    for(int dim = 0; dim < 3; dim++){
        inflate_r_size_ = ceil(max(inflate_r_scale_ * 0.5f - resolution_ * 0.5f + 1e-3f, 0.0f) * res_inv_) + 0.001f;
        if(block_size_(dim) < inflate_r_size_){
            cout<<block_size_.transpose()<<endl;
            cout<<inflate_r_size_<<endl;
            ROS_ERROR("error: inflate_size >= block_size(%d)", dim);
            ros::shutdown();
        }
        if(block_size_(dim) < 2){
            cout<<block_size_.transpose()<<endl;
            ROS_ERROR("error: inflate_size >= 2");
            ros::shutdown();
        }
        if(inflate_r_size_ == 0){
            cout<<block_size_.transpose()<<endl;
            cout<<inflate_r_size_<<endl;
            ROS_ERROR("error: inflate_size = 0");
            ros::shutdown();
        }
        bline_margin_(dim) = resolution_ * max(inflate_r_size_, 2);
    }

    thr_max_ = log(thr_max_ / (1 - thr_max_));
    thr_min_ = log(thr_min_ / (1 - thr_min_));
    pro_miss_ = log((1 - pro_miss_) / pro_miss_);
    free_thr_ = thr_min_ - 1e-3f;

    cout<<"thr_max_:"<<thr_max_<<endl;
    cout<<"pro_miss_:"<<pro_miss_<<endl;
    cout<<"thr_min_:"<<thr_min_<<endl;
    cout<<"block_size_:"<<block_size_.transpose()<<endl;
    cout<<"blockscale_:"<<blockscale_.transpose()<<endl;
    cout<<"block_num_:"<<block_num_.transpose()<<endl;
    cout<<"origin_:"<<origin_.transpose()<<endl;
    cout<<"map_upbd_:"<<map_upbd_.transpose()<<endl;
    cout<<"voxel_num_:"<<voxel_num_.transpose()<<endl;

    /* create read write path */
    nh_private.param(ns + "/block_map/Path", 
        map_path_, map_path_);
    int tn;
    nh_private.param(ns + "/block_map/thread_num", 
        tn, 5);
    thread_num_ = tn;
    string command;
    time_t now = time(0);
    tm* t=localtime(&now);
    map_path_ = map_path_+"/"+to_string(t->tm_year+1900)+"_"+to_string(t->tm_mon+1)+"_"+to_string(t->tm_mday)
    +"_"+to_string(t->tm_hour)+"_"+to_string(t->tm_min)+"_"+to_string(t->tm_sec);//+"_"+to_string(SDM_->self_id_);
    command = "mkdir "+map_path_;
    cout<<"map command:"<<command<<endl;
    cout<<"map path_:"<<map_path_<<endl;
    system(command.c_str());


    /* for quick iter */
    six_connect_vec_.emplace_back(Eigen::Vector3i(-1, 0, 0));
    six_iter_idx_.push_back({0, -1});
    six_cross_iter_idx_.push_back({-1, block_size_(0) - 1});

    six_connect_vec_.emplace_back(Eigen::Vector3i(1, 0, 0));
    six_iter_idx_.push_back({0, 1});
    six_cross_iter_idx_.push_back({1, 1 - block_size_(0)});

    six_connect_vec_.emplace_back(Eigen::Vector3i(0, -1, 0));
    six_iter_idx_.push_back({0, -block_size_(0)});
    six_cross_iter_idx_.push_back({-local_block_num_(0), block_size_(0) * (block_size_(1) - 1)});

    six_connect_vec_.emplace_back(Eigen::Vector3i(0, 1, 0));
    six_iter_idx_.push_back({0, block_size_(0)});
    six_cross_iter_idx_.push_back({local_block_num_(0), -block_size_(0) * (block_size_(1) - 1)});

    six_connect_vec_.emplace_back(Eigen::Vector3i(0, 0, -1));
    six_iter_idx_.push_back({0, -block_size_(0)*block_size_(1)});
    six_cross_iter_idx_.push_back({-local_block_num_(0) * local_block_num_(1), block_size_(0) * block_size_(1) * (block_size_(2) - 1)});

    six_connect_vec_.emplace_back(Eigen::Vector3i(0, 0, 1));
    six_iter_idx_.push_back({0, block_size_(0)*block_size_(1)});
    six_cross_iter_idx_.push_back({local_block_num_(0) * local_block_num_(1), -block_size_(0) * block_size_(1) * (block_size_(2) - 1)});

    for(int x = -inflate_r_size_; x <= inflate_r_size_; x++){
        for(int y = -inflate_r_size_; y <= inflate_r_size_; y++){
            for(int z = -inflate_r_size_; z <= inflate_r_size_; z++){
                float dx = max(0.0f, abs(x) * resolution_ - 0.5f * resolution_);
                float dy = max(0.0f, abs(y) * resolution_ - 0.5f * resolution_);
                float dz = max(0.0f, abs(z) * resolution_ - 0.5f * resolution_);
                if(dx * dx + dy * dy + dz * dz < inflate_r_scale_ * inflate_r_scale_){
                    inflate_vec_.emplace_back(Eigen::Vector3i(x, y, z));
                }
            }
        }
    }

    local_origin_ = origin_;
    local_upbd_ = local_origin_ + local_scale_ - Eigen::Vector3f::Ones() * 1e-3f;
    local_lowbd_ = local_origin_ + Eigen::Vector3f::Ones() * 1e-3f;
    update_upbd_ = local_upbd_ - bline_margin_;
    update_lowbd_ = local_lowbd_ + bline_margin_;

    local_origin_idx_.setZero();
    local_up_idx_ = local_origin_idx_ + local_block_num_ - Eigen::Vector3i::Ones();
    local_origin_v_idx_ = local_origin_idx_.cwiseProduct(block_size_);
    local_up_v_idx_ = (local_up_idx_ + Eigen::Vector3i::Ones()).cwiseProduct(block_size_) - Eigen::Vector3i::Ones();

    PVBs_.resize(local_block_num_.x()*local_block_num_.y()*local_block_num_.z());
    Eigen::Vector3i it;
    for(int z = 0; z < local_block_num_.z(); z++){
        for(int y = 0; y < local_block_num_.y(); y++){
            for(int x = 0; x < local_block_num_.x(); x++){
                it = Eigen::Vector3i(x, y, z);
                int idx = x + y * local_block_num_.x() + z * local_block_num_.x() * local_block_num_.y();
                PVBs_[idx] = make_shared<PV_Block>();
                float loggodds_init = thr_min_ - 100.0;
                PVBs_[idx]->Reset(block_size_(0) * block_size_(1) * block_size_(2), loggodds_init);
                PVBs_[idx]->PvbFlags_ |= PVB_EMPTY;
                PVBs_[idx]->origin_ = Eigen::Vector3i(x * block_size_(0), y * block_size_(1), z * block_size_(2));
                Eigen::Vector3f p = PVBs_[idx]->origin_.cast<float>() * resolution_ + origin_ + Eigen::Vector3f::Ones() * 0.5;
                if(!InsideLocalMap(p)){
                    cout<<"p:"<<p.transpose()<<endl;
                    cout<<"x:"<<x<<" y:"<<y<<" z:"<<z<<endl;
                    cout<<"local_upbd_:"<<local_upbd_.transpose()<<endl;
                    cout<<"local_lowbd_:"<<local_lowbd_.transpose()<<endl;
                    cout<<"origin_:"<<origin_.transpose()<<endl;
                    cout<<"origin_:"<<origin_.transpose()<<endl;
                    StopageDebug("change failed");
                }

                PVBs_[idx]->id_ = GetBlockId(it);
            }
        }
    }

	/**** points lidar image ****/
    nh_private_.param(ns + "/block_map/Dtheta", 
        dtheta_, 0.1f);
    nh_private_.param(ns + "/block_map/Dpsi", 
        dpsi_, 0.1f);
    uint16_t h_size, v_size;
    dtheta_inv_ = 1.0 / dtheta_;
    dpsi_inv_ = 1.0 / dpsi_;
    h_size = ceil(M_PI * 2 * dtheta_inv_);
    v_size = ceil(M_PI * dpsi_inv_);
    dtheta_ = M_PI * 2 / h_size; 
    dpsi_ = M_PI / v_size; 
    dtheta_inv_ = 1.0 / dtheta_;
    dpsi_inv_ = 1.0 / dpsi_;
    cout<<"h_size:"<<h_size<<endl;
    cout<<"v_size:"<<v_size<<endl;
    cout<<"dtheta_:"<<dtheta_<<endl;
    cout<<"dpsi_:"<<dpsi_<<endl;

    InitRdImg(ray_depth_img_, h_size, v_size);

	/**** init color ****/
    vector<float> CR, CB, CG;
	nh_private_.param(ns + "/block_map/HeightcolorR", 
        CR, {});
    nh_private_.param(ns + "/block_map/HeightcolorG", 
        CG, {});
    nh_private_.param(ns + "/block_map/HeightcolorB", 
        CB, {});
	nh_private_.param(ns + "/block_map/show_freq", 
        show_freq_, 2.0f);
	nh_private_.param(ns + "/block_map/show_voxel", 
        show_vox_, true);
	nh_private_.param(ns + "/block_map/show_pts", 
        show_pts_, false);
	nh_private_.param(ns + "/block_map/show_frontier", 
        show_fontier_, false);


	std_msgs::ColorRGBA color;
    colorhsize_ = (map_upbd_(2) - origin_(2)) / (CG.size() - 1);
    for(int i = 0; i < CG.size(); i++){
        color.a = 1.0;
        color.r = CR[i]/255;
        color.g = CG[i]/255;
        color.b = CB[i]/255;
        color_list_.push_back(color);
    }

	vox_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(ns + "/block_map/voxvis", 10);
	pts_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(ns + "/block_map/points", 10);
	vis_timer_ = nh_.createTimer(ros::Duration(1.0 / show_freq_), &RipNeon::ShowMapCallback, this);
    sen_ready_pub_ = nh_.advertise<std_msgs::Empty>(ns + "/sensor/ready", 1);

    cout<<"color_list_:"<<color_list_.size()<<endl;

	/**** statistic ****/
	nh_private_.param(ns + "/block_map/statistic_v", 
		stat_, false);
    nh_private_.param(ns + "/Exp/maxX", 
        stat_upbd_.x(), -10.0f);
    nh_private_.param(ns + "/Exp/maxY", 
        stat_upbd_.y(), -10.0f);
    nh_private_.param(ns + "/Exp/maxZ", 
        stat_upbd_.z(), 0.0f);
    nh_private_.param(ns + "/Exp/minX", 
        stat_lowbd_.x(), 10.0f);
    nh_private_.param(ns + "/Exp/minY", 
        stat_lowbd_.y(), 10.0f);
    nh_private_.param(ns + "/Exp/minZ", 
        stat_lowbd_.z(), 0.0f);
    stat_n_ = 0;
    if(stat_){
        // stat_pub_ = nh_.advertise<std_msgs::Float32>(ns + "/block_map/stat_v", 1);
        // stat_timer_ = nh_.createTimer(ros::Duration(0.5), &RipNeon::StatisticV, this);
        // CS_.init(nh_, nh_private_);
    }

    debug_pub_ = nh_.advertise<visualization_msgs::Marker>(ns + "/block_map/debug", 10);
}