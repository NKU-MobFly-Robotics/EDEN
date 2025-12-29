#include <block_map_lite/block_map_lite.h>
void BlockMapLite::init(ros::NodeHandle &nh, ros::NodeHandle &nh_private){

    nh_ = nh;
    nh_private_ = nh_private;
    std::string ns = ros::this_node::getName();
    
    vector<double> CR, CB, CG;
    bool depth;
    Eigen::Quaterniond cam2bodyrot;
    Eigen::Vector3d robots_scale;

    cam2body_.setZero();
    int drone_num;
    nh_private_.param(ns + "/block_map/HeightcolorR", 
        CR, {});
    nh_private_.param(ns + "/block_map/HeightcolorG", 
        CG, {});
    nh_private_.param(ns + "/block_map/HeightcolorB", 
        CB, {});
    nh_private_.param(ns + "/block_map/minX", 
        origin_.x(), -10.0);
    nh_private_.param(ns + "/block_map/minY", 
        origin_.y(), -10.0);
    nh_private_.param(ns + "/block_map/minZ", 
        origin_.z(), 0.0);
    nh_private_.param(ns + "/block_map/maxX", 
        map_upbd_.x(), 10.0);
    nh_private_.param(ns + "/block_map/maxY", 
        map_upbd_.y(), 10.0);
    nh_private_.param(ns + "/block_map/maxZ", 
        map_upbd_.z(), 0.0);
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
        resolution_, 0.2);
    nh_private_.param(ns + "/block_map/sensor_max_range", 
        max_range_, 4.5);
    nh_private_.param(ns + "/block_map/CamtoBody_Quater_x",
        cam2bodyrot.x(), 0.0);
    nh_private_.param(ns + "/block_map/CamtoBody_Quater_y",
        cam2bodyrot.y(), 0.0);
    nh_private_.param(ns + "/block_map/CamtoBody_Quater_z",
        cam2bodyrot.z(), 0.0);
    nh_private_.param(ns + "/block_map/CamtoBody_Quater_w",
        cam2bodyrot.w(), 1.0);
    nh_private_.param(ns + "/block_map/CamtoBody_x",
        cam2body_(0, 3), 0.0);
    nh_private_.param(ns + "/block_map/CamtoBody_y",
        cam2body_(1, 3), 0.0);
    nh_private_.param(ns + "/block_map/CamtoBody_z",
        cam2body_(2, 3), 0.0);
    // nh_private_.param(ns + "/block_map/update_freq", 
    //     update_interval_, 5.0);
    nh_private_.param(ns + "/block_map/show_freq", 
        show_freq_, 2.0);
    nh_private_.param(ns + "/block_map/depth_step", 
        depth_step_, 2);
    nh_private_.param(ns + "/block_map/occ_max", 
        thr_max_, 0.9);
    nh_private_.param(ns + "/block_map/occ_min", 
        thr_min_, 0.1);
    nh_private_.param(ns + "/block_map/pro_hit_occ", 
        pro_hit_, 0.7);
    nh_private_.param(ns + "/block_map/pro_miss_free", 
        pro_miss_, 0.8);    
    nh_private_.param(ns + "/block_map/statistic_v", 
        stat_, false);
    nh_private_.param(ns + "/Exp/statistic", 
        stat_write_, false);
    // nh_private_.param(ns + "/block_map/min_finish_t", 
    //     min_finish_t_, 30.0);
    nh_private_.param(ns + "/Exp/maxX", 
        stat_upbd_.x(), -10.0);
    nh_private_.param(ns + "/Exp/maxY", 
        stat_upbd_.y(), -10.0);
    nh_private_.param(ns + "/Exp/maxZ", 
        stat_upbd_.z(), 0.0);
    nh_private_.param(ns + "/Exp/minX", 
        stat_lowbd_.x(), 10.0);
    nh_private_.param(ns + "/Exp/minY", 
        stat_lowbd_.y(), 10.0);
    nh_private_.param(ns + "/Exp/minZ", 
        stat_lowbd_.z(), 0.0);
    nh_private_.param(ns + "/block_map/RobotVisX", 
        robots_scale.x(), 0.5);
    nh_private_.param(ns + "/block_map/RobotVisY", 
        robots_scale.y(), 0.5);
    nh_private_.param(ns + "/block_map/RobotVisZ", 
        robots_scale.z(), 0.3);
    nh_private_.param(ns + "/block_map/swarm_pub_thresh", 
        swarm_pub_thresh_, 0.95);
    nh_private.param(ns + "/block_map/bline_occ_range", 
        bline_occ_range_, 0.7);
    nh_private.param(ns + "/block_map/vis_mode", 
        vis_mode_, false);


    nh_private.param(ns + "/block_map/swarm_tol", 
        swarm_tol_, 0.2);
    nh_private.param(ns + "/block_map/swarm_send_delay", 
        swarm_send_delay_, 0.2);
    nh_private.param(ns + "/block_map/show_block", 
        show_block_, true);
    nh_private.param(ns + "/block_map/Path", 
        map_path_, map_path_);

    string command;
    time_t now = time(0);
    tm* t=localtime(&now);
    map_path_ = map_path_+"/"+to_string(t->tm_year+1900)+"_"+to_string(t->tm_mon+1)+"_"+to_string(t->tm_mday)
    +"_"+to_string(t->tm_hour)+"_"+to_string(t->tm_min)+"_"+to_string(t->tm_sec);//+"_"+to_string(SDM_->self_id_);
    command = "mkdir "+map_path_;
    cout<<"map command:"<<command<<endl;
    cout<<"map path_:"<<map_path_<<endl;
    system(command.c_str());

    total_map_t_ = 0;
    map_count_ = 0;
    start_t_ = ros::WallTime::now().toSec();
    req_flag_ = false;
    finish_flag_ = false;
    // if(SDM_->drone_num_ > 1){  //swarm exploration
    //     use_swarm_ = true;
    //     drone_num = SDM_->drone_num_;
    //     swarm_filter_dict_ = new tr1::unordered_map<int, int>;
    //     swarm_pose_.resize(drone_num);
    //     cout<<"robots_scale_:"<<robots_scale.transpose()<<endl;
    //     for(int i = 0; i < drone_num; i++) robots_scale_.emplace_back(robots_scale);
    //     for(auto &sp : swarm_pose_) sp.first = ros::WallTime::now().toSec() - 1000.0;
    //     self_id_ = SDM_->self_id_;
    // }
    // else {
        drone_num = 1;
        use_swarm_ = false;
    // }
    stat_n_ = 0;

    // cout<<pro_hit_<<" "<<pro_miss_<<endl;

    cam2body_.block(0, 0, 3, 3) = cam2bodyrot.matrix();

    cam2body_(3, 3) = 1.0;

    map_upbd_.x() = ceil((map_upbd_.x() - origin_.x())/resolution_) * resolution_;
    map_upbd_.y() = ceil((map_upbd_.y() - origin_.y())/resolution_) * resolution_;
    map_upbd_.z() = ceil((map_upbd_.z() - origin_.z())/resolution_) * resolution_;

    // block_num_ = 



    double dx = origin_.x() - (floor((origin_.x())/resolution_)) * resolution_;
    double dy = origin_.y() - (floor((origin_.y())/resolution_)) * resolution_;
    double dz = origin_.z() - (floor((origin_.z())/resolution_)) * resolution_;

    origin_.x() -= dx;
    origin_.y() -= dy;
    origin_.z() -= dz;

    map_upbd_.x() += resolution_;
    map_upbd_.y() += resolution_;
    map_upbd_.z() += resolution_;
    voxel_num_.x() = ceil((map_upbd_.x())/resolution_);
    voxel_num_.y() = ceil((map_upbd_.y())/resolution_);
    voxel_num_.z() = ceil((map_upbd_.z())/resolution_);

    block_num_.x() = ceil(double(voxel_num_.x()) / block_size_.x());
    block_num_.y() = ceil(double(voxel_num_.y()) / block_size_.y());
    block_num_.z() = ceil(double(voxel_num_.z()) / block_size_.z());
    blockscale_.x() = resolution_*block_size_.x();
    blockscale_.y() = resolution_*block_size_.y();
    blockscale_.z() = resolution_*block_size_.z();

    voxel_num_ = block_num_.cwiseProduct(block_size_);
    map_upbd_ = origin_ + blockscale_.cwiseProduct(block_num_.cast<double>()) - Vector3d(1e-4, 1e-4, 1e-4);
    map_lowbd_ = origin_ + Vector3d(1e-4, 1e-4, 1e-4);

    local_scale_.x() = blockscale_.x() * local_block_num_.x();
    local_scale_.y() = blockscale_.y() * local_block_num_.y();
    local_scale_.z() = blockscale_.z() * local_block_num_.z();

    // Eigen::Vector3i local_origin_idx_, local_up_idx_; // block idx
    local_origin_ = origin_;
    local_upbd_ = local_origin_ + local_scale_ - Eigen::Vector3d::Ones() * 1e-3;
    local_lowbd_ = local_origin_ + Eigen::Vector3d::Ones() * 1e-3;
    local_origin_idx_.setZero();
    local_up_idx_ = local_origin_idx_ + local_block_num_ - Eigen::Vector3i::Ones();
    GBS_local_.resize(local_block_num_.x()*local_block_num_.y()*local_block_num_.z());
    Eigen::Vector3i it;
    for(int z = 0; z < local_block_num_.z(); z++){
        for(int y = 0; y < local_block_num_.y(); y++){
            for(int x = 0; x < local_block_num_.x(); x++){
                it = Eigen::Vector3i(x, y, z);
                int idx = x + y * local_block_num_.x() + z * local_block_num_.x() * local_block_num_.y();
                GBS_local_[idx] = make_shared<Grid_Block>();
                GBS_local_[idx]->Reset(-9999.0, block_size_(0) * block_size_(1) * block_size_(2));
                GBS_local_[idx]->origin_ = Eigen::Vector3i(x * block_num_(0), y * block_num_(1), z * block_num_(2));
                GBS_local_[idx]->id_ = GetBlockId(it);
            }
        }
    }
    cout<<"local_up_idx_:"<<local_up_idx_.transpose()<<endl;
    cout<<"local_block_num_:"<<local_block_num_.transpose()<<endl;
    cout<<"local_block_num_:"<<local_block_num_.transpose()<<endl;

    cout<<"origin_:"<<origin_.transpose()<<endl;
    cout<<"blockscale_:"<<blockscale_.transpose()<<endl;
    cout<<"block_num_:"<<block_num_.transpose()<<endl;
    cout<<"block_size_:"<<block_size_.transpose()<<endl;
    cout<<"voxel_num_:"<<voxel_num_.transpose()<<endl;
    cout<<"map_upbd_:"<<map_upbd_.transpose()<<endl;
    cout<<"origin_:"<<origin_.transpose()<<endl;

    std_msgs::ColorRGBA color;
    colorhsize_ = (map_upbd_(2) - origin_(2)) / (CG.size() - 1);
    for(int i = 0; i < CG.size(); i++){
        color.a = 1.0;
        color.r = CR[i]/255;
        color.g = CG[i]/255;
        color.b = CB[i]/255;
        color_list_.push_back(color);
    }

    have_odom_ = false;
    have_cam_param_ = false;
    last_odom_ = ros::WallTime::now().toSec() + 100000.0;
    last_update_ = ros::WallTime::now().toSec() - 10.0;
    cout<<"last_update_:"<<last_update_<<endl;
    // update_interval_ = 1 / update_interval_;
    // cout<<"update_interval_:"<<update_interval_<<endl;

    thr_max_ = log(thr_max_ / (1 - thr_max_));
    thr_min_ = log(thr_min_ / (1 - thr_min_));
    pro_hit_ = log(pro_hit_ / (1 - pro_miss_));
    pro_miss_ = log((1 - pro_miss_) / pro_miss_);
    cout<<"thr_max_:"<<thr_max_<<endl;
    cout<<"thr_min_:"<<thr_min_<<endl;
    cout<<"pro_hit_:"<<pro_hit_<<endl;
    cout<<"pro_miss_:"<<pro_miss_<<endl;
    visall_sub_ = nh_.subscribe(ns + "/block_map/visall", 1, &BlockMapLite::VisMapAll, this);
    vox_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(ns + "/block_map/voxvis", 10);
    debug_pub_ = nh_.advertise<visualization_msgs::Marker>(ns + "/block_map/debug", 10);

    if(stat_) {
        statistic_pub_ = nh.advertise<std_msgs::Float32>(ns + "/block_map/stat_v", 1);
        statistic_timer_ = nh.createTimer(ros::Duration(0.5), &BlockMapLite::StatisticV, this);
    }
    // if(depth_){
    camparam_sub_ = nh_.subscribe("/block_map/caminfo", 10, &BlockMapLite::CamParamCallback, this);
    // }
    if(show_block_)
        show_timer_ = nh_.createTimer(ros::Duration(1.0 / show_freq_), &BlockMapLite::ShowMapCallback, this);

    // manage_map_timer_ = nh_.createTimer(ros::Duration(0.2), &BlockMapLite::MangageMapCallback, this);

    // if(use_swarm_){
    //     if(SDM_->is_ground_) swarm_timer_ = nh_.createTimer(ros::Duration(0.2), &BlockMapLite::SwarmMapCallback, this);
    //     else swarm_timer_ = nh_.createTimer(ros::Duration(1.0), &BlockMapLite::SwarmMapCallback, this);
    // }
}

void BlockMapLite::AlignInit(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, 
    const Eigen::Vector3d &origin, const Eigen::Vector3i &block_size, 
    const Eigen::Vector3i &block_num, const Eigen::Vector3i &local_block_num){
    nh_ = nh;
    nh_private_ = nh_private;
    std::string ns = ros::this_node::getName();
    nh_private_.param(ns + "/block_map/resolution", 
        resolution_, 0.2);
    ROS_WARN("BM");
    origin_ = origin;
    block_size_ = block_size;
    blockscale_.x() = resolution_*block_size_.x();
    blockscale_.y() = resolution_*block_size_.y();
    blockscale_.z() = resolution_*block_size_.z();
    block_num_ = block_num;
    map_upbd_ = origin_ + blockscale_.cwiseProduct(block_num_.cast<double>()) - Vector3d(1e-4, 1e-4, 1e-4);
    map_lowbd_ = origin_ + Vector3d(1e-4, 1e-4, 1e-4);
    voxel_num_ = block_size.cwiseProduct(block_num_);
    local_origin_ = origin_;
    local_scale_ = blockscale_.cwiseProduct(local_block_num.cast<double>());
    local_block_num_ = local_block_num;
    local_upbd_ = local_origin_ + local_scale_ - Eigen::Vector3d::Ones() * 1e-3;
    local_lowbd_ = local_origin_ + Eigen::Vector3d::Ones() * 1e-3;
    local_origin_idx_.setZero();
    local_up_idx_ = local_origin_idx_ + local_block_num_ - Eigen::Vector3i::Ones();
    vn_ = 1;
    vn_ *= voxel_num_(0);
    vn_ *= voxel_num_(1);
    vn_ *= voxel_num_(2);
    ROS_WARN("BM");
    cout<<"block_size_:"<<block_size_.transpose()<<endl;
    cout<<"blockscale_:"<<blockscale_.transpose()<<endl;
    cout<<"block_num_:"<<block_num_.transpose()<<endl;
    cout<<"origin_:"<<origin_.transpose()<<endl;
    cout<<"map_upbd_:"<<map_upbd_.transpose()<<endl;
    cout<<"voxel_num_:"<<voxel_num_.transpose()<<endl;
    cout<<"vn_:"<<vn_<<endl;

    GBS_local_.resize(local_block_num_.x()*local_block_num_.y()*local_block_num_.z());
    Eigen::Vector3i it;
    for(int z = 0; z < local_block_num_.z(); z++){
        for(int y = 0; y < local_block_num_.y(); y++){
            for(int x = 0; x < local_block_num_.x(); x++){
                it = Eigen::Vector3i(x, y, z);
                int idx = x + y * local_block_num_.x() + z * local_block_num_.x() * local_block_num_.y();
                GBS_local_[idx] = make_shared<Grid_Block>();
                GBS_local_[idx]->Reset(-9999.0, block_size_(0) * block_size_(1) * block_size_(2));
                GBS_local_[idx]->origin_ = Eigen::Vector3i(x * block_num_(0), y * block_num_(1), z * block_num_(2));
                GBS_local_[idx]->id_ = GetBlockId(it);
            }
        }
    }

    vector<double> CR, CB, CG;
    Eigen::Vector3d robots_scale;
    Eigen::Quaterniond cam2bodyrot;
    
    nh_private_.param(ns + "/block_map/HeightcolorR", 
        CR, {});
    nh_private_.param(ns + "/block_map/HeightcolorG", 
        CG, {});
    nh_private_.param(ns + "/block_map/HeightcolorB", 
        CB, {});
    nh_private_.param(ns + "/block_map/sensor_max_range", 
        max_range_, 4.5);
    nh_private_.param(ns + "/block_map/CamtoBody_Quater_x",
        cam2bodyrot.x(), 0.0);
    nh_private_.param(ns + "/block_map/CamtoBody_Quater_y",
        cam2bodyrot.y(), 0.0);
    nh_private_.param(ns + "/block_map/CamtoBody_Quater_z",
        cam2bodyrot.z(), 0.0);
    nh_private_.param(ns + "/block_map/CamtoBody_Quater_w",
        cam2bodyrot.w(), 1.0);
    nh_private_.param(ns + "/block_map/CamtoBody_x",
        cam2body_(0, 3), 0.0);
    nh_private_.param(ns + "/block_map/CamtoBody_y",
        cam2body_(1, 3), 0.0);
    nh_private_.param(ns + "/block_map/CamtoBody_z",
        cam2body_(2, 3), 0.0);
    nh_private_.param(ns + "/block_map/show_freq", 
        show_freq_, 2.0);
    nh_private_.param(ns + "/block_map/depth_step", 
        depth_step_, 2);
    nh_private_.param(ns + "/block_map/occ_max", 
        thr_max_, 0.9);
    nh_private_.param(ns + "/block_map/occ_min", 
        thr_min_, 0.1);
    nh_private_.param(ns + "/block_map/pro_hit_occ", 
        pro_hit_, 0.7);
    nh_private_.param(ns + "/block_map/pro_miss_free", 
        pro_miss_, 0.8);    
    nh_private_.param(ns + "/block_map/statistic_v", 
        stat_, false);
    nh_private_.param(ns + "/Exp/statistic", 
        stat_write_, false);
    nh_private_.param(ns + "/Exp/maxX", 
        stat_upbd_.x(), -10.0);
    nh_private_.param(ns + "/Exp/maxY", 
        stat_upbd_.y(), -10.0);
    nh_private_.param(ns + "/Exp/maxZ", 
        stat_upbd_.z(), 0.0);
    nh_private_.param(ns + "/Exp/minX", 
        stat_lowbd_.x(), 10.0);
    nh_private_.param(ns + "/Exp/minY", 
        stat_lowbd_.y(), 10.0);
    nh_private_.param(ns + "/Exp/minZ", 
        stat_lowbd_.z(), 0.0);
    nh_private_.param(ns + "/block_map/RobotVisX", 
        robots_scale.x(), 0.5);
    nh_private_.param(ns + "/block_map/RobotVisY", 
        robots_scale.y(), 0.5);
    nh_private_.param(ns + "/block_map/RobotVisZ", 
        robots_scale.z(), 0.3);
    nh_private_.param(ns + "/block_map/swarm_pub_thresh", 
        swarm_pub_thresh_, 0.95);
    nh_private.param(ns + "/block_map/vis_mode", 
        vis_mode_, false);
    nh_private.param(ns + "/block_map/swarm_tol", 
        swarm_tol_, 0.2);
    nh_private.param(ns + "/block_map/swarm_send_delay", 
        swarm_send_delay_, 0.2);
    nh_private.param(ns + "/block_map/show_block", 
        show_block_, true);
    nh_private.param(ns + "/block_map/Path", 
        map_path_, map_path_);
    thr_max_ = log(thr_max_ / (1 - thr_max_));
    thr_min_ = log(thr_min_ / (1 - thr_min_));
    pro_hit_ = log(pro_hit_ / (1 - pro_miss_));
    pro_miss_ = log((1 - pro_miss_) / pro_miss_);

    string command;
    time_t now = time(0);
    tm* t=localtime(&now);
    map_path_ = map_path_+"/"+to_string(t->tm_year+1900)+"_"+to_string(t->tm_mon+1)+"_"+to_string(t->tm_mday)
    +"_"+to_string(t->tm_hour)+"_"+to_string(t->tm_min)+"_"+to_string(t->tm_sec);//+"_"+to_string(SDM_->self_id_);
    command = "mkdir "+map_path_;
    cout<<"map command:"<<command<<endl;
    cout<<"map path_:"<<map_path_<<endl;
    system(command.c_str());

    total_map_t_ = 0;
    map_count_ = 0;
    start_t_ = ros::WallTime::now().toSec();
    req_flag_ = false;
    finish_flag_ = false;
    // if(SDM_->drone_num_ > 1){  //swarm exploration
    //     use_swarm_ = true;
    //     swarm_filter_dict_ = new tr1::unordered_map<int, int>;
    //     swarm_pose_.resize(SDM_->drone_num_);
    //     cout<<"robots_scale_:"<<robots_scale.transpose()<<endl;
    //     for(int i = 0; i < SDM_->drone_num_; i++) robots_scale_.emplace_back(robots_scale);
    //     for(auto &sp : swarm_pose_) sp.first = ros::WallTime::now().toSec() - 1000.0;
    //     self_id_ = SDM_->self_id_;
    // }
    // else {
        use_swarm_ = false;
    // }
    stat_n_ = 0;

    cam2body_.block(0, 0, 3, 3) = cam2bodyrot.matrix();

    cam2body_(3, 3) = 1.0;

    std_msgs::ColorRGBA color;
    colorhsize_ = (map_upbd_(2) - origin_(2)) / (CG.size() - 1);
    for(int i = 0; i < CG.size(); i++){
        color.a = 1.0;
        color.r = CR[i]/255;
        color.g = CG[i]/255;
        color.b = CB[i]/255;
        color_list_.push_back(color);
    }

    have_odom_ = false;
    have_cam_param_ = false;
    last_odom_ = ros::WallTime::now().toSec() + 100000.0;
    last_update_ = ros::WallTime::now().toSec() - 10.0;

    visall_sub_ = nh_.subscribe(ns + "/block_map/visall", 1, &BlockMapLite::VisMapAll, this);
    vox_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(ns + "/block_map/voxvis", 10);
    debug_pub_ = nh_.advertise<visualization_msgs::Marker>(ns + "/block_map/debug", 10);

    if(stat_) {
        statistic_pub_ = nh_.advertise<std_msgs::Float32>(ns + "/block_map/stat_v", 1);
        statistic_timer_ = nh_.createTimer(ros::Duration(0.5), &BlockMapLite::StatisticV, this);
    }
    camparam_sub_ = nh_.subscribe("/block_map/caminfo", 10, &BlockMapLite::CamParamCallback, this);
    if(show_block_)
        show_timer_ = nh_.createTimer(ros::Duration(1.0 / show_freq_), &BlockMapLite::ShowMapCallback, this);

    // manage_map_timer_ = nh_.createTimer(ros::Duration(0.2), &BlockMapLite::MangageMapCallback, this);
}


