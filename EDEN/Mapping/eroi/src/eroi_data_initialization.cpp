#include <eroi/eroi.h>

// void EroiGrid::init(ros::NodeHandle &nh, ros::NodeHandle &nh_private){
//     nh_ = nh;
//     nh_private_ = nh_private;
//     std::string ns = ros::this_node::getName();
//     string sensor;
//     bool show_frontier;
//     nh_private_.param(ns + "/Exp/minX", 
//         origin_.x(), -10.0);
//     nh_private_.param(ns + "/Exp/minY", 
//         origin_.y(), -10.0);
//     nh_private_.param(ns + "/Exp/minZ", 
//         origin_.z(), 0.0);
//     nh_private_.param(ns + "/Exp/maxX", 
//         up_bd_.x(), 10.0);
//     nh_private_.param(ns + "/Exp/maxY", 
//         up_bd_.y(), 10.0);
//     nh_private_.param(ns + "/Exp/maxZ", 
//         up_bd_.z(), 5.0);
//     nh_private_.param(ns + "/block_map/resolution", 
//         resolution_, 0.1);
//     nh_private_.param(ns + "/block_map/sensor_max_range", 
//         sensor_range_, 5.0);
//     nh_private_.param(ns + "/Frontier/sample_max_range", 
//         sample_max_range_, 4.5);
//     nh_private_.param(ns + "/Frontier/grid_scale", 
//         node_scale_(0), 5.0);
//     nh_private_.param(ns + "/Frontier/grid_scale", 
//         node_scale_(1), 5.0);
//     nh_private_.param(ns + "/Frontier/grid_scale", 
//         node_scale_(2), 5.0);
//     nh_private_.param(ns + "/Frontier/viewpoint_thresh", 
//         vp_thresh_, 2.0);
//     nh_private_.param(ns + "/Frontier/observe_thresh", 
//         obs_thresh_, 0.85);
//     nh_private_.param(ns + "/Frontier/resample_duration", 
//         resample_duration_, 1.0);
//     nh_private_.param(ns + "/Frontier/sample_hor_dir_num", 
//         samp_h_dir_num_, 10);
//     nh_private_.param(ns + "/Frontier/sample_ver_dir_num", 
//         samp_v_dir_num_, 3);
//     nh_private_.param(ns + "/Frontier/sample_dist_num", 
//         samp_dist_num_, 3);
//     nh_private_.param(ns + "/Frontier/sensor_type", 
//         sensor, string("Depth_Camera"));
//     nh_private_.param(ns + "/Frontier/FOV_hor_num", 
//         FOV_h_num_, 15);
//     nh_private_.param(ns + "/Frontier/FOV_ver_num", 
//         FOV_v_num_, 10); 
//     nh_private_.param(ns + "/Frontier/cam_hor", 
//         cam_hor_, 0.5 * M_PI);
//     nh_private_.param(ns + "/Frontier/cam_ver", 
//         cam_ver_, 0.5 * M_PI);    
//     nh_private_.param(ns + "/Frontier/livox_ver_low", 
//         livox_ver_low_, -10/180 * M_PI);
//     nh_private_.param(ns + "/Frontier/livox_ver_up", 
//         livox_ver_up_, 75/180 * M_PI);    
//     nh_private_.param(ns + "/Frontier/ray_samp_dist1", 
//         ray_samp_dist1_, 0.2);
//     // nh_private_.param(ns + "/Frontier/samp_free_thresh", 
//     //     samp_free_thresh_, 25);
//     nh_private_.param(ns + "/Frontier/ray_samp_dist2", 
//         ray_samp_dist2_, 0.1);    
//     nh_private_.param(ns + "/Frontier/show_frontier", 
//         show_frontier, true);
//     nh_private_.param(ns + "/Frontier/min_vp_num", 
//         min_vp_num_, 6);    
//     nh_private_.param(ns + "/Exp/robot_sizeX", 
//         Robot_size_(0), 0.5);    
//     nh_private_.param(ns + "/Exp/robot_sizeY", 
//         Robot_size_(1), 0.5);    
//     nh_private_.param(ns + "/Exp/robot_sizeZ", 
//         Robot_size_(2), 0.4);    
//     nh_private_.param(ns + "/Frontier/show_frontier", 
//         show_frontier, true);
        
//     for(int dim = 0; dim < 3; dim++){
//         node_scale_(dim) = ceil(node_scale_(dim) / resolution_) * resolution_;
//         node_num_(dim) = ceil((up_bd_(dim) - origin_(dim)) / node_scale_(dim));
//     }
//     scan_count_ = 0;
//     cout<<"node_scale_:"<<node_scale_<<endl;
//     cout<<"up_bd_:"<<up_bd_.transpose()<<endl;
//     cout<<"origin_:"<<origin_.transpose()<<endl;
//     cout<<"node_num_:"<<node_num_.transpose()<<endl;
    
//     // use_swarm_ = (SDM_->drone_num_ > 1 && !SDM_->is_ground_);
//     // cout<<"use_swarm_:"<<use_swarm_<<endl;
//     // cout<<"SDM_->drone_num_:"<<int(SDM_->drone_num_)<<endl;
//     // cout<<"SDM_->is_ground_:"<<SDM_->is_ground_<<endl;

//     sample_flag_ = false;

//     Eigen::Vector3i v_it;
//     samp_dir_num_ = samp_v_dir_num_ * samp_dist_num_;
//     samp_num_ = samp_h_dir_num_ * samp_v_dir_num_ * samp_dist_num_;
//     samp_h_dir_ = M_PI * 2 / samp_h_dir_num_;
//     for(v_it(2) = 0; v_it(2) < node_num_(2); v_it(2)++){
//         for(v_it(1) = 0; v_it(1) < node_num_(1); v_it(1)++){
//             for(v_it(0) = 0; v_it(0) < node_num_(0); v_it(0)++){
//                 EroiNode EN;
//                 Eigen::Vector3i vox_num;
//                 for(int dim = 0; dim < 3; dim++){
//                     vox_num(dim) = floor(node_scale_(dim) / resolution_);
//                     double remain_d = up_bd_(dim) - origin_(dim) - v_it(dim) * node_scale_(dim);
//                     EN.down_(dim) = v_it(dim) * node_scale_(dim) + origin_(dim);
//                     EN.up_(dim) = v_it(dim) * node_scale_(dim) + node_scale_(dim) + origin_(dim);
//                     EN.center_(dim) = EN.down_(dim) / 2 + EN.up_(dim) / 2;
//                     if(remain_d < node_scale_(dim)){
//                         EN.center_(dim) = remain_d * 0.5 + v_it(dim) * node_scale_(dim) + origin_(dim);
//                         EN.up_(dim) = remain_d + v_it(dim) * node_scale_(dim) + origin_(dim);
//                         vox_num(dim) = floor((EN.up_(dim) - EN.down_(dim)) / resolution_);
//                     }
//                     EN.last_sample_ = ros::WallTime::now().toSec();
//                     EN.last_strong_check_ = ros::WallTime::now().toSec();
//                 }
//                 EN.unknown_num_ = vox_num(0) * vox_num(1) * vox_num(2);
//                 EN.thresh_num_ = floor((1.0 - obs_thresh_) * EN.unknown_num_);
//                 EN.f_state_ = 0;
                
//                 EN.local_vps_.resize(samp_num_, 0);
//                 EN.flags_ = 0;
//                 EN.owner_ = 0;
//                 EROI_.emplace_back(EN);                
//             }
//         }
//     }
//     // SDM_->SetFrontierVpNum(samp_num_);

//     if(1){
//         sample_timer_ = nh.createTimer(ros::Duration(0.33), &EroiGrid::SampleVpsCallback, this);
//         lazy_samp_timer_ = nh.createTimer(ros::Duration(0.2), &EroiGrid::LazySampleCallback, this);
//     }
//     if(show_frontier)
//         show_timer_ = nh.createTimer(ros::Duration(0.5), &EroiGrid::ShowVpsCallback, this);
//     show_pub_ = nh.advertise<visualization_msgs::MarkerArray>(ns+"/Frontier/grid", 1);
//     debug_pub_ = nh.advertise<visualization_msgs::Marker>("/Frontier/debug", 1);
//     // //FOV down sample
//     if(sensor == "Depth_Camera"){
//         sensor_type_ = CAMERA;
//         ROS_WARN("use camera");
//     }
//     else if(sensor == "Livox"){
//         sensor_type_ = LIVOX;
//         ROS_WARN("use Livox");
//     }
//     else{
//         ROS_ERROR("error sensor type!");
//         ros::shutdown();
//         return;
//     }

//     for(int i = 0; i < samp_dist_num_; i++){
//         // sample_dists_.push_back((sample_max_range_ - sqrt(3) * node_scale_) / max(1, samp_dist_num_) * i + sqrt(3)/2 * node_scale_ - 1e-3);
//         sample_dists_.push_back((sample_max_range_ - sqrt(3)/2 * node_scale_(0)) / max(1, samp_dist_num_-1) * i + sqrt(3)/2 * node_scale_(0) - 1e-3);
//         cout<<i<<"samp d:"<<sample_dists_.back()<<endl;
//     }
//     for(int i = 0; i < samp_v_dir_num_; i++){
//         if(sensor_type_ == CAMERA){
//             sample_vdir_sins_.push_back(sin(M_PI * 0.25 / max(samp_v_dir_num_ - 1, 1) * i - M_PI * 0.22 / 2));
//             sample_vdir_coses_.push_back(cos(M_PI * 0.25 / max(samp_v_dir_num_ - 1, 1) * i - M_PI * 0.22 / 2));
//             sample_v_dirs_.push_back(M_PI * 0.25 / max(samp_v_dir_num_ - 1, 1) * i - M_PI * 0.22 / 2);
//         }
//         else if(sensor_type_ == LIVOX){
//             sample_vdir_sins_.push_back(sin(M_PI * 0.12 / max(samp_v_dir_num_ - 1, 1) * i - M_PI * 0.22 / 2));
//             sample_vdir_coses_.push_back(cos(M_PI * 0.12 / max(samp_v_dir_num_ - 1, 1) * i - M_PI * 0.22 / 2));
//             sample_v_dirs_.push_back(M_PI * 0.12 / max(samp_v_dir_num_ - 1, 1) * i - M_PI * 0.25 / 2);
//         }
//     }
//     for(int i = 0; i < samp_h_dir_num_; i++){
//         sample_hdir_sins_.push_back(sin(M_PI * 2.0 / max(samp_h_dir_num_, 1) * i - M_PI));
//         sample_hdir_coses_.push_back(cos(M_PI * 2.0 / max(samp_h_dir_num_, 1) * i - M_PI));
//         sample_h_dirs_.push_back(M_PI * 2.0 / max(samp_h_dir_num_, 1) * i - M_PI);
//     }
//     gain_rays_.resize(samp_num_);
//     gain_dirs_.resize(samp_num_);
//     Robot_pos_.setZero();
//     InitGainRays();
//     cout<<"samp_h_dir_num_:"<<samp_h_dir_num_<<endl;
//     cout<<"samp_v_dir_num_:"<<samp_v_dir_num_<<endl;
//     cout<<"samp_dist_num_:"<<samp_dist_num_<<endl;
//     cout<<"samp_num_:"<<samp_num_<<endl;
//     cout<<"sensor_type_:"<<sensor_type_<<endl;
// }

void EroiGrid::AlignInit(const ros::NodeHandle &nh, 
    const ros::NodeHandle &nh_private,
    Eigen::Vector3d &origin, Eigen::Vector3i &block_size, 
    Eigen::Vector3i &block_num, Eigen::Vector3i &local_block_num){

    nh_ = nh;
    nh_private_ = nh_private;
    std::string ns = ros::this_node::getName();
    string sensor;
    bool show_frontier;
    double geh;

    nh_private_.param(ns + "/block_map/resolution", 
        resolution_, 0.1);
    nh_private_.param(ns + "/block_map/sensor_max_range", 
        sensor_range_, 5.0);
    nh_private_.param(ns + "/Exp/minX", 
        exp_lowbd_.x(), -10.0);
    nh_private_.param(ns + "/Exp/minY", 
        exp_lowbd_.y(), -10.0);
    nh_private_.param(ns + "/Exp/minZ", 
        exp_lowbd_.z(), 0.0);
    nh_private_.param(ns + "/Exp/maxX", 
        exp_upbd_.x(), 10.0);
    nh_private_.param(ns + "/Exp/maxY", 
        exp_upbd_.y(), 10.0);
    nh_private_.param(ns + "/Exp/maxZ", 
        exp_upbd_.z(), 5.0);
    nh_private_.param(ns + "/Frontier/viewpoint_thresh", 
        vp_thresh_, 2.0);
    nh_private_.param(ns + "/Frontier/observe_thresh", 
        obs_thresh_, 0.85);
    nh_private_.param(ns + "/Frontier/resample_duration", 
        resample_duration_, 1.0);
    nh_private_.param(ns + "/Frontier/sensor_type", 
        sensor, string("Depth_Camera"));
    nh_private_.param(ns + "/Frontier/FOV_hor_num", 
        FOV_h_num_, 15);
    nh_private_.param(ns + "/Frontier/FOV_ver_num", 
        FOV_v_num_, 10); 
    nh_private_.param(ns + "/Frontier/cam_hor", 
        cam_hor_, 0.5 * M_PI);
    nh_private_.param(ns + "/Frontier/cam_ver", 
        cam_ver_, 0.5 * M_PI);    
    nh_private_.param(ns + "/Frontier/livox_ver_low", 
        livox_ver_low_, -10/180 * M_PI);
    nh_private_.param(ns + "/Frontier/livox_ver_up", 
        livox_ver_up_, 75/180 * M_PI);    
    nh_private_.param(ns + "/Frontier/ray_samp_dist1", 
        ray_samp_dist1_, 0.2);
    nh_private_.param(ns + "/Frontier/ray_samp_dist2", 
        ray_samp_dist2_, 0.1);    
    nh_private_.param(ns + "/Frontier/min_vp_num", 
        min_vp_num_, 6);    
    nh_private_.param(ns + "/Frontier/VpSampleRange", 
        vp_sample_range_, {3.0});
    nh_private_.param(ns + "/Frontier/VpSampleVerRes", 
        VpSampleVerRes_, 1.0);
    nh_private_.param(ns + "/Frontier/VpSampleHorRes", 
        VpSampleHorRes_, 2.0);
    nh_private_.param(ns + "/Frontier/VpSampleVerNum", 
        VpSampleVerNum_, 3);
    nh_private_.param(ns + "/Frontier/GainEnh", 
        geh, 1.2);
    nh_private_.param(ns + "/Frontier/show_frontier", 
        show_frontier, true);
    nh_private_.param(ns + "/Exp/robot_sizeX", 
        Robot_size_(0), 0.5);    
    nh_private_.param(ns + "/Exp/robot_sizeY", 
        Robot_size_(1), 0.5);    
    nh_private_.param(ns + "/Exp/robot_sizeZ", 
        Robot_size_(2), 0.4);    
    nh_private_.param(ns + "/opt/MaxVel", 
        v_max_, 1.0);    
    nh_private_.param(ns + "/opt/MaxAcc", 
        a_max_, 1.0);     

    origin_ = origin;
    node_num_ = block_num;
    node_scale_.x() = resolution_*block_size.x();
    node_scale_.y() = resolution_*block_size.y();
    node_scale_.z() = resolution_*block_size.z();    
    low_bd_ = origin_ + Eigen::Vector3d(1e-4, 1e-4, 1e-4);
    exp_lowbd_ = exp_lowbd_ + Eigen::Vector3d(1e-4, 1e-4, 1e-4);
    exp_upbd_ = exp_upbd_ - Eigen::Vector3d(1e-4, 1e-4, 1e-4);
    up_bd_ = origin_ + node_scale_.cwiseProduct(node_num_.cast<double>()) - Eigen::Vector3d(1e-4, 1e-4, 1e-4);
    for(int dim = 0; dim < 3; dim++){
        if(exp_lowbd_(dim) < low_bd_(dim) || exp_upbd_(dim) > up_bd_(dim)){
            StopageDebug("AlignInit exp_lowbd_ or exp_upbd_ error");
        }
    }
    

    scan_count_ = 0;
    sample_flag_ = false;
    vp_update_ = false;
    rough_sample_ = false;
    single_sample_ = false;

    if(1){
        sample_timer_ = nh.createTimer(ros::Duration(0.33), &EroiGrid::SampleVpsCallback, this);
        lazy_samp_timer_ = nh.createTimer(ros::Duration(0.2), &EroiGrid::LazySampleCallback, this);
        sample_single_timer_ = nh.createTimer(ros::Duration(0.2), &EroiGrid::SampleSingleVpsCallback, this);
    }
    if(show_frontier)
        show_timer_ = nh.createTimer(ros::Duration(0.5), &EroiGrid::ShowVpsCallback, this);
    show_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(ns+"/Frontier/grid", 1);
    debug_pub_ = nh_.advertise<visualization_msgs::Marker>("/Frontier/debug", 1);
    // //FOV down sample
    if(sensor == "Depth_Camera"){
        sensor_type_ = CAMERA;
        ROS_WARN("use camera");
    }
    else if(sensor == "Livox"){
        sensor_type_ = LIVOX;
        ROS_WARN("use Livox");
    }
    else{
        ROS_ERROR("error sensor type!");
        ros::shutdown();
        return;
    }


    Robot_pos_ = LRM_->node_scale_;

    samp_total_num_ = 0;
    for(int i = 0; i < vp_sample_range_.size(); i++){
        for(int h = 0; h < VpSampleVerNum_; h++){
            double z = VpSampleVerRes_*(h - (VpSampleVerNum_-1) * 0.5);
            int c_num = floor(vp_sample_range_[i]*2*M_PI / VpSampleHorRes_);
            // cout<<"c_num:"<<c_num<<endl;
            double dir = M_PI*2 / c_num;
            for(double ang = -M_PI; ang < M_PI; ang += dir + 1e-3){
                Eigen::Vector4d p(cos(ang) * vp_sample_range_[i] + 0.02, sin(ang) * vp_sample_range_[i] + 0.02, z + 0.02, ang + M_PI);
                if(p(3) > M_PI) p(3) -= 2*M_PI;
                vps_.emplace_back(p);
                samp_total_num_++;
            }
        }
    }
    // cout<<"vps[29]:"<<vps_[29].transpose()<<endl;
    // getchar();
    cout<<"samp_total_num_:"<<samp_total_num_<<endl;
    if(samp_total_num_ >= 256) {
        cout<<"samp_total_num_:"<<samp_total_num_<<endl;
        StopageDebug("AlignInit sample num too large");
    }

    // for(int dim = 0; dim < 3; dim++){
    //     sample_num_(dim) = floor(sample_BOX_(dim) / sample_res_(dim)) + 1;
    //     sample_BOX_(dim) = (sample_num_(dim) - 1) * sample_res_(dim);
    //     if(dim < 2 && sample_res_(dim) * (sample_num_(dim) - 1) < node_scale_(dim)){
    //         cout<<"sample_num_:"<<sample_num_.transpose()<<endl;
    //         cout<<"sample_res_:"<<sample_res_.transpose()<<endl;
    //         cout<<"node_scale_:"<<node_scale_.transpose()<<endl;
    //         StopageDebug("AlignInit sample_res_ too small");
    //     }
    // }
    // cout<<"sample_num_:"<<sample_num_.transpose()<<endl;
    // cout<<"sample_BOX_:"<<sample_BOX_.transpose()<<endl;
    // cout<<"sample_res_:"<<sample_res_.transpose()<<endl;

    // for(int x = 0; x < sample_num_(0); x++){
    //     for(int y = 0; y < sample_num_(1); y++){
    //         for(int z = 0; z < sample_num_(2); z++){   
    //             Eigen::Vector4d p(x*sample_res_(0) + 1e-3, y*sample_res_(1) + 1e-3, z*sample_res_(2) + 1e-3, 0.0);
    //             p.head(3) -= sample_BOX_*0.5;
    //             if(p.head(2).norm() < 0.1) continue;
    //             else{
    //                 p(3) = atan2(-p(1), -p(0));

    //                 vps_.emplace_back(p);
    //                 // cout<<"p:"<<p.transpose()<<endl;
    //                 samp_total_num_++;
    //             }
    //         }
    //     }
    // }
    // cout<<"samp_total_num_:"<<samp_total_num_<<endl;


    gain_rays_.resize(samp_total_num_);
    gain_dirs_.resize(samp_total_num_);

    Eigen::Vector3i v_it;
    for(v_it(2) = 0; v_it(2) < node_num_(2); v_it(2)++){
        for(v_it(1) = 0; v_it(1) < node_num_(1); v_it(1)++){
            for(v_it(0) = 0; v_it(0) < node_num_(0); v_it(0)++){
                EroiNode EN;
                Eigen::Vector3i vox_num;
                bool dead_eroi = false;
                for(int dim = 0; dim < 3; dim++){
                    vox_num(dim) = block_size(dim);
                    double remain_d = exp_upbd_(dim) - origin_(dim) - v_it(dim) * node_scale_(dim);
                    EN.down_(dim) = v_it(dim) * node_scale_(dim) + origin_(dim);


                    EN.up_(dim) = v_it(dim) * node_scale_(dim) + node_scale_(dim) + origin_(dim);
                    EN.center_(dim) = EN.down_(dim) / 2 + EN.up_(dim) / 2;
                    // if(EROI_.size() == 40756){
                    //     cout<<"EN.up_:"<<EN.up_(dim)<<endl;
                    //     cout<<"EN.down_:"<<EN.down_(dim)<<endl;
                    //     cout<<"EN.center_:"<<EN.center_(dim)<<endl;
                    // }
                    if(EN.up_(dim) < exp_lowbd_(dim)){
                        dead_eroi = true;
                        break;
                    }
                    else if(EN.down_(dim) > exp_upbd_(dim)){
                        dead_eroi = true;
                        break;
                    }
                    else{
                        double d, u;
                        d = max(EN.down_(dim), exp_lowbd_(dim));
                        u = min(EN.up_(dim), exp_upbd_(dim));
                        vox_num(dim) = floor((u - d) / resolution_);
                    }
                    EN.last_sample_ = ros::WallTime::now().toSec();
                    EN.last_strong_check_ = ros::WallTime::now().toSec();
                }
                // if(EN.down_(0) < 7.5 && EN.down_(1) < -0.75 && EN.down_(2) < 1.0 && EN.up_(0) > 7.5 && EN.up_(1) > -0.75 && EN.up_(2) > 1.0){
                //     cout<<"id:"<<v_it(0) + v_it(1)*node_num_(0) + v_it(2)*node_num_(1)*node_num_(0)<<endl;
                //     cout<<"center:"<<EN.center_.transpose()<<endl;
                //     cout<<"EROI_:"<<EROI_.size()<<endl;
                //     getchar();
                // }

                if(dead_eroi){
                    EN.f_state_ = 3;
                }
                else{
                    // if(123986 == EROI_.size()){
                    //     for(int dim = 0; dim < 3; dim++){
                    //         double d, u;
                    //         d = max(EN.down_(dim), exp_lowbd_(dim));
                    //         u = min(EN.up_(dim), exp_upbd_(dim));
                    //         cout<<"u:"<<u<<"  d:"<<d<<endl;
                    //     }
                    //     cout<<"vox_num:"<<vox_num.transpose()<<endl;
                    //     cout<<"thresh_num_:"<<EN.thresh_num_<<endl;

                    // }
                    EN.unknown_num_ = vox_num(0) * vox_num(1) * vox_num(2);
                    EN.thresh_num_ = floor((1.0 - obs_thresh_) * EN.unknown_num_);
                    EN.f_state_ = 0;
                    // if(EN.thresh_num_ < 0|| EN.unknown_num_ > block_size(0) * block_size(1) * block_size(2)){
                    //     cout<<"vox_num:"<<vox_num.transpose()<<endl;
                    //     ROS_ERROR("EN.thresh_num_ < 0");
                    //     getchar();
                    // }
                    
                    EN.local_vps_.resize(samp_total_num_, 0);
                    EN.flags_ = 0;
                    EN.owner_ = 0;
                }

                EROI_.emplace_back(EN);                
            }
        }
    }


    InitGainRays(geh);
    InitVpDict();
    InitMotionDict();
    ROS_WARN("eroi init finish");
    // getchar();
}

void EroiGrid::InitGainRays(double geh){
    Eigen::Vector3d f = Eigen::Vector3d::Zero();
    Eigen::Vector4d vp;
    double dtheta = cam_hor_ / FOV_h_num_;
    double dphi = cam_ver_ / FOV_v_num_;
    double cos_phi;
    double sin_2_dphi = sin(dphi / 2);

    for(int v_id = 0; v_id < samp_total_num_; v_id++){
        vp = vps_[v_id];
        // if(!GetVp(f, v_id, vp)){
        //     ROS_ERROR("InitGainRays0");
        //     ros::shutdown();
        // }
        if(sensor_type_ == SensorType::CAMERA){
            for(int h = 0; h < FOV_h_num_; h++){
                double h_dir = vp(3) + double(h + 0.5) / FOV_h_num_ * cam_hor_ - cam_hor_ / 2;
                for(int v = 0; v < FOV_v_num_; v++){
                    bool valid_ray = false;
                    double v_dir = double(v+0.5) / double(FOV_v_num_) * cam_ver_ - cam_ver_ / 2;
                    tr1::unordered_map<int, double> l1;
                    list<Eigen::Vector3d> ray1;
                    list<pair<Eigen::Vector3d, double>> ray2;
                    Eigen::Vector3d dir;
                    cos_phi = cos(v_dir);
                    dir(0) = cos(h_dir) * cos(v_dir);
                    dir(1) = sin(h_dir) * cos(v_dir);
                    dir(2) = sin(v_dir);
                    double l_gain = 0;
                    for(double l = 0.5 * ray_samp_dist1_; l < sensor_range_; l += ray_samp_dist1_){
                        Eigen::Vector3d p = dir * l;
                        Eigen::Vector3d pm = p + vp.head(3);
                        int id = BM_->PostoId(p);
                        if(abs(pm.x()) < node_scale_(0) / 2 && abs(pm.y()) < node_scale_(1) / 2
                             && abs(pm.z()) < node_scale_(2) / 2){
                            valid_ray = true;
                            l_gain = l + ray_samp_dist2_ * 0.5;
                            break;
                        }
                        if(l1.find(id) == l1.end()){
                            l1.insert(pair<int, double>{id, l});
                            ray1.push_back(pm);
                        }
                    }
                    tr1::unordered_map<int, double> l2;
                    for(; l_gain < sensor_range_; l_gain += ray_samp_dist2_){
                        Eigen::Vector3d p = dir * l_gain;
                        Eigen::Vector3d pm = p + vp.block(0, 0, 3, 1);
                        if(abs(pm.x()) > geh*node_scale_(0) / 2 || abs(pm.y()) > geh*node_scale_(1) / 2
                             || abs(pm.z()) > geh*node_scale_(2) / 2){
                            break;
                        }
                        int id = BM_->PostoId(pm);
                        if(l2.find(id) == l2.end()){
                            l2.insert(pair<int, double>{id, l_gain});
                            double gain = (2*pow(l_gain, 2)*ray_samp_dist2_ + 1.0/6*pow(ray_samp_dist2_, 3)) *dtheta*sin_2_dphi*cos_phi;
                            ray2.push_back({pm, gain});
                        }
                    }
                    if(valid_ray && ray2.size() > 0){
                        gain_rays_[v_id].push_back({ray1, ray2});
                        gain_dirs_[v_id].push_back({dir * l_gain + vp.block(0, 0, 3, 1), dtheta*sin_2_dphi*cos_phi});
                    }
                }
            }
        }
        else if(sensor_type_ == SensorType::LIVOX){
            dtheta = M_PI * 2 / FOV_h_num_;
            dphi = (livox_ver_up_ - livox_ver_low_) / FOV_v_num_;
            sin_2_dphi = sin(dphi / 2);
            for(int h = 0; h < FOV_h_num_; h++){
                double h_dir = vp(3) + double(h) / FOV_h_num_ * M_PI * 2 - M_PI;
                for(int v = 0; v < FOV_v_num_; v++){
                    bool valid_ray = false;
                    double v_dir = double(v) / FOV_v_num_ * (livox_ver_up_ - livox_ver_low_) + livox_ver_low_;
                    tr1::unordered_map<int, double> l1;
                    list<Eigen::Vector3d> ray1;
                    list<pair<Eigen::Vector3d, double>> ray2;
                    Eigen::Vector3d dir;
                    cos_phi = cos(v_dir);
                    dir(0) = cos(h_dir) * cos(v_dir);
                    dir(1) = sin(h_dir) * cos(v_dir);
                    dir(2) = sin(v_dir);
                    double l_gain = 0;
                    for(double l = ray_samp_dist1_*0.5; l < sensor_range_; l += ray_samp_dist1_){
                        Eigen::Vector3d p = dir * l;
                        Eigen::Vector3d pm = p + vp.block(0, 0, 3, 1);
                        int id = BM_->PostoId(p);
                        if(abs(pm.x()) < node_scale_(0) / 2 && abs(pm.y()) < node_scale_(1) / 2
                             && abs(pm.z()) < node_scale_(2) / 2){
                            valid_ray = true;
                            l_gain = l + ray_samp_dist2_ * 0.5;
                            break;
                        }
                        if(l1.find(id) == l1.end()){
                            l1.insert(pair<int, double>{id, l});
                            ray1.push_back(pm);
                        }
                    }
                    tr1::unordered_map<int, double> l2;
                    for(; l_gain < sensor_range_; l_gain += ray_samp_dist2_){
                        Eigen::Vector3d p = dir * l_gain;
                        Eigen::Vector3d pm = p + vp.block(0, 0, 3, 1);
                        if(abs(pm.x()) > geh*node_scale_(0) / 2 || abs(pm.y()) > geh*node_scale_(1) / 2
                             || abs(pm.z()) > geh*node_scale_(2) / 2){
                            break;
                        }
                        int id = BM_->PostoId(pm);
                        if(l2.find(id) == l2.end()){
                            l2.insert(pair<int, double>{id, l_gain});
                            double gain = (2*pow(l_gain, 2)*ray_samp_dist2_ + 1.0/6*pow(ray_samp_dist2_, 3)) *dtheta*sin_2_dphi*cos_phi;
                            ray2.push_back({pm, gain});
                        }
                    }
                    if(valid_ray && ray2.size() > 0){
                        gain_rays_[v_id].push_back({ray1, ray2});
                        gain_dirs_[v_id].push_back({dir * l_gain + vp.block(0, 0, 3, 1), dtheta*sin_2_dphi*cos_phi});
                    }
                }
            }
        }
        else{
            ROS_ERROR("InitGainRays1");
            ros::shutdown();
            return;
        }
    }
}

void EroiGrid::InitVpDict(){
    Eigen::Vector3i ps, pe;
    for(int dim = 0; dim < 3; dim++){
        ps(dim) = -ceil((vp_sample_range_[0] + 0.02)/ node_scale_(dim));
        pe(dim) = ceil((vp_sample_range_[0] + 0.02)/ node_scale_(dim));
    }
    cout<<"ps:"<<ps.transpose()<<endl;
    cout<<"pe:"<<pe.transpose()<<endl;
    int c = 0;
    vp_dict_.resize(LRM_->block_size_(0) * LRM_->block_size_(1) * LRM_->block_size_(2));
    for(int x = ps(0); x <= pe(0); x++){
        for(int y = ps(1); y <= pe(1); y++){
            for(int z = ps(2); z <= pe(2); z++){
                Eigen::Vector3d p(x*node_scale_(0), y*node_scale_(1), z*node_scale_(2));
                for(uint8_t vp_id = 0; vp_id < samp_total_num_; vp_id++){
                    Eigen::Vector4d vp = vps_[vp_id];
                    Eigen::Vector3d vp_pos = vp.head(3) + p;
                    if(vp_pos(0) <= -node_scale_(0)*0.5 || vp_pos(0) >= node_scale_(0)*0.5
                        || vp_pos(1) <= -node_scale_(1)*0.5 || vp_pos(1) >= node_scale_(1)*0.5
                        || vp_pos(2) <= -node_scale_(2)*0.5 || vp_pos(2) >= node_scale_(2)*0.5){
                        continue;
                    }
                    Eigen::Vector3d org = - 0.5*node_scale_;
                    int p_x = (vp_pos(0) - org(0)) / LRM_->node_scale_(0);
                    int p_y = (vp_pos(1) - org(1)) / LRM_->node_scale_(1);
                    int p_z = (vp_pos(2) - org(2)) / LRM_->node_scale_(2);
                    int id = p_x + p_y * LRM_->block_size_(0) + p_z * LRM_->block_size_(0) * LRM_->block_size_(1);

                    if(id < 0 || id >= vp_dict_.size()){
                        cout<<"id:"<<id<<endl;
                        cout<<"p_x:"<<p_x<<" p_y:"<<p_y<<" p_z:"<<p_z<<endl;
                        cout<<"p:"<<p.transpose()<<endl;
                        cout<<"vp:"<<vp.transpose()<<endl;
                        StopageDebug("InitVpDict id error");
                    }
                    vp_dict_[id].emplace_back(Eigen::Vector3i(x, y, z), vp_id);
                    c++;
                }
            }
        }
    }
    // cout<<"eroi c:"<<c<<endl;
}

void EroiGrid::InitMotionDict(){
    // vector<vector<vector<Eigen::Vector3d>>> motion_trajs_;
    // vector<vector<double>> motion_vel_;
    // vector<vector<pair<uint8_t, Eigen::Vector3i>>> motion_covered_vps_; //
    ros::Duration(1.5).sleep();
    cout<<"cam_hor_:"<<cam_hor_<<"   cam_ver_:"<<cam_ver_<<endl;
    Eigen::Vector4d vp1, vp2;
    Eigen::Vector3d diff, vel_ter, diff_norm, pm, p2, X, b, pi1, pi2;
    Eigen::Matrix3d R, M;
    Eigen::Vector3d inv_res, ray_iter, ns, half_res, min_p, max_p;
    list<Eigen::Vector3d> debug_pts;
    list<Eigen::Vector4d> debug_vps;
    min_p.setZero();
    max_p.setZero();
    for(int dim = 0; dim < 3; dim++) inv_res(dim) = 1.0 / LRM_->node_scale_(dim);
    ns = LRM_->node_scale_;
    half_res = ns * 0.5;
    int i1, i2;
    i1 = 0;
    motion_trajs_.resize(vps_.size());
    motion_vel_.resize(vps_.size());
    motion_length_.resize(vps_.size());
    motion_covered_vps_.resize(vps_.size());
    for(auto vp : vps_){
        vp1 = vp;
        // cout<<"i1:"<<i1<<endl;

        i2 = 0;
        for(auto vp_l : vp_dict_){
            for(auto i : vp_l){
                if(abs(i.first(0)) + abs(i.first(1)) + abs(i.first(2)) == 0) continue;
                auto v = vps_[i.second];
                vp2 = v;
                vp2.head(3) += i.first.cast<double>().cwiseProduct(node_scale_);

                if(abs(YawDiff(vp2(3), vp1(3))) > 0.5 * M_PI) continue;

                diff = vp2.head(3) - vp1.head(3);
                debug_vps.clear();
                debug_pts.clear();
                debug_vps.emplace_back(vp1);
                debug_vps.emplace_back(vp2);
                if(diff.norm() < 1.5) {
                    i2++;
                    // if(i1 == 1) ROS_WARN("to short");
                    continue;
                }

                R << cos(vp1(3)), -sin(vp1(3)), 0, sin(vp1(3)), cos(vp1(3)), 0, 0, 0, 1;
                if(!InsideFov(R, diff)) {
                    i2++;
                    // if(i1 == 1) {
                    //     Eigen::Vector3d p = R * diff;
                    //     p.normalize();

                    //     cout<<"vp1:"<<vp1.transpose()<<" vp2:"<<vp2.transpose()<<endl;
                    //     cout<<"R:"<<R<<endl;
                    //     cout<<"p:"<<p.transpose()<<endl;
                    //     cout<<"theta:"<<atan2(diff(1), diff(0))<<endl;
                    //     cout<<"diff:"<<diff.transpose()<<endl;
                    //     ROS_WARN("not in fov");
                    // }
                    continue;
                }
                vel_ter(0) = cos(vp2(3));
                vel_ter(1) = sin(vp2(3));
                vel_ter(2) = 0;
                if(vel_ter.dot(diff) < 0) {
                    // if(i1 == 1) ROS_WARN("larger than 0.5Pi");
                    continue;
                }

                diff_norm = diff.normalized();
                if(1 || diff_norm.dot(vel_ter) >= 0.99){

                    motion_vel_[i1].emplace_back(v_max_);
                    motion_covered_vps_[i1].emplace_back(i.second, i.first);
                    motion_length_[i1].emplace_back(diff.norm());
                    vector<Eigen::Vector3d> tj;
                    Eigen::Vector3d origin = -node_scale_ * 0.5;
                    RayCaster rc;
                    rc.setInput((vp1.head(3) - origin).cwiseProduct(inv_res), (vp2.head(3) - origin).cwiseProduct(inv_res));
                    while(rc.step(ray_iter)){
                        ray_iter = ray_iter.cwiseProduct(ns) + origin + half_res;
                        debug_pts.emplace_back(ray_iter);
                        tj.emplace_back(ray_iter);
                        if(!tj.empty() && (tj.back() - ray_iter).norm() > 0.71){
                            cout<<"tj.back():"<<tj.back().transpose()<<"  "<<ray_iter.transpose()<<endl;
                            StopageDebug("too long 1");
                        }
                        // cout<<"ray_iter0:"<<ray_iter.transpose()<<endl;
                        for(int dim = 0; dim < 3; dim++){
                            min_p(dim) = min(min_p(dim), ray_iter(dim));
                            max_p(dim) = max(max_p(dim), ray_iter(dim));
                        }
                    }
                    ray_iter = ray_iter.cwiseProduct(ns) + origin + half_res;
                    tj.emplace_back(ray_iter);
                    // if(!tj.empty() && (tj.back() - ray_iter).norm() > 0.71){
                    //     cout<<"tj.back():"<<tj.back().transpose()<<"  "<<ray_iter.transpose()<<endl;
                    //     StopageDebug("too long 1");
                    // }
                    // cout<<"ray_iter00:"<<ray_iter.transpose()<<endl;
                    debug_pts.emplace_back(ray_iter);
                    for(int dim = 0; dim < 3; dim++){
                        min_p(dim) = min(min_p(dim), ray_iter(dim));
                        max_p(dim) = max(max_p(dim), ray_iter(dim));
                    }
                    motion_trajs_[i1].emplace_back(tj);
                    if(motion_trajs_[i1].back().size() == 0){
                        StopageDebug("motion_trajs_[i1].back().size() == 0, 0");
                    }
                    // Debug(debug_pts);
                    // VisVps(debug_vps);
                    // getchar();
                }
                else{

                    p2 = vp2.head(3);
                    pm = vp1.head(3) + diff * 0.5;
                    M.row(0) = diff_norm.transpose();
                    M.row(1) = vel_ter.transpose();
                    M.row(2) = M.row(0).transpose().cross(M.row(1).transpose());
                    M.row(2).normalize();
                    b(0) = diff_norm.dot(pm);
                    b(1) = vel_ter.dot(p2);
                    b(2) = p2.dot(M.row(2).transpose());
                    X = M.inverse() * b;
                    double r = (p2 - X).norm();

                    vector<Eigen::Vector3d> tj;
                    double theta_comp = acos(diff.norm()*0.5/r);
                    double theta = (M_PI*0.5 - theta_comp) * 2;
                    R.col(0) = (p2 - X).normalized();
                    R.col(1) = vel_ter;
                    R.col(2) = R.col(0).cross(R.col(1));
                    pi1 = vp1.head(3);
                    if(theta > M_PI*0.5) continue;
                    // cout<<"r1:"<<(p2 - X).norm()<<"r2:"<<(vp1.head(3) - X).norm()<<endl;
                    // cout<<"X:"<<X.transpose()<<endl;
                    // cout<<"perpen?:"<<(p2 - X).dot(vel_ter)<<endl;
                    // cout<<"theta:"<<theta / M_PI * 180.0<<endl;
                    // cout<<"vp1:"<<vp1.transpose()<<endl;
                    // cout<<"vp2:"<<vp2.transpose()<<endl;
                    for(double psi = 0; psi < theta; psi += 0.025){

                        pi2(0) = cos(-theta + psi);
                        pi2(1) = sin(-theta + psi);
                        pi2(2) = 0;
                        pi2 = R * pi2 * r;
                        pi2 += X;
                        // cout<<"pi1:"<<pi1.transpose()<<endl;
                        // cout<<"pi2:"<<pi2.transpose()<<endl;
                        // getchar();
                        debug_pts.emplace_back(pi2);

                        Eigen::Vector3d origin = -node_scale_ * 0.5;
                        RayCaster rc;
                        rc.setInput((pi1.head(3) - origin).cwiseProduct(inv_res), (pi2.head(3) - origin).cwiseProduct(inv_res));
                        // if(i1 == 16 && motion_trajs_[i1].size() == 12 && tj.size() == 1){
                        //     cout<<(pi1.head(3) - origin).cwiseProduct(inv_res).transpose()<<"   -> "<<(pi2.head(3) - origin).cwiseProduct(inv_res).transpose()<<endl;
                        //     cout<<((pi1.head(3) - origin).cwiseProduct(inv_res).cwiseProduct(ns) + origin ).transpose()<<"   =-> "
                        //     <<((pi2.head(3) - origin).cwiseProduct(inv_res).cwiseProduct(ns) + origin ).transpose()<<endl;
                        //     cout<<rc.endX_<<"  "<<rc.endY_<<"  "<<rc.endZ_<<endl;
                        //     cout<<rc.x_<<"  "<<rc.y_<<"  "<<rc.z_<<endl;
                        // }
                        while(rc.step(ray_iter)){
                            ray_iter = ray_iter.cwiseProduct(ns) + origin + half_res;
                            // if(i1 == 16 && motion_trajs_[i1].size() == 12 && tj.size() == 1){
                            //     cout<<"it1:"<<ray_iter.transpose()<<endl;
                            // }
                            if(!tj.empty() && (tj.back() - ray_iter).norm() > 1e-3){
                                tj.emplace_back(ray_iter);
                                for(int dim = 0; dim < 3; dim++){
                                    min_p(dim) = min(min_p(dim), ray_iter(dim));
                                    max_p(dim) = max(max_p(dim), ray_iter(dim));
                                }
                            }
                            // if(!tj.empty() && (tj.back() - ray_iter).norm() > 0.71){
                            //     cout<<"tj.back():"<<tj.back().transpose()<<"  "<<ray_iter.transpose()<<endl;
                            //     StopageDebug("too long 1");
                            // }
                            if(tj.empty()) {
                                tj.emplace_back(ray_iter);
                                for(int dim = 0; dim < 3; dim++){
                                    min_p(dim) = min(min_p(dim), ray_iter(dim));
                                    max_p(dim) = max(max_p(dim), ray_iter(dim));
                                }
                            }
                        }
                        // if(i1 == 16 && motion_trajs_[i1].size() == 12 && tj.size() == 1){
                        //     cout<<"ituuu2:"<<ray_iter.transpose()<<endl;
                        // }
                        ray_iter = ray_iter.cwiseProduct(ns) + origin + half_res;
                        // if(i1 == 16 && motion_trajs_[i1].size() == 12 && tj.size() == 1){
                        //     cout<<"it2:"<<ray_iter.transpose()<<endl;
                        // }

                        if(!tj.empty() && (tj.back() - ray_iter).norm() > 1e-3){
                            tj.emplace_back(ray_iter);
                        }

                        // if(!tj.empty() && (tj.back() - ray_iter).norm() > 0.71){
                        //     cout<<"tj.back():"<<tj.back().transpose()<<"  "<<ray_iter.transpose()<<endl;
                        //     StopageDebug("too long 2");
                        // }
                        if(tj.empty()) {
                            tj.emplace_back(ray_iter);
                            for(int dim = 0; dim < 3; dim++){
                                min_p(dim) = min(min_p(dim), ray_iter(dim));
                                max_p(dim) = max(max_p(dim), ray_iter(dim));
                            }
                        }

                        pi1 = pi2;
                    }

                    // tj.emplace_back(ray_iter);
                    motion_trajs_[i1].emplace_back(tj);
                    // if(i1 == 16 && motion_trajs_[i1].size() == 13){
                    //     for(auto p : tj){
                    //         cout<<p.transpose()<<endl;
                    //     }
                    //     getchar();
                    // }
                    if(motion_trajs_[i1].back().size() == 0){
                        StopageDebug("motion_trajs_[i1].back().size() == 0, 1");
                    }
                    motion_length_[i1].emplace_back(theta * r);
                    motion_vel_[i1].emplace_back(min(sqrt(r*a_max_), v_max_));
                    // cout<<"vel:"<<min(sqrt(r*a_max_), v_max_)<<endl;
                    motion_covered_vps_[i1].emplace_back(i.second, i.first);

                    // Debug(debug_pts);
                    // VisVps(debug_vps);
                    // getchar();
                }
            }
        }
        // cout<<"i1:"<<i1<<" num:"<<motion_vel_[i1].size()<<endl;
        i1++;
    }

    // ROS_WARN("InitMotionDict1");

    Eigen::Vector3d d;
    d = min_p + node_scale_ * 0.5;
    Eigen::Vector3i di;
    // cout<<"min_p:"<<min_p.transpose()<<endl;
    // cout<<"max_p:"<<max_p.transpose()<<endl;
    for(int dim = 0; dim < 3; dim++){
        di(dim) = floor(d(dim) / ns(dim));
        motion_p_origin_(dim) = di(dim) * ns(dim) - node_scale_(0) * 0.5;
        d(dim) = max_p(dim) - motion_p_origin_(dim);
        di(dim) = ceil(d(dim) / ns(dim));
        motion_p_num_(dim) = di(dim);
    }
    // cout<<"min_p:"<<min_p.transpose()<<endl;
    // cout<<"max_p:"<<max_p.transpose()<<endl;
    // cout<<"motion_p_num_:"<<motion_p_num_.transpose()<<endl;
    // cout<<"motion_p_origin_:"<<motion_p_origin_.transpose()<<endl;

    // min_p = ns;
    Eigen::Vector3d z(0,0,0);
    for(i1 = 0; i1 < motion_trajs_.size(); i1++){
        for(i2 = 0; i2 < motion_trajs_[i1].size(); i2++){
            // if(i1 == 2 && i2 == 16){
            //     for(auto p : motion_trajs_[i1][i2]){
            //         cout<<"piii:"<<p.transpose()<<endl;
            //     }
            // }
            for(auto p : motion_trajs_[i1][i2]){
                int idx = GetMotionId(z, p);
                // if(idx == 3 && i1 == 2 && i2 == 16){
                //     cout<<"idx:"<<idx<<endl;
                //     cout<<"i1:"<<i1<<endl;
                //     cout<<"i2:"<<i2<<endl;
                //     int x = (p(0) - motion_p_origin_(0)) / LRM_->node_scale_(0);
                //     int y = (p(1) - motion_p_origin_(1)) / LRM_->node_scale_(1);
                //     int z = (p(2) - motion_p_origin_(2)) / LRM_->node_scale_(2);
                //     cout<<"x:"<<x<<endl;
                //     cout<<"y:"<<y<<endl;
                //     cout<<"z:"<<z<<endl;
                //     cout<<"p:"<<p.transpose()<<endl;
                // }
                auto md = motion_block_dict_.find(idx);
                if(md == motion_block_dict_.end()){
                    list<pair<int, int>> l;
                    l.push_back({i1, i2});
                    // if(i1 == 2) cout<<"i2:"<<i2<<endl;
                    motion_block_dict_.insert({idx, l});
                }
                else{
                    // if(i1 == 2) cout<<"i2:"<<i2<<endl;
                    md->second.emplace_back(i1, i2);

                }
                // if(idx == 3) {
                //     ROS_WARN("!!!");
                //     for(auto sss : motion_block_dict_[idx]){
                //         cout<<sss.first<<"  "<<sss.second<<endl;
                //     }
                // }
            }
        }
    }

    // for(auto &b: motion_block_dict_){
    //     Eigen::Vector3d c(0, 0, 0);
    //     ROS_WARN("debug-1");
    //     cout<<"b.first:"<<b.first<<endl;
    //     cout<<"b.second:"<<b.second.size()<<endl;

    //     Eigen::Vector3d p = MotionId2Pos(c, b.first);
    //     cout<<"p:"<<p.transpose()<<endl;

    //     ROS_WARN("debug0");

    //     // if(!LRM_->IsFeasible(p, true)){

    //     for(auto d : b.second){
    //         cout<<"d.first:"<<d.first<<"  d.second:"<<d.second<<endl;
    //         cout<<"motion_trajs_:"<<motion_trajs_.size()<<endl;
    //         cout<<"motion_trajs_[d.first]:"<<motion_trajs_[d.first].size()<<endl;
    //         for(auto pt : motion_trajs_[d.first][d.second])
    //             debug_pts.emplace_back(pt);
    //         Debug(debug_pts);
    //         Debug(p);
    //         debug_pts.clear();
    //         ROS_WARN("debug");
    //         getchar();
    //     }
    //     // }
    // }

    // ROS_WARN("InitMotionDict3");

}