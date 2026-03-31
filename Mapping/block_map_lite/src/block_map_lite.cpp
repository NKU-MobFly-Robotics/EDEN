#include <block_map_lite/block_map_lite.h>


// void BlockMapLite::InitSwarmBlock(vector<uint16_t> &id_l, vector<pair<Eigen::Vector3d, Eigen::Vector3d>> &bound){
//     SBS_.resize(id_l.size());
//     for(int i = 0; i < id_l.size(); i++){
//         SBS_[i].id_ = id_l[i];
//         SBS_[i].down_ = bound[i].second + Eigen::Vector3d::Ones() * 1e-3;
//         SBS_[i].up_ = bound[i].first - Eigen::Vector3d::Ones() * 1e-3;
//         SBS_[i].sub_num_ = 8;
//         SBS_[i].exploration_rate_.resize(8, 0);
//         SBS_[i].last_pub_rate_.resize(8, 0);
//         SBS_[i].to_pub_.resize(8, false);
//     }
// }

void BlockMapLite::OdomCallback(const nav_msgs::OdometryConstPtr &odom){
    have_odom_ = true;
    Eigen::Matrix4d body2world = Matrix4d::Identity();
    Eigen::Quaterniond rot;
    rot.x() = odom->pose.pose.orientation.x;
    rot.y() = odom->pose.pose.orientation.y;
    rot.z() = odom->pose.pose.orientation.z;
    rot.w() = odom->pose.pose.orientation.w;

    body2world(0, 3) = odom->pose.pose.position.x;
    body2world(1, 3) = odom->pose.pose.position.y;
    body2world(2, 3) = odom->pose.pose.position.z;

    body2world.block(0, 0, 3, 3) = rot.matrix();

    cam2world_ = body2world * cam2body_;
    // AwakeBlocks(cam2world_.block(0, 3, 3, 1), max_range_);
    // ResetLocalMap(cam2world_.block(0, 3, 3, 1));
    bline_ = false;
    for(double x = 0; x < 0.51; x += resolution_){
        for(double y = -0.3; y < 0.31; y += resolution_){
            for(double z = -0.3; z < 0.31; z += resolution_){
                Eigen::Vector3d pos = body2world.block(0, 0, 3, 3) * Vector3d(x, y, z) + body2world.block(0, 3, 3, 1);
                if(GetVoxState(pos) == occupied) {
                    bline_ = true;
                    break;
                }
            }
        }
    }

    last_odom_ = ros::Time::now().toSec();
}

void BlockMapLite::ResetLocalMap(const Eigen::Vector3d &p){
    Eigen::Vector3d new_origin;
    Eigen::Vector3i new_origin_idx, new_up_idx, it, diff;
    // new_origin = p - local_scale_ * 0.5;
    // for(int dim = 0; dim < 3; dim++) new_origin(dim) = min(max(new_origin(dim), map_lowbd_(dim) + 1e-3), map_upbd_(dim) - 1e-3);

    for(int dim = 0; dim < 3; dim++){
        if(p(dim) + 0.5*local_scale_(dim) >= map_upbd_(dim)) new_origin(dim) = map_upbd_(dim) - local_scale_(dim) + 1e-3;
        else if(p(dim) - 0.5*local_scale_(dim) <= map_lowbd_(dim)) new_origin(dim) = map_lowbd_(dim)+1e-3;
        else new_origin(dim) = p(dim) - local_scale_(dim) * 0.5;
    }

    if(!GetBlock3Id(new_origin, new_origin_idx)){
        ROS_ERROR("error ResetLocalMap");
        ros::shutdown();
        return;
    }
    new_origin = new_origin_idx.cast<double>().cwiseProduct(blockscale_) + origin_;
    new_up_idx = new_origin_idx + local_block_num_ - Eigen::Vector3i::Ones();

    diff = new_origin_idx - local_origin_idx_;
    if(diff.dot(diff) < 1e-3) return;
    map_count_ += 1.0;
    double t0 = ros::WallTime::now().toSec();

    // change blockids, out blockids and new blockids
    list<Eigen::Vector3i> out_idxs; // global idx3
    list<pair<int, Eigen::Vector3i>> new_idxs; // global idx, global idx3
    list<pair<int, Eigen::Vector3i>> change_idxs; // old local idx, global idx3
    for(int x = 0; x < local_block_num_(0); x++){
        for(int y = 0; y < local_block_num_(1); y++){
            for(int z = 0; z < local_block_num_(2); z++){
                it = Eigen::Vector3i(x, y, z) + local_origin_idx_;
                bool isc = true;
                for(int dim = 0; dim < 3; dim++){
                    if(it(dim) < new_origin_idx(dim) || it(dim) > new_up_idx(dim)){
                        isc = false;
                        break;
                    }
                }
                if(!isc) {
                    out_idxs.emplace_back(it);
                    it = new_up_idx - Eigen::Vector3i(x, y, z);
                    int lid = GetBlockId(it);
                    new_idxs.push_back({lid, it});

                }
                else {
                    int lid = GetLocalBlockId(it);
                    if(lid < 0 || lid >= GBS_local_.size()) { //debug
                        cout<<"di:"<<(it - local_origin_idx_).transpose()<<endl;
                        cout<<"it:"<<it.transpose()<<endl;

                        ROS_ERROR("ResetLocalMap() impossible GetLocalBlockId-1");
                        ros::shutdown();
                        continue;
                    }
                    change_idxs.push_back({lid, it});
                    // it = new_up_idx - Eigen::Vector3i(x, y, z);
                    // lid = GetBlockId(it);
                    // new_idxs.push_back({lid, it});
                }
            }
        }
    }

    // add out block to list
    // std::vector<std::thread> threads_w;
    // vector<int> oidx;
    for(auto &b : out_idxs){
        int id = GetBlockId(b);
        int lid = GetLocalBlockId(b);
        // cout<<"out lid:"<<lid<<endl;

        if(id < 0) { // out map, dont save
            continue;
        }
        // if(lid >= GBS_local_.size()) { //debug
        //     ROS_ERROR("ResetLocalMap() impossible GetLocalBlockId1 %d", lid);
        //     ros::shutdown();
        //     continue;
        // }
        // ROS_WARN("out_idxs");
        // cout<<"id:"<<id<<endl;
        // cout<<"bd:"<<(b - local_origin_idx_).transpose()<<endl;
        // cout<<"b:"<<(b).transpose()<<endl;
        SaveData(id, GBS_local_[lid]);
        // oidx.emplace_back(id);
        // threads_w.emplace_back(&BlockMapLite::SaveData, this, oidx.back(), std::ref(GBS_local_[lid]));
    }
    // for(auto& t : threads_w){
    //     t.join();
    // }

    // change idx
    Eigen::Vector3d lo = local_origin_; //debug
    Eigen::Vector3d lo_ub = local_upbd_; //debug
    Eigen::Vector3d lo_lb = local_lowbd_; //debug
    Eigen::Vector3i lo_oi = local_origin_idx_;
    Eigen::Vector3i lo_ui = local_up_idx_;

    local_origin_ = new_origin;
    local_origin_idx_ = new_origin_idx;
    local_up_idx_ = new_up_idx;
    local_upbd_ = local_origin_ + local_scale_ - Eigen::Vector3d::Ones() * 1e-3;
    local_lowbd_ = local_origin_ + Eigen::Vector3d::Ones() * 1e-3;

    list<pair<int, shared_ptr<Grid_Block>>> change_list; // new id, block
    for(auto &bid : change_idxs){
        int lid = GetLocalBlockId(bid.second);
        if(lid < 0 || lid >= GBS_local_.size()) { //debug
            
            cout<<"lo:"<<lo.transpose()<<endl;
            cout<<"lo_ub:"<<lo_ub.transpose()<<endl;
            cout<<"lo_lb:"<<lo_lb.transpose()<<endl;
            cout<<"lo_oi:"<<lo_oi.transpose()<<endl;
            cout<<"lo_ui:"<<lo_ui.transpose()<<endl;
            cout<<"lid:"<<lid<<endl;
            cout<<"di:"<<(bid.second - local_origin_idx_).transpose()<<endl;

            cout<<"new_up_idx:"<<new_up_idx.transpose()<<endl;
            cout<<"new_origin_idx:"<<new_origin_idx.transpose()<<endl;
            cout<<"local_origin_:"<<local_origin_.transpose()<<endl;
            cout<<"change_idxs:"<<change_idxs.size()<<endl;
            ROS_ERROR("ResetLocalMap() impossible GetLocalBlockId2");
            ros::shutdown();
            break;
        }
        change_list.push_back({lid, GBS_local_[bid.first]});
    }
    for(auto &ci : change_list){
        GBS_local_[ci.first] = ci.second;
    }

    // ROS_WARN("ResetLocalMap4");

    // cout<<"lo:"<<lo.transpose()<<endl;
    // cout<<"lo_ub:"<<lo_ub.transpose()<<endl;
    // cout<<"lo_lb:"<<lo_lb.transpose()<<endl;
    // cout<<"lo_oi:"<<lo_oi.transpose()<<endl;
    // cout<<"lo_ui:"<<lo_ui.transpose()<<endl;

    // cout<<"new_up_idx:"<<new_up_idx.transpose()<<endl;
    // cout<<"new_origin_idx:"<<new_origin_idx.transpose()<<endl;
    // cout<<"local_origin_:"<<local_origin_.transpose()<<endl;
    // cout<<"new_idxs:"<<new_idxs.size()<<endl;

    // read or initilize new blocks
    // std::vector<std::thread> threads_r;
    // vector<int> nidxs;
    for(auto &bid: new_idxs){
        int lid = GetLocalBlockId(bid.second);
        // if(lid < 0) { //debug
        //     ROS_ERROR("ResetLocalMap() impossible GetLocalBlockId3");
        //     ros::shutdown();
        //     continue;
        // }
        // cout<<"lid:"<<lid<<endl;
        // cout<<"dp:"<<(bid.second - local_origin_idx_).transpose()<<endl;
        // cout<<"bid.first:"<<bid.first<<endl;
        // cout<<"bid.second:"<<bid.second.transpose()<<endl;
        // if(lid < 0 || lid >= GBS_local_.size()){ //debug
        //     ROS_ERROR("ResetLocalMap() impossible lid");
        //     ros::shutdown();
        //     continue;
        // }
        // if(bid.first == 20000 || 19799 == bid.first){
        //     cout<<"lid:"<<lid<<endl;
        //     cout<<"bid.first:"<<bid.first<<endl;
        //     cout<<"bid.second:"<<bid.second.transpose()<<endl;
        //     cout<<"d:"<<bid.second.transpose() - local_origin_idx_.transpose()<<endl;
        // }
        
        ReadData(bid.first, GBS_local_[lid]);

        // nidxs.emplace_back(bid.first);
        // threads_r.emplace_back(&BlockMapLite::ReadData, this, nidxs.back(), std::ref(GBS_local_[lid]));
    }

    // for(auto& t : threads_r){
    //     t.join();
    // }
    total_map_t_ += ros::WallTime::now().toSec() - t0;
    cout<<"avg_t_:"<<total_map_t_ / map_count_<<endl;
    cout<<"t:"<<ros::WallTime::now().toSec() - t0<<endl;

    // ROS_WARN("end");
}

// void BlockMapLite::LoadRawMap(string occ_path, string free_path, int reset_type, bool reset){
//     if(reset){
//         changed_blocks_.clear();
//         for(auto &G : GBS_) {
//             if(reset_type == 0) G->Reset(-999.0);
//             else if(reset_type == 1) G->Reset(thr_min_ + 1e-3);
//             else G->Reset(thr_max_ - 1e-3);
//         }
//     }

//     int block_id, vox_id;
//     pcl::PointCloud<pcl::PointXYZ> cloud;
//     int status = pcl::io::loadPCDFile<pcl::PointXYZ>(occ_path, cloud);
//     if (status == -1) {
//         cout << "can't read occ file." << endl;
//     }
//     else{
//         cout << "loading occ file." << endl;
//         for(auto &pt : cloud){
//             Eigen::Vector3d p(pt.x, pt.y, pt.z);
//             if(GetLocalVox(block_id, vox_id, p)){
//                 GBS_[block_id]->odds_log_[vox_id] = thr_max_ - 1e-3;
//                 if(!GBS_[block_id]->show_){
//                     changed_blocks_.push_back(block_id);
//                     GBS_[block_id]->show_ = true;
//                 }
//             }
//         }
//     }
//     ROS_WARN("finish occ");
//     status = pcl::io::loadPCDFile<pcl::PointXYZ>(free_path, cloud);
//     if (status == -1) {
//         cout << "can't read free file." << endl;
//     }
//     else{
//         cout << "loading free file." << endl;
//         for(auto &pt : cloud){
//             Eigen::Vector3d p(pt.x, pt.y, pt.z);
//             if(GetLocalVox(block_id, vox_id, p)){
//                 GBS_[block_id]->odds_log_[vox_id] = thr_min_ + 1e-3;
//                 if(!GBS_[block_id]->show_){
//                     changed_blocks_.push_back(block_id);
//                     GBS_[block_id]->show_ = true;
//                 }
//             }
//         }
//     }
//     ROS_WARN("finish free");
// }

// void BlockMapLite::VisTotalMap(){
//     visualization_msgs::MarkerArray mka;
//     visualization_msgs::Marker mk_stand;
//     mk_stand.action = visualization_msgs::Marker::ADD;
//     mk_stand.pose.orientation.w = 1.0;
//     mk_stand.type = visualization_msgs::Marker::POINTS;
//     mk_stand.scale.x = resolution_;
//     mk_stand.scale.y = resolution_;
//     mk_stand.scale.z = resolution_;
//     mk_stand.color.a = 1.0;
//     mk_stand.header.frame_id = "world";
//     mk_stand.header.stamp = ros::Time::now();

//     DIR* dir;
//     struct dirent* ent;
//     std::string path = "/home/charliedog/rosprojects/DoomSea/map/2025_3_12_16_5_12_1"; // 指定目录路径
//     shared_ptr<Grid_Block> GB;
//     int bid;
//     if ((dir = opendir(path.c_str())) != nullptr) {
//         while ((ent = readdir(dir)) != nullptr) {
//             std::cout << ent->d_name << std::endl; // 输出文件名，不包含路径
//             bid = atoi(ent->d_name);
//             if(ReadDataRaw(bid, GB)){

//             }
//         }
//         closedir(dir);
//     } else {
//         // 错误处理
//         std::cerr << "Failed to open directory" << std::endl;
//     }


// }


// void BlockMapLite::InsertPCLCallback(const sensor_msgs::PointCloud2ConstPtr &pcl){
//     if(!bline_ && have_cam_param_ && ros::Time::now().toSec() - last_odom_ < 0.02 && ros::WallTime::now().toSec() - last_update_ > update_interval_){
//         last_update_ = ros::WallTime::now().toSec();

//         InsertPcl(pcl);
//     }
// }

// void BlockMapLite::InsertDepthCallback(const sensor_msgs::ImageConstPtr &img){
//     if(!bline_ && have_cam_param_ && ros::Time::now().toSec() - last_odom_ < 0.02 && ros::WallTime::now().toSec() - last_update_ > update_interval_){
//         last_update_ = ros::WallTime::now().toSec();

//         InsertImg(img);

//     }
// }

void BlockMapLite::CamParamCallback(const sensor_msgs::CameraInfoConstPtr &param){
    ROS_WARN("get param!");

    fx_ = param->K[0];
    cx_ = param->K[2];
    fy_ = param->K[4];
    cy_ = param->K[5];

    have_cam_param_ = true;
    u_max_ = param->width;
    v_max_ = param->height;

    downsample_size_ = (int)floor(resolution_ * fx_ / max_range_);
    downsample_size_ = min((int)floor(resolution_ * fy_ / max_range_), downsample_size_);
    

    u_down_max_ = (int)floor((u_max_) / downsample_size_);
    v_down_max_ = (int)floor((v_max_) / downsample_size_);
    u_max_ = u_down_max_ * downsample_size_;
    v_max_ = v_down_max_ * downsample_size_;

    double hl = abs(atan2((0.5 - cx_) / fx_, 1.0));
    double hr = abs(atan2((u_max_ + 0.5 - cx_) / fx_, 1.0));
    double vu = abs(atan2((0.5 - cy_) / fy_, 1.0));
    double vd = abs(atan2((v_max_ + 0.5 - cy_) / fy_, 1.0));
    FOV_.resize(4, 3);
    // FOV_<< -sin(hl), cos(hl), 0,
    //         -sin(hr), -cos(hr), 0,
    //         -sin(vu), 0, cos(vu), 
    //         -sin(vd), 0, -cos(vd);
    FOV_<<  -cos(hl), 0, -sin(hl),
            cos(hr), 0, -sin(hr),
             0, -cos(vu), -sin(vu),
            0, cos(vd), -sin(vd);
    cout<<"fx_:"<<fx_<<endl;
    cout<<"fy_:"<<fy_<<endl;
    cout<<"cx_:"<<cx_<<endl;
    cout<<"cy_:"<<cy_<<endl;
    cout<<"depth_step_:"<<depth_step_<<endl;
    cout<<"u_down_max_:"<<u_down_max_<<endl;
    cout<<"v_down_max_:"<<v_down_max_<<endl;
    cout<<"downsample_size_:"<<downsample_size_<<endl;
    camparam_sub_.shutdown();
}


void BlockMapLite::ShowMapCallback(const ros::TimerEvent &e){
    visualization_msgs::MarkerArray mks;
    visualization_msgs::Marker mk_stand;
    mk_stand.action = visualization_msgs::Marker::ADD;
    mk_stand.pose.orientation.w = 1.0;
    mk_stand.type = visualization_msgs::Marker::CUBE_LIST;
    mk_stand.scale.x = resolution_;
    mk_stand.scale.y = resolution_;
    mk_stand.scale.z = resolution_;
    mk_stand.color.a = 1.0;
    mk_stand.header.frame_id = "world";
    mk_stand.header.stamp = ros::Time::now();
    // mk_stand.id = *block_it;

    int lid;
    geometry_msgs::Point pt;

    for(vector<int>::iterator block_it = changed_blocks_.begin(); block_it != changed_blocks_.end(); block_it++){
        mks.markers.push_back(mk_stand);
        mks.markers.back().id = *block_it;
        lid = GetLocalBlockId(*block_it);
        if(lid < 0){
            mks.markers.back().action = visualization_msgs::Marker::DELETE;
            continue;
        }

        GBS_local_[lid]->show_ = 0;
        Eigen::Vector3d block_end = GBS_local_[lid]->origin_.cast<double>() * resolution_ + resolution_ * block_size_.cast<double>()
           + origin_;
        double x, y, z;
        int idx = 0;
        int debug_bk, debug_id;
        if(GBS_local_[lid]->state_ == MIXED){//debug
            for( z = resolution_ * (GBS_local_[lid]->origin_(2) + 0.5) + origin_(2); z < block_end(2); z += resolution_){
                for( y = resolution_ * (GBS_local_[lid]->origin_(1)  + 0.5) + origin_(1); y < block_end(1); y += resolution_){
                    for( x = resolution_ * (GBS_local_[lid]->origin_(0) + 0.5) + origin_(0); x < block_end(0); x += resolution_){

                        if(GBS_local_[lid]->odds_log_[idx] > 0){
                            pt.x = x;
                            pt.y = y; 
                            pt.z = z;
                            mks.markers.back().points.push_back(pt);
                            mks.markers.back().colors.push_back(Getcolor(z));
                        }

                        idx++;
                    }
                }
            }
        }
        if(mks.markers.back().points.size() == 0){
            mks.markers.back().action = visualization_msgs::Marker::DELETE;
        } 
    }

    mks.markers.push_back(mk_stand);
    mks.markers.back().type = visualization_msgs::Marker::CUBE;
    mks.markers.back().color.a = 0.07;
    mks.markers.back().color.g = 0.8;
    mks.markers.back().pose.position.x = (local_upbd_(0) + local_lowbd_(0))*0.5;
    mks.markers.back().pose.position.y = (local_upbd_(1) + local_lowbd_(1))*0.5;
    mks.markers.back().pose.position.z = (local_upbd_(2) + local_lowbd_(2))*0.5;
    mks.markers.back().scale.x = local_scale_(0);
    mks.markers.back().scale.y = local_scale_(1);
    mks.markers.back().scale.z = local_scale_(2);


    if(mks.markers.size() > 0) vox_pub_.publish(mks);
    changed_blocks_.clear();
}


// void BlockMapLite::MangageMapCallback(const ros::TimerEvent &e){
//     if(!have_odom_) return;
//     Eigen::Vector3d p = cam2world_.block(0, 3, 3, 1);
//     ResetLocalMap(p);
// }

// void BlockMapLite::SwarmMapCallback(const ros::TimerEvent &e){
//     if(SDM_->is_ground_){
//         while (!SDM_->swarm_sub_map_.empty()){
//             InsertSwarmPts(SDM_->swarm_sub_map_.front());
//             SDM_->swarm_sub_map_.pop_front();
//         }
        
//         if((SDM_->req_flag_ && vis_mode_) || double(SDM_->finish_num_) / SDM_->drone_num_ > SDM_->finish_thresh_ && !finish_flag_ && ros::WallTime::now(). toSec() - start_t_ > min_finish_t_){
//             finish_flag_ = true;

//             exp_comm_msgs::MapReqC mq;
//             Eigen::Vector3d up, down;
//             list<Eigen::Vector3d> pts;
//             for(int f_id = 0; f_id < SBS_.size(); f_id++){
//                 for(int i = 0; i < 8; i++){
//                     GetSBSBound(f_id, i, up, down);//debug

//                     if(SBS_[f_id].exploration_rate_[i] < swarm_pub_thresh_){
//                         mq.f_id.emplace_back(f_id);
//                         mq.block_id.emplace_back(i);
//                         pts.push_back((up + down) / 2);
//                     }
//                 }
//             }
//             for(int i = 0; i < 5; i++){
//                 cout<<"ground finish!!!===="<<endl;
//             }
//             Debug(pts);
//             mq.flag = 1;
//             if(vis_mode_ && SDM_->req_flag_) {
//                 SDM_->req_flag_ = false;
//                 mq.flag = 0;
//             }
//             SDM_->SetMapReq(mq);
//         }

//         if(SDM_->req_flag_ && !finish_flag_){
//             SDM_->req_flag_ = false;
//             exp_comm_msgs::MapReqC mq;
//             Eigen::Vector3d up, down;
//             for(int f_id = 0; f_id < SBS_.size(); f_id++){
//                 for(int i = 0; i < 8; i++){
//                     GetSBSBound(f_id, i, up, down);//debug

//                     if(SBS_[f_id].exploration_rate_[i] < swarm_pub_thresh_ && SBS_[f_id].exploration_rate_[i] > 5e-3){
//                         mq.f_id.emplace_back(f_id);
//                         mq.block_id.emplace_back(i);
//                     }
//                 }
//             }

//             mq.flag = 0; 
//             SDM_->SetMapReq(mq);
//         }
//         if(SDM_->statistic_ && stat_){
//             SDM_->CS_.SetVolume(stat_n_ * pow(resolution_, 3), 0);
//         }
//     }
//     else{
//         double cur_t = ros::WallTime::now().toSec();
//         if(SDM_->finish_list_[SDM_->self_id_ - 1] && !finish_flag_ && SDM_->req_flag_){
//             finish_flag_ = true;
//             list<Eigen::Vector3d> pts;
//             for(int i = 0; i < SDM_->mreq_.block_id.size(); i++){

//                 if(SBS_[SDM_->mreq_.f_id[i]].to_pub_[SDM_->mreq_.block_id[i]]) continue;
//                 SBS_[SDM_->mreq_.f_id[i]].to_pub_[SDM_->mreq_.block_id[i]] = true;
//                 double pub_t = cur_t - swarm_send_delay_ - 0.5;
//                 swarm_pub_block_.push_back({{SDM_->mreq_.f_id[i], SDM_->mreq_.block_id[i]}, pub_t});
//             }
//             for(int i = 0; i < 10; i++) ROS_ERROR("finishhhhhh");
//             SDM_->mreq_.block_id.clear();
//             SDM_->mreq_.f_id.clear();
//         }

//         if(SDM_->req_flag_ && !SDM_->finish_list_[SDM_->self_id_ - 1] && !finish_flag_){
//             SDM_->req_flag_ = false;
//             for(int i = 0; i < SDM_->mreq_.block_id.size(); i++){

//                 if(SBS_[SDM_->mreq_.f_id[i]].to_pub_[SDM_->mreq_.block_id[i]]) continue;
//                 SBS_[SDM_->mreq_.f_id[i]].to_pub_[SDM_->mreq_.block_id[i]] = true;
//                 double pub_t = cur_t;
//                 if(vis_mode_) pub_t -= swarm_send_delay_ + 0.5;
//                 swarm_pub_block_.push_back({{SDM_->mreq_.f_id[i], SDM_->mreq_.block_id[i]}, pub_t});
//             }
//         }

//         for(auto pub_it = swarm_pub_block_.begin(); pub_it != swarm_pub_block_.end(); pub_it++){
//             bool t_o_pub = false;
//             exp_comm_msgs::MapC msg;
//             if(pub_it->second + swarm_send_delay_ < cur_t) t_o_pub = true; // time out
//             Vox2Msg(msg, pub_it->first.first, pub_it->first.second);
//             if(((!finish_flag_ ||vis_mode_) && SBS_[msg.f_id].last_pub_rate_[msg.block_id] + 0.15 > SBS_[msg.f_id].exploration_rate_[msg.block_id]
//                 || finish_flag_ && SBS_[msg.f_id].last_pub_rate_[msg.block_id] + 0.01 > SBS_[msg.f_id].exploration_rate_[msg.block_id])
//                  && t_o_pub){
//                 SBS_[msg.f_id].to_pub_[msg.block_id] = false;
//                 auto erase_it = pub_it;
//                 pub_it--;
//                 swarm_pub_block_.erase(erase_it);
//                 continue;
//             }

//             if(msg.block_state != 0 && SBS_[msg.f_id].exploration_rate_[msg.block_id] > swarm_pub_thresh_ || t_o_pub){
//                 SDM_->SetMap(msg);
//                 SBS_[msg.f_id].last_pub_rate_[msg.block_id] = SBS_[msg.f_id].exploration_rate_[msg.block_id];
//                 SBS_[msg.f_id].to_pub_[msg.block_id] = false;
//                 auto erase_it = pub_it;
//                 pub_it--;
//                 swarm_pub_block_.erase(erase_it);
//             }

//         }

//     }
// }

void BlockMapLite::InsertPcl(const sensor_msgs::PointCloud2ConstPtr &pcl){
    vector<int> block_ids, vox_ids;
    vector<int>::iterator block_it, vox_it;

    Eigen::Vector3d end_point, dir, cam, ray_iter;
    Eigen::Vector3d half_res = Eigen::Vector3d(0.5, 0.5, 0.5) * resolution_;
    RayCaster rc;
    int block_id, vox_id;

    newly_register_idx_.clear();
    cam = cam2world_.block(0,3,3,1);

    if(InsideMap(cam) && InsideLocalMap(cam)){
        // LoadSwarmFilter();
        
        std::vector<int> indices;
        pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*pcl, *points);
        pcl::removeNaNFromPointCloud(*points, *points, indices);
        // cur_pcl_.clear();

        for(pcl::PointCloud<pcl::PointXYZ>::const_iterator pcl_it = points->begin(); pcl_it != points->end(); pcl_it++){
            bool occ;
            end_point(0) = pcl_it->x;
            end_point(1) = pcl_it->y;
            end_point(2) = pcl_it->z;

            dir = end_point - cam;
            occ = dir.norm() <= max_range_;
            if(!occ)
                end_point = cam + (end_point - cam).normalized() * max_range_;

            GetRayEndInsideLocalAndGlobalMap(cam, end_point, occ);
            if(!GetLocalVox(block_id, vox_id, end_point)) continue;
            if(occ){
                if(use_swarm_ && swarm_filter_dict_->find(PostoId(end_point)) != swarm_filter_dict_->end()) continue;
                // cur_pcl_.emplace_back(end_point);
                if(!(GBS_local_[block_id]->flags_[vox_id] & 2)){

                    GBS_local_[block_id]->flags_[vox_id] |= 2;

                    if(GBS_local_[block_id]->flags_[vox_id] & 1){
                    }
                    else{
                        GBS_local_[block_id]->flags_[vox_id] |= 1;
                        block_ids.push_back(block_id);
                        vox_ids.push_back(vox_id);
                    }
                }
                else{
                    continue;
                }
            }
            else{
                if((GBS_local_[block_id]->flags_[vox_id] & 1)){
                    continue;
                }
            }
            rc.setInput((end_point - origin_) / resolution_, (cam - origin_) / resolution_);
            
            while (rc.step(ray_iter))
            {
                ray_iter = (ray_iter) * resolution_ + origin_ + half_res;
                if(use_swarm_ && swarm_filter_dict_->find(PostoId(ray_iter)) != swarm_filter_dict_->end()) continue;
                if(GetLocalVox(block_id, vox_id, ray_iter)){

                    if((GBS_local_[block_id]->flags_[vox_id] & 1)){
                        continue;
                    }
                    else{
                        GBS_local_[block_id]->flags_[vox_id] |= 1;
                        block_ids.push_back(block_id);
                        vox_ids.push_back(vox_id);
                    }
                }
            }
        }
        Eigen::Vector3d p_it;
        uint64_t p_id;
        tr1::unordered_map<uint64_t,pair<uint8_t, uint16_t>>::iterator cp;
        for(block_it = block_ids.begin(), vox_it = vox_ids.begin(); block_it != block_ids.end(); block_it++, vox_it++){
            float odds_origin = GBS_local_[*block_it]->odds_log_[*vox_it];
            GBS_local_[*block_it]->state_ = MIXED;
            p_id = PostoId(p_it);
            if(GBS_local_[*block_it]->odds_log_[*vox_it] < thr_min_ - 1.0){
                p_it = Id2LocalPos(GBS_local_[*block_it], *vox_it);
                if(stat_){
                    if(p_it(0) > stat_lowbd_(0) && p_it(1) > stat_lowbd_(1) && p_it(2) > stat_lowbd_(2) &&
                    p_it(0) < stat_upbd_(0) && p_it(1) < stat_upbd_(1) && p_it(2) < stat_upbd_(2))
                    stat_n_++;
                }
                newly_register_idx_.push_back(p_it);
                odds_origin = 0.0;
                
                if(GBS_local_[*block_it]->flags_[*vox_it] & 2) changed_pts_.insert({p_id,{1, 0}});
                else changed_pts_.insert({p_id,{0, 0}});

                if(GBS_local_[*block_it]->flags_[*vox_it] & 2){
                    GBS_local_[*block_it]->odds_log_[*vox_it] = min(odds_origin + pro_hit_, thr_max_);
                }
                else{
                    p_it = Id2LocalPos(GBS_local_[*block_it], *vox_it);
                    if((p_it - cam).norm() > bline_occ_range_ || GBS_local_[*block_it]->odds_log_[*vox_it] < 0)
                    GBS_local_[*block_it]->odds_log_[*vox_it] = max(odds_origin + pro_miss_, thr_min_);
                }
            }
            else{
                if(GBS_local_[*block_it]->flags_[*vox_it] & 2){
                    GBS_local_[*block_it]->odds_log_[*vox_it] = min(odds_origin + pro_hit_, thr_max_);

                }
                else{
                    p_it = Id2LocalPos(GBS_local_[*block_it], *vox_it);
                    if((p_it - cam).norm() > bline_occ_range_ || GBS_local_[*block_it]->odds_log_[*vox_it] < 0)
                    GBS_local_[*block_it]->odds_log_[*vox_it] = max(odds_origin + pro_miss_, thr_min_);
                }
                
                if(abs(GBS_local_[*block_it]->odds_log_[*vox_it]) < 1e-5) GBS_local_[*block_it]->odds_log_[*vox_it] = 1e-5;

                if(odds_origin * GBS_local_[*block_it]->odds_log_[*vox_it] < 0){
                    cp = changed_pts_.find(p_id);
                    if(cp == changed_pts_.end()){
                        if(odds_origin > 0)  changed_pts_.insert({p_id, {3, 1}});
                        else changed_pts_.insert({p_id, {2, 1}});
                    }
                    else{
                        cp->second.second++;
                    }
                }
            }
            GBS_local_[*block_it]->flags_[*vox_it] = 0;
            if(!GBS_local_[*block_it]->show_ && show_block_){
                changed_blocks_.push_back(GBS_local_[*block_it]->id_);
                GBS_local_[*block_it]->show_ = true;
            }
        }
    }
}

void BlockMapLite::InsertImg(const sensor_msgs::ImageConstPtr &depth){
    if(!have_cam_param_) return;
    vector<int> block_ids, vox_ids;
    vector<int>::iterator block_it, vox_it;
    Eigen::Vector3d cam;
    RayCaster rc;

    newly_register_idx_.clear();

    cam = cam2world_.block(0,3,3,1);

    if(InsideMap(cam) && InsideLocalMap(cam)){
        // LoadSwarmFilter();
        double pix_depth;
        int downsamp_u, downsamp_v, downsamp_nex_u, downsamp_nex_v, downsamp_idx;
        int block_id, vox_id;
        Eigen::Vector3i  end3i;
        Eigen::Vector3d end_point, dir, cam, ray_iter;
        Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
        Eigen::Vector3d half_res = Eigen::Vector3d(0.5, 0.5, 0.5) * resolution_;

        // cur_pcl_.clear();
        cam = cam2world_.block(0, 3, 3, 1);

        uint16_t* row_ptr;
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(depth, depth->encoding);
        if(depth->encoding != sensor_msgs::image_encodings::TYPE_16UC1){
            (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, 1000.0);
        }
        tr1::unordered_map<int, Eigen::Vector3d> cast_pts;
        int k_l = ceil(max_range_ * 10.0 / resolution_); 
        for(int v = 0; v < v_max_; v += depth_step_){
            row_ptr = cv_ptr->image.ptr<uint16_t>(v);
            for(int u = 0; u < u_max_; u += depth_step_, row_ptr += depth_step_){

                pix_depth = (*row_ptr) / 1000.0;
                if(pix_depth == 0 && bline_) continue;

                if(pix_depth == 0)
                    pix_depth = max_range_ + 0.1;

                downsamp_u = floor(u / downsample_size_);
                downsamp_v = floor(v / downsample_size_);
                downsamp_idx = downsamp_u + downsamp_v * u_down_max_;

                end_point.x() = ((u + 0.5)  - cx_) * pix_depth / fx_;
                end_point.y() =  ((v + 0.5)  - cy_) * pix_depth / fy_;
                end_point.z() = pix_depth;
                end_point = cam2world_.block(0, 0, 3, 3) * end_point + cam;
                int key = floor((end_point(2) - cam(2) - max_range_ * 2)/resolution_)*k_l*k_l + 
                    floor((end_point(1)- cam(1) - max_range_ * 2)/resolution_)*k_l + 
                    floor((end_point(0)- cam(0) - max_range_ * 2)/resolution_);

                if(cast_pts.find(key) == cast_pts.end()) cast_pts.insert({key, end_point});

            }
        }

        for(auto &k_p : cast_pts){
                bool occ;
                end_point = k_p.second;
                occ = (k_p.second - cam).norm() < max_range_;
                if(!occ)
                    end_point = cam + (end_point - cam).normalized() * max_range_;
                // else{
                //     if(!use_swarm_ || (swarm_filter_dict_->find(PostoId(end_point)) == swarm_filter_dict_->end()))
                //         cur_pcl_.emplace_back(end_point);
                // }

                end_point = (PostoId3(end_point).cast<double>() + half) * resolution_ + origin_;

                GetRayEndInsideLocalAndGlobalMap(cam, end_point, occ);

                if(!GetLocalVox(block_id, vox_id, end_point)) continue;

                if(occ){
                    if(use_swarm_ && swarm_filter_dict_->find(PostoId(end_point)) != swarm_filter_dict_->end()) continue;

                    if(!(GBS_local_[block_id]->flags_[vox_id] & 2)){

                        GBS_local_[block_id]->flags_[vox_id] |= 2;

                        if(GBS_local_[block_id]->flags_[vox_id] & 1){
                        }
                        else{
                            GBS_local_[block_id]->flags_[vox_id] |= 1;
                            block_ids.push_back(block_id);
                            vox_ids.push_back(vox_id);
                        }
                    }
                    else{
                        continue;
                    }
                }


                rc.setInput((end_point - origin_) / resolution_, (cam - origin_) / resolution_);
                while (rc.step(ray_iter))
                {
                    ray_iter = (ray_iter) * resolution_ + origin_ + half_res;
                    if(use_swarm_ && swarm_filter_dict_->find(PostoId(ray_iter)) != swarm_filter_dict_->end()) continue;
                    if(GetLocalVox(block_id, vox_id, ray_iter)){

                        if((GBS_local_[block_id]->flags_[vox_id] & 1)){
                            continue;
                        }
                        else{
                            GBS_local_[block_id]->flags_[vox_id] |= 1;
                            block_ids.push_back(block_id);
                            vox_ids.push_back(vox_id);
                        }
                    }
                }
        }

        Eigen::Vector3d p_it;
        uint64_t p_id;
        tr1::unordered_map<uint64_t,pair<uint8_t, uint16_t>>::iterator cp;
        int c = 0;
        for(block_it = block_ids.begin(), vox_it = vox_ids.begin(); block_it != block_ids.end(); block_it++, vox_it++, c++){
            float odds_origin = GBS_local_[*block_it]->odds_log_[*vox_it];
            GBS_local_[*block_it]->state_ = MIXED;
            p_it = Id2LocalPos(GBS_local_[*block_it], *vox_it);
            p_id = PostoId(p_it);

            if(GBS_local_[*block_it]->odds_log_[*vox_it] < thr_min_ - 1.0){
                if(stat_){
                    if(p_it(0) > stat_lowbd_(0) && p_it(1) > stat_lowbd_(1) && p_it(2) > stat_lowbd_(2) &&
                    p_it(0) < stat_upbd_(0) && p_it(1) < stat_upbd_(1) && p_it(2) < stat_upbd_(2))
                    stat_n_++;
                }
                newly_register_idx_.push_back(p_it);
                odds_origin = 0.0;
                
                if(GBS_local_[*block_it]->flags_[*vox_it] & 2) changed_pts_.insert({p_id,{1, 0}});
                else changed_pts_.insert({p_id,{0, 0}});

                if(GBS_local_[*block_it]->flags_[*vox_it] & 2){
                    GBS_local_[*block_it]->odds_log_[*vox_it] = min(odds_origin + pro_hit_, thr_max_);
                }
                else{
                    if((p_it - cam).norm() > bline_occ_range_ || GBS_local_[*block_it]->odds_log_[*vox_it] < 0)
                    GBS_local_[*block_it]->odds_log_[*vox_it] = max(odds_origin + pro_miss_, thr_min_);
                }
            }
            else{
                if(GBS_local_[*block_it]->flags_[*vox_it] & 2){
                    GBS_local_[*block_it]->odds_log_[*vox_it] = min(odds_origin + pro_hit_, thr_max_);

                }
                else{
                    if((p_it - cam).norm() > bline_occ_range_ || GBS_local_[*block_it]->odds_log_[*vox_it] < 0)
                    GBS_local_[*block_it]->odds_log_[*vox_it] = max(odds_origin + pro_miss_, thr_min_);
                }
                
                if(abs(GBS_local_[*block_it]->odds_log_[*vox_it]) < 1e-5) GBS_local_[*block_it]->odds_log_[*vox_it] = 1e-5;

                if(odds_origin * GBS_local_[*block_it]->odds_log_[*vox_it] < 0){
                    cp = changed_pts_.find(p_id);
                    if(cp == changed_pts_.end()){
                        if(odds_origin > 0)  changed_pts_.insert({p_id, {3, 1}});
                        else changed_pts_.insert({p_id, {2, 1}});
                    }
                    else{
                        cp->second.second++;
                    }
                }
            }
            GBS_local_[*block_it]->flags_[*vox_it] = 0;
            if(!GBS_local_[*block_it]->show_ && show_block_){
                changed_blocks_.push_back(GBS_local_[*block_it]->id_);
                GBS_local_[*block_it]->show_ = true;
            }
        }
    }
}

void BlockMapLite::CastInsertImg(const sensor_msgs::ImageConstPtr &depth){
    if(!have_cam_param_) return;
    vector<int> block_ids, vox_ids;
    vector<int>::iterator block_it, vox_it;

    Eigen::Vector3i upi, downi, cam3i, it;
    Eigen::Matrix3d R = cam2world_.block(0, 0, 3, 3);
    Eigen::Matrix3d Rt = R.transpose();
    Eigen::Vector3d trans = cam2world_.block(0,3,3,1), pi;
    vector<vector<float>> img_d;
    img_d.resize(v_max_);
    int block_id, vox_id;
    // // RayCaster rc;

    
    cam3i = PostoId3(trans);
    upi = cam3i;
    downi = cam3i;
    newly_register_idx_.clear();
    if(InsideMap(cam3i)){
        double pix_depth, dm;
        int downsamp_u, downsamp_v, downsamp_nex_u, downsamp_nex_v, downsamp_idx;
        Eigen::Vector3d end_point, dir, ray_iter;
        Eigen::Vector3d cam = cam2world_.block(0, 3, 3, 1);

        bool occ;

        uint16_t* row_ptr;
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(depth, depth->encoding);
        if(depth->encoding != sensor_msgs::image_encodings::TYPE_16UC1){
            (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, 1000.0);
        }
        for(int v = 0; v < v_max_; v++){
            row_ptr = cv_ptr->image.ptr<uint16_t>(v);
            for(int u = 0; u < u_max_; u++, row_ptr ++){

                
                pix_depth = (*row_ptr) / 1000.0;
                if(pix_depth == 0 && bline_) {
                    img_d[v].emplace_back(1e-3);
                    continue;
                }

                if(pix_depth == 0){
                    pix_depth = max_range_;
                    occ = false;
                }
                else {
                    occ = true;
                }


                downsamp_u = floor(u / downsample_size_);
                downsamp_v = floor(v / downsample_size_);
                downsamp_idx = downsamp_u + downsamp_v * u_down_max_;
                end_point.x() = ((u + 0.5)  - cx_) * pix_depth / fx_;
                end_point.y() =  ((v + 0.5)  - cy_) * pix_depth / fy_;
                end_point.z() = pix_depth;
                dm = pix_depth / end_point.norm() * max_range_;
                img_d[v].emplace_back(min(pix_depth, dm));
                

                if(end_point.norm() > max_range_){
                    occ = false;
                }
                end_point = R * end_point + trans;
                
                if(InsideLocalMap(end_point)){
                    // vox_id = GetVoxId(end_point);
                    GetLocalVox(block_id, vox_id, end_point);
                    if(occ && !(GBS_local_[block_id]->flags_[vox_id] & 2) /*!(VX_flags_[vox_id] & 2)*/){
                        GBS_local_[block_id]->flags_[vox_id] = 3;
                        vox_ids.emplace_back(vox_id);
                        block_ids.emplace_back(block_id);
                        it = PostoId3(end_point);
                        // if(it(2) < 0) {
                        //     ROS_ERROR("< 0 0");
                        //     getchar();
                        // }
                    }
                    else if(!occ){
                        it = PostoId3(end_point);
                        // if(it(2) < 0) {
                        //     ROS_ERROR("< 0 1");
                        //     getchar();
                        // }
                    }
                    else{
                        continue;
                    }
                }
                else{
                    it = Pos2Id3Inside(end_point);
                    // if(it(2) < 0) {
                    //     ROS_ERROR("< 0 2 %d", it(2));
                    //     getchar();
                    // }
                }

                for(int dim = 0; dim < 3; dim++){
                    upi(dim) = max(it(dim), upi(dim));
                    downi(dim) = min(it(dim), downi(dim));
                }

            }
        }

        vector<Eigen::Vector3d> pts;
        int u, v, xui, xdi;
        double xu, xd;
        double depit, depim;
        Eigen::Vector3d p_it;
        p_it(2) = origin_(2) + (downi(2) + 0.5) * resolution_;
        for(it(2) = downi(2); it(2) <= upi(2); it(2)++, p_it(2) += resolution_){
            p_it(1) = origin_(1) + (downi(1) + 0.5) * resolution_;
            for(it(1) = downi(1); it(1) <= upi(1); it(1)++, p_it(1) += resolution_){

                if(GetXLineIntersec(R, trans, p_it(1), p_it(2), xd, xu)){
                    xdi = floor((xd - origin_(0)) / resolution_);
                    xui = floor((xu - origin_(0)) / resolution_);
                }
                else{
                    continue;
                }

                p_it(0) = origin_(0) + (xdi + 0.5) * resolution_;
                for(it(0) = xdi; it(0) <= xui; it(0)++, p_it(0) += resolution_){

                    if(!Cast2Img(Rt, trans, p_it, u, v, depit)) continue;


                    depim = img_d[v][u];

                    if(depim < depit){
                        continue;
                    }
                    if(!GetLocalVox(block_id, vox_id, p_it)){
                        continue;
                    }


                    if(!(GBS_local_[block_id]->flags_[vox_id] & 1)){
                        GBS_local_[block_id]->flags_[vox_id] |= 1;
                        block_ids.emplace_back(block_id);
                        vox_ids.emplace_back(vox_id);
                        // pts.emplace_back(p_it);
                    }
                    else{
                        continue;
                    }

                }

            }
        }
        // ROS_WARN("CastImg2");

        uint64_t p_id;
        tr1::unordered_map<uint64_t,pair<uint8_t, uint16_t>>::iterator cp;
        int c = 0;
        for(block_it = block_ids.begin(), vox_it = vox_ids.begin(); block_it != block_ids.end(); block_it++, vox_it++, c++){
            float odds_origin = GBS_local_[*block_it]->odds_log_[*vox_it];
            GBS_local_[*block_it]->state_ = MIXED;
            p_it = Id2LocalPos(GBS_local_[*block_it], *vox_it);
            p_id = PostoId(p_it);

            if(GBS_local_[*block_it]->odds_log_[*vox_it] < thr_min_ - 1.0){
                if(stat_){
                    if(p_it(0) > stat_lowbd_(0) && p_it(1) > stat_lowbd_(1) && p_it(2) > stat_lowbd_(2) &&
                    p_it(0) < stat_upbd_(0) && p_it(1) < stat_upbd_(1) && p_it(2) < stat_upbd_(2))
                    stat_n_++;
                }
                newly_register_idx_.push_back(p_it);
                odds_origin = 0.0;
                
                if(GBS_local_[*block_it]->flags_[*vox_it] & 2) changed_pts_.insert({p_id,{1, 0}});
                else changed_pts_.insert({p_id,{0, 0}});

                if(GBS_local_[*block_it]->flags_[*vox_it] & 2){
                    GBS_local_[*block_it]->odds_log_[*vox_it] = min(odds_origin + pro_hit_, thr_max_);
                }
                else{
                    if((p_it - cam).norm() > bline_occ_range_ || GBS_local_[*block_it]->odds_log_[*vox_it] < 0)
                    GBS_local_[*block_it]->odds_log_[*vox_it] = max(odds_origin + pro_miss_, thr_min_);
                }
            }
            else{
                if(GBS_local_[*block_it]->flags_[*vox_it] & 2){
                    GBS_local_[*block_it]->odds_log_[*vox_it] = min(odds_origin + pro_hit_, thr_max_);

                }
                else{
                    if((p_it - cam).norm() > bline_occ_range_ || GBS_local_[*block_it]->odds_log_[*vox_it] < 0)
                    GBS_local_[*block_it]->odds_log_[*vox_it] = max(odds_origin + pro_miss_, thr_min_);
                }
                
                if(abs(GBS_local_[*block_it]->odds_log_[*vox_it]) < 1e-5) GBS_local_[*block_it]->odds_log_[*vox_it] = 1e-5;

                if(odds_origin * GBS_local_[*block_it]->odds_log_[*vox_it] < 0){
                    cp = changed_pts_.find(p_id);
                    if(cp == changed_pts_.end()){
                        if(odds_origin > 0)  changed_pts_.insert({p_id, {3, 1}});
                        else changed_pts_.insert({p_id, {2, 1}});
                    }
                    else{
                        cp->second.second++;
                    }
                }
            }
            GBS_local_[*block_it]->flags_[*vox_it] = 0;
            if(!GBS_local_[*block_it]->show_ && show_block_){
                changed_blocks_.push_back(GBS_local_[*block_it]->id_);
                GBS_local_[*block_it]->show_ = true;
            }
        }

    }
}
// void BlockMapLite::AwakeBlocks(const Eigen::Vector3d &center, const double &range){
//     Eigen::Vector3d upbd, lowbd;
//     Eigen::Vector3i upbd_id3, lowbd_id3; 
//     int idx;
//     upbd.x() = min(range + center(0) + resolution_, map_upbd_.x());
//     upbd.y() = min(range + center(1) + resolution_, map_upbd_.y());
//     upbd.z() = min(range + center(2) + resolution_, map_upbd_.z());

//     lowbd.x() = max(center(0) - range - resolution_, map_lowbd_.x());
//     lowbd.y() = max(center(1) - range - resolution_, map_lowbd_.y());
//     lowbd.z() = max(center(2) - range - resolution_, map_lowbd_.z());

//     upbd_id3.x() = floor((upbd.x() - origin_(0)) / blockscale_.x());
//     upbd_id3.y() = floor((upbd.y() - origin_(1)) / blockscale_.y());
//     upbd_id3.z() = floor((upbd.z() - origin_(2)) / blockscale_.z());
//     lowbd_id3.x() = floor((lowbd.x() - origin_(0)) / blockscale_.x());
//     lowbd_id3.y() = floor((lowbd.y() - origin_(1)) / blockscale_.y());
//     lowbd_id3.z() = floor((lowbd.z() - origin_(2)) / blockscale_.z());

//     for(int x = lowbd_id3.x(); x <= upbd_id3.x(); x++){
//         for(int y = lowbd_id3.y(); y <= upbd_id3.y(); y++){
//             for(int z = lowbd_id3.z(); z <= upbd_id3.z(); z++){
//                 idx = x + y * block_num_(0) + z * block_num_(0) * block_num_(1);
//                 // GBS_[idx]->Awake((float)thr_max_, (float)thr_min_);
//             }
//         }
//     }
// }

// void BlockMapLite::InsertSwarmPts(exp_comm_msgs::MapC &msg){
//     if(msg.f_id < 0 || msg.f_id >= SBS_.size()) return;
//     if(msg.block_id < 0 || msg.block_id >= 8) return;
//     if(msg.block_state == 0) return;
//     Eigen::Vector3d up, down;
//     if(!GetSBSBound(msg.f_id, msg.block_id, up, down)){
//         ROS_ERROR("error InsertSwarmPts GetSBSBound");
//         return;
//     }

//     Eigen::Vector3d center = (up + down) / 2;
//     double range = 0;
//     for(int dim = 0; dim < 3; dim++) range = max(range, up(dim) - down(dim) + 0.5);
//     AwakeBlocks(center, range);

//     vector<Eigen::Vector3d> pts;
//     vector<uint8_t> states;

//     Flags2Vox(msg.flags, up, down, msg.block_state, pts, states);
//     InseartVox(pts, states);
//     UpdateSBS(msg.f_id, msg.block_id);
// }

// void BlockMapLite::SendSwarmBlockMap(const int &f_id, const bool &send_now){
//     if(f_id < 0 || f_id >= SBS_.size()) return;
//     if(send_now){
//         exp_comm_msgs::MapC msg;
//         for(uint8_t i = 0; i < 8; i++){
//             SBS_[f_id].to_pub_[i] = false;
//             Vox2Msg(msg, f_id, i);
//             if(msg.block_state != 0){
//                 SDM_->SetMap(msg);
//             }
//         }
//     }
//     else{
//         for(uint8_t i = 0; i < 8; i++){
//             if(!SBS_[f_id].to_pub_[i]){
//                 swarm_pub_block_.push_back({{f_id, i}, ros::WallTime::now().toSec()});
//                 SBS_[f_id].to_pub_[i] = true;
//             }
//         }
//     }
// }

void BlockMapLite::VisMapAll(const std_msgs::Empty &e){
    visualization_msgs::MarkerArray mka;
    visualization_msgs::Marker mk_stand;
    mk_stand.action = visualization_msgs::Marker::ADD;
    mk_stand.pose.orientation.w = 1.0;
    mk_stand.type = visualization_msgs::Marker::POINTS;
    mk_stand.scale.x = resolution_;
    mk_stand.scale.y = resolution_;
    mk_stand.scale.z = resolution_;
    mk_stand.color.a = 1.0;
    mk_stand.header.frame_id = "world";
    mk_stand.header.stamp = ros::Time::now();
    int block_num = block_num_(0) * block_num_(1) * block_num_(2);
    int lid;
    Eigen::Vector3i boi;
    Eigen::Vector3d block_origin;
    geometry_msgs::Point pt;
    shared_ptr<Grid_Block> GB;
    for(int i = 0; i < block_num; i++){
        mka.markers.push_back(mk_stand);
        mka.markers.back().id = i;
        
        lid = GetLocalBlockId(i);
        boi = Id2BlockIdx3(i);
        block_origin = boi.cast<double>().cwiseProduct(blockscale_) + origin_;
        if(lid < 0){
            if(!ReadDataRaw(i, GB)){
                mka.markers.back().type = visualization_msgs::Marker::CUBE;
                mka.markers.back().pose.position.x = block_origin(0) + blockscale_(0);
                mka.markers.back().pose.position.y = block_origin(1) + blockscale_(1);
                mka.markers.back().pose.position.z = block_origin(2) + blockscale_(2);
                mka.markers.back().scale.x = blockscale_(0);
                mka.markers.back().scale.y = blockscale_(1);
                mka.markers.back().scale.z = blockscale_(2);
                mka.markers.back().color.b = 0.5;
                mka.markers.back().color.a = 0.1;
                mka.markers.back().color.r = 0.5;
                continue;
            }
        }
        else{
            GB = GBS_local_[lid];
        }
        int idx = 0;
        
        Eigen::Vector3d block_end = GB->origin_.cast<double>() * resolution_ + resolution_ * block_size_.cast<double>()
        + origin_;
        if(GB->state_ == MIXED){//debug
            for(double z = resolution_ * (GB->origin_(2) + 0.5) + origin_(2); z < block_end(2); z += resolution_){
                for(double y = resolution_ * (GB->origin_(1)  + 0.5) + origin_(1); y < block_end(1); y += resolution_){
                    for(double x = resolution_ * (GB->origin_(0) + 0.5) + origin_(0); x < block_end(0); x += resolution_){
                        if(GB->odds_log_[idx] > 0){
                            pt.x = x;
                            pt.y = y; 
                            pt.z = z;
                            mka.markers.back().points.push_back(pt);
                            mka.markers.back().colors.push_back(Getcolor(z));
                        }
                        idx++;
                    }
                }
            }
        }
        if(mka.markers.back().points.size() == 0){
            mka.markers.back().action = visualization_msgs::Marker::DELETE;
        } 
    }
    if(mka.markers.size() > 0) vox_pub_.publish(mka);
}

bool BlockMapLite::PosBBXOccupied(const Eigen::Vector3d &pos, const Eigen::Vector3d &bbx){
    Eigen::Vector3d lowbd, upbd, v_it;
    VoxelState state;
    lowbd = pos - bbx / 2;
    upbd = pos + bbx / 2 + Eigen::Vector3d::Ones() * (resolution_ - 1e-3);
    for(v_it(0) = lowbd(0); v_it(0) < upbd(0); v_it(0) += resolution_){
        for(v_it(1) = lowbd(1); v_it(1) < upbd(1); v_it(1) += resolution_){
            for(v_it(2) = lowbd(2); v_it(2) < upbd(2); v_it(2) += resolution_){
                state = GetVoxState(v_it);
                if(state == VoxelState::occupied) return true;
            }
        }
    }
    return false;
}

bool BlockMapLite::PosBBXFree(const Eigen::Vector3d &pos, const Eigen::Vector3d &bbx){
    Eigen::Vector3d lowbd, upbd, v_it;
    VoxelState state;
    lowbd = pos - bbx / 2;
    upbd = pos + bbx / 2 + Eigen::Vector3d::Ones() * (resolution_ - 1e-3);
    for(v_it(0) = lowbd(0); v_it(0) < upbd(0); v_it(0) += resolution_){
        for(v_it(1) = lowbd(1); v_it(1) < upbd(1); v_it(1) += resolution_){
            for(v_it(2) = lowbd(2); v_it(2) < upbd(2); v_it(2) += resolution_){
                state = GetVoxState(v_it);
                if(state != VoxelState::free) return false;
            }
        }
    }
    return true;
}

void BlockMapLite::StatisticV(const ros::TimerEvent &e){
    std_msgs::Float32 msg;
    msg.data = stat_n_ * pow(resolution_, 3);
    statistic_pub_.publish(msg);
    if(stat_write_){
        CS_->SetVolume(stat_n_ * pow(resolution_, 3), 0);
    }
}

void BlockMapLite::ChangePtsDebug(){
    for(auto &cp : changed_pts_){
        auto state = GetVoxState(IdtoPos(cp.first));
        if(state == VoxelState::unknown){
            ROS_ERROR("error state unknown");
            getchar();
        }
        else if(state == VoxelState::outlocal){
            ROS_ERROR("error state outlocal");
            getchar();
        }
        else if(state == VoxelState::out){
            ROS_ERROR("error state out");
            getchar();
        }
    }
}

void BlockMapLite::Debug(list<Eigen::Vector3d> &pts, int ddd){
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = 2 + ddd;
    mk.action = visualization_msgs::Marker::ADD;
    mk.type = visualization_msgs::Marker::SPHERE_LIST;
    // mk.pose.position.x = SDM_->self_id_ * 0.1 - 0.5;
    mk.scale.x = 0.1;
    mk.scale.y = 0.1;
    mk.scale.z = 0.1;
    // if(SDM_->is_ground_)
    //     mk.color.r = 1.0;
    // else{
        if(ddd == 1){
            mk.color.g = 1.0;
        }
        else if(ddd == 2){
            mk.color.g = 1.0;
            mk.color.r = 0.5;
        }
        else{
            mk.color.b = 1.0;
            // mk.pose.position.z = 0.1;
        }
    // }
    mk.color.a = 0.5;
    geometry_msgs::Point pt;
    // if(!finish_flag_) mk.lifetime = ros::Duration(1.0);
    // ROS_WARN("id:%d Debug", SDM_->self_id_);
    for(auto &p : pts){
        pt.x = p(0);
        pt.y = p(1);
        pt.z = p(2);
        mk.points.emplace_back(pt);
    }
    debug_pub_.publish(mk);
}