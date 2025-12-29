#include <eroi/eroi.h>

void EroiGrid::LoadNewFeasibleLrmVox(vector<Eigen::Vector3d> &pts){
    Eigen::Vector3d dp, origin_local, ns = LRM_->node_scale_;
    Eigen::Vector3i bs(LRM_->block_size_(0), LRM_->block_size_(1) * LRM_->block_size_(0), LRM_->block_size_(0) * LRM_->block_size_(1) * LRM_->block_size_(2));
    Eigen::Vector3i gid3, gid3n, nid3;
    int gid, gidn, nid;
    // cout<<"LoadNewFeasibleLrmVox pts:"<<pts.size()<<endl;
    for(auto &p : pts){

        if(!InsideExpMap(p)) continue;
        // dp(0) = p(0) - origin_(0) - floor((p(0) - origin_(0)) / node_scale_(0)) * node_scale_(0);
        // dp(1) = p(1) - origin_(1) - floor((p(1) - origin_(1)) / node_scale_(1)) * node_scale_(0);
        // dp(2) = p(2) - origin_(2) - floor((p(2) - origin_(2)) / node_scale_(2)) * node_scale_(0);
        gid3(0) = floor((p(0) - origin_(0)) / node_scale_(0));
        gid3(1) = floor((p(1) - origin_(1)) / node_scale_(1));
        gid3(2) = floor((p(2) - origin_(2)) / node_scale_(2));
        dp = p - origin_ - gid3.cast<double>().cwiseProduct(node_scale_);
        nid3(0) = dp(0) / ns(0);
        nid3(1) = dp(1) / ns(1);
        nid3(2) = dp(2) / ns(2);
        nid = nid3(2) * bs(1) + nid3(1) * bs(0) + nid3(0);

        // if(nid < 0 || nid >= vp_dict_.size()) {
        //     cout<<"dp:"<<dp.transpose()<<endl;
        //     cout<<"nid3:"<<nid3.transpose()<<endl;
        //     cout<<"nid:"<<nid<<endl;
        //     StopageDebug("LoadNewFeasibleLrmVox nid out of range");
        // }
        for(auto &vs : vp_dict_[nid]){

            gid3n = vs.first + gid3;


            gidn = gid3n(2) * node_num_(1) * node_num_(0) + gid3n(1) * node_num_(0) + gid3n(0);
            if(gidn < 0 || gidn >= EROI_.size()) {
                // ROS_WARN("gidn < 0 || gidn >= EROI_.size()");
                continue;
            }
            if(!LRM_->InsideLocalMap(EROI_[gidn].center_)) {
                // gidn = gid3n(2) * node_num_(1) * node_num_(0) + gid3n(1) * node_num_(0) + gid3n(0);
                // Eigen::Vector3d vp_pos;
                // vp_pos = EROI_[gidn].center_ + vps_[vs.second].head(3);
                // cout<<"vp_pos:"<<vp_pos.transpose()<<endl;
                // cout<<"p:"<<p.transpose()<<endl;
                // cout<<"EROI_[gidn].center_:"<<EROI_[gidn].center_.transpose()<<endl;
                // cout<<"vs.first:"<<vs.first.transpose()<<endl;
                // cout<<"gid3n:"<<gid3n.transpose()<<endl;
                // cout<<"LRM_->local_map_upbd_:"<<LRM_->local_map_upbd_.transpose()<<endl;
                // cout<<"LRM_->local_map_lowbd_:"<<LRM_->local_map_lowbd_.transpose()<<endl;
                // cout<<"LRM_->local_up_node_idx_:"<<LRM_->local_up_node_idx_.transpose()<<endl;
                // cout<<"LRM_->local_origin_node_idx_:"<<LRM_->local_origin_node_idx_.transpose()<<endl;
                // StopageDebug("!LRM_->InsideLocalMap(gid3n)");
                continue;
            }
            if(EROI_[gidn].f_state_ >= 2) {
                // ROS_WARN("EROI_[gidn].f_state_ >= 2");
                continue;
            }

            // if(EROI_[gidn].flags_ & 2) {
            //     // ROS_WARN("EROI_[gidn].flags_ & 2");
            //     continue;
            // }
            // Eigen::Vector3d debug_p = EROI_[gidn].center_ + vps_[vs.second].head(3);
            // if(!LRM_->IsFeasible(debug_p)){
            //     cout<<"debug_p:"<<debug_p.transpose()<<endl;
            //     cout<<"vps_[vs.second].head(3):"<<vps_[vs.second].head(3).transpose()<<endl;
            //     cout<<"EROI_[gidn].center_:"<<EROI_[gidn].center_.transpose()<<endl;
            //     cout<<"gid3:"<<gid3.transpose()<<endl;
            //     cout<<"EROI_ size:"<<EROI_.size()<<endl;
            //     gid = gid3(2) * node_num_(1) * node_num_(0) + gid3(1) * node_num_(0) + gid3(0);
            //     cout<<"EROI_[gid].center_:"<<EROI_[gid].center_.transpose()<<endl;
            //     cout<<"nid:"<<nid<<endl;
            //     cout<<"gid:"<<gid<<endl;
            //     cout<<"p:"<<p.transpose()<<endl;
            //     cout<<"dp:"<<dp.transpose()<<endl;
            //     StopageDebug("LoadNewFeasibleLrmVox not feasible");
            // }
            auto ssdi = single_sample_dict_.find(gidn);

            if(ssdi == single_sample_dict_.end()){
                single_sample_dict_.insert({gidn, {}});
                single_sample_dict_[gidn].resize(samp_total_num_, false);
                single_sample_dict_[gidn][vs.second] = true;
                Eigen::Vector3d vp_pos;
                vp_pos = EROI_[gidn].center_ + vps_[vs.second].head(3);
                // if(!LRM_->IsFeasible(vp_pos)) {
                //     cout<<"vp_pos:"<<vp_pos.transpose()<<endl;
                //     cout<<"p:"<<p.transpose()<<endl;
                //     StopageDebug("!LRM_->IsFeasible(vp_pos)");
                // }
            }
            else{
                ssdi->second[vs.second] = true;
            }

        }

    }
}

void EroiGrid::GetLocalValidVps(vector<pair<uint32_t, uint8_t>> &erois, vector<pair<uint32_t, uint8_t>> &fvs, vector<uint32_t> &root_ids, vector<double> &distances){
    fvs.clear();
    erois.clear();
    root_ids.clear();
    distances.clear();
    uint32_t s_idx = root_ids.size();
    int rnum;
    bool new_root;
    for(int x = LRM_->local_origin_idx_(0); x <= LRM_->local_up_idx_(0); x++){
        for(int y = LRM_->local_origin_idx_(1); y <= LRM_->local_up_idx_(1); y++){
            for(int z = LRM_->local_origin_idx_(2); z <= LRM_->local_up_idx_(2); z++){
                uint32_t idx = x + y * node_num_(0) + z * node_num_(0) * node_num_(1);
                if(idx < 0 || idx >= EROI_.size()) continue;
                erois.push_back({idx, EROI_[idx].f_state_});

                if(EROI_[idx].f_state_ == 2 || EROI_[idx].f_state_ == 3) continue;
                // ROS_WARN("GetLocalValidVps1");

                s_idx = root_ids.size();
                rnum = 0;
                for(uint8_t i = 0; i < samp_total_num_; i++){
                    // ROS_WARN("GetLocalValidVps2");
                    if(idx >= EROI_.size()) {
                        StopageDebug("GetLocalValidVps idx >= EROI_.size()");
                    }
                    // if(i >= EROI_[idx].local_vps_.size()) {
                    //     cout<<"i:"<<i<<endl;
                    //     cout<<"idx:"<<idx<<endl;
                    //     cout<<"EROI_[idx].local_vps_.size():"<<EROI_[idx].local_vps_.size()<<endl;
                    //     cout<<"x:"<<x<<" y:"<<y<<" z:"<<z<<endl;
                    //     StopageDebug("GetLocalValidVps i >= EROI_[idx].local_vps_.size()");
                    // }
                    if(EROI_[idx].local_vps_[i] != 1) continue;
                    // ROS_WARN("GetLocalValidVps2.1");
                    Eigen::Vector3d vp_pos = EROI_[idx].center_ + vps_[i].head(3);
                    // ROS_WARN("GetLocalValidVps2.2");
                    if(!InsideExpMap(vp_pos) && !LRM_->IsFeasible(vp_pos)) continue;
                    // ROS_WARN("GetLocalValidVps3");
                    
                    new_root = true;
                    auto n = LRM_->GlobalPos2LocalNode(vp_pos);
                    // ROS_WARN("GetLocalValidVps4");

                    if(n->root_id_ == 0) continue;
                    for(int j = 0; j < rnum; j++){
                        if(j + s_idx >= fvs.size()) {
                            StopageDebug("GetLocalValidVps j + s_idx >= fvs.size()");
                        }
                        if(root_ids[j+s_idx] == n->root_id_){
                            if(distances[j+s_idx] > n->path_g_){
                                fvs[j+s_idx] = {idx, i};
                                distances[j+s_idx] = n->path_g_;
                            }


                            new_root = false;
                            break;
                        }
                    }
                    // ROS_WARN("GetLocalValidVps5");

                    if(new_root){
                        fvs.emplace_back(idx, i);
                        root_ids.emplace_back(n->root_id_);
                        distances.emplace_back(n->path_g_);
                        rnum++;
                    }
                    // ROS_WARN("GetLocalValidVps6");

                }
            }
        }
    }
}

void EroiGrid::SampleVps(list<Eigen::Vector3i> &posis){
    list<int> idxs;
    for(auto &p : posis){
        int idx = Posi2Idx(p);
        if(idx != -1) idxs.push_back(idx);
    }
    SampleVps(idxs);
}

void EroiGrid::SampleVps(list<Eigen::Vector3d> &poses){
    list<int> idxs;
    for(auto &p : poses){
        int idx = Pos2Idx(p);
        if(idx != -1) idxs.push_back(idx);
    }
    SampleVps(idxs);
}

void EroiGrid::SampleVps(list<int> &idxs, bool reset_flag){
    vp_update_ = true;
    double gain;
    Eigen::Vector4d vp_pose;
    Eigen::Vector3d vp_pos;
    // list<Eigen::Vector3d> debug_pts;
    int vp_id;
    // list<Eigen::Vector3d> debug_pts;
    // list<int> debug_fs;
    // int c = 0;
    double cur_t = ros::WallTime::now().toSec();
    bool skip_alive;
    for(auto &idx : idxs){
        if(idx < 0 || idx  >= EROI_.size() || EROI_[idx].f_state_ >= 2) continue;
        if(reset_flag) EROI_[idx].flags_ &= 253;
        skip_alive = (cur_t - EROI_[idx].last_sample_ < resample_duration_);
        if(!skip_alive) EROI_[idx].last_sample_ = cur_t;

        for(int vp_id = 0; vp_id < samp_total_num_; vp_id++){
            vp_pose = vps_[vp_id];
            vp_pose.head(3) += EROI_[idx].center_;
            vp_pos = vp_pose.head(3);
            // debug_pts.emplace_back(vp_pos);
            // debug_pts.emplace_back(EROI_[idx].center_);
            if(EROI_[idx].local_vps_[vp_id] == 2) {
                // ROS_ERROR("out 1");
                // Debug(debug_pts);
                // getchar();
                continue;
            }

            if(!LRM_->InsideLocalMap(vp_pos)) continue;
            if(!LRM_->IsFeasible(vp_pos) || BM_->PosBBXOccupied(vp_pos, Robot_size_ * 1.5) || !InsideExpMap(vp_pos) || LRM_->StrangePoint(vp_pos)){
                if(BM_->PosBBXOccupied(vp_pos, Robot_size_ * 1.5) || LRM_->StrangePoint(vp_pos) || !InsideExpMap(vp_pos)){
                    EROI_[idx].local_vps_[vp_id] = 2;
                    // if(idx == 63905 && (vp_id == 25 || vp_id == 50 || vp_id == 74)){
                    //     ROS_ERROR("SampleVps(list<int> &idxs, bool reset_flag) vp");
                    //     cout<<"vp:"<<int(vp_id)<<endl;
                    //     cout<<"!LRM_->IsFeasible(vp_pos):"<<(!LRM_->IsFeasible(vp_pos))<<endl;
                    //     cout<<"LRM_->StrangePoint(vp_pos):"<<LRM_->StrangePoint(vp_pos)<<endl;
                    //     cout<<"InsideExpMap(vp_pos):"<<InsideExpMap(vp_pos)<<endl;
                    // }
                    if(CheckValidFnode(idx, vp_id)){
                        dead_fnodes_.push_back({idx, vp_id});
                    }
                }
                // cout<<"!LRM_->IsFeasible(vp_pos):"<<!LRM_->IsFeasible(vp_pos)<<endl;
                // cout<<"BM_->PosBBXOccupied(vp_pos, Robot_size_ * 1.5):"<<BM_->PosBBXOccupied(vp_pos, Robot_size_ * 1.5)<<endl;
                // cout<<"InsideExpMap(vp_pos):"<<!InsideExpMap(vp_pos)<<endl;
                // cout<<"LRM_->StrangePoint(vp_pos):"<<LRM_->StrangePoint(vp_pos)<<endl;

                // ROS_ERROR("out 2");
                // Debug(debug_pts);
                // getchar();
                continue;
            }

            if(skip_alive && EROI_[idx].local_vps_[vp_id] == 1) continue;
            // gain = GetGain(idx, vp_id);
            // Eigen::Vector3d debug_p;
            // debug_p = EROI_[idx].center_ + vps_[vp_id].head(3);
            // debug_pts.emplace_back(debug_p);
            if(!StrongCheckViewpoint(idx, vp_id, true)) {


                EROI_[idx].local_vps_[vp_id] = 2;
                // if(idx == 63905 && (vp_id == 25 || vp_id == 50 || vp_id == 74)){
                //     ROS_ERROR("SampleVps(list<int> &idxs, bool reset_flag) StrongCheckViewpoint");
                //     cout<<"vp:"<<int(vp_id)<<endl;
                // }
                if(CheckValidFnode(idx, vp_id)){
                    dead_fnodes_.push_back({idx, vp_id});
                }
                // cout<<"g:"<<g<<"  vp_thresh_:"<<vp_thresh_<<endl;
                // ROS_ERROR("out 4");
                // Debug(debug_pts);
                // getchar();

            }
            else{
                // if(!StrongCheckViewpoint(idx, vp_id, true)){
                //     cout<<"gain:"<<gain<<"  vp_thresh_:"<<vp_thresh_<<endl;
                //     StrongCheckViewpointDebug(idx, vp_id, true);
                //     StopageDebug("!StrongCheckViewpointDebug next GetGainDebug");
                //     GetGainDebug(idx, vp_id);
                //     StopageDebug("!StrongCheckViewpoint && gain > vp_thresh_");
                // }

                EROI_[idx].local_vps_[vp_id] = 1;
                // ROS_ERROR("success");
            }

        }


        int alive_num = 0;
        for(int h_id = 0; h_id < samp_total_num_; h_id++){
            if(EROI_[idx].local_vps_[h_id] != 2){
                alive_num++;
            }
        }
        if(alive_num < min_vp_num_){
            ROS_WARN("dead eroi SampleVps %d", idx);
            EROI_[idx].f_state_ = 2;
            for(auto v : EROI_[idx].valid_vps_){
                dead_fnodes_.emplace_back(idx,v);
            }
        }
        // if(EROI_[idx].f_state_ == 1) c++;
        if(!(EROI_[idx].flags_ & 4)){
            EROI_[idx].flags_ |= 4;
            exploring_frontiers_show_.emplace_back(idx);
        }
    }
    // cout<<"c:"<<c<<endl;
    // if(idxs.size() == 0) return;
    // Debug(debug_pts);
    // getchar();
}

void EroiGrid::SampleTargetEroi(uint32_t &f_id){
    ROS_WARN("SampleTargetEroi");
    list<int> ids;
    Eigen::Vector3i n(1, node_num_(0), node_num_(0) * node_num_(1));
    Eigen::Vector3i id3;
    // list<Eigen::Vector3d> debug_pts;
    double cur_t = ros::WallTime::now().toSec();
    if(!Idx2Posi(f_id, id3)) return;
    for(int x = id3(0) - 1; x < id3(0) + 2; x++){
        if(x < 0 || x >= node_num_(0)) continue;
        for(int y = id3(1) - 1; y < id3(1) + 2; y++){
            if(y < 0 || y >= node_num_(1)) continue;
            for(int z = id3(2) - 1; z < id3(2) + 2; z++){
                if(z < 0 || z >= node_num_(2)) continue;
                int n1_idx = x + y * n(1)+ z * n(2);
                if(n1_idx < 0 || n1_idx >= EROI_.size()) continue;
                // if(dir.dot(f_grid_[n1_idx].center_ - cur_p) < 0.0) continue;
                if(EROI_[n1_idx].f_state_ == 0){
                    EROI_[n1_idx].f_state_ = 1;
                    EROI_[n1_idx].last_sample_ = cur_t - 100.0;
                    if(EROI_[n1_idx].unknown_num_ < EROI_[n1_idx].thresh_num_) {
                        // ROS_WARN("dead eroi SampleTargetEroi %d", n1_idx);
                        EROI_[n1_idx].f_state_ = 2;
                        for(auto v : EROI_[n1_idx].valid_vps_){
                            dead_fnodes_.emplace_back(n1_idx,v);
                        }
                    }

                    if(!(EROI_[n1_idx].flags_ & 4)){
                        exploring_frontiers_show_.push_back(n1_idx);
                        EROI_[n1_idx].flags_ |= 4;
                    }
                    if(!(EROI_[n1_idx].flags_ & 1) && EROI_[n1_idx].f_state_ == 1){
                        EROI_[n1_idx].flags_ |= 1;
                        ids.push_back(n1_idx);
                    }
                }
                else if(EROI_[n1_idx].f_state_ == 1){
                    EROI_[n1_idx].last_sample_ = cur_t - 100.0;
                    if(!(EROI_[n1_idx].flags_ & 1)){
                        ids.push_back(n1_idx);
                        EROI_[n1_idx].flags_ |= 1;
                    }
                }
            }
        }
    }

    // for(auto i : ids){
    //     for(int j = 0; j < EROI_[i].local_vps_.size(); j++){
    //         debug_pts.emplace_back(EROI_[i].center_ + vps_[j].head(3));
    //     }
    // }
    // Debug(debug_pts);
    // cout<<"ids:"<<ids.size()<<endl;
    SampleVps(ids, false);
    for(auto i : ids){
        EROI_[i].flags_ &= 254;
    }
}

void EroiGrid::SampleSingleVpsCallback(){
    vp_update_ = true;
    single_sample_ = true;
    Eigen::Vector4d vp_pose;
    Eigen::Vector3d vp_pos;
    double gain;
    list<Eigen::Vector3d> debug_pts;

    for(auto &f : single_sample_dict_){
        if(f.first < 0 || f.first >= EROI_.size() || EROI_[f.first].f_state_ >= 2 /*|| EROI_[f.first].flags_ & 2*/) continue;
        for(int i = 0; i < f.second.size(); i++){
            if(!f.second[i]) continue;
            if(EROI_[f.first].local_vps_[i] == 2) continue;
            vp_pose = vps_[i];
            vp_pose.head(3) += EROI_[f.first].center_;
            vp_pos = vp_pose.head(3);
            debug_pts.emplace_back(vp_pos);

            if(!LRM_->InsideLocalMap(vp_pos)) continue;
            if(!LRM_->InsideLocalMap(EROI_[f.first].center_)) continue;
            if(!LRM_->IsFeasible(vp_pos) || BM_->PosBBXOccupied(vp_pos, Robot_size_ * 1.5) || !InsideExpMap(vp_pos) || LRM_->StrangePoint(vp_pos)){
                if(BM_->PosBBXOccupied(vp_pos, Robot_size_ * 1.5) || LRM_->StrangePoint(vp_pos) || !InsideExpMap(vp_pos)){
                    EROI_[f.first].local_vps_[i] = 2;
                    // if(f.first == 63905 && (i == 25 || i == 50 || i == 74)){
                    //     ROS_ERROR("SampleSingleVpsCallback() vp");
                    //     cout<<"vp:"<<int(i)<<endl;
                    //     cout<<"!LRM_->IsFeasible(vp_pos):"<<(!LRM_->IsFeasible(vp_pos))<<endl;
                    //     cout<<"LRM_->StrangePoint(vp_pos):"<<LRM_->StrangePoint(vp_pos)<<endl;
                    //     cout<<"InsideExpMap(vp_pos):"<<InsideExpMap(vp_pos)<<endl;
                    // }
                    if(CheckValidFnode(f.first, i)){
                        dead_fnodes_.push_back({f.first, i});
                    }
                }
                continue;
            }

            // gain = GetGain(f.first, i);
            if(!StrongCheckViewpoint(f.first, i, true)) {
                EROI_[f.first].local_vps_[i] = 2;
                // if(f.first == 63905 && (i == 25 || i == 50 || i == 74)){
                //     ROS_ERROR("SampleSingleVpsCallback() StrongCheckViewpoint");
                //     cout<<"vp:"<<int(i)<<endl;
                // }
                if(CheckValidFnode(f.first, i)){
                    dead_fnodes_.push_back({f.first, i});
                }
            }
            else{
                EROI_[f.first].local_vps_[i] = 1;
                EROI_[f.first].f_state_ = 1;
            }
        }
        int alive_num = 0;
        for(int h_id = 0; h_id < samp_total_num_; h_id++){
            if(EROI_[f.first].local_vps_[h_id] != 2){
                alive_num++;
            }
        }
        if(alive_num < min_vp_num_){
            // ROS_WARN("dead eroi SampleSingleVpsCallback %d", f.first);
            EROI_[f.first].f_state_ = 2;
            for(auto v : EROI_[f.first].valid_vps_){
                dead_fnodes_.emplace_back(f.first,v);
            }
        }

        if(!(EROI_[f.first].flags_ & 4)){
            EROI_[f.first].flags_ |= 4;
            exploring_frontiers_show_.emplace_back(f.first);
        }
    }
    // cout<<"single_sample_dict_:"<<single_sample_dict_.size()<<endl;
    single_sample_dict_.clear();
    Debug(debug_pts, 1);

}

void EroiGrid::SampleSingleVpsCallback(const ros::TimerEvent &e){
    vp_update_ = true;
    single_sample_ = true;
    Eigen::Vector4d vp_pose;
    Eigen::Vector3d vp_pos;
    double gain;
    list<Eigen::Vector3d> debug_pts;

    for(auto &f : single_sample_dict_){
        if(f.first < 0 || f.first >= EROI_.size() || EROI_[f.first].f_state_ >= 2 /*|| EROI_[f.first].flags_ & 2*/) continue;
        for(int i = 0; i < f.second.size(); i++){
            if(!f.second[i]) continue;
            if(EROI_[f.first].local_vps_[i] == 2) continue;
            vp_pose = vps_[i];
            vp_pose.head(3) += EROI_[f.first].center_;
            vp_pos = vp_pose.head(3);
            debug_pts.emplace_back(vp_pos);

            if(!LRM_->InsideLocalMap(vp_pos)) continue;
            if(!LRM_->IsFeasible(vp_pos) || BM_->PosBBXOccupied(vp_pos, Robot_size_ * 1.5) || !InsideExpMap(vp_pos) || LRM_->StrangePoint(vp_pos)){
                if(BM_->PosBBXOccupied(vp_pos, Robot_size_ * 1.5) || LRM_->StrangePoint(vp_pos) || !InsideExpMap(vp_pos)){
                    EROI_[f.first].local_vps_[i] = 2;
                    // if(f.first == 63905 && (i == 25 || i == 50 || i == 74)){
                    //     ROS_ERROR("SampleSingleVpsCallback(e) vp");
                    //     cout<<"vp:"<<int(i)<<endl;
                    //     cout<<"!LRM_->IsFeasible(vp_pos):"<<(!LRM_->IsFeasible(vp_pos))<<endl;
                    //     cout<<"LRM_->StrangePoint(vp_pos):"<<LRM_->StrangePoint(vp_pos)<<endl;
                    //     cout<<"InsideExpMap(vp_pos):"<<InsideExpMap(vp_pos)<<endl;
                    // }
                    if(CheckValidFnode(f.first, i)){
                        dead_fnodes_.push_back({f.first, i});
                    }
                }
                continue;
            }

            // gain = GetGain(f.first, i);
            if(!StrongCheckViewpoint(f.first, i, true)) {
                EROI_[f.first].local_vps_[i] = 2;
                // if(f.first == 63905 && (i == 25 || i == 50 || i == 74)){
                //     ROS_ERROR("SampleSingleVpsCallback(e) StrongCheckViewpoint");
                //     // StrongCheckViewpointDebug(f.first, i, true);
                //     cout<<"vp:"<<int(i)<<endl;
                // }
                if(CheckValidFnode(f.first, i)){
                    dead_fnodes_.push_back({f.first, i});
                }
            }
            else{
                EROI_[f.first].local_vps_[i] = 1;
                EROI_[f.first].f_state_ = 1;
            }
        }
        int alive_num = 0;
        for(int h_id = 0; h_id < samp_total_num_; h_id++){
            if(EROI_[f.first].local_vps_[h_id] != 2){
                alive_num++;
            }
        }
        if(alive_num < min_vp_num_){
            // ROS_WARN("dead eroi SampleSingleVpsCallback %d", f.first);
            EROI_[f.first].f_state_ = 2;
            for(auto v : EROI_[f.first].valid_vps_){
                dead_fnodes_.emplace_back(f.first,v);
            }
        }

        if(!(EROI_[f.first].flags_ & 4)){
            EROI_[f.first].flags_ |= 4;
            exploring_frontiers_show_.emplace_back(f.first);
        }
    }
    // cout<<"single_sample_dict_e:"<<single_sample_dict_.size()<<endl;
    single_sample_dict_.clear();
    Debug(debug_pts, 2);

    // if(!debug_pts.empty()){
    //     Debug(debug_pts);
    //     ROS_WARN("SampleSingleVpsCallback");
    //     getchar();
    // }
}

void EroiGrid::UpdateFrontier(const vector<Eigen::Vector3d> &pts){
    int idx;
    list<int> idx_list;
    for(auto &p : pts){
        if(!InsideExpMap(p)) continue;;
        idx = Pos2Idx(p);
        if(idx == -1) continue;
        EROI_[idx].unknown_num_--;

        
        if(!(EROI_[idx].flags_ & 1)){
            EROI_[idx].flags_ |= 1;
            idx_list.push_back(idx);
        }
    }

    for(auto &it_idx : idx_list){
        EROI_[it_idx].flags_ &= 254;
        if(EROI_[it_idx].f_state_ == 0) {
            EROI_[it_idx].f_state_ = 1;
        }

        if(EROI_[it_idx].f_state_ == 1){
            if(EROI_[it_idx].unknown_num_ < EROI_[it_idx].thresh_num_){
                ExpandFrontier(it_idx, true);
                EROI_[it_idx].f_state_ = 2;
                LoadSampleVps(it_idx);
                // if(it_idx == 63904){
                //     // list<Eigen::Vector3d> debug_pts;
                //     // Eigen::Vector3d ns = LRM_->node_scale_, p;
                //     // for(p(2) = EROI_[it_idx].down_(2) + ns(2) * 0.5; p(2) < EROI_[it_idx].up_(2); p(2) += ns(2)){
                //     //     for(p(1) = EROI_[it_idx].down_(1) + ns(1) * 0.5; p(1) < EROI_[it_idx].up_(1); p(1) += ns(1)){
                //     //         for(p(0) = EROI_[it_idx].down_(0) + ns(0) * 0.5; p(0) < EROI_[it_idx].up_(0); p(0) += ns(0)){
                //     //             debug_pts.emplace_back(p);
                //     //         }
                //     //     }
                //     // }
                //     // Debug(debug_pts);
                //     DebugFunc();
                // }
                // ROS_WARN("dead eroi UpdateFrontier %d", it_idx);
                for(auto v : EROI_[it_idx].valid_vps_){
                    dead_fnodes_.emplace_back(it_idx,v);
                }
            }

            //show
            if(!(EROI_[it_idx].flags_ & 4) && EROI_[it_idx].f_state_ == 1){
                exploring_frontiers_show_.push_back(it_idx);
                EROI_[it_idx].flags_ |= 4;
            }
            else if(!(EROI_[it_idx].flags_ & 4) && EROI_[it_idx].f_state_ == 2){
                explored_frontiers_show_.push_back(it_idx);
                EROI_[it_idx].flags_ |= 4;
            }

            if(!(EROI_[it_idx].flags_ & 2) && EROI_[it_idx].f_state_ == 1){
                EROI_[it_idx].flags_ |= 2;
                exploring_frontiers_.push_back(it_idx);
                // ROS_WARN("UpdateFrontier push %d", it_idx);
            }

        }
    }
    // Debug(idxs);

    // getchar();
}

double EroiGrid::GetGain(const int &f_id, const int &vp_id){
    Eigen::Vector4d vp_pose;
    if(!GetVp(f_id, vp_id, vp_pose)) return 0;
    auto &frontier = EROI_[f_id];

    Eigen::Vector3d pos = vp_pose.block(0, 0, 3, 1);

    Eigen::Vector3d eroi_up, eroi_down;
    eroi_up = EROI_[f_id].center_ + node_scale_ * 0.5;
    eroi_down = EROI_[f_id].center_ - node_scale_ * 0.5;
    double dtheta = cam_hor_ / FOV_h_num_;
    double dphi = cam_ver_ / FOV_v_num_;
    double cos_phi;


    list<Eigen::Vector3d> ray;
    double gain = 0;
    bool inside_f;
    VoxelState state;
    for(auto &d_l : gain_dirs_[vp_id]){
        BM_->GetCastLine(pos, d_l.first + EROI_[f_id].center_, ray);
        double dist = 0, dist_l = 0;
        double c_g = 0, vn = 0;
        bool ray_in_eroi = false;
        for(auto &p : ray){
            if(!InsideExpMap(p)) break;
            if(p(0) > eroi_up(0) || p(0) < eroi_down(0) || p(1) > eroi_up(1) || p(1) < eroi_down(1) || p(2) > eroi_up(2) || p(2) < eroi_down(2)){
                if(ray_in_eroi) break;
            }
            else{
                ray_in_eroi = true;
            }
            
            state = BM_->GetVoxState(p);
            dist = (pos - p).norm();
            vn += 1;
            if(dist - dist_l > ray_samp_dist2_) {
                gain += (2*pow((dist + dist_l)*0.5, 2)*(dist - dist_l) + 1.0/6*pow(dist - dist_l, 3)) * d_l.second * c_g / vn;
                dist_l = dist;
                c_g = 0, vn = 0;
            }
            if(state == VoxelState::free){
                continue;
            }
            else if(state == VoxelState::occupied || state == VoxelState::out){
                break;
            }
            else{
                if(ray_in_eroi) c_g += 1;
            }
        }
    }

    return gain;

}


double EroiGrid::GetGainDebug(const int &f_id, const int &vp_id){
    bool vis_free;
    double gain = 0;
    auto &rays = gain_rays_[vp_id];
    VoxelState state;
    Eigen::Vector3d chk_pt;
    Eigen::Vector4d v_pose;
    Eigen::Vector3d v_pos;
    list<Eigen::Vector3d> debug_pts;

    v_pose = vps_[vp_id];
    v_pose.head(3) += EROI_[f_id].center_;
    v_pos = v_pose.head(3);
    // if(!GetVp(f_id, vp_id, v_pose)){
    //     ROS_ERROR("GetGain: error vp id%d", int(vp_id));
    //     cout<<int(EROI_[f_id].f_state_)<<endl;
    //     ros::shutdown();
    //     return -1;
    // }
    list<Eigen::Vector3d> vis_ray;

    for(auto &ray : rays){
        vis_free = true;
        if(!ray.first.empty()){
            BM_->GetCastLine(v_pos, ray.first.back() + EROI_[f_id].center_, vis_ray);
            for(auto &vox : vis_ray){
                chk_pt = vox + EROI_[f_id].center_;
                state = BM_->GetExpVoxState(chk_pt);
                if(state == VoxelState::occupied || state == VoxelState::out){
                    vis_free = false;
                    break;
                }
            }
        }
        if(!vis_free) continue;

        for(auto &v_g : ray.second){
            chk_pt = v_g.first + EROI_[f_id].center_;
            if(!InsideExpMap(chk_pt)) break;
            state = BM_->GetExpVoxState(chk_pt);
            if(state == VoxelState::free){
                continue;
            }
            else if(state == VoxelState::occupied || state == VoxelState::out){
                break;
            }
            else{   // out local or unknown
                debug_pts.emplace_back(chk_pt);
                gain += v_g.second;
                // break;
            }
        }
    }
    Debug(debug_pts);
    cout<<"debug_pts:"<<debug_pts.size()<<endl;
    cout<<"debug_pts:"<<debug_pts.front().transpose()<<endl;
    cout<<"GetGainDebug:"<<gain<<endl;
    return gain;
}


void EroiGrid::SampleVps(){
    SampleVps(exploring_frontiers_);
    exploring_frontiers_.clear();
    sample_flag_ = true;
    rough_sample_ = true;
}

void EroiGrid::SampleVpsCallback(const ros::TimerEvent &e){
    SampleVps(exploring_frontiers_);
    exploring_frontiers_.clear();
    sample_flag_ = true;
    rough_sample_ = true;
}

void EroiGrid::ExpandFrontier(const int &idx, const bool &local_exp){
    Eigen::Vector3i n(1, node_num_(0), node_num_(0) * node_num_(1));
    Eigen::Vector3i id3;
    double cur_t = ros::WallTime::now().toSec();
    if(!Idx2Posi(idx, id3)) return;
    for(int dim = 0; dim < 3; dim++){
        if(id3(dim) != 0){
            int n1_idx = idx - n(dim);
            if(n1_idx < EROI_.size() && n1_idx >= 0 && EROI_[n1_idx].f_state_ == 0){

                EROI_[n1_idx].f_state_ = 1;
                // if(use_swarm_ && !SDM_->is_ground_){ //send awake
                //     list<uint8_t> vp_temp;
                //     SDM_->SetDTGFn(n1_idx, EROI_[n1_idx].local_vps_, 0, true);
                //     // SDM_->SetDTGFnDeadvps(n1_idx, vp_temp);
                // }
                EROI_[n1_idx].last_sample_ = cur_t - 100.0;
                if(EROI_[n1_idx].unknown_num_ < EROI_[n1_idx].thresh_num_) {
                    // if(EROI_[n1_idx].f_state_ != 2 && use_swarm_ && !SDM_->is_ground_) SDM_->SetDTGFn(n1_idx, EROI_[n1_idx].local_vps_, 0, false); //SDM_->SetDTGFnDead(n1_idx);//send dead
                    EROI_[n1_idx].f_state_ = 2;
                    // ROS_WARN("dead eroi UpdateFrontier %d", n1_idx);
                    for(auto v : EROI_[n1_idx].valid_vps_){
                        dead_fnodes_.emplace_back(n1_idx,v);
                    }
                    // cout<<"EROI_[n1_idx].unknown_num_:"<<EROI_[n1_idx].unknown_num_<<endl;
                    // cout<<"EROI_[n1_idx].thresh_num_:"<<EROI_[n1_idx].thresh_num_<<endl;
                    // ROS_WARN("ExpandFrontiereeeee %d", n1_idx);

                    ExpandFrontier(n1_idx, true);
                    // if(use_swarm_ && !SDM_->is_ground_) BM_->SendSwarmBlockMap(n1_idx, false);
                }

                if(!(EROI_[n1_idx].flags_ & 4)){
                    exploring_frontiers_show_.push_back(n1_idx);
                    EROI_[n1_idx].flags_ |= 4;
                }


                if(!(EROI_[n1_idx].flags_ & 2) && EROI_[n1_idx].f_state_ == 1){
                    EROI_[n1_idx].flags_ |= 2;
                    EROI_[n1_idx].f_state_ = 1;
                    exploring_frontiers_.push_back(n1_idx);

                }
            }
        }
        if(id3(dim) != node_num_(dim) - 1){
            int n2_idx = idx + n(dim);
            if(n2_idx < EROI_.size() && n2_idx >= 0 && EROI_[n2_idx].f_state_ == 0){

                EROI_[n2_idx].f_state_ = 1;
                // if(use_swarm_ && !SDM_->is_ground_){ //send awake
                //     list<uint8_t> vp_temp;
                //     SDM_->SetDTGFn(n2_idx, EROI_[n2_idx].local_vps_, 0, true);
                // }
                EROI_[n2_idx].last_sample_ = cur_t - 100.0;
                if(EROI_[n2_idx].unknown_num_ < EROI_[n2_idx].thresh_num_) {
                    // if(EROI_[n2_idx].f_state_ != 2 && use_swarm_ && !SDM_->is_ground_) SDM_->SetDTGFn(n2_idx, EROI_[n2_idx].local_vps_, 0, false);//SDM_->SetDTGFnDead(n2_idx);//send dead
                    EROI_[n2_idx].f_state_ = 2;
                    // ROS_WARN("dead eroi UpdateFrontier %d", n2_idx);
                    for(auto v : EROI_[n2_idx].valid_vps_){
                        dead_fnodes_.emplace_back(n2_idx,v);
                    }
                    // cout<<"EROI_[n2_idx].unknown_num_:"<<EROI_[n2_idx].unknown_num_<<endl;
                    // cout<<"EROI_[n2_idx].thresh_num_:"<<EROI_[n2_idx].thresh_num_<<endl;
                    // ROS_WARN("ExpandFrontiereeeee %d", n2_idx);

                    ExpandFrontier(n2_idx, true);
                    // if(use_swarm_ && !SDM_->is_ground_) BM_->SendSwarmBlockMap(n2_idx, false);
                }

                if(!(EROI_[n2_idx].flags_ & 4)){
                    exploring_frontiers_show_.push_back(n2_idx);
                    EROI_[n2_idx].flags_ |= 4;
                }

                if(!(EROI_[n2_idx].flags_ & 2) && EROI_[n2_idx].f_state_ == 1){
                    EROI_[n2_idx].flags_ |= 2;
                    EROI_[n2_idx].f_state_ = 1;
                    exploring_frontiers_.push_back(n2_idx);

                    // exploring_frontiers_.push_back(n2_idx);
                    // ROS_WARN("ExpandFrontier4 push %d", n2_idx);
                }
            }
        }
    }
}

void EroiGrid::GetAliveGridsBBX(const Eigen::Vector3d &center, const Eigen::Vector3d &box_scale, list<pair<int, list<pair<int, Eigen::Vector3d>>>> &f_list){
    Eigen::Vector3d upbd, lowbd;
    Eigen::Vector3i upid, lowid, it;
    int f_id;
    upbd = center + box_scale / 2;
    lowbd = center - box_scale / 2;

    for(int dim = 0; dim < 3; dim++){
        upbd(dim) = min(upbd(dim), up_bd_(dim) - 1e-3);
        upbd(dim) = max(upbd(dim), origin_(dim) + 1e-3);
        lowbd(dim) = min(lowbd(dim), up_bd_(dim) - 1e-3);
        lowbd(dim) = max(lowbd(dim), origin_(dim) + 1e-3);
        upid(dim) = (upbd(dim) - origin_(dim)) / node_scale_(dim);
        lowid(dim) = (lowbd(dim) - origin_(dim)) / node_scale_(dim);
    }
    for(it(0) = lowid(0); it(0) <= upid(0); it(0)++){
        for(it(1) = lowid(1); it(1) <= upid(1); it(1)++){
            for(it(2) = lowid(2); it(2) <= upid(2); it(2)++){
                f_id = Posi2Idx(it);
                if(f_id == -1) continue;
                list<pair<int, Eigen::Vector3d>> vps;
                Eigen::Vector4d vp_pose;
                Eigen::Vector3d vp_pos;
                for(int v_id = 0; v_id < samp_total_num_; v_id++){
                    if(EROI_[f_id].local_vps_[v_id] == 1 && GetVp(f_id, v_id, vp_pose)){
                        vp_pos = vp_pose.block(0, 0, 3, 1);
                        vps.push_back({v_id, vp_pos});
                    }
                }
                if(vps.size() > 0){
                    f_list.push_back({f_id, vps});
                }
            }   
        }   
    }
}

void EroiGrid::LazySampleCallback(const ros::TimerEvent &e){
    Eigen::Vector3d up, low;
    Eigen::Vector3i up_id3, low_id3, it;
    int f_id;
    double cur_t = ros::WallTime::now().toSec();
    up = Robot_pos_ + Eigen::Vector3d::Ones() * sensor_range_;
    low = Robot_pos_ - Eigen::Vector3d::Ones() * sensor_range_;
    for(int dim = 0; dim < 3; dim++){
        up_id3(dim) = min(node_num_(dim) - 1, int(floor((up(dim) - origin_(dim))/node_scale_(dim))));
        low_id3(dim) = max(0, int(floor((low(dim) - origin_(dim))/node_scale_(dim))));
    }
    for(it(0) = low_id3(0); it(0) <= up_id3(0); it(0)++){
        for(it(1) = low_id3(1); it(1) <= up_id3(1); it(1)++){
            for(it(2) = low_id3(2); it(2) <= up_id3(2); it(2)++){
                f_id = Posi2Idx(it);
                if(f_id == -1) continue;
                // if(f_id == 123986){
                //     cout<<"EROI_[f_id].f_state_:"<<int(EROI_[f_id].f_state_)<<endl;
                //     cout<<"EROI_[f_id].unknown_num_:"<<EROI_[f_id].unknown_num_<<endl;
                //     cout<<"EROI_[f_id].thresh_num_:"<<EROI_[f_id].thresh_num_<<endl;
                //     cout<<"EROI_[f_id].last_sample_:"<<cur_t - EROI_[f_id].last_sample_<<endl;
                //     cout<<"!(EROI_[f_id].flags_ & 2):"<<!(EROI_[f_id].flags_ & 2)<<endl;
                //     for(auto i : exploring_frontiers_) cout<<"i:"<<i<<endl;
                // }
                if(EROI_[f_id].f_state_ == 1 && cur_t - EROI_[f_id].last_sample_ > 2.0 && !(EROI_[f_id].flags_ & 2)){
                    EROI_[f_id].flags_ |= 2;
                    exploring_frontiers_.push_back(f_id);
                }
            }   
        }   
    }
}

bool EroiGrid::StrongCheckViewpoint(const int &f_id, const int &v_id, const bool &allow_unknown){
    Eigen::Vector4d vp_pose;
    if(EROI_[f_id].f_state_ >= 2 || EROI_[f_id].local_vps_[v_id] == 2) return false;
    vp_pose = vps_[v_id];
    vp_pose.head(3) += EROI_[f_id].center_;

    // if(!GetVp(f_id, v_id, vp_pose)) return false;
    auto &frontier = EROI_[f_id];

    // block check
    Eigen::Vector3d pos = vp_pose.block(0, 0, 3, 1);
    if(allow_unknown && BM_->PosBBXOccupied(pos, Robot_size_)) {
        ROS_WARN("occ");
        return false;
    }
    else if(!allow_unknown && !BM_->PosBBXFree(pos, Robot_size_)) {
        ROS_WARN("not free");
        return false;
    }

    // bool vis_free;
    // double gain = 0;
    // auto &rays = gain_rays_[v_id];
    // VoxelState state;
    // Eigen::Vector3d chk_pt;
    // Eigen::Vector4d v_pose;
    // v_pose = vps_[v_id];
    // v_pose.head(3) += EROI_[f_id].center_;

    // for(auto &ray : rays){
    //     vis_free = true;
    //     for(auto &vox : ray.first){
    //         chk_pt = vox + EROI_[f_id].center_;
    //         state = BM_->GetExpVoxState(chk_pt);
    //         if(state == VoxelState::occupied || state == VoxelState::out){
    //             vis_free = false;
    //             break;
    //         }
    //     }
    //     if(!vis_free) continue;

    //     for(auto &v_g : ray.second){
    //         if(InsideExpMap(chk_pt)) break;
    //         chk_pt = v_g.first + EROI_[f_id].center_;
    //         state = BM_->GetExpVoxState(chk_pt);
    //         if(state == VoxelState::free){
    //             continue;
    //         }
    //         else if(state == VoxelState::occupied || state == VoxelState::out){
    //             break;
    //         }
    //         else{
    //             // debug_pts.emplace_back(chk_pt);
    //             gain += v_g.second;
    //             // break;
    //         }
    //     }

    //     if(gain > vp_thresh_){
    //         return true;
    //     }
    // }

    Eigen::Vector3d eroi_up, eroi_down;
    eroi_up = EROI_[f_id].center_ + node_scale_ * 0.5;
    eroi_down = EROI_[f_id].center_ - node_scale_ * 0.5;

    list<Eigen::Vector3d> ray;
    double gain = 0;
    bool inside_f;
    VoxelState state;
    int gr = 0;
    for(auto &d_l : gain_dirs_[v_id]){
        BM_->GetCastLine(pos, d_l.first + EROI_[f_id].center_, ray);
        double dist = 0, dist_l = 0;
        double c_g = 0, vn = 0;
        bool ray_in_eroi = false;
        bool g = false;
        for(auto &p : ray){
            if(!InsideExpMap(p)) break;
            if(p(0) > eroi_up(0) || p(0) < eroi_down(0) || p(1) > eroi_up(1) || p(1) < eroi_down(1) || p(2) > eroi_up(2) || p(2) < eroi_down(2)){
                if(ray_in_eroi) break;
            }
            else{
                ray_in_eroi = true;
            }
            
            state = BM_->GetVoxState(p);
            dist = (pos - p).norm();
            vn += 1;
            if(dist - dist_l > ray_samp_dist2_) {
                gain += (2*pow(dist, 2)*ray_samp_dist2_ + 1.0/6*pow(ray_samp_dist2_, 3)) * d_l.second * c_g / vn;
                dist_l = dist;
                c_g = 0, vn = 0;
            }
            if(state == VoxelState::free){
                continue;
            }
            else if(state == VoxelState::occupied || state == VoxelState::out){
                break;
            }
            else{
                g = true;
                c_g += 1;
            }
        }
        if(g) gr++;
        if(gain > vp_thresh_ && gr > 6) return true;
    }
    return false;

    // 
    // if(gain < vp_thresh_) {
    //     return false;
    // }
    // return true;
}


bool EroiGrid::StrongCheckViewpointDebug(const int &f_id, const int &v_id, const bool &allow_unknown){
    Eigen::Vector4d vp_pose;
    list<Eigen::Vector3d> debug_pts;
    // if(!GetVp(f_id, v_id, vp_pose)) return false;
    vp_pose = vps_[v_id];
    vp_pose.head(3) += EROI_[f_id].center_;
    auto &frontier = EROI_[f_id];

    // block check
    Eigen::Vector3d pos = vp_pose.block(0, 0, 3, 1);
    if(allow_unknown && BM_->PosBBXOccupied(pos, Robot_size_)) {
        ROS_WARN("occ");
        return false;
    }
    else if(!allow_unknown && !BM_->PosBBXFree(pos, Robot_size_)) {
        ROS_WARN("not free");
        return false;
    }

    Eigen::Vector3d eroi_up, eroi_down;
    eroi_up = EROI_[f_id].center_ + node_scale_ * 0.5;
    eroi_down = EROI_[f_id].center_ - node_scale_ * 0.5;

    list<Eigen::Vector3d> ray;
    double gain = 0;
    bool inside_f;
    VoxelState state;
    for(auto &d_l : gain_dirs_[v_id]){
        BM_->GetCastLine(pos, d_l.first + EROI_[f_id].center_, ray);
        double dist = 0, dist_l = 0;
        double c_g = 0, vn = 0;
        bool ray_in_eroi = false;
        for(auto &p : ray){
            if(!InsideExpMap(p)) break;
            if(p(0) > eroi_up(0) || p(0) < eroi_down(0) || p(1) > eroi_up(1) || p(1) < eroi_down(1) || p(2) > eroi_up(2) || p(2) < eroi_down(2)){
                if(ray_in_eroi) break;
            }
            else{
                ray_in_eroi = true;
            }
            
            state = BM_->GetVoxState(p);
            dist = (pos - p).norm();
            vn += 1;
            if(dist - dist_l > ray_samp_dist2_) {
                gain += (2*pow(dist, 2)*ray_samp_dist2_ + 1.0/6*pow(ray_samp_dist2_, 3)) * d_l.second * c_g / vn;
                dist_l = dist;
                c_g = 0, vn = 0;
            }
            if(state == VoxelState::free){
                continue;
            }
            else if(state == VoxelState::occupied || state == VoxelState::out){
                break;
            }
            else{
                debug_pts.emplace_back(p);
                c_g += 1;
            }
        }
    }
    Debug(debug_pts);
    cout<<"gain:"<<gain<<endl;
    getchar();
    if(gain < vp_thresh_) {
        return false;
    }
    return true;
}

void EroiGrid::FastMotionCheck(const uint32_t &fid, vector<vector<bool>> &valid_motions){
    Eigen::Vector3d center = EROI_[fid].center_;
    Eigen::Vector3d p;
    valid_motions.resize(vps_.size());
    for(int i = 0; i < vps_.size(); i++){
        valid_motions[i].resize(motion_trajs_[i].size(), true);
    }
    for(auto &b: motion_block_dict_){
        p = MotionId2Pos(center, b.first);
        if(!LRM_->IsFeasible(p, true)){
            for(auto d : b.second){
                valid_motions[d.first][d.second] = false;
            }
        }
    }
}

void EroiGrid::GetReachVps(const Eigen::Vector4d &vp, double dp, double dy, list<pair<uint32_t, uint8_t>> &reached_vps){
    reached_vps.clear();
    Eigen::Vector3d p = vp.head(3);
    if(!InsideExpMap(p)) return;
    Eigen::Vector3i gid3, diff, nid3;
    Eigen::Vector3d vp_pos;
    uint32_t idx, nidx;
    idx = Pos2Idx(p);
    if(!Idx2Posi(idx, gid3)){
        for(diff(0) = -1; diff(0) < 2; diff(0)++){
            for(diff(1) = -1; diff(1) < 2; diff(1)++){
                for(diff(2) = -1; diff(2) < 2; diff(2)++){
                    nid3 = diff + gid3;
                    nidx = Posi2Idx(nid3);
                    if(nidx == -1) continue;
                    for(uint8_t i = 0; i < vps_.size(); i++){
                        vp_pos = EROI_[nidx].center_ + vps_[i].head(3);
                        if((p - vp_pos).norm() < dp && abs(YawDiff(vps_[i](3), vp(3)))){
                            reached_vps.emplace_back(nidx, i);
                        }
                    }
                }
            }
        }
    }
}

void EroiGrid::DebugFunc(){
    for(auto i : exploring_frontiers_){
        if(i == 63905){
            ROS_ERROR("have 63905 in exploring_frontiers_");
        }
    }
    if(single_sample_dict_.find(63905) != single_sample_dict_.end()){
        for(int i = 0; i < single_sample_dict_[63905].size(); i++){
            if(single_sample_dict_[63905][i]) cout<<"single sample vp:"<<i<<endl;
        }
    }

    for(int i = 0; i < EROI_[63905].local_vps_.size(); i++){
        cout<<"EROI_[63905].local_vps_:"<<int(i)<<"  "<<int(EROI_[63905].local_vps_[i])<<endl;
        list<Eigen::Vector4d> fov_poses;
        Eigen::Vector4d vp = vps_[i];
        vp.head(3) += EROI_[63905].center_;

        fov_poses.emplace_back(vp);
        VisVps(fov_poses);
        getchar();
    }

}

void EroiGrid::ShowVpsCallback(const ros::TimerEvent &e){
    Eigen::Vector4d vp_pose;
    visualization_msgs::MarkerArray mka;
    visualization_msgs::Marker mk1, mk2;
    mk1.header.frame_id = "world";
    mk1.header.stamp = ros::Time::now();
    mk1.id = 1;
    mk1.action = visualization_msgs::Marker::ADD;
    mk1.type = visualization_msgs::Marker::CUBE;
    mk1.scale.x = node_scale_(0);
    mk1.scale.y = node_scale_(1);
    mk1.scale.z = node_scale_(2);
    mk1.color.a = 0.125;
    mk1.color.b = 0.7;
    mk1.color.g = 0.6;
    mk1.color.r = 0.6;
    mk1.pose.position.x = 0;
    mk1.pose.position.y = 0;
    mk1.pose.position.z = 0;
    mk1.pose.orientation.x = 0;
    mk1.pose.orientation.y = 0;
    mk1.pose.orientation.z = 0;
    mk1.pose.orientation.w = 1;
    mk2 = mk1;
    mk2.type = visualization_msgs::Marker::LINE_LIST;
    mk2.scale.x = 0.02;
    mk2.scale.y = 0.02;
    mk2.scale.z = 0.02;
    std_msgs::ColorRGBA cl;
    for(auto &f : exploring_frontiers_show_){
        auto &frontier = EROI_[f];
        frontier.flags_ &= 251;
        if(frontier.f_state_ == 0)
            continue;
        if(frontier.f_state_ == 2){
            explored_frontiers_show_.push_back(f);
            continue;
        }
        mk1.action = visualization_msgs::Marker::ADD;
        mk2.action = visualization_msgs::Marker::ADD;
        mk2.points.clear();
        mk1.id = f * 2;
        mk2.id = f * 2 + 1;

        // cl = CM_->Id2Color(EROI_[f].owner_, 0.15);
        cl = CM_->Id2Color(1, 0.125);
        mk1.color = cl;
        mk1.pose.position.x = frontier.center_(0);
        mk1.pose.position.y = frontier.center_(1);
        mk1.pose.position.z = frontier.center_(2);
        mk1.scale.x = frontier.up_(0) - frontier.down_(0);
        mk1.scale.y = frontier.up_(1) - frontier.down_(1);
        mk1.scale.z = frontier.up_(2) - frontier.down_(2);
        mk2.color = cl;
        mk2.color.a = 0.2;
        bool have_valid = false;
        for(int vp_id = 0; vp_id < samp_total_num_; vp_id++){
            if(frontier.local_vps_[vp_id] == 1 && GetVp(f, vp_id, vp_pose)){
                have_valid = true;
                geometry_msgs::Point pt;
                pt.x = frontier.center_(0);
                pt.y = frontier.center_(1);
                pt.z = frontier.center_(2);
                mk2.points.push_back(pt);
                pt.x = vp_pose(0);
                pt.y = vp_pose(1);
                pt.z = vp_pose(2);
                mk2.points.push_back(pt);
                LoadVpLines(mk2, vp_pose);
                // break;
            }
        }
        if(!have_valid){
            explored_frontiers_show_.push_back(f);
            continue;
        }
        mka.markers.emplace_back(mk1);
        mka.markers.emplace_back(mk2);
    }
    mk1.points.clear();
    mk2.points.clear();
    mk1.action = visualization_msgs::Marker::ADD;
    mk2.action = visualization_msgs::Marker::DELETE;
    mk1.color.a = 0.01;
    mk1.color.b = 0.7;
    mk1.color.g = 0.7;
    mk1.color.r = 0.7;
    for(auto &f : explored_frontiers_show_){
        auto &frontier = EROI_[f];
        frontier.flags_ &= 251;
        mk1.id = f * 2;
        mk2.id = f * 2 + 1;
        mk1.pose.position.x = frontier.center_(0);
        mk1.pose.position.y = frontier.center_(1);
        mk1.pose.position.z = frontier.center_(2);
        mk1.scale.x = frontier.up_(0) - frontier.down_(0);
        mk1.scale.y = frontier.up_(1) - frontier.down_(1);
        mk1.scale.z = frontier.up_(2) - frontier.down_(2);
        
        mka.markers.emplace_back(mk1);
        mka.markers.emplace_back(mk2);
    }
    show_pub_.publish(mka);
    explored_frontiers_show_.clear();
    exploring_frontiers_show_.clear();
}

void EroiGrid::ShowGainDebug(){
    visualization_msgs::Marker mk, mkr1, mkr2;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = 1;
    mk.action = visualization_msgs::Marker::ADD;
    mk.type = visualization_msgs::Marker::CUBE;
    mk.scale.x = node_scale_(0);
    mk.scale.y = node_scale_(1);
    mk.scale.z = node_scale_(2);
    mk.color.a = 0.2;
    mk.color.b = 1.0;
    mk.pose.position.x = 0;
    mk.pose.position.y = 0;
    mk.pose.position.z = 0;
    mk.pose.orientation.x = 0;
    mk.pose.orientation.y = 0;
    mk.pose.orientation.z = 0;
    mk.pose.orientation.w = 1;

    Eigen::Vector3d z_pos = Eigen::Vector3d::Zero();
    Eigen::Vector4d v_pose;
    mk.color.b = 0;
    mkr1 = mk;
    mkr2 = mk;
    mkr1.color.r = 0.7;
    mkr2.color.g = 0.7;
    mkr1.color.a = 0.7;
    mkr2.color.a = 0.7;
    mkr1.scale.x = resolution_;
    mkr1.scale.y = resolution_;
    mkr1.scale.z = resolution_;
    mkr2.scale.x = resolution_;
    mkr2.scale.y = resolution_;
    mkr2.scale.z = resolution_;
    mkr1.type = visualization_msgs::Marker::CUBE_LIST;
    mkr2.type = visualization_msgs::Marker::CUBE_LIST;
    geometry_msgs::Point pt;
    for(int i = 0; i < gain_rays_.size(); i++){

        mkr1.id = i*2 + 2;
        mkr2.id = i*2 + 3;
        mkr1.points.clear();
        mkr2.points.clear();

        mkr1.action = visualization_msgs::Marker::ADD;
        mkr2.action = visualization_msgs::Marker::ADD;
        if(!ros::ok()) return;
        if(!GetVp(z_pos, i, v_pose)){
            ROS_ERROR("error vp id%d", int(i));
            return;
        }
        pt.x = v_pose(0);
        pt.y = v_pose(1);
        pt.z = v_pose(2);
        mkr1.points.push_back(pt);
        for(auto &rays : gain_rays_[i]){
            for(auto & p1 : rays.first){
                pt.x = p1.x();
                pt.y = p1.y();
                pt.z = p1.z();
                mkr1.points.push_back(pt);
            }
            for(auto & p2 : rays.second){
                pt.x = p2.first.x();
                pt.y = p2.first.y();
                pt.z = p2.first.z();
                mkr2.points.push_back(pt);
            }
        }
        ros::Duration(0.05).sleep();
        debug_pub_.publish(mkr2);
        debug_pub_.publish(mkr1);
        ros::Duration(0.3).sleep();
        
        mkr1.action = visualization_msgs::Marker::DELETE;
        mkr2.action = visualization_msgs::Marker::DELETE;
        debug_pub_.publish(mkr2);
        debug_pub_.publish(mkr1);
    }

}

void EroiGrid::VisVps(list<Eigen::Vector4d> &poses){
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = -100;
    mk.action = visualization_msgs::Marker::ADD;
    mk.type = visualization_msgs::Marker::LINE_LIST;
    mk.scale.x = 0.05;
    mk.scale.y = 0.05;
    mk.scale.z = 0.05;
    mk.color.a = 0.8;
    mk.color.b = 1.0;
    mk.pose.position.x = 0;
    mk.pose.position.y = 0;
    mk.pose.position.z = 0;
    mk.pose.orientation.x = 0;
    mk.pose.orientation.y = 0;
    mk.pose.orientation.z = 0;
    mk.pose.orientation.w = 1;
    for(auto vp : poses) LoadVpLines(mk, vp);
    debug_pub_.publish(mk);
}

void EroiGrid::Debug(list<int> &f_ids){
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = -1;
    mk.action = visualization_msgs::Marker::ADD;
    mk.type = visualization_msgs::Marker::CUBE_LIST;
    mk.scale.x = node_scale_(0);
    mk.scale.y = node_scale_(1);
    mk.scale.z = node_scale_(2);
    mk.color.a = 1.0;
    mk.color.b = 1.0;
    mk.pose.position.x = 0;
    mk.pose.position.y = 0;
    mk.pose.position.z = 0;
    mk.pose.orientation.x = 0;
    mk.pose.orientation.y = 0;
    mk.pose.orientation.z = 0;
    mk.pose.orientation.w = 1;
    geometry_msgs::Point pt;
    for(auto &f_id : f_ids){
        if(f_id < 0 || f_id >= EROI_.size()) continue;
        pt.x = EROI_[f_id].center_(0);
        pt.y = EROI_[f_id].center_(1);
        pt.z = EROI_[f_id].center_(2);
        mk.points.emplace_back(pt);
    }
    debug_pub_.publish(mk);
}

void EroiGrid::Debug(list<Eigen::Vector3d> &pts, int id){
    visualization_msgs::Marker mk, mkr1, mkr2;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = id;
    // mk.id = scan_count_;
    // scan_count_++;
    mk.action = visualization_msgs::Marker::ADD;
    mk.type = visualization_msgs::Marker::SPHERE_LIST;
    mk.scale.x = 0.1;
    mk.scale.y = 0.1;
    mk.scale.z = 0.1;
    mk.color.a = 1.0;
    if(id == 1){
        mk.color.r = 1.0;
        mk.color.g = 0.0;
        mk.color.b = 0.0;
        mk.color.a = 0.7;
        mk.scale.x = 0.11;
        mk.scale.y = 0.11;
        mk.scale.z = 0.11;    
        mk.pose.position.z = 0.15;
    }
    else if(id == 2){
        mk.color.a = 0.7;
        mk.color.g = 1.0;
        mk.scale.x = 0.12;
        mk.scale.y = 0.12;
        mk.scale.z = 0.12;    
        mk.pose.position.z = -0.1;

    }
    else{
        mk.color.b = 1.0;
    }
    mk.pose.position.x = 0;
    mk.pose.position.y = 0;
    mk.pose.position.z = 0;
    mk.pose.orientation.x = 0;
    mk.pose.orientation.y = 0;
    mk.pose.orientation.z = 0;
    mk.pose.orientation.w = 1;

    geometry_msgs::Point pt;
    for(auto &p : pts){
        pt.x = p.x();
        pt.y = p.y();
        pt.z = p.z();
        mk.points.push_back(pt);
    }
    debug_pub_.publish(mk);
}

void EroiGrid::Debug(Eigen::Vector3d &pt){
    visualization_msgs::Marker mk, mkr1, mkr2;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = 1;
    // mk.id = scan_count_;
    // scan_count_++;
    mk.action = visualization_msgs::Marker::ADD;
    mk.type = visualization_msgs::Marker::CUBE;
    mk.scale.x = 0.2;
    mk.scale.y = 0.2;
    mk.scale.z = 0.2;
    mk.color.a = 0.5;
    mk.color.r = 1.0;
    mk.pose.position.x = pt.x();
    mk.pose.position.y = pt.y();
    mk.pose.position.z = pt.z();
    mk.pose.orientation.x = 0;
    mk.pose.orientation.y = 0;
    mk.pose.orientation.z = 0;
    mk.pose.orientation.w = 1;

    debug_pub_.publish(mk);
}

void EroiGrid::DebugShowAll(){
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = -1;
    mk.action = visualization_msgs::Marker::ADD;
    mk.type = visualization_msgs::Marker::CUBE_LIST;
    mk.scale.x = 0.25;
    mk.scale.y = 0.25;
    mk.scale.z = 0.25;
    mk.color.a = 0.3;
    mk.color.b = 1.0;
    mk.pose.position.x = 0;
    mk.pose.position.y = 0;
    mk.pose.position.z = 0;
    mk.pose.orientation.x = 0;
    mk.pose.orientation.y = 0;
    mk.pose.orientation.z = 0;
    mk.pose.orientation.w = 1;
    for(auto &f : EROI_){
        if(f.f_state_ == 1){
            for(int i = 0; i < samp_total_num_; i++){
                if(f.local_vps_[i] == 1){
                    Eigen::Vector4d vpt;
                    GetVp(f.center_, i, vpt);
                    geometry_msgs::Point pt;
                    pt.x = vpt(0);
                    pt.y = vpt(1);
                    pt.z = vpt(2);
                    mk.points.emplace_back(pt);
                }
            }
        }
    }
    debug_pub_.publish(mk);
}


void EroiGrid::LoadVpLines(visualization_msgs::Marker &mk, Eigen::Vector4d &vp){
    geometry_msgs::Point pt0, pt1, pt2, pt3, pt4;
    pt0.x = vp(0);    
    pt0.y = vp(1);    
    pt0.z = vp(2);
    pt1.x = vp(0) + 0.5 * cos(vp(3) + cam_hor_/2) * cos(-cam_ver_/2);    
    pt1.y = vp(1) + 0.5 * sin(vp(3) + cam_hor_/2) * cos(-cam_ver_/2);
    pt1.z = vp(2) + sin(-cam_ver_/2) * 0.5;
    pt2.x = vp(0) + 0.5 * cos(vp(3) - cam_hor_/2) * cos(-cam_ver_/2);      
    pt2.y = vp(1) + 0.5 * sin(vp(3) - cam_hor_/2) * cos(-cam_ver_/2);    
    pt2.z = vp(2) + sin(-cam_ver_/2) * 0.5;
    pt3.x = vp(0) + 0.5 * cos(vp(3) + cam_hor_/2) * cos(cam_ver_/2);    
    pt3.y = vp(1) + 0.5 * sin(vp(3) + cam_hor_/2) * cos(cam_ver_/2);
    pt3.z = vp(2) + sin(cam_ver_/2) * 0.5;
    pt4.x = vp(0) + 0.5 * cos(vp(3) - cam_hor_/2) * cos(cam_ver_/2);    
    pt4.y = vp(1) + 0.5 * sin(vp(3) - cam_hor_/2) * cos(cam_ver_/2); 
    pt4.z = vp(2) + sin(cam_ver_/2) * 0.5;

    mk.points.push_back(pt0);
    mk.points.push_back(pt1);
    mk.points.push_back(pt0);
    mk.points.push_back(pt2);
    mk.points.push_back(pt0);
    mk.points.push_back(pt3);
    mk.points.push_back(pt0);
    mk.points.push_back(pt4);

    mk.points.push_back(pt1);
    mk.points.push_back(pt2);

    mk.points.push_back(pt2);
    mk.points.push_back(pt4);

    mk.points.push_back(pt3);
    mk.points.push_back(pt4);
    
    mk.points.push_back(pt3);
    mk.points.push_back(pt1);
}

// void EroiGrid::DebugViewpoint(){
//     for(int i = 0; i < EROI_.size(); i++){
//         if(EROI_[i].f_state_ != 2){
//             ROS_WARN("id: %d unexplored f:%d", SDM_->self_id_, i);
//             cout<<"id:"<<int(SDM_->self_id_)<<" c:"<<EROI_[i].center_.transpose()<<" flag:"<<int(EROI_[i].f_state_)<<endl;
//             for(int j = 0; j < EROI_[i].local_vps_.size(); j++){
//                 cout<<"id:"<<int(SDM_->self_id_)<<" v:"<<j<<" vs:"<<int(EROI_[i].local_vps_[j])<<endl;
//             }
//         }
//     }
// }