#include <RipNeon/ripneon_struct.h>
#include <RipNeon/ripneon.h>


void RipNeon::SetPose(const geometry_msgs::Pose &pose){
    Eigen::Quaternionf q;
    q.x() = pose.orientation.x;
    q.y() = pose.orientation.y;
    q.z() = pose.orientation.z;
    q.w() = pose.orientation.w;
    q.normalize();
    sen_rot_ = q.toRotationMatrix() * sen_odom_rot_.toRotationMatrix();
    sen_rot_inv_ = sen_rot_.transpose();
    sen_tra_.x() = pose.position.x;
    sen_tra_.y() = pose.position.y;
    sen_tra_.z() = pose.position.z;
    have_pose_ = true;
}

void RipNeon::CheckAccuracy(string gt_path, string test_path){
    std::vector<std::string> files;
    DIR *dir;
    struct dirent *ent;
    
    if ((dir = opendir(gt_path.c_str())) != nullptr) {
        while ((ent = readdir(dir)) != nullptr) {
            if (ent->d_type == DT_REG) { 
                files.push_back(ent->d_name);
            }
        }
        closedir(dir);
    } else {
        StopageDebug("cant open gt_path");
    }

    uint64_t bid;
    tr1::unordered_map<uint64_t, bool> gt_dict;
    int voxel_num = block_size_(0) * block_size_(1) * block_size_(2);
    int data_size = ceil(voxel_num / 4);
    uint8_t data[data_size];
    uint8_t data_seg[4]; 

    int fnum = 0;
    uint64_t occ_num = 0;
    uint64_t free_num = 0;
    uint64_t correct_occ = 0, correct_free = 0;

    for(auto &f : files){
        if(!ros::ok()) return;
        cout<<"read process:"<<double(fnum) / files.size()  * 100<<"%"<<endl;
        cout<<"accuracy:"<<double(correct_occ + correct_free) / (occ_num + free_num)  * 100<<"%"<<endl;
        cout<<"free accuracy:"<<double(correct_free) / (free_num)  * 100<<"%"<<endl;
        cout<<"occ accuracy:"<<double(correct_occ) / (occ_num)  * 100<<"%"<<endl;
        fnum++;
        bid = stol(f);
        string p, p2;
        p = gt_path + "/" + to_string(bid);
        std::ifstream infile(p.c_str(), ios::binary);

        if(!infile.is_open()){
            cout<<"path:"<<p<<endl;
            cout<<"pathf:"<<f<<endl;
            ROS_ERROR("open file failed");
            ros::shutdown();
            return;
        }
        infile.read(reinterpret_cast<char*>(&data), sizeof(uint8_t[data_size]));
        infile.close();
        gt_dict.clear();

        for(int i = 0, j = 0; i < data_size; i++){
            DecodeVoxel(data[i], data_seg);
            for(int k = 0; k < 4; k++){
                if(j >= voxel_num){
                    // do nothing
                }
                else{
                    uint64_t key = bid * voxel_num + j;
                    if(data_seg[k] == 0) {
                    }
                    else if(data_seg[k] == 1) {
                        occ_num++;
                        gt_dict.insert({key, true});
                    }
                    else if(data_seg[k] == 3) {
                        free_num++;
                        gt_dict.insert({key, false});
                    }
                    else {
                        free_num++;
                        gt_dict.insert({key, false});
                    }
                }
                j++;
            }
        }

        p2 = test_path + "/" + to_string(bid);
        std::ifstream infile_t(p2.c_str(), ios::binary);

        if(!infile_t.is_open()){
            continue;
        }
        infile_t.read(reinterpret_cast<char*>(&data), sizeof(uint8_t[data_size]));
        infile_t.close();
        
        for(int i = 0, j = 0; i < data_size; i++){
            DecodeVoxel(data[i], data_seg);
            for(int k = 0; k < 4; k++){
                if(j >= voxel_num){
                    // do nothing
                }
                else{
                    uint64_t key = bid * voxel_num + j;
                    auto check_it = gt_dict.find(key);
                    if(check_it != gt_dict.end()){
                        if(data_seg[k] == 0) {
                        }
                        else if(data_seg[k] == 1) {
                            if(check_it->second) correct_occ++;
                        }
                        else if(data_seg[k] == 3) {
                            if(!check_it->second) correct_free++;
                        }
                        else {
                            if(!check_it->second) correct_free++;
                        }
                    }
                }
                j++;
            }
        }

    }
    cout<<"accuracy:"<<double(correct_occ + correct_free) / (occ_num + free_num)  * 100<<"%"<<endl;
    cout<<"free accuracy:"<<double(correct_free) / (free_num)  * 100<<"%"<<endl;
    cout<<"occ accuracy:"<<double(correct_occ) / (occ_num)  * 100<<"%"<<endl;
    cout<<"occ occ_num:"<<occ_num<<endl;
    cout<<"occ free_num:"<<free_num<<endl;
    // std::vector<std::string> files_t;
    // DIR *dir_t;
    // struct dirent *ent_t;

    // if ((dir_t = opendir(test_path.c_str())) != nullptr) {
    //     while ((ent_t = readdir(dir_t)) != nullptr) {
    //         if (ent_t->d_type == DT_REG) { 
    //             files_t.push_back(ent_t->d_name);
    //         }
    //     }
    //     closedir(dir_t);
    // } else {
    //     StopageDebug("cant open test_path");
    // }

    // fnum = 0;
    // uint64_t correct_num = 0, correct_occ = 0, correct_free = 0;
    // uint64_t oc=0, fr=0;
    // for(auto &f : files_t){
    //     if(!ros::ok()) return;
    //     cout<<"check process:"<<double(fnum) / files_t.size() * 100<<"%"<<endl;
    //     cout<<"accuracy:"<<double(correct_num) / (occ_num + free_num) * 100<<"%"<<endl;
    //     cout<<"occ:"<<double(correct_occ) / occ_num * 100<<"%"<<endl;
    //     cout<<"free:"<<double(correct_free) / free_num * 100<<"%"<<endl;
    //     fnum++;

    //     bid = stol(f);

    //     string p;
    //     p = test_path + "/" + to_string(bid);
    //     std::ifstream infile(p.c_str(), ios::binary);

    //     if(!infile.is_open()){
    //         ROS_ERROR("open file failed");
    //         ros::shutdown();
    //         return;
    //     }
    //     infile.read(reinterpret_cast<char*>(&data), sizeof(uint8_t[data_size]));
    //     infile.close();
        
    //     for(int i = 0, j = 0; i < data_size; i++){
    //         DecodeVoxel(data[i], data_seg);
    //         for(int k = 0; k < 4; k++){
    //             if(j >= voxel_num){
    //                 // do nothing
    //             }
    //             else{
    //                     if(data_seg[k] == 0) {
    //                     }
    //                     else if(data_seg[k] == 1) {
    //                         oc++;
    //                     }
    //                     else {
    //                         fr++;
    //                     }


    //                 uint64_t key = bid * voxel_num + j;
    //                 auto check_it = gt_dict.find(key);
    //                 if(check_it != gt_dict.end()){

    //                     if(data_seg[k] == 0) {
    //                     }
    //                     else if(data_seg[k] == 1) {
    //                         if(check_it->second){
    //                             correct_num++;
    //                             correct_occ++;
    //                         }
    //                     }
    //                     else if(data_seg[k] == 3) {
    //                         if(!check_it->second){
    //                             correct_num++;
    //                             correct_free++;
    //                         }
    //                     }
    //                     else {
    //                         if(!check_it->second){
    //                             correct_num++;
    //                             correct_free++;
    //                         }
    //                     }
    //                 }
    //             }
    //             j++;
    //         }
    //     }
    // }
    // cout<<"occnum:"<<"  cast:"<<occ_num<<"  key:"<<oc<<endl;
    // cout<<"freenum:"<<"  cast:"<<free_num<<"  key:"<<fr<<endl;
}

bool RipNeon::ResetLocalMap(const Eigen::Vector3f &p){
    double t0 = ros::WallTime::now().toSec();
    Eigen::Vector3f new_origin, half_blk;
    Eigen::Vector3i new_origin_idx, new_up_idx, it, diff;
    // cout<<"local_lowbd_0:"<<local_lowbd_.transpose()<<endl;
    // cout<<"local_upbd_0:"<<local_upbd_.transpose()<<endl;
    for(int dim = 0; dim < 3; dim++){
        if(p(dim) + 0.5*local_scale_(dim) >= map_upbd_(dim)) new_origin(dim) = map_upbd_(dim) - local_scale_(dim) + 1e-3;
        else if(p(dim) - 0.5*local_scale_(dim) <= map_lowbd_(dim)) new_origin(dim) = map_lowbd_(dim)+1e-3;
        else new_origin(dim) = p(dim) - local_scale_(dim) * 0.5;
    }


    if(!GetBlock3Id(new_origin, new_origin_idx)){
        cout<<"new_origin:"<<new_origin.transpose()<<endl;
        cout<<"origin_:"<<origin_.transpose()<<endl;
        cout<<"p:"<<p.transpose()<<endl;
        cout<<"local_scale_:"<<local_scale_.transpose()<<endl;
        cout<<"p - 0.5*local_scale_:"<<(p - 0.5f*local_scale_).transpose()<<endl;
        cout<<"map_lowbd_:"<<map_lowbd_.transpose()<<endl;
        ROS_ERROR("error ResetLocalMap");
        ros::shutdown();
        return false;
    }
    new_origin = new_origin_idx.cast<float>().cwiseProduct(blockscale_) + origin_;
    // cout<<"new_origin1:"<<new_origin.transpose()<<endl;

    new_up_idx = new_origin_idx + local_block_num_ - Eigen::Vector3i::Ones();

    diff = new_origin_idx - local_origin_idx_;
    if(diff.dot(diff) < 1e-3f) return false;

    // change blockids, out blockids and new blockids
    // list<Eigen::Vector3i> out_idxs; // global idx3
    // list<pair<int, Eigen::Vector3i>> new_idxs; // global idx, global idx3
    list<pair<int, Eigen::Vector3i>> change_idxs; // old local idx, global idx3
    new_idxs_.clear();
    out_idxs_.clear();
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
                    out_idxs_.emplace_back(it);
                    it = new_up_idx - Eigen::Vector3i(x, y, z);
                    int lid = GetBlockId(it);
                    new_idxs_.push_back({lid, it});
                }
                else {
                    int lid = GetLocalBlockId(it);
                    if(lid < 0 || lid >= PVBs_.size()) { //debug
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
    ROS_WARN("save0");
    std::vector<std::thread> threads_w;
    for(int i = 0; i < thread_num_; i++){
        threads_w.emplace_back(&RipNeon::SaveDataThread, this);
    }
    for(auto &t : threads_w){
        t.join();
    }
    ROS_WARN("save1");

    // cout<<"out num:"<<out_idxs_.size()<<endl;

    // for(auto &b : out_idxs_){
    //     int id = GetBlockId(b);
    //     int lid = GetLocalBlockId(b);

    //     if(id < 0) { // out map, dont save
    //         continue;
    //     }
    //     SaveData(id, PVBs_[lid]);
    // }

    local_origin_ = new_origin;
    local_origin_idx_ = new_origin_idx;
    local_up_idx_ = new_up_idx;
    local_origin_v_idx_ = local_origin_idx_.cwiseProduct(block_size_);
    local_up_v_idx_ = (local_up_idx_ + Eigen::Vector3i::Ones()).cwiseProduct(block_size_) - Eigen::Vector3i::Ones();
    local_upbd_ = local_origin_ + local_scale_ - Eigen::Vector3f::Ones() * 1e-3f;
    local_lowbd_ = local_origin_ + Eigen::Vector3f::Ones() * 1e-3f;
    update_upbd_ = local_upbd_ - bline_margin_;
    update_lowbd_ = local_lowbd_ + bline_margin_;
    // cout<<"local_lowbd_:"<<local_lowbd_.transpose()<<endl;
    // cout<<"local_upbd_:"<<local_upbd_.transpose()<<endl;
    // for(int dim = 0; dim < 3; dim++){
    //     if(local_origin_idx_(dim) < 0){
    //         StopageDebug("local_origin_idx_(dim) < 0");
    //     }
    //     if(local_up_idx_(dim) >= block_num_(dim)){
    //         cout<<"local_up_idx_:"<<local_up_idx_.transpose()<<endl;
    //         cout<<"block_num_:"<<block_num_.transpose()<<endl;
    //         cout<<"map_upbd_:"<<map_upbd_.transpose()<<endl;
    //         cout<<"map_lowbd_:"<<map_lowbd_.transpose()<<endl;
            
    //         StopageDebug("local_up_idx_(dim) >= block_num_(dim)");
    //     }
    // }

    list<pair<int, shared_ptr<PV_Block>>> change_list; // new id, block
    for(auto &bid : change_idxs){
        int lid = GetLocalBlockId(bid.second);
        change_list.push_back({lid, PVBs_[bid.first]});
        Eigen::Vector3f p = PVBs_[bid.first]->origin_.cast<float>() * resolution_ + origin_  + Eigen::Vector3f::Ones() * 0.5;
        if(!InsideLocalMap(p)){
            cout<<"p:"<<p.transpose()<<endl;
            cout<<"local_upbd_:"<<local_upbd_.transpose()<<endl;
            cout<<"local_lowbd_:"<<local_lowbd_.transpose()<<endl;
            StopageDebug("change failed");
        }
    }
    for(auto &ci : change_list){
        PVBs_[ci.first] = ci.second;
    }
    // for(auto &bid: new_idxs_){
    //     int lid = GetLocalBlockId(bid.second);
    //     cout<<"or new:"<<PVBs_[lid]->origin_.transpose()<<endl;
    //     cout<<"lid:"<<lid<<endl;
    // }
    // ReadDataThread();
    // cout<<"new_idxs_ block num:"<<new_idxs_.size()<<endl;
    // cout<<"PVBs_.size:"<<PVBs_.size()<<endl;
    std::vector<std::thread> threads_r;
    // read_blk_num_ = 0;
    for(int i = 0; i < thread_num_; i++){
        threads_r.emplace_back(&RipNeon::ReadDataThread, this);
    }
    for(auto &t : threads_r){
        t.join();
    }
    // cout<<"read_blk_num_:"<<read_blk_num_<<endl;


    // for(auto &bid: new_idxs_){
    //     int lid = GetLocalBlockId(bid.second);
    //     ReadData(bid.first, PVBs_[lid]);
    // }
    // for(int i = 0; i < PVBs_.size(); i++){
    //     Eigen::Vector3i pi((PVBs_[i]->origin_(0)) / block_size_(0), 
    //         (PVBs_[i]->origin_(1)) / block_size_(1), 
    //         (PVBs_[i]->origin_(2)) / block_size_(2));
    //     if(i != GetLocalBlockId(pi)){
    //         cout<<"i:"<<i<<" li:"<<GetLocalBlockId(pi)<<endl;
    //         cout<<"or:"<<PVBs_[i]->origin_.transpose()<<endl;
    //         cout<<"pi:"<<pi.transpose()<<endl;
    //         StopageDebug("i != GetLocalBlockId(PVBs_[i]->origin_");
    //     }
    // }
    cout<<"read write cost t:"<<ros::WallTime::now().toSec() - t0<<endl;
    // if(stat_){
    //     t0 = ros::WallTime::now().toSec() - t0;
    //     CS_.SetVolume(t0, t_msg_, 7);
    // }
    return true;
}

void RipNeon::SaveAll(){
    cout<<"save all"<<endl;
    Eigen::Vector3i it;

    for(int x = 0; x < local_block_num_(0); x++){
        for(int y = 0; y < local_block_num_(1); y++){
            for(int z = 0; z < local_block_num_(2); z++){
                it = Eigen::Vector3i(x, y, z) + local_origin_idx_;
                out_idxs_.emplace_back(it);
            }
        }
    }
    std::vector<std::thread> threads_w;
    for(int i = 0; i < thread_num_; i++){
        threads_w.emplace_back(&RipNeon::SaveDataThread, this);
    }
    for(auto &t : threads_w){
        t.join();
    }
    cout<<"save end"<<endl;
}

bool RipNeon::InsertPts(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pcl){
    // ROS_WARN("InsertPts0");
    newly_register_idx_.clear();
    if(!have_pose_) return false;
    double ts = ros::WallTime::now().toSec();
    // ROS_WARN("InsertPts1");
    if(!void_initialized_){
        InsertPts2Void(pcl);

        return false;
    }
    Eigen::Vector3f up_diff = update_upbd_ - sen_tra_ - Eigen::Vector3f::Ones() * 1e-3;
    Eigen::Vector3f low_diff = update_lowbd_ - sen_tra_ + Eigen::Vector3f::Ones() * 1e-3;
    for(int dim = 0; dim < 3; dim++){
        if(up_diff(dim) < 0 || low_diff(dim) > 0) {
            cout<<"sen_tra_:"<<sen_tra_.transpose()<<endl;
            cout<<"update_upbd_:"<<update_upbd_.transpose()<<endl;
            cout<<"update_lowbd_:"<<update_lowbd_.transpose()<<endl;
            return false;
        }    
    }

    // new_occ_.clear(), delete_occ_.clear(), new_free_.clear(), new_frontier_.clear(), delete_frontier_.clear();

    // vector<Eigen::Vector3f> debug_occ;
    // vector<Eigen::Vector3f> debug_free_cast;
    // vector<Eigen::Vector3f> debug_free;
    // vector<Eigen::Vector3f> debug_unknown;

    vector<pair<uint32_t, uint16_t>> expanded_list; // <bid, vid>
    vector<pair<uint32_t, uint16_t>> open_list; // <bid, vid>
    vector<pair<uint32_t, uint16_t>> occ_list; // occupied pts, new + old
    vector<pair<uint32_t, uint16_t>> cast_free_list; // pts out update range, update after occ pts
    vector<pair<uint32_t, uint16_t>> frontier_out_fc; // frontier out fc bbx, directly maintained
    vector<pair<uint32_t, uint16_t>> frontier_update_edge; // frontier \in (fc bbx - update bbx), must be examined  
    vector<pair<uint32_t, uint16_t>> frontier_unknown; // frontier must near these voxels
    // vector<pair<uint32_t, uint16_t>> pre_frontier; // frontier may be deleted

    double t_reset_img_s, t_iter_pts_s, t_iter_occ_and_frontier_s, t_bfs_s, t_frontier_s;
    // /*** iterate points ***/
    t_reset_img_s = ros::WallTime::now().toSec();
    ResetRdImg(ray_depth_img_);

    t_iter_pts_s = ros::WallTime::now().toSec();
    t_reset_img_s = t_iter_pts_s - t_reset_img_s;

    Eigen::Vector3f Point_up, Point_low;
    Point_up = sen_tra_;
    Point_low = sen_tra_;
    pair<uint32_t, uint16_t> idx;
    Eigen::Vector3f pt_w, pt_c;
    uint64_t idx_key;
    PointVox *vox;
    tr1::unordered_map<uint64_t, pair<uint8_t, uint16_t>>::iterator cp;
    bool om = true, nom = false, ou = true, nou = false;
    double logodds_or;
    for(pcl::PointCloud<pcl::PointXYZ>::const_iterator pcl_it = pcl->begin(); pcl_it != pcl->end(); pcl_it++){
        if(world_frame_pts_){
            pt_w(0) = pcl_it->x;
            pt_w(1) = pcl_it->y;
            pt_w(2) = pcl_it->z;
            pt_c = sen_rot_inv_ * (pt_w - sen_tra_);
        }
        else{
            pt_c(0) = pcl_it->x;
            pt_c(1) = pcl_it->y;
            pt_c(2) = pcl_it->z;
            pt_w = sen_rot_ * pt_c + sen_tra_;
        }

        if(LoadRdImgPt(ray_depth_img_, pt_c)){ // load lidar point in Rd image
            if(CastPointInUpdate(idx, pt_w, up_diff, low_diff)){
                // if(idx.first >= PVBs_.size() || idx.second >= PVBs_[idx.first]->vox_.size()){
                //     cout<<"PVBs_:"<<PVBs_.size()<<endl;
                //     cout<<"pt_w:"<<pt_w.transpose()<<endl;
                //     cout<<"update_lowbd_:"<<update_lowbd_.transpose()<<endl;
                //     cout<<"update_upbd_:"<<update_upbd_.transpose()<<endl;
                //     cout<<"idx:"<<idx.first<<" "<<idx.second<<endl;
                //     StopageDebug("IP1");
                // }

                vox = &PVBs_[idx.first]->vox_[idx.second];
                if(vox->PvFlags_ & PV_FRONTIER){
                    // delete_frontier_.emplace_back(idx);
                    vox->PvFlags_ &= PV_RESET_FRONTIER;
                }
                // if(vox->PvFlags_ & PV_FULL){
                //     vox->pts_.pop_front();
                // }
                // else{
                //     if(vox->pts_.size() + 1 >= pts_num_){
                //         vox->PvFlags_ |= PV_FULL;
                //     }
                // }
                // cout<<"pcl_it3"<<endl;

                if(!(vox->PvFlags_ & PV_OCC)){
                    occ_list.emplace_back(idx);
                    // Eigen::Vector3f ppp;
                    // ppp = Id2LocalPos(PVBs_[idx.first]->origin_, idx.second);
                    // if((ppp - pt_w).norm() > resolution_){
                    //     cout<<"ppp:"<<ppp.transpose()<<endl;
                    //     cout<<"pt_w:"<<pt_w.transpose()<<endl;
                    //     StopageDebug("error ppp - pt_w");
                    // }

                    expanded_list.emplace_back(idx);
                    BfsExpand(open_list, idx, pt_w, nom);
                    // if(vox->log_odds_ < 0.0){
                    //     new_occ_.emplace_back(idx);
                    // }

                    if(vox->log_odds_ < free_thr_){
                        idx_key = LocalId2GlobalId(idx.second, PVBs_[idx.first]);
                        // if(changed_pts_.find(idx_key) != changed_pts_.end()){
                        //     StopageDebug("out1");
                        // }   
                        changed_pts_.insert({idx_key, {1, 0}});
                        newly_register_idx_.emplace_back(pt_w);
                    }
                    else if(vox->log_odds_ < 0.0){
                        // if(changed_pts_.find(idx_key) != changed_pts_.end()){
                        //     StopageDebug("out2");
                        // }   
                        idx_key = LocalId2GlobalId(idx.second, PVBs_[idx.first]);
                        changed_pts_.insert({idx_key, {3, 0}});
                    }
                    vox->log_odds_ = thr_max_;

                    vox->PvFlags_ |= PV_BFS_OCC;
                    PVBs_[idx.first]->PvbFlags_ &= PVB_RESET_EMPTY;
                    // PVBs_[idx.first]->PvbFlags_ |= PVB_OCC;
                }
                // vox->pts_.emplace_back(pt_w);

                Point_up(0) = max(Point_up(0), pt_w(0));
                Point_up(1) = max(Point_up(1), pt_w(1));
                Point_up(2) = max(Point_up(2), pt_w(2));
                Point_low(0) = min(Point_low(0), pt_w(0));
                Point_low(1) = min(Point_low(1), pt_w(1));
                Point_low(2) = min(Point_low(2), pt_w(2));
            }
            else{ // update after updating all occ pts
                // if(idx.first >= PVBs_.size() || idx.second >= PVBs_[idx.first]->vox_.size()){
                //     cout<<"idx:"<<idx.first<<" "<<idx.second<<endl;
                //     cout<<"pt_w:"<<pt_w.transpose()<<endl;
                //     cout<<"update_lowbd_:"<<update_lowbd_.transpose()<<endl;
                //     cout<<"update_upbd_:"<<update_upbd_.transpose()<<endl;

                //     StopageDebug("IP2");
                // }
                cast_free_list.emplace_back(idx);
            }
        }
    }
    // for(auto o : occ_list){
    //     if(PVBs_[o.first]->id_ == 2823780){
    //         cout<<"occ1:"<<o.second<<endl;
    //     }
    // }
    // cout<<"cast_free_list:"<<cast_free_list.size()<<endl;
    // ROS_WARN("InsertPts2");

    for(auto &idx_f : cast_free_list){

        // if(idx_f.first >= PVBs_.size() || idx_f.second >= PVBs_[idx_f.first]->vox_.size()){
        //     cout<<"idx_f:"<<idx_f.first<<" "<<idx_f.second<<endl;
        //     StopageDebug("IP2");
        // }

        vox = &PVBs_[idx_f.first]->vox_[idx_f.second];

        pt_w = Id2LocalPos(PVBs_[idx_f.first]->origin_, idx_f.second);


        if(vox->PvFlags_ & PV_FRONTIER_BFS) continue;
        // if(vox->log_odds_ > 0) continue; // pervious occ
        Point_up(0) = max(Point_up(0), pt_w(0));
        Point_up(1) = max(Point_up(1), pt_w(1));
        Point_up(2) = max(Point_up(2), pt_w(2));
        Point_low(0) = min(Point_low(0), pt_w(0));
        Point_low(1) = min(Point_low(1), pt_w(1));
        Point_low(2) = min(Point_low(2), pt_w(2));

        logodds_or = vox->log_odds_;
        vox->log_odds_ += pro_miss_;
        vox->PvFlags_ |= PV_BFS;
        expanded_list.emplace_back(idx_f);
        BfsExpand(open_list, idx_f, pt_w, om);
        if(vox->log_odds_ >= 0) occ_list.emplace_back(idx_f); // still occ
        else{
            if(logodds_or < free_thr_){
                idx_key = LocalId2GlobalId(idx_f.second, PVBs_[idx_f.first]);
                // if(changed_pts_.find(idx_key) != changed_pts_.end()){
                //     StopageDebug("out3");
                // }
                changed_pts_.insert({idx_key, {0, 0}});
                newly_register_idx_.emplace_back(pt_w);
            }
            else if(logodds_or >= 0){
                idx_key = LocalId2GlobalId(idx_f.second, PVBs_[idx_f.first]);
                // if(changed_pts_.find(idx_key) != changed_pts_.end()){
                //     StopageDebug("out4");
                // }
                changed_pts_.insert({idx_key, {2, 0}});
            }
        }
        // PVBs_[idx.first]->PvbFlags_ |= PVB_OCC;

        if(vox->log_odds_ < free_thr_){ // uknown 2 free
            PVBs_[idx_f.first]->PvbFlags_ &= PVB_RESET_EMPTY;
            vox->log_odds_ = thr_min_;
            // new_free_.emplace_back(idx_f);
        }
    }
    // ROS_WARN("InsertPts3");
    t_iter_occ_and_frontier_s = ros::WallTime::now().toSec();
    t_iter_pts_s = t_iter_occ_and_frontier_s - t_iter_pts_s;


    /*** add bfs occ and frontier ***/
    float sr2 = (sensor_range_ + resolution_*2) * (sensor_range_ + resolution_*2);
    idx.first = 0;
    Eigen::Vector3f pt, bs;
    Eigen::Vector3f fc_upbd, fc_lowbd; // frontier check bbx
    Eigen::Vector3f update_upbd, update_lowbd, update_center, update_scale, update_thresh; // update bbx
    Eigen::Vector3f block_center; // update bbx
    Eigen::Vector3f half_bs =  blockscale_ * 0.5f;

    for(int dim = 0; dim < 3; dim++){ // get update bbx
        update_upbd(dim) = min(update_upbd_(dim), Point_up(dim) + resolution_*2);
        update_lowbd(dim) = max(update_lowbd_(dim), Point_low(dim) - resolution_*2);
        update_center(dim) = (update_upbd(dim) + update_lowbd(dim)) * 0.5f;
        update_scale(dim) = update_upbd(dim) - update_lowbd(dim);
        update_thresh(dim) = update_scale(dim) + half_bs(dim);
    }


    fc_upbd = update_upbd + Eigen::Vector3f::Ones() * resolution_;
    fc_lowbd = update_lowbd - Eigen::Vector3f::Ones() * resolution_;
    vector<uint32_t> update_blks;
    for(auto &blk : PVBs_){
        // if(!(blk->PvbFlags_ & PVB_OCC)){
            float r = 0.0;
            bool next_blk = false;
            for(int dim = 0; dim < 3; dim++){
                block_center(dim) = blk->origin_(dim) * resolution_ + origin_(dim) + half_bs(dim);
                r += pow(max(0.0f, abs(sen_tra_(dim) - (block_center(dim))) - half_bs(dim)), 2);
                if(abs(block_center(dim) - update_center(dim)) > update_thresh(dim)){
                    // idx.first++;
                    next_blk = true;
                    break;
                }
            }
            if(next_blk){
                // if(blk->id_ == 2823780){
                //     cout<<"next_blk"<<endl;
                // }
                idx.first++;
                continue;
            }
            if(r > sr2){
                // if(blk->id_ == 2823780){
                //     cout<<"r > sr2"<<endl;
                //     cout<<"r:"<<r<<endl;
                //     cout<<"sr2:"<<sr2<<endl;
                //     cout<<"sen_tra_:"<<sen_tra_.transpose()<<endl;
                //     for(int dim = 0; dim < 3; dim++){
                //        cout<<"dp:"<<max(0.0f, abs(sen_tra_(dim) - (half_bs(dim) + blk->origin_(dim) * resolution_ + origin_(dim))) - half_bs(dim))<<endl;
                //     }
                //     cout<<"center:"<<(half_bs + blk->origin_.cast<float>() * resolution_ + origin_).transpose()<<endl;
                // }
                idx.first++;
                continue;
            }
        // }
        bs = origin_ + blk->origin_.cast<float>() * resolution_;
        /** check occ pts **/
        bool out_ud = false;
        for(int dim = 0; dim < 3; dim++){
            pt(dim) = (blk->origin_(dim) + 0.5) * resolution_ + origin_(dim);
            if(pt(dim) < update_lowbd(dim)) {
                out_ud = true;
                break;
            } 
            pt(dim) = (blk->origin_(dim) + block_size_(dim) - 0.5) * resolution_ + origin_(dim);
            if(pt(dim) > update_upbd(dim)){
                out_ud = true;
                break;
            }
        }
        if(!out_ud){
            update_blks.emplace_back(idx.first);
            blk->PvbFlags_ |= PVB_UP;
        }
        
        for(auto &p : blk->occ_idx_){
            // if(p >= blk->vox_.size()){
            //     cout<<"p:"<<p<<endl;
            //     StopageDebug("IP3");
            // }

            vox = &blk->vox_[p];
            if(vox->PvFlags_ & PV_BFS) {
                continue; // updated
            }
            idx.second = p;

            /* get pt */
            // pt = Id2LocalPos(blk->origin_, p);
            pt = Id2LocalDiffPos(p) + bs;

            /* pt out update bbx, directly up load occ and continue */
            if(out_ud){
                if(pt(0) > update_upbd(0) || pt(1) > update_upbd(1) || pt(2) > update_upbd(2) || pt(0) < update_lowbd(0)
                        || pt(1) < update_lowbd(1) || pt(2) < update_lowbd(2)){
                    occ_list.emplace_back(idx);
                    continue;
                }
            }

            /* occ pt update */
            if(!TwoStepRdImgNotFree(pt)){ // occ pt inside free space, expand
                vox->log_odds_ += pro_miss_;

                if(vox->log_odds_ < 0.0){
                    if(vox->log_odds_ - pro_miss_ < 0){
                        cout<<"p0:"<<vox->log_odds_ - pro_miss_<<endl;
                        StopageDebug("vox->log_odds_ - pro_miss_ < 0");
                    }
                    vox->log_odds_ = thr_min_;
                    vox->PvFlags_ |= PV_BFS;
                    expanded_list.emplace_back(idx);
                    BfsExpand(open_list, idx, pt, om);
                    // vox->pts_.clear();
                    vox->PvFlags_ &= PV_RESET_FULL;

                    idx_key = LocalId2GlobalId(idx.second, PVBs_[idx.first]);
                    // if(changed_pts_.find(idx_key) != changed_pts_.end()){
                    //     StopageDebug("out5");
                    // }   
                    changed_pts_.insert({idx_key, {2, 1}});
                    // delete_occ_.emplace_back(idx);
                    // new_free_.emplace_back(idx);
                }
                else{
                    occ_list.emplace_back(idx);
                    TwoStepRdImgSetVoid();
                }

            }
            else{ // occ pt inside unknown space, maintain occ list
                occ_list.emplace_back(idx);
            }
        }
        blk->occ_idx_.clear();

        /** try expand frontier **/
        bool out_fc = false;
        for(int dim = 0; dim < 3; dim++){
            pt(dim) = (blk->origin_(dim) + 0.5) * resolution_ + origin_(dim);
            if(pt(dim) < fc_lowbd(dim)) {
                out_fc = true;
                break;
            } 
            pt(dim) = (blk->origin_(dim) + block_size_(dim) - 0.5) * resolution_ + origin_(dim);
            if(pt(dim) > fc_upbd(dim)){
                out_fc = true;
                break;
            }
        }
        for(auto &p : blk->fron_idx_){
            // if(p >= blk->vox_.size()){
            //     cout<<"p:"<<p<<endl;
            //     StopageDebug("IP4");
            // }
            vox = &blk->vox_[p];
            if(vox->PvFlags_ & PV_BFS) continue; // updated
            vox->PvFlags_ &= PV_RESET_FRONTIER;
            vox->PvFlags_ |= PV_PREFRONTIER;
            idx.second = p;
            // pre_frontier.emplace_back(idx);

            /* get pt */
            // pt = Id2LocalPos(blk->origin_, p);
            pt = Id2LocalDiffPos(p) + bs;

            /* pt out update bbx or fc bbx */
            if(out_fc){
                if(pt(0) > update_upbd(0) || pt(1) > update_upbd(1) || pt(2) > update_upbd(2) || pt(0) < update_lowbd(0)
                        || pt(1) < update_lowbd(1) || pt(2) < update_lowbd(2)){
                    if(pt(0) > fc_upbd(0) || pt(1) > fc_upbd(1) || pt(2) > fc_upbd(2) || pt(0) < fc_lowbd(0)
                            || pt(1) < fc_lowbd(1) || pt(2) < fc_lowbd(2)){
                        frontier_out_fc.emplace_back(idx);
                    }
                    else{
                        frontier_update_edge.emplace_back(idx);
                    }
                    continue;
                }
            }

            /* occ pt update */
            vox->PvFlags_ |= PV_BFS;
            expanded_list.emplace_back(idx);
            BfsExpand(open_list, idx, pt, om);
        }
        idx.first++;
        blk->fron_idx_.clear();

    }

    // ROS_WARN("InsertPts4");
    t_bfs_s = ros::WallTime::now().toSec();
    t_iter_occ_and_frontier_s = t_bfs_s - t_iter_occ_and_frontier_s;

    // for(auto o : occ_list){
    //     if(PVBs_[o.first]->id_ == 2823780){
    //         cout<<"occ2:"<<o.second<<endl;
    //     }
    // }
    /*** BFS ***/
    while (!open_list.empty()){
        idx = open_list.back();
        expanded_list.emplace_back(idx);

        open_list.pop_back();
        pt = Id2LocalPos(PVBs_[idx.first]->origin_, idx.second);
        // if(idx.first >= PVBs_.size() || idx.second >= PVBs_[idx.first]->vox_.size()){
        //     cout<<"idx:"<<idx.first<<" "<<idx.second<<endl;
        //     StopageDebug("IP5");
        // }

        vox = &PVBs_[idx.first]->vox_[idx.second];

        if(!(PVBs_[idx.first]->PvbFlags_ & PVB_UP)){
            if(pt(0) > update_upbd(0) || pt(1) > update_upbd(1) || pt(2) > update_upbd(2) || pt(0) < update_lowbd(0)
                        || pt(1) < update_lowbd(1) || pt(2) < update_lowbd(2)){
                if(vox->log_odds_ < free_thr_){ // voxels on the edge may cause frontier inside update bbx
                    frontier_unknown.emplace_back(idx);
                }
                continue;
            }
        }
        if((vox->log_odds_ > free_thr_)){ // dont update free voxels or occ & not frontier
            continue;
        }

        if(RdImgFree(pt)){
            vox->log_odds_ = thr_min_;
            idx_key = LocalId2GlobalId(idx.second, PVBs_[idx.first]);
            // if(changed_pts_.find(idx_key) != changed_pts_.end()){
            //     StopageDebug("out6");
            // }   
            changed_pts_.insert({idx_key, {0, 0}});
            newly_register_idx_.emplace_back(pt);

            PVBs_[idx.first]->PvbFlags_ &= PVB_RESET_EMPTY;
            BfsExpand(open_list, idx, pt, om);
        }   
        else{
            frontier_unknown.emplace_back(idx); // its neighbours are frontier
        }
    }

    for(auto &blk : update_blks){
        PVBs_[blk]->PvbFlags_ &= PVB_RESET_UP;
    }

    // ROS_WARN("InsertPts5");
    t_bfs_s =  ros::WallTime::now().toSec() - t_bfs_s;

    /*** occ update ***/
    for(auto &pid : occ_list){
        // if(pid.first >= PVBs_.size()){
        //     cout<<"pid:"<<pid.first<<" "<<pid.second<<endl;
        //     StopageDebug("IP6");
        // }
        PVBs_[pid.first]->occ_idx_.emplace_back(pid.second);
    }
    // ROS_WARN("InsertPts6");
    
    t_frontier_s = ros::WallTime::now().toSec();
    /*** frontier update ***/
    for(auto &foc : frontier_out_fc){
        // if(foc.first >= PVBs_.size() || foc.second >= PVBs_[foc.first]->vox_.size()){
        //     cout<<"foc:"<<foc.first<<" "<<foc.second<<endl;
        //     StopageDebug("IP7");
        // }
        PVBs_[foc.first]->fron_idx_.emplace_back(foc.second);
        PVBs_[foc.first]->vox_[foc.second].PvFlags_ |= PV_FRONTIER;
    }
    // ROS_WARN("InsertPts7");

    vector<pair<uint32_t, uint16_t>> neighbours;
    /* get frontiers near unknown voxels */
    for(auto &fu : frontier_unknown){
        neighbours.clear();
        BfsNeighbours(neighbours, fu);
        for(auto &n : neighbours){
            // if(n.first >= PVBs_.size() || n.second >= PVBs_[n.first]->vox_.size()){
            //     cout<<"fu:"<<fu.first<<" "<<fu.second<<endl;
            //     cout<<"n:"<<n.first<<" "<<n.second<<endl;
            //     StopageDebug("IP8");
            // }
            vox = &PVBs_[n.first]->vox_[n.second];
            if(vox->PvFlags_ & PV_FRONTIER) continue; // already been set
            if(vox->log_odds_ < 0 && vox->log_odds_ > free_thr_){
                vox->PvFlags_ |= PV_FRONTIER;
                PVBs_[n.first]->fron_idx_.emplace_back(n.second);
                // if(!vox->PvFlags_ & PV_PREFRONTIER){
                    // new_frontier_.emplace_back(n);
                // }
            }
        }
    }
    // ROS_WARN("InsertPts8");

    /* check frontier on update edges */
    for(auto &fue : frontier_update_edge){
        vox = &PVBs_[fue.first]->vox_[fue.second];
        if(vox->PvFlags_ & PV_FRONTIER) continue;// already been set
        neighbours.clear();
        BfsNeighbours(neighbours, fue);
        for(auto &n : neighbours){
            // if(n.first >= PVBs_.size() || n.second >= PVBs_[n.first]->vox_.size()){
            //     cout<<"n:"<<n.first<<" "<<n.second<<endl;
            //     StopageDebug("IP9");
            // }
            if(/*PVBs_[n.first]->vox_[n.second].log_odds_ < 0 &&*/ PVBs_[n.first]->vox_[n.second].log_odds_ <= free_thr_){
                vox->PvFlags_ |= PV_FRONTIER;
                PVBs_[fue.first]->fron_idx_.emplace_back(fue.second);
                break;
            }
        }
        // if(!find_unknown) delete_frontier_.emplace_back(fue);
    }
    
    t_frontier_s = ros::WallTime::now().toSec() - t_frontier_s;

    // ROS_WARN("InsertPts9");

    /* load deleted frontier */
    // for(auto &pf : pre_frontier){
        // if(pf.first >= PVBs_.size() || pf.second >= PVBs_[pf.first]->vox_.size()){
        //     cout<<"pf:"<<pf.first<<" "<<pf.second<<endl;
        //     StopageDebug("IP10");
        // }
        // if(!PVBs_[pf.first]->vox_[pf.second].PvFlags_ & PV_FRONTIER)
        //     delete_frontier_.emplace_back(pf);
    // }
    // ROS_WARN("InsertPts10");

    /*** clear flags ***/
    for(auto &ep : expanded_list){
        // if(ep.first >= PVBs_.size() || ep.second >= PVBs_[ep.first]->vox_.size()){
        //     cout<<"ep:"<<ep.first<<" "<<ep.second<<endl;
        //     StopageDebug("IP10");
        // }
        PVBs_[ep.first]->vox_[ep.second].PvFlags_ &= PV_RESET_BFS_OCC_PREFRONTIER;
    }
    for(auto &b : PVBs_){
        b->PvbFlags_ &= PVB_RESET_OCC;
    }
    
    ts = ros::WallTime::now().toSec() - ts;
    // if(stat_){
    //     CS_.SetVolume(ts, t_msg_, 0);
    //     CS_.SetVolume(t_reset_img_s, t_msg_, 1);
    //     CS_.SetVolume(t_iter_pts_s, t_msg_, 2);
    //     CS_.SetVolume(t_iter_occ_and_frontier_s, t_msg_, 3);
    //     CS_.SetVolume(t_bfs_s, t_msg_, 4);
    //     CS_.SetVolume(t_frontier_s, t_msg_, 5);
    //     double t_other = ts - t_reset_img_s - t_iter_pts_s - t_iter_occ_and_frontier_s - t_bfs_s - t_frontier_s;
    //     CS_.SetVolume(t_other, t_msg_, 6);

    //     // getrusage(RUSAGE_SELF, &usage_);
    //     // double mem = usage_.ru_maxrss / 1024.0;
    //     double mem = GetMemory() / 1024.0 / 1024.0;
    //     CS_.SetVolume(mem, t_msg_, 8);
    //     cout<<"-------------------------------------"<<endl;
    //     cout<<"memory:"<<mem<<endl;
    //     cout<<"update t:"<<ts<<endl;
    //     cout<<"reset imd t:"<<t_reset_img_s<<endl;
    //     cout<<"iter pts t:"<<t_iter_pts_s<<endl;
    //     cout<<"occ and frontier t:"<<t_iter_occ_and_frontier_s<<endl;
    //     cout<<"bfs t:"<<t_bfs_s<<endl;
    //     cout<<"frontier detection t:"<<t_frontier_s<<endl;
    //     t_mean_ = t_mean_ * t_num_ + ts;
    //     t_num_++;
    //     t_mean_ = t_mean_ / t_num_;
    //     cout<<"t_mean_:"<<t_mean_<<endl;

    //     // DebugShow(); //debug
    // }
    return true;
}

bool RipNeon::InsertPtsCast(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pcl){
    if(!have_pose_) return false;
    double ts = ros::WallTime::now().toSec();

    Eigen::Vector3f up_diff = update_upbd_ - sen_tra_ - Eigen::Vector3f::Ones() * 1e-3;
    Eigen::Vector3f low_diff = update_lowbd_ - sen_tra_ + Eigen::Vector3f::Ones() * 1e-3;
    for(int dim = 0; dim < 3; dim++){
        if(up_diff(dim) < 0 || low_diff(dim) > 0) {
            cout<<"sen_tra_:"<<sen_tra_.transpose()<<endl;
            cout<<"update_upbd_:"<<update_upbd_.transpose()<<endl;
            cout<<"update_lowbd_:"<<update_lowbd_.transpose()<<endl;
            return false;
        }    
    }

    RayCaster rc;
    PointVox *vox;
    Eigen::Vector3f end_point, dir, cam, it_p, half_res(resolution_ * 0.5, resolution_ * 0.5, resolution_ * 0.5);
    Eigen::Vector3d cs, ce, ray_iter;
    uint32_t block_id;
    uint16_t vox_id;
    vector<uint32_t> block_ids;
    vector<uint16_t> vox_ids;
    cam = sen_tra_;
    for(pcl::PointCloud<pcl::PointXYZ>::const_iterator pcl_it = pcl->begin(); pcl_it != pcl->end(); pcl_it++){
        bool occ;
        if(world_frame_pts_){
            end_point(0) = pcl_it->x;
            end_point(1) = pcl_it->y;
            end_point(2) = pcl_it->z;
        }
        else{
            end_point(0) = pcl_it->x;
            end_point(1) = pcl_it->y;
            end_point(2) = pcl_it->z;
            end_point = sen_rot_ * end_point + sen_tra_;
        }

        dir = end_point - cam;
        occ = dir.norm() <= sensor_range_;
        if(!occ)
            end_point = cam + (end_point - cam).normalized() * sensor_range_;
        GetRayEndInsideLocalAndGlobalMap(cam, end_point, occ);
        if(!GetLocalVox(block_id, vox_id, end_point)) continue;
        if(occ){
            if(!(PVBs_[block_id]->vox_[vox_id].PvFlags_ & PV_OCC)){

                PVBs_[block_id]->vox_[vox_id].PvFlags_ |= PV_OCC;

                if(PVBs_[block_id]->vox_[vox_id].PvFlags_ & PV_BFS){
                }
                else{
                    PVBs_[block_id]->vox_[vox_id].PvFlags_ |= PV_BFS;
                    block_ids.push_back(block_id);
                    vox_ids.push_back(vox_id);
                }
            }
        }
        
        cs(0) = (end_point(0) - origin_(0)) / resolution_;
        cs(1) = (end_point(1) - origin_(1)) / resolution_;
        cs(2) = (end_point(2) - origin_(2)) / resolution_;
        ce(0) = (cam(0) - origin_(0)) / resolution_;
        ce(1) = (cam(1) - origin_(1)) / resolution_;
        ce(2) = (cam(2) - origin_(2)) / resolution_;
        rc.setInput(cs, ce);
        
        while (rc.step(ray_iter))
        {
            it_p = (ray_iter.cast<float>()) * resolution_ + origin_ + half_res;
            if(GetLocalVox(block_id, vox_id, it_p)){

                if((PVBs_[block_id]->vox_[vox_id].PvFlags_ & PV_BFS)){
                    continue;
                }
                else{
                    PVBs_[block_id]->vox_[vox_id].PvFlags_ |= PV_BFS;
                    block_ids.push_back(block_id);
                    vox_ids.push_back(vox_id);
                }
            }
        }
    }

    vector<uint32_t>::iterator block_it;
    vector<uint16_t>::iterator vox_it;
    uint32_t occ_num = 0, free_num = 0;
    for(block_it = block_ids.begin(), vox_it = vox_ids.begin(); block_it != block_ids.end(); block_it++, vox_it++){
        if(*block_it >= PVBs_.size()) StopageDebug("*block_it >= PVBs_.size()");
        if(*vox_it >= PVBs_[*block_it]->vox_.size()) StopageDebug("*vox_it >= PVBs_[*block_it]->vox_.size()");
        if(PVBs_[*block_it]->vox_[*vox_it].PvFlags_ & PV_OCC){
            PVBs_[*block_it]->vox_[*vox_it].log_odds_ = thr_max_;
            occ_num++;
        }
        else{
            free_num++;
            if(PVBs_[*block_it]->vox_[*vox_it].log_odds_ < free_thr_)
                PVBs_[*block_it]->vox_[*vox_it].log_odds_ = thr_min_;
            else {
                PVBs_[*block_it]->vox_[*vox_it].log_odds_ += pro_miss_;
                PVBs_[*block_it]->vox_[*vox_it].log_odds_ = max(PVBs_[*block_it]->vox_[*vox_it].log_odds_, thr_min_);
            }
        }
        PVBs_[*block_it]->vox_[*vox_it].PvFlags_ &= PV_RESET_BFS_OCC_PREFRONTIER;
        PVBs_[*block_it]->PvbFlags_ &= PVB_RESET_EMPTY;
    }
    // cout<<"occ_num:"<<occ_num<<endl;
    // cout<<"free_num:"<<free_num<<endl;
    // DebugShow(block_ids, vox_ids);
    ts = ros::WallTime::now().toSec() - ts;
    cout<<"-------------------------------------"<<endl;
    cout<<"update t:"<<ts<<endl;
    return true;
}

double RipNeon::GetMemory(){
    double mem = 0;
    mem += PVBs_.size() * sizeof(shared_ptr<PV_Block>);
    // cout<<"mem1:"<<mem / 1024.0 / 1024.0<<endl;
    mem += PVBs_.size() * sizeof(PV_Block);
    // cout<<"mem2:"<<mem / 1024.0 / 1024.0<<endl;

    for(auto &b : PVBs_){
        if(b == NULL) continue;
        mem += (b->occ_idx_.size() + b->fron_idx_.size()) * sizeof(uint16_t) + b->vox_.size() * (sizeof(float) + sizeof(uint8_t));
    }
    // cout<<"sizeof(PointVox):"<<sizeof(PointVox)<<endl;
    // cout<<"mem3:"<<mem / 1024.0 / 1024.0<<endl;

    return mem;
}


void RipNeon::ShowMapCallback(const ros::TimerEvent &e){
    visualization_msgs::MarkerArray mka;
    visualization_msgs::Marker mk_c, ml_p;
    mk_c.action = visualization_msgs::Marker::ADD;
    mk_c.pose.orientation.w = 1.0;
    mk_c.type = visualization_msgs::Marker::CUBE_LIST;
    mk_c.scale.x = resolution_;
    mk_c.scale.y = resolution_;
    mk_c.scale.z = resolution_;
    mk_c.color.a = 1.0;
    mk_c.header.frame_id = "world";
    mk_c.header.stamp = ros::Time::now();

    // ml_p.action = visualization_msgs::Marker::ADD;
    // ml_p.pose.orientation.w = 1.0;
    // ml_p.type = visualization_msgs::Marker::POINTS;
    // ml_p.scale.x = 0.05;
    // ml_p.scale.y = 0.05;
    // ml_p.scale.z = 0.05;
    // ml_p.color.a = 1.0;
    // ml_p.header.frame_id = "world";
    // ml_p.header.stamp = ros::Time::now();
    // mka.markers.emplace_back();
    Eigen::Vector3f Pt;
    geometry_msgs::Point pt;
    int i = 0;
    for(auto &b : PVBs_){

        /** occ voxels **/
        mka.markers.push_back(mk_c);
        mka.markers.back().id = b->id_*2;
        // mka.markers.back().id = i*2;
        // cout<<"show2"<<endl;
        // int occ_num = 0;
        // for(uint16_t i = 0; i < b->vox_.size(); i++){
        //     if(b->vox_[i].log_odds_ <= 0) continue;
        //     occ_num++;
        // }
        for(auto &p : b->occ_idx_){ 
            // cout<<"show2.1"<<endl;
            // cout<<"bo:"<<b->origin_.transpose()<<endl;
            // cout<<"p:"<<p<<endl;
            Pt = Id2LocalPos(b->origin_, p);
            // cout<<"show2.2"<<endl;
            pt.x = Pt(0);
            pt.y = Pt(1);
            pt.z = Pt(2);
            mka.markers.back().points.push_back(pt);
            mka.markers.back().colors.push_back(Getcolor(Pt(2)));
            mka.markers.back().colors.back().a = 0.0;
        }
        // if(occ_num != b->occ_idx_.size()){
        //     cout<<"occ:"<<occ_num<<"  size:"<<b->occ_idx_.size()<<endl;
        //     StopageDebug("occ_num != b->occ_idx_.size()");
        // }
        // cout<<"show2.5"<<endl;
        if(mka.markers.back().points.size() == 0){
            mka.markers.back().action = visualization_msgs::Marker::DELETE;
        } 
        // cout<<"show3"<<endl;

        /** frontier voxels **/
        mka.markers.push_back(mk_c);
        mka.markers.back().id = b->id_*2 + 1;
        // mka.markers.back().id = i*2 + 1;
        mka.markers.back().color.a = 0.2;
        mka.markers.back().color.g = 1.0;
        // vector<pair<uint32_t, uint16_t>> neighbours;
        if(show_fontier_){
            for(auto &p : b->fron_idx_){
                Pt = Id2LocalPos(b->origin_, p);
                pt.x = Pt(0);
                pt.y = Pt(1);
                pt.z = Pt(2);
                mka.markers.back().points.push_back(pt);
            }
        }

        if(mka.markers.back().points.size() == 0){
            mka.markers.back().action = visualization_msgs::Marker::DELETE;
        }
        i++;

    }
    if(mka.markers.size() > 0) vox_pub_.publish(mka);
}

void RipNeon::InsertPts2Void(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pcl){
    void_frame_num_++;
    if(void_frame_num_ >= void_frame_thresh_){
        ROS_WARN("InsertPts2Void");
        void_initialized_ = true;
        sen_ready_pub_.publish(std_msgs::Empty());

        // visualization_msgs::Marker mk_o;
        // mk_o.action = visualization_msgs::Marker::ADD;
        // mk_o.pose.orientation.w = 1.0;
        // mk_o.id = 1;
        // mk_o.color.a = 1.0;
        // mk_o.color.b = 0.2;
        // mk_o.type = visualization_msgs::Marker::SPHERE_LIST;
        // mk_o.scale.x = resolution_ * 0.5;
        // mk_o.scale.y = resolution_ * 0.5;
        // mk_o.scale.z = resolution_ * 0.5;
        // mk_o.header.frame_id = "world";
        // mk_o.header.stamp = ros::Time::now();
        // pair<uint16_t, uint16_t> idx;
        // for(idx.first = 0; idx.first < void_standard_.size(); idx.first++){
        //     for(idx.second = 0; idx.second < void_standard_[idx.first].size(); idx.second++){
        //         if(void_standard_[idx.first][idx.second]){
        //             Eigen::Vector3f p;
        //             ImgIdx2Pos(idx, p);
        //             geometry_msgs::Point pt;
        //             pt.x = p(0);
        //             pt.y = p(1);
        //             pt.z = p(2);
        //             mk_o.points.emplace_back(pt);
        //         }
        //     }
        // }
        // debug_pub_.publish(mk_o);
        return;
    }

    Eigen::Vector3f pt_w, pt_c;
    for(pcl::PointCloud<pcl::PointXYZ>::const_iterator pcl_it = pcl->begin(); pcl_it != pcl->end(); pcl_it++){
        if(world_frame_pts_){
            pt_w(0) = pcl_it->x;
            pt_w(1) = pcl_it->y;
            pt_w(2) = pcl_it->z;
            pt_c = sen_rot_inv_ * (pt_w - sen_tra_);
        }
        else{
            pt_c(0) = pcl_it->x;
            pt_c(1) = pcl_it->y;
            pt_c(2) = pcl_it->z;
            pt_w = sen_rot_ * pt_c + sen_tra_;
        }
        pair<uint16_t, uint16_t> idx;
        if(Pos2ImgIdx(idx, pt_c)){
            void_stat_[idx.first][idx.second]++;
            if(void_stat_[idx.first][idx.second] >= void_pts_thresh_){
                void_standard_[idx.first][idx.second] = true;
            } 
        }
    }
}

void RipNeon::DebugShow(){
    int i = 0;
    Eigen::Vector3f Pt;
    // ROS_WARN("DebugShow");
    for(auto &b : PVBs_){
        // if(b->id_ != 2823780) continue;
        int occ = 0;
        // cout<<"==occ_idx_:"<<b->occ_idx_.size()<<endl;
        // cout<<"==bid:"<<b->id_<<endl;
        for(int j = 0; j < b->vox_.size(); j++){
            if(b->vox_[j].log_odds_ >= 0){
                occ++;
            }
        }

        if(occ != b->occ_idx_.size()){
            cout<<"occ:"<<occ<<endl;
            cout<<"occ_idx_:"<<b->occ_idx_.size()<<endl;
            cout<<"bid:"<<b->id_<<endl;
            for(int j = 0; j < b->vox_.size(); j++){
                if(b->vox_[j].log_odds_ >= 0){
                    cout<<"b:"<<b->id_<<" v:"<<j<<endl;
                    // cout<<"log_odds_:"<<b->vox_[j].log_odds_<<endl;
                }
            }
            StopageDebug("occ != occ idx");
        }
        // vector<pair<uint32_t, uint16_t>> neighbours;
        // for(auto &p : b->fron_idx_){
        //     Pt = Id2LocalPos(b->origin_, p);
        //     if(Pt(0) > update_lowbd_(0) && Pt(0) < update_upbd_(0) && Pt(1) > update_lowbd_(1) && Pt(1) < update_upbd_(1) &&
        //             Pt(2) > update_lowbd_(2) && Pt(2) < update_upbd_(2)){
        //         neighbours.clear();
        //         pair<uint32_t, uint16_t> ptx;
        //         ptx.first = i;
        //         ptx.second = p;
        //         BfsNeighbours(neighbours, ptx);
        //         bool find_u = false;
        //         for(auto n : neighbours){
        //             if(n.first >= PVBs_.size() || n.second >= PVBs_[n.first]->vox_.size()){
        //                 cout<<"n:"<<n.first<<" "<<n.second<<endl;
        //                 cout<<"ptx:"<<ptx.first<<" "<<ptx.second<<endl;
        //                 cout<<"Pt:"<<Pt.transpose()<<endl;
        //                 cout<<"update_upbd_:"<<update_upbd_.transpose()<<endl;
        //                 cout<<"update_lowbd_:"<<update_lowbd_.transpose()<<endl;
        //                 StopageDebug("out frontier");
        //             }
        //             if(PVBs_[n.first]->vox_[n.second].log_odds_ < free_thr_){
        //                 find_u = true;
        //                 break;
        //             }
        //         }
        //         if(!find_u){
        //             cout<<"update_upbd_:"<<update_upbd_.transpose()<<endl;
        //             cout<<"update_lowbd_:"<<update_lowbd_.transpose()<<endl;
        //             cout<<"Pt:"<<Pt.transpose()<<endl;
        //             for(auto n : neighbours){
        //                 cout<<"nei:"<<Id2LocalPos(PVBs_[n.first]->origin_, n.second).transpose()<<endl;
        //                 cout<<"logodds:"<<PVBs_[n.first]->vox_[n.second].log_odds_<<endl;
        //             }
        //             StopageDebug("invalid frontier");
        //         }
        //     }
        // }
    }
}

void RipNeon::DebugShow(vector<Eigen::Vector3f> &occ, vector<Eigen::Vector3f> &free, vector<Eigen::Vector3f> &free_cast, vector<Eigen::Vector3f> &unknown){
    visualization_msgs::Marker mk;
    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.orientation.w = 1.0;
    mk.type = visualization_msgs::Marker::SPHERE_LIST;
    mk.scale.x = resolution_ * 0.15;
    mk.scale.y = resolution_ * 0.15;
    mk.scale.z = resolution_ * 0.15;
    mk.color.a = 0.5;
    mk.color.r = 1.0;
    mk.id = 1;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();

    std_msgs::ColorRGBA r;
    r.a = 1.0;
    r.r = 1.0;
    std_msgs::ColorRGBA g;
    g.a = 1.0;
    g.g = 1.0;
    std_msgs::ColorRGBA b;
    b.a = 1.0;
    b.b = 1.0;
    std_msgs::ColorRGBA rb;
    rb.a = 1.0;
    rb.r = 1.0;
    rb.b = 1.0;

    geometry_msgs::Point pt;
    for(auto p : occ){
        pt.x = p(0);
        pt.y = p(1);
        pt.z = p(2);
        mk.points.emplace_back(pt);
        mk.colors.emplace_back(r);
    }

    for(auto p : free){
        pt.x = p(0);
        pt.y = p(1);
        pt.z = p(2);
        mk.points.emplace_back(pt);
        mk.colors.emplace_back(g);
    }

    for(auto p : free_cast){
        pt.x = p(0);
        pt.y = p(1);
        pt.z = p(2);
        mk.points.emplace_back(pt);
        mk.colors.emplace_back(b);
    }

    for(auto p : unknown){
        pt.x = p(0);
        pt.y = p(1);
        pt.z = p(2);
        mk.points.emplace_back(pt);
        mk.colors.emplace_back(rb);
    }
    debug_pub_.publish(mk);
}

void RipNeon::DebugShow(vector<Eigen::Vector3f> &occ, vector<Eigen::Vector3f> &delete_occ, vector<Eigen::Vector3f> &cam_occ){
    visualization_msgs::Marker mk_o, ml_d;
    mk_o.action = visualization_msgs::Marker::ADD;
    mk_o.pose.orientation.w = 1.0;
    mk_o.type = visualization_msgs::Marker::SPHERE_LIST;
    mk_o.scale.x = resolution_ * 0.5;
    mk_o.scale.y = resolution_ * 0.5;
    mk_o.scale.z = resolution_ * 0.5;
    mk_o.color.a = 0.5;
    mk_o.color.r = 1.0;
    mk_o.id = 1;
    mk_o.header.frame_id = "world";
    mk_o.header.stamp = ros::Time::now();

    ml_d.action = visualization_msgs::Marker::ADD;
    ml_d.pose.orientation.w = 1.0;
    ml_d.type = visualization_msgs::Marker::SPHERE_LIST;
    ml_d.scale.x = resolution_ * 0.5;
    ml_d.scale.y = resolution_ * 0.5;
    ml_d.scale.z = resolution_ * 0.5;
    ml_d.color.a = 0.5;
    ml_d.color.g = 1.0;
    ml_d.id = 2;
    ml_d.header.frame_id = "world";
    ml_d.header.stamp = ros::Time::now();


    geometry_msgs::Point pt;
    for(auto &p : occ){
        pt.x = p(0);
        pt.y = p(1);
        pt.z = p(2);
        mk_o.points.emplace_back(pt);
    }
    // for(auto &p : cam_occ){
    //     pair<uint16_t, uint16_t> idx;
    //     if(Pos2ImgIdx(idx, p)){
    //         cout<<"idx:"<<idx.first<<"  "<<idx.second<<"  p:"<<p.transpose()<<endl;
    //     }
    //     else{
    //         cout<<"no cast IMG:"<<p.transpose()<<endl;
    //     }
    // }
    debug_pub_.publish(mk_o);
    ros::Duration(0.005).sleep();
    for(auto &p : delete_occ){
        pt.x = p(0);
        pt.y = p(1);
        pt.z = p(2);
        ml_d.points.emplace_back(pt);
        RdImgFreeDebug(p);
    }

    debug_pub_.publish(ml_d);
    if(delete_occ.size() == 1){
        pair<uint16_t, uint16_t> idx;
        Eigen::Vector3f dp = sen_rot_inv_ * (delete_occ.front() - sen_tra_);
        Pos2ImgIdx(idx, dp);
        cout<<"idx.f:"<<idx.first<<" idx.s:"<<idx.second<<endl;    
        cout<<"delete_occ:"<<delete_occ.front().transpose()<<" dp:"<<dp.transpose()<<endl;    
        mk_o.color.b = 1.0;
        mk_o.color.a = 1.0;
        mk_o.id = 10;
        mk_o.points.clear();
        
        for(int i = 0; i < cam_occ.size(); i++){
            Eigen::Vector3f p = cam_occ[i];
            pair<uint16_t, uint16_t> idx_c;
            if(Pos2ImgIdx(idx_c, p)){

                if(idx_c.first == idx.first && idx_c.second == idx.second){
                    pt.x = occ[i](0);
                    pt.y = occ[i](1);
                    pt.z = occ[i](2);
                    mk_o.points.emplace_back(pt);
                    float r = p(0) * p(0) + p(1) * p(1) + p(2) * p(2);
                    cout<<"r:"<<sqrt(r)<<"   occ p:"<<p.transpose()<<endl;                    
                }
            }
        }
        debug_pub_.publish(mk_o);
        StopageDebug("1 size");
    }
}

void RipNeon::DebugShow(vector<Eigen::Vector3f> &read_blk, bool blk){
    if(read_blk.size() == 0) return;
    visualization_msgs::Marker mk_o;
    mk_o.action = visualization_msgs::Marker::ADD;
    mk_o.pose.orientation.w = 1.0;
    if(blk){
        mk_o.type = visualization_msgs::Marker::CUBE_LIST;
        mk_o.scale.x = blockscale_(0);
        mk_o.scale.y = blockscale_(1);
        mk_o.scale.z = blockscale_(2);
        mk_o.color.a = 0.5;
        mk_o.color.g = 1.0;
    }
    else{
        mk_o.type = visualization_msgs::Marker::SPHERE_LIST;
        mk_o.scale.x = resolution_ * 0.5;
        mk_o.scale.y = resolution_ * 0.5;
        mk_o.scale.z = resolution_ * 0.5;
        mk_o.color.a = 1.0;
        mk_o.color.r = 1.0;
    }

    mk_o.id = 1;
    mk_o.header.frame_id = "world";
    mk_o.header.stamp = ros::Time::now();

    geometry_msgs::Point pt;
    for(auto &b : read_blk){
        pt.x = b(0);
        pt.y = b(1);
        pt.z = b(2);
        mk_o.points.emplace_back(pt);
    }
    debug_pub_.publish(mk_o);
}

void RipNeon::DebugShow(vector<uint32_t> &blks, vector<uint16_t> &vs){
    visualization_msgs::Marker mk_o;
    mk_o.action = visualization_msgs::Marker::ADD;
    mk_o.pose.orientation.w = 1.0;
    mk_o.id = 1;
    mk_o.color.a = 1.0;
    mk_o.color.b = 0.2;
    mk_o.type = visualization_msgs::Marker::SPHERE_LIST;
    mk_o.scale.x = resolution_ * 0.5;
    mk_o.scale.y = resolution_ * 0.5;
    mk_o.scale.z = resolution_ * 0.5;
    mk_o.header.frame_id = "world";
    mk_o.header.stamp = ros::Time::now();
    geometry_msgs::Point pt;
    Eigen::Vector3f Pt;
    for(int i = 0 ; i < blks.size(); i++){
        Pt = Id2LocalPos(PVBs_[blks[i]]->origin_, vs[i]);
        pt.x = Pt(0);
        pt.y = Pt(1);
        pt.z = Pt(2);
        mk_o.points.emplace_back(pt);
    }
    debug_pub_.publish(mk_o);
    ROS_WARN("debug");
}


void RipNeon::StatisticV(const ros::TimerEvent &e){

}

