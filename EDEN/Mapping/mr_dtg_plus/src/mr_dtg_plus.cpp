#include <mr_dtg_plus/mr_dtg_plus.h>
using namespace DTGPlus;
// void MultiDtgPlus::init(ros::NodeHandle &nh, ros::NodeHandle &nh_private){
//     double topo_range;
//     std::string ns = ros::this_node::getName();
//     nh_private.param(ns + "/Exp/minX", origin_(0), 0.0);
//     nh_private.param(ns + "/Exp/minY", origin_(1), 0.0);
//     nh_private.param(ns + "/Exp/minZ", origin_(2), 0.0);
//     nh_private.param(ns + "/Exp/maxX", map_upbd_(0), 0.0);
//     nh_private.param(ns + "/Exp/maxY", map_upbd_(1), 0.0);
//     nh_private.param(ns + "/Exp/maxZ", map_upbd_(2), 0.0);
//     nh_private.param(ns + "/MR_DTG/Toporange", topo_range, 6.0);
//     nh_private.param(ns + "/MR_DTG/H_thresh", H_thresh_, 5.0);
//     nh_private.param(ns + "/MR_DTG/update_FF", update_FF_, false);
//     nh_private.param(ns + "/MR_DTG/resX", vox_scl_(0), 1.0);
//     nh_private.param(ns + "/MR_DTG/resY", vox_scl_(1), 1.0);
//     nh_private.param(ns + "/MR_DTG/resZ", vox_scl_(2), 1.0);
//     nh_private.param(ns + "/MR_DTG/show_edge_details", show_e_details_, false);
//     nh_private.param(ns + "/Exp/UAV_id", uav_id_, 10);
//     nh_private.param(ns + "/Exp/drone_num", drone_num_, 1);
//     nh_private.param(ns + "/block_map/sensor_max_range", sensor_range_, 5.0);


//     // EROI_ = NULL;
//     for(int dim = 0; dim < 3; dim++){
//         vox_num_(dim) = ceil((map_upbd_(dim) - origin_(dim)) / vox_scl_(dim));
//     }
//     H_depot_.resize(vox_num_(0) * vox_num_(1) * vox_num_(2));
//     F_depot_.resize(vox_num_(0) * vox_num_(1) * vox_num_(2));
//     cout<<"F_depot"<<F_depot_.size()<<"  "<<EROI_->f_grid_.size()<<endl;
//     cout<<vox_num_(0) * vox_num_(1) * vox_num_(2)<<endl;
//     cout<<"vox_num_:"<<vox_num_.transpose()<<"  map_upbd_ - origin_:"<<(map_upbd_-origin_).transpose()<<endl;
//     vector<uint16_t> id_l;
//     vector<pair<Eigen::Vector3d, Eigen::Vector3d>> bounds;
//     for(int i = 0; i < EROI_->f_grid_.size(); i++){
//         F_depot_[i] = make_shared<F_node>();
//         F_depot_[i]->eroi_ = &(EROI_->f_grid_[i]);
//         F_depot_[i]->id_ = i;
//         F_depot_[i]->center_ = EROI_->f_grid_[i].center_;
//         id_l.emplace_back(i);
//         bounds.push_back({F_depot_[i]->eroi_->up_, F_depot_[i]->eroi_->down_});
//     }
//     BM_->InitSwarmBlock(id_l, bounds);
//     map_upbd_ = LRM_->map_upbd_;

//     root_ = NULL;
//     cur_hid_ = uav_id_ + drone_num_;

//     eurange_ = topo_range/2 + H_thresh_/2; //heuristic range for local topo search
//     LRM_->SetEuRange(eurange_);
//     LRM_->SetTopoRange(topo_range);
//     LRM_->SetHthresh(H_thresh_); 

//     topo_pub_ = nh.advertise<visualization_msgs::MarkerArray>(ns + "/MR_DTG/Graph", 10);
//     debug_pub_ = nh.advertise<visualization_msgs::Marker>(ns + "/MR_DTG/Debug", 10);
//     show_timer_ = nh.createTimer(ros::Duration(0.2), &MultiDtgPlus::ShowAll, this);
//     if(drone_num_ > 1){
//         use_swarm_ = true;
//         swarm_timer_ = nh.createTimer(ros::Duration(SDM_->local_comm_intv_), &MultiDtgPlus::DTGCommunicationCallback, this);
//     }

//     cout<<"DTG origin:"<<origin_.transpose()<<endl;
//     cout<<"DTG topo_range:"<<topo_range<<endl;
//     cout<<"DTG H_thresh_:"<<H_thresh_<<endl;
//     cout<<"DTG eurange:"<<eurange_<<endl;
//     cout<<"DTG update_FF_:"<<update_FF_<<endl;
//     cout<<"DTG map_upbd_:"<<map_upbd_.transpose()<<endl;
//     cout<<"DTG vox_num_:"<<vox_num_.transpose()<<endl;
//     cout<<"DTG vox_scl_:"<<vox_scl_.transpose()<<endl;
//     cout<<"DTG uav_id_:"<<uav_id_<<endl;
//     cout<<"DTG drone_num_:"<<drone_num_<<endl;
// }

void MultiDtgPlus::GetHnodesBBX(const Eigen::Vector3d &upbd, const Eigen::Vector3d &lowbd, list<h_ptr> &H_list){
    Eigen::Vector3i low, up, it;
    list<f_ptr> f_l_temp;
    list<h_ptr> h_l_temp;
    Eigen::Vector3d u, l;
    for(int dim = 0; dim < 3; dim++){
        u(dim) = min(upbd(dim), map_upbd_(dim));
        l(dim) = max(lowbd(dim), map_lowbd_(dim));
    }
    // if(!InsideMap(upbd) && !InsideMap(lowbd)) return;
    low = GetVoxId3(lowbd);
    up = GetVoxId3(upbd);
    for(int dim = 0; dim < 3; dim++){
        low(dim) = max(0, low(dim));
        up(dim) = min(vox_num_(dim) - 1, up(dim));
    }

    // cout<<"lowbd:"<<lowbd.transpose()<<"  upbd:"<<upbd.transpose()<<endl;
    // cout<<"low:"<<low.transpose()<<"  up:"<<up.transpose()<<endl;
    // cout<<"low:"<<(low.cast<double>().cwiseProduct(vox_scl_) + origin_).transpose()<<"  up:"<<(up.cast<double>().cwiseProduct(vox_scl_) + origin_).transpose()<<endl;
    for(it(0) = low(0); it(0) <= up(0); it(0)++)
        for(it(1) = low(1); it(1) <= up(1); it(1)++)
            for(it(2) = low(2); it(2) <= up(2); it(2)++)
    {
        if(GetVox(it, h_l_temp, f_l_temp))
            H_list.insert(H_list.end(), h_l_temp.begin(), h_l_temp.end());
    }
}

void MultiDtgPlus::GetFnodesBBX(const Eigen::Vector3d &upbd, const Eigen::Vector3d &lowbd, list<f_ptr> &F_list){
    Eigen::Vector3i low, up, it;
    list<f_ptr> f_l_temp;
    list<h_ptr> h_l_temp;
    Eigen::Vector3d u, l;
    for(int dim = 0; dim < 3; dim++){
        u(dim) = min(upbd(dim), map_upbd_(dim));
        l(dim) = max(lowbd(dim), map_lowbd_(dim));
    }
    // if(!InsideMap(upbd) && !InsideMap(lowbd)) return;
    low = GetVoxId3(lowbd);
    up = GetVoxId3(upbd);
    for(int dim = 0; dim < 3; dim++){
        low(dim) = max(0, low(dim));
        up(dim) = min(vox_num_(dim) - 1, up(dim));
    }

    for(it(0) = low(0); it(0) <= up(0); it(0)++)
        for(it(1) = low(1); it(1) <= up(1); it(1)++)
            for(it(2) = low(2); it(2) <= up(2); it(2)++)
    {
        if(GetVox(it, h_l_temp, f_l_temp))
            F_list.insert(F_list.end(), f_l_temp.begin(), f_l_temp.end());
    }
}

void MultiDtgPlus::GetHnodesAndFnodesBBX(const Eigen::Vector3d &upbd, const Eigen::Vector3d &lowbd,
    list<h_ptr> &H_list, list<f_ptr> &F_list){
    Eigen::Vector3i low, up, it;
    list<f_ptr> f_l_temp;
    list<h_ptr> h_l_temp;
    // if(!InsideMap(upbd) && !InsideMap(lowbd)) return;
    low = GetVoxId3(lowbd);
    up = GetVoxId3(upbd);
    for(int dim = 0; dim < 3; dim++){
        low(dim) = max(0, low(dim));
        up(dim) = min(vox_num_(dim) - 1, up(dim));
    }

    for(it(0) = low(0); it(0) <= up(0); it(0)++)
        for(it(1) = low(1); it(1) <= up(1); it(1)++)
            for(it(2) = low(2); it(2) <= up(2); it(2)++)
    {
        if(GetVox(it, h_l_temp, f_l_temp)){
        H_list.insert(H_list.end(), h_l_temp.begin(), h_l_temp.end());
        F_list.insert(F_list.end(), f_l_temp.begin(), f_l_temp.end());
        }
    }
}


// void MultiDtgPlus::Update(const Eigen::Matrix4d &robot_pose, bool clear_x){
//     Eigen::Vector3d robot_pos, upbd, lowbd;
//     robot_pos = robot_pose.block(0, 3, 3, 1);
//     list<hhe_ptr> hhe_free_list, hhe_occ_list;               
//     list<hfe_ptr> hfe_free_list, hfe_occ_list;               
//     list<pair<int, pair<int, f_ptr>>> C_F_list;    //connect idx & Fnode
//     list<h_ptr> H_list_temp, L_H_list;
//     list<f_ptr> F_list_temp;
//     list<int> wild_f_list;
//     if(!InsideMap(robot_pos)){
//         ROS_WARN("id:%d out map! DTG refuse to update!", SDM_->self_id_);
//         return;
//     }

//     LRM_->SetEternalNode(robot_pos);
//     for(int dim = 0; dim < 3; dim++){
//         upbd(dim) = robot_pos(dim) + sensor_range_;
//         lowbd(dim) = robot_pos(dim) - sensor_range_;
//     }

//     GetHnodesAndFnodesBBX(upbd, lowbd, H_list_temp, F_list_temp);     
//     if(EROI_ != NULL){
//         /* get local fn for djkstra */
//         EROI_->GetWildGridsBBX(robot_pos, upbd - lowbd, LRM_->frontier_list_);
        
//         /* erase dead Fnodes */
//         for(auto &f : F_list_temp){
//             int idx = f->id_;
//             if(f->eroi_->f_state_ == 2){
//                 EraseFnodeFromGraph(f->center_, f->id_);
//             }
//             else if(f->vp_id_ != -1 && f->eroi_->local_vps_[f->vp_id_] != 1){
//                 // BlockEdge(f->hf_edge_);
//                 EraseEdge(f->hf_edge_);
//             }
//         }
//     }

//     /* check if Hnode is blocked */
//     for(auto &h : H_list_temp){
//         if(h->state_ == BLOCKED && LRM_->CheckAddHnode(h->pos_, h->id_)){
//             h->state_ = L_FREE;
//         }
//     }

//     /* update lrm topo relationships */
//     LRM_->prep_idx_ = cur_hid_;
//     root_ = NULL;

//     if(LRM_->UpdateLocalTopo(robot_pose, BM_->cur_pcl_, clear_x)){
//         if(!LRM_->IsFeasible(robot_pos)){//debug
//             ROS_WARN("id:%d Infeasible1! DTG refuse to update!", SDM_->self_id_);
//             return;
//         }
        
//         //generate new Hnode
//         root_ = CreateHnode(robot_pos);
//         if(root_ == NULL){
//             ROS_WARN("id:%d Infeasible2! DTG refuse to update!", SDM_->self_id_);
//             return;
//         }
//     }
//     else{
//         if(!GetVoxBBX(LRM_->cur_root_h_idx_, LRM_->node_scale_, LRM_->cur_root_h_id_, root_)){
//             cout<<LRM_->cur_root_h_idx_.transpose()<<":"<<LRM_->cur_root_h_id_<<";"<<LRM_->prep_idx_<<endl;
//             ROS_ERROR("error root h!");
//             ros::shutdown();
//         }
//     }

//     /* load edges between the robot and frontier viewpoint */
//     if(EROI_ != NULL){
//         Eigen::Vector3d vp_pos;
//         shared_ptr<F_node> fn;
//         r_f_edges_.clear();
//         local_f_list_.clear();
//         local_f_dist_list_.clear();

//         for(auto &f_p : LRM_->frontier_path_){
//             shared_ptr<ffe> r_e = make_shared<ffe>();
//             r_e->head_n_ = NULL;
//             bool gen_f = true;

//             EROI_->GetVpPos(f_p.first.first, f_p.second.first, vp_pos);
//             fn = F_depot_[f_p.first.first];

//             fn->vp_id_ = f_p.second.first;
//             r_e->tail_n_ = fn;
//             r_e->tail_ = fn->id_;
//             r_e->path_ = f_p.second.second;
//             r_e->path_.emplace_front(robot_pos);
//             r_e->path_.emplace_back(vp_pos);
//             local_f_list_.emplace_back(fn);
//             local_f_dist_list_.emplace_back(f_p.first.second);
//             r_f_edges_.emplace_back(r_e);

//             //Get local Fnodes
//             int c_id = LRM_->PostoId(vp_pos);
//             //debug
//             if(!LRM_->IsFeasible(vp_pos) || (LRM_->GetNode(c_id)==NULL)){
//                 cout<<LRM_->IsFeasible(vp_pos)<<"  "<<(LRM_->GetNode(c_id) == NULL)<<"   "<<(LRM_->GetNode(vp_pos) == NULL)<<endl;
//                 ROS_ERROR("error f vp");
//                 ros::shutdown();
//                 return;
//             }
//             C_F_list.push_back({c_id, {f_p.second.first, fn}});
//         }
//     }

//     /* clear occ edges */
//     for(auto &hid : LRM_->h_id_clear_){
//         h_ptr h_clear;
//         if(FindHnode(hid, h_clear)){
//             for(auto &e : h_clear->hh_edges_){
//                 if(!(e->e_flag_ & 1)){
//                     e->e_flag_ |= 1;
//                     if(LRM_->PathCheck(e->path_)){
//                         hhe_free_list.emplace_back(e);
//                     }
//                     else{
//                         // ROS_WARN("id:%d clear edge1", SDM_->self_id_);
//                         // cout<<"h:"<<e->head_<<"  t:"<<e->tail_<<endl;
//                         hhe_occ_list.emplace_back(e);
//                     }
//                 }
//             }
//             for(auto &e : h_clear->hf_edges_){
//                 if(!(e->e_flag_ & 1)){
//                     e->e_flag_ |= 1;
//                     if(LRM_->PathCheck(e->path_, true)){
//                         hfe_free_list.emplace_back(e);
//                     }
//                     else{
//                         // ROS_WARN("id:%d clear edge2", SDM_->self_id_);
//                         // cout<<"h:"<<e->head_<<"  t:"<<e->tail_<<"PN:"<<e->path_.size()<<endl;
//                         hfe_occ_list.emplace_back(e);
//                     }
//                 }
//             }
//             if(!LRM_->IsFeasible(h_clear->pos_)){
//                 h_clear->state_ = BLOCKED;
//             }
//         }
//         else{
//             ROS_ERROR("h missed %d", int(hid));
//         }
//     }

//     for(auto &e : hhe_free_list) e->e_flag_ &= 254;
//     for(auto &e : hfe_free_list) e->e_flag_ &= 254;
//     for(auto &e : hhe_occ_list) BlockEdge(e);
//     for(auto &e : hfe_occ_list) BlockEdge(e);
//     // for(auto &e : hhe_occ_list) EraseEdge(e);
//     // for(auto &e : hfe_occ_list) EraseEdge(e);

//     /* get local hn (inside djkstra area) */
//     h_ptr c_h;
//     for(auto &h : LRM_->id_Hpos_dist_){
//         if(GetVoxBBX(h.second, LRM_->node_scale_, h.first, c_h)){
//             L_H_list.push_back(c_h);
//         }
//         else{//debug
//             cout<<h.second.transpose()<<":"<<int(h.first)<<endl;
//             ROS_ERROR("error lhnode!");
//             ros::shutdown();
//         }
//     }

//     /* connect root and local hnodes */
//     list<Eigen::Vector3d> path, path_h1, path_h2, path_prune;
//     double length, length_h1, length_h2;
//     local_h_list_.clear();
//     local_h_dist_list_.clear();
//     local_h_list_.emplace_back(root_);
//     local_h_dist_list_.emplace_back(LRM_->GetRootHnCost());

//     for(auto &h : L_H_list){
//         if(LRM_->RetrieveHPath(h->pos_, root_->id_, path, length) && LRM_->PrunePath(path, path_prune, length)){
//             reverse(path_prune.begin(), path_prune.end());
//             if(root_ == h){
//                 ROS_ERROR("error connect1 %d %d", root_->id_, h->id_);
//                 ros::shutdown();
//             }
//             ConnectHH(root_, h, path_prune, length);
//             // if((root_->pos_ - path_prune.front()).norm() > 1e-3) ROS_ERROR("error connect1");
//             // if((h->pos_ - path_prune.back()).norm() > 1e-3) ROS_ERROR("error connect2");
//             local_h_list_.emplace_back(h);
//             local_h_dist_list_.emplace_back(length);
//             // local_h_list_
//         }
//         else{
//             ROS_ERROR("Error hpath0!");
//             ros::shutdown();
//         }
//     }

//     /* connect hands-shaking hnodes */
//     for(auto &CH : LRM_->id_idx_dist_){
//         if(LRM_->RetrieveHPath(CH.second.first, CH.first, path_h1, length_h1)){
//             if(LRM_->RetrieveHPath(CH.second.first, root_->id_, path_h2, length_h2)){
//                 length = length_h1 + length_h2;
//                 path = path_h1;
//                 path.pop_front();
//                 for(auto &p : path_h2){
//                     path.emplace_front(p);
//                 }
//                 if(LRM_->PrunePath(path, path_prune, length)){
//                     if(GetVoxBBX(path.back(), LRM_->node_scale_, CH.first, c_h)){//debug
//                         if(root_ == c_h){
//                             ROS_ERROR("error connect2 %d %d", root_->id_, c_h->id_);
//                             ros::shutdown();
//                         }
//                         ConnectHH(root_, c_h, path_prune, length);
//                         // if((root_->pos_ - path_prune.front()).norm() > 1e-3) ROS_ERROR("error connect3");
//                         // if((c_h->pos_ - path_prune.back()).norm() > 1e-3) ROS_ERROR("error connect4");
//                     }
//                     else{
//                         if(FindHnode(CH.first, c_h)){
//                             cout<<"id:"<<CH.first<<"pos:"<<c_h->pos_.transpose()<<endl;
//                             cout<<path.front().transpose()<<"p:"<<path.back().transpose()<<endl;
//                         }
//                         ROS_ERROR("Error Hnode!");
//                         ros::shutdown();
//                     }
//                 }
//                 else{
//                     ROS_ERROR("error hpath1.5!");
//                     ros::shutdown();
//                 }
//             }
//             else{//debug
//                 cout<<"root:"<<int(root_->id_)<<"  h:"<<int(CH.first)<<endl;
//                 cout<<LRM_->id_idx_dist_.size()<<"  "<<CH.second.first<<endl;
//                 ROS_ERROR("error hpath2!");
//                 ros::shutdown();
//             }
//         }
//         else{//debug
//             ROS_ERROR("error hpath1!");
//             ros::shutdown();
//         }
//     }

//     /* connect fnodes and rootH */
//     list<Eigen::Vector3d> path_f;
//     double length_f;
//     for(auto &CF : C_F_list){
//         if(LRM_->RetrieveHPath(CF.first, root_->id_, path_f, length_f) && LRM_->PrunePath(path_f, path_prune, length_f)){

//             reverse(path_prune.begin(), path_prune.end());
//             Eigen::Vector3d vp_pos;
//             //debug
//             if(!EROI_->GetVpPos(CF.second.second->id_, CF.second.first, vp_pos)){
//                 ROS_ERROR("error fpath2!");
//                 ros::shutdown();
//             }
//             path_prune.push_back(vp_pos);
//             ConnectHF(root_, CF.second.second, CF.second.first, path_prune, length_f);
//         }
//         else{//debug
//             ROS_ERROR("error fpath!");
//             ros::shutdown();
//         }
//     }
// }

void MultiDtgPlus::BfUpdate(const Eigen::Vector3d &robot_pos){
        // to be modified
    // LRM_->ReloadNodesBf(robot_pos);
    // LRM_->SetNodeStatesBf(u2f_list, u2o_list, o2f_list, f2o_list);
    ros::WallTime t0 = ros::WallTime::now();
    list<h_ptr> H_list;
    list<pair<uint32_t, Eigen::Vector3d>> search_origins;
    GetHnodesBBX(LRM_->local_map_upbd_, LRM_->local_map_lowbd_, H_list);
    // ROS_WARN("BfUpdate0.01");

    /* clear old dead hh edges */
    list<hhe_ptr> hhe_clear_list;
    for(auto &h : H_list){
        // cout<<"h id:"<<h->id_<<endl;
        // cout<<"h pos:"<<h->pos_.transpose()<<endl;
        search_origins.push_back({h->id_, h->pos_});
        // if(h == NULL){
        //     StopageDebug("h == NULL");
        // }
        // cout<<"h_e num:"<<h->hh_edges_.size()<<endl;
        for(auto &hhe : h->hh_edges_){
            // ROS_WARN("BfUpdate0.05");
            // if(hhe == NULL){
            //     StopageDebug("hhe == NULL");
            // }
            // cout<<"h_e len:"<<hhe->path_.size()<<endl;

            // ROS_WARN("BfUpdate0.075");

            if(hhe->e_flag_ & 1) continue;
            int s = LRM_->PathCheck(hhe->path_, true);
            // ROS_WARN("BfUpdate0.1");
            hhe->e_flag_ |= 1; 
            if(s == 2){
                hhe->e_flag_ |= 64; 
                // EraseEdge(hhe);
            }
            else if(s == 0){
                hhe->e_flag_ |= 4; 
            }
            else {
                hhe->e_flag_ &= 251; 
            }
            hhe_clear_list.emplace_back(hhe);
            // ROS_WARN("BfUpdate0.2");
        }
        // ROS_WARN("BfUpdate0.3");

    }
    // ROS_WARN("BfUpdate0.4");

    for(auto &hhe : hhe_clear_list){ 
        hhe->e_flag_ &= 254;
        if(hhe->e_flag_ & 64){
            EraseEdge(hhe);
        }
    }
    // cout<<"DTG update hh paths check:"<<(ros::WallTime::now() - t0).toSec()<<endl;
    // t0 = ros::WallTime::now();
    // ROS_WARN("BfUpdate0.5");

    /* voronoi partition of hnodes */
    list<Eigen::Vector3i> margins;
    list<double> lenghts_new;
    tr1::unordered_map<uint32_t, uint32_t> h2i;
    vector<h_ptr> H_vec;
    uint32_t idx = 0;
    list<list<Eigen::Vector3d>> paths_new;
    list<pair<uint32_t, uint32_t>> hh_idx_edges;
    LRM_->ClearTopo();
    // ROS_WARN("VoronoiPartitionSearch1");
    LRM_->VoronoiPartitionSearch(search_origins, margins);
    // ROS_WARN("VoronoiPartitionSearch2");

    auto nr =  LRM_->GlobalPos2LocalNode(robot_pos);
    // ROS_WARN("BfUpdate1");
    if(nr->path_g_ > H_thresh_){ // create a node if current hnode is too far from the closest hnode
        // cout<<"nr->path_g_:"<<nr->path_g_<<endl;
        // cout<<"nr->ROOT_:"<<nr->root_id_<<endl;
        // if(cur_hid_ == 2) cout<<"fea:"<<LRM_->IsFeasible(H_list_.front()->pos_)<<endl;
        root_ = CreateHnode(robot_pos);
        if(root_ != NULL){
            // if(root_->id_ == 2){
            //     StopageDebug("root id == 2 debug");
            // }
            search_origins.clear();
            search_origins.push_back({root_->id_, root_->pos_});
            ROS_WARN("VoronoiPartitionSearch3");
            LRM_->VoronoiPartitionSearch(search_origins, margins);
            ROS_WARN("VoronoiPartitionSearch4");
            H_list.emplace_back(root_);

        }

    }
    // ROS_WARN("BfUpdate2");

    for(auto &h : H_list){
        H_vec.emplace_back(h);
        h2i[h->id_] = idx;
        idx++;
    }
    // ROS_WARN("BfUpdate3");
    // cout<<"DTG update voronoi partition:"<<(ros::WallTime::now() - t0).toSec()<<endl;
    // t0 = ros::WallTime::now();

    LRM_->VoronoiPathRetrieve(h2i, margins, hh_idx_edges, paths_new, lenghts_new);

    list<Eigen::Vector3d> path_old;
    auto hhi = hh_idx_edges.begin();
    auto pni = paths_new.begin();
    auto ln = lenghts_new.begin();
    bool swarm_pub_hh;
    for(;hhi != hh_idx_edges.end(); hhi++, pni++, ln++){
        swarm_pub_hh = false;
        if(pni->empty()){
            for(auto &path : paths_new){
                cout<<"path s:"<<path.size()<<endl;
            }
            StopageDebug("BfUpdate pni->empty()");
        }
        ConnectHH(H_vec[hhi->first], H_vec[hhi->second], *pni, path_old, *ln);
        if(path_old.empty()) swarm_pub_hh = true;
        if(!path_old.empty() && !LRM_->CheckPathHomo(*pni, path_old)) swarm_pub_hh = true;
    }
    // ROS_WARN("BfUpdate4");
    // cout<<"DTG update VoronoiPathRetrieve:"<<(ros::WallTime::now() - t0).toSec()<<endl;
    // t0 = ros::WallTime::now();

    /* clear dead hfedges */
    list<hfe_ptr> hfe_clear_list;
    for(auto &h : H_list){
        for(auto &hfe : h->hf_edges_){
            if(hfe->e_flag_ & 1) continue;
            int s = LRM_->PathCheck(hfe->path_, true);
            hfe->e_flag_ |= 1;
            if(s == 2){
                hfe->e_flag_ |= 64; 
                // EraseFnodeFromGraph(hfe->tail_n_->fid_, hfe->tail_n_->vid_);
            }
            else if(s == 0){
                hfe->e_flag_ |= 4; 
            }
            else {
                hfe->e_flag_ &= 251; 
            }
            hfe_clear_list.emplace_back(hfe);
        }
    }
    // ROS_WARN("BfUpdate5");

    // cout<<"DTG update clear dead hfedges:"<<(ros::WallTime::now() - t0).toSec()<<endl;
    // t0 = ros::WallTime::now();

    /* clear dead hfedges, those fns that are not connected to local hns */
    list<f_ptr> F_list; 
    GetFnodesBBX(LRM_->local_map_lowbd_, LRM_->local_map_upbd_, F_list);
    list<Eigen::Vector3d> path_f;
    for(auto &f : F_list){
        if(f->hf_edge_ == NULL) continue;
        f->hf_edge_->e_flag_ |= 1;
        int s = LRM_->PathCheck(f->hf_edge_->path_, true);
        if(s == 2){
            // EraseEdge(f->hf_edge_);
            f->hf_edge_->e_flag_ |= 64; 
            // EraseFnodeFromGraph(f->fid_, f->vid_);
        }
        else if(s == 0){
            f->hf_edge_->e_flag_ |= 4; 
        }
        else {
            f->hf_edge_->e_flag_ &= 251; 
        }
        hfe_clear_list.emplace_back(f->hf_edge_);
    }
    for(auto &hfe : hfe_clear_list){ 
        hfe->e_flag_ &= 254;
        if(hfe->e_flag_ & 64){
            EraseFnodeFromGraph(hfe->tail_n_->fid_, hfe->tail_n_->vid_);
        }
    }
    // ROS_WARN("BfUpdate6");
    // cout<<"DTG update clear dead hfedges:"<<(ros::WallTime::now() - t0).toSec()<<endl;
    // t0 = ros::WallTime::now();


    /* update paths between hnodes and fnodes */
    double length_f;
    bool swarm_pub_hf;
    vector<pair<uint32_t, uint8_t>> vps;
    vector<uint32_t> root_ids;
    vector<pair<uint32_t, uint8_t>> erois; // <fid, state>
    vector<double> distances;
    Eigen::Vector3d vp_pos;
    Eigen::Vector4d vp_pose;
    DTGPlus::h_ptr hn;
    DTGPlus::f_ptr fn;
    // ROS_WARN("BfUpdate6.1");

    EROI_->GetLocalValidVps(erois, vps, root_ids, distances);
    // ROS_WARN("BfUpdate6.2");

    for(auto &eroi : erois){
        if(eroi.second == 2){
            // cout<<"erase dead:"<<eroi.first<<endl;
            EraseEROIFromGraph(eroi.first);
        }
    }
    // ROS_WARN("BfUpdate6.5");

    for(int i = 0; i < vps.size(); i++){
        auto &vp = vps[i];
        auto &r_id = root_ids[i];
        auto &d = distances[i];
        vp_pose = EROI_->vps_[vp.second];
        vp_pose.head(3) += EROI_->EROI_[vp.first].center_;
        vp_pos = vp_pose.head(3); 
        auto hi = h2i.find(r_id);
        // if(hi == h2i.end()) {
        //     StopageDebug("DTG BfUpdate hi == h2i.end()");
        //     continue;
        // }
        hn = H_vec[hi->second];
        swarm_pub_hf = false;

        if(!FindFnode(vp_pos, vp.first, vp.second, fn)){ // if fn not exist, try to create a new fn
            //generate fn
            // fn = CreateFnode(vp_pos, vp.first, vp.second);
            CreateConnectFnode(vp_pos, vp.first, vp.second, d, hn);
        }
        else if(LRM_->RetrieveHPath(vp_pos, path_f, length_f, true)){ // if exists, try to build a shorter edge
            ConnectHF(hn, fn, vp.second, path_f, path_old, d);
            // cout<<"ConnectHF:"<<hn->id_<<" "<<fn->fid_<<endl;
        }
        // else {
        //     Eigen::Vector3i c;
        //     LRM_->PostoId3(vp_pos, c);
        //     auto n = LRM_->GlobalPos2LocalNode(c);
        //     cout<<(n == LRM_->Outnode_)<<endl;
        //     n = LRM_->GlobalPos2LocalNode(vp_pos);
        //     cout<<(n == LRM_->Outnode_)<<endl;
        //     cout<<"vp_pos:"<<vp_pos.transpose()<<endl;
        //     cout<<"InsideLocalMap(start):"<<LRM_->InsideLocalMap(vp_pos)<<endl;
        //     StopageDebug("DTG BfUpdate RetrieveHPath failed");
        // }
        if(path_old.empty()) swarm_pub_hf = true;
        if(!path_old.empty() && !LRM_->CheckPathHomo(path_f, path_old)) swarm_pub_hf = true;
    }


    for(auto &h : H_list){
        if(!h->hf_edges_.empty() && h->h_flags_ & 8){
            h->h_flags_ |= 8;
            update_list_.emplace_back(h);
        }
    }
    // cout<<"DTG update update paths between hnodes and fnodes:"<<(ros::WallTime::now() - t0).toSec()<<endl;
    // t0 = ros::WallTime::now();
    // NodesDebug();
}

void MultiDtgPlus::RemoveVp(uint32_t const &f_id, uint8_t const &v_id, bool broad_cast){
    EraseEROIFromGraph(f_id);
    EROI_->RemoveVp(f_id, v_id, broad_cast);
}

void MultiDtgPlus::RemoveVp(const Eigen::Vector3d &center, int const &f_id, int const &v_id, bool broad_cast){
    f_ptr fn;
    /* remove vp in fronter grid */
    EROI_->RemoveVp(f_id, v_id, broad_cast);
    // if(SDM_->is_ground_) return;
}

// void MultiDtgPlus::DTGCommunicationCallback(const ros::TimerEvent &e){
//     // ROS_WARN("id:%d DTGCommunicationCallback", SDM_->self_id_);
//     /* update Fn */
//     vector<uint8_t> vp_states;
//     while (!SDM_->swarm_sub_fn_.empty())
//     {
//         auto &f_msg = SDM_->swarm_sub_fn_.front();
//         if(0 > f_msg.f_id || F_depot_.size() <= f_msg.f_id){
//             cout<<"invalid fn:"<<int(f_msg.f_id)<<" max:"<<F_depot_.size()<<endl;
//         }
//         else{
//             if(F_depot_[f_msg.f_id]->eroi_->f_state_ != 2){
//                 if(f_msg.alive){
//                     SDM_->GetFnvp(vp_states, f_msg.vp_flags);
//                     if(F_depot_[f_msg.f_id]->eroi_->f_state_ == 0) F_depot_[f_msg.f_id]->eroi_->f_state_ = 1;
//                     for(int i = 0; i < vp_states.size(); i++){
//                         if(vp_states[i] == 2 && F_depot_[f_msg.f_id]->eroi_->local_vps_[i] == 1){
//                             RemoveVp(F_depot_[f_msg.f_id]->eroi_->center_, f_msg.f_id, i, false);
//                         }
//                         else if(vp_states[i] == 2 && F_depot_[f_msg.f_id]->eroi_->local_vps_[i] == 0){
//                             RemoveVp(F_depot_[f_msg.f_id]->eroi_->center_, f_msg.f_id, i, false);
//                         }
//                         else if(vp_states[i] == 1 && F_depot_[f_msg.f_id]->eroi_->local_vps_[i] == 0){
//                             F_depot_[f_msg.f_id]->eroi_->local_vps_[i] = 1;
//                         }
//                     }
//                     int alive_num = 0;
//                     if(F_depot_[f_msg.f_id]->eroi_->f_state_ != 2){
//                         for(auto &vs : F_depot_[f_msg.f_id]->eroi_->local_vps_) if(vs != 2) alive_num++;
//                         if(alive_num < EROI_->min_vp_num_){
                            
//                             EROI_->SetExplored(f_msg.f_id);
//                             if(!SDM_->is_ground_) SDM_->SetDTGFn(f_msg.f_id, F_depot_[f_msg.f_id]->eroi_->local_vps_, 1, false);
//                         }
//                         else EROI_->AddShow(f_msg.f_id);
//                     }
                    
//                     if(f_msg.need_help && F_depot_[f_msg.f_id]->eroi_->f_state_ == 1 && 
//                         F_depot_[f_msg.f_id]->hf_edge_ != NULL && F_depot_[f_msg.f_id]->hf_edge_->e_flag_ & 16){
//                         vector<uint32_t> path;
//                         int p_idx;
//                         for(auto &p : F_depot_[f_msg.f_id]->hf_edge_->path_){
//                             p_idx = LRM_->PostoId(p);
//                             if(path.empty() || p_idx != path.back())
//                                 path.push_back(p_idx);
//                         }
//                         SDM_->SetDTGHFEdge(F_depot_[f_msg.f_id]->hf_edge_->head_, 
//                                 F_depot_[f_msg.f_id]->hf_edge_->tail_, F_depot_[f_msg.f_id]->vp_id_, path);
//                     }

//                 }
//                 else{
//                     // cout<<"kill:"<<int(f_msg.f_id)<<endl;
//                     if(f_msg.need_help){
//                         BM_->SendSwarmBlockMap(f_msg.f_id, false);
//                     }
//                     EROI_->SetExplored(f_msg.f_id);
//                     // debug_pts_.emplace_back(F_depot_[f_msg.f_id]->eroi_->center_);
//                     // Debug(debug_pts_);
//                     EraseFnodeFromGraph(F_depot_[f_msg.f_id]->eroi_->center_, f_msg.f_id);
//                 }
//             }
//         }
//         SDM_->swarm_sub_fn_.pop_front();
//     }

//     /* update Hn */
//     h_ptr hn;
//     while (!SDM_->swarm_sub_hn_.empty()){
//         auto &h_msg = SDM_->swarm_sub_hn_.front();
//         Eigen::Vector3d pos = LRM_->IdtoPos(h_msg.pos_idx);
//         if(FindHnode(pos, h_msg.h_id, hn)){
//             ROS_WARN("id:%d exists%d", SDM_->self_id_, h_msg.h_id);
//         }
//         else{
//             CreateSwarmHnode(pos, h_msg.h_id);
//         }
//         SDM_->swarm_sub_hn_.pop_front();
//     }

//     /* update HFe */
//     for(list<swarm_exp_msgs::DtgHFEdge>::iterator hfe_it = SDM_->swarm_sub_hfe_.begin(); hfe_it != SDM_->swarm_sub_hfe_.end(); hfe_it++){
//         if(hfe_it->f_id < F_depot_.size()){
//             list<Eigen::Vector3d> path;
//             h_ptr hn_head;
//             f_ptr fn_tail = F_depot_[hfe_it->f_id];
//             Eigen::Vector3d h_pos, t_pos;
//             h_pos = LRM_->IdtoPos(hfe_it->points_idx.front());
//             t_pos = fn_tail->eroi_->center_;
//             if(FindHnode(h_pos, hfe_it->h_id, hn_head) && hfe_it->f_id < F_depot_.size()){
//                 // if(!hfe_it->erase){
//                 list<Eigen::Vector3d> path;
//                 for(auto &p : hfe_it->points_idx) path.emplace_back(LRM_->IdtoPos(p));
//                 if(!ConnectSwarmHF(hn_head, fn_tail, path, hfe_it->vp_id)){
//                     // ROS_ERROR("id:%d error swarm connect hf", SDM_->self_id_);
//                 }
//                 // }
//                 // else{
//                 //     EraseEdge(fn_tail->hf_edge_, false);
//                 // }
//                 list<swarm_exp_msgs::DtgHFEdge>::iterator erase_it = hfe_it;
//                 hfe_it--;
//                 SDM_->swarm_sub_hfe_.erase(erase_it);
//             }
//             else{
//                 ROS_WARN("id:%d fail find hn fn, dont connect", SDM_->self_id_);
//                 cout<<int(hfe_it->h_id)<<"  pos:"<<h_pos.transpose()<<"  "<<FindHnode(h_pos, hfe_it->h_id, hn_head)<<endl;
//                 cout<<int(hfe_it->f_id)<<endl;
//                 cout<<int(fn_tail->eroi_->f_state_)<<" gt pos:"<<fn_tail->eroi_->center_.transpose()<<endl;
//             }
//         }
//         else{
//             list<swarm_exp_msgs::DtgHFEdge>::iterator erase_it = hfe_it;
//             hfe_it--;
//             SDM_->swarm_sub_hfe_.erase(erase_it);
//         }
//     }

//     /* update HHe */
//     for(list<swarm_exp_msgs::DtgHHEdge>::iterator hhe_it = SDM_->swarm_sub_hhe_.begin(); hhe_it != SDM_->swarm_sub_hhe_.end(); hhe_it++){
//         h_ptr hn_head, hn_tail;
//         Eigen::Vector3d h_pos, t_pos;
//         h_pos = LRM_->IdtoPos(hhe_it->points_idx.front());
//         t_pos = LRM_->IdtoPos(hhe_it->points_idx.back());
//         if(FindHnode(h_pos, hhe_it->head_h_id, hn_head) && FindHnode(t_pos, hhe_it->tail_h_id, hn_tail)){
//             list<Eigen::Vector3d> path;
//             // if(!hhe_it->erase){
//             for(auto &p : hhe_it->points_idx) path.emplace_back(LRM_->IdtoPos(p));
//             if(!ConnectSwarmHH(hn_head, hn_tail, path)){
//                 // ROS_ERROR("fail hh");
//             }
//             // else{
//             //     // ROS_WARN("success connect %d, %d", hn_head->id_, hn_tail->id_);
//             // }
//             // }
//             // else{
//             //     for(auto &e : hn_head->hh_edges_){
//             //         if((e->head_ == hn_head->id_ && e->tail_ == hn_tail->id_) ||
//             //             (e->tail_ == hn_head->id_ && e->head_ == hn_tail->id_)){
//             //             EraseEdge(e, false);
//             //         }
//             //     }
//             // }
//             list<swarm_exp_msgs::DtgHHEdge>::iterator erase_it = hhe_it;
//             hhe_it--;
//             SDM_->swarm_sub_hhe_.erase(erase_it);
//         }
//         else{
//             ROS_WARN("id:%d fail find hn hn, dont connect", SDM_->self_id_);
//             cout<<int(hhe_it->head_h_id)<<"  pos:"<<h_pos.transpose()<<FindHnode(h_pos, hhe_it->head_h_id, hn_head)<<endl;
//             cout<<int(hhe_it->tail_h_id)<<"  pos:"<<t_pos.transpose()<<FindHnode(t_pos, hhe_it->tail_h_id, hn_tail)<<endl;
//         }
//     }
//     if(SDM_->swarm_sub_hhe_.size() > 0){//debug
//         ROS_ERROR("error hhe swarm");
//     }


// }

void MultiDtgPlus::NodesDebug(){
    for(auto &h : H_list_){
        for(auto e1 = h->hh_edges_.begin(); e1 != h->hh_edges_.end(); e1++){
            auto e2 = e1;
            e2++;
            for(; e2 != h->hh_edges_.end(); e2++){
                if((*e1)->head_n_->id_ == (*e2)->tail_n_->id_ && (*e1)->tail_n_->id_ == (*e2)->head_n_->id_) {
                    StopageDebug("NodesDebug error hh edge00");
                }
                else if((*e1)->tail_n_->id_ == (*e2)->tail_n_->id_ && (*e1)->head_n_->id_ == (*e2)->head_n_->id_){
                    StopageDebug("NodesDebug error hh edge000");
                }
            }
            if(h->id_ == (*e1)->head_n_->id_){
                bool found = false;
                for(auto e3 : (*e1)->tail_n_->hh_edges_){
                    if(e3->head_n_->id_ == h->id_) {
                        found = true;
                        break;
                    }
                }
                if(!found){
                    StopageDebug("NodesDebug error hh edge1");
                }
            }
            else{
                bool found = false;
                for(auto e3 : (*e1)->head_n_->hh_edges_){
                    if(e3->tail_n_->id_ == h->id_) {
                        found = true;
                        break;
                    }
                }
                if(!found){
                    StopageDebug("NodesDebug error hh edge2");
                }
            }
        }

        for(auto hfe = h->hf_edges_.begin(); hfe != h->hf_edges_.end(); hfe++){

            if((*hfe)->head_n_->id_ != h->id_){
                StopageDebug("NodesDebug error hf edge1");
            }
            auto hfe2 = hfe;
            hfe2++;
            for(; hfe2 != h->hf_edges_.end(); hfe2++){
                if((*hfe2)->tail_n_->fid_ == (*hfe)->tail_n_->fid_) {
                    cout<<"f1:"<<(*hfe)->tail_n_->fid_<<" vid:"<<int((*hfe)->tail_n_->vid_)<<endl;
                    cout<<"f1:"<<(*hfe2)->tail_n_->fid_<<" vid:"<<int((*hfe2)->tail_n_->vid_)<<endl;
                    StopageDebug("NodesDebug error hf edge2");
                }
            }

            if((*hfe)->tail_n_->fid_ >= EROI_->EROI_.size()){
                StopageDebug("NodesDebug error (*hfe)->tail_n_->fid_ >= EROI_->EROI_.size()");
            }

            if(EROI_->EROI_[(*hfe)->tail_n_->fid_].f_state_ >=2){
                cout<<"EROI_->EROI_[(*hfe)->tail_n_->fid].f_state_:"<<int(EROI_->EROI_[(*hfe)->tail_n_->fid_].f_state_)<<endl;
                cout<<"(*hfe)->tail_n_->fid_:"<<int((*hfe)->tail_n_->fid_)<<endl;
                cout<<"(*hfe)->tail_n_->vid:"<<int((*hfe)->tail_n_->vid_)<<endl;
                cout<<"h id:"<<int((*hfe)->head_n_->id_)<<endl;
                StopageDebug("NodesDebug EROI_->EROI_[(*hfe)->tail_n_->vid_] >=2");
            }

            // if(EROI_->EROI_[(*hfe)->tail_n_->fid_].local_vps_[(*hfe)->tail_n_->vid_] != 1){
            //     cout<<"EROI_->EROI_[(*hfe)->tail_n_->fid].f_state_:"<<int(EROI_->EROI_[(*hfe)->tail_n_->fid_].local_vps_[(*hfe)->tail_n_->vid_])<<endl;
            //     cout<<"(*hfe)->tail_n_->fid_:"<<int((*hfe)->tail_n_->fid_)<<endl;
            //     cout<<"(*hfe)->tail_n_->vid_:"<<int((*hfe)->tail_n_->vid_)<<endl;
            //     cout<<"h id:"<<int((*hfe)->head_n_->id_)<<endl;
            //     for(auto &d : EROI_->dead_fnodes_){
            //         cout<<d.first<<"  "<<int(d.second)<<endl;
            //     }
            //     StopageDebug("NodesDebug EROI_->EROI_[(*hfe)->tail_n_->vid_].local_vps_[(*hfe)->tail_n_->vid_] != 1");
            // }

            if(!EROI_->CheckValidFnode((*hfe)->tail_n_->fid_, (*hfe)->tail_n_->vid_)){
                cout<<"f1:"<<(*hfe)->tail_n_->fid_<<" vid:"<<int((*hfe)->tail_n_->vid_)<<endl;
                cout<<"vpstate:"<<int(EROI_->EROI_[(*hfe)->tail_n_->fid_].local_vps_[(*hfe)->tail_n_->vid_])<<endl;
                cout<<"h id:"<<int((*hfe)->head_n_->id_)<<endl;
                StopageDebug("NodesDebug EROI_->CheckValidFnode((*hfe)->tail_n_->fid_, (*hfe)->tail_n_->vid_)");
            }

        }

    }

}

bool MultiDtgPlus::TspApproxiPlan(const Eigen::Vector3d &ps, vector<h_ptr> &route_h, vector<Eigen::Vector3d> &path2fh, double &d1){
    /* find p to local hns */
    list<h_ptr> H_list, hvalid;
    GetHnodesBBX(LRM_->local_map_upbd_, LRM_->local_map_lowbd_, H_list);
    vector<Eigen::Vector3d> tars;
    vector<double> dist;
    vector<h_ptr> p_hs;
    vector<vector<Eigen::Vector3d>> or_paths;
    vector<vector<Eigen::Vector3d>> hs_paths;
    vector<double> g0_hs;
    bool fea = false;
    for(auto &h : H_list){
        tars.emplace_back(h->pos_);
        hvalid.emplace_back(h);
    }
    // cout<<"ps:"<<ps.transpose()<<endl;
    cout<<"psfea:"<<LRM_->IsFeasible(ps)<<endl;
    LRM_->ClearTopo();
    // ROS_WARN("search 1");
    LRM_->GetDists(ps, tars, dist, or_paths, false);
    auto d = dist.begin();
    auto hi = hvalid.begin();
    auto pti = or_paths.begin();
    // cout<<"dist size:"<<dist.size()<<endl;
    for(; d != dist.end(); d++, hi++, pti++){
        // cout<<"dist:"<<*d<<endl;
        if(*d < 999998.0){
            fea = true;
            p_hs.emplace_back(*hi);
            g0_hs.emplace_back(*d);
            hs_paths.emplace_back(*pti);
            // break;
        }
    }
    // cout<<"fea:"<<fea<<endl;
    // cout<<"p_hs:"<<p_hs.size()<<endl;
    if(!fea) return false;
    /* find p to all active hns */
    vector<h_ptr> tarhs;
    vector<pair<double, list<h_ptr>>> paths;
    ros::WallTime t0 = ros::WallTime::now();

    ParallelDijkstra(p_hs, g0_hs, tarhs, paths);
    cout<<"ParallelDijkstra time:"<<(ros::WallTime::now() - t0).toSec()<<endl;
    t0 = ros::WallTime::now();
    if(tarhs.empty()){
        cout<<"tarhs empty"<<endl;
        return false;
    }

    /* prim algorithm */
    Eigen::MatrixXd dist_mat;
    vector<int> parent; 
    vector<vector<int>> branches;
    tr1::unordered_map<int, int> banned_branch;
    int ha_idx;
    int banned_leaf;
    int cur_idx;

    dist_mat.resize(tarhs.size() + 1, tarhs.size() + 1);
    dist_mat.setOnes();
    dist_mat *= 999999.0;
    dist_mat.col(0).setZero();
    dist_mat.diagonal().setZero();

    for(int i = 0; i < tarhs.size() + 1; i++){
        // if(i != 0)
            // cout<<"h id:"<<tarhs[i-1]->id_<<endl;

        for(int j = i + 1; j < tarhs.size() + 1; j++){
            if(i == 0) {
                dist_mat(i, j) = paths[j - 1].first;
                // cout<<"h:"<<tarhs[j - 1]->id_<<" t:"<<ros::WallTime::now().toSec() - tarhs[j - 1]->last_maintain_t_<<endl;
            }
            else{
                if(tarhs[i-1]->id_ > tarhs[j-1]->id_) ha_idx = tarhs[j-1]->id_ * 10000000 + tarhs[i-1]->id_;
                else ha_idx = tarhs[i-1]->id_ * 10000000 + tarhs[j-1]->id_;
                auto d = h_dist_map_.find(ha_idx);
                if(d != h_dist_map_.end()){
                    dist_mat(i, j) = d->second.first;
                    dist_mat(j, i) = d->second.first;
                }

            }
        }
    }
    // cout<<"dist_mat:\n"<<dist_mat<<endl;
    cout<<"mat load time:"<<(ros::WallTime::now() - t0).toSec()<<endl;
    t0 = ros::WallTime::now();
    /* create tree */
    Prim(dist_mat, parent, branches);
    cout<<"Prim time:"<<(ros::WallTime::now() - t0).toSec()<<endl;
    t0 = ros::WallTime::now();
    // cout<<"tarhs.size():"<<tarhs.size()<<endl;
    // cout<<"paths.size():"<<paths.size()<<endl;
    // cout<<"branches.size():"<<branches.size()<<endl;
    // cout<<"parent.size():"<<parent.size()<<endl;
    // cout<<"dist_mat.size():"<<dist_mat.size()<<endl;
    // cout<<"dist_mat:\n"<<dist_mat<<endl;

    /* ban the worst branch, whitch will be finally executed */
    double max_dist = -0.1;
    for(int i = 0; i < tarhs.size(); i++){
        if(branches[i + 1].empty()){
            if(paths[i].first > max_dist){
                max_dist = paths[i].first;
                banned_leaf = i + 1;
            }
            // cout<<"paths[i].first:"<<paths[i].first<<endl;
        }
    }

    cur_idx = banned_leaf;
    while(1){
        if(cur_idx != 0){
            if(cur_idx >= parent.size() || cur_idx < 0){
                cout<<"cur_idx:"<<cur_idx<<endl;
                cout<<"parent.size():"<<parent.size()<<endl;
                StopageDebug("TspApproxiPlan cur_idx >= parent.size()1");
            }
            banned_branch.insert({cur_idx, parent[cur_idx]});
            // cout<<"banned idx:"<<cur_idx<<endl;
            cur_idx = parent[cur_idx];

            if(cur_idx >= parent.size()  || cur_idx < 0){
                cout<<"cur_idx:"<<cur_idx<<endl;
                cout<<"parent.size():"<<parent.size()<<endl;
                StopageDebug("TspApproxiPlan cur_idx >= parent.size()2");
            }
            if(cur_idx == 0) break;
        }
    }
    
    /* heuristicly choose route */
    route_h.clear();
    cur_idx = 0;
    vector<int> expanding_vec;
    vector<pair<int, double>> expanding_branches;
    expanding_vec.emplace_back(0);
    int banned_idx;
    bool have_banned = false;
    while(!expanding_vec.empty()){
        cur_idx = expanding_vec.back();
        expanding_vec.pop_back();
        if(cur_idx != 0) {
            route_h.emplace_back(tarhs[cur_idx-1]);
            // cout<<"["<<cur_idx-1<<"-"<<route_h.back()->id_<<"]->";
            if(route_h.size() == 1) {
                if(!Astar(ps, p_hs, g0_hs, tarhs[cur_idx-1], hs_paths, path2fh, d1)){ // dont notate
                    StopageDebug("TspApproxiPlan Astar failed");
                }
                if(path2fh.size() == 0){
                    for(auto P : hs_paths) cout<<"P.size:"<<P.size()<<endl;
                    StopageDebug("path2fh.size() == 0");
                }
            }
            // cout<<"cur_idx-1:"<<cur_idx-1<<endl;
        }
        have_banned = false;
        for(int i = 0; i < branches[cur_idx].size(); i++){
            if(banned_branch.find(branches[cur_idx][i]) != banned_branch.end()){
                banned_idx = branches[cur_idx][i];
                have_banned = true;
            }
            else{
                expanding_branches.push_back({branches[cur_idx][i], dist_mat(cur_idx, branches[cur_idx][i])});
            }
        }
        sort(expanding_branches.begin(), expanding_branches.end(), [](pair<int, double> &b1, pair<int, double> &b2){
            return b1.second > b2.second;
        });
        if(have_banned) {
            expanding_vec.emplace_back(banned_idx);
        }
        for(auto &b : expanding_branches){
            expanding_vec.emplace_back(b.first);
        }
        expanding_branches.clear();
    }
    // cout<<endl;

    // cout<<"route_h.size():"<<route_h.size()<<endl;
    // for(int i = 0; i < branches.size(); i++){
    //     cout<<"branches["<<i<<"].size():"<<branches[i].size()<<endl;
    //     for(int j = 0; j < branches[i].size(); j++){
    //         cout<<branches[i][j]<<" ";
    //     }
    //     cout<<endl;
    // }
    if(route_h.size() + 1 != branches.size()){
        cout<<"route_h.size():"<<route_h.size()<<endl;
        cout<<"branches.size():"<<branches.size()<<endl;        
        StopageDebug("TspApproxiPlan route_h.size() != branches.size()");
    }
    cout<<"route time:"<<ros::WallTime::now().toSec() - t0.toSec()<<endl;
    // cout<<"route_h.size():"<<route_h.size()<<endl;

    DebugLineStrip(ps, route_h);
    // DebugTree(ps, tarhs, branches);

    return true;
}

void MultiDtgPlus::DistMaintTimerCallback(const ros::TimerEvent &e){
    if(H_list_.empty()) return;
    vector<h_ptr> valid_vec;
    vector<h_ptr> h_vec;

    for(auto &h : H_list_){
        h_vec.emplace_back(h);
        if(!h->hf_edges_.empty()){
            valid_vec.emplace_back(h);
        }
    }

    int n = 1;
    uint32_t h1, h2;
    uint64_t ha;
    auto rand_idx = uniform_int_distribution<int>(0, H_list_.size() - 1);
    double t0 = ros::WallTime::now().toSec();
    double ts = ros::WallTime::now().toSec();

    // while(1){
    //     if(n %10 == 0){
    //         if(ros::WallTime::now().toSec() - t0 > 0.005){
    //             break;
    //         }
    //     }
    //     n++;
    //     h1 = rand_idx(eng_);
    //     h2 = rand_idx(eng_);
    //     if(h1 == h2) continue;
    //     if(!h_vec[h1]->hf_edges_.empty() && !h_vec[h2]->hf_edges_.empty()) continue;
    //     if(h1 > h2) swap(h1, h2);
    //     ha = h1 * 10000000 + h2;
    //     h_dist_map_.erase(ha);
    // }

    // cout<<"DistMaintTimerCallback2"<<endl;
    // cout<<"valid_vec:"<<valid_vec.size()<<endl;
    if(valid_vec.empty()) return;
    sort(valid_vec.begin(), valid_vec.end(), [](h_ptr &h1, h_ptr &h2){
        return h1->last_maintain_t_ > h2->last_maintain_t_;
    });

    h_ptr hs;
    n = 0;
    t0 = ros::WallTime::now().toSec();
    while(1){
        if(ros::WallTime::now().toSec() - t0 > 0.025){
            break;
        }
        if(valid_vec.size() == 1) break;
        hs = valid_vec.back();
        valid_vec.pop_back();
        MainTainDistMap(hs, valid_vec);
        hs->last_maintain_t_ = ros::WallTime::now().toSec();
        // cout<<"MainTainDistMap:"<<hs->id_<<endl;
    }
    // cout<<"DistMaintTimerCallback:"<<ros::WallTime::now().toSec() - ts<<endl;
    // cout<<"DistMaintTimerCallback3"<<endl;

}

void MultiDtgPlus::ShowAll(const ros::TimerEvent &e){
    Show();
}

void MultiDtgPlus::Show(){
    visualization_msgs::MarkerArray mka;
    geometry_msgs::Point p1, p2;
    list<hhe_ptr> hh_e_list;
    list<hfe_ptr> hf_e_list;
    mka.markers.resize(3);

    mka.markers[0].header.frame_id = "world";
    mka.markers[0].header.stamp = ros::Time::now();
    mka.markers[0].id = 0;
    mka.markers[0].action = visualization_msgs::Marker::ADD;
    mka.markers[0].type = visualization_msgs::Marker::SPHERE_LIST;
    mka.markers[0].scale.x = 0.25;
    mka.markers[0].scale.y = 0.25;
    mka.markers[0].scale.z = 0.25;
    mka.markers[0].color.a = 0.8;
    // cout<<"show????????"<<H_list_.size()<<endl;

    for(auto &h : H_list_){
        p1.x = h->pos_(0);
        p1.y = h->pos_(1);
        p1.z = h->pos_(2);
        if(h->h_flags_ & 4)
            mka.markers[0].colors.push_back(CM_->Id2Color(1, 1.0));
        else 
            mka.markers[0].colors.push_back(CM_->Id2Color(0, 0.1));
        mka.markers[0].points.emplace_back(p1);
        for(auto &e : h->hh_edges_){
            if(e->e_flag_ & 2) continue;
            else {
                e->e_flag_ |= 2;
                hh_e_list.emplace_back(e);
            }
        }
        hf_e_list.insert(hf_e_list.end(), h->hf_edges_.begin(), h->hf_edges_.end()); 
    }

//     // cout<<"show!!!!!!!!!!!!!!   "<<hh_e_list.size()<<endl;
    for(auto &e : hh_e_list){
        e->e_flag_ &= 253;
    }

    mka.markers[1].header.frame_id = "world";
    mka.markers[1].header.stamp = ros::Time::now();
    mka.markers[1].id = 4;
    mka.markers[1].action = visualization_msgs::Marker::ADD;
    mka.markers[1].type = visualization_msgs::Marker::LINE_LIST;
    mka.markers[1].scale.x = 0.07;
    mka.markers[1].scale.y = 0.07;
    mka.markers[1].scale.z = 0.07;
    mka.markers[1].color.a = 0.6;
    mka.markers[1].color.r = 1.0;
    mka.markers[1].color.g = 0.5;
    mka.markers[1].color.b = 0.0;
    for(auto &e : hh_e_list){
        if(show_e_details_){
            if(e->path_.size() <= 1) continue;
            for(list<Eigen::Vector3d>::iterator p_it = e->path_.begin(); p_it != e->path_.end(); p_it++){
                p1.x = p_it->x();
                p1.y = p_it->y();
                p1.z = p_it->z();
                p_it++;
                if(p_it == e->path_.end()) break;
                p2.x = p_it->x();
                p2.y = p_it->y();
                p2.z = p_it->z();
                p_it--;
                mka.markers[1].points.emplace_back(p1);
                mka.markers[1].points.emplace_back(p2);
            }
        }
        // cout<<"h1:"<<e->head_n_->id_<<" h2:"<<e->tail_n_->id_<<endl;
        // cout<<"pf:"<<e->path_.front().transpose()<<" pe:"<<e->path_.back().transpose()<<endl;
        // p1.x = e->path_.front().x();
        // p1.y = e->path_.front().y();
        // p1.z = e->path_.front().z();

        // p2.x = e->path_.back().x();
        // p2.y = e->path_.back().y();
        // p2.z = e->path_.back().z();
        // mka.markers[1].points.emplace_back(p1);
        // mka.markers[1].points.emplace_back(p2);
    }

    mka.markers[2].header.frame_id = "world";
    mka.markers[2].header.stamp = ros::Time::now();
    mka.markers[2].id = 3;
    mka.markers[2].action = visualization_msgs::Marker::ADD;
    mka.markers[2].type = visualization_msgs::Marker::LINE_LIST;
    mka.markers[2].scale.x = 0.05;
    mka.markers[2].scale.y = 0.05;
    mka.markers[2].scale.z = 0.05;
    mka.markers[2].color.a = 0.4;
    mka.markers[2].color.r = 0.5;
    mka.markers[2].color.g = 0.0;
    mka.markers[2].color.b = 1.0;
    // cout<<"debug show"<<endl;
    // tr1::unordered_map<int, int> debug_dict;
    for(auto &e : hf_e_list){

        // if(!(e->e_flag_ & 16)) continue;

        p1.x = e->path_.front().x();
        p1.y = e->path_.front().y();
        p1.z = e->path_.front().z();

        p2.x = e->path_.back().x();
        p2.y = e->path_.back().y();
        p2.z = e->path_.back().z();

        mka.markers[2].points.emplace_back(p1);
        mka.markers[2].points.emplace_back(p2);
    }

    int i = 0;
    for(auto &mk : mka.markers){
        if(mk.points.size() == 0) mk.action = visualization_msgs::Marker::DELETE;
        i++;
    }
    topo_pub_.publish(mka);
    // if(H_list_.size() == 3){
    //     StopageDebug("H size == 3");
    // }
}

// void MultiDtgPlus::Debug(){
//     for(auto &h_l : H_depot_){
//         for(auto &h : h_l){
//             for(auto &e : h->hh_edges_){
//                 h_ptr t;
//                 if(e->head_n_ == h){
//                     t = e->tail_n_;
//                 }
//                 else{
//                     t = e->head_n_;
//                 }
//                 bool have_edge = false;
//                 for(auto &e2 : t->hh_edges_){
//                     if(e2 == e) {
//                         have_edge = true;
//                         break;
//                     }
//                 }
//                 if(!have_edge){
//                     ROS_ERROR("error edge");
//                     cout<<e->head_<<" "<<e->tail_<<endl;
//                 }
//             }
//         }
//     }
// }

void MultiDtgPlus::Debug(list<Eigen::Vector3d> &fl, int idx){
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = idx;
    mk.action = visualization_msgs::Marker::ADD;
    mk.type = visualization_msgs::Marker::SPHERE_LIST;
    mk.scale.x = 0.2;
    mk.scale.y = 0.2;
    mk.scale.z = 0.2;
    mk.color.a = 0.8;
    mk.color.b = 0.8;
    if(idx == 2){
        mk.color.a = 0.8;
        mk.color.r = 0.8;
    }
    geometry_msgs::Point pt;
    for(auto &f : fl){
        // for(auto &v : f.second){
        //     pt.x = v.second(0);
        //     pt.y = v.second(1);
        //     pt.z = v.second(2);
        //     mk.points.emplace_back(pt);
        // }
        pt.x = f(0);
        pt.y = f(1);
        pt.z = f(2);
        mk.points.emplace_back(pt);
    }
    if(mk.points.size() != 0)
        debug_pub_.publish(mk);
    // fl.clear();
}


void MultiDtgPlus::Debug(vector<Eigen::Vector3d> &fl, int idx){
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = idx;
    mk.action = visualization_msgs::Marker::ADD;
    mk.type = visualization_msgs::Marker::LINE_STRIP;
    mk.scale.x = 0.1;
    mk.scale.y = 0.1;
    mk.scale.z = 0.1;
    mk.color.a = 0.8;
    mk.color.b = 0.8;
    if(idx == 2){
        mk.color.a = 0.8;
        mk.color.r = 0.8;
    }
    geometry_msgs::Point pt;
    for(auto &f : fl){
        // for(auto &v : f.second){
        //     pt.x = v.second(0);
        //     pt.y = v.second(1);
        //     pt.z = v.second(2);
        //     mk.points.emplace_back(pt);
        // }
        pt.x = f(0);
        pt.y = f(1);
        pt.z = f(2);
        mk.points.emplace_back(pt);
    }
    if(mk.points.size() != 0)
        debug_pub_.publish(mk);
    // fl.clear();
}


void MultiDtgPlus::DebugLineStrip(Eigen::Vector3d ps, vector<h_ptr> &route_h){
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = 100;
    mk.action = visualization_msgs::Marker::ADD;
    mk.type = visualization_msgs::Marker::LINE_STRIP;
    mk.scale.x = 0.125;
    mk.scale.y = 0.125;
    mk.scale.z = 0.125;
    mk.color.a = 0.8;
    mk.color.g = 1.0;
    geometry_msgs::Point pt;
    pt.x = ps(0);
    pt.y = ps(1);   
    pt.z = ps(2);
    mk.points.emplace_back(pt);
    for(auto &h : route_h){
        // cout<<"=====>"<<endl;
        // cout<<"route_h id:"<<h->id_<<endl;
        // cout<<"route_h pos:"<<h->pos_.transpose()<<endl;
        pt.x = h->pos_(0);
        pt.y = h->pos_(1);
        pt.z = h->pos_(2) + 0.2;
        mk.points.emplace_back(pt);
    }
    if(mk.points.size() != 0)
        debug_pub_.publish(mk);
}

void MultiDtgPlus::DebugTree(Eigen::Vector3d ps, vector<h_ptr> &hs, vector<vector<int>> &branches){
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = 101;
    mk.action = visualization_msgs::Marker::ADD;
    mk.type = visualization_msgs::Marker::LINE_LIST;
    mk.scale.x = 0.04;
    mk.scale.y = 0.04;
    mk.scale.z = 0.04;
    mk.color.a = 0.8;
    mk.color.r = 1.0;
    geometry_msgs::Point pt1, pt2;
    for(int i = 0; i < branches.size(); i++){
        if(i == 0){
            pt1.x = ps(0);
            pt1.y = ps(1);
            pt1.z = ps(2) + 0.3;
        }
        else{
            pt1.x = hs[i-1]->pos_(0);
            pt1.y = hs[i-1]->pos_(1);
            pt1.z = hs[i-1]->pos_(2) + 0.3;
        }
        for(auto &b : branches[i]){
            pt2.x = hs[b-1]->pos_(0);
            pt2.y = hs[b-1]->pos_(1);
            pt2.z = hs[b-1]->pos_(2) + 0.3;
            mk.points.emplace_back(pt1);
            mk.points.emplace_back(pt2);
        }

    }
    if(mk.points.size() != 0)
    debug_pub_.publish(mk);
}