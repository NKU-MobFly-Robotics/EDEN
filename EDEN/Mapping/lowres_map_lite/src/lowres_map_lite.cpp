#include <lowres_map_lite/lowres_map_lite.h>
using namespace std;
using namespace lowres_lite;


void LowResMap::UpdateLocalBBX( const pair<Eigen::Vector3d, Eigen::Vector3d> bbx){
    // mtx_.lock();
    double time = ros::WallTime::now().toSec();
    // idx_path_clear_.clear();
    h_id_clear_.clear();

    Eigen::Vector3d pos;

    Eigen::Vector3i node3i, upi, downi, searchidx;
    for(int dim = 0; dim < 3; dim++){
        upi(dim) = ceil((bbx.first(dim) - origin_(dim)) / node_scale_(dim));
        downi(dim) = floor((bbx.second(dim) - origin_(dim)) / node_scale_(dim));
        upi(dim) = min(upi(dim), voxel_num_(dim)-1); 
        downi(dim) = max(downi(dim), 0); 
    }

    for(node3i(0) = downi(0); node3i(0) <= upi(0); node3i(0)++){
        for(node3i(1) = downi(1); node3i(1) <= upi(1); node3i(1)++){
            for(node3i(2) = downi(2); node3i(2) <= upi(2); node3i(2)++){
                pos = (node3i.cast<double>() + Eigen::Vector3d(0.5,0.5,0.5)).cwiseProduct(node_scale_)+origin_;
                CheckNode(node3i);
            }
        }
    }
    // Debug(debug_list);
    // mtx_.unlock();
    if(showmap_){
        bool add;

        for(searchidx(0) = downi(0); searchidx(0) <= upi(0); searchidx(0)+=1){
            for(searchidx(1) = downi(1); searchidx(1) <= upi(1); searchidx(1)+=1){
                for(searchidx(2) = downi(2); searchidx(2) <= upi(2); searchidx(2)+=1){
                    int idx = GlobalPos2LocalBid(searchidx);
                    if(idx != -1){
                        add = true;
                        if(!LG_[idx]->show_flag_){
                            int gbid = LocalBid2GlobalBid(idx);
                            if(gbid == -1){
                                StopageDebug("LoadShowList, error gbid");
                            }
                            Showblocklist_.push_front(gbid);
                            LG_[idx]->show_flag_ = true;
                        }
                    }
                    else{
                        int gbid = LocalBid2GlobalBid(idx);
                        if(gbid == -1){
                            StopageDebug("LoadShowList, error gbid");
                        }
                        Showblocklist_.push_front(gbid);
                    }
                }
            }
        }        
    }
}


// bool LowResMap::UpdateLRM(const Eigen::Matrix4d &rob_pose, const vector<Eigen::Vector3d> &u2f_list, const vector<Eigen::Vector3d> &u2o_list, 
//                                     const vector<Eigen::Vector3d> &o2f_list, const vector<Eigen::Vector3d> &f2o_list, vector<pair<int, int>> &search_origins){
//     search_origins.clear();
//     mtx_.lock();
//     Robot_pose_ = rob_pose;
//     Robot_pos_ = Robot_pose_.block(0,3,3,1);
//     Eigen::Vector3i robot_pid;
//     PostoId3(Robot_pos_, robot_pid);
//     ReloadNodes(search_origins);
//     SetNodeStates(u2f_list, u2o_list, o2f_list, f2o_list, search_origins);

//     if(showmap_){
//         LoadShowList();
//     }
//     return true;
// }

void LowResMap::ReloadNodes(const Eigen::Vector3d &r_pos, vector<pair<int, int>> &candidate_search_origins, 
                            list<list<pair<uint32_t, Eigen::Vector3d>>> &h_margins, 
                            list<pair<uint32_t, Eigen::Vector3d>> &hns){
    Eigen::Vector3i new_origin_idx, new_up_idx, it, diff;
    Eigen::Vector3d new_origin;
    Robot_pos_ = r_pos;
    for(int dim = 0; dim < 3; dim++){
        if(Robot_pos_(dim) + 0.5*local_mapscale_(dim) >= map_upbd_(dim)) new_origin(dim) = map_upbd_(dim) - local_mapscale_(dim) + 1e-3;
        else if(Robot_pos_(dim) - 0.5*local_mapscale_(dim) <= map_lowbd_(dim)) new_origin(dim) = map_lowbd_(dim)+1e-3;
        else new_origin(dim) = Robot_pos_(dim) - local_mapscale_(dim) * 0.5;
    }

    if(!GlobalPos2GlobalBlock3Id(new_origin, new_origin_idx)){
        ROS_ERROR("error ResetLocalMap");
        cout<<"new_origin:"<<new_origin.transpose()<<endl;
        ros::shutdown();
        return;
    }   

    /* get local new, changed, and out block ids */
    list<pair<int, int>> out_idxs; // local id
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
                    int lid = GlobalPos2LocalBid(it);
                    int gid = GlobalBid32GlobalBid(it);
                    out_idxs.push_back({lid, gid});
                    it = new_up_idx - Eigen::Vector3i(x, y, z);
                    lid = GlobalBid32GlobalBid(it);
                    new_idxs.push_back({lid, it});

                }
                else {
                    int lid = GlobalPos2LocalBid(it);
                    if(lid < 0 || lid >= LG_.size()) { //debug
                        cout<<"di:"<<(it - local_origin_idx_).transpose()<<endl;
                        cout<<"it:"<<it.transpose()<<endl;

                        ROS_ERROR("ResetLocalMap() impossible GetLocalBlockId-1");
                        ros::shutdown();
                        continue;
                    }
                    change_idxs.push_back({lid, it});
                }
            }
        }
    }


    /* delete out blocks in rviz */
    if(showmap_){
        for(auto &b : out_idxs){
            if(!LG_[b.first]->show_flag_){
                Showblocklist_.emplace_back(b.second);
            }
        }
    }
    
    /* change parameters */
    local_origin_ = new_origin;
    local_origin_idx_ = new_origin_idx;
    local_up_idx_ = new_up_idx;
    local_map_upbd_ = local_origin_ + local_mapscale_ - Eigen::Vector3d::Ones() * 1e-3;
    local_map_lowbd_ = local_origin_ + Eigen::Vector3d::Ones() * 1e-3;

    /* change blocks' postitions */
    list<pair<int, shared_ptr<LR_block>>> change_list; // new id, block
    for(auto &bid : change_idxs){
        int lid = GlobalBid32GlobalBid(bid.second);
        change_list.push_back({lid, LG_[bid.first]});
    }
    for(auto &ci : change_list){
        LG_[ci.first] = ci.second;
    }
    
    /* initialize new blocks */
    for(auto &bid: new_idxs){
        int lid = GlobalBid32GlobalBid(bid.second);
        InitializeBlock(lid, bid.first);
        LG_[lid]->show_flag_ = true;
        Showblocklist_.emplace_back(bid.first);
    }

    /* clear out hn and corresponding margins */
    auto hm = h_margins.begin();
    auto hn = hns.begin();
    for(; hm != h_margins.end(); hm++, hn++){
        if(!InsideLocalMap(hn->second)){ // hn out, clear
            auto hnt = hn;
            auto hmt = hm;
            hnt--, hmt--;
            h_margins.erase(hm);
            hns.erase(hn);
            hnt++;
            hmt++;
            hm = hmt, hn = hnt;
        }
        else{ // only clear margins outside local
            for(auto hmn = hm->begin(); hmn != hm->end(); hmn++){
                if(!InsideLocalMap(hmn->second)){
                    auto hmnt = hmn;
                    hmnt--;
                    hm->erase(hmn);
                    hmnt++;
                    hmn = hmnt;
                }
            }
        }
    }

    diff = (new_origin_idx - local_origin_idx_).cwiseProduct(block_size_);
    vector<Eigen::Vector3i> edge_in, edge_out;
	vector<pair<int, int>> expand_clear_list, clear_list; // bid, nid
    vector<Eigen::Vector3i> dirs;
	Eigen::Vector3i c, cn;
    Eigen::Vector3i par, new_local_node_, new_local_origin_node_idx, new_local_up_node_idx;
    int bid, nid, cbid, cnid;
    uint8_t pdir;
    new_local_origin_node_idx = new_origin_idx.cwiseProduct(block_size_);
    new_local_up_node_idx = new_local_origin_node_idx + local_node_num_ - Eigen::Vector3i::Ones();
    GetEdgeNodes(edge_in, edge_out, diff);
    for(auto &p : edge_out){
        if(!GlobalPos2LocalBidNid(p, bid, nid)){ 
            StopageDebug("UpdateLocalTopo, impossible GlobalPos2LocalBidNid parent0");
        }
        pdir = LG_[bid]->local_grid_[nid]->parent_dir_;
        if(pdir == 0) continue;
        par = p + connect_diff_[pdir];
        /* if parent direction out side new region, clear this tree branch */
        if(par(0) < new_local_origin_node_idx(0) || par(1) < new_local_origin_node_idx(1) || par(2) < new_local_origin_node_idx(2)
            || par(0) > new_local_up_node_idx(0) || par(1) > new_local_up_node_idx(1) || par(2) > new_local_up_node_idx(2)) { 
            expand_clear_list.push_back({bid, nid});
            while(!expand_clear_list.empty()){
                bid = expand_clear_list.back().first;
                nid = expand_clear_list.back().second;
                expand_clear_list.pop_back();
                GetChildrenDirs(LG_[bid]->local_grid_[nid]->children_dir_, dirs);
                for(auto &d : dirs){
                    cn = c + d;
                    if(GlobalPos2LocalBidNid(cn, cbid, cnid)){
                        expand_clear_list.push_back({cbid, cnid});
                    }
                }
                if(LG_[bid]->local_grid_[nid]->root_id_ != 0){
                    LG_[bid]->local_grid_[nid]->root_id_ = 0;
                    LG_[bid]->local_grid_[nid]->parent_dir_ = 13;
                    LG_[bid]->local_grid_[nid]->path_g_ = 999999.0;
                    LG_[bid]->local_grid_[nid]->children_dir_ = 0;
                    clear_list.push_back({bid, nid});
                }
            }
        }
        else{ // if parent direction inside new region, check if children nodes are outside
            GetChildrenDirs(LG_[bid]->local_grid_[nid]->children_dir_, dirs);
            for(auto &d : dirs){
                cn = c + d;
                if(!InsideLocalMap(cn)){

                }
            }
                // c = BlockNodeId2GlobalNodeId(LG_[bid], nid);
            // for(auto &oc : occlude_check_list_){
            //     cn = c + oc.first;
            //     if(GlobalPos2LocalBidNid(cn, cbid, cnid)){
            //         pdir = LG_[cbid]->local_grid_[cnid]->parent_dir_;
            //         if(pdir == 0) continue;
            //         int ocid = oc.second + (pdir - 1) * 18;
            //         if(connection_check_[ocid]) {
            //             expand_clear_list.push_back({cbid, cnid});
            //             cn = cn + connect_diff_[pdir];
            //             if(GlobalPos2LocalBidNid(cn, cbid, cnid)){
            //                 LG_[cbid]->local_grid_[cnid]->children_dir_ &= ~(1 << pdir_2_chcn_[pdir]);
            //             }
            //             else{
            //                 StopageDebug("UpdateLocalTopo, impossible GlobalPos2LocalBidNid parent0");
            //             }
            //         }
            //     }
            // }

        }
    }
    for(auto &p : edge_in){
        if(!GlobalPos2LocalBidNid(p, bid, nid)){ 
            StopageDebug("UpdateLocalTopo, impossible GlobalPos2LocalBidNid parent0");
        }
        if(LG_[bid]->local_grid_[nid]->root_id_ != 0){
            // LG_[bid]->local_grid_[nid]->root_id_ = 0;
            // LG_[bid]->local_grid_[nid]->parent_dir_ = 0;
            // LG_[bid]->local_grid_[nid]->path_g_ = 0;
            // LG_[bid]->local_grid_[nid]->children_dir_ = 0;
            clear_list.push_back({bid, nid});
        }
    }

    vector<Eigen::Vector3i> nei_diffs;
	for(int x = -1; x < 2; x++){
		for(int y = -1; y < 2; y++){
			for(int z = -1; z < 2; z++){
				if(x != 0 && y != 0 && z != 0) nei_diffs.emplace_back(Eigen::Vector3i(x, y, z));
			}
		}
	}
	for(auto &n : clear_list){
		c = BlockNodeId2GlobalNodeId(LG_[n.first], n.second);
		for(auto &nd : nei_diffs){
			cn = c + nd;
			if(GlobalPos2LocalBidNid(cn, cbid, cnid)){
				if(LG_[cbid]->local_grid_[cnid]->root_id_ != 0 && !(LG_[cbid]->local_grid_[cnid]->flags_ & 2)){
					candidate_search_origins.push_back({cbid, cnid});
                    LG_[cbid]->local_grid_[cnid]->flags_ |= 2;
				}
			}
		}
	}
}

void LowResMap::SetNodeStates(const vector<Eigen::Vector3d> &u2f_list, const vector<Eigen::Vector3d> &u2o_list, 
    const vector<Eigen::Vector3d> &o2f_list, const vector<Eigen::Vector3d> &f2o_list, vector<pair<int, int>> &candidate_search_origins){
    int gbid, bid, nid, cbid, cnid, ocid, pdir;
    uint8_t path_dir;
    vector<uint8_t> old_f_state, old_o_state;
    vector<Eigen::Vector3i> dirs;
	Eigen::Vector3i dir, c, cn;
    uint8_t state;  // 0: occ, 1: free
	uint32_t children_dir;
	vector<pair<int, int>> o_list, f_list, clear_list, expand_clear_list; // bid, nid
	// vector<Eigen::Vector3i>

    for(auto p : u2f_list){
        bid = GlobalPos2LocalBid(p);
        if(bid == -1){
            // StopageDebug("SetNodeStates, bid == -1, u2f_list");
            continue;
        }
        nid = GetBlkNid(p, LG_[bid]);
        if(LG_[bid]->local_grid_[nid] == Outnode_) continue;
        if(!(LG_[bid]->local_grid_[nid]->flags_ & 1)){
            LG_[bid]->local_grid_[nid]->flags_ |= 1;
            f_list.push_back({bid, nid});
            state = LG_[bid]->local_grid_[nid]->free_num_ == nv_num_;
            old_f_state.emplace_back(state);
        }
        LG_[bid]->local_grid_[nid]->free_num_++;
        LG_[bid]->local_grid_[nid]->unknown_num_--;
    }

    for(auto p : o2f_list){
        bid = GlobalPos2LocalBid(p);
        if(bid == -1){
            // StopageDebug("SetNodeStates, bid == -1, o2f_list");
            continue;
        }
        nid = GetBlkNid(p, LG_[bid]);
        if(LG_[bid]->local_grid_[nid] == Outnode_) continue;
        if(!(LG_[bid]->local_grid_[nid]->flags_ & 1)){
            LG_[bid]->local_grid_[nid]->flags_ |= 1;
            f_list.push_back({bid, nid});
            state = LG_[bid]->local_grid_[nid]->free_num_ == nv_num_;
            old_f_state.emplace_back(state);
        }
        LG_[bid]->local_grid_[nid]->free_num_++;
    }

    for(auto p : u2o_list){
        bid = GlobalPos2LocalBid(p);
        if(bid == -1){
            // StopageDebug("SetNodeStates, bid == -1, u2o_list");
            continue;
        }
        nid = GetBlkNid(p, LG_[bid]);
        if(LG_[bid]->local_grid_[nid] == Outnode_) continue;
        if(!(LG_[bid]->local_grid_[nid]->flags_ & 1)){
            LG_[bid]->local_grid_[nid]->flags_ |= 1;
            o_list.push_back({bid, nid});
            state = LG_[bid]->local_grid_[nid]->free_num_ == nv_num_;
            old_o_state.emplace_back(state);
        }
        LG_[bid]->local_grid_[nid]->unknown_num_--;
    }
    
    for(auto p : f2o_list){
        bid = GlobalPos2LocalBid(p);
        if(bid == -1){
            // StopageDebug("SetNodeStates, bid == -1, f2o_list");
            continue;
        }
        nid = GetBlkNid(p, LG_[bid]);
        if(LG_[bid]->local_grid_[nid] == Outnode_) continue;
        if(!(LG_[bid]->local_grid_[nid]->flags_ & 1)){
            LG_[bid]->local_grid_[nid]->flags_ |= 1;
            o_list.push_back({bid, nid});
            state = LG_[bid]->local_grid_[nid]->free_num_ == nv_num_;
            old_o_state.emplace_back(state);
        }
        LG_[bid]->local_grid_[nid]->free_num_--;
    }

	
	// new occ nodes, clear chains
	vector<Eigen::Vector3i> nei_diffs;
	for(int x = -1; x < 2; x++){
		for(int y = -1; y < 2; y++){
			for(int z = -1; z < 2; z++){
				if(x != 0 && y != 0 && z != 0) nei_diffs.emplace_back(Eigen::Vector3i(x, y, z));
			}
		}
	}
    for(int i = 0; i < o_list.size(); i++){
        bid = o_list[i].first;
        nid = o_list[i].second;
        state = LG_[bid]->local_grid_[nid]->free_num_ == nv_num_;
        LG_[bid]->local_grid_[nid]->flags_ &= 254;
        if(state == old_o_state[i]){
            continue;
        }
        else{
            // clear graph path
            if(state == 0){
                if(LG_[bid]->local_grid_[nid]->root_id_ != 0){ 
					c = BlockNodeId2GlobalNodeId(LG_[bid], nid);
					expand_clear_list.push_back({bid, nid});
					pdir = LG_[bid]->local_grid_[nid]->parent_dir_;
					if(pdir != 0){
						cn = c + connect_diff_[pdir];
						if(GlobalPos2LocalBidNid(cn, cbid, cnid)){ // clear parent's record
							LG_[cbid]->local_grid_[cnid]->children_dir_ &= ~(1 << pdir);
						}
						else{
							StopageDebug("SetNodeStates, impossible GlobalPos2LocalBidNid parent0");
						}
					}

					// for(auto &oc : occlude_check_list_){ //clear occluded diagnal connected nodes
					// 	cn = c + oc.first;
					// 	if(GlobalPos2LocalBidNid(cn, cbid, cnid)){
					// 		pdir = LG_[cbid]->local_grid_[cnid]->parent_dir_;
					// 		if(pdir == 0) continue;
					// 		ocid = oc.second + (pdir - 1) * 18;
					// 		if(connection_check_[ocid]) {
					// 			expand_clear_list.push_back({cbid, cnid});
					// 			cn = cn + connect_diff_[pdir];
					// 			if(GlobalPos2LocalBidNid(cn, cbid, cnid)){ // clear parent's record
					// 				LG_[cbid]->local_grid_[cnid]->children_dir_ &= ~(1 << pdir);
					// 			}
					// 			else{
					// 				StopageDebug("SetNodeStates, impossible GlobalPos2LocalBidNid parent");
					// 			}
					// 		}
					// 	}
					// }

					// forward clear
					while(!expand_clear_list.empty()){
						bid = expand_clear_list.back().first;
						nid = expand_clear_list.back().second;
						expand_clear_list.pop_back();
						GetChildrenDirs(LG_[bid]->local_grid_[nid]->children_dir_, dirs);
						for(auto &d : dirs){
							cn = c + d;
							if(GlobalPos2LocalBidNid(cn, cbid, cnid)){
								expand_clear_list.push_back({cbid, cnid});
							}
							else{
								StopageDebug("SetNodeStates, GlobalPos2LocalBidNid fail1");
							}
						}
						if(LG_[bid]->local_grid_[nid]->root_id_ != 0){
							LG_[bid]->local_grid_[nid]->root_id_ = 0;
							LG_[bid]->local_grid_[nid]->parent_dir_ = 13;
							LG_[bid]->local_grid_[nid]->path_g_ = 999999.0;
							LG_[bid]->local_grid_[nid]->children_dir_ = 0;
							clear_list.push_back({bid, nid});
						}
					}
                }
            }
            if(showmap_ && !LG_[bid]->show_flag_){
                LG_[bid]->show_flag_ = true;
                gbid = LocalBid2GlobalBid(bid);
                if(gbid == -1){
                    StopageDebug("SetNodeStates, error gbid");
                }
                Showblocklist_.emplace_back(gbid);
            }
        }
    }

	// new free nodes, add to list
	for(int i = 0; i < f_list.size(); i++){
		bid = f_list[i].first;
        nid = f_list[i].second;
        state = LG_[bid]->local_grid_[nid]->free_num_ == nv_num_;
		if(state == old_f_state[i]){
            continue;
        }
		else{
			clear_list.push_back({bid, nid});
			if(showmap_ && !LG_[bid]->show_flag_){
                LG_[bid]->show_flag_ = true;
                gbid = LocalBid2GlobalBid(bid);
                if(gbid == -1){
                    StopageDebug("SetNodeStates, error gbid");
                }
                Showblocklist_.emplace_back(gbid);
            }
		}
	}
	
	// add search origins
	for(auto &n : clear_list){
		c = BlockNodeId2GlobalNodeId(LG_[n.first], n.second);
		for(auto &nd : nei_diffs){
			cn = c + nd;
			if(GlobalPos2LocalBidNid(cn, cbid, cnid)){
				if(LG_[cbid]->local_grid_[cnid]->root_id_ != 0 && !(LG_[cbid]->local_grid_[cnid]->flags_ & 2)){
					candidate_search_origins.push_back({cbid, cnid});
                    LG_[cbid]->local_grid_[cnid]->flags_ |= 2;
				}
			}
		}
	}
}

void LowResMap::GetEdgeNodes(vector<Eigen::Vector3i> &edge_in, vector<Eigen::Vector3i> &edge_out, Eigen::Vector3i &move_diff){
    Eigen::Vector3i intersec_origin, intersec_size, it1, it2;
    intersec_size = local_node_num_ - move_diff;
    intersec_origin = local_origin_idx_.cwiseProduct(block_size_);
    for(int dim = 0; dim < 3; dim++){
        if(abs(move_diff(dim)) >= local_node_num_(dim)) return;

    }

    if(move_diff(0) != 0){
        for(int y = 0; y < intersec_size(1); y++){
            for(int z = 0; z < intersec_size(2); z++){
                it1 = intersec_origin + Eigen::Vector3i(0, y, z);
                it2 = intersec_origin + Eigen::Vector3i(intersec_size(0) - 1, y, z);
                if(move_diff(0) > 0){
                    edge_out.emplace_back(it1);
                    edge_in.emplace_back(it2);
                }
                else{
                    edge_out.emplace_back(it2);
                    edge_in.emplace_back(it1);
                }
            }
        }
    }

    if(move_diff(1) != 0){
        for(int x = 0; x < intersec_size(1); x++){
            for(int z = 0; z < intersec_size(2); z++){
                it1 = intersec_origin + Eigen::Vector3i(x, 0, z);
                it2 = intersec_origin + Eigen::Vector3i(x, intersec_size(1) - 1, z);
                if(move_diff(1) > 0){
                    edge_out.emplace_back(it1);
                    edge_in.emplace_back(it2);
                }
                else{
                    edge_out.emplace_back(it2);
                    edge_in.emplace_back(it1);
                }
            }
        }
    }

    if(move_diff(2) != 0){
        for(int x = 0; x < intersec_size(0); x++){
            for(int y = 0; y < intersec_size(1); y++){
                it1 = intersec_origin + Eigen::Vector3i(x, y, 0);
                it2 = intersec_origin + Eigen::Vector3i(x, y, intersec_size(2) - 1);
                if(move_diff(2) > 0){
                    edge_out.emplace_back(it1);
                    edge_in.emplace_back(it2);
                }
                else{
                    edge_out.emplace_back(it2);
                    edge_in.emplace_back(it1);
                }
            }
        }
    }
}

void LowResMap::ReloadNodesBf(const Eigen::Vector3d &r_pos){
    Eigen::Vector3i new_origin_idx, new_up_idx, it, diff;
    Eigen::Vector3d new_origin;
    Robot_pos_ = r_pos;
    new_origin = Robot_pos_ - local_mapscale_ * 0.5;
    for(int dim = 0; dim < 3; dim++){
        if(Robot_pos_(dim) + 0.5*local_mapscale_(dim) >= map_upbd_(dim)) new_origin(dim) = map_upbd_(dim) - local_mapscale_(dim) + 1e-3;
        else if(Robot_pos_(dim) - 0.5*local_mapscale_(dim) <= map_lowbd_(dim)) new_origin(dim) = map_lowbd_(dim)+1e-3;
        else new_origin(dim) = Robot_pos_(dim) - local_mapscale_(dim) * 0.5;
    }
    // for(int dim = 0; dim < 3; dim++) new_origin(dim) = min(max(new_origin(dim), map_lowbd_(dim) + 1e-3), map_upbd_(dim) - 1e-3);
    if(!GlobalPos2GlobalBlock3Id(new_origin, new_origin_idx)){
        ROS_ERROR("error ResetLocalMap");
        ros::shutdown();
        return;
    }   
    new_origin = new_origin_idx.cast<double>().cwiseProduct(blockscale_) + origin_;
    new_up_idx = new_origin_idx + local_block_num_ - Eigen::Vector3i::Ones();

    /* get local new, changed, and out block ids */
    list<pair<int, int>> out_idxs; // local id
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
                    int lid = GlobalPos2LocalBid(it);
                    int gid = GlobalBid32GlobalBid(it);
                    out_idxs.push_back({lid, gid});
                    it = new_up_idx - Eigen::Vector3i(x, y, z);
                    lid = GlobalBid32GlobalBid(it);
                    new_idxs.push_back({lid, it});

                }
                else {
                    int lid = GlobalPos2LocalBid(it);
                    if(lid < 0 || lid >= LG_.size()) { //debug
                        cout<<"di:"<<(it - local_origin_idx_).transpose()<<endl;
                        cout<<"it:"<<it.transpose()<<endl;

                        ROS_ERROR("ResetLocalMap() impossible GetLocalBlockId-1");
                        ros::shutdown();
                        continue;
                    }
                    change_idxs.push_back({lid, it});
                }
            }
        }
    }

    /* delete out blocks in rviz */
    if(showmap_){
        for(auto &b : out_idxs){
            // if(b.first < 0 || b.first >= LG_.size()){
            //     ROS_WARN("LG_ size out");
            //     cout<<"lg size:"<<LG_.size()<<endl;
            //     cout<<"b.first:"<<b.first<<endl;
            //     getchar();
            // }
            if(!LG_[b.first]->show_flag_){
                Showblocklist_.emplace_back(b.second);
            }
        }
    }
    
    /* change parameters */
    local_origin_ = new_origin;
    local_origin_idx_ = new_origin_idx;
    local_up_idx_ = new_up_idx;
    local_map_upbd_ = local_origin_ + local_mapscale_ - Eigen::Vector3d::Ones() * 1e-3;
    local_map_lowbd_ = local_origin_ + Eigen::Vector3d::Ones() * 1e-3;
    local_origin_node_idx_ = local_origin_idx_.cwiseProduct(block_size_);
    local_up_node_idx_ = local_origin_node_idx_ + local_node_num_ - Eigen::Vector3i::Ones();

    /* change blocks' postitions */
    list<pair<int, shared_ptr<LR_block>>> change_list; // new id, block
    for(auto &bid : change_idxs){
        int lid = GlobalPos2LocalBid(bid.second);
        if(lid == -1) {
            cout<<"bid.second:"<<bid.second.transpose()<<endl;
            cout<<"bid.first:"<<bid.first<<endl;
            cout<<"lid:"<<lid<<endl;
            StopageDebug("ReloadNodesBf GlobalPos2LocalBid lid == -1");
        }
        change_list.push_back({lid, LG_[bid.first]});
        for(auto &n : LG_[bid.first]->local_grid_){
            if(n != Outnode_){
                n->children_dir_ = 0;
                n->parent_dir_ = 13;
                n->root_id_ = 0;
            }
        }
    }
    for(auto &ci : change_list){
        LG_[ci.first] = ci.second;
        // if((LocalBid2LocalPos3i(ci.first).cwiseProduct(block_size_) + local_origin_node_idx_ - LG_[ci.first]->origin_).norm() > 0){
        //     cout<<"ci.first:"<<ci.first<<endl;
        //     StopageDebug("LocalBid2LocalPos3i -1");
        // }
        // Eigen::Vector3i poi;
        // poi(0) = ci.first % local_block_num_(0);
        // poi(1) = ((ci.first - poi(0)) / local_block_num_(0)) % local_block_num_(1);
        // poi(2) = ((ci.first - poi(0)) - poi(1)* local_block_num_(0)) / l_b_n_(1);
        // if(((poi + local_origin_idx_).cwiseProduct(block_size_) - LG_[ci.first]->origin_).norm() > 0){
        //     cout<<"ci.first:"<<ci.first<<endl;
        //     cout<<"poi.cwiseProduct(block_size_):"<<poi.cwiseProduct(block_size_).transpose()<<endl;
        //     cout<<"LG_[ci.first]->origin_:"<<LG_[ci.first]->origin_.transpose()<<endl;
        //     StopageDebug("ReloadNodesBf error change bid");
        // }
    }
    
    /* initialize new blocks */
    for(auto &bid: new_idxs){
        int lid = GlobalPos2LocalBid(bid.second);
        if(lid < 0 || lid > LG_.size()){
            cout<<"lid:"<<lid<<endl;
            cout<<"LG_.size():"<<LG_.size()<<endl;
            cout<<"bid.second:"<<bid.second.transpose()<<endl;
            StopageDebug("lid out!");
        }
        InitializeBlock(lid, bid.first);

        // Eigen::Vector3i poi;
        // poi(0) = lid% local_block_num_(0);
        // poi(1) = ((lid - poi(0)) / local_block_num_(0)) % local_block_num_(1);
        // poi(2) = ((lid - poi(0)) - poi(1)* local_block_num_(0)) / l_b_n_(1);
        // if(((poi + local_origin_idx_).cwiseProduct(block_size_) - LG_[lid]->origin_).norm() > 0){
        //     cout<<"lid:"<<lid<<endl;
        //     cout<<"poi.cwiseProduct(block_size_):"<<poi.cwiseProduct(block_size_).transpose()<<endl;
        //     cout<<"local_origin_idx_:"<<local_origin_idx_.transpose()<<endl;
        //     cout<<"bid.second:"<<bid.second.transpose()<<endl;
        //     cout<<"LG_[lid]->origin_:"<<LG_[lid]->origin_.transpose()<<endl;
        //     StopageDebug("ReloadNodesBf error change bid2");
        // }

        // if((LocalBid2LocalPos3i(lid).cwiseProduct(block_size_) + local_origin_node_idx_ - LG_[lid]->origin_).norm() > 0){
        //     StopageDebug("LocalBid2LocalPos3i 0");
        // }
        LG_[lid]->show_flag_ = true;
        Showblocklist_.emplace_back(bid.first);
    }
}


void LowResMap::SetNodeStatesBf(vector<Eigen::Vector3d> &new_free_nodes){
    int gbid, bid, nid, cbid, cnid, ocid, pdir;
    uint8_t path_dir;
    vector<Eigen::Vector3i> dirs;
    Eigen::Vector3d p;
    Eigen::Vector3i dir, c, cn;
    uint32_t children_dir;
    vector<pair<int, int>> o_list, f_list; // bid, nid
    // vector<Eigen::Vector3i>
    for(auto cn : map_->changed_pts_){
        p = map_->IdtoPos(cn.first);
        if(!map_->InsideExpMap(p)) continue;
        if(!GlobalPos2LocalBidNid(p, bid, nid)){
            StopageDebug("SetNodeStates, impossible GlobalPos2LocalBidNid1");
        }
        if(bid < 0 || bid >= LG_.size()){
            StopageDebug("SetNodeStatesBf bid error");
        }
        if(nid < 0 || nid >= block_size_(0) * block_size_(1) * block_size_(2)){
            int x = (p(0) - local_origin_(0)) /  blockscale_(0);
            int y = (p(1) - local_origin_(1)) /  blockscale_(1);
            int z = (p(2) - local_origin_(2)) /  blockscale_(2);

            bid = x + l_b_n_(0) * y + l_b_n_(1) * z;

            x = (p(0) - origin_(0)) / node_scale_(0);
            y = (p(1) - origin_(1)) / node_scale_(1);
            z = (p(2) - origin_(2)) / node_scale_(2);

            x = x - LG_[bid]->origin_(0);
            y = y - LG_[bid]->origin_(1);
            z = z - LG_[bid]->origin_(2);
            StopageDebug("SetNodeStatesBf nid error");
        }
        if(LG_[bid]->local_grid_[nid] == Outnode_) continue;

        if(cn.second.first == 0){
            if(cn.second.second & 1){
                if(!(LG_[bid]->local_grid_[nid]->flags_ & 1)){
                    LG_[bid]->local_grid_[nid]->flags_ |= 1;
                    if(LG_[bid]->local_grid_[nid]->free_num_ == nv_num_){
                        f_list.push_back({bid, nid});
                    }
                    else{
                        o_list.push_back({bid, nid});
                    }
                }
                LG_[bid]->local_grid_[nid]->unknown_num_--;

            }
            else{
                if(!(LG_[bid]->local_grid_[nid]->flags_ & 1)){
                    LG_[bid]->local_grid_[nid]->flags_ |= 1;
                    if(LG_[bid]->local_grid_[nid]->free_num_ == nv_num_){
                        f_list.push_back({bid, nid});
                    }
                    else{
                        o_list.push_back({bid, nid});
                    }
                }
                LG_[bid]->local_grid_[nid]->free_num_++;
                LG_[bid]->local_grid_[nid]->unknown_num_--;

            }
        }
        else if(cn.second.first == 1){
            if(!(cn.second.second & 1)){
                if(!(LG_[bid]->local_grid_[nid]->flags_ & 1)){
                    LG_[bid]->local_grid_[nid]->flags_ |= 1;
                    if(LG_[bid]->local_grid_[nid]->free_num_ == nv_num_){
                        f_list.push_back({bid, nid});
                    }
                    else{
                        o_list.push_back({bid, nid});
                    }
                }
                LG_[bid]->local_grid_[nid]->unknown_num_--;

                // int x = nid % block_size_(0);
                // int y = ((nid - x) / block_size_(0)) % block_size_(1);
                // int z = ((nid - x - y*block_size_(0))) /(block_size_(1)*block_size_(0));
                // Eigen::Vector3i pi = LG_[bid]->origin_ + Eigen::Vector3i(x, y, z);
                // Eigen::Vector3d p1;
                // int f = 0, u = 0;
                // for(int xi = 0; xi < node_size_(0); xi++)
                //     for(int yi = 0; yi < node_size_(1); yi++)
                //         for(int zi = 0; zi < node_size_(2); zi++){
                //             p1 = pi.cast<double>().cwiseProduct(node_scale_) + Eigen::Vector3d(0.5+xi, 0.5+yi, 0.5+zi) * resolution_ + origin_;
                //             auto cstatus = map_->GetVoxState(p1);
                //             if(cstatus == VoxelState::free) f++;
                //             else if(cstatus == VoxelState::unknown) u++;
                //             else if(cstatus == VoxelState::outlocal){   

                //             }
                // }
                // if(u == LG_[bid]->local_grid_[nid]->unknown_num_ && f != LG_[bid]->local_grid_[nid]->free_num_){
                //     cout<<"bid:"<<bid<<endl;
                //     cout<<"nid:"<<nid<<endl;
                //     cout<<"x:"<<x<<"  y:"<<y<<"  z:"<<z<<endl;
                //     cout<<"f:"<<int(f)<<"  u:"<<int(u)<<endl;
                //     cout<<"g->f:"<<int(LG_[bid]->local_grid_[nid]->free_num_)<<"  g->u:"<<int(LG_[bid]->local_grid_[nid]->unknown_num_)<<endl;
                //     cout<<"p:"<<p.transpose()<<endl;
                //     auto cstatus = map_->GetVoxState(p);
                //     cout<<"cstatus:"<<int(cstatus)<<endl;

                //     cout<<"p1:"<<p1.transpose()<<endl;
                //     cout<<"p2i:"<<map_->PostoId(p)<<endl;
                //     cout<<"cn id:"<<cn.first<<endl;
                //     cout<<"cn s:"<<int(cn.second.first)<<endl;
                //     cout<<"cn n:"<<int(cn.second.second)<<endl;
                //     StopageDebug("SetNodeStatesBf not work1.5");
                // }
            }
            else{
                if(!(LG_[bid]->local_grid_[nid]->flags_ & 1)){
                    LG_[bid]->local_grid_[nid]->flags_ |= 1;
                    if(LG_[bid]->local_grid_[nid]->free_num_ == nv_num_){
                        f_list.push_back({bid, nid});
                    }
                    else{
                        o_list.push_back({bid, nid});
                    }
                }
                LG_[bid]->local_grid_[nid]->free_num_++;
                LG_[bid]->local_grid_[nid]->unknown_num_--;

                // int x = nid % block_size_(0);
                // int y = ((nid - x) / block_size_(0)) % block_size_(1);
                // int z = ((nid - x - y*block_size_(0))) /(block_size_(1)*block_size_(0));
                // Eigen::Vector3i pi = LG_[bid]->origin_ + Eigen::Vector3i(x, y, z);
                // Eigen::Vector3d p1;
                // int f = 0, u = 0;
                // for(int xi = 0; xi < node_size_(0); xi++)
                //     for(int yi = 0; yi < node_size_(1); yi++)
                //         for(int zi = 0; zi < node_size_(2); zi++){
                //             p1 = pi.cast<double>().cwiseProduct(node_scale_) + Eigen::Vector3d(0.5+xi, 0.5+yi, 0.5+zi) * resolution_ + origin_;
                //             auto cstatus = map_->GetVoxState(p1);
                //             if(cstatus == VoxelState::free) f++;
                //             else if(cstatus == VoxelState::unknown) u++;
                //             else if(cstatus == VoxelState::outlocal){   

                //             }
                // }
                // if(u == LG_[bid]->local_grid_[nid]->unknown_num_ && f != LG_[bid]->local_grid_[nid]->free_num_){
                //     cout<<"p:"<<p.transpose()<<endl;
                //     auto cstatus = map_->GetVoxState(p);
                //     cout<<"cstatus:"<<int(cstatus)<<endl;
                //     cout<<"f:"<<int(f)<<"  u:"<<int(u)<<endl;
                //     cout<<"g->f:"<<int(LG_[bid]->local_grid_[nid]->free_num_)<<"  g->u:"<<int(LG_[bid]->local_grid_[nid]->unknown_num_)<<endl;
                //     cout<<"p1:"<<p1.transpose()<<endl;
                //     cout<<"cn id:"<<cn.first<<endl;
                //     cout<<"cn s:"<<int(cn.second.first)<<endl;
                //     cout<<"cn n:"<<int(cn.second.second)<<endl;
                //     StopageDebug("SetNodeStatesBf not work2");
                // }
            }
        }
        else if(cn.second.first == 2){
            if(cn.second.second & 1){
                if(!(LG_[bid]->local_grid_[nid]->flags_ & 1)){
                    LG_[bid]->local_grid_[nid]->flags_ |= 1;
                    if(LG_[bid]->local_grid_[nid]->free_num_ == nv_num_){
                        f_list.push_back({bid, nid});
                    }
                    else{
                        o_list.push_back({bid, nid});
                    }
                }
                LG_[bid]->local_grid_[nid]->free_num_--;

                // int x = nid % block_size_(0);
                // int y = ((nid - x) / block_size_(0)) % block_size_(1);
                // int z = ((nid - x - y*block_size_(0))) /(block_size_(1)*block_size_(0));
                // Eigen::Vector3i pi = LG_[bid]->origin_ + Eigen::Vector3i(x, y, z);
                // Eigen::Vector3d p1;
                // int f = 0, u = 0;
                // for(int xi = 0; xi < node_size_(0); xi++)
                //     for(int yi = 0; yi < node_size_(1); yi++)
                //         for(int zi = 0; zi < node_size_(2); zi++){
                //             p1 = pi.cast<double>().cwiseProduct(node_scale_) + Eigen::Vector3d(0.5+xi, 0.5+yi, 0.5+zi) * resolution_ + origin_;
                //             auto cstatus = map_->GetVoxState(p1);
                //             if(cstatus == VoxelState::free) f++;
                //             else if(cstatus == VoxelState::unknown) u++;
                //             else if(cstatus == VoxelState::outlocal){   

                //             }
                // }
                // if(u == LG_[bid]->local_grid_[nid]->unknown_num_ && f != LG_[bid]->local_grid_[nid]->free_num_){
                //     cout<<"bid:"<<bid<<endl;
                //     cout<<"nid:"<<nid<<endl;
                //     cout<<"x:"<<x<<"  y:"<<y<<"  z:"<<z<<endl;
                //     cout<<"f:"<<int(f)<<"  u:"<<int(u)<<endl;
                //     cout<<"g->f:"<<int(LG_[bid]->local_grid_[nid]->free_num_)<<"  g->u:"<<int(LG_[bid]->local_grid_[nid]->unknown_num_)<<endl;
                //     cout<<"p:"<<p.transpose()<<endl;
                //     auto cstatus = map_->GetVoxState(p);
                //     cout<<"cstatus:"<<int(cstatus)<<endl;

                //     cout<<"p1:"<<p1.transpose()<<endl;
                //     cout<<"p2i:"<<map_->PostoId(p)<<endl;
                //     cout<<"cn id:"<<cn.first<<endl;
                //     cout<<"cn s:"<<int(cn.second.first)<<endl;
                //     cout<<"cn n:"<<int(cn.second.second)<<endl;
                //     StopageDebug("SetNodeStatesBf not work3");
                // }
            }
        }
        else if(cn.second.first == 3){
            if(cn.second.second & 1){
                if(!(LG_[bid]->local_grid_[nid]->flags_ & 1)){
                    LG_[bid]->local_grid_[nid]->flags_ |= 1;
                    if(LG_[bid]->local_grid_[nid]->free_num_ == nv_num_){
                        f_list.push_back({bid, nid});
                    }
                    else{
                        o_list.push_back({bid, nid});
                    }
                }
                LG_[bid]->local_grid_[nid]->free_num_++;

                // int x = nid % block_size_(0);
                // int y = ((nid - x) / block_size_(0)) % block_size_(1);
                // int z = ((nid - x - y*block_size_(0))) /(block_size_(1)*block_size_(0));
                // Eigen::Vector3i pi = LG_[bid]->origin_ + Eigen::Vector3i(x, y, z);
                // Eigen::Vector3d p1;
                // int f = 0, u = 0;
                // for(int xi = 0; xi < node_size_(0); xi++)
                //     for(int yi = 0; yi < node_size_(1); yi++)
                //         for(int zi = 0; zi < node_size_(2); zi++){
                //             p1 = pi.cast<double>().cwiseProduct(node_scale_) + Eigen::Vector3d(0.5+xi, 0.5+yi, 0.5+zi) * resolution_ + origin_;
                //             auto cstatus = map_->GetVoxState(p1);
                //             if(cstatus == VoxelState::free) f++;
                //             else if(cstatus == VoxelState::unknown) u++;
                //             else if(cstatus == VoxelState::outlocal){   

                //             }
                // }
                // if(u == LG_[bid]->local_grid_[nid]->unknown_num_ && f != LG_[bid]->local_grid_[nid]->free_num_){
                //     cout<<"bid:"<<bid<<endl;
                //     cout<<"nid:"<<nid<<endl;
                //     cout<<"x:"<<x<<"  y:"<<y<<"  z:"<<z<<endl;
                //     cout<<"f:"<<int(f)<<"  u:"<<int(u)<<endl;
                //     cout<<"g->f:"<<int(LG_[bid]->local_grid_[nid]->free_num_)<<"  g->u:"<<int(LG_[bid]->local_grid_[nid]->unknown_num_)<<endl;
                //     cout<<"p:"<<p.transpose()<<endl;
                //     auto cstatus = map_->GetVoxState(p);
                //     cout<<"cstatus:"<<int(cstatus)<<endl;

                //     cout<<"p1:"<<p1.transpose()<<endl;
                //     cout<<"p2i:"<<map_->PostoId(p)<<endl;
                //     cout<<"cn id:"<<cn.first<<endl;
                //     cout<<"cn s:"<<int(cn.second.first)<<endl;
                //     cout<<"cn n:"<<int(cn.second.second)<<endl;
                //     StopageDebug("SetNodeStatesBf not work4");
                // }
            }
        }
    }

    // new occ nodes
    for(int i = 0; i < o_list.size(); i++){
        bid = o_list[i].first;
        nid = o_list[i].second;
        LG_[bid]->local_grid_[nid]->flags_ &= 254;
        if(LG_[bid]->local_grid_[nid]->free_num_ != nv_num_){
            continue;
        }
        else{
            // int x = bid % local_block_num_(0);
            // int y = ((bid - x)/local_block_num_(0)) % local_block_num_(1);
            // int z = ((bid - x) - y*local_block_num_(0))/l_b_n_(1);

            c(0) = nid % block_size_(0);
            c(1) = ((nid - c(0)) / block_size_(0)) % block_size_(0);
            c(2) = (nid - c(0) - c(1)*block_size_(0)) / (block_size_(0)*block_size_(1));
            p = (LG_[bid]->origin_+c).cast<double>().cwiseProduct(node_scale_) + origin_ + node_scale_ * 0.5;
            new_free_nodes.emplace_back(p);
            if(showmap_ && !LG_[bid]->show_flag_){
                LG_[bid]->show_flag_ = true;
                gbid = LocalBid2GlobalBid(bid);
                if(gbid == -1){
                    StopageDebug("SetNodeStates, error gbid");
                }
                Showblocklist_.emplace_back(gbid);
            }
        }
    }

    // new free nodes
    for(int i = 0; i < f_list.size(); i++){
        bid = f_list[i].first;
        nid = f_list[i].second;
        LG_[bid]->local_grid_[nid]->flags_ &= 254;
        if(LG_[bid]->local_grid_[nid]->free_num_ == nv_num_){
            continue;
        }
        else{
            if(showmap_ && !LG_[bid]->show_flag_){
                LG_[bid]->show_flag_ = true;
                gbid = LocalBid2GlobalBid(bid);
                if(gbid == -1){
                    StopageDebug("SetNodeStates, error gbid");
                }
                Showblocklist_.emplace_back(gbid);
            }
        }
    }

}

int LowResMap::PathCheck(list<Eigen::Vector3d> &path, bool allow_uknown){
    // ROS_WARN("PathCheck0");
    for(auto &p : path){
        if(!InsideLocalMap(p)) return 1;
    }
    // ROS_WARN("PathCheck1");

    if(path.size() == 0) {
        ROS_ERROR("strange path! %ld", path.size());
        StopageDebug("strange path!");
        return 0;
    // ROS_WARN("PathCheck2");
    }
    if(path.size() == 1) {
        if(!InsideLocalMap(path.front())){
            return 1;
        }
        if(!IsFeasible(path.front(), allow_uknown)) {
            return 2;
        }
        return 0;
    }

    RayCaster rc;
    Eigen::Vector3d inv_res, ray_iter;
    Eigen::Vector3d half_res = 0.5 * node_scale_;
    list<Eigen::Vector3d>::iterator ps_it, pe_it;
    for(int dim = 0; dim < 3; dim++) inv_res(dim) = 1.0 / node_scale_(dim);
    // ROS_WARN("PathCheck3");

    ps_it = path.begin();
    pe_it = path.begin();
    pe_it++;
    while(pe_it != path.end()){
        rc.setInput((*pe_it - origin_).cwiseProduct(inv_res), (*ps_it - origin_).cwiseProduct(inv_res));
        // ROS_WARN("PathCheck4");

        while(rc.step(ray_iter)){

            ray_iter = ray_iter.cwiseProduct(node_scale_) + origin_ + half_res;
            
            if(!InsideLocalMap(ray_iter)){
                if(allow_uknown){
                    continue;
                }              
                else return 1;
            }
            // ROS_WARN("PathCheck4.3");

            if(!IsFeasible(ray_iter, allow_uknown)) {
                // ROS_WARN("PathCheck4.35");
                return 2;
            }
            // ROS_WARN("PathCheck4.4");

        }
        // ROS_WARN("PathCheck5");

        ray_iter = ray_iter.cwiseProduct(node_scale_) + origin_ + half_res;
        if(!InsideLocalMap(ray_iter)){
            if(allow_uknown){
                ps_it++;
                pe_it++;
                continue;
            }              
            else return 1;
        }
        // ROS_WARN("PathCheck6");

        if(!IsFeasible(ray_iter, allow_uknown)) {
            return 2;
        }
        ps_it++;
        pe_it++;
        // ROS_WARN("PathCheck7");

    }
    // ROS_WARN("PathCheck8");

    return 0;
}

int LowResMap::PathCheck(vector<Eigen::Vector3d> &path, bool allow_uknown){
    for(auto &p : path){
        if(!InsideLocalMap(p)) return 1;
    }
    if(path.size() <= 1) {
        ROS_ERROR("strange path! %ld", path.size());
        return 0;
    }
    RayCaster rc;
    Eigen::Vector3d inv_res, ray_iter;
    Eigen::Vector3d half_res = 0.5 * node_scale_;
    vector<Eigen::Vector3d>::iterator ps_it, pe_it;
    for(int dim = 0; dim < 3; dim++) inv_res(dim) = 1.0 / node_scale_(dim);

    ps_it = path.begin();
    pe_it = path.begin();
    pe_it++;
    while(pe_it != path.end()){
        rc.setInput((*pe_it - origin_).cwiseProduct(inv_res), (*ps_it - origin_).cwiseProduct(inv_res));
        while(rc.step(ray_iter)){
            ray_iter = ray_iter.cwiseProduct(node_scale_) + origin_ + half_res;
            if(!InsideLocalMap(ray_iter)){
                if(allow_uknown){
                    continue;
                }              
                else return 1;
            }
            if(!IsFeasible(ray_iter, allow_uknown)) {
                return false;
            }
        }
        ray_iter = ray_iter.cwiseProduct(node_scale_) + origin_ + half_res;
        if(!InsideLocalMap(ray_iter)){
            if(allow_uknown){
                ps_it++;
                pe_it++;
                continue;
            }              
            else return 1;
        }
        if(!IsFeasible(ray_iter, allow_uknown)) {
            return false;
        }
        ps_it++;
        pe_it++;
    }
    return true;
}

bool LowResMap::CheckPathHomo(list<Eigen::Vector3d> &path1, list<Eigen::Vector3d> &path2){
    double l1, l2;
    

    return false;
}

int LowResMap::FindCorridors(const vector<Eigen::Vector3d> path, 
    vector<Eigen::MatrixX4d> &corridors, 
    vector<Eigen::Matrix3Xd> &corridorVs,
    vector<Eigen::Vector3d> &pruned_path,
    bool allow_unknown,
    double prune_length){
    vector<Eigen::Vector3d> path_cast, path_local;
    Eigen::Vector3i cor_size, cor_sidx, cor_eidx;
    Eigen::Vector3d cor_start, cor_end;
    double cur_total_length = 0, cur_length = 0;
    bool newseg_flag = true;
    int end_idx = 0, start_idx = 0;
    pruned_path.clear();
    corridors.clear();
    corridorVs.clear(); 
    int s = 1;
    for(auto pt : path){
        // if(allow_unknown) cout<<"pt:"<<pt.transpose()<<endl;
        if(!IsFeasible(pt, allow_unknown)){
            // cout<<"allow_unknown:"<<allow_unknown<<endl;
            cout<<"!!!!"<<endl;
            for(auto &p : path){
                cout<<"p:"<<p.transpose()<<"  "<<IsFeasible(p, true)<<"  "<<IsFeasible(p, false)<<"  "<< InsideLocalMap(p)<<endl;
            }
            StopageDebug("FindCorridors infeasible");
        }
        if(InsideLocalMap(pt)) path_local.emplace_back(pt);
        else {

            cout<<"!!!!"<<endl;
            for(auto &p : path){
                cout<<"p:"<<p.transpose()<<"  "<<InsideLocalMap(pt)<<endl;
            }
            s = 2;
            break;
        }
    }

    if(!path_local.empty()){
        path_cast.emplace_back(path_local.front());
        // cout<<"pca:"<<path_local.front().transpose()<<endl;
        // path_cast.emplace_back(GetStdPos(path.front()));
    }

    RayCaster rc;

    for(auto pt : path_local){
        int last_id = GlobalPos2GlobalNodeId(path_cast.back());
        if(GlobalPos2GlobalNodeId(pt) != last_id){
            Eigen::Vector3d rc_s, rc_e, it;
            rc_s = path_cast.back() - origin_;
            rc_e = pt - origin_;
            for(int dim = 0; dim < 3; dim ++){
                rc_s(dim) /= node_scale_(dim);
                rc_e(dim) /= node_scale_(dim);
            }

            rc.setInput(rc_s, rc_e);
            while (rc.step(it)) {
                it = it.cwiseProduct(node_scale_) + origin_ + node_scale_/2;
                int id = GlobalPos2GlobalNodeId(it);
                if(id == last_id) continue;
                // if(allow_unknown) cout<<"pca:"<<it.transpose()<<endl;

                path_cast.emplace_back(it);
                // cout<<"pca:"<<it.transpose()<<endl;
            }
            it = it.cwiseProduct(node_scale_) + origin_ + node_scale_/2;
            int id = GlobalPos2GlobalNodeId(it);
            if(id != last_id) {
                path_cast.emplace_back(it);
                // cout<<"pca:"<<it.transpose()<<endl;
                
                // if(allow_unknown) cout<<"pca:"<<it.transpose()<<endl;
            }
        }
    }
    path_cast.emplace_back(path_local.back());
    // cout<<"pca:"<<path_local.back().transpose()<<endl;

    // if(allow_unknown) cout<<"pca:"<<path_local.back().transpose()<<endl;
    if(path_cast.size() == 1 && s == 1){
        path_cast.emplace_back(path_cast[0]);
        // cout<<"pca0:"<<path_cast[0].transpose()<<endl;
        // if(allow_unknown) cout<<"pca:"<<path_cast[0].transpose()<<endl;
    }

    pruned_path.emplace_back(path_local[0]);
    // cout<<"pruned_path"<<pruned_path.size()<<"  :"<<pruned_path.back().transpose()<<endl;


    while(1){

        if(newseg_flag){
            cor_size.setOnes();
            cur_length = 0;
            start_idx = end_idx;
            cor_start = path_cast[start_idx];
            cor_end = cor_start;
            // pruned_path.emplace_back(cor_start);
            // cout<<"pruned_path_n"<<pruned_path.size()<<"  :"<<pruned_path.back().transpose()<<endl;
            PostoId3(cor_start, cor_sidx);
            cor_eidx = cor_sidx;
            newseg_flag = false;
        }

        end_idx++;
        cur_length += (path_cast[end_idx] - path_cast[end_idx - 1]).norm();
        cur_total_length += (path_cast[end_idx] - path_cast[end_idx - 1]).norm();

        bool success = ExpandPath(cor_sidx, cor_eidx, path_cast[end_idx], allow_unknown);
        // cout<<"end_idx:"<<end_idx<<"  success:"<<success<<endl;

        if(!success || cur_length >= seg_length_ || end_idx + 1 >= path_cast.size() || start_idx == 0){
            
            if(end_idx == start_idx && cur_length < seg_length_){
                ROS_ERROR("error path");
                return 0;
            }

            Eigen::Vector3d up_corner, down_corner;
            CoarseExpand(cor_sidx, cor_eidx, allow_unknown);
            FineExpand(cor_sidx, cor_eidx, up_corner, down_corner, allow_unknown);

            Eigen::MatrixX4d h(6, 4);
            Eigen::Matrix3Xd p(3, 8);
            h.setZero();
            p.setZero();
            for(int dim = 0; dim < 3; dim++){
                up_corner(dim) -= Robot_size_(dim) * 0.5;
                down_corner(dim) += Robot_size_(dim) * 0.5;
                if(up_corner(dim) <= down_corner(dim)) {
                    ROS_ERROR("narrow!!!");
                    return 0;
                }
                h(dim*2, dim) = 1;
                h(dim*2, 3) = -up_corner(dim);
                h(dim*2 + 1, dim) = -1;
                h(dim*2 + 1, 3) = down_corner(dim);
            }

            for(int dim1 = 0; dim1 <= 1; dim1++){
                for(int dim2 = 0; dim2 <= 1; dim2++){
                    for(int dim3 = 0; dim3 <= 1; dim3++){
                        p(0, 4*dim3 + 2*dim2 + dim1) = dim1 ? down_corner(0) : up_corner(0);
                        p(1, 4*dim3 + 2*dim2 + dim1) = dim2 ? down_corner(1) : up_corner(1);
                        p(2, 4*dim3 + 2*dim2 + dim1) = dim3 ? down_corner(2) : up_corner(2);
                    }
                }
            }

            if(!success) {
                end_idx--;
                if(start_idx == end_idx){
                    cout<<path_cast[end_idx+1].transpose()<<"   "<<path_cast[end_idx].transpose()<<endl;

                    Eigen::Vector3d max_p, min_p;
                    for(int dim = 0; dim < 3; dim++){
                        max_p(dim) = max(path_cast[end_idx + 1](dim), path_cast[end_idx](dim));
                        min_p(dim) = min(path_cast[end_idx + 1](dim), path_cast[end_idx](dim));
                    }
                    for(double x = min_p(0); x < max_p(0) + 1e-3; x += 0.7){
                        for(double y = min_p(1); y < max_p(1) + 1e-3; y += 0.7){
                            for(double z = min_p(2); z < max_p(2) + 1e-3; z += 0.7){
                                Eigen::Vector3d pc(x, y, z);
                                // cout<<"pc:"<<pc.transpose()<<"  fea:"<<IsFeasible(pc, allow_unknown)<<endl;
                            }
                        }
                    }

                    StopageDebug("start_idx == end_idx");
                }
            }
            corridors.emplace_back(h);
            corridorVs.emplace_back(p);
            pruned_path.emplace_back(path_cast[end_idx]);
            // cout<<"pruned_path_f"<<pruned_path.size()<<"  :"<<pruned_path.back().transpose()<<endl;
            newseg_flag = true;
            if(end_idx + 1 >= path_cast.size() || cur_total_length >= prune_length){
                if(end_idx + 1 >= path_cast.size()) s = 1;
                else {
                    cout<<"???"<<endl;
                    cout<<"end_idx:"<<end_idx<<endl;
                    cout<<"success:"<<success<<endl;
                    s = 2;
                }
                if(corridors.size() == 1){
                    corridors.emplace_back(corridors.back());
                    corridorVs.emplace_back(corridorVs.back());
                }
                break;
            }
        }
    }
    return s;
}


bool LowResMap::ExpandPath(Eigen::Vector3i &cor_start, Eigen::Vector3i &cor_end, const Eigen::Vector3d &pos, bool allow_unknown){
    Eigen::Vector3i corp_idx, d_corsize, cor_start_temp, cor_end_temp, cor_it;
    cor_start_temp = cor_start;
    cor_end_temp = cor_end;
    PostoId3(pos, corp_idx);
    for(int i = 0; i < 3; i++){
        d_corsize = cor_end_temp - cor_start_temp + Eigen::Vector3i(1, 1, 1);
        cor_it = cor_start_temp;
        
        if(corp_idx(i) < cor_start_temp(i)){
            d_corsize(i) = cor_start_temp(i) - corp_idx(i);
            cor_it(i) = corp_idx(i);
            cor_start_temp(i) = corp_idx(i);
        }
        else if(corp_idx(i) > cor_end_temp(i)){
            d_corsize(i) = corp_idx(i) - cor_end_temp(i);
            cor_it(i) = corp_idx(i);
            cor_end_temp(i) = corp_idx(i);
        } 
        else{
            continue;
        }


        for(int x = 0; x < d_corsize(0); x++){
            for(int y = 0; y < d_corsize(1); y++){
                for(int z = 0; z < d_corsize(2); z++){
                    Eigen::Vector3i chk_idx = cor_it + Eigen::Vector3i(x, y, z);
                    if(!IsFeasible(chk_idx, allow_unknown)) return false;
                }
            }
        }   
    }
    cor_start = cor_start_temp;
    cor_end = cor_end_temp;
    return true;
}

void LowResMap::CoarseExpand(Eigen::Vector3i &coridx_start, Eigen::Vector3i &coridx_end, bool allow_unknown){
    vector<Eigen::Vector3i> corners(2);
    vector<bool> expand(6, true);
    vector<int> expanded(6, 0);
    vector<Eigen::Vector3d> debug_pts;
    corners[0] = coridx_end;
    corners[1] = coridx_start;
    while(1){
        bool expandable = false;
        Eigen::Vector3i cd = corners[0] - corners[1];
        list<int> prio;
        bool push_back = true;
        for(int dim = 0; dim < 3; dim++){
            for(auto i = prio.begin(); i != prio.end(); i++){
                if(abs(cd(*i)) > abs(cd(dim))){
                    prio.insert(i, dim);
                    push_back = false;
                    break;
                }
            }
            if(push_back) prio.emplace_back(dim);
        }
        for(auto &dim : prio){
            for(int dir = -1; dir <= 1; dir += 2){
                if(!expand[dim*2 + int((1-dir)/2)]) continue;
                Eigen::Vector3i corridor_scale = corners[0] - corners[1] + Eigen::Vector3i::Ones();
                // Eigen::Vector3i corridor_center = (corners[0] + corners[1]) * 0.5;
                Eigen::Vector3i chk_it1, chk_it2;
                Eigen::Vector3i start_p = corners[1];
                start_p(dim) = corners[int((1-dir)/2)](dim) + dir;
                // start_p(dim) = corridor_center(dim) + dir * 0.5 * (corridor_scale(dim) + resolution_);
                if(!InsideMap(start_p)) expand[dim*2 + int((1-dir)/2)] = false;

                for(chk_it1(0) = 0; chk_it1(0) < (dim == 0 ? 1 : corridor_scale(0)) - 1e-4 && expand[dim*2 + int((1-dir)/2)]; chk_it1(0)++){
                    for(chk_it1(1) = 0; chk_it1(1) < (dim == 1 ? 1 : corridor_scale(1)) - 1e-4 && expand[dim*2 + int((1-dir)/2)]; chk_it1(1)++){
                        for(chk_it1(2) = 0; chk_it1(2) < (dim == 2 ? 1 : corridor_scale(2)) - 1e-4 && expand[dim*2 + int((1-dir)/2)]; chk_it1(2)++){
                            chk_it2 = start_p + chk_it1;
                            if(!IsFeasible(chk_it2, allow_unknown)){
                                expand[dim*2 + int((1-dir)/2)] = false;
                            }
                        }
                    }
                }

                expandable |= expand[dim*2 + int((1-dir)/2)];


                if(expand[dim*2 + int((1-dir)/2)]){
                    corners[int((1-dir)/2)](dim) += dir;
                    expanded[dim*2 + int((1-dir)/2)] += 1;
                    expand[dim*2 + int((1-dir)/2)] = (expanded[dim*2 + int((1-dir)/2)] < corridor_exp_r_(dim)) ? 1 : 0;
                }
            }
        }
        
        if(!expandable) break;
    }
    coridx_end = corners[0];
    coridx_start = corners[1];
}

void LowResMap::FineExpand(Eigen::Vector3i &coridx_start, Eigen::Vector3i &coridx_end,  Eigen::Vector3d &up_corner, Eigen::Vector3d &down_corner, bool allow_unknown){
    vector<Eigen::Vector3d> corners(2);
    vector<bool> expand(6, true);
    vector<double> expanded(6, 0.0);
    VoxelState vs;
    corners[0] = (coridx_end.cast<double>() + Eigen::Vector3d::Ones()).cwiseProduct(node_scale_) + origin_ - Eigen::Vector3d::Ones() * 1e-4;
    corners[1] = (coridx_start.cast<double>()).cwiseProduct(node_scale_) + origin_ + Eigen::Vector3d::Ones() * 1e-4;
    // cout<<"up_corner:"<<corners[0].transpose()<<endl;
    // cout<<"down_corner:"<<corners[1].transpose()<<endl;
    while(1){
        bool expandable = false;
        Eigen::Vector3d cd = corners[1] - corners[0];
        list<int> prio;
        bool push_back = true;
        for(int dim = 0; dim < 3; dim++){
            for(auto i = prio.begin(); i != prio.end(); i++){
                if(abs(cd(*i)) > abs(cd(dim))){
                    prio.insert(i, dim);
                    push_back = false;
                    break;
                }
            }
            if(push_back) prio.emplace_back(dim);
        }
        for(auto &dim : prio){
            for(int dir = -1; dir <= 1; dir += 2){
                if(!expand[dim*2 + int((1-dir)/2)]) continue;
                Eigen::Vector3d corridor_scale = corners[0] - corners[1];
                Eigen::Vector3d corridor_center = (corners[0] + corners[1]) * 0.5;
                Eigen::Vector3d chk_it1, chk_it2;
                Eigen::Vector3d start_p = corners[1] + Eigen::Vector3d::Ones() * resolution_ * 0.5;
                start_p(dim) = corridor_center(dim) + dir * 0.5 * (corridor_scale(dim) + resolution_);
                if(!InsideMap(start_p)) expand[dim*2 + int((1-dir)/2)] = false;
                for(chk_it1(0) = 0.0; chk_it1(0) < (dim == 0 ? resolution_ : corridor_scale(0)) - 1e-4 && expand[dim*2 + int((1-dir)/2)]; chk_it1(0) += resolution_){
                    for(chk_it1(1) = 0.0; chk_it1(1) < (dim == 1 ? resolution_ : corridor_scale(1)) - 1e-4 && expand[dim*2 + int((1-dir)/2)]; chk_it1(1) += resolution_){
                        for(chk_it1(2) = 0.0; chk_it1(2) < (dim == 2 ? resolution_ : corridor_scale(2))  - 1e-4 && expand[dim*2 + int((1-dir)/2)]; chk_it1(2) += resolution_){
                            chk_it2 = start_p + chk_it1;
                            // debug_pts.push_back(chk_it2);
                            vs = map_->GetVoxState(chk_it2);
                            if((!allow_unknown && vs != VoxelState::free) || (allow_unknown && vs == VoxelState::occupied)){
                                // ROS_ERROR("occ");
                                expand[dim*2 + int((1-dir)/2)] = false;
                            }
                        }
                    }
                }

                expandable |= expand[dim*2 + int((1-dir)/2)];
                if(expand[dim*2 + int((1-dir)/2)]){
                    corners[int((1-dir)/2)](dim) += resolution_ * dir;
                    expanded[dim*2 + int((1-dir)/2)] += resolution_;
                    expand[dim*2 + int((1-dir)/2)] = (expanded[dim*2 + int((1-dir)/2)] < expand_r_(dim) * 0.5 - 1e-4) ? 1 : 0;
                }
            }
        }
        if(!expandable) break;
    }
    up_corner = corners[0];
    down_corner = corners[1];
    // cout<<"up_corner1:"<<up_corner.transpose()<<endl;
    // cout<<"down_corner1:"<<down_corner.transpose()<<endl;
}

bool LowResMap::PrunePath(const list<Eigen::Vector3d> &path, list<Eigen::Vector3d> &pruned_path, double &length){
    length = 0;
    if(path.size() == 0) return false;
    else if(path.size() == 1){
        pruned_path = path;
        return true;
    }
    pruned_path.clear();
    Eigen::Vector3d inv_res, ray_iter;
    Eigen::Vector3d half_res = 0.5 * node_scale_;
    Eigen::Vector3i p_i;
    list<Eigen::Vector3d> std_off_path;
    list<Eigen::Vector3d>::iterator ps_it, pe_it;
    bool free_ray;
    RayCaster rc;

    for(auto &p : path){
        if(!InsideLocalMap(p)) return false;
        std_off_path.push_back(GetStdPos(p));
    }

    pruned_path.push_back(std_off_path.front());
    for(int dim = 0; dim < 3; dim++) inv_res(dim) = 1.0 / node_scale_(dim);
    for(list<Eigen::Vector3d>::iterator ps_it = std_off_path.begin(); pe_it != std_off_path.end(); pe_it++){
        pe_it = ps_it;
        double seg_length = 0;
        for(list<Eigen::Vector3d>::iterator pf_it = pe_it; pe_it != std_off_path.end() && seg_length < prune_seg_length_; pe_it++) {
            seg_length += ((*pf_it) - (*pe_it)).norm();
            pf_it = pe_it;
        }
        if(pe_it != std_off_path.end()) pe_it--;
        pe_it--;

        while (1)
        {
            if(ps_it == pe_it) return false;
            free_ray = true;
            rc.setInput((*ps_it - origin_).cwiseProduct(inv_res), (*pe_it - origin_).cwiseProduct(inv_res));
            while(rc.step(ray_iter)){
                ray_iter = ray_iter.cwiseProduct(node_scale_) + origin_ + half_res;
                if(!IsFeasible(ray_iter)) {
                    free_ray = false;
                    break;
                }
            }
            ray_iter = ray_iter.cwiseProduct(node_scale_) + origin_ + half_res;
            if(!IsFeasible(ray_iter)) {
                free_ray = false;
            }

            if(free_ray) {
                length += (pruned_path.back() - (*pe_it)).norm();
                pruned_path.push_back(*pe_it);
                break;
            }
            pe_it--;
        }
        ps_it = pe_it;
    }
    length += (pruned_path.back() - path.back()).norm();
    length += (pruned_path.front() - path.front ()).norm();
    pruned_path.push_back(path.back());
    pruned_path.push_front(path.front());
    return true;
}

bool LowResMap::PrunePath(const vector<Eigen::Vector3d> &path, vector<Eigen::Vector3d> &pruned_path, double &length){
    length = 0;
    if(path.size() == 0) return false;
    else if(path.size() == 1){
        pruned_path = path;
        return true;
    }
    pruned_path.clear();
    Eigen::Vector3d inv_res, ray_iter;
    Eigen::Vector3d half_res = 0.5 * node_scale_;
    Eigen::Vector3i p_i;
    list<Eigen::Vector3d> std_off_path;
    list<Eigen::Vector3d>::iterator ps_it, pe_it;
    bool free_ray;
    RayCaster rc;

    for(auto &p : path){
        if(!InsideLocalMap(p)) return false;
        std_off_path.push_back(GetStdPos(p));
    }
    pruned_path.push_back(path.front());
    pruned_path.push_back(std_off_path.front());
    for(int dim = 0; dim < 3; dim++) inv_res(dim) = 1.0 / node_scale_(dim);
    for(list<Eigen::Vector3d>::iterator ps_it = std_off_path.begin(); pe_it != std_off_path.end(); pe_it++){
        pe_it = ps_it;
        double seg_length = 0;
        for(list<Eigen::Vector3d>::iterator pf_it = pe_it; pe_it != std_off_path.end() && seg_length < prune_seg_length_; pe_it++) {
            seg_length += ((*pf_it) - (*pe_it)).norm();
            pf_it = pe_it;
        }
        if(pe_it != std_off_path.end()) pe_it--;
        pe_it--;

        while (1)
        {
            if(ps_it == pe_it) return false;
            free_ray = true;
            rc.setInput((*ps_it - origin_).cwiseProduct(inv_res), (*pe_it - origin_).cwiseProduct(inv_res));
            while(rc.step(ray_iter)){
                ray_iter = ray_iter.cwiseProduct(node_scale_) + origin_ + half_res;
                if(!IsFeasible(ray_iter)) {
                    free_ray = false;
                    break;
                }
            }
            ray_iter = ray_iter.cwiseProduct(node_scale_) + origin_ + half_res;
            if(!IsFeasible(ray_iter)) {
                free_ray = false;
            }

            if(free_ray) {
                length += (pruned_path.back() - (*pe_it)).norm();
                pruned_path.push_back(*pe_it);
                break;
            }
            pe_it--;
        }
        ps_it = pe_it;
    }
    length += (pruned_path.back() - path.back()).norm();
    length += (pruned_path.front() - path.front ()).norm();
    pruned_path.push_back(path.back());
    return true;
}

bool LowResMap::CheckThorough(){
    int c = 0;
    for(int bz = local_origin_idx_(2); bz <= local_up_idx_(2); bz++)
    for(int by = local_origin_idx_(1); by <= local_up_idx_(1); by++)
    for(int bx = local_origin_idx_(0); bx <= local_up_idx_(0); bx++){
        Eigen::Vector3i boi = Eigen::Vector3i(bx, by, bz).cwiseProduct(block_size_);
        if((boi - LG_[c]->origin_).norm() > 0){
            cout<<"local_origin_idx_:"<<local_origin_idx_.transpose()<<endl;
            cout<<"local_up_idx_:"<<local_up_idx_.transpose()<<endl;
            cout<<"boi:"<<boi.transpose()<<endl;
            // cout<<"c:"<<c<<endl;
            cout<<"LG_[c]->origin_:"<<LG_[c]->origin_.transpose()<<endl;
            int bid = bx + by * b_n_(0) + bz * b_n_(1);
            Eigen::Vector3i bidx3;
            bidx3(0) = bid % block_num_(0);
            bidx3(1) = ((bid - bidx3(0))/block_num_(0)) % block_num_(1);
            bidx3(2) = ((bid - bidx3(0)) - bidx3(1)*block_num_(0))/b_n_(1);
            cout<<"bidx3:"<<bidx3.transpose()<<endl;


            StopageDebug("CheckThorough (boi - LG_[c]->origin_).norm() > 0");
        }
        c++;
    }
    for(auto &g : LG_){
        int  id = 0;
        for(int z = 0; z < block_size_(2); z++){
            for(int y = 0; y < block_size_(1); y++){
                for(int x = 0; x < block_size_(0); x++){
                    Eigen::Vector3i pi = g->origin_ + Eigen::Vector3i(x, y, z);
                    if(InsideLocalMap(pi) && (g->local_grid_[id] == NULL || g->local_grid_[id] == Outnode_)) {
                        StopageDebug("CheckThorough InsideLocalMap(pi) && (g->local_grid_[id] == NULL || g->local_grid_[id] == Outnode_");
                    }
                    if(g->local_grid_[id] == NULL) continue;
                    if(g->local_grid_[id] == Outnode_) continue;
                    Eigen::Vector3d p;
                    int f = 0, u = 0;
                    for(int xi = 0; xi < node_size_(0); xi++)
                        for(int yi = 0; yi < node_size_(1); yi++)
                            for(int zi = 0; zi < node_size_(2); zi++){
                                p = pi.cast<double>().cwiseProduct(node_scale_) + Eigen::Vector3d(0.5+xi, 0.5+yi, 0.5+zi) * resolution_ + origin_;
                                auto cstatus = map_->GetVoxState(p);
                                if(cstatus == VoxelState::free) f++;
                                else if(cstatus == VoxelState::unknown) u++;
                                else if(cstatus == VoxelState::outlocal){   
                                    cout<<"local_map_upbd_:"<<local_map_upbd_.transpose()<<endl;
                                    cout<<"local_map_lowbd_:"<<local_map_lowbd_.transpose()<<endl;
                                    cout<<"local_upbd_:"<<map_->local_upbd_.transpose()<<endl;
                                    cout<<"local_lowbd_:"<<map_->local_lowbd_.transpose()<<endl;
                                    cout<<"p:"<<p.transpose()<<endl;
                                    StopageDebug("CheckThorough outlocal");
                                }
                    }
                    if(f != g->local_grid_[id]->free_num_ || u != g->local_grid_[id]->unknown_num_){
                        cout<<"pi:"<<pi.transpose()<<endl;
                        p = pi.cast<double>().cwiseProduct(node_scale_) + Eigen::Vector3d(0.5, 0.5, 0.5) * resolution_ + origin_;
                        cout<<"p:"<<p.transpose()<<endl;
                        cout<<"id:"<<id<<endl;

                        cout<<"f:"<<int(f)<<"  u:"<<int(u)<<endl;
                        cout<<"g->f:"<<int(g->local_grid_[id]->free_num_)<<"  g->u:"<<int(g->local_grid_[id]->unknown_num_)<<endl;
                        cout<<(f != g->local_grid_[id]->free_num_)<<(u != g->local_grid_[id]->unknown_num_)<<endl;
                        StopageDebug("CheckThorough f != g->local_grid_[id]->free_num_ || u != g->local_grid_[id]->unknown_num_");
                    }
                    int tt = int(g->local_grid_[id]->unknown_num_) + int(g->local_grid_[id]->free_num_);
                    if(g->local_grid_[id]->free_num_ > nv_num_ || g->local_grid_[id]->unknown_num_ > nv_num_ || tt > nv_num_){
                        cout<<"f:"<<int(g->local_grid_[id]->free_num_)<<endl;
                        cout<<"u:"<<int(g->local_grid_[id]->unknown_num_)<<endl;
                        StopageDebug("CheckThorough too much");
                    }
                    id++;
                }
            }
        }
    }
    return true;
}

void LowResMap::ShowGridLocal(const ros::TimerEvent& e){
    if(Showblocklist_.size() > 0 && marker_pub_.getNumSubscribers() > 0){
        visualization_msgs::MarkerArray MKArray;
        MKArray.markers.resize(Showblocklist_.size());
        //load makers
        int i = 0;
        for(list<int>::iterator idit = Showblocklist_.begin(); idit != Showblocklist_.end(); idit++){
            MKArray.markers[i].action = visualization_msgs::Marker::ADD;
            MKArray.markers[i].pose.orientation.w = 1.0;
            MKArray.markers[i].type = visualization_msgs::Marker::SPHERE_LIST;      //nodes
            MKArray.markers[i].scale.x = resolution_;
            MKArray.markers[i].scale.y = resolution_;
            MKArray.markers[i].scale.z = resolution_;
            MKArray.markers[i].header.frame_id = "world";
            MKArray.markers[i].header.stamp = ros::Time::now();
            MKArray.markers[i].id = (*idit);
            i++;
        }

        i = 0;
        Eigen::Vector3d pos;
        Eigen::Vector3i iterp;
        geometry_msgs::Point pt;
        int nodeid, bid;
        std_msgs::ColorRGBA localcolor, globalcolor, Xcolor, Ecolor;
        Ecolor.a = 1.0;
        Ecolor.g = 1.0;

        Xcolor.a = 1.0;
        Xcolor.r = 1.0;

        localcolor.a = 1.0;
        localcolor.b = 0.3;
        localcolor.g = 0.5;
        localcolor.r = 0.8;
        globalcolor.a = 1.0;
        globalcolor.b = 0.8;
        globalcolor.g = 0.5;
        globalcolor.r = 0.3;
        //publish nodes
        for(list<int>::iterator idit = Showblocklist_.begin(); idit != Showblocklist_.end(); idit++){

            bid = GlobalBid2LocalBid(*idit);
            if(bid == -1){
                MKArray.markers[i].color.a = 0.2;
                MKArray.markers[i].type = visualization_msgs::Marker::CUBE;
                MKArray.markers[i].action = visualization_msgs::Marker::DELETE;
            }
            else{
                LG_[bid]->show_flag_ = false;

                //load points in block
                for(iterp(0) = 0; iterp(0) < block_size_(0); iterp(0)++){
                    for(iterp(1) = 0; iterp(1) < block_size_(1); iterp(1)++){
                        for(iterp(2) = 0; iterp(2) < block_size_(2); iterp(2)++){
                            nodeid = iterp(2)*block_size_(0)*block_size_(1)+
                                iterp(1)*block_size_(0) + iterp(0);
                            
                            if(LG_[bid]->local_grid_[nodeid] != Outnode_){
                                pt.x = (iterp(0)+LG_[bid]->origin_(0)+0.5)*node_scale_(0)+origin_(0);
                                pt.y = (iterp(1)+LG_[bid]->origin_(1)+0.5)*node_scale_(1)+origin_(1);
                                pt.z = (iterp(2)+LG_[bid]->origin_(2)+0.5)*node_scale_(2)+origin_(2);
                                if(LG_[bid]->local_grid_[nodeid]->free_num_ == nv_num_){
                                    MKArray.markers[i].colors.push_back(localcolor);
                                    MKArray.markers[i].points.push_back(pt);
                                }
                                // else{
                                //     MKArray.markers[i].colors.push_back(globalcolor);
                                //     MKArray.markers[i].points.push_back(pt);
                                // }
                            }
                        }
                    }
                }
            }
            i++;
        }        

        marker_pub_.publish(MKArray);
    }
    Showblocklist_.clear();
}