#include <lowres_map_lite/lowres_map_lite.h>
using namespace std;
using namespace lowres_lite;

void LowResMap::ClearSearched(vector<shared_ptr<sch_node>> &node_list, bool clear_g){
    int bid, nid;
    for(vector<shared_ptr<sch_node>>::iterator node_it = node_list.begin(); node_it != node_list.end(); node_it++){
        GlobalPos2LocalBidNid((*node_it)->pos_, bid, nid);
        shared_ptr<LR_node> node = LG_[bid]->local_grid_[nid];
        if(node != Outnode_ && node != NULL){
            node->topo_sch_ = NULL;
            if(clear_g){
                node->path_g_ = 999999.0;
            }
        }
        else{//debug
            cout<<(*node_it)->pos_.transpose()<<endl;
            cout<<(*node_it)->g_score_<<endl;
            ROS_WARN("ERROR FreeWorker");
        }
    }
}

void LowResMap::ClearTopo(){
    for(auto &G : LG_){
        for(auto &n : G->local_grid_){
            if(n != Outnode_ && n != NULL){
                n->topo_sch_ = NULL;
                n->root_id_ = 0;
                n->parent_dir_ = 0;
                n->path_g_ = 999999.0;
                n->children_dir_ = 0;
            }
        }
    }
}

void LowResMap::GetPathInLocal(const double &ye_origin, const vector<Eigen::Vector3d> &path_origin, double &ye_local, vector<Eigen::Vector3d> &path_local){
    Eigen::Vector3d last_fea_p;
    int ni = -1;
    // for(int i = 0; i < path_origin.size(); i++){
    //     cout<<"path_origin:"<<path_origin[i].transpose()<<endl;
    // }
    for(int i = 0; i < path_origin.size(); i++){
        // cout<<"c:"<<path_origin[i].transpose()<<endl;
        if(Connected(path_origin[i])){
            last_fea_p = path_origin[i];
        }
        else {
            ni = i;
            break;
        }
    }
    // if(ni == -1){
    //     StopageDebug("GetPathInLocal ni == -1");
    // }
    ye_local = atan2(path_origin[ni](1) - last_fea_p(1), path_origin[ni](0) - last_fea_p(0));
    if((path_origin[ni].head(2) - last_fea_p.head(2)).norm() < 1e-3){
        ye_local = ye_origin;
    }
    Eigen::Vector3i last_fea_pi = GlobalPos2GlobalId3(last_fea_p);
    // cout<<"last_fea_p:"<<last_fea_p.transpose()<<endl;
    double l;
    RetrieveHPath(last_fea_pi, path_local, l, true);
}

void LowResMap::VoronoiPartitionSearch(list<pair<uint32_t, Eigen::Vector3d>> &search_origins, list<Eigen::Vector3i> &margins){
    vector<shared_ptr<sch_node>> node_list; //to maintain nodes 
    shared_ptr<sch_node> c_node, ep_node;
    shared_ptr<LR_node> lr_node;
    priority_queue<shared_ptr<sch_node>, vector<shared_ptr<sch_node>>, DCompare> open_set;
    Eigen::Vector3i std_origin, diff, p3i;
    Eigen::Vector3d pt;
    vector<Eigen::Vector3i> iter_list;
    int bid, nid, id;
    uint32_t ch_dirs;
    bool have_child;
    bool push_searched = false;
    // cout<<"VoronoiPartitionSearch-2"<<endl;

    for(auto &sr : search_origins){
        if(IsFeasible(sr.second)){
            PostoId3(sr.second, std_origin);
            if(!GlobalPos2LocalBidNid(std_origin, bid, nid)){
                // cout<<"std_origin:"<<std_origin.transpose()<<endl;
                // cout<<"sr.second:"<<sr.second.transpose()<<endl;
                // cout<<"local_origin_idx_:"<<local_origin_idx_.transpose()<<endl;
                // cout<<"local_up_idx_:"<<local_up_idx_.transpose()<<endl;
                // cout<<"local_map_lowbd_:"<<local_map_lowbd_.transpose()<<endl;
                // cout<<"local_map_upbd_:"<<local_map_upbd_.transpose()<<endl;

                // StopageDebug("VoronoiPartitionSearch, Impossible initiate");
            }
            LG_[bid]->local_grid_[nid]->topo_sch_ = make_shared<sch_node>();
            LG_[bid]->local_grid_[nid]->topo_sch_->pos_ = std_origin;
            LG_[bid]->local_grid_[nid]->topo_sch_->g_score_ = 0;
            LG_[bid]->local_grid_[nid]->topo_sch_->root_id_ = sr.first;
            LG_[bid]->local_grid_[nid]->root_id_ = sr.first;
            LG_[bid]->local_grid_[nid]->parent_dir_ = 13;
            LG_[bid]->local_grid_[nid]->path_g_ = 0.0;
            node_list.emplace_back(LG_[bid]->local_grid_[nid]->topo_sch_);
            open_set.push(LG_[bid]->local_grid_[nid]->topo_sch_);
            // cout<<"sr.first:"<<sr.first<<endl;

        }
        // else{
        //     cout<<"Not feasible:"<<endl;
        //     cout<<"search_origin:"<<sr.second.transpose()<<endl;
        //     // cout<<"std_origin:"<<std_origin.transpose()<<endl;
        //     cout<<"local_origin_idx_:"<<local_origin_idx_.transpose()<<endl;
        //     cout<<"local_up_idx_:"<<local_up_idx_.transpose()<<endl;
        //     cout<<"local_map_lowbd_:"<<local_map_lowbd_.transpose()<<endl;
        //     cout<<"local_map_upbd_:"<<local_map_upbd_.transpose()<<endl;
        // }
    }
    // cout<<"open_set:"<<open_set.size()<<endl;
    while(!open_set.empty()){
        c_node = open_set.top();
        open_set.pop();
        // cout<<"VoronoiPartitionSearch-1"<<endl;
        // ROS_WARN("VoronoiPartitionSearch-1");
        // if(c_node == NULL){
        //     StopageDebug("c_node == NULL");

        // }
        if(c_node->status_ == in_close) continue;
        c_node->status_ = in_close;
        // ROS_WARN("VoronoiPartitionSearch0");

        id = 0;
        ch_dirs = 4294967295;
        have_child = false;
        /* find feasible neighbours */
        for(diff(2) = -1; diff(2) < 2; diff(2)++){
            for(diff(1) = -1; diff(1) < 2; diff(1)++){
                for(diff(0) = -1; diff(0) < 2; diff(0)++){
                    // cout<<"VoronoiPartitionSearch0.1"<<endl;
                    // if(c_node == NULL){
                    //     StopageDebug("c_node == NULL");
                    // }
                    p3i = diff + c_node->pos_;
                    // pt = p3i.cast<double>().cwiseProduct(node_scale_) + origin_;
                    // cout<<"VoronoiPartitionSearch0.15"<<endl;
                    // cout<<"z:"<<p3i(2)*0.7 - 0.2<<endl;
                    // if(p3i(2)*0.7 - 0.2 > 8.0){
                    //     cout<<"z:"<<p3i(2)*0.7 - 0.2<<endl;
                    //     cout<<"p:"<<GlobalId2GlobalPos(p3i).transpose()<<endl;
                    //     cout<<"local_map_upbd_:"<<local_map_upbd_.transpose()<<endl;
                    //     cout<<"local_map_lowbd_:"<<local_map_lowbd_.transpose()<<endl;
                        
                    // }
                    if(!IsFeasible(p3i)){

                        // if(id >= occdiff_2_ch_dir_.size()){
                        //     cout<<"id:"<<id<<endl;
                        //     cout<<"occdiff_2_ch_dir_:"<<occdiff_2_ch_dir_.size()<<endl;
                        //     StopageDebug("VoronoiPartitionSearch, Impossible initiate1");
                        // }

                        ch_dirs &= occdiff_2_ch_dir_[id];
                    }
                    // cout<<"VoronoiPartitionSearch0.5"<<endl;
                    id++;
                }
            }
        }
        // ROS_WARN("VoronoiPartitionSearch1");
        // cout<<"VoronoiPartitionSearch1"<<endl;

        iter_list.clear();
        GetChildrenDirs(ch_dirs, iter_list);
        // ROS_WARN("VoronoiPartitionSearch2");
        // cout<<"VoronoiPartitionSearch2"<<endl;

        /* update neighbours' relationship */
        for(auto &d : iter_list){
            p3i = d + c_node->pos_;

            // ROS_WARN("VoronoiPartitionSearch2.1");
            if(!GlobalPos2LocalBidNid(p3i, bid, nid))
                StopageDebug("VoronoiPartitionSearch, Impossible initiate2");
            double g_tmp = c_node->g_score_ + d.cast<double>().cwiseProduct(node_scale_).norm() + 0.0001;// + GetDist(diff(0), diff(1), diff(2));
            // ROS_WARN("VoronoiPartitionSearch2.2");
            // cout<<"g_tmp:"<<g_tmp<<endl;
            // cout<<"LG_[bid]->local_grid_[nid]->path_g_:"<<LG_[bid]->local_grid_[nid]->path_g_<<endl;
            // if(bid < 0 || bid >= LG_.size()){
            //     cout<<"bid:"<<bid<<endl;
            //     cout<<"LG_.size():"<<LG_.size()<<endl;
            //     StopageDebug("VoronoiPartitionSearch, Impossible initiate3");
            // }
            // if(LG_[bid] == NULL){
            //     cout<<"LG_[bid] == NULL"<<endl;
            //     cout<<"bid:"<<bid<<endl;
            //     StopageDebug("VoronoiPartitionSearch, Impossible initiate3.1");
            // }
            // if(nid < 0 || nid >= LG_[bid]->local_grid_.size()){
            //     cout<<"nid:"<<nid<<endl;
            //     cout<<"bid:"<<bid<<endl;
            //     cout<<"d:"<<d.transpose()<<endl;
            //     cout<<"p3i:"<<p3i.transpose()<<endl;
            //     int x = (p3i(0) - local_origin_node_idx_(0)) /  local_block_num_(0);
            //     int y = (p3i(1) - local_origin_node_idx_(1)) /  local_block_num_(1);
            //     int z = (p3i(2) - local_origin_node_idx_(2)) /  local_block_num_(2);
            //     cout<<"x:"<<x<<" y:"<<y<<" z:"<<z<<endl;

            //     cout<<"c_node->pos_:"<<c_node->pos_.transpose()<<endl;
            //     cout<<"LG_[bid]->origin_:"<<LG_[bid]->origin_.transpose()<<endl;
            //     cout<<"local_origin_node_idx_:"<<local_origin_node_idx_.transpose()<<endl;
                
            //     cout<<"LG_[bid]->local_grid_.size():"<<LG_[bid]->local_grid_.size()<<endl;
            //     StopageDebug("VoronoiPartitionSearch, Impossible initiate4");
            // }

            // if(c_node->g_score_ < 1e-3){ 
            //     cout<<"d:"<<d.transpose()<<endl;
            //     cout<<"LG_[bid]->local_grid_[nid]->path_g_:"<<LG_[bid]->local_grid_[nid]->path_g_<<endl;
            //     cout<<"g_tmp:"<<g_tmp<<endl;
            //     cout<<"c_node->g_score_ :"<<c_node->g_score_ <<endl;
            //     cout<<(LG_[bid]->local_grid_[nid]->topo_sch_ == NULL)<<endl;
            // }

            if(LG_[bid]->local_grid_[nid]->path_g_ > g_tmp){                         //create a new node
                // ROS_WARN("VoronoiPartitionSearch2.21");
                push_searched = false;
                ep_node = LG_[bid]->local_grid_[nid]->topo_sch_;
                if(ep_node != NULL && ep_node->status_ == in_close) continue;
                if(ep_node != NULL) ep_node->status_ = in_close;
                else{
                    push_searched = true;
                }
                // ROS_WARN("VoronoiPartitionSearch2.22");
                
                ep_node = make_shared<sch_node>();
                LG_[bid]->local_grid_[nid]->topo_sch_ = ep_node;
                ep_node->pos_ = c_node->pos_ + d;
                ep_node->g_score_ = c_node->g_score_ + d.cast<double>().cwiseProduct(node_scale_).norm();// + LRM_->GetDist(diff(0), diff(1), diff(2));
                ep_node->parent_ = c_node;
                ep_node->status_ = in_open;
                ep_node->root_id_ = c_node->root_id_;
                LG_[bid]->local_grid_[nid]->path_g_ = ep_node->g_score_;
                LG_[bid]->local_grid_[nid]->root_id_ = c_node->root_id_;
                LG_[bid]->local_grid_[nid]->parent_dir_ = -d(0) - d(1) * 3 - d(2) * 9 + 13;
                // if((connect_diff_[LG_[bid]->local_grid_[nid]->parent_dir_] + d).norm() != 0){
                //     cout<<"(connect_diff_[LG_[bid]->local_grid_[nid]->parent_dir_]:"<<connect_diff_[LG_[bid]->local_grid_[nid]->parent_dir_].transpose()<<endl;
                //     cout<<"d:"<<d.transpose()<<endl;
                //     cout<<"-d(0) - d(1) * 3 - d(2) * 9 + 13:"<<-d(0) - d(1) * 3 - d(2) * 9 + 13<<endl;
                //     StopageDebug("...");
                // }
                open_set.push(ep_node);
                have_child = true;
                if(push_searched) node_list.emplace_back(ep_node);

                // cout<<"add pt root:"<<int(ep_node->root_id_)<<endl;

            }
            // ROS_WARN("VoronoiPartitionSearch2.3");
        }
        // ROS_WARN("VoronoiPartitionSearch3");
        // cout<<"VoronoiPartitionSearch3"<<endl;

        /* add margins */
        if(!have_child){
            margins.emplace_back(c_node->pos_);
            // cout<<"margins root:"<<int(c_node->root_id_)<<endl;
            // auto n1 = GlobalPos2LocalNode(c_node->pos_);
            // if(n1 == NULL || n1 == Outnode_){
            //     StopageDebug("VoronoiPartitionSearch, n1 == NULL || n1 == Outnode_");
            // }
            // if(n1->topo_sch_ != c_node){            
            //     cout<<"n1 root:"<<int(n1->root_id_)<<endl;
            //     cout<<"bool:"<<(n1->topo_sch_ == c_node)<<endl;
            //     cout<<"bool2:"<<(n1 == Outnode_)<<endl;
            //     cout<<"c_node->pos_:"<<c_node->pos_.transpose()<<endl;
            //     StopageDebug("VoronoiPartitionSearch, Impossible initiate5");
            // }

        }
        // cout<<"VoronoiPartitionSearch4"<<endl;

        // ROS_WARN("VoronoiPartitionSearch4");

    }
    // cout<<"VoronoiPartitionSearch, node_list size:"<<node_list.size()<<endl;
    
    // for(auto &n : node_list){
    //     cout<<"n g_score_:"<<n->g_score_<<endl;
    //     cout<<"n root_id_:"<<n->root_id_<<endl;
        
    // }
    ClearSearched(node_list);
}


void LowResMap::VoronoiPathRetrieve(tr1::unordered_map<uint32_t, uint32_t> &idx_dict, 
    list<Eigen::Vector3i> &margins, list<pair<uint32_t, uint32_t>> &hh_idx, list<list<Eigen::Vector3d>> &paths, list<double> &lengths){
    tr1::unordered_map<uint32_t, uint32_t>::iterator idx1, idx2;
    uint32_t ch_dirs;
    Eigen::Vector3i diff, p3i;
    vector<Eigen::Vector3i> iter_list;
    list<Eigen::Vector3d> path1, path2;
    vector<vector<pair<Eigen::Vector3i, Eigen::Vector3i>>> path_connections;
    vector<uint32_t> h_ids;
    int dn;
    int id;
    double dist, dist1, dist2;
    dn = idx_dict.size();
    path_connections.resize(dn);
    for(auto &pc : path_connections) pc.resize(dn);
    h_ids.resize(dn);
    for(auto &hi : idx_dict) h_ids[hi.second] = hi.first;

    Eigen::MatrixXd D;
    D.resize(dn, dn);
    D.setOnes();
    D *= 999999.9;
    // ROS_WARN("VoronoiPathRetrieve0");
    for(auto &p : margins){
        // ROS_WARN("VoronoiPathRetrieve1");

        auto n1 = GlobalPos2LocalNode(p);

        // if(n1 == NULL || n1 == Outnode_){
        //     StopageDebug("VoronoiPathRetrieve, n1 == NULL || n1 == Outnode_");
        // }

        idx1 = idx_dict.find(n1->root_id_);
        // if(idx1 == idx_dict.end()) {
        //     cout<<"n1->root_id_:"<<int(n1->root_id_)<<endl;
        //     cout<<"p:"<<p.transpose()<<endl;
        //     StopageDebug("VoronoiPathRetrieve idx_dict end");
        // }

        /* find feasible neighbours */
        ch_dirs = 4294967295;
        id = 0;
        // ROS_WARN("VoronoiPathRetrieve2");
        
        // list<Eigen::Vector3i> debug_list;
        for(diff(2) = -1; diff(2) < 2; diff(2)++){
            for(diff(1) = -1; diff(1) < 2; diff(1)++){
                for(diff(0) = -1; diff(0) < 2; diff(0)++){


                    p3i = diff + p;
                    if(!IsFeasible(p3i)){
                        ch_dirs &= occdiff_2_ch_dir_[id];
                    }

                    // if(diff.norm() == 1){
                    //     if(IsFeasible(p3i)){
                    //         debug_list.emplace_back(diff);
                    //     }
                    // }
                    id++;
                }
            }
        }
        GetChildrenDirs(ch_dirs, iter_list);
        // ROS_WARN("VoronoiPathRetrieve3");
        // if(debug_list.size() != iter_list.size()){
        //     StopageDebug("VoronoiPathRetrieve debug_list.size() != iter_list.size()");
        // }
        // int debug_c = 0;

        for(auto &mn : iter_list){
            // for(auto &d :debug_list){// debug
            //     if((d - mn).norm() == 0){
            //         debug_c++;
            //         break;
            //     }
            // }

            p3i = mn + p;
            auto n2 = GlobalPos2LocalNode(p3i);
            // if(n2 == Outnode_) {
            //     cout<<"p:"<<p.transpose()<<endl;
            //     cout<<"mn:"<<mn.transpose()<<endl;
            //     cout<<"local_origin_node_idx_:"<<local_origin_node_idx_.transpose()<<endl;
            //     cout<<"local_up_node_idx_:"<<local_up_node_idx_.transpose()<<endl;
            //     cout<<"p3i f:"<<IsFeasible(p3i)<<endl;
            //     cout<<"p3i f:"<<InsideLocalMap(p3i)<<endl;
                
            //     StopageDebug("VoronoiPathRetrieve n2 == Outnode_");
            // }
            // if(n1 == Outnode_) {
            //     StopageDebug("VoronoiPathRetrieve n1 == Outnode_");
            // }
            // if(n1 == n2) {
            //     StopageDebug("VoronoiPathRetrieve n1 == n2");
            // }
            if(n2->root_id_ == n1->root_id_ || n2->root_id_ == 0) continue; // same root
            idx2 = idx_dict.find(n2->root_id_);
            // if(idx2 == idx_dict.end()) {
            //     cout<<"n2->root_id_:"<<int(n2->root_id_)<<endl;
            //     StopageDebug("VoronoiPathRetrieve idx_dict end2");
            // }
            dist = n2->path_g_ + n1->path_g_ + mn.cast<double>().cwiseProduct(node_scale_).norm();
            // cout<<"dist:"<<dist<<endl;
            if(dist + 1e-3 < D(idx1->second, idx2->second)){
                D(idx1->second, idx2->second) = dist;
                D(idx2->second, idx1->second) = dist;
                path_connections[idx1->second][idx2->second] = {p, p3i};
                path_connections[idx2->second][idx1->second] = {p3i, p};
            }
        }
        // if(debug_c != iter_list.size()){
        //     StopageDebug("VoronoiPathRetrieve debug_c != iter_list.size()");
        // }
        // cout<<"iter_list:"<<iter_list.size()<<" debug_list:"<<debug_list.size()<<" debug_c:"<<debug_c<<endl;

        // ROS_WARN("VoronoiPathRetrieve4");
    }
    // ROS_WARN("VoronoiPathRetrieve5");
    // cout<<"D:\n"<<D<<endl;
    // if(dn >= 2) StopageDebug("D.size() >= 2");
    for(int i = 0; i < dn; i++){
        for(int j = i + 1; j < dn; j++){
            // ROS_WARN("VoronoiPathRetrieve5.1");
            // if(i >= D.rows() || j >= D.cols()){
            //     StopageDebug("i >= D.rows() || j >= D.cols()");
            // }

            // retireve if have path 
            if(D(i, j) < 999999.8){ 
                // if(i >= path_connections.size() || j >= path_connections[i].size()){
                //     StopageDebug("i >= path_connections.size() || j >= path_connections[i].size()");
                // }
                // // ROS_WARN("VoronoiPathRetrieve5.2");

                if(!RetrieveHPath(path_connections[i][j].first, path1, dist, true)){
                    StopageDebug("VoronoiPathRetrieve, no path1");
                }
                // // ROS_WARN("VoronoiPathRetrieve5.25");
                if(!RetrieveHPath(path_connections[i][j].second, path2, dist, false)){
                    StopageDebug("VoronoiPathRetrieve, no path2");
                }
                path1.insert(path1.end(), path2.begin(), path2.end());
                hh_idx.emplace_back(i, j);
                // hh_idx.emplace_back(idx1->second, idx2->second);
                paths.emplace_back(path1);
                lengths.emplace_back(D(i, j));
            }
            // ROS_WARN("VoronoiPathRetrieve5.3");

        }
    }
    // ROS_WARN("VoronoiPathRetrieve6");

}

void LowResMap::GetDists(const Eigen::Vector3d &origin, vector<Eigen::Vector3d> &tars, vector<double> &dist, vector<vector<Eigen::Vector3d>> &paths, bool clear_searched){
    dist.clear();
    dist.resize(tars.size(), 999999.0);
    paths.resize(tars.size());
    vector<shared_ptr<sch_node>> node_list; //to maintain nodes 
    shared_ptr<sch_node> c_node, ep_node;
    shared_ptr<LR_node> lr_node;
    priority_queue<shared_ptr<sch_node>, vector<shared_ptr<sch_node>>, DCompare> open_set;
    Eigen::Vector3i std_origin, diff, p3i;
    Eigen::Vector3d pt;
    vector<Eigen::Vector3i> iter_list;
    int bid, nid, id;
    uint32_t ch_dirs;
    bool have_child;
    tr1::unordered_map<int, int> tar_dict;
    int ans_num = 0;
    // int searched = 0;
    int c_idx;
    bool push_searched;

    for(int i = 0; i < tars.size(); i++){
        auto p = tars[i];
        if(IsFeasible(p)){
            int idx = GlobalPos2GlobalNodeId(p);
            // cout<<"p:"<<p.transpose()<<endl;
            tar_dict.insert({idx, i});
        }
    }

    if(tar_dict.size() == 0){
        ROS_WARN("tar empty");
        return;
    }
    if(!IsFeasible(origin)){
        cout<<"origin:"<<origin.transpose()<<endl;
        ROS_WARN("!IsFeasible(origin)");
        return;
    }
    else{
        PostoId3(origin, std_origin);
        if(GlobalPos2LocalBidNid(std_origin, bid, nid)){
            LG_[bid]->local_grid_[nid]->topo_sch_ = make_shared<sch_node>();
            LG_[bid]->local_grid_[nid]->topo_sch_->pos_ = std_origin;
            LG_[bid]->local_grid_[nid]->topo_sch_->g_score_ = 0;
            LG_[bid]->local_grid_[nid]->path_g_ = 0.0;
            LG_[bid]->local_grid_[nid]->parent_dir_ = 13;
            open_set.push(LG_[bid]->local_grid_[nid]->topo_sch_);
            node_list.emplace_back(LG_[bid]->local_grid_[nid]->topo_sch_);

        }
    }
    while(!open_set.empty()){
        c_node = open_set.top();
        open_set.pop();
        if(c_node->status_ == in_close) continue;
        c_node->status_ = in_close;
        // searched++;

        c_idx = GlobalPos2GlobalNodeId(c_node->pos_);
        // cout<<"c_node->pos_:"<<(c_node->pos_.cast<double>().cwiseProduct(node_scale_) + origin_ + node_scale_ * 0.5).transpose()<<endl;
        if(tar_dict.find(c_idx) != tar_dict.end()){
            // ROS_WARN("found target");
            // cout<<tar_dict[c_idx]<<endl;
            // cout<<c_node->g_score_<<endl;
            dist[tar_dict[c_idx]] = c_node->g_score_;
            double dist;
            RetrieveHPath(c_node->pos_, paths[tar_dict[c_idx]], dist, true);
            // ans_num++;
            // if(ans_num == tar_dict.size()){
            //     break;
            // }
        }

        id = 0;
        ch_dirs = 4294967295;
        /* find feasible neighbours */
        for(diff(2) = -1; diff(2) < 2; diff(2)++){
            for(diff(1) = -1; diff(1) < 2; diff(1)++){
                for(diff(0) = -1; diff(0) < 2; diff(0)++){
                    p3i = diff + c_node->pos_;
                    if(!IsFeasible(p3i)){
                        ch_dirs &= occdiff_2_ch_dir_[id];
                    }
                    id++;
                }
            }
        }

        iter_list.clear();
        GetChildrenDirs(ch_dirs, iter_list);

        /* update neighbours' relationship */
        for(auto &d : iter_list){
            p3i = d + c_node->pos_;
            if(!GlobalPos2LocalBidNid(p3i, bid, nid))
                StopageDebug("VoronoiPartitionSearch, Impossible initiate2");
            double g_tmp = c_node->g_score_ + d.cast<double>().cwiseProduct(node_scale_).norm() + 0.0001;// + GetDist(diff(0), diff(1), diff(2));

            if(LG_[bid]->local_grid_[nid]->path_g_ > g_tmp){                         //create a new node
                ep_node = LG_[bid]->local_grid_[nid]->topo_sch_;
                push_searched = false;
                if(ep_node != NULL && ep_node->status_ == in_close) continue;
                if(ep_node != NULL) ep_node->status_ = in_close;
                else{
                    push_searched = true;
                }
                
                ep_node = make_shared<sch_node>();
                LG_[bid]->local_grid_[nid]->topo_sch_ = ep_node;
                ep_node->pos_ = p3i;
                ep_node->g_score_ = c_node->g_score_ + d.cast<double>().cwiseProduct(node_scale_).norm();// + LRM_->GetDist(diff(0), diff(1), diff(2));
                ep_node->parent_ = c_node;
                ep_node->status_ = in_open;
                LG_[bid]->local_grid_[nid]->path_g_ = ep_node->g_score_;
                LG_[bid]->local_grid_[nid]->parent_dir_ = -d(0) - d(1) * 3 - d(2) * 9 + 13;

                open_set.push(ep_node);
                if(push_searched) node_list.emplace_back(ep_node);

            }
        }

    }
    if(clear_searched) ClearSearched(node_list, true);
}

bool LowResMap::GetPath(const Eigen::Vector3d &start, const Eigen::Vector3d &end, vector<Eigen::Vector3d> &path,  double &dist){
    priority_queue<shared_ptr<sch_node>, vector<shared_ptr<sch_node>>, ACompare> open_set;
    vector<shared_ptr<sch_node>> node_list; //to maintain nodes 
    shared_ptr<sch_node> c_node, ep_node;
    shared_ptr<LR_node> lr_node;
    int bid, nid, id;
    Eigen::Vector3i std_start, diff, p3i;
    Eigen::Vector3d pt;
    vector<Eigen::Vector3i> iter_list;
    int c_idx, tar_idx;
    tar_idx = GlobalPos2GlobalNodeId(end);
    uint32_t ch_dirs;
    bool push_searched;


    if(!IsFeasible(start)){
        cout<<"start:"<<start.transpose()<<endl;
        ROS_WARN("!IsFeasible(origin)");
        return false;
    }
    else{
        PostoId3(start, std_start);
        if(GlobalPos2LocalBidNid(std_start, bid, nid)){
            LG_[bid]->local_grid_[nid]->topo_sch_ = make_shared<sch_node>();
            LG_[bid]->local_grid_[nid]->topo_sch_->pos_ = std_start;
            LG_[bid]->local_grid_[nid]->topo_sch_->f_score_ = (start - end).norm();
            LG_[bid]->local_grid_[nid]->topo_sch_->g_score_ = 0.0;
            LG_[bid]->local_grid_[nid]->path_g_ = 0.0;
            LG_[bid]->local_grid_[nid]->parent_dir_ = 13;
            open_set.push(LG_[bid]->local_grid_[nid]->topo_sch_);
            node_list.emplace_back(LG_[bid]->local_grid_[nid]->topo_sch_);
        }
    }
    while(!open_set.empty()){
        c_node = open_set.top();
        open_set.pop();
        if(c_node->status_ == in_close) continue;
        c_node->status_ = in_close;
        // searched++;

        c_idx = GlobalPos2GlobalNodeId(c_node->pos_);
        // cout<<"c_idx:"<<c_idx<<endl;
        // cout<<"c_node->pos_:"<<(c_node->pos_.cast<double>().cwiseProduct(node_scale_) + origin_ + node_scale_ * 0.5).transpose()<<endl;
        if(c_idx == tar_idx){
            // double dist;
            RetrieveHPath(c_node->pos_, path, dist, true);
            ClearSearched(node_list, true);
            return true;
            // ans_num++;
            // if(ans_num == tar_dict.size()){
            //     break;
            // }
        }

        id = 0;
        ch_dirs = 4294967295;
        /* find feasible neighbours */
        for(diff(2) = -1; diff(2) < 2; diff(2)++){
            for(diff(1) = -1; diff(1) < 2; diff(1)++){
                for(diff(0) = -1; diff(0) < 2; diff(0)++){
                    p3i = diff + c_node->pos_;
                    if(!IsFeasible(p3i)){
                        ch_dirs &= occdiff_2_ch_dir_[id];
                    }
                    id++;
                }
            }
        }

        iter_list.clear();
        GetChildrenDirs(ch_dirs, iter_list);

        /* update neighbours' relationship */
        for(auto &d : iter_list){
            p3i = d + c_node->pos_;
            if(!GlobalPos2LocalBidNid(p3i, bid, nid))
                StopageDebug("VoronoiPartitionSearch, Impossible initiate2");

            pt = GlobalId2GlobalPos(p3i);
            double f_tmp = c_node->g_score_ + d.cast<double>().cwiseProduct(node_scale_).norm() + 0.0001 + (pt - end).norm();// + GetDist(diff(0), diff(1), diff(2));

            if(LG_[bid]->local_grid_[nid]->path_g_ > f_tmp){                         //create a new node
                ep_node = LG_[bid]->local_grid_[nid]->topo_sch_;
                push_searched = false;
                if(ep_node != NULL && ep_node->status_ == in_close) continue;
                if(ep_node != NULL) ep_node->status_ = in_close;
                else{
                    push_searched = true;
                }
                
                ep_node = make_shared<sch_node>();
                LG_[bid]->local_grid_[nid]->topo_sch_ = ep_node;
                ep_node->pos_ = p3i;
                ep_node->g_score_ = c_node->g_score_ + d.cast<double>().cwiseProduct(node_scale_).norm();// + LRM_->GetDist(diff(0), diff(1), diff(2));
                ep_node->f_score_ = f_tmp - 0.0001;
                ep_node->parent_ = c_node;
                ep_node->status_ = in_open;
                LG_[bid]->local_grid_[nid]->path_g_ = f_tmp - 0.0001;
                LG_[bid]->local_grid_[nid]->parent_dir_ = -d(0) - d(1) * 3 - d(2) * 9 + 13;

                open_set.push(ep_node);
                if(push_searched) node_list.emplace_back(ep_node);

            }
        }

    }
    ClearSearched(node_list, true);
    return false;
}