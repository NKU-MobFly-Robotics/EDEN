#include <mr_dtg_plus/mr_dtg_plus.h>
using namespace DTGPlus;

void MultiDtgPlus::RetrieveHFPath(f_ptr &tar_f, list<h_ptr> &h_path, list<Eigen::Vector3d> &path, double &length){
    path.clear();
    h_path.clear();
    length = 0;
    h_ptr cur_h = tar_f->hf_edge_->head_n_;
    h_ptr par_h;
    path.insert(path.end(), tar_f->hf_edge_->path_.begin(), tar_f->hf_edge_->path_.end());
    while(cur_h->sch_node_->parent_ != NULL){
        if(!FindHnode(cur_h->sch_node_->parent_->pos_, cur_h->sch_node_->parent_->id_, par_h)){
            ROS_ERROR("error get hnode RetrieveHFPath");
            ros::shutdown();
            return;
        }
        for(auto &hhe : cur_h->hh_edges_){
            if(hhe->head_n_ == par_h){
                for (auto pit = hhe->path_.rbegin(); pit != hhe->path_.rend(); ++pit) {
                    path.emplace_front(*pit);
                }
                break;
            }
            else if(hhe->tail_n_ == par_h){
                for (auto pit = hhe->path_.begin(); pit != hhe->path_.end(); ++pit) {
                    path.emplace_front(*pit);
                }
                break;
            }
        }
        h_path.emplace_front(cur_h);
        cur_h = par_h;
    }
    h_path.emplace_front(cur_h);
}

void MultiDtgPlus::RetrieveHPath(h_ptr &start_h, h_ptr &tar_h, vector<Eigen::Vector3d> &path, double &length){
    path.clear();
    length = tar_h->sch_node_->g_;
    h_ptr cur_h = tar_h;
    h_ptr par_h;
    while(cur_h->sch_node_->parent_ != NULL){
        if(!FindHnode(cur_h->sch_node_->parent_->pos_, cur_h->sch_node_->parent_->id_, par_h)){
            if(start_h->id_ != cur_h->sch_node_->parent_->id_){
                ROS_ERROR("error get hnode RetrieveHPath");
                cout<<"start_h->id_:"<<start_h->id_<<"  cur_h->sch_node_->parent_->id_:"<<cur_h->sch_node_->parent_->id_<<endl;
                ros::shutdown();
                return;
            }
            else{
                par_h = start_h;
            }
        }
        bool debug_flag = true;
        for(auto &hhe : par_h->hh_edges_){
            if(hhe->tail_n_ == cur_h){
                for (auto pit = hhe->path_.rbegin(); pit != hhe->path_.rend(); ++pit) {
                    path.emplace_back(*pit);
                }
                debug_flag = false;
                break;
            }
            else if(hhe->head_n_ == cur_h){
                for (auto pit = hhe->path_.begin(); pit != hhe->path_.end(); ++pit) {
                    path.emplace_back(*pit);
                }
                debug_flag = false;
                break;
            }
        }
        if(debug_flag){
            ROS_ERROR("kidding??");
        }
        cur_h = par_h;
    }
    reverse(path.begin(), path.end());
}

void MultiDtgPlus::RetrieveHPath(h_ptr &start_h, h_ptr &tar_h, list<Eigen::Vector3d> &path, double &length){
    path.clear();
    length = tar_h->sch_node_->g_;
    h_ptr cur_h = tar_h;
    h_ptr par_h;
    while(cur_h->sch_node_->parent_ != NULL){
        if(!FindHnode(cur_h->sch_node_->parent_->pos_, cur_h->sch_node_->parent_->id_, par_h)){
            if(start_h->id_ != cur_h->sch_node_->parent_->id_){
                ROS_ERROR("error get hnode RetrieveHPath");
                ros::shutdown();
                return;
            }
            else{
                par_h = start_h;
            }
        }
        bool debug_flag = true;
        for(auto &hhe : par_h->hh_edges_){
            if(hhe->tail_n_ == cur_h){
                for (auto pit = hhe->path_.rbegin(); pit != hhe->path_.rend(); ++pit) {
                    path.emplace_front(*pit);
                }
                debug_flag = false;
                break;
            }
            else if(hhe->head_n_ == cur_h){
                for (auto pit = hhe->path_.begin(); pit != hhe->path_.end(); ++pit) {
                    path.emplace_front(*pit);
                }
                debug_flag = false;
                break;
            }
        }
        if(debug_flag){
            ROS_ERROR("kidding??");
        }
        cur_h = par_h;
    }
}

// void MultiDtgPlus::RetrieveHPathDebug(h_ptr &start_h, h_ptr &tar_h, list<Eigen::Vector3d> &path, double &length){
//     path.clear();
//     length = tar_h->sch_node_->g_;
//     h_ptr cur_h = tar_h;
//     h_ptr par_h;
//     while(cur_h->sch_node_->parent_ != NULL){
//         if(!FindHnode(cur_h->sch_node_->parent_->pos_, cur_h->sch_node_->parent_->id_, par_h)){
//             if(start_h->id_ != cur_h->sch_node_->parent_->id_){
//                 ROS_ERROR("error get hnode RetrieveHPath");
//                 ros::shutdown();
//                 return;
//             }
//             else{
//                 par_h = start_h;
//             }
//         }
//         bool debug_flag = true;
//         for(auto &hhe : par_h->hh_edges_){
//             if(hhe->tail_n_ == cur_h){
//                 for (auto pit = hhe->path_.rbegin(); pit != hhe->path_.rend(); ++pit) {
//                     path.emplace_front(*pit);
//                 }
//                 debug_flag = false;
//                 break;
//             }
//             else if(hhe->head_n_ == cur_h){
//                 for (auto pit = hhe->path_.begin(); pit != hhe->path_.end(); ++pit) {
//                     path.emplace_front(*pit);
//                 }
//                 debug_flag = false;
//                 break;
//             }
//         }
//         if(debug_flag){
//             ROS_ERROR("kidding??");
//         }
//         cur_h = par_h;
//     }
// }

void MultiDtgPlus::RetrieveHnPath(h_ptr &tar_h, list<h_ptr> &path){
    path.clear();
    h_ptr cur_h = tar_h;
    h_ptr par_h;
    while(cur_h->sch_node_->parent_ != NULL){
        if(!FindHnode(cur_h->sch_node_->parent_->pos_, cur_h->sch_node_->parent_->id_, par_h)){
            ROS_ERROR("error get hnode RetrieveHnPath");
            ros::shutdown();
            return;
        }
        path.emplace_front(cur_h);
        cur_h = par_h;
    }
}

void MultiDtgPlus::RetrieveHDebug(h_ptr &start_h, h_ptr &tar_h, list<h_ptr> &h_path){
    h_path.clear();
    h_ptr cur_h = tar_h;
    h_ptr par_h;
    while(cur_h->sch_node_->parent_ != NULL){
        if(!FindHnode(cur_h->sch_node_->parent_->pos_, cur_h->sch_node_->parent_->id_, par_h)){
            if(start_h->id_ != cur_h->sch_node_->parent_->id_){
                ROS_ERROR("error get hnode RetrieveHPath");
                ros::shutdown();
                return;
            }
            else{
                par_h = start_h;
            }
        }
        h_path.emplace_front(cur_h);
        cout<<"cur_h->id_:"<<int(cur_h->id_)<<endl;
        cur_h = par_h;
    }
}

void MultiDtgPlus::ClearSearched(list<h_ptr> &h_l){
    for(auto &h : h_l) h->sch_node_ = NULL;
}

void MultiDtgPlus::ClearSearched(list<f_ptr> &f_l){
    for(auto &f : f_l) f->sch_node_ = NULL;
}

bool MultiDtgPlus::GetClosestGlobalTarget(list<Eigen::Vector3d> &path, list<h_ptr> &H_path, int &f_id, int &v_id, double &length){
    if(root_ == NULL) return false;
    Eigen::Vector3d vp_pos, cvp, vp_best;
    double length_e, dmin, d;
    f_ptr tar_f;
    bool find_f;

    list<h_ptr> searched_h;
    list<f_ptr> searched_f;
    h_ptr hc, hn;
    f_ptr fn;

    prio_D empty_set;
    open_D_.swap(empty_set);
    shared_ptr<DTG_sch_node> cur_n, nei_n;
    searched_h.emplace_back(root_);
    root_->sch_node_ = make_shared<DTG_sch_node>(0.0, 0.0, root_->id_, root_->pos_);
    cur_n = root_->sch_node_;
    cur_n->flag_ = 1;
    open_D_.push(cur_n);
    while(!open_D_.empty()){
        cur_n = open_D_.top();

        open_D_.pop();
        if(cur_n->flag_ & 2) continue;
        cur_n->flag_ |= 2;
        if((cur_n->flag_ & 1) && !FindHnode(cur_n->pos_, cur_n->id_, hc)){
            ROS_ERROR("error get hnode GetClosestGlobalTarget");
            ros::shutdown();
            return false;
        }

        /* find h with f edges, success */
        if(!(cur_n->flag_ & 1)){
            find_f = false;
            dmin = 999999999.0;
            for(auto &f : F_depot_[cur_n->id_]){
                if(f->vid_ == cur_n->vid_ && cur_n->id_ == f->fid_){
                    EROI_->GetVpPos(f->fid_, f->vid_, cvp, false);
                    tar_f = f;
                    f_id = f->fid_;
                    v_id = f->vid_;
                    if(!EROI_->GetVpPos(f->fid_, f->vid_, vp_pos)){
                        vp_best = vp_pos;
                    }
                    // for(auto &c : f->vps_){
                    //     if(!EROI_->GetVpPos(f->fid_, c, vp_pos)){
                    //         d = (vp_pos - cvp).norm();
                    //         if(d < dmin){
                    //             vp_best = vp_pos;
                    //             d = dmin;
                    //             v_id = c;
                    //         }
                    //     }
                    //     else{
                    //         StopageDebug("GetClosestGlobalTarget, cant find vp pos");
                    //     }
                    // }
                    find_f = true;
                    break;
                }
            }
            // if(!find_f)
            //     StopageDebug("GetClosestGlobalTarget, find_f == false");

            RetrieveHFPath(tar_f, H_path, path, length_e);
            path.emplace_back(vp_best);
            ClearSearched(searched_h);
            ClearSearched(searched_f);
            return true;
        }

        /* expand Hneighbours */
        for(auto &hhe : hc->hh_edges_){
            if(hhe->head_n_ == hc)
                hn = hhe->tail_n_;
            else
                hn = hhe->head_n_;
            if(hn->sch_node_ == NULL){
                /* new node */
                hn->sch_node_ = make_shared<DTG_sch_node>(cur_n->g_ + hhe->length_, 0.0, hn->id_, hn->pos_);
                nei_n = hn->sch_node_;
                nei_n->parent_ = cur_n;
                nei_n->flag_ = 1;
                searched_h.push_back(hn);
                open_D_.push(nei_n);
            }
            else{
                /* in close list */
                if(hn->sch_node_->flag_ & 2) continue;

                /* try to change parent */
                nei_n = hn->sch_node_;
                if(cur_n->g_ + hhe->length_ + 1e-3 < nei_n->g_){
                    nei_n->flag_ |= 2;
                    hn->sch_node_ = make_shared<DTG_sch_node>(cur_n->g_ + hhe->length_, 0.0, hn->id_, hn->pos_);
                    hn->sch_node_->parent_ = cur_n;
                    hn->sch_node_->flag_ = 1;
                    open_D_.push(hn->sch_node_);
                }
            }
        }

        /* expand Fneighbours */
        for(auto &hfe : hc->hf_edges_){
            fn = hfe->tail_n_;            

            if(fn->sch_node_ == NULL){
                /* new node */
                fn->sch_node_ = make_shared<DTG_sch_node>(cur_n->g_ + hfe->length_, 0.0, fn->fid_, EROI_->EROI_[fn->fid_].center_);
                nei_n = fn->sch_node_;
                nei_n->parent_ = cur_n;
                nei_n->id_ = fn->fid_;
                nei_n->g_ = cur_n->g_ + hfe->length_;
                nei_n->flag_ = 0;
                searched_f.push_back(fn);
                open_D_.push(nei_n);
            }
            else{
                /* in close list */
                if(fn->sch_node_->flag_ & 2) continue;
                // ROS_WARN("id:%d how double f GetClosestGlobalTarget", SDM_->self_id_);
                /* try to change parent */
                nei_n = fn->sch_node_;
                if(cur_n->g_ + hfe->length_ + 1e-3 < nei_n->g_){
                    nei_n->parent_ = cur_n;
                    nei_n->g_ = cur_n->g_ + hfe->length_;
                }
            }
        }

    }
    ClearSearched(searched_h);
    ClearSearched(searched_f);
    return false;
}

// void MultiDtgPlus::GetGlobalTarget(h_ptr &s_hn, vector<pair<double, list<Eigen::Vector3d>>> &paths, vector<h_ptr> &t_hn/*,  vector<list<h_ptr>> &debug_paths*/){
//     h_ptr hc, hn;
//     f_ptr fn;
//     list<h_ptr> searched_h;
//     list<f_ptr> searched_f;
//     int reach_count = 0;
//     // debug_paths.resize(t_hn.size());
//     paths.resize(t_hn.size());
//     for(auto &p : paths) p.first = -1.0;

//     prio_D empty_set;
//     open_D_.swap(empty_set);
//     shared_ptr<DTG_sch_node> cur_n, nei_n;
//     searched_h.emplace_back(s_hn);
//     s_hn->sch_node_ = make_shared<DTG_sch_node>(0.0, 0.0, s_hn->id_, s_hn->pos_);
//     cur_n = s_hn->sch_node_;
//     cur_n->flag_ = 1;
//     open_D_.push(cur_n);

//     while(!open_D_.empty()){
//         cur_n = open_D_.top();
//         open_D_.pop();
//         if(cur_n->flag_ & 2) continue;
//         cur_n->flag_ |= 2;

//         if(cur_n->id_ > SDM_->drone_num_ && (cur_n->flag_ & 1) && !FindHnode(cur_n->pos_, cur_n->id_, hc)){
//             ROS_ERROR("error get hnode GetGlobalTarget");
//             cout<<"cur_n->id_:"<<int(cur_n->id_)<<endl;
//             ros::shutdown();
//             return;
//         }
//         else if(cur_n->id_ == s_hn->id_){
//             hc = s_hn;
//         }

//         /* reach target? */
//         for(int i = 0; i < t_hn.size(); i++){
//             if(t_hn[i]->id_ == hc->id_){
//                 reach_count++;
//                 /* retrieve path */
//                 RetrieveHPath(s_hn, hc, paths[i].second, paths[i].first);
//                 // RetrieveHDebug(s_hn, hc, debug_paths[i]);
//                 if(reach_count == t_hn.size()){
//                     ClearSearched(searched_h);
//                     return;
//                 }
//                 else break;
//             }
//         }

//         /* expand Hneighbours */
//         for(auto &hhe : hc->hh_edges_){
//             // if(!(hhe->flag_ & 4)) continue;

//             if(hhe->head_n_ == hc)
//                 hn = hhe->tail_n_;
//             else
//                 hn = hhe->head_n_;

//             double edge_length = hhe->length_s_;
//             if(hhe->e_flag_ & 16)
//                 edge_length = hhe->length_;
//             if(hn->sch_node_ == NULL){
//                 /* new node */
//                 hn->sch_node_ = make_shared<DTG_sch_node>(cur_n->g_ + edge_length, 0.0, hn->id_, hn->pos_);
//                 nei_n = hn->sch_node_;
//                 nei_n->parent_ = cur_n;
//                 nei_n->flag_ = 1;
//                 searched_h.push_back(hn);
//                 open_D_.push(nei_n);
//             }
//             else{
//                 /* in close list */
//                 if(hn->sch_node_->flag_ & 2) continue;
//                 /* try to change parent */
//                 nei_n = hn->sch_node_;
//                 if(cur_n->g_ + edge_length + 1e-3 < nei_n->g_){
//                     nei_n->flag_ |= 2;
//                     hn->sch_node_ = make_shared<DTG_sch_node>(cur_n->g_ + edge_length, 0.0, hn->id_, hn->pos_);
//                     hn->sch_node_->parent_ = cur_n;
//                     hn->sch_node_->flag_ = 1;
//                     open_D_.push(hn->sch_node_);
//                 }
//             }
//         }
//     }
//     ClearSearched(searched_h);
// }


void MultiDtgPlus::MainTainDistMap(h_ptr &hs, vector<h_ptr> &tars){
    list<h_ptr> searched_h;
    list<h_ptr> h_path;
    h_ptr hc, hn;
    prio_D empty_set;
    shared_ptr<DTG_sch_node> cur_n, nei_n;
    int found_num = 0;
    for(auto &t : tars) t->h_flags_ |= 16;

    open_D_.swap(empty_set);
    searched_h.emplace_back(hs);
    hs->sch_node_ = make_shared<DTG_sch_node>(0.0, 0.0, hs->id_, hs->pos_);
    cur_n = hs->sch_node_;
    open_D_.push(cur_n);

    while(!open_D_.empty()){
        cur_n = open_D_.top();

        open_D_.pop();
        if(cur_n->flag_ & 2) continue;
        cur_n->flag_ |= 2;
        if(!FindHnode(cur_n->pos_, cur_n->id_, hc)){
            ROS_ERROR("error get hnode MainTainDistMap");
            ros::shutdown();
            return;
        }

        if(hc->h_flags_ & 16){
            /* retrieve path */
            RetrieveHnPath(hc, h_path);
            uint64_t idx = max(hc->id_, hs->id_) + min(hc->id_, hs->id_) * 10000000;
            h_dist_map_[idx] = {hc->sch_node_->g_, h_path};
            found_num++;
            if(found_num == tars.size()){
                for(auto &t : tars) t->h_flags_ &= 239;
                ClearSearched(searched_h);
                return;
            }
        }

        /* expand Hneighbours */
        for(auto &hhe : hc->hh_edges_){
            if(hhe->head_n_ == hc)
                hn = hhe->tail_n_;
            else
                hn = hhe->head_n_;
            if(hn->sch_node_ == NULL){
                /* new node */
                hn->sch_node_ = make_shared<DTG_sch_node>(cur_n->g_ + hhe->length_, 0.0, hn->id_, hn->pos_);
                nei_n = hn->sch_node_;
                nei_n->parent_ = cur_n;
                nei_n->flag_ = 1;
                searched_h.push_back(hn);
                open_D_.push(nei_n);
            }
            else{
                /* in close list */
                if(hn->sch_node_->flag_ & 2) continue;

                /* try to change parent */
                nei_n = hn->sch_node_;
                if(cur_n->g_ + hhe->length_ + 1e-3 < nei_n->g_){
                    nei_n->flag_ |= 2;
                    hn->sch_node_ = make_shared<DTG_sch_node>(cur_n->g_ + hhe->length_, 0.0, hn->id_, hn->pos_);
                    hn->sch_node_->parent_ = cur_n;
                    hn->sch_node_->flag_ = 1;
                    open_D_.push(hn->sch_node_);
                }
            }
        }

    }
    ClearSearched(searched_h);
    for(auto &t : tars) t->h_flags_ &= 239;

    return;
}

bool MultiDtgPlus::Astar(const Eigen::Vector3d &ps, const vector<h_ptr> &p_hs, const vector<double> &g0_hs, h_ptr &tar, 
    const vector<vector<Eigen::Vector3d>> &paths, vector<Eigen::Vector3d> &path, double &dist){
    list<h_ptr> searched_h;
    list<h_ptr> h_path;
    h_ptr hc, hn, hs;
    prio_A empty_set;
    shared_ptr<DTG_sch_node> cur_n, nei_n;
    path.clear();
    hs = make_shared<H_node>();
    hs->id_ = 0;
    // for(auto &p : paths){
    //     if(p.size() == 0){
    //         for(auto h : p_hs){
    //             cout<<"in local:"<<LRM_->InsideMap(h->pos_)<<endl;
    //             cout<<"Connected:"<<LRM_->Connected(h->pos_)<<endl;
    //             cout<<"h pos:"<<h->pos_.transpose()<<endl;
    //         }
    //         StopageDebug("p.size() == 0");
    //     }
    // }
    // if(paths.size() == 0){
    //     StopageDebug("paths.size() == 0");
    // }
    hs->sch_node_ = make_shared<DTG_sch_node>(0.0, 0.0, 0, ps);
    open_A_.swap(empty_set);
    open_A_.push(hs->sch_node_);
    for(int i = 0; i < p_hs.size(); i++){
        hhe_ptr edge = make_shared<hhe>();
        edge->head_n_ = hs;
        edge->tail_n_ = p_hs[i];
        edge->length_ = g0_hs[i];
        for(auto p : paths[i]) edge->path_.emplace_back(p);
        hs->hh_edges_.emplace_back(edge);
    }


    while(!open_A_.empty()){
        cur_n = open_A_.top();

        open_A_.pop();
        if(cur_n->flag_ & 2) continue;
        cur_n->flag_ |= 2;
        if(!FindHnode(cur_n->pos_, cur_n->id_, hc) && cur_n->id_ != 0){
            ROS_ERROR("error get hnode MainTainDistMap");
            ros::shutdown();
            ClearSearched(searched_h);
            return false;
        }
        else if(cur_n->id_ == 0) hc = hs;

        /* find target */
        if(hc == tar){
            RetrieveHPath(hs, tar, path, dist);
            // Debug(path, -205);
            ClearSearched(searched_h);
            
            return true;
        }

        /* expand Hneighbours */
        for(auto &hhe : hc->hh_edges_){
            if(hhe->head_n_ == hc)
                hn = hhe->tail_n_;
            else
                hn = hhe->head_n_;

            if(hn == NULL){
                StopageDebug("hn == NULL");
            }

            if(hn->sch_node_ == NULL){
                /* new node */
                hn->sch_node_ = make_shared<DTG_sch_node>(cur_n->g_ + hhe->length_, cur_n->g_ + hhe->length_ + (hn->pos_ - tar->pos_).norm(), hn->id_, hn->pos_);
                nei_n = hn->sch_node_;
                nei_n->parent_ = cur_n;
                nei_n->flag_ = 1;
                searched_h.push_back(hn);
                open_A_.push(nei_n);
            }
            else{
                /* in close list */
                if(hn->sch_node_->flag_ & 2) continue;

                /* try to change parent */
                nei_n = hn->sch_node_;

                if(cur_n->g_ + hhe->length_ + 1e-3 + (hn->pos_ - tar->pos_).norm() < nei_n->f_){
                    nei_n->flag_ |= 2;
                    hn->sch_node_ = make_shared<DTG_sch_node>(cur_n->g_ + hhe->length_, cur_n->g_ + hhe->length_ + (hn->pos_ - tar->pos_).norm(), hn->id_, hn->pos_);
                    hn->sch_node_->parent_ = cur_n;
                    hn->sch_node_->flag_ = 1;
                    open_A_.push(hn->sch_node_);
                }
            }
        }

    }
    ClearSearched(searched_h);

    return false;
}

void MultiDtgPlus::ParallelDijkstra(vector<h_ptr> &p_hs, vector<double> &g0_hs, vector<h_ptr> &tars, vector<pair<double, list<h_ptr>>> &paths){
    list<h_ptr> searched_h;
    list<h_ptr> h_path;
    h_ptr hc, hn;
    prio_D empty_set;
    shared_ptr<DTG_sch_node> cur_n, nei_n;
    // int found_num = 0;
    paths.clear();
    tars.clear();
    // paths.resize(tars.size());
    // for(auto &t : tars) {
    //     t->h_flags_ |= 16;
    // }

    open_D_.swap(empty_set);
    for(int i = 0; i < p_hs.size(); i++){
        auto hs = p_hs[i];
        searched_h.emplace_back(hs);
        hs->sch_node_ = make_shared<DTG_sch_node>(g0_hs[i], 0.0, hs->id_, hs->pos_);
        // cout<<"g0_hs[i]:"<<g0_hs[i]<<endl;
        cur_n = hs->sch_node_;
        open_D_.push(cur_n);
    }
    while(!open_D_.empty()){
        cur_n = open_D_.top();

        open_D_.pop();
        if(cur_n->flag_ & 2) continue;
        cur_n->flag_ |= 2;
        if(!FindHnode(cur_n->pos_, cur_n->id_, hc)){
            ROS_ERROR("error get hnode MainTainDistMap");
            ros::shutdown();
            return;
        }

        if(!hc->hf_edges_.empty()){
            /* retrieve path */
            // RetrieveHnPath(hc, h_path);
            tars.emplace_back(hc);
            paths.emplace_back(make_pair(hc->sch_node_->g_, h_path));

        }

        /* expand Hneighbours */
        for(auto &hhe : hc->hh_edges_){
            if(hhe->head_n_ == hc)
                hn = hhe->tail_n_;
            else
                hn = hhe->head_n_;
            if(hn->sch_node_ == NULL){
                /* new node */
                hn->sch_node_ = make_shared<DTG_sch_node>(cur_n->g_ + hhe->length_, 0.0, hn->id_, hn->pos_);
                nei_n = hn->sch_node_;
                nei_n->parent_ = cur_n;
                nei_n->flag_ = 1;
                searched_h.push_back(hn);
                open_D_.push(nei_n);
            }
            else{
                /* in close list */
                if(hn->sch_node_->flag_ & 2) continue;

                // cout<<"hhe->length_ :"<<hhe->length_ <<endl;
                // cout<<"cur_n->g_ :"<<cur_n->g_ <<endl;
                // cout<<"cur_n->id_ :"<<cur_n->id_ <<endl;

                /* try to change parent */
                nei_n = hn->sch_node_;
                if(cur_n->g_ + hhe->length_ + 1e-3 < nei_n->g_){
                    nei_n->flag_ |= 2;
                    hn->sch_node_ = make_shared<DTG_sch_node>(cur_n->g_ + hhe->length_, 0.0, hn->id_, hn->pos_);
                    hn->sch_node_->parent_ = cur_n;
                    hn->sch_node_->flag_ = 1;
                    open_D_.push(hn->sch_node_);
                }
            }
        }

    }
    // cout<<"searched_h num:"<<searched_h.size()<<endl;
    ClearSearched(searched_h);

    return;
}

void MultiDtgPlus::Prim(Eigen::MatrixXd &dist_mat, vector<int> &parent, vector<vector<int>> &branches){
    int n = dist_mat.rows();
    vector<shared_ptr<DTG_sch_node>> nodes(n);
    shared_ptr<DTG_sch_node> cur_n, nei_n;
    for(int i = 0; i < n; i++){

        nodes[i] = make_shared<DTG_sch_node>(dist_mat(0, i)+1e-3, 0.0, i, Eigen::Vector3d(0.0, 0.0, 0.0));
        if(i > 0){
            nodes[i]->parent_ = nodes[0];
        }
        open_D_.push(nodes[i]);
    }

    open_D_.push(nodes[0]);
    while(!open_D_.empty()){
        cur_n = open_D_.top();
        open_D_.pop();
        if(cur_n->flag_ & 2) continue;
        cur_n->flag_ |= 2;
        // cout<<"cur:"<<cur_n->id_<<endl;
        for(int i = 1; i < n; i++){
            if(i == cur_n->id_) continue;
            if(!(nodes[i]->flag_ & 2) && dist_mat(cur_n->id_, i) < nodes[i]->g_){
                nodes[i]->flag_ |= 2;
                // cout<<"dist_mat:"<<dist_mat(cur_n->id_, i)<<endl;
                // cout<<"i:"<<i<<endl;
                // cout<<"par:"<<cur_n->id_<<endl;
                nodes[i] = make_shared<DTG_sch_node>(dist_mat(cur_n->id_, i), 0.0, i, Eigen::Vector3d(0.0, 0.0, 0.0));
                nodes[i]->parent_ = cur_n;
                nodes[i]->g_ = dist_mat(cur_n->id_, i);
                open_D_.push(nodes[i]);
            }
        }
    }

    parent.resize(n);
    branches.resize(n);
    for(int i = 1; i < n; i++){
        if(nodes[i]->parent_ == NULL){
            parent[i] = -1;
            StopageDebug("Prim parent == NULL");
        }
        else{
            parent[i] = nodes[i]->parent_->id_;
            branches[nodes[i]->parent_->id_].emplace_back(i);
        }
    }

}

void MultiDtgPlus::FindFastExpTarget( const vector<h_ptr> &route, const Eigen::Vector4d &ps, 
    const Eigen::Vector4d &vs, const vector<Eigen::Vector3d> &path1, const double &d1,
    uint8_t &plan_res, 
    vector<Eigen::Vector3d> &path_stem, vector<Eigen::Vector3d> &path_main, 
    vector<Eigen::Vector3d> &path_sub, vector<Eigen::Vector3d> &path_norm, 
    pair<uint32_t, uint8_t> &tar_stem, pair<uint32_t, uint8_t> &tar_main, 
    pair<uint32_t, uint8_t> &tar_sub, pair<uint32_t, uint8_t> &tar_norm,
    double &y_stem, double &y_main, double &y_sub, double &y_norm){
    plan_res = 1;

    if(route.empty()){
        return;
    }

    /* find the first vp */
    Eigen::Vector3d vs3 = vs.head(3);
    vector<uint8_t> local_fastexp_candidate_vps;
    list<pair<uint32_t, uint8_t>> connected_vps;
    list<hfe_ptr> not_connected_vps;
    list<hfe_ptr> candidate_hfe; // first hn's hfe + virtual hfes
    vector<vector<bool>> feasible_motions;
    Eigen::Vector4d vp, refined_main_vp;
    Eigen::Vector3d vp_pos, vp_best1, vp_best2;
    Eigen::Vector3i ci1, ci2;
    uint32_t fid1, fid2;
    shared_ptr<lowres_lite::LR_node> lrn;
    pair<uint8_t, Eigen::Vector3i> sec_vp;
    int sec_vp_idx;
    hfe_ptr f_hfe, s_hfe;



    bool find_first = false;
    bool first_branch = false;
    pair<uint32_t, uint8_t> best_tar1, best_tar2, sbest_tar1;
    double best_gain = 0, sbest_gain = 0;
    double gain1=0, gain2=0;
    double t;
    double vm, arc;
    double vm_best, arc_best;
    auto h = route.front();

    for(auto &hfe : h->hf_edges_){
        candidate_hfe.emplace_back(hfe);
        EROI_->EROI_[hfe->tail_n_->fid_].flags_ |= 1;
    }

    list<Eigen::Vector3d> debug_vps;
    for(int x = LRM_->local_origin_idx_(0); x <= LRM_->local_up_idx_(0); x++){
        for(int y = LRM_->local_origin_idx_(1); y <= LRM_->local_up_idx_(1); y++){
            for(int z = LRM_->local_origin_idx_(2); z <= LRM_->local_up_idx_(2); z++){
                int idx = x + y*LRM_->block_num_(0) + z*LRM_->block_num_(0)*LRM_->block_num_(1);
                if(idx < 0 && idx >= EROI_->EROI_.size()) continue;
                if(EROI_->EROI_[idx].flags_ & 1 || EROI_->EROI_[idx].f_state_ != 1) continue;
                double min_dist = 9999999999.0;
                int best_vp_id = -1;

                Eigen::Vector3d p_best;
                for(int i = 0; i < EROI_->EROI_[idx].local_vps_.size(); i++){
                    if(EROI_->EROI_[idx].local_vps_[i] != 1) continue;
                    vp_pos = EROI_->vps_[i].head(3) + EROI_->EROI_[idx].center_;

                    if(!LRM_->Connected(vp_pos) || !LRM_->InsideLocalMap(EROI_->EROI_[idx].center_)) continue;
                    lrn = LRM_->GlobalPos2LocalNode(vp_pos);
                    if(lrn->path_g_ > H_thresh_ * 0.8) continue;
                    if(lrn->path_g_ < min_dist) {
                        min_dist = lrn->path_g_;
                        best_vp_id = i;
                        p_best = vp_pos;
                    }
                }

                if(best_vp_id != -1){
                    auto e = make_shared<hfe>();
                    e->head_n_ = NULL;
                    e->tail_n_ = make_shared<FC_node>();
                    e->tail_n_->fid_ = idx;
                    e->tail_n_->vid_ = best_vp_id;
                    candidate_hfe.emplace_back(e);
                    debug_vps.emplace_back(p_best);
                }
            }
        }
    }
    // cout<<"debug_vps:"<<debug_vps.size()<<endl;
    // Debug(debug_vps);

    for(auto &hfe : h->hf_edges_){
        EROI_->EROI_[hfe->tail_n_->fid_].flags_ &= 254;
    }

    for(auto &hfe : candidate_hfe){
        fid1 = hfe->tail_n_->fid_;
        auto &eroi = EROI_->EROI_[fid1];
        EROI_->Idx2Posi(fid1, ci1);

        if(LRM_->InsideLocalMap(eroi.center_)){
            local_fastexp_candidate_vps.clear();
            for(uint8_t i = 0; i < eroi.local_vps_.size(); i++){

                if(eroi.local_vps_[i] == 1){
                    vp = EROI_->vps_[i];
                    vp.head(3) += eroi.center_;
                    vp_pos = vp.head(3);
                    vp_pos = LRM_->GetStdPos(vp_pos);
                    vp.head(3) = vp_pos;
                    if(LRM_->Connected(vp_pos)){
                        local_fastexp_candidate_vps.emplace_back(i);
                        connected_vps.emplace_back(fid1, i);
                    }
                    else if(hfe->tail_n_->vid_ == i){
                        not_connected_vps.emplace_back(hfe);
                        double dist = d1 + hfe->length_;

                        gain1 = GetGainExp1(ps, vp, vs3, dist);

                        if(gain1 > best_gain){

                            best_gain = gain1;

                            best_tar1.first = fid1;
                            best_tar1.second = i;
                            first_branch = false;
                            find_first = true;
                            vp_best1 = vp.head(3);
                        }
                    }
                }
            }

            if(!local_fastexp_candidate_vps.empty()){
                feasible_motions.clear();
                EROI_->FastMotionCheck(fid1, feasible_motions);
                for(auto &vp1 : local_fastexp_candidate_vps){
                    vp = EROI_->vps_[vp1];
                    vp.head(3) += eroi.center_;
                    vp_pos = vp.head(3);
                    
                    // debug_vps.emplace_back(vp_pos);
                    vp_pos = LRM_->GetStdPos(vp_pos);
                    vp.head(3) = vp_pos;

                    lrn = LRM_->GlobalPos2LocalNode(vp_pos);
                    if(lrn == NULL || lrn == LRM_->Outnode_){
                        StopageDebug("FindFastExpTarget lrn == NULL || lrn == LRM_->Outnode_");
                    }

                    vector<Eigen::Vector3d> path_temp;
                    double l;
                    Eigen::Vector3i vpi = LRM_->GlobalPos2GlobalId3(vp_pos);
                    if(!LRM_->RetrieveHPath(vpi, path_temp, l, true)){
                        StopageDebug("FindFastExpTarget RetrieveHPath failed--");
                    }
                    double amp = 1.0;
                    if(path_temp.size() < 4){
                        amp = 0.5;
                    }
                    if(path_temp.size() >= 5){
                        vp.head(3) = path_temp[4];
                    }
                    else{
                        // amp = 1.0;
                        vp.head(3) = path_temp.back();
                    }


                    gain1 = GetGainExp1(ps, vp, vs3, lrn->path_g_ ) * amp;
                    // if(gain1 == 0){
                    //     gain1 = GetGainExp1Debug(ps, vp, vs3, lrn->path_g_ );
                    //     cout<<"f v:"<<int(fid1)<<" "<<int(vp1)<<endl;
                    //     cout<<"lrn->path_g_:"<<lrn->path_g_<<endl;
                    //     cout<<"vp:"<<vp.transpose()<<endl;
                    //     cout<<"ps:"<<ps.transpose()<<endl;
                    //     cout<<"vs3:"<<vs3.transpose()<<endl;
                    //     cout<<"gain1:"<<gain1<<endl;
                    //     StopageDebug("gain == 1");
                    // }
                    if(gain1 > best_gain){
                        best_gain = gain1;

                        best_tar1.first = fid1;
                        best_tar1.second = vp1;
                        first_branch = false;
                        find_first = true;
                        vp_best1 = vp_pos;
                    }


                    for(int i2 = 0; i2 < feasible_motions[vp1].size(); i2++){

                        if(feasible_motions[vp1][i2]){
                            EROI_->GetMotionInfo(vp1, i2, vm, arc, sec_vp);
                            ci2 = ci1 + sec_vp.second;
                            fid2 = EROI_->Posi2Idx(ci2);
                            if(fid2 == -1) continue;
                            if(EROI_->EROI_[fid2].f_state_ >= 2 || EROI_->EROI_[fid2].local_vps_[sec_vp.first] == 2) continue;
                            Eigen::Vector3d vp2_pos = EROI_->vps_[sec_vp.first].head(3) + EROI_->EROI_[fid2].center_;
                            
                            gain2 = GetGainExp2(ps, vp, vs3, lrn->path_g_, vp2_pos/*eroi.center_*/, arc, vm, EROI_->vps_[sec_vp.first](3)) * amp;
                            // if((eroi.center_ + EROI_->vps_[vp1].head(3) - vp2_pos).norm() < 1.5){
                            //     cout<<"i2:"<<i2<<endl;
                            //     cout<<"vp1:"<<vp1<<endl;
                            //     StopageDebug("too close");
                            // }
                            if(gain2 > best_gain){



                                best_gain = gain2;

                                vm_best = vm;
                                arc_best = arc;
                                best_tar2.first = fid2;
                                best_tar2.second = sec_vp.first;
                                best_tar1.first = fid1;
                                best_tar1.second = vp1;
                                first_branch = true;
                                sec_vp_idx = i2;
                                find_first = true;
                                vp_best1 = vp_pos;
                            }



                        }
                        else{
                            fid2 = -1;
                        }
                    }

                }
            }
        }
        else{
            vp = EROI_->vps_[hfe->tail_n_->vid_];
            vp.head(3) += eroi.center_;
            vp_pos = vp.head(3);
            not_connected_vps.emplace_back(hfe);
            double dist = d1 + hfe->length_;

            gain1 = GetGainExp1(ps, vp, vs3, dist);

            if(gain1 > best_gain){
                best_gain = gain1;
                // if(debug_plan_) {
                //     debug_f_ <<"best_gain:"<<best_gain<<endl;
                // }
                best_tar1.first = fid1;
                best_tar1.second = hfe->tail_n_->vid_;
                find_first = true;
                first_branch = false;
                vp_best1 = vp_pos;
            }
        }
    }

    /* clear the first vp */
    for(auto i = connected_vps.begin(); i != connected_vps.end(); i++){
        if(i->first == best_tar1.first && i->second == best_tar1.second){
            connected_vps.erase(i);
            break;
        }
    }
    for(auto i = not_connected_vps.begin(); i != not_connected_vps.end(); i++){
        if((*i)->tail_n_->fid_ == best_tar1.first && (*i)->tail_n_->vid_ == best_tar1.second){
            f_hfe = *i;
            not_connected_vps.erase(i);
            break;
        }
    }
    /* find second target if current target is in local */
    if(LRM_->Connected(vp_best1)){
        tar_norm = best_tar1;
        tar_stem = best_tar1;
        plan_res = 3;
        // ROS_WARN("FindFastExpTarget2.1");

        /* get path to best_target1 */
        double l;
        Eigen::Vector3i vp_best1i = LRM_->GlobalPos2GlobalId3(vp_best1);
        if(!LRM_->RetrieveHPath(vp_best1i, path_norm, l, true)){
            // StopageDebug("FindFastExpTarget RetrieveHPath failed1");
        }

        path_stem = path_norm;
        y_stem = EROI_->YawDiff(EROI_->vps_[best_tar1.second](3), ps(3)) + ps(3);
        y_norm = y_stem;       
        if(first_branch) {

            EROI_->GetMotionPath(EROI_->EROI_[best_tar1.first].center_, best_tar1.second, sec_vp_idx, path_main);
            y_main = EROI_->YawDiff(EROI_->vps_[best_tar2.second](3), ps(3)) + ps(3);
            // cout<<"sec_vp_idx:"<<sec_vp_idx<<endl;
            // vp.head(3) = EROI_->EROI_[best_tar1.first].center_ + EROI_->vps_[sec_vp_idx].head(3);
            // vp(3) = y_main;
            // vector<Eigen::Vector3d> path_main_tempt = path_main;


            // debug
            // lrn = LRM_->GlobalPos2LocalNode(path_stem.back());

            // Eigen::Vector3d vp2_pos = EROI_->vps_[best_tar2.second].head(3) + EROI_->EROI_[best_tar2.first].center_;


            // gain2 = GetGainExp2(ps, vp, vs3, lrn->path_g_, vp2_pos/*eroi.center_*/, arc, vm, EROI_->vps_[sec_vp.first](3));

            // const Eigen::Vector3d &vs, const double &dist,
            // const Eigen::Vector3d &exp_center, const double &arc, const double &vm, const double &y2){

            // Eigen::Vector3d dp = (path_stem.back().head(3) - ps.head(3)).normalized();
            // Eigen::Vector3d dv = dp * vm_best - vs.head(3);
            // cout<<"====dp0:"<<dp.transpose()<<" dv0:"<<dv.transpose()<<endl;

            // double t = max(lrn->path_g_ / vm_best + dv.norm() / a_max_ * acc_gain_, abs(EROI_->YawDiff(y_stem, ps(3))) / yv_max_ * yaw_gain_);
            // double gain = exp(-lambda_ * t);

            // cout<<"ty0:"<<abs(EROI_->YawDiff(y_stem, ps(3))) / yv_max_ * yaw_gain_<<endl;
            // cout<<"tp0:"<<lrn->path_g_ / vm_best + dv.norm() / a_max_ * acc_gain_<<endl;
            // cout<<"g0:"<<exp(-lambda_ * t)<<endl;
            // dv = (vp2_pos - path_stem.back()).normalized()*vm_best - (path_stem.back() - ps.head(3)).normalized()*vm_best;
            // cout<<"dp2:"<<arc_best<<" dv2:"<<dv.transpose()<<endl;
            // t += max(arc_best / vm + dv.norm() / a_max_ * acc_gain_, abs(EROI_->YawDiff(y_main, y_stem)) / yv_max_ * yaw_gain_);
            // gain += exp(-lambda_ * t);
            // cout<<"g2:"<<exp(-lambda_ * t)<<endl;
            // cout<<"ty2:"<<abs(EROI_->YawDiff(y_main, y_stem)) / yv_max_ * yaw_gain_<<endl;


            // cout<<"gain:"<<gain<<endl;
            // cout<<"best_gain:"<<best_gain<<endl;
            


            // cout<<"vp1:"<<path_stem.back().transpose()<<" "<<y_stem<<endl;

            // ReFineTargetVp(ps, vs, vp, best_tar1.first, best_tar1.second, path_main_tempt, best_gain, vm_best, arc_best, refined_main_vp, path_stem, path_main);
            // cout<<"vp2 or:"<<vp.transpose()<<"  refined_main_vp:"<<refined_main_vp.transpose()<<endl;
            tar_main = best_tar2;
        }

        list<h_ptr> H_list, hvalid;
        vector<Eigen::Vector3d> tars;
        vector<double> dist;
        vector<h_ptr> p_hs; 
        vector<double> g0_hs;
        vector<vector<Eigen::Vector3d>> or_paths, h2_paths;
        vector<Eigen::Vector3d> path_2_h, path_2_best_h;
        double gain_sec = 0;
        double h_dist;
        int debug_count = 0;
        GetHnodesBBX(LRM_->local_map_upbd_, LRM_->local_map_lowbd_, H_list);
        for(auto &h : H_list){
            tars.emplace_back(h->pos_);
            if(LRM_->Connected(h->pos_)) debug_count++;
            hvalid.emplace_back(h);
        }
        // cout<<"debug_count:"<<debug_count<<endl;
        // ROS_WARN("FindFastExpTarget2.4");
        
        /* dijkstra, get all the connected nodes's shrotest paths */
        LRM_->ClearTopo();

        // ROS_WARN("search 2");
        LRM_->GetDists(vp_best1, tars, dist, or_paths, false); 
        // ROS_WARN("FindFastExpTarget2.45");

        auto d = dist.begin();
        auto hi = hvalid.begin();
        auto h_path = or_paths.begin();
        for(; d != dist.end(); d++, hi++, h_path++){
            if(*d < 999998.0){
                p_hs.emplace_back(*hi);
                g0_hs.emplace_back(*d);
                h2_paths.emplace_back(*h_path);
            }
        }
        // ROS_WARN("FindFastExpTarget2.5");

        /* find the second target connected to this h node */
        if(!not_connected_vps.empty() || !connected_vps.empty()){
            // ROS_WARN("FindFastExpTarget2.51");
            for(auto &v : connected_vps){
                vp = EROI_->vps_[v.second];
                vp.head(3) += EROI_->EROI_[v.first].center_;
                vp_pos = vp.head(3);
                lrn = LRM_->GlobalPos2LocalNode(vp_pos);
                gain_sec = exp(-lambda_e_ * lrn->path_g_ / v_max_); // yaw does not matter
                if(sbest_gain < gain_sec){
                    sbest_gain = gain_sec;
                    sbest_tar1 = v;
                    plan_res = 2;
                    if(!first_branch){
                        best_tar2 = v;
                    }
                }
            }
            // ROS_WARN("FindFastExpTarget2.52");

            /* get path to the first hn */
            if(not_connected_vps.size() > 0){
                auto h1 = route.front();
                if(!Astar(vp_best1, p_hs, g0_hs, h1, h2_paths, path_2_h, h_dist)){
                    StopageDebug("FindFastExpTarget Astar failed");
                }
            }
            // ROS_WARN("FindFastExpTarget2.53");

            for(auto &hfe : not_connected_vps){
                gain_sec = exp(-lambda_e_ * (h_dist + hfe->length_) / v_max_); // yaw does not matter
                // if(!LRM_->InsideMap(hfe->path_.back())) gain_sec = 9999999;
                if(sbest_gain < gain_sec){
                    path_2_best_h = path_2_h;
                    sbest_gain = gain_sec;
                    sbest_tar1.first = hfe->tail_n_->fid_;
                    sbest_tar1.second = hfe->tail_n_->vid_;
                    plan_res = 2;
                    s_hfe = hfe;
                    if(!first_branch){
                        best_tar2 = sbest_tar1;
                    }
                }
            }
            // if(plan_res != 2){
            //     StopageDebug("FindFastExpTarget, fail find sec 1");
            // }

            vp = EROI_->vps_[sbest_tar1.second];
            vp.head(3) += EROI_->EROI_[sbest_tar1.first].center_;
            vp_pos = vp.head(3);
            tar_sub = sbest_tar1;


            // vector<Eigen::Vector3d> path_de;
            // Eigen::Vector3d pde = ps.head(3);
            // if(LRM_->Connected(pde)){
            //     Eigen::Vector3d pd = vp_pos;
            //     path_de.emplace_back(pd);
            //     while(LRM_->Connected(pd)){
            //         pd(0) += 0.7;
            //         path_de.emplace_back(pd);
            //         cout<<"pd:"<<pd.transpose()<<endl;
            //         cout<<"!LRM_->InsideMap(pd):"<<!LRM_->InsideMap(pd)<<endl;
            //         cout<<"LRM_->Connected(pd):"<<LRM_->Connected(pd)<<endl;
            //         cout<<"LRM_->IsFeasible(pd):"<<LRM_->IsFeasible(pd)<<endl;

            //     }
            //     if(!LRM_->InsideLocalMap(pd)){
            //         for(auto p : path_de){
            //             cout<<"path_de:"<<p.transpose()<<endl;
            //         }                
            //         double uu;    
            //         LRM_->GetPathInLocal(uu, path_de, uu, path_de);
            //         for(auto p : path_de){
            //             cout<<"path_de:"<<p.transpose()<<endl;
            //         }            
            //         StopageDebug("path_de");
            //     }
            // }

            if(LRM_->Connected(vp_pos)){
                double ls;
                Eigen::Vector3i si = LRM_->GlobalPos2GlobalId3(vp_pos);
                LRM_->RetrieveHPath(si, path_sub, l, true);
                y_sub = y_stem;
            }
            else{
                // for(auto p : path_2_best_h){
                //     cout<<"p in path_2_best_h:"<<p.transpose()<<endl;
                // }
                path_sub = path_2_best_h;
                path_sub.pop_back();
                path_sub.insert(path_sub.end(), s_hfe->path_.begin(), s_hfe->path_.end());
                LRM_->GetPathInLocal(vp(3), path_sub, y_sub, path_sub);
                y_sub = y_stem;
            }
            if(!first_branch){
                tar_main = best_tar2;
                path_main = path_sub;
                y_main = y_sub;
            }

        }
        else if(route.size() > 1){
            auto h2 = route[1];
            if(!Astar(vp_best1, p_hs, g0_hs, h2, h2_paths, path_2_best_h, h_dist)){ // dont comment out
                // StopageDebug("FindFastExpTarget Astar failed");
            }

            for(auto &hfe : route[1]->hf_edges_){ 
                fid2 = hfe->tail_n_->fid_;
                auto &eroi = EROI_->EROI_[fid2];
                for(uint8_t i = 0; i < eroi.local_vps_.size(); i++){
                    vp = EROI_->vps_[i];
                    vp.head(3) += eroi.center_;
                    vp_pos = vp.head(3);
                    lrn = LRM_->GlobalPos2LocalNode(vp_pos);
                    if(hfe->tail_n_->vid_ == i) { 
                        gain_sec = exp(-lambda_e_ * (h_dist + hfe->length_) / v_max_); // yaw does not matter
                        if(sbest_gain < gain_sec){
                            sbest_gain = gain_sec;
                            sbest_tar1.first = hfe->tail_n_->fid_;
                            sbest_tar1.second = hfe->tail_n_->vid_;
                            plan_res = 2;
                            s_hfe = hfe;
                            if(!first_branch){
                                best_tar2 = sbest_tar1;
                            }
                        }
                    }
                    // else if(lrn->topo_sch_ != NULL && hfe->tail_n_->vid_ == i){ // this vp is connected
                    //     gain_sec = exp(-lambda_ * lrn->path_g_ / v_max_); // yaw does not matter
                    //     if(sbest_gain < gain_sec){
                    //         sbest_gain = gain_sec;
                    //         sbest_tar1.first = hfe->tail_n_->fid_;
                    //         sbest_tar1.second = hfe->tail_n_->vid_;
                    //         plan_res = 2;
                    //         if(!first_branch){
                    //             best_tar2 = sbest_tar1;
                    //         }
                    //     }
                    // }
                }
            }
            // if(plan_res != 2){
            //     StopageDebug("FindFastExpTarget, fail find sec 2");
            // }

            vp = EROI_->vps_[sbest_tar1.second];
            vp.head(3) += EROI_->EROI_[sbest_tar1.first].center_;
            vp_pos = vp.head(3);
            tar_sub = sbest_tar1;

            if(LRM_->Connected(vp_pos)){
                double ls;
                Eigen::Vector3i si = LRM_->GlobalPos2GlobalId3(vp_pos);
                LRM_->RetrieveHPath(si, path_sub, l, true);
                y_sub = y_stem;
            }
            else{
                path_sub = path_2_best_h;
                path_sub.pop_back();
                cout<<(s_hfe == NULL)<<endl;
                if((s_hfe == NULL)){
                    for(auto &hfe : route[1]->hf_edges_){ 
                        fid2 = hfe->tail_n_->fid_;
                        auto &eroi = EROI_->EROI_[fid2];
                        // cout<<"hfe v:"<<int(hfe->tail_n_->vid_)<<endl;

                        // for(auto &valid_vp : eroi.valid_vps_){
                        //     cout<<"eroi id:"<<fid2<<"valid_vp:"<<valid_vp<<endl;
                        // }
                        for(uint8_t i = 0; i < eroi.local_vps_.size(); i++){
                            vp = EROI_->vps_[i];
                            vp.head(3) += eroi.center_;
                            vp_pos = vp.head(3);
                            lrn = LRM_->GlobalPos2LocalNode(vp_pos);
                            if(hfe->tail_n_->vid_ == i) { // this vp is not connected, but connected to hn
                                gain_sec = exp(-lambda_e_ * (h_dist + hfe->length_) / v_max_); // yaw does not matter
                                // cout<<"gain_sec1:"<<gain_sec<<endl;
                            }
                            // else if(lrn->topo_sch_ != NULL){
                            //     gain_sec = exp(-lambda_ * lrn->path_g_ / v_max_); // yaw does not matter
                            //     cout<<"gain_sec2:"<<gain_sec<<endl;
                            // }

                        }
                    }


                    // StopageDebug("(s_hfe == NULL)");
                }
                path_sub.insert(path_sub.end(), s_hfe->path_.begin(), s_hfe->path_.end());

                LRM_->GetPathInLocal(vp(3), path_sub, y_sub, path_sub);
                y_sub = y_stem;
            }

            if(!first_branch){

                tar_main = best_tar2;
                path_main = path_sub;
                y_main = y_sub;
            }

        }

    }
    else{

        tar_norm = best_tar1;
        plan_res = 3;
        path_norm = path1;
        // list<Eigen::Vector3d> debug_pts2;
        // for(double x = -10.0; x < 10.0; x += EROI_->node_scale_(0)){
        //     for(double y = -10.0; y < 10.0; y += EROI_->node_scale_(1)){
        //         for(double z = -10.0; z < 10.0; z += EROI_->node_scale_(2)){
        //             Eigen::Vector3d p(x, y, z);
        //             p += ps.head(3);
        //             int id = EROI_->Pos2Idx(p);
        //             if(id != -1 && EROI_->EROI_[id].f_state_ == 1){
        //                 // debug_pts2.emplace_back(EROI_->EROI_[id].center_);
        //                 for(uint8_t vid = 0; vid < EROI_->EROI_[id].local_vps_.size(); vid++){
        //                     if(EROI_->EROI_[id].local_vps_[vid] == 1){
        //                         p = EROI_->EROI_[id].center_ + EROI_->vps_[vid].head(3);
        //                         cout<<"alive p:"<<p.transpose()<<" connected:"<<LRM_->Connected(p)<<endl;
        //                         if(LRM_->Connected(p)){
        //                             // debug_pts2.emplace_back(p);
        //                         }
        //                     }
        //                 }
        //             }
        //         }
        //     }
        // }
        // Debug(debug_pts2, 2);

        // StopageDebug("just stop");

        path_norm.pop_back();
        path_norm.insert(path_norm.end(), f_hfe->path_.begin(), f_hfe->path_.end());

        LRM_->GetPathInLocal(EROI_->vps_[tar_norm.second](3), path_norm, y_norm, path_norm);
        y_norm = EROI_->YawDiff(y_norm, ps(3)) + ps(3);

    }

    return;
}