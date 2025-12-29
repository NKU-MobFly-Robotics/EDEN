#ifndef MR_DTG_PLUS_H_
#define MR_DTG_PLUS_H_

#include <ros/ros.h>
#include <thread>
#include <Eigen/Eigen>
#include <vector>
#include <list>
#include <visualization_msgs/MarkerArray.h>
#include <swarm_exp_msgs/DtgFFEdge.h>
#include <swarm_exp_msgs/DtgHFEdge.h>
#include <swarm_exp_msgs/DtgHHEdge.h>
#include <swarm_exp_msgs/DtgHNode.h>
#include <swarm_exp_msgs/DtgFNode.h>

#include <fstream>
#include <tr1/unordered_map>
#include <eroi/eroi.h>

#include <mr_dtg_plus/mr_dtg_plus_structures.h>
#include <block_map_lite/color_manager.h>
#include <block_map_lite/block_map_lite.h>
#include <lowres_map_lite/lowres_map_lite.h>
#include <swarm_data/swarm_data.h>

using namespace std;
namespace DTGPlus{
typedef shared_ptr<H_node> h_ptr;
typedef shared_ptr<FC_node> f_ptr;
typedef shared_ptr<DTG_edge<H_node, FC_node>> hfe_ptr; // only length_ is used
typedef shared_ptr<DTG_edge<H_node, H_node>> hhe_ptr;
typedef shared_ptr<DTG_edge<FC_node, FC_node>> ffe_ptr;
typedef DTG_edge<H_node, FC_node> hfe;
typedef DTG_edge<H_node, H_node> hhe;
typedef DTG_edge<FC_node, FC_node> ffe;
class GraphVoronoiPartition;
class MultiDtgPlus{

public:
    MultiDtgPlus(){};
    ~MultiDtgPlus(){};

    void AlignInit(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
    void GetHnodesBBX(const Eigen::Vector3d &upbd, const Eigen::Vector3d &lowbd, list<h_ptr> &H_list);
    void GetFnodesBBX(const Eigen::Vector3d &upbd, const Eigen::Vector3d &lowbd, list<f_ptr> &F_list);
    void GetHnodesAndFnodesBBX(const Eigen::Vector3d &upbd, const Eigen::Vector3d &lowbd,
         list<h_ptr> &H_list, list<f_ptr> &F_list);


    void BfUpdate(const Eigen::Vector3d &robot_pos);
    void SetLowresMap(lowres_lite::LowResMap *LRM){LRM_ = LRM;};
    void SetBlockMap(BlockMapLite *BM){BM_ = BM;};
    void SetFrontierMap(EroiGrid *FG){EROI_ = FG;};
    void SetColorManager(ColorManager *CM){CM_ = CM;};
    // void SetSwarmDataManager(SwarmDataManager *SDM){SDM_ = SDM;};

    void RemoveVp(const Eigen::Vector3d &center, int const &f_id, int const &v_id, bool broad_cast = false);
    void RemoveVp(uint32_t const &f_id, uint8_t const &v_id, bool broad_cast = false);

    /**
     * @brief find exploration target, assume lrm dijkstra is still
     * 
     * @param route         h route
     * @param ps            start pose
     * @param vs            start vel
     * @param plan_res      plan result: 1=no target, 2=sms, 3=n
     * @param path1         path to h1
     * @param d1            dist to h1
     * @param path_stem     stem: path to the first viewpoint 
     * @param path_main     main: path to the second viewpoint, not ensure safety 
     * @param path_sub      sub: path to the second viewpoint or a safe place, ensure safety
     * @param path_norm     norm: only one target left, i.e., last viewpoint or viewpoint not in local, need long distance transfer
     * @param tar_stem      <eroi id, vp id>
     * @param tar_main      <eroi id, vp id>
     * @param tar_sub       <eroi id, vp id>
     * @param tar_norm      <eroi id, vp id>
     * @param y_stem        yaw stem
     * @param y_main        yaw main
     * @param y_sub         yaw sub 
     * @param y_norm        yaw norm
     */
    void FindFastExpTarget( const vector<h_ptr> &route,
        const Eigen::Vector4d &ps, const Eigen::Vector4d &vs, 
        const vector<Eigen::Vector3d> &path1, const double &d1, uint8_t &plan_res, 
        vector<Eigen::Vector3d> &path_stem, vector<Eigen::Vector3d> &path_main, 
        vector<Eigen::Vector3d> &path_sub, vector<Eigen::Vector3d> &path_norm, 
        pair<uint32_t, uint8_t> &tar_stem, pair<uint32_t, uint8_t> &tar_main, 
        pair<uint32_t, uint8_t> &tar_sub, pair<uint32_t, uint8_t> &tar_norm,
        double &y_stem, double &y_main, double &y_sub, double &y_norm);

    /**
     * @brief Get the Closest Global Frontier Node. Djkstra.
     * 
     * @param path          path to the corresponding frontier viewpoint
     * @param H_path        hnodes of the path passed by
     * @param f_id          frontier id
     * @param v_id          viewpoint id
     * @param length        path length
     * @return true         find the closest frontier
     * @return false        no frontier in current connected graph
     */
    bool GetClosestGlobalTarget(list<Eigen::Vector3d> &path, list<h_ptr> &H_path, int &f_id, int &v_id, double &length);

    /**
     * @brief Get the Global Targets
     * 
     * @param s_hn start hn
     * @param paths <length, path>
     * @param t_hn target hns
     */
    void GetGlobalTarget(h_ptr &s_hn, vector<pair<double, list<Eigen::Vector3d>>> &paths, vector<h_ptr> &t_hn/*, vector<list<h_ptr>> &debug_paths*/);
    
    void ClearSearched(list<h_ptr> &h_l);
    void ClearSearched(list<f_ptr> &f_l);

    /**
     * @brief retrieve DTG path from a hnode to a fnode
     * 
     * @param tar_f   target fnode
     * @param h_path  h node path
     * @param path    pos path
     * @param length  path length
     */
    void RetrieveHFPath(f_ptr &tar_f, list<h_ptr> &h_path, list<Eigen::Vector3d> &path, double &length);

    /**
     * @brief retrieve DTG path from a hnode to a hnode
     * 
     * @param start_h   start hnode
     * @param tar_f   target hnode
     * @param path    pos path
     * @param length  path length
     */
    void RetrieveHPath(h_ptr &start_h, h_ptr &tar_h, list<Eigen::Vector3d> &path, double &length);
    void RetrieveHPath(h_ptr &start_h, h_ptr &tar_h, vector<Eigen::Vector3d> &path, double &length);
    void RetrieveHPathDebug(h_ptr &start_h, h_ptr &tar_h, list<Eigen::Vector3d> &path, double &length);
    void RetrieveHnPath(h_ptr &tar_h, list<h_ptr> &path);

    void RetrieveHDebug(h_ptr &start_h, h_ptr &tar_h, list<h_ptr> &h_path);
    
    /**
     * @brief find tsp ans through approximate method
     * 
     * @param ps      pos start
     * @param route_h ans
     * @param path2fh h path to the first h
     * @param d1      distance to h1
     * @return true current pos is connected to graph
     * @return false 
     */
    bool TspApproxiPlan(const Eigen::Vector3d &ps, vector<h_ptr> &route_h, vector<Eigen::Vector3d> &path2fh, double &d1);

    /**
     * @brief Erase the fc node and its corresponding edge from the graph
     * 
     * @param fid eroi id
     * @param vid cluster id
     * @param erase_edge if erase hf edge
     */
    inline void EraseFnodeFromGraph(const uint32_t &fid, const uint8_t &vid, bool erase_edge = true);

    
    /**
     * @brief Erase the fc node and its corresponding edge from the graph
     * 
     * @param fid eroi id
     */
    inline void EraseEROIFromGraph(const uint32_t &fid);

    /**
     * @brief find a eroi's cluster
     * 
     * @param vp_pos viewpoint position
     * @param id     eroi id
     * @param vid    vp id
     * @param fp     the ptr will point to the corresponding eroi's cluster if exists
     * @return true     cluster exists
     * @return false    fail to find 
     */
    inline bool FindFnode(const Eigen::Vector3d &vp_pos, const uint32_t &id, const uint8_t &vid, f_ptr &fp);

    /**
     * @brief find a history node
     * 
     * @param pos position of the history node
     * @param id  history node id
     * @param hp  the ptr will point to the corresponding history node if exists
     * @return true     cluster exists
     * @return false    fail to find 
     */
    inline bool FindHnode(const Eigen::Vector3d &pos, const uint32_t &id, h_ptr &hp);

    /**
     * @brief find a history node
     * 
     * @param idx voxel index
     * @param id  history node id
     * @param hp  the ptr will point to the corresponding history node if exists
     * @return true     cluster exists
     * @return false    fail to find 
     */
    inline bool FindHnode(const uint32_t &idx, const uint32_t &id, h_ptr &hp);


    /**
     * @brief A*, start from p_hs
     * 
     * @param ps    pos start
     * @param p_hs  starts
     * @param g0_hs start g
     * @param tar   target 
     * @param paths start paths
     * @param path  searched path
     * @param dist  path length
     * @return true fand path
     * @return false 
     */
    bool Astar(const Eigen::Vector3d &ps, const vector<h_ptr> &p_hs, const vector<double> &g0_hs, h_ptr &tars, 
        const vector<vector<Eigen::Vector3d>> &paths, vector<Eigen::Vector3d> &path, double &dist);
    // SwarmDataManager *SDM_;

private:
    // void ClearLocalRootPath();
    friend class GraphVoronoiPartition;

    inline bool InsideMap(const Eigen::Vector3i &idx3);
    inline bool InsideMap(const Eigen::Vector3d &pos);
    inline bool GetVox(const Eigen::Vector3i &idx3, list<h_ptr> &h_l, list<f_ptr> &f_l);
    inline bool GetVox(const Eigen::Vector3d &pos, list<h_ptr> &h_l, list<f_ptr> &f_l);
    inline bool GetVox(const Eigen::Vector3d &pos, const uint32_t &idx, h_ptr &h_l);

    inline Eigen::Vector3i GetVoxId3(const Eigen::Vector3d &pos);
    inline int GetVoxId(const Eigen::Vector3i &idx3);
    inline int GetVoxId(const Eigen::Vector3d &pos);

    inline void BreakHFEdge(h_ptr &h, f_ptr &f);

    inline bool GetEdge(h_ptr &h, f_ptr &f, hfe_ptr &e, bool search_f = true);
    inline bool GetEdge(h_ptr &h1, h_ptr &h2, hhe_ptr &e);

    inline h_ptr CreateSwarmHnode(const Eigen::Vector3d &pos, const uint32_t &id);
    inline h_ptr CreateHnode(const Eigen::Vector3d &pos);
    // inline f_ptr CreateFnode(const Eigen::Vector3d &vp_pos, const uint32_t &f_id, const uint8_t &vid);
    inline bool CreateConnectFnode(const Eigen::Vector3d &vp_pos, const uint32_t &f_id, const uint8_t &vid, const double &el, h_ptr &hn);

    /**
     * @brief block edge, set edge infeasible, length = 2e5, dont send to other robots
     * 
     * @param e 
     */
    inline void BlockEdge(hhe_ptr &e);

    /**
     * @brief block edge, set edge infeasible, length = 2e5, fn->p_id = -1, dont send to other robots
     * 
     * @param e 
     */
    inline void BlockEdge(hfe_ptr &e);

    inline void EraseEdge(hhe_ptr &e, const bool &broadcast = true);
    inline void EraseFnodeOnly(const uint32_t &fid, const uint8_t &vid);

    // inline void EraseEdge(ffe_ptr &e, const bool &broadcast = true);
    inline void EraseEdge(hfe_ptr &e, const bool &broadcast = true);
    /**
     * @brief create edge and connect hnode and fnode
     * 
     * @param h         head
     * @param f         tail
     * @param path      from head to tail
     * @param path_old  previous path, for later homotopy check
     * @param length    path length
     */
    inline void ConnectHF(h_ptr &h, f_ptr &f, const int &v_id,  list<Eigen::Vector3d> &path, list<Eigen::Vector3d> &path_old, double length);

    /**
     * @brief create edge and connect two Hnodes
     * 
     * @param head      head hnode
     * @param tail      tail hnode
     * @param path      from head to tail
     * @param path_old  previous path, for later homotopy check
     * @param length    path length
     */
    inline void ConnectHH(h_ptr &head, h_ptr &tail, list<Eigen::Vector3d> &path, list<Eigen::Vector3d> &path_old, double length);

    /**
     * @brief create edge and connect two Hnodes in swarm
     * 
     * @param head      head hnode
     * @param tail      tail hnode
     * @param path      from head to tail
     */
    inline bool ConnectSwarmHH(h_ptr &head, h_ptr &tail, list<Eigen::Vector3d> &path);

    /**
     * @brief create edge and connect Hnode and Fnode in swarm
     * 
     * @param head      head hnode
     * @param tail      fead hnode
     * @param path      from head to tail
     * @return true     shortest path
     * @return false    shorter path exists
     */
    inline bool ConnectSwarmHF(h_ptr &head, f_ptr &tail, list<Eigen::Vector3d> &path, uint8_t &vp_id);

    /**
     * @brief check if h and f in the node
     * 
     * @param pos position of the node
     * @param h    
     * @param f 
     * @return true 
     * @return false 
     */
    inline bool CheckNode(const Eigen::Vector3d &pos, const h_ptr &h, const f_ptr &f);

    // inline void SetTarget(double &g1, double &g2, double &bg, double &sbg,  
    //                         pair<uint32_t, uint8_t> &nt1,  pair<uint32_t, uint8_t> &nt2,
    //                         pair<uint32_t, uint8_t> &bt11,  pair<uint32_t, uint8_t> &bt12,  pair<uint32_t, uint8_t> &bt2,
    //                         bool &ff, bool &fs, bool &fb);

    // inline int LoadHHEdgeMSG(swarm_exp_msgs::DtgHHEdge &e); //0 success, 1 no node, 2 not the longest 
    // inline int LoadHFEdgeMSG(swarm_exp_msgs::DtgHFEdge &e); //0 success, 1 no node, 2 not the longest
    // inline h_ptr LoadHNodeMSG(swarm_exp_msgs::DtgHNode &h); //0 success, 1 fail

    // void DTGCommunicationCallback(const ros::TimerEvent &e);
    void NodesDebug();
    void ShowAll(const ros::TimerEvent &e); 
    void DistMaintTimerCallback(const ros::TimerEvent &e); 
    void Show();
    void Debug();
    void Debug(list<Eigen::Vector3d>  &fl, int idx = 0);
    void Debug(vector<Eigen::Vector3d>  &fl, int idx = 0);
    void DebugTree(Eigen::Vector3d ps, vector<h_ptr> &hs, vector<vector<int>> &branches);
    void DebugLineStrip(Eigen::Vector3d ps, vector<h_ptr> &route_h);
    void MainTainDistMap(h_ptr &hs, vector<h_ptr> &tars);
    void ParallelDijkstra(vector<h_ptr> &p_hs, vector<double> &g0_hs, vector<h_ptr> &tars, vector<pair<double, list<h_ptr>>> &paths);
    void Prim(Eigen::MatrixXd &dist_mat, vector<int> &parent, vector<vector<int>> &branches);

    /* call after LRM_->GetDists */
    void ReFineTargetVp(const Eigen::Vector4d &ps, const Eigen::Vector4d &vs, 
        const Eigen::Vector4d &vp2, const uint32_t &eroi_id, const uint8_t &vp_id, 
        const vector<Eigen::Vector3d> &path2v2, const double &motion_gain, const double &vm, const double &arc,
        Eigen::Vector4d &refined_vp1, vector<Eigen::Vector3d> &path2v1_r, vector<Eigen::Vector3d> &path2v2_r);
    void ReFineTargetVpSample(const Eigen::Vector4d &ps, const Eigen::Vector4d &vs, 
        const Eigen::Vector3d &r_vp_pos, const Eigen::Vector4d &vp2, const uint32_t &eroi_id, 
        const double &vm, const double &arc, const double &gain_0, const double &motion_gain,
                                Eigen::Vector4d &r_vp, double &better_gain);

    inline double GetGainExp1(const Eigen::Vector4d &ps, const Eigen::Vector4d &vp, const Eigen::Vector3d &vs, const double &dist);
    // inline double GetGainExp1Debug(const Eigen::Vector4d &ps, const Eigen::Vector4d &vp, const Eigen::Vector3d &vs, const double &dist);
    inline double GetGainExp2(const Eigen::Vector4d &ps, const Eigen::Vector4d &vp1, const Eigen::Vector3d &vs, const double &dist,
                            const Eigen::Vector3d &exp_center, const double &arc, const double &vm, const double &y2);



    inline void StopageDebug(string c);

    ros::NodeHandle nh_, nh_private_;
    ros::Publisher topo_pub_, debug_pub_;
    uint32_t cur_hid_;
    double H_thresh_, sensor_range_, eurange_;
    double lambda_e_, lambda_a_;
    double v_max_, a_max_, yv_max_;
    int uav_id_;
    bool show_e_details_;

    // list<f_ptr> local_f_list_;
    // list<double> local_f_dist_list_;
    // list<h_ptr> local_h_list_;
    // list<double> local_h_dist_list_;

    vector<list<h_ptr>> H_depot_;
    vector<list<f_ptr>> F_depot_;
    list<h_ptr> H_list_;
    list<Eigen::Vector3d> debug_pts_;
    h_ptr root_; 
    // list<shared_ptr<ffe>> r_f_edges_;//robot -> frontier
    Eigen::Vector3d origin_, vox_scl_, map_upbd_, map_lowbd_;
    Eigen::Vector3i vox_num_;

    prio_A open_A_;
    prio_D open_D_;

    ros::Timer show_timer_, maintain_timer_;

    EroiGrid *EROI_;
    lowres_lite::LowResMap *LRM_;
    BlockMapLite *BM_;
    ColorManager *CM_;

    // TSP
    random_device rd_;
    default_random_engine eng_;
    bool maintain_h_dist_;
    tr1::unordered_map<uint64_t, pair<double, list<h_ptr>>> h_dist_map_;
    list<h_ptr> update_list_;

    // motion 
    double acc_gain_, yaw_gain_, yaw_slice_ang_, g_thr_fac_;
    int refine_num_;

    // swarm
    ros::Timer swarm_timer_;
    bool use_swarm_;
    int drone_num_;

    //debug
    bool debug_plan_;
    ofstream debug_f_;
    
};

inline void MultiDtgPlus::EraseFnodeFromGraph(const uint32_t &fid, const uint8_t &vid, bool erase_edge){
    if(fid < EROI_->EROI_.size()){
        Eigen::Vector3d vp_pos = EROI_->vps_[vid].head(3);
        vp_pos += EROI_->EROI_[fid].center_;
        int idx = GetVoxId(vp_pos);
        if(idx == -1) return;
        for(auto fit = F_depot_[idx].begin(); fit != F_depot_[idx].end(); fit++){
            if((*fit)->vid_ == vid && fid == (*fit)->fid_){
                // cout<<"EraseFnodeFromGraph1:"<<fid<<"  "<<int(vid)<<endl;
                if(erase_edge) {
                    // if((*fit)->hf_edge_ == NULL){
                    //     StopageDebug("(*fit)->hf_edge_ == NULL");
                    // }
                    // if((*fit)->hf_edge_->head_n_ == NULL){
                    //     StopageDebug("(*fit)->hf_edge_->head_n_ == NULL");
                    // }
                    // if((*fit)->hf_edge_->tail_n_ == NULL){
                    //     StopageDebug("(*fit)->hf_edge_->tail_n_ == NULL");
                    // }
                    // cout<<"EraseFnodeFromGraph erase hfe:"<<(*fit)->hf_edge_->head_n_->id_<<"  "<<(*fit)->hf_edge_->tail_n_->fid_<<endl;
                    EraseEdge((*fit)->hf_edge_);
                }
                F_depot_[idx].erase(fit);
                // cout<<"EraseFnodeFromGraph2"<<endl;
                break;
            }
        }
        // cout<<"EraseFnodeFromGraph3"<<endl;
        EROI_->DeleteValidFnode(fid, vid);
        // cout<<"EraseFnodeFromGraph4"<<endl;

    }

}

inline void MultiDtgPlus::EraseEROIFromGraph(const uint32_t &fid){
    if(fid < EROI_->EROI_.size()){
        int c = 0;
        for(auto &i : EROI_->EROI_[fid].valid_vps_){
            if(fid < EROI_->EROI_.size()){
                Eigen::Vector3d vp_pos = EROI_->vps_[i].head(3);
                vp_pos += EROI_->EROI_[fid].center_;
                int idx = GetVoxId(vp_pos);
                if(idx == -1) return;
                // bool e = false;
                for(auto fit = F_depot_[idx].begin(); fit != F_depot_[idx].end(); fit++){
                    if((*fit)->vid_ == i && fid == (*fit)->fid_){
                        // cout<<"erase eroi:"<<fid<<"  "<<int(i)<<"   h:"<<(*fit)->hf_edge_->head_n_->id_<<endl;
                        EraseEdge((*fit)->hf_edge_);
                        F_depot_[idx].erase(fit);
                        // e = true;
                        break;
                    }
                }
                // if(!e) StopageDebug("e");
                // EROI_->DeleteValidFnode(fid, i);
            }
        }
        EROI_->EROI_[fid].valid_vps_.clear();
    }
}

inline bool MultiDtgPlus::InsideMap(const Eigen::Vector3i &idx3){
    if(idx3(0) < 0 || idx3(1) < 0 || idx3(2) < 0 ||
        idx3(0) >  vox_num_(0) - 1 || idx3(1) > vox_num_(1) - 1 || idx3(2) > vox_num_(2) - 1 )
        return false;
    return true;
}

inline bool MultiDtgPlus::InsideMap(const Eigen::Vector3d &pos){
    if(pos(0) < map_lowbd_(0)|| pos(1) < map_lowbd_(1)|| pos(2) < map_lowbd_(2)||
        pos(0) >  map_upbd_(0) || pos(1) > map_upbd_(1) || pos(2) > map_upbd_(2) )
        return false;
    return true;
}

inline bool MultiDtgPlus::GetVox(const Eigen::Vector3i &idx3, list<h_ptr> &h_l, list<f_ptr> &f_l){
    int idx = GetVoxId(idx3);
    if(idx != -1){
        h_l = H_depot_[idx];
        f_l = F_depot_[idx];
        return true;
    }
    return false;
}

inline bool MultiDtgPlus::GetVox(const Eigen::Vector3d &pos, list<h_ptr> &h_l, list<f_ptr> &f_l){
    int idx = GetVoxId(pos);
    if(idx != -1){
        h_l = H_depot_[idx];
        f_l = F_depot_[idx];
        return true;
    }
    return false;
}

inline bool MultiDtgPlus::GetVox(const Eigen::Vector3d &pos, const uint32_t &idx, h_ptr &h_l){
    int vox_idx = GetVoxId(pos);
    if(vox_idx != -1){
        for(auto &h : H_depot_[vox_idx]){
            if(h->id_ == idx){
                h_l = h;
                return true;
            }
        }
    }
    return false;
}

inline Eigen::Vector3i MultiDtgPlus::GetVoxId3(const Eigen::Vector3d &pos){
    return Eigen::Vector3i((int)floor((pos(0) - origin_(0))/vox_scl_(0)), (int)floor((pos(1) - origin_(1))/vox_scl_(1)),
         (int)floor((pos(2) - origin_(2))/vox_scl_(2))); 
}

inline int MultiDtgPlus::GetVoxId(const Eigen::Vector3i &idx3){
    if(InsideMap(idx3)){
        return idx3(0) + idx3(1) * vox_num_(0) + idx3(2) * vox_num_(0) * vox_num_(1); 
    }
    else return -1;
}

inline int MultiDtgPlus::GetVoxId(const Eigen::Vector3d &pos){
    if(InsideMap(pos)){
        int x = floor((pos(0) - origin_(0)) / vox_scl_(0));
        int y = floor((pos(1) - origin_(1)) / vox_scl_(1));
        int z = floor((pos(2) - origin_(2)) / vox_scl_(2));
        return x + y * vox_num_(0) + z * vox_num_(0) * vox_num_(1); 
    }
    else return -1;
}

inline void MultiDtgPlus::BreakHFEdge(h_ptr &h, f_ptr &f){
    for(list<shared_ptr<DTG_edge<H_node, FC_node>>>::iterator e = h->hf_edges_.begin(); e != h->hf_edges_.end(); e++){
        if((*e)->tail_n_ == f){
            h->hf_edges_.erase(e);
            return;
        }
    }
}

inline bool MultiDtgPlus::GetEdge(h_ptr &h, f_ptr &f, hfe_ptr &e, bool search_f){

    if(search_f){
        if(f->hf_edge_ != NULL && h == f->hf_edge_->head_n_){
            e = f->hf_edge_;
            return true;
        }
    }
    else{
        if(h != NULL){
            for(auto &hfe : h->hf_edges_){
                if(hfe->tail_n_->fid_ == f->fid_){
                    e = hfe;
                    return true;
                }
            }
        }
    }
    return false;
}

inline bool MultiDtgPlus::GetEdge(h_ptr &h1, h_ptr &h2, hhe_ptr &e){
    for(auto &ei : h1->hh_edges_){
        if(ei->tail_n_ == h2 || ei->head_n_ == h2){
            e = ei;
            return true;
        }
    }
    return false;
}

inline bool MultiDtgPlus::FindFnode(const Eigen::Vector3d &vp_pos, const uint32_t &fid, const uint8_t &vid, f_ptr &fp){
    int idx = GetVoxId(vp_pos);
    if(idx >= F_depot_.size() || idx < 0) return false;
    for(auto &c : F_depot_[idx]){
        if(fid == c->fid_ && vid == c->vid_){
            fp = c;
            return true;
        }
    }
    return false;
}

inline bool MultiDtgPlus::FindHnode(const Eigen::Vector3d &pos, const uint32_t &id, h_ptr &hp){
    int idx = GetVoxId(pos);
    if(idx == -1) return false;
    for(auto &h_l : H_depot_){
        for(auto &h : h_l){
            if(h->id_ == id){
                hp = h;
                return true;
            }
        }
    }
    return false;
}

inline bool MultiDtgPlus::FindHnode(const uint32_t &idx, const uint32_t &id, h_ptr &hp){
    if(idx < 0 || idx >= H_depot_.size()) return false;

    for(auto &h : H_depot_[idx]){
        if(h->id_ == id){
            hp = h;
            return true;
        }
    }
    return false;
}


// // inline bool MultiDtgPlus::SetConnectUAV(const uint32_t &h_id, const Eigen::Vector3d &h_pos, const uint8_t &UAV_id){
// //     h_ptr hn;
// //     if(FindHnode(h_pos, h_id, hn)){
// //         bool new_connect = false;
// //         for(auto &u_d : hn->uav_dist_){
// //             if(u_d.first == UAV_id){
// //                 new_connect = true;
// //                 break;
// //             }
// //         }
// //         if(new_connect){
// //             u_d.second = 
// //         }

// //     }
// //     return false;
// // }

// // inline bool MultiDtgPlus::EraseConnectUAV(const uint32_t &h_id, const Eigen::Vector3d &h_pos, const uint8_t &UAV_id){

// // }

// // inline bool MultiDtgPlus::FindFnode(const Eigen::Vector3d &center, const uint32_t &id, f_ptr &fp){
// //     int idx = GetVoxId(center);
// //     if(idx == -1) return false;
// //     else{
// //         for(auto &f : F_depot_[idx]){
// //             if(f->id_ == id){
// //                 fp = f;
// //                 return true;
// //             }
// //         }
// //         return false;
// //     }
// // }

// inline h_ptr MultiDtgPlus::CreateSwarmHnode(const Eigen::Vector3d &pos, const uint32_t &id){
//     h_ptr hnode = make_shared<H_node>();
//     hnode->pos_ = LRM_->GetStdPos(pos);
//     std::cout<<"id:"<<int(SDM_->self_id_)<< "\033[0;42m try create swarm h:"<<hnode->pos_.transpose()<<"  ;"<<pos.transpose()<<" \033[0m"<< std::endl;
//     int idx = GetVoxId(hnode->pos_ );
//     if(idx == -1) return NULL;
//     hnode->id_ = id;
//     std::cout << "\033[0;42m new id:"<<int(hnode->id_)<<"  "<<idx<<" \033[0m" <<hnode->pos_.transpose()<<"  ;"<<pos.transpose()<< std::endl;
//     H_depot_[idx].emplace_back(hnode);
//     H_list_.emplace_back(hnode);
//     return hnode;
// }

inline h_ptr MultiDtgPlus::CreateHnode(const Eigen::Vector3d &pos){
    h_ptr hnode = make_shared<H_node>();
    hnode->pos_ = LRM_->GetStdPos(pos);
    if(!LRM_->IsFeasible(pos)) return NULL;
    std::cout/*<<"id:"<<int(SDM_->self_id_)*/ << "\033[0;42m try create:"<<hnode->pos_.transpose()<<"  ;"<<pos.transpose()<<" \033[0m"<< std::endl;
    int idx = GetVoxId(hnode->pos_ );
    if(idx == -1) return NULL;

    hnode->id_ = cur_hid_;
    cur_hid_ += drone_num_;
    std::cout << "\033[0;42m new id:"<<int(hnode->id_)<<"  "<<idx<<" \033[0m"<< std::endl;
    H_depot_[idx].emplace_back(hnode);
    H_list_.emplace_back(hnode);
    // SDM_->SetDTGHn(hnode->id_, uint32_t(LRM_->GlobalPos2GlobalNodeId(pos)));
    hnode->h_flags_ |= 6;
    if(maintain_h_dist_) hnode->last_maintain_t_ = ros::WallTime::now().toSec() - 999999;
    return hnode;
}

// // inline f_ptr MultiDtgPlus::CreateSwarmFnode(const Eigen::Vector3d &pos, const uint32_t &id){
// //     if(id >= EROI_->EROI_.size())return NULL;
// //     // f_ptr f = make_shared<F_node>();
// //     // f->center_ = EROI_->EROI_[id].center_;
// //     // cout<<"get swarmf:"<<int(id)<<"      "<<f->center_.transpose()<<endl;
// //     // f->id_ = id;
// //     // int idx = GetVoxId(f->center_);
    
// //     // F_depot_[idx].emplace_back(f);
// //     return F_depot_[id];
// // }

// inline f_ptr MultiDtgPlus::CreateFnode(const Eigen::Vector3d &vp_pos, const uint32_t &f_id, const uint8_t &v_id){
//     f_ptr f = make_shared<FC_node>();
//     f->fid_ = f_id;
//     f->vid_ = v_id;
//     // EROI_->AddValidFnode(f_id, v_id);
//     F_depot_[f->fid_].emplace_back(f);
//     return f;
// }

inline bool MultiDtgPlus::CreateConnectFnode(const Eigen::Vector3d &vp_pos, const uint32_t &f_id, const uint8_t &vid, const double &el, h_ptr &hn){
    double l;
    list<Eigen::Vector3d> path;
    hfe_ptr edge;
    bool change_vp = false;
    int idx_old, idx;
    // ROS_WARN("CreateConnectFnode0");
    // cout<<"hn:"<<(hn == NULL)<<endl;
    // cout<<"hn id:"<<(hn->id_)<<endl;

    for(auto &hfe : hn->hf_edges_){
        // cout<<"hfe:"<<(hfe == NULL)<<endl;
        // cout<<"hfe->tail_n_:"<<(hfe->tail_n_ == NULL)<<endl;
        if(hfe->tail_n_->fid_ == f_id){
            if(el >= hfe->length_) return false; // exist a shorter path
            edge = hfe;
            change_vp = true;
            // cout<<"hfe->tail_n_->fid_:"<<hfe->tail_n_->fid_<<endl;
            // cout<<"hfe->tail_n_->vid_:"<<int(hfe->tail_n_->vid_)<<endl;

            Eigen::Vector3d vp_old = EROI_->EROI_[hfe->tail_n_->fid_].center_ + EROI_->vps_[hfe->tail_n_->vid_].head(3);
            idx_old = GetVoxId(vp_old);
            break;
        }
    }
    // cout<<"change_vp:"<<change_vp<<endl;
    // ROS_WARN("CreateConnectFnode1");

    if(LRM_->RetrieveHPath(vp_pos, path, l, true)){
        // ROS_WARN("CreateConnectFnode2");
        idx = GetVoxId(vp_pos);
        // ROS_WARN("CreateConnectFnode3");

        if(change_vp){
            // ROS_WARN("CreateConnectFnode4");
            for(auto fi = F_depot_[idx_old].begin(); fi != F_depot_[idx_old].end(); fi++){
                // if((*fi) == NULL) StopageDebug("CreateConnectFnode (*fi) == NULL");
                // if((*fi)->hf_edge_ == NULL) StopageDebug("CreateConnectFnode (*fi)->hf_edge_ == NULL");
                // if((*fi)->hf_edge_->head_n_ == NULL) StopageDebug("CreateConnectFnode (*fi)->hf_edge_->head_n_ == NULL");
                if((*fi)->fid_ == f_id && (*fi)->hf_edge_->head_n_ == hn){
                    if(vid != (*fi)->vid_){
                        EROI_->DeleteValidFnode(f_id, (*fi)->vid_);
                        EROI_->AddValidFnode(f_id, vid);
                        (*fi)->vid_ = vid;
                        F_depot_[idx].emplace_back((*fi));
                        F_depot_[idx_old].erase(fi);
                    }

                    edge->path_ = path;
                    edge->length_ = el;

                    return true;
                }
            }

            // cout<<"hn id:"<<hn->id_<<"   "<<f_id<<endl;

            for(auto &hfe : hn->hf_edges_){
                if(hfe->tail_n_->fid_ == f_id){
                    edge = hfe;
                    change_vp = true;
                    // cout<<"hfe->tail_n_->fid_:"<<hfe->tail_n_->fid_<<endl;
                    // cout<<"hfe->tail_n_->vid_:"<<int(hfe->tail_n_->vid_)<<endl;
        
                    Eigen::Vector3d vp_old = EROI_->EROI_[hfe->tail_n_->fid_].center_ + EROI_->vps_[hfe->tail_n_->vid_].head(3);
                    // cout<<"vp_old:"<<vp_old.transpose()<<endl;
                    // cout<<"EROI_->EROI_[hfe->tail_n_->fid_].center_ :"<<EROI_->EROI_[hfe->tail_n_->fid_].center_.transpose()<<endl;
                    // cout<<"hfe->tail_n_->fid_:"<<hfe->tail_n_->fid_<<endl;
                    idx_old = GetVoxId(vp_old);
                    // cout<<"idx_old:"<<idx_old<<endl;
                    break;
                }
            }

            for(auto fi = F_depot_[idx_old].begin(); fi != F_depot_[idx_old].end(); fi++){
                cout<<"fi:"<<(*fi)->fid_<<"  "<<int((*fi)->vid_)<<endl;
            }
            StopageDebug("CreateConnectFnode not find fi");
        }
        else{
            // ROS_WARN("CreateConnectFnode5");
            // cout<<"hn id:"<<hn->id_<<"   "<<f_id<<endl;
            // cout<<"fid:"<<f_id<<"  vid:"<<int(vid)<<endl;
            EROI_->AddValidFnode(f_id, vid);
            f_ptr f = make_shared<FC_node>();
            f->fid_ = f_id;
            f->vid_ = vid;
            F_depot_[idx].emplace_back(f);
            edge = make_shared<hfe>();
            edge->head_n_ = hn;
            edge->tail_n_ = f;
            edge->path_ = path;
            edge->length_ = el;
            edge->e_flag_ = 4;
            
            f->hf_edge_ = edge;
            if(hn->hf_edges_.empty()) hn->last_maintain_t_ = ros::WallTime::now().toSec() - 999999;
            hn->hf_edges_.emplace_back(edge);
        }
        // if((path.back() - vp_pos).norm() > 1.0){
        //     cout<<"vp:"<<vp_pos.transpose()<<endl;
        //     for(auto &p : path)
        //         cout<<"path:"<<p.transpose()<<endl;

        //     StopageDebug("CreateConnectFnode path failed");
        // }
        // ROS_WARN("CreateConnectFnode6");

        // hn->hf_edges_.emplace_back(f->hf_edge_);
        // hn->h_flags_ |= 2;
        // hn->pos_ = LRM_->GetStdPos(vp_pos);
        return true;
    }
    else{
        StopageDebug("CreateConnectFnode RetrieveHPath failed");
        return false;
    }
}


// inline f_ptr MultiDtgPlus::CreateFnode(const uint32_t &id, const Eigen::Vector3d &upbd, const Eigen::Vector3d &lowbd){
//     f_ptr f = make_shared<F_node>();
//     // f->pos_ = pos;
//     f->center_ = (upbd + lowbd) / 2;
//     f->id_ = id;
//     // f->upbd_ = upbd;
//     // f->lowbd_ = lowbd;
//     int idx = GetVoxId(f->center_);
    
//     F_depot_[idx].emplace_back(f);
//     return f;
// }

inline void MultiDtgPlus::BlockEdge(hhe_ptr &e){
    if(e == NULL) return;
    e->length_ = 2e5;
    e->e_flag_ &= 239;
}

inline void MultiDtgPlus::BlockEdge(hfe_ptr &e){
    if(e == NULL) return;
    e->length_ = 2e5;
    e->e_flag_ &= 239;
    f_ptr f;
    f = e->tail_n_;
}

inline void MultiDtgPlus::EraseEdge(hhe_ptr &e, const bool &broadcast){
    if(e == NULL) return;
    list<hhe_ptr>::iterator e_it;
    h_ptr h;
    h = e->head_n_;
    for(e_it = h->hh_edges_.begin(); e_it != h->hh_edges_.end(); e_it++){
        if((*e_it) == e) {
            h->hh_edges_.erase(e_it);
            // if(use_swarm_ && broadcast) SDM_->EraseDTGHHEdge(e->head_, e->tail_);
            break;
        }
    }
    h = e->tail_n_;
    for(e_it = h->hh_edges_.begin(); e_it != h->hh_edges_.end(); e_it++){
        if((*e_it) == e) {
            // if(use_swarm_ && broadcast) SDM_->EraseDTGHHEdge(e->head_, e->tail_);
            h->hh_edges_.erase(e_it);
            break;
        }
    }
}

inline void MultiDtgPlus::EraseEdge(hfe_ptr &e, const bool &broadcast){
    if(e == NULL) return;
    list<hfe_ptr>::iterator e_it;
    h_ptr h;
    f_ptr f;
    h = e->head_n_;
    f = e->tail_n_;
    for(e_it = h->hf_edges_.begin(); e_it != h->hf_edges_.end(); e_it++){
        if((*e_it) == e) {
            // ROS_WARN("erase1 h:%d f:%d",  h->id_, f->fid_);
            h->hf_edges_.erase(e_it);
            // for(auto fie = F_depot_[f->fid_].begin(); fie != F_depot_[f->fid_].end(); fie++){
            //     if((*fie) == e){
            //         F_depot_[f->fid_].erase(fie);
            //         break;
            //     }
            // }


            break;
        }
    }
    // if(use_swarm_ && broadcast) SDM_->EraseDTGHFEdge(f->id_);

    // f->vp_id_ = -1;
    f->hf_edge_ = NULL;
}

inline void MultiDtgPlus::ConnectHF(h_ptr &h, f_ptr &f, const int &v_id, list<Eigen::Vector3d> &path, list<Eigen::Vector3d> &path_old, double length){
    hfe_ptr e_ptr; 
    path_old.clear();
    // cout<<"ConnectHF h:"<<(h == NULL)<<endl;
    // cout<<"ConnectHF h id:"<<(h->id_)<<endl;
    // ROS_WARN("ConnectHF0");
    if(!GetEdge(h, f, e_ptr, true)) {
        // ROS_WARN("ConnectHF1");

        // if(f->hf_edge_ != NULL && length + 1e-3 > f->hf_edge_->length_) return; // dont change if the new path is longer
        h_ptr h_old = f->hf_edge_->head_n_;
        if(GetEdge(h, f, e_ptr, false)){
            if(e_ptr->length_ < length + 1e-3) return;
            // ROS_WARN("ConnectHF2");

            // change old fn to current fn
            EraseFnodeFromGraph(e_ptr->tail_n_->fid_, e_ptr->tail_n_->vid_, false);
            e_ptr->tail_n_ = f;
            f->hf_edge_ = e_ptr;
        }
        else{
            // ROS_WARN("ConnectHF3");

            if(length + 1e-3 > f->hf_edge_->length_) return; // change hn
            if(h->hf_edges_.empty()) h->last_maintain_t_ = ros::WallTime::now().toSec() - 999999;

            h->hf_edges_.emplace_back(f->hf_edge_);
        }
        for(list<hfe_ptr>::iterator e_it = h_old->hf_edges_.begin(); e_it != h_old->hf_edges_.end(); e_it++){
            if((*e_it)->tail_n_ == f){
                h_old->hf_edges_.erase(e_it);
                break;
            }
        }
        f->hf_edge_->head_n_ = h;
    }
    else{
        // ROS_WARN("ConnectHF4");

        if(length + 1e-3 > e_ptr->length_) return;
        if(e_ptr->e_flag_ & 4){
            path_old = e_ptr->path_;
        }
    }
    // e_ptr->head_ = h->id_;
    // e_ptr->tail_ = f->fid_;
    // EROI_->AddValidFnode(f->fid_, v_id);
    // f->hf_edge_->tail_n_ = f;
    f->hf_edge_->length_ = length;
    f->hf_edge_->path_ = path;

    // Eigen::Vector3d vp_pos = EROI_->EROI_[f->fid_].center_ + EROI_->vps_[f->vid_].head(3);
    // if((path.back() - vp_pos).norm() > 1.0){
    //     cout<<"vp:"<<vp_pos.transpose()<<endl;
    //     for(auto &p : path)
    //         cout<<"path:"<<p.transpose()<<endl;

    //     StopageDebug("ConnectHF path failed");
    // }

    // EROI_
    // if(f->vid_ == -1){
    //     ROS_ERROR("error vp_id");
    //     ros::shutdown();
    //     return;
    // }
    // f->hf_edge_ = e_ptr;
    // if(f->hf_edge_ == NULL) StopageDebug("ConnectHF e_ptr == NULL");
}

inline void MultiDtgPlus::ConnectHH(h_ptr &head, h_ptr &tail, list<Eigen::Vector3d> &path, list<Eigen::Vector3d> &path_old, double length){
    hhe_ptr e_ptr; 
    bool push_edge = false;
    path_old.clear();
    if(!GetEdge(head, tail, e_ptr)){
        e_ptr = make_shared<hhe>(); 
        push_edge = true;
    }
    else{
        if(e_ptr->e_flag_ & 4){
            path_old = e_ptr->path_;
        }
    }
    if(length >= e_ptr->length_) return;

    // e_ptr->head_ = head->id_;
    // e_ptr->tail_ = tail->id_;
    // cout<<"path size:"<<path.size()<<endl;
    // if((path.front() - head->pos_).norm() > 1e-3){
    //     cout<<"path.front():"<<path.front().transpose()<<endl;
    //     cout<<"path.back():"<<path.back().transpose()<<endl;
    //     cout<<"head->pos_:"<<head->pos_.transpose()<<endl;
    //     cout<<"tail->pos_:"<<tail->pos_.transpose()<<endl;
    //     StopageDebug("ConnectHH path head diff");
    // }
    // if((path.back() - tail->pos_).norm() > 1e-3){
    //     cout<<"path.front():"<<path.front().transpose()<<endl;
    //     cout<<"path.back():"<<path.back().transpose()<<endl;
    //     cout<<"head->pos_:"<<head->pos_.transpose()<<endl;
    //     cout<<"tail->pos_:"<<tail->pos_.transpose()<<endl;
    //     StopageDebug("ConnectHH path back diff");
    // }

    e_ptr->head_n_ = head;
    e_ptr->tail_n_ = tail;
    e_ptr->length_ = length;
    e_ptr->path_ = path;

    e_ptr->e_flag_ |= 16;

    if(push_edge){
        head->hh_edges_.emplace_back(e_ptr);
        tail->hh_edges_.emplace_back(e_ptr);
    }

    if(e_ptr->length_ < e_ptr->length_s_){
        e_ptr->e_flag_ |= 32;
        e_ptr->length_s_ = e_ptr->length_;
    }
}

inline bool MultiDtgPlus::ConnectSwarmHH(h_ptr &head, h_ptr &tail, list<Eigen::Vector3d> &path){
    return false;
    // hhe_ptr edge;
    // bool new_edge = true;
    // for(auto &e : head->hh_edges_){
    //     if(e->tail_ == tail->id_ || e->head_ == tail->id_){
    //         new_edge = false;
    //         edge = e;
    //         break;
    //     }
    // }

    // if(new_edge){
    //     edge = make_shared<hhe>(head, tail);
    // }

    // double length = 0;
    // list<Eigen::Vector3d>::iterator p1, p2;
    // p1 = path.begin();
    // p2 = path.begin();
    // p2++;
    // for(; p2 != path.end(); p2++, p1++){
    //     length += (*p1 - *p2).norm();
    // }
    // if(length + 1e-3 < edge->length_s_){
    //     if(!(edge->e_flag_ & 16)){
    //         edge->path_ = path;
    //     }
    //     if(length + 1e-3 < edge->length_ && LRM_->PathCheck(path)){
    //         edge->e_flag_ |= 16;
    //         edge->length_ = length;
    //     }
    //     edge->e_flag_ |= 32;
    //     edge->length_s_ = length;
    //     if(new_edge){
    //         head->hh_edges_.emplace_back(edge);
    //         tail->hh_edges_.emplace_back(edge);
    //     }
    //     return true;
    // }
    // else return false;
}

inline bool MultiDtgPlus::ConnectSwarmHF(h_ptr &head, f_ptr &tail, list<Eigen::Vector3d> &path, uint8_t &vp_id){
    return false;
    // bool new_e;
    // hfe_ptr edge;
    // if(tail->eroi_->f_state_ == 2) return false;
    // if(vp_id < 0 || vp_id >= tail->eroi_->local_vps_.size() || tail->eroi_->local_vps_[vp_id] == 2) return false;
    // if(tail->hf_edge_ != NULL && tail->hf_edge_->head_ == head->id_){
    //     new_e = false;
    //     edge = tail->hf_edge_;
    // }
    // else new_e = true;

    // if(new_e){
    //     if(tail->hf_edge_ == NULL)
    //         tail->hf_edge_ = make_shared<hfe>(head, tail);
    //     else{
    //         for(auto e_it = tail->hf_edge_->head_n_->hf_edges_.begin(); e_it != tail->hf_edge_->head_n_->hf_edges_.end(); e_it++){
    //             if((*e_it) == tail->hf_edge_) {
    //                 tail->hf_edge_->head_n_->hf_edges_.erase(e_it);
    //                 break;
    //             }
    //         }
    //         tail->hf_edge_->head_ = head->id_;
    //         tail->hf_edge_->head_n_ = head;
    //     }
    //     edge = tail->hf_edge_;
    // }

    // double length = 0;
    // list<Eigen::Vector3d>::iterator p1, p2;
    // Eigen::Vector3d vp_pos;
    // EROI_->GetVpPos(tail->id_, vp_id, vp_pos);
    // path.emplace_back(vp_pos);
    // p1 = path.begin();
    // p2 = path.begin();
    // p2++;
    // for(; p2 != path.end(); p2++, p1++){
    //     length += (*p1 - *p2).norm();
    // }

    // tail->eroi_->flags_.reset(3);
    
    // if((tail->f_flag_ & 2) && !(head->h_flags_ & 2)){
    //     head->h_flags_ |= 2;
    //     local_hn_->push_back({GetVoxId(head->pos_), head->id_});
    // }

    // if(!new_e && head->id_ != edge->head_){
    //     for(list<hfe_ptr>::iterator e_it = edge->head_n_->hf_edges_.begin(); e_it != edge->head_n_->hf_edges_.end(); e_it++){
    //         if((*e_it)->tail_ == tail->id_){
    //             edge->head_n_->hf_edges_.erase(e_it);
    //             break;
    //         }
    //     }
    //     head->hf_edges_.emplace_back(edge);
    // }
    // else if(new_e){
    //     head->hf_edges_.emplace_back(edge);
    // }

    // if((!(edge->e_flag_ & 16) || length + 1e-3 < edge->length_) && LRM_->PathCheck(path, true)){
    //     edge->e_flag_ |= 16;
    //     edge->length_ = length;
    //     edge->path_ = path;
    //     tail->vp_id_ = vp_id;
    //     if(tail->vp_id_ == -1){
    //         ROS_ERROR("error vp_id2");
    //         ros::shutdown();
    //         return false;
    //     }
    // }
    // else{
    //     return false;
    // }
    // return true;
}

inline bool MultiDtgPlus::CheckNode(const Eigen::Vector3d &pos, const h_ptr &h, const f_ptr &f){
    bool have_h = false;
    bool have_f = false;
    list<h_ptr> h_list;
    list<f_ptr> f_list;
    GetVox(pos, h_list, f_list);
    if(h != NULL){
        for(auto &h_it: h_list){
            if(h == h_it){
                have_h = true;
                break;
            }
        }
    }
    else have_h = true;

    if(f != NULL){
        for(auto &f_it: f_list){
            if(f == f_it){
                have_f = true;
                break;
            }
        }
    }
    else have_f = true;

    return (have_f && have_h);
}

// // inline int MultiDtgPlus::LoadHHEdgeMSG(swarm_exp_msgs::DtgHHEdge &e){
// //     Eigen::Vector3d head_p, tail_p;
// //     h_ptr h_s, t_s;
// //     list<Eigen::Vector3d> path;
// //     head_p = LRM_->IdtoPos(int(e.points_idx.front()));
// //     tail_p = LRM_->IdtoPos(int(e.points_idx.back()));
// //     if(!FindHnode(head_p, e.head_h_id, h_s) || !FindHnode(head_p, e.tail_h_id, t_s)){
// //         ROS_ERROR("LoadHHEdgeMSG fail");
// //         cout<<int(e.head_h_id)<<"success:"<<(h_s==NULL)<<FindHnode(e.head_h_id, h_s)<<endl;
// //         cout<<int(e.tail_h_id)<<"success:"<<(t_s==NULL)<<FindHnode(e.tail_h_id, t_s)<<endl;
// //         return 1;
// //     }

// //     for(auto &p : e.points_idx) path.emplace_back(LRM_->IdtoPos(int(p)));
// //     if(ConnectSwarmHH(h_s, t_s, path)) return 0;
// //     else return 2;
// // }

// // inline int MultiDtgPlus::LoadHFEdgeMSG(swarm_exp_msgs::DtgHFEdge &e){
// //     if(e.f_id >= F_depot_.size()) {
// //         cout<<int(e.f_id)<<endl;
// //         ROS_ERROR("LoadHFEdgeMSG fail1");
// //         return 1;
// //     }
// //     list<Eigen::Vector3d> path;
// //     f_ptr t_s = F_depot_[e.f_id];
// //     h_ptr h_s;
// //     Eigen::Vector3d head_p = LRM_->IdtoPos(int(e.points_idx.front()));
// //     if(!FindHnode(head_p, e.h_id, h_s) ){
// //         ROS_ERROR("LoadHFEdgeMSG fail2");
// //         cout<<int(e.h_id)<<"success:"<<(h_s==NULL)<<FindHnode(e.h_id, h_s)<<endl;
// //         return 1;
// //     }
// //     for(auto &p : e.points_idx) path.emplace_back(LRM_->IdtoPos(int(p)));
// //     if(ConnectSwarmHF(h_s, t_s, path)) return 0;
// //     else return 2;
// // }

// inline h_ptr MultiDtgPlus::LoadHNodeMSG(swarm_exp_msgs::DtgHNode &h){
//     h_ptr hnode;
//     if(FindHnode(LRM_->IdtoPos(int(h.h_id)), h.h_id, hnode)){
//         return hnode;
//     }
//     hnode = make_shared<H_node>();
//     hnode->pos_ = LRM_->IdtoPos(int(h.h_id));
//     std::cout << "\033[0;42m try swarm create:"<<hnode->pos_.transpose()<<"  ;"<<hnode->pos_ <<" \033[0m"<< std::endl;
//     int idx = GetVoxId(hnode->pos_ );
//     if(idx == -1) return NULL;

//     hnode->id_ = h.h_id;
//     std::cout << "\033[0;42m new swarmid:"<<int(hnode->id_)<<"  "<<idx<<" \033[0m" <<hnode->pos_.transpose()<< std::endl;
//     H_depot_[idx].emplace_back(hnode);
//     H_list_.emplace_back(hnode);
//     return hnode;
// }

// inline void MultiDtgPlus::SetTarget(double &g1, double &g2, double &bg, double &sbg,  
//     pair<uint32_t, uint8_t> &nt1,  pair<uint32_t, uint8_t> &nt2,
//     pair<uint32_t, uint8_t> &bt11,  pair<uint32_t, uint8_t> &bt12,  pair<uint32_t, uint8_t> &bt2,
//     bool &ff, bool &fs, bool &fb){
//     if(g1 > g2){
//         if(g1 > bg){
//             if(ff){
//                 bt2 = bt11;
//                 bt12 = bt11;
//                 fs = true;
//             }
//             fb = true;
//             bg = g1;
//             bt11 = nt1;
//             ff = true;
//         }
//         else if(g1 > sbg){
//             if(fb){
//                 bt2 = nt1;
//                 bt12 = bt11;
//             }
//             else{
//                 bt2 = nt1;
//             }
//         }
//     }
//     else{
//         if(g2 > bg){
//             if(ff){                
//                 bt2 = bt11;
//                 fs = true;
//             }
//             bt11 = nt1;
//             bt12 = nt2;
//             ff = true;
//             fb = false;
//         }
//         else if(g2 > sbg){
//             if(fb){
//                 bt2 = nt1;
//                 bt12 = bt11;
//             }
//             else{
//                 bt2 = nt1;
//             }
//         }
//     }

// }


inline double MultiDtgPlus::GetGainExp1(const Eigen::Vector4d &ps, const Eigen::Vector4d &vp, const Eigen::Vector3d &vs, const double &dist){
    Eigen::Vector3d dp = (vp.head(3) - ps.head(3)).normalized();
    Eigen::Vector3d dv = dp * v_max_ - vs.head(3);
    double a_cost = 0.0;
    double det_theta;
    // double omiga;
    double v1_max;

    if(vs.norm() < 0.01){
        a_cost = 0;
        v1_max = v_max_;
    }
    else{
        det_theta = acos(dp.dot(vs.normalized()))*2;
        double r = (vp.head(3) - ps.head(3)).norm() / sin(det_theta*0.5);
        double omiga = sqrt(a_max_ * lambda_a_ / r);
        v1_max = min(r * omiga, v_max_); 
        // omiga = a_max_ / vs.norm();
        a_cost = det_theta / omiga * acc_gain_;
        if(vs.head(3).norm() > v1_max){
            // a_cost += (vs.head(3).norm() - v1_max) / a_max_ * acc_gain_*0;
            // a_cost += (v_max_ - v1_max) / a_max_ * acc_gain_;
        }
        else{
            // a_cost += (v_max_ - vs.head(3).norm()) / a_max_ * acc_gain_;
        }
        // if(isnan(a_cost)){
        //     cout<<"det_theta:"<<det_theta<<endl;
        //     cout<<"r:"<<r<<endl;
        //     cout<<"omiga:"<<omiga<<endl;
        //     cout<<"v1_max:"<<v1_max<<endl;
        //     cout<<"acc_gain_:"<<acc_gain_<<endl;
        //     cout<<"a_cost:"<<a_cost<<endl;
        //     cout<<"vs:"<<vs.transpose()<<endl;
        //     cout<<"dp:"<<dp.transpose()<<endl;
        //     StopageDebug("????");
        // }

    }

    double t = max(dist / v1_max/*dv.norm() / a_max_ * acc_gain_*/, abs(EROI_->YawDiff(vp(3), ps(3))) / yv_max_ * yaw_gain_) + a_cost;

    // double t;// = max(dist / v_max_ + a_cost /*dv.norm() / a_max_ * acc_gain_*/, abs(EROI_->YawDiff(vp(3), ps(3))) / yv_max_ * yaw_gain_);
    // t = max(dist / v1_max + dv.norm() / a_max_ * acc_gain_, abs(EROI_->YawDiff(vp(3), ps(3))) / yv_max_ * yaw_gain_);
    return exp(-lambda_e_ * t);
}

inline double MultiDtgPlus::GetGainExp2(const Eigen::Vector4d &ps, const Eigen::Vector4d &vp1, 
    const Eigen::Vector3d &vs, const double &dist,
    const Eigen::Vector3d &vp2, const double &arc, const double &vm, const double &y2){
    Eigen::Vector3d dp = (vp1.head(3) - ps.head(3)).normalized();
    Eigen::Vector3d dv = dp * v_max_ - vs.head(3);

    double a_cost = 0.0;
    double det_theta, det_theta2;
    double v1_max;

    if(vs.norm() < 0.01){
        a_cost = 0;
        v1_max = v_max_;
    }
    else{
        det_theta = acos(dp.dot(vs.normalized()))*2;
        double r = (vp1.head(3) - ps.head(3)).norm() / sin(det_theta*0.5);
        double omiga = sqrt(a_max_ * lambda_a_ / r);
        v1_max = min(r * omiga, v_max_); 
        a_cost = det_theta / omiga * acc_gain_;
        if(vs.head(3).norm() > v1_max){
            // a_cost += (vs.head(3).norm() - v1_max) / a_max_ * acc_gain_*0;
            // a_cost += (v_max_ - v1_max) / a_max_ * acc_gain_;
        }
        else{
            // a_cost += (v_max_ - vs.head(3).norm()) / a_max_ * acc_gain_;
        }
    }

    double t = max(dist / v1_max/*dv.norm() / a_max_ * acc_gain_*/, abs(EROI_->YawDiff(vp1(3), ps(3))) / yv_max_ * yaw_gain_) + a_cost;
    double gain = exp(-lambda_e_ * t);
    dv = (vp2 - vp1.head(3)).normalized()*v_max_ - (vp1.head(3) - ps.head(3)).normalized()*v1_max;
    t += max(arc / v1_max , abs(EROI_->YawDiff(y2, vp1(3))) / yv_max_ * yaw_gain_);// + dv.norm() / (a_max_ * lambda_a_) * acc_gain_;
    gain += exp(-lambda_e_ * t);
    return gain;
}

inline void MultiDtgPlus::StopageDebug(string c){
    std::cout << "\033[0;32m "<<c<<" \033[0m" << std::endl;    
    ros::shutdown();
    getchar();
}

}
#endif

//                 @@@@@
//                 ##**#%@@
//           @@@@%*-:::--+%
//         @@%#*%#-::::=:.+@%
//        @#=:..-#-.:::+++#@%
//       @#:...:.:#-...:*@@%
//     @%=........-#:++=+%@%
//    @%- . =:.....+*-...:#@%
//    @%: . =#......#: ... +@%
//    @%:   -:.....:#..... :%@
//      @*: ...... -* ..... =@%
//       @%=:... . -= ..... +@%
//        @@*==--::--:...  .%@%
//        @+=++++=======--=*@%
//       @#++++==========+@@
//      @#++++====-=======*%
//     %*+++===--===----===+%
//    #*++===--=*%%%#=-=====*@
//  *++==--+%@@%%%@%*--====#@
// #*====-=#@        @#=--===%@
// #*+=--+##          @@*=--=+#@
// #*+++=*#              @%+=+++%
// #*++=+#*                #++++*#
// #*++=+#                  #=+++#
// #*+++#                   %*=++*#
// #++=*%                   #%+=++#
                            