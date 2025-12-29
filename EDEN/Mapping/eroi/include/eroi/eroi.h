#ifndef EROI_H_
#define EROI_H_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <list>
#include <memory>
#include <math.h>
#include <random>
// #include <octomap_world/octomap_manager.h>
#include <tr1/unordered_map>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <eroi/eroi_struct.h>
#include <block_map_lite/color_manager.h>
#include <block_map_lite/block_map_lite.h>
#include <lowres_map_lite/lowres_map_lite.h>
#include <swarm_data/swarm_data.h>


using namespace std;
using namespace Eigen;
using namespace EROIStruct;

class EroiGrid{
public: 
    EroiGrid(){};
    ~EroiGrid(){};
    void init(ros::NodeHandle &nh, ros::NodeHandle &nh_private);

    void AlignInit(const ros::NodeHandle &nh, 
        const ros::NodeHandle &nh_private,
        Eigen::Vector3d &origin, Eigen::Vector3i &block_size, 
        Eigen::Vector3i &block_num, Eigen::Vector3i &local_block_num);
    
    void SetColorManager(ColorManager &CM){CM_ = &CM;}
    void SetLowresMap(lowres_lite::LowResMap &LRM){LRM_ = &LRM;}
    // void SetSwarmDataManager(SwarmDataManager *SDM){SDM_ = SDM;}
    void SetMap(BlockMapLite &BM){BM_ = &BM;}

    // /**
    //  * @brief find exploration target
    //  * 
    //  * @param ps            start pose
    //  * @param vs            start vel
    //  * @param plan_res      plan result
    //  * @param path_stem     stem: path to the first viewpoint 
    //  * @param path_main     main: path to the second viewpoint, not ensure safety 
    //  * @param path_sub      sub: path to the second viewpoint or a safe place, ensure safety
    //  * @param path_norm     norm: only one target left, i.e., last viewpoint or viewpoint not in local, need long distance transfer
    //  * @param tar_stem      <eroi id, vp id>
    //  * @param tar_main      <eroi id, vp id>
    //  * @param tar_sub       <eroi id, vp id>
    //  * @param tar_norm      <eroi id, vp id>
    //  */
    // void FindFastExpTarget(const Eigen::Vector4d &ps, const Eigen::Vector4d &vs, uint8_t &plan_res, 
    //     vector<Eigen::Vector3d> &path_stem, vector<Eigen::Vector3d> &path_main, 
    //     vector<Eigen::Vector3d> &path_sub, vector<Eigen::Vector3d> &path_norm, 
    //     pair<uint32_t, uint8_t> &tar_stem, pair<uint32_t, uint8_t> &tar_main, 
    //     pair<uint32_t, uint8_t> &tar_sub, pair<uint32_t, uint8_t> &tar_norm);

    /**
     * @brief load new feasible lrm voxels, vps in them will be sampled in SampleSingleVpsCallback()
     * 
     * @param pts new feasible lrm voxels
     */
    void LoadNewFeasibleLrmVox(vector<Eigen::Vector3d> &pts);

    /**
     * @brief init motion primitive dictionary for cost evaluation 
     * 
     */
    void InitMotionDict();

    /**
     * @brief sample viewpoints inside this eroi
     * 
     * @param f_id eroi id
     */
    void SampleTargetEroi(uint32_t &f_id);

    /**
     * @brief Get the Local Valid Vps 
     * 
     * @param erois     <f_id, state> 
     * @param vps       <f_id, v_id> 
     * @param root_ids  root h nodes ids
     * @param distances distances to hns
     */
    void GetLocalValidVps(vector<pair<uint32_t, uint8_t>> &erois, vector<pair<uint32_t, uint8_t>> &vps, vector<uint32_t> &root_ids, vector<double> &distances);

    /**
     * @brief sample viewpoints of frontiers in exploring 
     * 
     */
    void SampleVps();

    /**
     * @brief remove a viewpoint 
     * 
     * @param f_id frontier id  
     * @param v_id viewpoint id
     */
    inline void RemoveVp(const int &f_id, const int &v_id, bool broad_cast = false);

    /**
     * @brief pos to idx
     * 
     * @param pos 
     * @return -1: invalid; else: valid idx
     */
    inline int Pos2Idx(Eigen::Vector3d &pos);

    /**
     * @brief idx to pos
     * 
     * @param idx  
     * @param pos the center position corresponding to the f_grid
     * @return true valid idx
     * @return false invalid idx
     */
    inline bool Idx2Pos(const int &idx, Eigen::Vector3d &pos);

    /**
     * @brief pos to idx
     * 
     * @param pos 
     * @return -1: invalid; else: valid idx
     */
    inline int Posi2Idx(Eigen::Vector3i &pos);

    /**
     * @brief idx to pos3i
     * 
     * @param idx 
     * @param pos 
     * @return true valid idx
     * @return false invalid idx
     */
    inline bool Idx2Posi(const uint32_t &idx, Eigen::Vector3i &pos);

    /**
     * @brief Get the viewpoint position
     * 
     * @param f_idx frontier index
     * @param v_id  viewpoint id 
     * @param v_pos  viewpoint pos (xyz)
     * @param must_alive  if true, requrie the viewpoint be exploring
     * @return true 
     * @return false 
     */
    inline bool GetVpPos(const int &f_idx, const int &v_id, Eigen::Vector3d &v_pos, bool must_exploring = true);

    /**
     * @brief Get the viewpoint
     * 
     * @param f_idx frontier index
     * @param v_id  viewpoint id 
     * @param v_pose  viewpoint pose (xyz_yaw)
     * 
     * @return true alive viewpoint
     * @return false dead viewpoint
     */
    inline bool GetVp(const int &f_idx, const int &v_id, Eigen::Vector4d &v_pose);

    /**
     * @brief Get the Vp object
     * 
     * @param f_center frontier center
     * @param v_id      viewpoint id 
     * @param v_pose    viewpoint pose (xyz_yaw)
     * @return true 
     * @return false 
     */
    inline bool GetVp(const Eigen::Vector3d &f_center, const int &v_id, Eigen::Vector4d &v_pose);

    /**
     * @brief sample viewpoints for exploration
     * 
     * @param poses the poses of tagets to be covered by the viewpoints
     */
    void SampleVps(list<Eigen::Vector3d> &poses);

    /**
     * @brief Set the corresponding frontier explored, dont broadcast
     * 
     * @param id id of the target frontier
     */
    inline void SetExplored(const int &id);

    /**
     * @brief Set the exploring frontier
     * 
     * @param dirs_state samplable dirs 
     * @param id         frontier id
     */
    inline void SetExploring(const vector<uint8_t> &dirs_states, const int &id);

    /**
     * @brief update frontier and viewpoints
     * 
     * @param pts newly registered points
     */
    void UpdateFrontier(const vector<Eigen::Vector3d> &pts);
    
    /**
     * @brief Get the grids inside a bounding box 
     * 
     * @param center 
     * @param box_scale 
     * @param f_list     
     */
    void GetAliveGridsBBX(const Eigen::Vector3d &center, const Eigen::Vector3d &box_scale, list<pair<int, list<pair<int, Eigen::Vector3d>>>> &f_list);

    /**
     * @brief check if a viewpoint is blocked or has little gain. This func is time-consuming, only for exploring viewpoint.
     * 
     * @param f_id          frontier id
     * @param v_id          viewpoint id
     * @param allow_unknown allow the viewpoint in unknown space
     * @return true         not blocked, has enough gain
     * @return false        blocked or has little gain
     */
    bool StrongCheckViewpoint(const int &f_id, const int &v_id, const bool &allow_unknown);
    bool StrongCheckViewpointDebug(const int &f_id, const int &v_id, const bool &allow_unknown);

    /**
     * @brief check motion feasibilities
     * 
     * @param fid           first explored eroi id
     * @param valid_motions valid motions
     */
    void FastMotionCheck(const uint32_t &fid, vector<vector<bool>> &valid_motions);

    /**
     * @brief Get the Reach Vps 
     * 
     * @param vp viewpoint pose
     * @param dp distance thresh
     * @param dy yaw thresh
     * @param reached_vps close viewpoints
     */
    void GetReachVps(const Eigen::Vector4d &vp, double dp, double dy, list<pair<uint32_t, uint8_t>> &reached_vps);

    void DebugViewpoint();
    void LoadVpLines(visualization_msgs::Marker &mk, Eigen::Vector4d &vp);

    double GetGain(const int &f_id, const int &vp_id);
    void SampleSingleVpsCallback();
    void DebugFunc();
    
    inline int Pos2Idx(const Eigen::Vector3d &pos);

    inline int IdCompress(const int &f_id, const int &v_id); 
    inline void AddShow(const int &f_id); 
    inline bool IdDecompress(const int &c_id, int &f_id, int &v_id); 
    inline bool ChangeOwner(const uint32_t &f_id, const uint8_t &n_owner, const double &n_dist, uint8_t &o_owner); 
    inline bool ClearOwner(const uint32_t &f_id); 
    inline double GetSensorRange(){return sensor_range_;}; 
    inline bool HaveAliveNeighbour(uint32_t f_id, Eigen::Vector3d dir);
    inline void YawNorm(double &yaw);
    inline double YawDiff(const double &yaw1, const double &yaw2);
    inline void AddValidFnode(const uint32_t &f_id, const uint8_t &v_id);
    inline bool DeleteValidFnode(const uint32_t &f_id, const uint8_t &v_id);
    inline bool CheckValidFnode(const uint32_t &f_id, const uint8_t &v_id);
    inline void GetMotionInfo(const int &i1, const int &i2, double &vm, double &arc, pair<uint8_t, Eigen::Vector3i> &vp2);
    inline void GetMotionPath(const Eigen::Vector3d &center, const int &i1, const int &i2, vector<Eigen::Vector3d> &path);
    inline bool InsideFov(const Eigen::MatrixX3d &R, const Eigen::Vector3d &dir);
    inline void GetCamFov(double &hor, double &ver){hor = cam_hor_, ver = cam_ver_;};
    inline bool InsideExpMap(const Eigen::Vector3d &pos);
    inline void LoadSampleVps(const int &eroi_id);

    void ShowGainDebug();
    void DebugShowAll();
    Eigen::Vector3d Robot_pos_; //for lazy sample
    bool sample_flag_;          
    int min_vp_num_;
    SensorType sensor_type_;
    vector<list<pair<Eigen::Vector3i, uint8_t>>> vp_dict_; // <list<eroi diff, vp_id>>
    vector<EroiNode> EROI_;
    // vector<uint32_t> dead_erois_;    
    vector<pair<uint32_t, uint8_t>> dead_fnodes_;    
    vector<Eigen::Vector4d> vps_;
    int samp_total_num_;
    Eigen::Vector3d origin_, up_bd_, low_bd_, exp_upbd_, exp_lowbd_, node_scale_;
    // for dtg update
    bool vp_update_, single_sample_, rough_sample_;
    
private:
    inline bool InsideMap(Eigen::Vector3i &pos);
    inline bool InsideMap(Eigen::Vector3d &pos);
    inline bool InsideMap(const Eigen::Vector3d &pos);
    inline int GetMotionId(const Eigen::Vector3d &center, const Eigen::Vector3d &p);
    inline Eigen::Vector3d MotionId2Pos(const Eigen::Vector3d &center, const int &id);

    void ExpandFrontier(const int &idx, const bool &local_exp);
    void InitGainRays(double geh);
    void InitVpDict();
    void SampleVps(list<int> &idxs, bool reset_flag = true);
    void SampleVps(list<Eigen::Vector3i> &posis);
    double GetGainDebug(const int &f_id, const int &vp_id);
    // inline int Pos2DirIdx(const Eigen::Vector3d &pos, const Eigen::Vector3d &center);
    void SampleVpsCallback(const ros::TimerEvent &e);
    void SampleSingleVpsCallback(const ros::TimerEvent &e);
    void LazySampleCallback(const ros::TimerEvent &e);
    void ShowVpsCallback(const ros::TimerEvent &e);
    void VisVps(list<Eigen::Vector4d> &poses);
    void Debug(list<Eigen::Vector3d> &pts, int id = 0);
    void Debug(list<int> &v_ids);
    void Debug(Eigen::Vector3d &pt);

    inline void StopageDebug(string c);

    Eigen::Vector3d Robot_size_;
    Eigen::Vector3i node_num_;
    double resolution_, obs_thresh_;
    double vp_thresh_, resample_duration_, sensor_range_;

    // int samp_free_thresh_;

    // Eigen::Vector3d sample_BOX_, sample_res_;

    int scan_count_;
    ros::Publisher show_pub_, debug_pub_;
    ros::Timer sample_timer_, sample_single_timer_, show_timer_, lazy_samp_timer_;
    //FOV down sample
    int FOV_h_num_, FOV_v_num_; 
    double cam_hor_, cam_ver_;
    double livox_ver_low_, livox_ver_up_;
    double ray_samp_dist1_, ray_samp_dist2_;
    vector<double> vp_sample_range_;
    double VpSampleVerRes_, VpSampleHorRes_;
    int VpSampleVerNum_;
    // vector<double> sample_dists_, sample_h_dirs_, sample_v_dirs_;
    // vector<double> sample_vdir_sins_, sample_vdir_coses_;
    // vector<double> sample_hdir_sins_, sample_hdir_coses_;
    vector<list<pair<list<Eigen::Vector3d>, list<pair<Eigen::Vector3d, double>>>>> gain_rays_;
    vector<list<pair<Eigen::Vector3d, double>>> gain_dirs_; //<<end pt of rays, raw gain>
    tr1::unordered_map<int, vector<bool>> single_sample_dict_;
    double v_max_, a_max_;

    // continuas exp planning
    vector<vector<vector<Eigen::Vector3d>>> motion_trajs_;
    vector<vector<double>> motion_vel_;
    vector<vector<double>> motion_length_;
    vector<vector<pair<uint8_t, Eigen::Vector3i>>> motion_covered_vps_; //

    tr1::unordered_map<int, list<pair<int, int>>> motion_block_dict_; // hash : relative pos idx, list<<vp origin, vp target>> 
    Eigen::Vector3d motion_p_origin_;
    Eigen::Vector3i motion_p_num_;

    //to be shown
    list<int> explored_frontiers_show_;
    list<int> exploring_frontiers_show_;

    //for viewpoints sampling
    list<int> exploring_frontiers_;

    ros::NodeHandle nh_, nh_private_;
    ColorManager *CM_;
    lowres_lite::LowResMap *LRM_;
    BlockMapLite *BM_;

    //swarm_data
    bool use_swarm_;
    // SwarmDataManager *SDM_;

};

inline void EroiGrid::RemoveVp(const int &f_id, const int &v_id, bool broad_cast){
    if(f_id < 0 || f_id >= EROI_.size() || v_id < 0 || v_id >= samp_total_num_) return;
    if(EROI_[f_id].local_vps_[v_id] == 2) return;

    EROI_[f_id].local_vps_[v_id] = 2;
    int alive_num = 0;
    for(auto &v : EROI_[f_id].local_vps_){
        if(v != 2) {
            alive_num++;
        }
    }
    bool kill_frontier = (alive_num < min_vp_num_);
    if(alive_num < min_vp_num_){
        // if(EROI_[f_id].f_state_ != 2 && use_swarm_ && !SDM_->is_ground_) {
        //     SDM_->SetDTGFn(f_id, EROI_[f_id].local_vps_, 1, false);
        //     if(use_swarm_ && !SDM_->is_ground_) BM_->SendSwarmBlockMap(f_id, false);
        // }
        EROI_[f_id].f_state_ = 2;
    }
    if(!EROI_[f_id].flags_ & 4){
        EROI_[f_id].flags_ |= 4;
        exploring_frontiers_show_.emplace_back(f_id);
    }
    if(!kill_frontier && !(EROI_[f_id].flags_ & 2)){
        EROI_[f_id].flags_ |= 2;
        exploring_frontiers_.emplace_back(f_id);
    }
    if(kill_frontier)
        ExpandFrontier(f_id, broad_cast);
    // if(use_swarm_ && !kill_frontier && broad_cast && !SDM_->is_ground_) {
    //     SDM_->SetDTGFn(f_id, EROI_[f_id].local_vps_, 0, true);
    // }
}

inline bool EroiGrid::InsideMap(Eigen::Vector3i &pos){
    if(pos(0) < 0 || pos(1) < 0 || pos(2) < 0 ||
        pos(0) >=  node_num_(0) || pos(1) >= node_num_(1) || pos(2) >= node_num_(2) )
        return false;
    return true;
}

inline bool EroiGrid::InsideMap(Eigen::Vector3d &pos){
    if(pos(0) < low_bd_(0) || pos(1) < low_bd_(1) || pos(2) < low_bd_(2) ||
        pos(0) >  up_bd_(0) || pos(1) > up_bd_(1) || pos(2) > up_bd_(2) )
        return false;
    return true;
}

inline bool EroiGrid::InsideMap(const Eigen::Vector3d &pos){
    if(pos(0) < low_bd_(0) || pos(1) < low_bd_(1) || pos(2) < low_bd_(2) ||
        pos(0) >  up_bd_(0) || pos(1) > up_bd_(1) || pos(2) > up_bd_(2) )
        return false;
    return true;
}

inline bool EroiGrid::InsideExpMap(const Eigen::Vector3d &pos){
    if(pos(0) < exp_lowbd_(0) || pos(1) < exp_lowbd_(1) || pos(2) < exp_lowbd_(2) ||
        pos(0) >  exp_upbd_(0) || pos(1) > exp_upbd_(1) || pos(2) > exp_upbd_(2) )
        return false;
    return true;
}

inline void EroiGrid::LoadSampleVps(const int &eroi_id){
    Eigen::Vector3d ns = LRM_->node_scale_;
    Eigen::Vector3d p;
    Eigen::Vector3i gid3, gid3n;
    int gidn;
    gid3(0) = floor((EROI_[eroi_id].center_(0) - origin_(0)) / node_scale_(0));
    gid3(1) = floor((EROI_[eroi_id].center_(1) - origin_(1)) / node_scale_(1));
    gid3(2) = floor((EROI_[eroi_id].center_(2) - origin_(2)) / node_scale_(2));
    ROS_WARN("LoadSampleVps0");
    
    int i = 0;// c = 0;
    for(p(2) = EROI_[eroi_id].down_(2) + ns(2) * 0.5; p(2) < EROI_[eroi_id].up_(2); p(2) += ns(2)){
        for(p(1) = EROI_[eroi_id].down_(1) + ns(1) * 0.5; p(1) < EROI_[eroi_id].up_(1); p(1) += ns(1)){
            for(p(0) = EROI_[eroi_id].down_(0) + ns(0) * 0.5; p(0) < EROI_[eroi_id].up_(0); p(0) += ns(0)){
                if(LRM_->IsFeasible(p)){
                    for(auto &vs : vp_dict_[i]){
                        gid3n = vs.first + gid3;
                        gidn = gid3n(2) * node_num_(1) * node_num_(0) + gid3n(1) * node_num_(0) + gid3n(0);
                        // if(63904 == eroi_id){
                        //     if(gidn == 63905){
                        //         cout<<int(vs.second)<<endl;
                        //     }
                        // }
                        if(gidn < 0 || gidn >= EROI_.size()) {
                            // if(gidn == 63905){
                            //     ROS_WARN("gidn < 0 || gidn >= EROI_.size()");
                            // }
                            continue;
                        }
                        if(!LRM_->InsideLocalMap(EROI_[gidn].center_)) {
                            // if(gidn == 63905){
                            //     ROS_WARN("!LRM_->InsideLocalMap(EROI_[gidn].center_)");
                            // }
                            continue;
                        }
                        if(EROI_[gidn].f_state_ >= 2 || EROI_[gidn].local_vps_[vs.second] == 2) {
                            // if(gidn == 63905){
                            //     ROS_WARN("EROI_[gidn].f_state_ >= 2 || EROI_[gidn].local_vps_[vs.second] == 2");
                            // }
                            continue;
                        }
                        auto ssdi = single_sample_dict_.find(gidn);
                        if(ssdi == single_sample_dict_.end()){
                            single_sample_dict_.insert({gidn, {}});
                            single_sample_dict_[gidn].resize(samp_total_num_, false);
                            single_sample_dict_[gidn][vs.second] = true;
                            // c++;
                            Eigen::Vector3d vp_pos;
                            vp_pos = EROI_[gidn].center_ + vps_[vs.second].head(3);
                        }
                        else{
                            ssdi->second[vs.second] = true;
                        }
                    }
                }
                i++;
            }
        }
    }
    // cout<<"c:"<<c<<endl;
}

inline int EroiGrid::Pos2Idx(Eigen::Vector3d &pos){
    if(InsideMap(pos)){
        Eigen::Vector3d dpos = pos - origin_;
        Eigen::Vector3i posid;
        posid.x() = floor(dpos.x() / node_scale_(0));
        posid.y() = floor(dpos.y() / node_scale_(1));
        posid.z() = floor(dpos.z() / node_scale_(2));
        return posid(2)*node_num_(0)*node_num_(1) + posid(1)*node_num_(0) + posid(0);
    }
    else{
        return -1;
    }
}


inline int EroiGrid::Pos2Idx(const Eigen::Vector3d &pos){
    if(InsideMap(pos)){
        Eigen::Vector3d dpos = pos - origin_;
        Eigen::Vector3i posid;
        posid.x() = floor(dpos.x() / node_scale_(0));
        posid.y() = floor(dpos.y() / node_scale_(1));
        posid.z() = floor(dpos.z() / node_scale_(2));
        return posid(2)*node_num_(0)*node_num_(1) + posid(1)*node_num_(0) + posid(0);
    }
    else{
        return -1;
    }
}

// inline int EroiGrid::Pos2DirIdx(const Eigen::Vector3d &pos, const Eigen::Vector3d &center){
//     double dir = atan2(pos(1) - center(1), pos(0) - center(0));
//     return int((dir + M_PI + 0.5 * samp_h_dir_) / (samp_h_dir_)) % samp_h_dir_num_;
// }


inline bool EroiGrid::Idx2Pos(const int &idx, Eigen::Vector3d &pos){
    if(idx >= 0 && idx < EROI_.size()){
        int x = idx % node_num_(0);
        int y = ((idx - x)/node_num_(0)) % node_num_(1);
        int z = ((idx - x) - y*node_num_(0))/node_num_(1)/node_num_(0);
        pos(0) = (double(x)+0.5)*node_scale_(0);
        pos(1) = (double(y)+0.5)*node_scale_(1);
        pos(2) = (double(z)+0.5)*node_scale_(2);
        return true;
    }
    else return false;
}

inline int EroiGrid::Posi2Idx(Eigen::Vector3i &pos){
    if(InsideMap(pos)){
        return pos(0) + pos(1) * node_num_(0) + pos(2) * node_num_(0) * node_num_(1); 
    }
    else{
        return -1; 
    }
}

inline bool EroiGrid::Idx2Posi(const uint32_t &idx, Eigen::Vector3i &pos){
    if(idx >= 0 && idx < EROI_.size()){
        int x = idx % node_num_(0);
        int y = ((idx - x)/node_num_(0)) % node_num_(1);
        int z = ((idx - x) - y*node_num_(0))/node_num_(1)/node_num_(0);
        pos(0) = x;
        pos(1) = y;
        pos(2) = z;
        return true;
    }
    else return false;
}

inline bool EroiGrid::GetVpPos(const int &f_idx, const int &v_id, Eigen::Vector3d &v_pos, bool must_exploring){
    if(f_idx < 0 || f_idx >= EROI_.size() || v_id < 0 || v_id >= samp_total_num_) return false;
    if(must_exploring && EROI_[f_idx].f_state_ != 1) return false;
    v_pos = vps_[v_id].head(3);
    v_pos += EROI_[v_id].center_;
    return true;
}


inline bool EroiGrid::GetVp(const int &f_idx, const int &v_id, Eigen::Vector4d &v_pose){
    if(f_idx < 0 || f_idx >= EROI_.size() || v_id < 0 || v_id >= samp_total_num_) return false;
    if(EROI_[f_idx].f_state_ != 1) return false;
    v_pose = vps_[v_id];
    v_pose.head(3) += EROI_[f_idx].center_;
    return true;
}

inline bool EroiGrid::GetVp(const Eigen::Vector3d &f_center, const int &v_id, Eigen::Vector4d &v_pose){
    if(v_id < 0 || v_id >= samp_total_num_) return false;
    int f_idx = Pos2Idx(f_center);
    if(f_idx == -1) return false;
    v_pose = vps_[v_id];
    v_pose.head(3) += EROI_[f_idx].center_;
    return true;
}

inline void EroiGrid::SetExplored(const int &id){
    EROI_[id].f_state_ = 2;
    if(!(EROI_[id].flags_ & 4)){
        explored_frontiers_show_.emplace_back(id);
        EROI_[id].flags_ |= 4;
    }
}

inline void EroiGrid::SetExploring(const vector<uint8_t> &dirs_states, const int &id){

}

inline int EroiGrid::IdCompress(const int &f_id, const int &v_id){
    if(f_id < 0 || f_id >= EROI_.size() || v_id < 0 || v_id >= samp_total_num_) return -1;
    return f_id * samp_total_num_ + v_id;
}

inline void EroiGrid::AddShow(const int &f_id){
    if(!(EROI_[f_id].flags_ & 4)){
        exploring_frontiers_show_.emplace_back(f_id);
        EROI_[f_id].flags_ |= 4;
    }
}

inline bool EroiGrid::IdDecompress(const int &c_id, int &f_id, int &v_id){
    v_id = c_id % samp_total_num_;
    f_id = (c_id + 0.1) / samp_total_num_;
    if(f_id < 0 || f_id >= EROI_.size()) return false;
    return true;
}

inline bool EroiGrid::ChangeOwner(const uint32_t &f_id, const uint8_t &n_owner, const double &n_dist, uint8_t &o_owner){
    if(f_id < 0 || f_id >= EROI_.size()) return false;
    o_owner = EROI_[f_id].owner_;
    if(EROI_[f_id].owner_dist_ > n_dist || EROI_[f_id].owner_ == 0){
        EROI_[f_id].owner_ = n_owner;
        EROI_[f_id].owner_dist_ = n_dist;
        if(!(EROI_[f_id].flags_ & 4)){
            exploring_frontiers_show_.push_back(f_id);
            EROI_[f_id].flags_ |= 4;
        }
        return true;
    }
    return false;
}

inline bool EroiGrid::ClearOwner(const uint32_t &f_id){
    if(f_id < 0 || f_id >= EROI_.size()) return false;
    EROI_[f_id].owner_ = 0;
    if(!(EROI_[f_id].flags_ & 4)){
        exploring_frontiers_show_.push_back(f_id);
        EROI_[f_id].flags_ |= 4;
    }
    return true;
}

inline bool EroiGrid::HaveAliveNeighbour(uint32_t f_id, Eigen::Vector3d dir){
    Eigen::Vector3i pi, p_it;
    int f_n;
    if(!Idx2Posi(f_id, pi)) return false;
    for(int dim = 0; dim < 3; dim++){
        p_it = pi;
        if(dir(dim) < 0) p_it(dim) = pi(dim) - 1;
        else p_it(dim) = pi(dim) + 1;
        f_n = Posi2Idx(p_it);
        if(f_n != -1 && EROI_[f_n].f_state_ != 2){
            return true;
        }
    }
    return false;
}

inline void EroiGrid::YawNorm(double &yaw){
    double yawn;
    int c = yaw / M_PI / 2;
    yawn = yaw - c * M_PI * 2;
    
    if(yawn < -M_PI) yawn += M_PI * 2;
    if(yawn > M_PI) yawn -= M_PI * 2;
    yaw = yawn;
    return;
}

inline double EroiGrid::YawDiff(const double &yaw1, const double &yaw2){
    double dy = yaw1 - yaw2;
    YawNorm(dy);
    return dy;
}

inline void EroiGrid::AddValidFnode(const uint32_t &f_id, const uint8_t &v_id){
    if(f_id < 0 || f_id >= EROI_.size() || v_id >= samp_total_num_) return;
    for(auto &v : EROI_[f_id].valid_vps_){
        if(v == v_id) return;
    }
    // cout<<"add valid:"<<f_id<<"  "<<int(v_id)<<endl;
    EROI_[f_id].valid_vps_.emplace_back(v_id);
}

inline bool EroiGrid::DeleteValidFnode(const uint32_t &f_id, const uint8_t &v_id){
    if(f_id < 0 || f_id >= EROI_.size() || v_id >= samp_total_num_) return false;
    for(auto it = EROI_[f_id].valid_vps_.begin(); it != EROI_[f_id].valid_vps_.end(); it++){
        if(*it == v_id){
            EROI_[f_id].valid_vps_.erase(it);
            // cout<<"delete valid:"<<f_id<<"  "<<int(v_id)<<endl;
            return true;
        }
    }
    return false;
}

inline bool EroiGrid::CheckValidFnode(const uint32_t &f_id, const uint8_t &v_id){
    if(f_id < 0 || f_id >= EROI_.size() || v_id >= samp_total_num_) return false;
    for(auto it = EROI_[f_id].valid_vps_.begin(); it != EROI_[f_id].valid_vps_.end(); it++){
        if(*it == v_id){
            return true;
        }
    }
    return false;
}

inline void EroiGrid::GetMotionInfo(const int &i1, const int &i2, double &vm, double &arc, pair<uint8_t, Eigen::Vector3i> &vp2){
    vp2 = motion_covered_vps_[i1][i2];
    vm = motion_vel_[i1][i2];
    arc = motion_length_[i1][i2];
}

inline void EroiGrid::GetMotionPath(const Eigen::Vector3d &center, const int &i1, const int &i2, vector<Eigen::Vector3d> &path){
    if(i1 < 0 || i1 >= motion_trajs_.size()){
        StopageDebug("GetMotionPath1");
    }
    if(i2 < 0 || i2 >= motion_trajs_[i1].size()){
        StopageDebug("GetMotionPath2");
    }
    // cout<<"i1:"<<i1<<" i2:"<<i2<<endl;
    path = motion_trajs_[i1][i2];

    // cout<<"path size:"<<path.size()<<endl;
    for(auto &p : path){
        p += center;
    }
}

inline void EroiGrid::StopageDebug(string c){
    std::cout << "\033[0;31m "<<c<<" \033[0m" << std::endl;    
    // ros::shutdown();
    getchar();
}

inline bool EroiGrid::InsideFov(const Eigen::MatrixX3d &R, const Eigen::Vector3d &dir){
    if(dir.norm() > sensor_range_) return false; 
    Eigen::Vector3d p = R.transpose() * dir;
    p.normalize();
    if(sensor_type_ == CAMERA){
        if(p(0) < cos(cam_hor_ * 0.5)) return false;
        if(p(1) > sin(cam_ver_ * 0.5)) return false;
    }
    else{
        // to be done
        return false;
    }
    return true;
}

inline int EroiGrid::GetMotionId(const Eigen::Vector3d &center, const Eigen::Vector3d &p){
    Eigen::Vector3d pd = p - center;
    int x = (p(0) - motion_p_origin_(0)) / LRM_->node_scale_(0);
    int y = (p(1) - motion_p_origin_(1)) / LRM_->node_scale_(1);
    int z = (p(2) - motion_p_origin_(2)) / LRM_->node_scale_(2);
    return x + y*motion_p_num_(0) + z*motion_p_num_(0)*motion_p_num_(1);
}

inline Eigen::Vector3d EroiGrid::MotionId2Pos(const Eigen::Vector3d &center, const int &id){

    int x = id % motion_p_num_(0);
    int y = ((id - x)/motion_p_num_(0)) % motion_p_num_(1);
    int z = ((id - x) - y*motion_p_num_(0))/motion_p_num_(1)/motion_p_num_(0);
    Eigen::Vector3d p = Eigen::Vector3d(x + 0.5, y + 0.5, z + 0.5).cwiseProduct(LRM_->node_scale_) + center + motion_p_origin_;
    return p;
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
                            