#ifndef RIPNEON_MAP_H_
#define RIPNEON_MAP_H_
#define sing_ang_ 1.539 // PI * 0.49

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <ros/ros.h>
#include <list>
#include <memory>
#include <thread>
#include <mutex>
#include <math.h>
#include <tr1/unordered_map>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <cv_bridge/cv_bridge.h>
#include <sys/types.h>
#include <dirent.h>

#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <RipNeon/ripneon_struct.h>
#include <RipNeon/raycast.h>
#include <data_statistics/computation_statistician.h>
#include <sys/resource.h>

using namespace std;
using namespace PointBlockStruct;

class RipNeon{
public:
    RipNeon() {};
    ~RipNeon() {SaveAll();};

    void Init(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
    void AlignInit(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private,     
        const Eigen::Vector3d &origin, const Eigen::Vector3i &block_size, 
        const Eigen::Vector3i &block_num, const Eigen::Vector3i &local_block_num);
    void SetDataStatistic(ComputationStatistician *CS){CS_ = CS;};

    /**
     * @brief update map
     * 
     * @param pcl lidar pts
     * @return true 
     * @return false sensor pose not in update bbx
     */
    bool InsertPts(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pcl);
    bool InsertPtsCast(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pcl);
    void InsertPtsDebug(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pcl);
    void LoadDebugDict(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pcl, tr1::unordered_map<uint64_t, bool> &dict);

    void SetPose(const geometry_msgs::Pose &pose);
    void SetMsgTime(const double &t){t_msg_ = t;};

    /**
     * @brief check the voxel accuracy
     * 
     * @param gt_path   ground truth map path
     * @param test_path test map path
     */
    void CheckAccuracy(string gt_path, string test_path);

    /**
     * @brief reset the block ptr, save blocks outside local area, read or init new blocks 
     * 
     * @param p opom position
     * @return true  reload map
     * @return false do nothing
     */
    bool ResetLocalMap(const Eigen::Vector3f &p);
    
    /**
     * @brief Get the Vox State: unknown/occupied/free
     * 
     * @param id 
     * @return VoxelState 
     */
    inline VoxelState GetVoxState(const uint64_t &id);

    /**
     * @brief Get the Vox State: unknown/occupied/free
     * 
     * @param id 
     * @return VoxelState 
     */
    inline VoxelState GetVoxState(const Eigen::Vector3i &id);

    /**
     * @brief Get the Vox State: unknown/occupied/free
     * 
     * @param id 
     * @return VoxelState 
     */
    inline VoxelState GetVoxState(const Eigen::Vector3f &id);

    /**
     * @brief Get the Vox State: unknown/occupied/free
     * 
     * @param id 
     * @return VoxelState 
     */
    inline VoxelState GetVoxState(const Eigen::Vector3d &id);

    /**
     * @brief Get the Vox State: unknown/occupied/free/frontier/out
     * 
     * @param pos 
     * @return VoxelState 
     */
    inline VoxelState GetVoxStateFrontier(const Eigen::Vector3f &pos);

    /**
     * @brief Get the Vox State: unknown/occupied/free/frontier/out
     * 
     * @param pos 
     * @return VoxelState 
     */
    inline VoxelState GetVoxStateFrontier(const Eigen::Vector3d &pos);
    inline VoxelState GetVoxStateFrontierDebug(const Eigen::Vector3d &pos);

    /**
     * @brief Get vox state, if out local ,return out
     * 
     * @param pos 
     * @return VoxelState 
     */
    inline VoxelState GetExpVoxState(const Eigen::Vector3d &pos);

    /**
     * @brief Get the Local Vox object
     * 
     * @param block_id block index 
     * @param vox_id   vox index
     * @param pos      queried pos
     * @return true: pos inisde local space
     * @return false: pos not inisde local space
     */
    inline bool GetLocalVox(uint32_t &block_id, uint16_t &vox_id, const Eigen::Vector3f &pos);

    /**
     * @brief check if the point is free in current Ray depth Image
     * 
     * @param p          check pos
     * @return true      free
     * @return false     occ
     */
    inline bool RdImgNotFree(const Eigen::Vector3f &p);

    /**
     * @brief check if the voxel is free in current Ray depth Image
     * 
     * @param bid       local block id
     * @param vid       voxel id
     * @return true 
     * @return false 
     */
    inline bool RdImgNotFree(const uint32_t &bid, const uint32_t &vid);

    /**
     * @brief check if the point is free in current Ray depth Image
     * 
     * @param p          check pos
     * @return true      free
     * @return false     occ
     */
    inline bool RdImgFree(const Eigen::Vector3f &p);

    /**
     * @brief check if the voxel is free in current Ray depth Image
     * 
     * @param bid       local block id
     * @param vid       voxel id
     * @return true 
     * @return false 
     */
    inline bool RdImgFree(const uint32_t &bid, const uint32_t &vid);

    inline bool RdImgFreeDebug(const Eigen::Vector3f &p);
    
    /**
     * @brief set void_
     * 
     */
    inline void TwoStepRdImgSetVoid();
    inline bool TwoStepRdImgNotFree(const Eigen::Vector3f &p);

    /**
     * @brief check if there is any occupied grid in the bbx 
     * 
     * @param pos box center 
     * @param bbx box size
     * @return true 
     * @return false 
     */
    inline bool PosBBXOccupied(const Eigen::Vector3d &pos, const Eigen::Vector3d &bbx);
    
    /**
     * @brief check if there is any non free grid in the bbx 
     * 
     * @param pos 
     * @param bbx 
     * @return true 
     * @return false 
     */
    inline bool PosBBXFree(const Eigen::Vector3d &pos, const Eigen::Vector3d &bbx);
    inline void GetCastLine(const Eigen::Vector3d &start, const Eigen::Vector3d &end, list<Eigen::Vector3d> &line);
    
    
    inline Eigen::Vector3f IdtoPos(const uint64_t &id);//
    inline Eigen::Vector3d IdtoPos3d(const uint64_t &id);//
    inline Eigen::Vector3i IdtoPos3i(const uint64_t &id);//
    inline bool InsideMap(const Eigen::Vector3i &pos);//
    inline bool InsideMap(const Eigen::Vector3f &pos);//
    inline bool InsideMap(const Eigen::Vector3d &pos);//
    inline bool InsideLocalMap(const Eigen::Vector3f &pos);//
    inline bool InsideLocalMap(const Eigen::Vector3d &pos);//
    inline bool InsideExpMap(const Eigen::Vector3d &pos);//

    // void LoadRayImage();


    void Inflate();
    inline uint64_t PostoId(const Eigen::Vector3f &pos);//dont check in side map, carefully use
    inline int GetBlockId(const Eigen::Vector3d &pos);//check

    // vector<pair<uint32_t, uint16_t>> new_occ_, delete_occ_, new_free_, new_frontier_, delete_frontier_;
    tr1::unordered_map<uint64_t, pair<uint8_t, uint16_t>> changed_pts_; // <pos id, <initial idx, changed times>>, initial idx: 0: unknown->free, 1: unknown->occ, 2: occ->free, 3: free->occ
    vector<Eigen::Vector3f> newly_register_idx_;
    float sensor_range_, sensor_range_2_, sensor_range_void_, sensor_range_void_2_;
    Eigen::Vector3f local_origin_, local_upbd_, local_lowbd_, local_scale_;
    Eigen::Vector3i local_block_num_, lbn_x_, local_origin_idx_, local_up_idx_; // block idx
    Eigen::Vector3i local_origin_v_idx_, local_up_v_idx_; // vox idx
    float resolution_, res_inv_; // voxel resolution
    int read_blk_num_;
private:

    /* Timer */ 
    void ShowMapCallback(const ros::TimerEvent &e);
    void StatisticV(const ros::TimerEvent &e);

    /* read write */ 
    void SaveAll();
    void SaveData(const uint32_t &bid, shared_ptr<PV_Block> &GB);
    void ReadData(const uint32_t &bid, shared_ptr<PV_Block> &GB);
    void SaveDataThread();
    void ReadDataThread();
    void InsertPts2Void(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pcl);

    inline uint8_t EncodeVoxel(const uint8_t *c);
    inline void DecodeVoxel(const uint8_t &d, uint8_t *c);

    inline uint64_t LocalId2GlobalId(const int &lid, const shared_ptr<PV_Block> &GB);
    inline uint64_t LocalId2GlobalId(const int &lid, const shared_ptr<PV_Block> &GB, const Eigen::Vector3i &bs, const Eigen::Vector3i &vn);


    inline Eigen::Vector3i Id2BlockIdx3(const int &id);
    inline Eigen::Vector3i Id2BlockIdx3(const uint32_t &id);
    // inline bool GetVox(int &block_id, int &vox_id, const Eigen::Vector3i &pos);   //return true: inside map; false: outside map
    // inline bool GetVox(int &block_id, int &vox_id, const Eigen::Vector3f &pos);
    inline bool CastPointInUpdate(pair<uint32_t, uint16_t> &idx, const Eigen::Vector3f &pos, // cast point in sensor range and update bbx
                                    const Eigen::Vector3f &updiff, const Eigen::Vector3f &lowdiff);

    // inline float GetVoxOdds(const Eigen::Vector3f &pos);//check if pos is in the block, dont check if pos is in the block
    // inline float GetVoxOdds(const int &id);//don't check, carefully use
    // inline float GetVoxOdds(const Eigen::Vector3f &pos, const shared_ptr<PV_Block> &GB);//don't check
    inline int IsFrontier(const pair<uint32_t, uint16_t> idx); // 0: not in update range; 1: frontier, 2: not frontier

    inline bool GetBlock3Id(const Eigen::Vector3f &pos, Eigen::Vector3i &blkid);//check
    inline int GetBlockId(const Eigen::Vector3f &pos);//check
    inline int GetBlockId(const Eigen::Vector3i &pos);//check, pos: block id/carefully use 
    inline int GetLocalBlockId(const Eigen::Vector3i &pos);//check, pos: global block id 
    inline int GetLocalBlockId(const Eigen::Vector3f &pos);//check, pos: global block id 
    inline int GetLocalBlockId(const Eigen::Vector3d &pos);//check, pos: global block id 
    inline int GlobalVox2LocalBlockId(const Eigen::Vector3i &pos);//check, pos: global block id 
    
    inline uint16_t GetVoxId(const Eigen::Vector3f &pos, const shared_ptr<PV_Block> &GB);//don't check, pos of world
    inline uint16_t GetVoxId(const Eigen::Vector3d &pos, const shared_ptr<PV_Block> &GB);//don't check, pos of world
    inline uint16_t GetVoxId(const Eigen::Vector3i &pos, const shared_ptr<PV_Block> &GB);//don't check, pos of world
    // inline Eigen::Vector3f Id2LocalPos(const shared_ptr<PV_Block> &GB, const int &id);
    inline Eigen::Vector3f Id2LocalPosDebug(const Eigen::Vector3i &PVB_or, uint16_t &vid);
    inline Eigen::Vector3f Id2LocalDiffPos(const uint16_t &vid);
    inline Eigen::Vector3f Id2LocalPos(const Eigen::Vector3i &PVB_or, const uint16_t &vid);
    inline Eigen::Vector3f Id2LocalPos(const Eigen::Vector3i &PVB_or, const uint32_t &vid);



    inline void BfsExpand(vector<pair<uint32_t, uint16_t>> &open_list, const pair<uint32_t, uint16_t> &idx, 
                            const Eigen::Vector3f &p, bool &om);//
    inline void BfsExpandDebug(vector<pair<uint32_t, uint16_t>> &open_list, const pair<uint32_t, uint16_t> &idx, Eigen::Vector3f &pt);//
    inline void GetRayEndInsideLocalAndGlobalMap(const Eigen::Vector3f &start, Eigen::Vector3f &end, bool &occ);

    inline void BfsNeighbours(vector<pair<uint32_t, uint16_t>> &open_list, const pair<uint32_t, uint16_t> &idx);//


    inline Eigen::Vector3i PostoId3(const Eigen::Vector3f &pos);

    inline void InitRdImg(RdImg &img, const uint16_t &h_size, const uint16_t &v_size);
    inline void ResetRdImg(RdImg &img);
    inline bool LoadRdImgPt(RdImg &img, const Eigen::Vector3f &p);
    inline bool Pos2ImgIdx(pair<uint16_t, uint16_t> &idx, const Eigen::Vector3f &p);
    inline void StopageDebug(string c);

    inline std_msgs::ColorRGBA Getcolor(const float &z);

    double GetMemory();
    /* debug */
    void DebugShow();
    void DebugShow(vector<Eigen::Vector3f> &occ, vector<Eigen::Vector3f> &delete_occ, vector<Eigen::Vector3f> &cam_occ);
    void DebugShow(vector<Eigen::Vector3f> &occ, vector<Eigen::Vector3f> &free, vector<Eigen::Vector3f> &free_cast, vector<Eigen::Vector3f> &unknown);
    void DebugShow(vector<Eigen::Vector3f> &read_blk, bool blk = true);
    void DebugShow(vector<uint32_t> &blks, vector<uint16_t> &vs);

    ros::NodeHandle nh_, nh_private_;
    ros::Publisher vox_pub_, pts_pub_, debug_pub_, stat_pub_, sen_ready_pub_;
    ros::Timer vis_timer_, stat_timer_;


    /* sensor param */

    // p = s_r * (p' - s_t)
    Eigen::Quaternionf sen_odom_rot_; // cam 2 world
    Eigen::Matrix3f sen_rot_; // cam 2 world
    Eigen::Matrix3f sen_rot_inv_; // world 2 cam 
    Eigen::Vector3f sen_tra_;

    string voxel_path_;
    Eigen::Vector3f origin_, blockscale_, bs_inv_, map_upbd_, map_lowbd_;
    Eigen::Vector3f update_upbd_, update_lowbd_, bline_margin_;
    Eigen::Vector3i block_size_, bs_x_, voxel_num_, block_num_, bn_x_; 
    
    list<pair<int, Eigen::Vector3i>> new_idxs_; // global idx, global idx3
    list<Eigen::Vector3i> out_idxs_; // global idx3

    float inflate_r_scale_;
    int inflate_r_size_;
    vector<Eigen::Vector3i> inflate_vec_;

    /* map updating params */

    // double pro_hit_;             //log(P(hit|occupied)/P(hit|free))
    float pro_miss_;           //log(P(miss|occupied)/P(miss|free))
    float thr_max_, thr_min_, free_thr_; //thr_min: min thresh of free, free_thr: for check unknown

    bool world_frame_pts_;
    uint8_t pts_num_; // n pts per voxel
    vector<Eigen::Vector3i> six_connect_vec_;
    vector<pair<int, int>> six_cross_iter_idx_;
    vector<pair<int, int>> six_iter_idx_; 
    

    string map_path_;
    bool have_pose_;

    /* points lidar image */
    RdImg ray_depth_img_;
    float dtheta_, dtheta_inv_, dpsi_, dpsi_inv_;
    bool void_initialized_;
    vector<vector<bool>> void_standard_; // for initialize
    vector<vector<int>> void_stat_; // get those void pixels
    int void_frame_num_, void_frame_thresh_, void_pts_thresh_;
    pair<uint16_t, uint16_t> idx_; // for two step rdimg void set
    // vector<Eigen::Vector3d> corners_; 

    /* map data & io*/
    vector<shared_ptr<PV_Block>> PVBs_;
    // thread pts_io_thread_;
    // tr1::unordered_map<uint32_t, shared_ptr<PV_Block>> pts_save_dict_; // for read write points
    mutex pts_io_mutex_, vox_io_mutex_;
    uint8_t thread_num_;

    /* visualization */
    vector<std_msgs::ColorRGBA> color_list_;
    float colorhsize_;
    float show_freq_;
    bool show_vox_, show_pts_, show_fontier_;
    // vector<uint32_t> show_blocks_;


    /* statistic */
    bool stat_;
    uint64_t stat_n_;
    Eigen::Vector3f stat_upbd_, stat_lowbd_;
    ComputationStatistician *CS_;
    struct rusage usage_;
    double t_msg_;
    int t_num_ = 0;
    double t_mean_ = 0.0;
};

inline VoxelState RipNeon::GetVoxState(const uint64_t &id){
    Eigen::Vector3i pos = IdtoPos3i(id);
    
    int blockid = GlobalVox2LocalBlockId(pos);
    if(blockid == -2) {
        return VoxelState::out;
    }
    else{
        float odds = PVBs_[blockid]->vox_[GetVoxId(pos, PVBs_[blockid])].log_odds_;
        if(odds > 0) return VoxelState::occupied;
        else if(odds < free_thr_) return VoxelState::unknown;
        else return VoxelState::free;
        // else return VoxelState::unknown;
    } 
}

inline VoxelState RipNeon::GetVoxState(const Eigen::Vector3i &id){
    int voxid = id(0) + id(1) * voxel_num_(0) + id(2) * voxel_num_(0) * voxel_num_(1);
    return GetVoxState(voxid);
}

inline VoxelState RipNeon::GetVoxState(const Eigen::Vector3f &pos){
    Eigen::Vector3i pi;
    pi(0) = (pos(0) - origin_(0)) * res_inv_;
    pi(1) = (pos(1) - origin_(1)) * res_inv_;
    pi(2) = (pos(2) - origin_(2)) * res_inv_;
    int blockid = GlobalVox2LocalBlockId(pi);
    if(blockid != -2){
        // if(blockid >= PVBs_.size()){
        //     cout<<"blockid:"<<blockid<<"     -"<<PVBs_.size()<<endl;
        //     StopageDebug("blockid >= PVBs_.size()");
        //     ros::shutdown();
        // }
        // if(GetVoxId(pi, PVBs_[blockid]) >= block_size_(0) * block_size_(1) * block_size_(2)){
        //     cout<<"vid:"<<GetVoxId(pi, PVBs_[blockid])<<endl;
        //     cout<<"null:"<<(PVBs_[blockid] == NULL)<<endl;
        //     cout<<"blockid:"<<blockid<<"     -"<<PVBs_.size()<<endl;
        //     cout<<"pos:"<<pos.transpose()<<endl;
        //     cout<<"or:"<<(origin_ + PVBs_[blockid]->origin_.cast<float>() * resolution_).transpose()<<endl;
        //     cout<<"origin:"<<(origin_).transpose()<<endl;
        //     cout<<"b origin:"<<(PVBs_[blockid]->origin_).transpose()<<endl;
        //     StopageDebug("GetVoxId(pos, PVBs_[blockid]) >= block_size_(0) * block_size_(1) * block_size_(2)");
        //     ros::shutdown();
        // }
        float odds = PVBs_[blockid]->vox_[GetVoxId(pi, PVBs_[blockid])].log_odds_;
        if(odds > 0) return VoxelState::occupied;
        else if(odds < free_thr_) return VoxelState::unknown;
        else return VoxelState::free;
    }
    else{
        return VoxelState::out;
    }
}

inline VoxelState RipNeon::GetVoxState(const Eigen::Vector3d &id){
    Eigen::Vector3f p(id(0), id(1), id(2));
    return GetVoxState(p);
}

inline VoxelState RipNeon::GetVoxStateFrontier(const Eigen::Vector3d &pos){
    Eigen::Vector3f p(pos(0), pos(1), pos(2));
    return GetVoxStateFrontier(p);
}

inline VoxelState RipNeon::GetVoxStateFrontier(const Eigen::Vector3f &pos){
    Eigen::Vector3i pi;
    pi(0) = (pos(0) - origin_(0)) * res_inv_;
    pi(1) = (pos(1) - origin_(1)) * res_inv_;
    pi(2) = (pos(2) - origin_(2)) * res_inv_;
    int blockid = GlobalVox2LocalBlockId(pi);

    if(blockid != -2){
        // cout<<"in f"<<endl;
        uint16_t v_id = GetVoxId(pi, PVBs_[blockid]);
        float odds = PVBs_[blockid]->vox_[v_id].log_odds_;
        if(odds > 0) return VoxelState::occupied;
        else if(odds < free_thr_) return VoxelState::unknown;
        else if(PVBs_[blockid]->vox_[v_id].PvFlags_ & 1) {
            // cout<<"frontier!!!!"<<endl;
            return VoxelState::frontier;
        }
        else return VoxelState::free;
    }
    else{
        // cout<<"out f"<<endl;
        return VoxelState::out;
    }
}

// inline VoxelState RipNeon::GetVoxStateFrontierDebug(const Eigen::Vector3d &pos){

// }

inline VoxelState RipNeon::GetExpVoxState(const Eigen::Vector3d &pos){
    if(!InsideExpMap(pos)) return VoxelState::out;
    Eigen::Vector3i pi;
    pi(0) = (pos(0) - origin_(0)) * res_inv_;
    pi(1) = (pos(1) - origin_(1)) * res_inv_;
    pi(2) = (pos(2) - origin_(2)) * res_inv_;
    int blockid = GlobalVox2LocalBlockId(pi);

    if(blockid != -2){
        shared_ptr<PV_Block> GB_ptr = PVBs_[blockid];
        float odds = GB_ptr->vox_[GetVoxId(pi, GB_ptr)].log_odds_;
        // cout<<odds<<"  "<<thr_min_<<endl;
        if(odds > 0) return VoxelState::occupied;
        else if(odds < 0 && odds > thr_min_ - 1e-3) return VoxelState::free;
        else return VoxelState::unknown;
    }
    else{
        return VoxelState::outlocal;
    }
}

inline bool RipNeon::GetLocalVox(uint32_t &block_id, uint16_t &vox_id, const Eigen::Vector3f &pos)
{
    if(InsideLocalMap(pos)){
        Eigen::Vector3i posid_l, bid_l, vid_l;
        posid_l(0) = (pos(0) - local_origin_(0)) * res_inv_;
        posid_l(1) = (pos(1) - local_origin_(1)) * res_inv_;
        posid_l(2) = (pos(2) - local_origin_(2)) * res_inv_;
        bid_l(0) = posid_l(0) / block_size_(0);
        bid_l(1) = posid_l(1) / block_size_(1);
        bid_l(2) = posid_l(2) / block_size_(2);
        block_id = bid_l(2)*lbn_x_(1) + bid_l(1)*lbn_x_(0) + bid_l(0);
        posid_l -= bid_l.cwiseProduct(block_size_);
        vox_id = posid_l(2) * bs_x_(1) + posid_l(1) * bs_x_(0) + posid_l(0);
        return true;
    }
    else {
        // cout<<"pos:"<<pos.transpose()<<endl;
        // StopageDebug("GetLocalVox");
        return false;
    }
}

inline bool RipNeon::RdImgNotFree(const Eigen::Vector3f &p){
    pair<uint16_t, uint16_t> idx;
    Eigen::Vector3f dp = sen_rot_inv_ * (p - sen_tra_);
    if(Pos2ImgIdx(idx, dp)){
        float d = ray_depth_img_.depth_[idx.first][idx.second].first;
        if(d < -0.01){
            return true;
        }

        float r = dp(0) * dp(0) + dp(1) * dp(1) + dp(2) * dp(2);
        if(r < d) return false;
        else return true;
    }
    return true;
}

inline bool RipNeon::RdImgNotFree(const uint32_t &bid, const uint32_t &vid){
    Eigen::Vector3i pi;
    pi(0) = vid % block_size_(0);
    pi(2) = vid / bs_x_(1);
    pi(1) = (vid - bs_x_(1) * pi(2)) / bs_x_(0);
    pi += PVBs_[bid]->origin_;
    Eigen::Vector3f p = origin_;
    p(0) += (pi(0) + 0.5) * resolution_;
    p(1) += (pi(1) + 0.5) * resolution_;
    p(2) += (pi(2) + 0.5) * resolution_;

    pair<uint16_t, uint16_t> idx;
    Eigen::Vector3f dp = sen_rot_inv_ * (p - sen_tra_);

    if(Pos2ImgIdx(idx, dp)){
        float d = ray_depth_img_.depth_[idx.first][idx.second].first;
        if(d < -0.01){
            return true;
        }

        float r = dp(0) * dp(0) + dp(1) * dp(1) + dp(2) * dp(2);
        if(r < d) return false;
        else return true;
    }
    return true;
}

inline bool RipNeon::RdImgFree(const uint32_t &bid, const uint32_t &vid){
    Eigen::Vector3i pi;
    pi(0) = vid % block_size_(0);
    pi(2) = vid / bs_x_(1);
    pi(1) = (vid - bs_x_(1) * pi(2)) / bs_x_(0);
    pi += PVBs_[bid]->origin_;
    Eigen::Vector3f p = origin_;
    p(0) += (pi(0) + 0.5) * resolution_;
    p(1) += (pi(1) + 0.5) * resolution_;
    p(2) += (pi(2) + 0.5) * resolution_;

    pair<uint16_t, uint16_t> idx;
    Eigen::Vector3f dp = sen_rot_inv_ * (p - sen_tra_);

    if(Pos2ImgIdx(idx, dp)){

        float d = ray_depth_img_.depth_[idx.first][idx.second].second;
        if(!ray_depth_img_.void_[idx.first][idx.second]){
            return false;
        }

        float r = dp(0) * dp(0) + dp(1) * dp(1) + dp(2) * dp(2);
        if(r < sensor_range_void_2_ || r < d) return true;
        else return false;
    }
    return false;
}

inline bool RipNeon::RdImgFree(const Eigen::Vector3f &p){
    pair<uint16_t, uint16_t> idx;
    Eigen::Vector3f dp = sen_rot_inv_ * (p - sen_tra_);
    if(Pos2ImgIdx(idx, dp)){
        // float d = ray_depth_img_.depth_[idx.first][idx.second].second;
        // if(d < -0.01){
        //     return false;
        // }

        // float r = dp(0) * dp(0) + dp(1) * dp(1) + dp(2) * dp(2);
        // if(r < d) return true;
        // else return false;

        float r = dp(0) * dp(0) + dp(1) * dp(1) + dp(2) * dp(2);
        if(!ray_depth_img_.void_[idx.first][idx.second]){
            float d = ray_depth_img_.depth_[idx.first][idx.second].second;
        
            if(r < d) return true;
            else return false;
        }
        else{
            if(r < sensor_range_void_2_) return true;
            else return false;
        }
    }
    return false;
}


inline bool RipNeon::RdImgFreeDebug(const Eigen::Vector3f &p){
    pair<uint16_t, uint16_t> idx;
    Eigen::Vector3f dp = sen_rot_inv_ * (p - sen_tra_);
    if(Pos2ImgIdx(idx, dp)){
        float d = ray_depth_img_.depth_[idx.first][idx.second].second;
        if(d < -0.01){
            return false;
        }

        float r = dp(0) * dp(0) + dp(1) * dp(1) + dp(2) * dp(2);
        // cout<<"idx.first"<<idx.first<<" idx.second:"<<int(idx.second)<<endl;
        // cout<<"p:"<<p.transpose()<<" dp:"<<dp.transpose()<<endl;
        // cout<<"r:"<<sqrt(r)<<" d:"<<sqrt(d)<<endl;
        if(r < d) return true;
        else return false;
    }
    return false;
}

inline uint8_t RipNeon::EncodeVoxel(const uint8_t *c){
    uint8_t d = 0;
    for(int i = 0; i < 4; i++){
        d = d * 4 + c[i];
    }
    return d;
}

inline void RipNeon::DecodeVoxel(const uint8_t &d, uint8_t *c){
    uint8_t dd = d;
    for(int i = 0; i < 4; i++){
        c[3 - i] = dd % 4;
        dd = dd / 4;
    }
}

inline uint64_t RipNeon::LocalId2GlobalId(const int &lid, const shared_ptr<PV_Block> &GB){
    uint64_t x = lid % block_size_(0);
    uint64_t y = ((lid - x)/block_size_(0)) % block_size_(1);
    uint64_t z = ((lid - x) - y*block_size_(0))/block_size_(1)/block_size_(0);
    x += GB->origin_(0);
    y += GB->origin_(1);
    z += GB->origin_(2);
    return x + y * voxel_num_(0) + z * voxel_num_(0) * voxel_num_(1);
}

inline uint64_t RipNeon::LocalId2GlobalId(const int &lid, const shared_ptr<PV_Block> &GB, const Eigen::Vector3i &bs, const Eigen::Vector3i &vn){
    uint64_t x = lid % bs(0);
    uint64_t y = ((lid - x)/bs(0)) % bs(1);
    uint64_t z = ((lid - x) - y*bs(0))/bs(1)/bs(0);
    x += GB->origin_(0);
    y += GB->origin_(1);
    z += GB->origin_(2);
    return x + y * vn(0) + z * vn(0) * vn(1);
}

inline Eigen::Vector3i RipNeon::Id2BlockIdx3(const int &id){
    Eigen::Vector3i p;
    p(0) = id % block_num_(0);
    p(1) = ((id - p(0))/block_num_(0)) % block_num_(1);
    p(2) = ((id - p(0)) - p(1)*block_num_(0))/bn_x_(1);
    return p;
}

inline Eigen::Vector3i RipNeon::Id2BlockIdx3(const uint32_t &id){
    Eigen::Vector3i p;
    p(0) = id % block_num_(0);
    p(1) = ((id - p(0))/block_num_(0)) % block_num_(1);
    p(2) = ((id - p(0)) - p(1)*block_num_(0))/bn_x_(1);
    return p;
}

// inline bool RipNeon::GetVox(int &block_id, int &vox_id, const Eigen::Vector3i &pos){
//     Eigen::Vector3f pos3d = pos.cast<double>() * resolution_;
//     return GetVox(block_id, vox_id, pos3d);
// }

// inline bool RipNeon::GetVox(int &block_id, int &vox_id, const Eigen::Vector3f &pos){
//     block_id = GetLocalBlockId(pos);
//     if(block_id >= 0){
//         // if(block_id>=PVBs_.size()){

//         //     Eigen::Vector3f dpos = pos - local_origin_;
//         //     Eigen::Vector3i posid;
//         //     posid.x() = floor(dpos.x() * bs_inv_.x());
//         //     posid.y() = floor(dpos.y() * bs_inv_.y());
//         //     posid.z() = floor(dpos.z() * bs_inv_.z());
//         //     cout<<posid.transpose()<<endl;
//         //     cout<<posid(2)*local_block_num_(0)*local_block_num_(1) + posid(1)*local_block_num_(0) + posid(0)<<endl;
//         //     cout<<"size:"<<PVBs_.size()<<endl;
//         //     ROS_ERROR("error --");
//         //     getchar();
//         // }
//         shared_ptr<PV_Block> GB_ptr = PVBs_[block_id];
//         vox_id = GetVoxId(pos, GB_ptr);
//         return true;
//     }
//     else{
//         return false;
//     }

// }

inline bool RipNeon::CastPointInUpdate(pair<uint32_t, uint16_t> &idx, const Eigen::Vector3f &pos,
                                                const Eigen::Vector3f &updiff, const Eigen::Vector3f &lowdiff){
    float amp_ = 1.0;
    bool iner_pt = true;
    Eigen::Vector3f dp = pos - sen_tra_;
    if(dp(0) * dp(0) + dp(1) * dp(1) + dp(2) * dp(2) > sensor_range_2_){
        iner_pt = false;
        dp = dp.normalized() * (sensor_range_ - 5e-3);
    }
    for(uint8_t dim = 0; dim < 3; dim++){
        if(dp(dim) > updiff(dim)){
            amp_ = min(updiff(dim) / dp(dim), amp_);
            iner_pt = false;
            continue;
        }
        if(dp(dim) < lowdiff(dim)){
            amp_ = min(lowdiff(dim) / dp(dim), amp_);
            iner_pt = false;
        }
    }
    Eigen::Vector3f dpos;
    if(!iner_pt) {
        dp = dp * amp_;
        dpos = dp + sen_tra_ - local_origin_;
    }
    else{
        dpos = pos - local_origin_;
    }
    Eigen::Vector3i posid_l, bid_l, vid_l;
    posid_l(0) = dpos(0) * res_inv_;
    posid_l(1) = dpos(1) * res_inv_;
    posid_l(2) = dpos(2) * res_inv_;
    bid_l(0) = posid_l(0) / block_size_(0);
    bid_l(1) = posid_l(1) / block_size_(1);
    bid_l(2) = posid_l(2) / block_size_(2);

    idx.first = bid_l(2)*lbn_x_(1) + bid_l(1)*lbn_x_(0) + bid_l(0);
    posid_l -= bid_l.cwiseProduct(block_size_);
    idx.second = posid_l(2) * bs_x_(1) + posid_l(1) * bs_x_(0) + posid_l(0);

    return iner_pt;
}

inline int RipNeon::IsFrontier(const pair<uint32_t, uint16_t> idx) // 0: not in update range; 1: frontier, 2: not frontier
{
    Eigen::Vector3f Pt = Id2LocalPos(PVBs_[idx.first]->origin_, idx.second);
    if(Pt(0) > update_upbd_(0) || Pt(0) < update_lowbd_(0) || Pt(1) > update_upbd_(1) || Pt(1) < update_lowbd_(1) 
            || Pt(2) > update_upbd_(2) || Pt(2) < update_lowbd_(2)){
        return 0;
    }
    vector<pair<uint32_t, uint16_t>> neighbours;
    BfsNeighbours(neighbours, idx);
    bool find_u = false;
    for(auto n : neighbours){
        // if(n.first >= PVBs_.size() || n.second >= PVBs_[n.first]->vox_.size()){
        //     cout<<"n:"<<n.first<<" "<<n.second<<endl;
        //     cout<<"idx:"<<idx.first<<" "<<idx.second<<endl;
        //     cout<<"Pt:"<<Pt.transpose()<<endl;
        //     cout<<"update_upbd_:"<<update_upbd_.transpose()<<endl;
        //     cout<<"update_lowbd_:"<<update_lowbd_.transpose()<<endl;
        //     StopageDebug("out frontier");
        // }
        if(PVBs_[n.first]->vox_[n.second].log_odds_ < free_thr_){
            find_u = true;
            break;
        }
    }
    // if(!find_u){
    //     cout<<"update_upbd_:"<<update_upbd_.transpose()<<endl;
    //     cout<<"update_lowbd_:"<<update_lowbd_.transpose()<<endl;
    //     cout<<"Pt:"<<Pt.transpose()<<endl;
    //     for(auto n : neighbours){
    //         cout<<"nei:"<<Id2LocalPos(PVBs_[n.first]->origin_, n.second).transpose()<<endl;
    //         cout<<"logodds:"<<PVBs_[n.first]->vox_[n.second].log_odds_<<endl;
    //     }
    //     StopageDebug("invalid frontier");
    //     return 2;    
    // }
    return 1;    
}

inline bool RipNeon::GetBlock3Id(const Eigen::Vector3f &pos, Eigen::Vector3i &blkid){//check
    // Eigen::Vector3i pos3;

    // cout<<"pos:"<<pos.transpose()<<endl;
    if(InsideMap(pos)){
        Eigen::Vector3f dpos = pos - origin_;
        blkid.x() = floor(dpos.x() * bs_inv_.x());
        blkid.y() = floor(dpos.y() * bs_inv_.y());
        blkid.z() = floor(dpos.z() * bs_inv_.z());
        return true;
    }
    else{
        return false;
    }
}

inline int RipNeon::GetBlockId(const Eigen::Vector3d &pos){
    if(InsideMap(pos)){
        Eigen::Vector3f dpos = pos.cast<float>() - origin_;
        Eigen::Vector3i posid;
        posid.x() = floor(dpos.x() * bs_inv_.x());
        posid.y() = floor(dpos.y() * bs_inv_.y());
        posid.z() = floor(dpos.z() * bs_inv_.z());
        return posid(2)*bn_x_(1) + posid(1)*bn_x_(0) + posid(0);
    }
    else{
        return -1;
    }
}

inline int RipNeon::GetBlockId(const Eigen::Vector3f &pos){//check
    if(InsideMap(pos)){
        Eigen::Vector3f dpos = pos - origin_;
        Eigen::Vector3i posid;
        posid.x() = floor(dpos.x() * bs_inv_.x());
        posid.y() = floor(dpos.y() * bs_inv_.y());
        posid.z() = floor(dpos.z() * bs_inv_.z());
        return posid(2)*bn_x_(1) + posid(1)*bn_x_(0) + posid(0);
    }
    else{
        return -1;
    }
}

inline int RipNeon::GetBlockId(const Eigen::Vector3i &pos){//check, pos: block id/carefully use 
    if(pos(0) < 0 || pos(1) < 0 || pos(2) < 0 ||
        pos(0) >=  block_num_(0) || pos(1) >= block_num_(1) || pos(2) >= block_num_(2)){
            return -1;
        }
    else{
        return pos(2)*bn_x_(1) + pos(1)*bn_x_(0) + pos(0);
    }
}

inline int RipNeon::GetLocalBlockId(const Eigen::Vector3i &pos){
    // if(!InsideMap(pos)) return -1;
    for(int dim = 0; dim < 3; dim++){
        if(pos(dim) < local_origin_idx_(dim) || pos(dim) > local_up_idx_(dim)){
            return -2;
        }
    }
    int id;
    Eigen::Vector3i p = pos - local_origin_idx_;
    return p(2)*lbn_x_(1) + p(1)*lbn_x_(0) + p(0);
}

inline int RipNeon::GetLocalBlockId(const Eigen::Vector3f &pos){
    for(int dim = 0; dim < 3; dim++){
        if(pos(dim) < local_lowbd_(dim) || pos(dim) > local_upbd_(dim)){
            return -2;
        }
    }
    int id, x, y, z;
    x = (pos(0) - local_origin_(0)) / blockscale_(0);
    y = (pos(1) - local_origin_(1)) / blockscale_(1);
    z = (pos(2) - local_origin_(2)) / blockscale_(2);
    // cout<<x<<"  y:"<<y<<" z:"<<z<<endl;

    return z*lbn_x_(1) + y*lbn_x_(0) + x;
}

inline int RipNeon::GetLocalBlockId(const Eigen::Vector3d &pos){
    for(int dim = 0; dim < 3; dim++){
        if(pos(dim) < local_lowbd_(dim) || pos(dim) > local_upbd_(dim)){
            return -2;
        }
    }
    int id, x, y, z;
    x = (pos(0) - local_origin_(0)) / blockscale_(0);
    y = (pos(1) - local_origin_(1)) / blockscale_(1);
    z = (pos(2) - local_origin_(2)) / blockscale_(2);
    // cout<<x<<"  y:"<<y<<" z:"<<z<<endl;

    return z*lbn_x_(1) + y*lbn_x_(0) + x;
}

inline int RipNeon::GlobalVox2LocalBlockId(const Eigen::Vector3i &pos){
    // if(!InsideMap(pos)) return -1;
    for(int dim = 0; dim < 3; dim++){
        if(pos(dim) < local_origin_v_idx_(dim) || pos(dim) > local_up_v_idx_(dim)){
            return -2;
        }
    }
    int id;
    Eigen::Vector3i p = pos - local_origin_v_idx_;
    p(0) /= block_size_(0);
    p(1) /= block_size_(1);
    p(2) /= block_size_(2);
    return p(2)*lbn_x_(1) + p(1)*lbn_x_(0) + p(0);
}

// inline uint16_t RipNeon::GetVoxId(const Eigen::Vector3f &pos, const shared_ptr<PV_Block> &GB){//don't check, pos of world
//     Eigen::Vector3f dpos = pos - origin_ - GB->origin_.cast<float>()*resolution_;
//     Eigen::Vector3i posid;
//     posid.x() = floor(dpos(0) * res_inv_);
//     posid.y() = floor(dpos(1) * res_inv_);
//     posid.z() = floor(dpos(2) * res_inv_);

//     return posid(2) * block_size_.x() * block_size_.y() + posid(1) * block_size_.x() + posid(0);
// }

// inline uint16_t RipNeon::GetVoxId(const Eigen::Vector3i &pos, const shared_ptr<PV_Block> &GB){//don't check, pos of world
//     Eigen::Vector3i dpos = pos - GB->origin_;
//     return dpos(2)*(block_size_.x())*(block_size_.y()) + dpos(1)*(block_size_.x()) + dpos(0);
// }

inline uint16_t RipNeon::GetVoxId(const Eigen::Vector3f &pos, const shared_ptr<PV_Block> &GB){//don't check, pos of world
    Eigen::Vector3f dpos = pos - origin_ - GB->origin_.cast<float>()*resolution_;
    Eigen::Vector3i posid;
    posid.x() = floor(dpos(0) / resolution_);
    posid.y() = floor(dpos(1) / resolution_);
    posid.z() = floor(dpos(2) / resolution_);

    return posid(2) * bs_x_(1) + posid(1) * bs_x_(0) + posid(0);
}

inline uint16_t RipNeon::GetVoxId(const Eigen::Vector3d &pos, const shared_ptr<PV_Block> &GB){//don't check, pos of world
    Eigen::Vector3f dpos = pos.cast<float>() - origin_ - GB->origin_.cast<float>()*resolution_;
    Eigen::Vector3i posid;
    posid.x() = floor(dpos(0) / resolution_);
    posid.y() = floor(dpos(1) / resolution_);
    posid.z() = floor(dpos(2) / resolution_);

    return posid(2) * bs_x_(1) + posid(1) * bs_x_(0) + posid(0);
}

inline uint16_t RipNeon::GetVoxId(const Eigen::Vector3i &pos, const shared_ptr<PV_Block> &GB){//don't check, pos of world
    Eigen::Vector3i dpos = pos - GB->origin_;
    return dpos(2)*bs_x_(1) + dpos(1)*bs_x_(0) + dpos(0);
}


inline Eigen::Vector3f RipNeon::Id2LocalPosDebug(const Eigen::Vector3i &PVB_or, uint16_t &vid){
    int xi, yi, zi, vi = vid;

    xi = vi % bs_x_(0);

    zi = vi / bs_x_(1);

    yi = (vi - bs_x_(1) * zi) / bs_x_(0);

    xi += PVB_or(0);
    yi += PVB_or(1);
    zi += PVB_or(2);
    Eigen::Vector3f pt((xi + 0.5f) * resolution_ + origin_(0), (yi + 0.5f) * resolution_ + origin_(1), (zi + 0.5f) * resolution_ + origin_(2));

    return pt;
}

inline Eigen::Vector3f RipNeon::Id2LocalDiffPos(const uint16_t &vid){
    int xi, yi, zi, vi = vid;
    xi = vi % bs_x_(0);
    zi = vi / bs_x_(1);
    yi = (vi - bs_x_(1) * zi) / bs_x_(0);
    return Eigen::Vector3f(xi + 0.5f, yi + 0.5f, zi + 0.5f) * resolution_;
}


inline Eigen::Vector3f RipNeon::Id2LocalPos(const Eigen::Vector3i &PVB_or, const uint16_t &vid){
    int xi, yi, zi, vi = vid;
    xi = vi % bs_x_(0);
    zi = vi / bs_x_(1);
    yi = (vi - bs_x_(1) * zi) / bs_x_(0);
    xi += PVB_or(0);
    yi += PVB_or(1);
    zi += PVB_or(2);
    Eigen::Vector3f pt((xi + 0.5f) * resolution_ + origin_(0), (yi + 0.5f) * resolution_ + origin_(1), (zi + 0.5f) * resolution_ + origin_(2));
    return pt;
}

inline Eigen::Vector3f RipNeon::Id2LocalPos(const Eigen::Vector3i &PVB_or, const uint32_t &vid){
    int xi, yi, zi;
    xi = vid % block_size_(0);
    zi = vid / bs_x_(1);
    yi = (vid - bs_x_(1) * zi) / bs_x_(0);
    xi += PVB_or(0);
    yi += PVB_or(1);
    zi += PVB_or(2);
    Eigen::Vector3f pt((xi + 0.5f) * resolution_ + origin_(0), (yi + 0.5f) * resolution_ + origin_(1), (zi + 0.5f) * resolution_ + origin_(2));
    return pt;
}

// inline Eigen::Vector3f RipNeon::Id2LocalPos(const shared_ptr<PV_Block> &GB, const int &id){
//     int x = id % block_size_(0);
//     int y = ((id - x)/block_size_(0)) % block_size_(1);
//     int z = ((id - x) - y*block_size_(0))/block_size_(1)/block_size_(0);
//     return Eigen::Vector3f((double(x)+0.5)*resolution_,(double(y)+0.5)*resolution_,(double(z)+0.5)*resolution_)+origin_ + GB->origin_.cast<double>() * resolution_;
// }

inline bool RipNeon::InsideMap(const Eigen::Vector3i &pos){
    // cout<<"dpos:"<<dpos.transpose()<<endl;
    if(pos(0) < 0 || pos(1) < 0 || pos(2) < 0 ||
        pos(0) >=  voxel_num_(0) || pos(1) >= voxel_num_(1) || pos(2) >= voxel_num_(2))
        return false;
    return true;
}

inline bool RipNeon::InsideMap(const Eigen::Vector3f &pos){
    if(pos(0) < map_lowbd_(0)|| pos(1) < map_lowbd_(1)|| pos(2) < map_lowbd_(2)||
        pos(0) >  map_upbd_(0) || pos(1) > map_upbd_(1) || pos(2) > map_upbd_(2) )
        return false;
    return true;
}

inline bool RipNeon::InsideMap(const Eigen::Vector3d &pos){
    if(pos(0) < map_lowbd_(0)|| pos(1) < map_lowbd_(1)|| pos(2) < map_lowbd_(2)||
        pos(0) >  map_upbd_(0) || pos(1) > map_upbd_(1) || pos(2) > map_upbd_(2) )
        return false;
    return true;
}


inline bool RipNeon::InsideLocalMap(const Eigen::Vector3f &pos){
    if(pos(0) < local_lowbd_(0)|| pos(1) < local_lowbd_(1)|| pos(2) < local_lowbd_(2)||
        pos(0) >  local_upbd_(0) || pos(1) > local_upbd_(1) || pos(2) > local_upbd_(2) )
        return false;
    return true;
}

inline bool RipNeon::InsideLocalMap(const Eigen::Vector3d &pos){
    if(pos(0) < local_lowbd_(0)|| pos(1) < local_lowbd_(1)|| pos(2) < local_lowbd_(2)||
        pos(0) >  local_upbd_(0) || pos(1) > local_upbd_(1) || pos(2) > local_upbd_(2) )
        return false;
    return true;
}

inline bool RipNeon::InsideExpMap(const Eigen::Vector3d &pos){
    if(pos(0) < stat_lowbd_(0)|| pos(1) < stat_lowbd_(1)|| pos(2) < stat_lowbd_(2)||
        pos(0) >  stat_upbd_(0) || pos(1) > stat_upbd_(1) || pos(2) > stat_upbd_(2) )
        return false;
    return true;
}

inline bool RipNeon::PosBBXOccupied(const Eigen::Vector3d &pos, const Eigen::Vector3d &bbx){
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

inline bool RipNeon::PosBBXFree(const Eigen::Vector3d &pos, const Eigen::Vector3d &bbx){
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

inline void RipNeon::GetCastLine(const Eigen::Vector3d &start, const Eigen::Vector3d &end, list<Eigen::Vector3d> &line){
    RayCaster rc;
    Eigen::Vector3d ray_iter;
    Eigen::Vector3d half_res = Eigen::Vector3d(0.5, 0.5, 0.5) * resolution_;
    line.clear();
    rc.setInput((start - origin_.cast<double>()) / resolution_, (end - origin_.cast<double>()) / resolution_);
    while (rc.step(ray_iter))
    {
        ray_iter = (ray_iter) * resolution_ + origin_.cast<double>() + half_res;
        line.emplace_back(ray_iter);
    }
}

inline void RipNeon::BfsExpand(vector<pair<uint32_t, uint16_t>> &open_list, const pair<uint32_t, uint16_t> &idx,
                                         const Eigen::Vector3f &p, bool &om){
    Eigen::Vector3i pi;
    pi(2) = idx.second / bs_x_(1);
    pi(1) = (idx.second - pi(2) * bs_x_(1)) / bs_x_(0);
    pi(0) = idx.second % block_size_(0);
    PointVox *vox;
    uint8_t d2, d21;

    pair<uint32_t, uint16_t> idx_n1, idx_n2;
    if(om){
        for(int dim = 0; dim < 3; dim++){
            // idx_n1 = idx;
            // idx_n2 = idx;
            d2 = dim *2;
            d21 = dim * 2 + 1;
            if(pi(dim) == 0){ // at block bottom edge
                idx_n1.first = idx.first + six_cross_iter_idx_[d2].first;
                idx_n1.second = idx.second + six_cross_iter_idx_[d2].second;
                idx_n2.first = idx.first + six_iter_idx_[d21].first;
                idx_n2.second = idx.second + six_iter_idx_[d21].second;
            }   
            else{
                if(pi(dim) + 1 == block_size_(dim)){ // at block upper edge
                    idx_n1.first = idx.first + six_iter_idx_[d2].first;
                    idx_n1.second = idx.second + six_iter_idx_[d2].second;
                    idx_n2.first = idx.first + six_cross_iter_idx_[d21].first;
                    idx_n2.second = idx.second + six_cross_iter_idx_[d21].second; 
                }
                else{ // at block inner 
                    idx_n1.first = idx.first + six_iter_idx_[d2].first;
                    idx_n1.second = idx.second + six_iter_idx_[d2].second;
                    idx_n2.first = idx.first + six_iter_idx_[d21].first;
                    idx_n2.second = idx.second + six_iter_idx_[d21].second;
                }
            }

            vox = &PVBs_[idx_n1.first]->vox_[idx_n1.second];
            if(!(vox->PvFlags_ & PV_FRONTIER_BFS) && vox->log_odds_ < free_thr_){
                open_list.emplace_back(idx_n1);
                vox->PvFlags_ |= PV_BFS;
            }

            vox = &PVBs_[idx_n2.first]->vox_[idx_n2.second];
            if(!(vox->PvFlags_ & PV_FRONTIER_BFS) && vox->log_odds_ < free_thr_){
                open_list.emplace_back(idx_n2);
                vox->PvFlags_ |= PV_BFS;
            }
        }
    }
    else{
        bool positive;
        for(int dim = 0; dim < 3; dim++){
            positive = (p(dim) - sen_tra_(dim) > 0);
            d2 = dim *2;
            d21 = dim * 2 + 1;
            if(pi(dim) == 0){ // at block bottom edge
                if(positive){
                    idx_n1.first = idx.first + six_cross_iter_idx_[d2].first;
                    idx_n1.second = idx.second + six_cross_iter_idx_[d2].second;
                }
                else{
                    idx_n2.first = idx.first + six_iter_idx_[d21].first;
                    idx_n2.second = idx.second + six_iter_idx_[d21].second;
                }
            }   
            else{
                if(pi(dim) + 1 == block_size_(dim)){ // at block upper edge
                    if(positive){
                        idx_n1.first = idx.first + six_iter_idx_[d2].first;
                        idx_n1.second = idx.second + six_iter_idx_[d2].second;
                    }
                    else{
                        idx_n2.first = idx.first + six_cross_iter_idx_[d21].first;
                        idx_n2.second = idx.second + six_cross_iter_idx_[d21].second; 
                    }
                }
                else{ // at block inner 
                    if(positive){
                        idx_n1.first = idx.first + six_iter_idx_[d2].first;
                        idx_n1.second = idx.second + six_iter_idx_[d2].second;
                    }
                    else{
                        idx_n2.first = idx.first + six_iter_idx_[d21].first;
                        idx_n2.second = idx.second + six_iter_idx_[d21].second;
                    }
                }
            }

            if(positive){
                vox = &PVBs_[idx_n1.first]->vox_[idx_n1.second];
                if(!(vox->PvFlags_ & PV_FRONTIER_BFS) && vox->log_odds_ < free_thr_){
                    open_list.emplace_back(idx_n1);
                    vox->PvFlags_ |= PV_BFS;
                }
            }
            else{
                vox = &PVBs_[idx_n2.first]->vox_[idx_n2.second];
                if(!(vox->PvFlags_ & PV_FRONTIER_BFS) && vox->log_odds_ < free_thr_){
                    open_list.emplace_back(idx_n2);
                    vox->PvFlags_ |= PV_BFS;
                }
            }
        }
    }
}

inline void RipNeon::BfsExpandDebug(vector<pair<uint32_t, uint16_t>> &open_list, const pair<uint32_t, uint16_t> &idx, Eigen::Vector3f &pt){
    Eigen::Vector3i p;
    p(2) = idx.second / bs_x_(1);
    p(1) = (idx.second - p(2) * bs_x_(1)) / bs_x_(0);
    p(0) = idx.second % block_size_(0);
    PointVox *vox;

    pair<uint32_t, uint16_t> idx_n1, idx_n2;
    for(int dim = 0; dim < 3; dim++){
        idx_n1 = idx;
        idx_n2 = idx;
        if(p(dim) == 0){ // at block bottom edge
            idx_n1.first += six_cross_iter_idx_[dim*2].first;
            idx_n1.second += six_cross_iter_idx_[dim*2].second;
            idx_n2.first += six_iter_idx_[dim*2 + 1].first;
            idx_n2.second += six_iter_idx_[dim*2 + 1].second;
        }   
        else{
            if(p(dim) + 1 == block_size_(dim)){ // at block upper edge
                idx_n1.first += six_iter_idx_[dim*2].first;
                idx_n1.second += six_iter_idx_[dim*2].second;
                idx_n2.first += six_cross_iter_idx_[dim*2 + 1].first;
                idx_n2.second += six_cross_iter_idx_[dim*2 + 1].second; 
            }
            else{ // at block inner 
                idx_n1.first += six_iter_idx_[dim*2].first;
                idx_n1.second += six_iter_idx_[dim*2].second;
                idx_n2.first += six_iter_idx_[dim*2 + 1].first;
                idx_n2.second += six_iter_idx_[dim*2 + 1].second;
            }
        }


        // if(idx_n1.first >= PVBs_.size() || idx_n1.second > PVBs_[idx_n1.first]->vox_.size()){
        //     cout<<"idx_n1:"<<idx_n1.first<<"  "<<idx_n1.second<<endl;
        //     cout<<"idx:"<<idx.first<<"  "<<idx.second<<endl;
        //     cout<<"pt:"<<Id2LocalPos(PVBs_[idx.first]->origin_, idx.second).transpose()<<endl;
        //     cout<<"upbd:"<<update_upbd_.transpose()<<endl;
        //     cout<<"lowbd:"<<update_lowbd_.transpose()<<endl;
        //     cout<<"pt:"<<pt.transpose()<<endl;
        //     StopageDebug("out bfs");
        // }

        vox = &PVBs_[idx_n1.first]->vox_[idx_n1.second];
        if(!(vox->PvFlags_ & PV_FRONTIER_BFS) && vox->log_odds_ < free_thr_){
            open_list.emplace_back(idx_n1);
            PVBs_[idx_n1.first]->vox_[idx_n1.second].PvFlags_ |= PV_BFS;
        }

        // if(idx_n2.first >= PVBs_.size() || idx_n2.second > PVBs_[idx_n2.first]->vox_.size()){
        //     cout<<"idx_n2:"<<idx_n2.first<<"  "<<idx_n2.second<<endl;
        //     cout<<"idx:"<<idx.first<<"  "<<idx.second<<endl;
        //     cout<<"pt:"<<Id2LocalPos(PVBs_[idx.first]->origin_, idx.second).transpose()<<endl;
        //     cout<<"upbd:"<<update_upbd_.transpose()<<endl;
        //     cout<<"lowbd:"<<update_lowbd_.transpose()<<endl;
        //     cout<<"pt:"<<pt.transpose()<<endl;
        //     StopageDebug("out bfs");
        // }

        vox = &PVBs_[idx_n2.first]->vox_[idx_n2.second];
        if(!(vox->PvFlags_ & PV_FRONTIER_BFS) && vox->log_odds_ < free_thr_){
            open_list.emplace_back(idx_n2);
            PVBs_[idx_n2.first]->vox_[idx_n2.second].PvFlags_ |= PV_BFS;
        }
    }
}

inline void RipNeon::GetRayEndInsideLocalAndGlobalMap(const Eigen::Vector3f &start, Eigen::Vector3f &end, bool &occ){
    Eigen::Vector3f upbd, lowbd;
    for(int dim = 0; dim < 3; dim++){
        upbd(dim) = min(local_upbd_(dim), map_upbd_(dim));
        lowbd(dim) = max(local_lowbd_(dim), origin_(dim));
    }

    float lx, ly, lz;
    if(end(0) > upbd(0)){
        lx = (upbd(0) - start(0)) / (end(0) - start(0)) - 2e-3f;
        occ = 0;
    }    
    else if(end(0) < lowbd(0)){
        lx = (start(0) - lowbd(0)) / (start(0) - end(0)) - 2e-3f;
        occ = 0;
    }    
    else lx = 1.0f;

    if(end(1) > upbd(1)){
        ly = (upbd(1) - start(1)) / (end(1) - start(1)) - 2e-3f;
        occ = 0;
    }    
    else if(end(1) < lowbd(1)){
        ly = (start(1) - lowbd(1)) / (start(1) - end(1)) - 2e-3f;
        occ = 0;
    }    
    else ly = 1.0f;

    if(end(2) > upbd(2)){
        lz = (upbd(2) - start(2)) / (end(2) - start(2)) - 2e-3f;
        occ = 0;
    }    
    else if(end(2) < lowbd(2)){
        lz = (start(2) - lowbd(2)) / (start(2) - end(2)) - 2e-3f;
        occ = 0;
    }    
    else lz = 1.0f;

    end = (end - start) * min(lx, min(ly, lz)) + start;
}

inline void RipNeon::BfsNeighbours(vector<pair<uint32_t, uint16_t>> &open_list, const pair<uint32_t, uint16_t> &idx){
    Eigen::Vector3i p;
    p(2) = idx.second / bs_x_(1);
    p(1) = (idx.second - p(2) * bs_x_(1)) / bs_x_(0);
    p(0) = idx.second % block_size_(0);

    pair<uint32_t, uint16_t> idx_n1, idx_n2;
    for(int dim = 0; dim < 3; dim++){

        if(p(dim) == 0){ // at block bottom edge
            idx_n1.first = idx.first + six_cross_iter_idx_[dim*2].first;
            idx_n1.second = idx.second + six_cross_iter_idx_[dim*2].second;
            idx_n2.first = idx.first + six_iter_idx_[dim*2 + 1].first;
            idx_n2.second = idx.second + six_iter_idx_[dim*2 + 1].second;
        }   
        else{
            if(p(dim) + 1 == block_size_(dim)){ // at block upper edge
                idx_n1.first = idx.first + six_iter_idx_[dim*2].first;
                idx_n1.second = idx.second + six_iter_idx_[dim*2].second;
                idx_n2.first = idx.first + six_cross_iter_idx_[dim*2 + 1].first;
                idx_n2.second = idx.second + six_cross_iter_idx_[dim*2 + 1].second; 
            }
            else{ // at block iner 
                idx_n1.first = idx.first + six_iter_idx_[dim*2].first;
                idx_n1.second = idx.second + six_iter_idx_[dim*2].second;
                idx_n2.first = idx.first + six_iter_idx_[dim*2 + 1].first;
                idx_n2.second = idx.second + six_iter_idx_[dim*2 + 1].second;
            }
        }
        open_list.emplace_back(idx_n1);
        open_list.emplace_back(idx_n2);
    }
}

inline Eigen::Vector3f RipNeon::IdtoPos(const uint64_t &id){
    int x = id % voxel_num_(0);
    int y = ((id - x)/voxel_num_(0)) % voxel_num_(1);
    int z = ((id - x) - y*voxel_num_(0))/voxel_num_(1)/voxel_num_(0);
    return Eigen::Vector3f((float(x)+0.5)*resolution_,(float(y)+0.5)*resolution_,(float(z)+0.5)*resolution_)+origin_;
}

inline Eigen::Vector3d RipNeon::IdtoPos3d(const uint64_t &id){
    int x = id % voxel_num_(0);
    int y = ((id - x)/voxel_num_(0)) % voxel_num_(1);
    int z = ((id - x) - y*voxel_num_(0))/voxel_num_(1)/voxel_num_(0);
    return Eigen::Vector3d((float(x)+0.5)*resolution_,(float(y)+0.5)*resolution_,(float(z)+0.5)*resolution_)+origin_.cast<double>();
}

inline Eigen::Vector3i RipNeon::IdtoPos3i(const uint64_t &id){
    int x = id % voxel_num_(0);
    int y = ((id - x)/voxel_num_(0)) % voxel_num_(1);
    int z = ((id - x) - y*voxel_num_(0))/voxel_num_(1)/voxel_num_(0);
    return Eigen::Vector3i(x, y, z);
}

inline uint64_t RipNeon::PostoId(const Eigen::Vector3f &pos){
    uint64_t x = (pos(0)-origin_(0))*res_inv_;
    uint64_t y = (pos(1)-origin_(1))*res_inv_;
    uint64_t z = (pos(2)-origin_(2))*res_inv_;
    return z*voxel_num_(0)*voxel_num_(1)+
        y*voxel_num_(0)+x;
}

inline Eigen::Vector3i RipNeon::PostoId3(const Eigen::Vector3f &pos){
    return Eigen::Vector3i((int)floor((pos(0) - origin_(0))*res_inv_), (int)floor((pos(1) - origin_(1))*res_inv_),
         (int)floor((pos(2) - origin_(2))*res_inv_)); 
}

// inline void RipNeon::InitRdImg(RdImg &img, const uint16_t &h_size, const uint16_t &v_size){
//     img.depth_.resize(v_size);
//     for(uint16_t i = 0; i < v_size; i++){
//         img.depth_[i].resize(h_size, {-1.0f, -1.0f});
//     }
// }

// inline void RipNeon::ResetRdImg(RdImg &img){
//     for (auto &occ_idx : img.points_idx_){
//         img.depth_[occ_idx.first][occ_idx.second].second = -1.0f;
//         img.depth_[occ_idx.first][occ_idx.second].first = -1.0f;
//     }
//     img.points_idx_.clear();
// }
inline bool RipNeon::TwoStepRdImgNotFree(const Eigen::Vector3f &p){
    Eigen::Vector3f dp = sen_rot_inv_ * (p - sen_tra_);
    if(Pos2ImgIdx(idx_, dp)){
        float d = ray_depth_img_.depth_[idx_.first][idx_.second].first;
        if(d < -0.01){
            return true;
        }

        float r = dp(0) * dp(0) + dp(1) * dp(1) + dp(2) * dp(2);
        if(r < d) return false;
        else return true;
    }
    return true;
}

inline void RipNeon::TwoStepRdImgSetVoid(){
    if(ray_depth_img_.void_[idx_.first][idx_.second]){
        ray_depth_img_.void_[idx_.first][idx_.second] = false;
        // ray_depth_img_.void_idx_.emplace_back(idx_);
    }
}

inline void RipNeon::InitRdImg(RdImg &img, const uint16_t &h_size, const uint16_t &v_size){
    img.depth_.resize(v_size);
    void_standard_.resize(v_size);
    void_stat_.resize(v_size);
    // img.void_.resize(v_size);
    for(uint16_t i = 0; i < v_size; i++){
        img.depth_[i].resize(h_size, {-1.0f, -1.0f});
        void_standard_[i].resize(h_size, false);
        void_stat_[i].resize(h_size, 0);
        // img.void_[i].resize(h_size, true);
    }
}

inline void RipNeon::ResetRdImg(RdImg &img){
    for (auto &occ_idx : img.points_idx_){
        img.depth_[occ_idx.first][occ_idx.second].second = -1.0f;
        img.depth_[occ_idx.first][occ_idx.second].first = -1.0f;
    }
    img.points_idx_.clear();

    img.void_ = void_standard_;
    // for (auto &void_idx : img.void_idx_){
    //     img.void_[void_idx.first][void_idx.second] = true;
    // }
    // img.void_idx_.clear();
}

inline bool RipNeon::LoadRdImgPt(RdImg &img, const Eigen::Vector3f &p){
    pair<uint16_t, uint16_t> idx;
    if(Pos2ImgIdx(idx, p)){
        float r = p(0) * p(0) + p(1) * p(1) + p(2) * p(2);
        float d1 = img.depth_[idx.first][idx.second].first;
        if(d1 < -0.01){
            img.depth_[idx.first][idx.second].first = min(r, sensor_range_2_);
            img.depth_[idx.first][idx.second].second = img.depth_[idx.first][idx.second].first;
            img.points_idx_.emplace_back(idx.first, idx.second);
        }
        else{
            r = min(r, sensor_range_2_);
            if(r < img.depth_[idx.first][idx.second].first){
                img.depth_[idx.first][idx.second].first = r;
            }
            if(r > img.depth_[idx.first][idx.second].second)
                img.depth_[idx.first][idx.second].second = r;
        }        
        return true;
    }
    return false;
}

inline bool RipNeon::Pos2ImgIdx(pair<uint16_t, uint16_t> &idx, const Eigen::Vector3f &p){
    float theta, psi;
    psi = atan2(p(2), sqrt(p(0) * p(0) + p(1) * p(1)));
    if(abs(psi) > sing_ang_) return false; // singular point
    
    psi += M_PI_2;
    theta = atan2(p(1), p(0)) + M_PI;
    
    idx.first = dpsi_inv_ * psi;
    idx.second = dtheta_inv_ * theta;
    return true;
}

inline std_msgs::ColorRGBA RipNeon::Getcolor(const float &z){
    std_msgs::ColorRGBA color;
    color.a = 1.0;
    if(z > map_upbd_(2)){
        return color_list_.back();
    }
    else if(z < origin_(2)){
        return color_list_.front();
    }
    else{
        float difz = z - origin_(2);
        int hieghtf = floor(difz / colorhsize_);
        int hieghtc = hieghtf + 1;
        float gain = (difz - colorhsize_*hieghtf)/colorhsize_;
        color.r = color_list_[hieghtf].r*(1.0-gain) + color_list_[hieghtc].r*gain;
        color.g = color_list_[hieghtf].g*(1.0-gain) + color_list_[hieghtc].g*gain;
        color.b = color_list_[hieghtf].b*(1.0-gain) + color_list_[hieghtc].b*gain;
    }
    return color;
}

inline void RipNeon::StopageDebug(string c){
    std::cout << "\033[0;31m "<<c<<" \033[0m" << std::endl;    
    // ros::shutdown();
    getchar();
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
                            