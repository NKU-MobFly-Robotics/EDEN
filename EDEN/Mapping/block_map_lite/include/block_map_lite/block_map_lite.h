#ifndef BLOCK_MAP_LITE_H_
#define BLOCK_MAP_LITE_H_
#include <dirent.h>

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
// #include <octomap_world/octomap_manager.h>
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

#include <block_map_lite/raycast.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <block_map_lite/mapping_struct.h>
#include <exp_comm_msgs/MapC.h>
#include <exp_comm_msgs/MapReqC.h>

#include <swarm_data/swarm_data.h>
#include <data_statistics/computation_statistician.h>



using namespace std;
using namespace BlockMapStruct;



class BlockMapLite{
public:
    BlockMapLite() {};
    ~BlockMapLite() {if(use_swarm_) delete swarm_filter_dict_;};

    void init(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
    void SetDataStatistic(ComputationStatistician *CS){CS_ = CS;};
    void AlignInit(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private,     
        const Eigen::Vector3d &origin, const Eigen::Vector3i &block_size, 
        const Eigen::Vector3i &block_num, const Eigen::Vector3i &local_block_num);
        
    /**
     * @brief initialize swarm block
     * 
     * @param id_l  block id list
     * @param bound <up, low> 
     */
    void InitSwarmBlock(vector<uint16_t> &id_l, vector<pair<Eigen::Vector3d, Eigen::Vector3d>> &bound);

    // void SetSwarmDataManager(SwarmDataManager *SDM){SDM_ = SDM;};
    /**
     * @brief check if voxes inside the bbx centered at pos contains any occpied vox 
     * 
     * @param pos center
     * @param bbx boundingbox
     * @return true 
     * @return false 
     */
    bool PosBBXOccupied(const Eigen::Vector3d &pos, const Eigen::Vector3d &bbx);

    /**
     * @brief check if voxes inside the bbx are all free
     * 
     * @param pos center
     * @param bbx boudingbox
     * @return true 
     * @return false 
     */
    bool PosBBXFree(const Eigen::Vector3d &pos, const Eigen::Vector3d &bbx);
    
    /**
     * @brief Get the Vox State: unknown/occupied/free/out/outlocal
     * 
     * @param id 
     * @return VoxelState 
     */
    inline VoxelState GetVoxState(const int &id);

    /**
     * @brief Get the Vox State: unknown/occupied/free/out/outlocal
     * 
     * @param id 
     * @return VoxelState 
     */
    inline VoxelState GetVoxState(const Eigen::Vector3i &id);

    /**
     * @brief Get the Vox State: unknown/occupied/free/out/outlocal
     * 
     * @param id 
     * @return VoxelState 
     */
    inline VoxelState GetVoxState(const Eigen::Vector3d &id);

    /**
     * @brief Get the Exp Vox State, the voxel outside exp space will be marked as out: unknown/occupied/free/out/outlocal
     * 
     * @param id 
     * @return VoxelState 
     */
    inline VoxelState GetExpVoxState(const Eigen::Vector3d &id);

    /**
     * @brief update map using pcl
     * 
     * @param pcl           point cloud
     */
    void InsertPcl(const sensor_msgs::PointCloud2ConstPtr &pcl);


    /**
     * @brief update map using depth img
     * 
     * @param depth         depth image
     */
    void InsertImg(const sensor_msgs::ImageConstPtr &depth);

    /**
     * @brief update map through cast method
     * 
     * @param depth 
     */
    void CastInsertImg(const sensor_msgs::ImageConstPtr &depth);

    /**
     * @brief set current robot pose
     * 
     * @param odom 
     */
    void OdomCallback(const nav_msgs::OdometryConstPtr &odom);

    // void SendSwarmBlockMap(const int &f_id, const bool &send_now);

    void VisMapAll(const std_msgs::Empty &e);

    /**
     * @brief reset the block ptr, save blocks outside local area, read or init new blocks 
     * 
     * @param p opom position
     */
    void ResetLocalMap(const Eigen::Vector3d &p);



    void ChangePtsDebug();
    // /**
    //  * @brief Visualize total map
    //  * 
    //  */
    // void VisTotalMap();
    
    // /**
    //  * @brief load pcl
    //  * 
    //  * @param occ_path  path of occ pcd file 
    //  * @param free_path path of free pcd file
    //  * @param reset_type     0: unknown, 1: free, 2: occ
    //  * @param reset     reset exist map
    //  */
    // void LoadRawMap(string occ_path, string free_path, int reset_type, bool reset);

    /**
     * @brief Get the vox poses on the line
     * 
     * @param start line start pos
     * @param end   line end pos
     * @param line 
     */
    inline void GetCastLine(const Eigen::Vector3d &start, const Eigen::Vector3d &end, list<Eigen::Vector3d> &line);
    inline uint64_t PostoId(const Eigen::Vector3d &pos);//dont check in side map, carefully use
    inline Eigen::Vector3d IdtoPos(const uint64_t &id);//

    inline bool InsideMap(const Eigen::Vector3i &pos);//
    inline bool InsideMap(const Eigen::Vector3d &pos);//
    inline bool InsideExpMap(const Eigen::Vector3d &pos);//


    //for low resolution map
    vector<Eigen::Vector3d> cur_pcl_;
    vector<Eigen::Vector3d> newly_register_idx_;
    double resolution_;
    tr1::unordered_map<uint64_t, pair<uint8_t, uint16_t>> changed_pts_; // <pos id, <initial idx, changed times>>, initial idx: 0: unknown->free, 1: unknown->occ, 2: free, 3: occ

    Eigen::Vector3d local_origin_, local_upbd_, local_lowbd_, local_scale_;
    Eigen::Vector3i local_block_num_, local_origin_idx_, local_up_idx_; // block idx

private:
    //callback func
    // void InsertPCLCallback(const sensor_msgs::PointCloud2ConstPtr &pcl);
    // void InsertDepthCallback(const sensor_msgs::ImageConstPtr &img);
    void CamParamCallback(const sensor_msgs::CameraInfoConstPtr &param);

    //Timer
    void ShowMapCallback(const ros::TimerEvent &e);
    // void MangageMapCallback(const ros::TimerEvent &e);
    void StatisticV(const ros::TimerEvent &e);

    // void ProjectToImg(const sensor_msgs::PointCloud2ConstPtr &pcl, vector<double> &depth_img);
    // void AwakeBlocks(const Eigen::Vector3d &center, const double &range);
    // void InsertSwarmPts(exp_comm_msgs::MapC &msg);

    // data manage
    bool SaveData(const int bid, shared_ptr<Grid_Block> &GB);
    bool ReadData(const int bid, shared_ptr<Grid_Block> &GB);
    bool ReadDataRaw(const int &bid, shared_ptr<Grid_Block> &GB);
    // inline void Vox2Msg(exp_comm_msgs::MapC &msg, const uint16_t &f_id, const uint8_t &b_id);
    // inline void Flags2Vox(vector<uint8_t> &flags, Eigen::Vector3d &up, Eigen::Vector3d &down, uint8_t &block_state, vector<Eigen::Vector3d> &pts, vector<uint8_t> &states);
    // inline void InseartVox(vector<Eigen::Vector3d> &pts, vector<uint8_t> &states);// newly_register_idx_ not update
    // inline void UpdateSBS(const int &SBS_id, const int &sub_id);
    // inline bool GetSBSBound(const uint16_t &SBS_id, const uint8_t &sub_id, Eigen::Vector3d &up, Eigen::Vector3d &down);

    // inline 
    inline uint8_t EncodeVoxel(const uint8_t *c);
    inline void DecodeVoxel(const uint8_t &d, uint8_t *c);

    inline bool GetLocalVox(int &block_id, int &vox_id, const Vector3i &pos);   //return true: inside map; false: outside map
    inline bool GetLocalVox(int &block_id, int &vox_id, const Vector3d &pos);

    // inline float GetVoxOdds(const Eigen::Vector3d &pos);//check if pos is in the block, dont check if pos is in the block
    // inline float GetVoxOdds(const int &id);//don't check, carefully use
    // inline float GetVoxOdds(const Eigen::Vector3d &pos, const shared_ptr<Grid_Block> &GB);//don't check

    inline bool GetBlock3Id(const Eigen::Vector3d &pos, Eigen::Vector3i &blkid);//check
    inline int GetBlockId(const Eigen::Vector3d &pos);//check
    inline int GetBlockId(const Eigen::Vector3i &pos);//check, pos: block id/carefully use 
    inline int GetLocalBlockId(const Eigen::Vector3d &pos);//check
    inline int GetLocalBlockId(const Eigen::Vector3i &pos);//check, pos: global block id 
    inline int GetLocalBlockId(const int &gbid);//check, gbid: global block id 
    inline int GetGlobalBlockId(const int &lbid);//check, gbid: global block id 
    inline Eigen::Vector3i Id2BlockIdx3(const int &id);

    inline int GetVoxId(const Eigen::Vector3d &pos, const shared_ptr<Grid_Block> &GB);//don't check, pos of world
    inline int GetVoxId(const Eigen::Vector3i &pos, const shared_ptr<Grid_Block> &GB);//don't check, pos of world
    inline int GetLocalVoxId(const Eigen::Vector3d &pos, const shared_ptr<Grid_Block> &GB);//don't check, pos of world
    inline int GetLocalVoxId(const Eigen::Vector3i &pos, const shared_ptr<Grid_Block> &GB);//don't check, pos of world
    inline Eigen::Vector3d Id2LocalPos(const shared_ptr<Grid_Block> &GB, const int &id);

    inline uint64_t LocalId2GlobalId(const int &lid, const shared_ptr<Grid_Block> &GB);

    inline bool InsideLocalMap(const Eigen::Vector3i &pos);//
    inline bool InsideLocalMap(const Eigen::Vector3d &pos);//
    inline bool InsideLocalMap(const int &bid);// bid: global block id

    inline Eigen::Vector3i PostoId3(const Eigen::Vector3d &pos);

    inline void GetRayEndInsideMap(const Eigen::Vector3d &start, Eigen::Vector3d &end, bool &occ);
    inline void GetRayEndInsideLocalAndGlobalMap(const Eigen::Vector3d &start, Eigen::Vector3d &end, bool &occ);

    inline void SpointToUV(const double &x, const double &y, Eigen::Vector2i &uv); //standard depth to camera vector index
    inline void UVToPoint(const int &u, const int &v, const double &depth, Eigen::Vector3d &point); //depthcamera point to 3d point 

    inline void UpdateSwarmData();
    inline void LoadSwarmFilter();
    inline bool DebugOut(){cout<<"??"<<endl; return false;}
    inline Eigen::Vector3i Pos2Id3Inside(const Eigen::Vector3d &p);
    inline bool GetXLineIntersec(const Eigen::Matrix3d &R, const Eigen::Vector3d &trans, const double &y, const double &z, double &xd, double &xu);
    inline bool Cast2Img(const Eigen::Matrix3d &R, const Eigen::Vector3d &trans, const Eigen::Vector3d &p, int &x, int &y, double &depth);
    inline std_msgs::ColorRGBA Getcolor(const double z);

    void Debug(list<Eigen::Vector3d> &pts, int ddd = 0);


    ros::NodeHandle nh_, nh_private_;
    ros::Subscriber odom_sub_, sensor_sub_, camparam_sub_, visall_sub_;
    ros::Publisher vox_pub_, debug_pub_;
    ros::Timer show_timer_, debug_timer_, manage_map_timer_;
    vector<std_msgs::ColorRGBA> color_list_;
    double colorhsize_;

    // map data
    // vector<shared_ptr<Grid_Block>> GBS_;
    vector<shared_ptr<Grid_Block>> GBS_local_;
    string map_path_;
    mutex mtx_;
    Eigen::Vector3d origin_, blockscale_, map_upbd_, map_lowbd_;
    Eigen::Vector3i block_size_, voxel_num_, block_num_;       //block_size: size of voxels in a block, block_num_:total number of blocks



    nav_msgs::Odometry robot_odom_;
    Eigen::Matrix4d cam2body_, cam2world_;
    // map updating params
    double pro_hit_;             //log(P(hit|occupied)/P(hit|free))
    double pro_miss_;           //log(P(miss|occupied)/P(miss|free))
    double thr_max_, thr_min_, thr_occ_;

    // sensor params
    bool bline_;
    double max_range_;
    double fx_, fy_, cx_, cy_;    
    int u_max_, v_max_, u_down_max_, v_down_max_, downsample_size_, depth_step_;
    uint64_t vn_;


    bool have_odom_, have_cam_param_, show_block_;
    double last_update_, update_interval_, show_freq_, last_odom_;
    // blocks to be shown
    vector<int> changed_blocks_;
    // awake blocks 
    list<int> awake_blocks_; 

    // statistic
    bool stat_, stat_write_;
    ros::Publisher statistic_pub_;
    ros::Timer statistic_timer_;
    Eigen::Vector3d stat_upbd_, stat_lowbd_;
    uint64_t stat_n_;
    double stat_v_;
    double total_map_t_, map_count_;

    // swarm
    ros::Timer swarm_timer_;
    bool use_swarm_, req_flag_, finish_flag_;
    // SwarmDataManager *SDM_;
    ComputationStatistician *CS_;
    
    // vector<SwarmBlock> SBS_;

    double start_t_;
    list<pair<pair<uint16_t, uint8_t>, double>> swarm_pub_block_; 
    vector<pair<double, Eigen::Matrix4d>> swarm_pose_;
    vector<Eigen::Vector3d> robots_scale_;
    uint8_t self_id_;
    double swarm_send_delay_, swarm_tol_, swarm_pub_thresh_, bline_occ_range_;
    tr1::unordered_map<int, int> *swarm_filter_dict_;
    bool vis_mode_;

    Eigen::MatrixX3d FOV_;

};

// inline void BlockMapLite::Flags2Vox(vector<uint8_t> &flags, Eigen::Vector3d &up, Eigen::Vector3d &down,
//                                     uint8_t &block_state, vector<Eigen::Vector3d> &pts, vector<uint8_t> &states){
//     Eigen::Vector3i v_n;
//     int vox_num = 1;
//     int flag_num = flags.size() * 4;    string map_path_;

//     pts.clear();
//     states.clear();
//     for(int dim = 0; dim < 3; dim++) {
//         vox_num *= int(ceil((up(dim) - down(dim)) / resolution_));
//         v_n(dim) = int(ceil((up(dim) - down(dim)) / resolution_));
//     }
//     if(flag_num < vox_num && block_state == 3){
//         ROS_ERROR("error Flags2Vox %d < %d, state: %d", flag_num, vox_num, block_state);
//         return;
//     }
//     else if(block_state == 0){
//         ROS_ERROR("error Flags2Vox state: 0");
//         return;
//     }

//     Eigen::Vector3d cur_pos, origin_pos;
//     int x_id, y_id, z_id;
//     origin_pos = down + Eigen::Vector3d::Ones() * resolution_ / 2;
//     // cout<<"pure origin_pos:"<<origin_pos.transpose()<<" down:"<<down.transpose()<<endl;

//     if(block_state == 0) return; // all unknown
//     else if(block_state == 1 || block_state == 2){
//         states.resize(vox_num, block_state);

//         for(int i = 0; i < vox_num; i++){
//             x_id = i % v_n(0);
//             y_id = ((i - x_id)/v_n(0)) % v_n(1);
//             z_id = ((i - x_id) - y_id*v_n(0))/v_n(1)/v_n(0);
//             cur_pos(0) = x_id * resolution_ + origin_pos(0);
//             cur_pos(1) = y_id * resolution_ + origin_pos(1);
//             cur_pos(2) = z_id * resolution_ + origin_pos(2);
//             pts.push_back(cur_pos);
//         }
//         return; 
//     }

//     Eigen::Vector3d vox_pos;
//     int id;
//     int flag_id = 0;
//     uint8_t flag1, flag2;
//     pts.resize(vox_num);
//     states.resize(vox_num, 0);
//     for(int i = 0; i < flags.size(); i++){
//         for(int j = 0; j < 4; j++){
//             id = i * 4 + j;
//             x_id = id % v_n(0);
//             y_id = ((id - x_id)/v_n(0)) % v_n(1);
//             z_id = ((id - x_id) - y_id*v_n(0))/v_n(1)/v_n(0);
//             cur_pos(0) = x_id * resolution_ + origin_pos(0);
//             cur_pos(1) = y_id * resolution_ + origin_pos(1);
//             cur_pos(2) = z_id * resolution_ + origin_pos(2);
//             if(id >= vox_num) break;
//             flag1 = 1<<(2*j);
//             flag2 = 2<<(2*j);
//             pts[id] = cur_pos;
//             if(flag1 & flags[i]){ // occ
//                 states[id] = 1;
//             }
//             else if(flag2 & flags[i]){ // free
//                 states[id] = 2;
//             }
//         }
//     }
// }

// inline void BlockMapLite::Vox2Msg(exp_comm_msgs::MapC &msg, const uint16_t &f_id, const uint8_t &b_id){
//     Eigen::Vector3d up, down, it;
//     if(!GetSBSBound(f_id, b_id, up, down)) return;
//     double unknown_num = 0;
//     double sub_num = 1.0;
//     for(int dim = 0; dim < 3; dim++) sub_num *= ceil((up(dim) - down(dim)) / resolution_);

//     msg.block_state = 0;
//     msg.f_id = f_id;
//     msg.block_id = b_id;
//     msg.flags.clear();
//     bool unknown_block = true;
//     bool f = false;
//     bool o = false;
//     int idx = 0;
//     int p;
//     uint8_t flag1, flag2;
//     VoxelState cur_vs, last_vs;
//     for(it(2) = down(2); it(2) < up(2); it(2) += resolution_)
//         for(it(1) = down(1); it(1) < up(1); it(1) += resolution_)
//             for(it(0) = down(0); it(0) < up(0); it(0) += resolution_){
//         cur_vs = GetVoxState(it);
//         p = idx % 4;
//         flag1 = 1<<(2*p);
//         flag2 = 2<<(2*p);
//         if(idx == 0) last_vs = cur_vs;
//         if(p == 0) msg.flags.push_back(0);

//         if(cur_vs == VoxelState::out){
//             ROS_ERROR("Vox2Msg how out?");
//             ros::shutdown();
//             return;
//         }
//         else if(cur_vs == VoxelState::unknown){
//             unknown_num += 1.0;
//         }
//         else if(cur_vs == VoxelState::free){
//             unknown_block = false;
//             f = true;
//             msg.flags.back() |= flag2;
//         }
//         else if(cur_vs == VoxelState::occupied){
//             unknown_block = false;
//             o = true;
//             msg.flags.back() |= flag1;
//         }

//         if(last_vs != cur_vs){
//             msg.block_state = 3;
//         }
//         last_vs = cur_vs;
//         idx++;
//     }

//     if(!msg.flags.empty() && msg.block_state == 0 && !unknown_block){
//         if(msg.flags.front() & 1) {
//             msg.block_state = 1;
//         }
//         else if(msg.flags.front() & 2) msg.block_state = 2;
//         else msg.block_state = 0;
//         msg.flags.clear();
//     }
//     SBS_[f_id].exploration_rate_[b_id] = 1.0 - unknown_num / sub_num;
// }

// inline void BlockMapLite::InseartVox(vector<Eigen::Vector3d> &pts, vector<uint8_t> &states){
//     int block_id, vox_id;
//     Eigen::Vector3d p_it;
//     for(int i = 0; i < pts.size(); i++){
//         if(states[i] == 0) continue;
//         if(!GetLocalVox(block_id, vox_id, pts[i])){
//             cout<<"fail:"<<pts[i].transpose()<<endl;
//             ros::shutdown();
//             return;
//             continue;
//         }
//         float odds_origin = GBS_[block_id]->odds_log_[vox_id];

//         if(GBS_[block_id]->odds_log_[vox_id] < thr_min_ - 1.0){
//             p_it = Id2LocalPos(GBS_[block_id], vox_id);
//             if(stat_){
//                 if(p_it(0) > stat_lowbd_(0) && p_it(1) > stat_lowbd_(1) && p_it(2) > stat_lowbd_(2) &&
//                 p_it(0) < stat_upbd_(0) && p_it(1) < stat_upbd_(1) && p_it(2) < stat_upbd_(2))
//                 stat_n_++;
//             }
//             odds_origin = 0.0;
//         }
//         if(states[i] == 1){
//             GBS_[block_id]->odds_log_[vox_id] = min(odds_origin + pro_hit_, thr_max_);
//         }
//         else if(states[i] == 2){
//             GBS_[block_id]->odds_log_[vox_id] = max(odds_origin + pro_miss_, thr_min_);
//         }
//         if(!GBS_[block_id]->show_ && show_block_){
//             changed_blocks_.push_back(block_id);
//             GBS_[block_id]->show_ = true;
//         }
//     }
// }

// inline void BlockMapLite::UpdateSBS(const int &SBS_id, const int &sub_id){
//     Eigen::Vector3d up, down, it;
//     if(!GetSBSBound(SBS_id, sub_id, up, down)) return;
//     double unknown_num = 0;
//     double sub_num = 1.0;
//     for(int dim = 0; dim < 3; dim++) sub_num *= ceil((up(dim) - down(dim)) / resolution_);

//     for(it(0) = down(0); it(0) < up(0); it(0) += resolution_)
//         for(it(1) = down(1); it(1) < up(1); it(1) += resolution_)
//             for(it(2) = down(2); it(2) < up(2); it(2) += resolution_){
//         VoxelState vs = GetVoxState(it);
//         if(vs == out){
//             ROS_ERROR("UpdateSBS how out?");
//             ros::shutdown();
//             return;
//         }
//         else if(vs == unknown){
//             unknown_num += 1.0;
//         }
//     }

//     SBS_[SBS_id].exploration_rate_[sub_id] = 1.0 - unknown_num / sub_num;
// }

// inline bool BlockMapLite::GetSBSBound(const uint16_t &SBS_id, const uint8_t &sub_id, Eigen::Vector3d &up, Eigen::Vector3d &down){
//     if(SBS_id < 0 || SBS_id >= SBS_.size()) return false;
//     if(sub_id < 0 || sub_id >= 8) return false;
//     Eigen::Vector3d origin = SBS_[SBS_id].down_;
//     for(int dim = 0; dim < 3; dim++){
//         uint8_t flag = 1<<dim;
//         int half_num = ceil((SBS_[SBS_id].up_(dim) - SBS_[SBS_id].down_(dim)) / 2 / resolution_);
//         if(sub_id & flag){
//             up(dim) = SBS_[SBS_id].up_(dim);
//             down(dim) = SBS_[SBS_id].down_(dim) + half_num * resolution_; 
//         }
//         else{
//             up(dim) = SBS_[SBS_id].down_(dim) + half_num * resolution_ - 2e-3;
//             down(dim) = SBS_[SBS_id].down_(dim); 
//         }
//     }

//     return true;
// }

inline uint8_t BlockMapLite::EncodeVoxel(const uint8_t *c){
    uint8_t d = 0;
    for(int i = 0; i < 5; i++){
        d = d * 3 + c[i];
    }
    return d;
}

inline void BlockMapLite::DecodeVoxel(const uint8_t &d, uint8_t *c){
    uint8_t dd = d;
    for(int i = 0; i < 5; i++){
        c[4 - i] = dd % 3;
        dd = dd / 3;
    }
}

inline bool BlockMapLite::GetLocalVox(int &block_id, int &vox_id, const Vector3i &pos){
    Vector3d pos3d = pos.cast<double>() * resolution_;
    return GetLocalVox(block_id, vox_id, pos3d);
}

inline bool BlockMapLite::GetLocalVox(int &block_id, int &vox_id, const Vector3d &pos){
    block_id = GetLocalBlockId(pos);
    if(block_id != -1){
        shared_ptr<Grid_Block> GB_ptr = GBS_local_[block_id];
        if(GB_ptr->state_ == MIXED || GB_ptr->state_ == UNKNOWN){
            vox_id = GetVoxId(pos, GB_ptr);
            // cout<<"blk size:"<<GB_ptr->block_size_.transpose()<<"pos:::"<<pos.transpose()<<"  B origin:"<<GB_ptr->origin_.transpose()<<" state:"<<GB_ptr->state_<<" size1:"<<GB_ptr->odds_log_.size()<<" size2:"<<GB_ptr->flags_.size()<<endl;
            return true;
        }
        else{
            return false;
        }
    }
    else{
        return false;
    }

}

// inline float BlockMapLite::GetVoxOdds(const Eigen::Vector3d &pos){//check if pos is in the block, dont check if pos is in the block
//     int blockid = GetBlockId(pos);
//     if(blockid != -1){
//         shared_ptr<Grid_Block> GB_ptr = GBS_[blockid];
//         if(GB_ptr->state_ == MIXED){
//             return GetVoxOdds(pos, GB_ptr);
//         }
//         else if(GB_ptr->state_ == GBSTATE::FREE){
//             return thr_min_;
//         }
//         else if(GB_ptr->state_ == GBSTATE::OCCUPIED){
//             return thr_max_;
//         }
//         else{
//             return 0.0;
//         }
//     }
//     else{
//         return 0.0;
//     }
// }

// inline float BlockMapLite::GetVoxOdds(const int &id){//don't check, carefully use
//     Eigen::Vector3d pos = IdtoPos(id);
    
//     int blockid = GetBlockId(pos);
//     if(blockid == -1) {
//         return 0.0;
//     }
//     else if(GBS_[blockid]->state_ == GBSTATE::MIXED){
//         return GBS_[blockid]->odds_log_[GetVoxId(pos, GBS_[blockid])];
//     } 
//     else if(GBS_[blockid]->state_ == GBSTATE::FREE){
//         return thr_min_;
//     }
//     else if(GBS_[blockid]->state_ == GBSTATE::OCCUPIED){
//         return thr_max_;
//     }
//     else {
//         return 0.0;
//     }
// }

// inline float BlockMapLite::GetVoxOdds(const Eigen::Vector3d &pos, const shared_ptr<Grid_Block> &FG){//don't check
//     return FG->odds_log_[GetVoxId(pos, FG)];
// }

inline bool BlockMapLite::GetBlock3Id(const Eigen::Vector3d &pos, Eigen::Vector3i &blkid){//check
    // Eigen::Vector3i pos3;

    // cout<<"pos:"<<pos.transpose()<<endl;
    if(InsideMap(pos)){
        Eigen::Vector3d dpos = pos - origin_;
        blkid.x() = floor(dpos.x() / blockscale_.x());
        blkid.y() = floor(dpos.y() / blockscale_.y());
        blkid.z() = floor(dpos.z() / blockscale_.z());
        return true;
    }
    else{
        return false;
    }
}

inline int BlockMapLite::GetBlockId(const Eigen::Vector3d &pos){//check
    if(InsideMap(pos)){
        Eigen::Vector3d dpos = pos - origin_;
        Eigen::Vector3i posid;
        posid.x() = floor(dpos.x() / blockscale_.x());
        posid.y() = floor(dpos.y() / blockscale_.y());
        posid.z() = floor(dpos.z() / blockscale_.z());
        return posid(2)*block_num_(0)*block_num_(1) + posid(1)*block_num_(0) + posid(0);
    }
    else{
        return -1;
    }
}

inline int BlockMapLite::GetBlockId(const Eigen::Vector3i &pos){//check, pos: block id/carefully use 
    if(pos(0) < 0 || pos(1) < 0 || pos(2) < 0 ||
        pos(0) >=  block_num_(0) || pos(1) >= block_num_(1) || pos(2) >= block_num_(2)){
            return -1;
        }
    else{
        return pos(2)*block_num_(0)*block_num_(1) + pos(1)*block_num_(0) + pos(0);
    }
}

inline int BlockMapLite::GetLocalBlockId(const Eigen::Vector3d &pos){
    if(InsideLocalMap(pos)){
        Eigen::Vector3d dpos = pos - local_origin_;
        Eigen::Vector3i posid;
        posid.x() = floor(dpos.x() / blockscale_.x());
        posid.y() = floor(dpos.y() / blockscale_.y());
        posid.z() = floor(dpos.z() / blockscale_.z());
        return posid(2)*local_block_num_(0)*local_block_num_(1) + posid(1)*local_block_num_(0) + posid(0);
    }
    else{
        return -1;
    }
}

inline int BlockMapLite::GetLocalBlockId(const Eigen::Vector3i &pos){
    // if(!InsideMap(pos)) return -1;
    for(int dim = 0; dim < 3; dim++){
        if(pos(dim) < local_origin_idx_(dim) || pos(dim) > local_up_idx_(dim)){
            return -2;
        }
    }
    int id;
    Eigen::Vector3i p = pos - local_origin_idx_;
    return p(2)*local_block_num_(0)*local_block_num_(1) + p(1)*local_block_num_(0) + p(0);
}

inline int BlockMapLite::GetLocalBlockId(const int &gbid){
    int lbid = 0;
    if(gbid < 0 || gbid >= block_num_(0) * block_num_(1) * block_num_(2)) return -1; // outside map
    int x = gbid % block_num_(0) - local_origin_idx_(0);
    if(x < 0 || x >= local_block_num_(0)) return -2;

    int y = ((gbid - x)/block_num_(0)) % block_num_(1) - local_origin_idx_(1);
    if(y < 0 || y >= local_block_num_(1)) return -2;

    int z = ((gbid - x) - y*block_num_(0))/block_num_(1)/block_num_(0) - local_origin_idx_(2);
    if(z < 0 || z >= local_block_num_(2)) return -2;
    lbid = x + y * local_block_num_(0)+ z * local_block_num_(0) * local_block_num_(1);
    return lbid;
}

inline Eigen::Vector3i BlockMapLite::Id2BlockIdx3(const int &id){
    Eigen::Vector3i p;
    p(0) = id % block_num_(0);
    p(1) = ((id - p(0))/block_num_(0)) % block_num_(1);
    p(2) = ((id - p(0)) - p(1)*block_num_(0))/block_num_(1)/block_num_(0);
    return p;
}


inline int BlockMapLite::GetVoxId(const Eigen::Vector3d &pos, const shared_ptr<Grid_Block> &GB){//don't check, pos of world
    Eigen::Vector3d dpos = pos - origin_ - GB->origin_.cast<double>()*resolution_;
    Eigen::Vector3i posid;
    posid.x() = floor(dpos(0) / resolution_);
    posid.y() = floor(dpos(1) / resolution_);
    posid.z() = floor(dpos(2) / resolution_);

    return posid(2) * block_size_.x() * block_size_.y() + posid(1) * block_size_.x() + posid(0);
}

inline int BlockMapLite::GetVoxId(const Eigen::Vector3i &pos, const shared_ptr<Grid_Block> &GB){//don't check, pos of world
    Eigen::Vector3i dpos = pos - GB->origin_;
    return dpos(2)*(block_size_.x())*(block_size_.y()) + dpos(1)*(block_size_.x()) + dpos(0);
}

inline int BlockMapLite::GetLocalVoxId(const Eigen::Vector3d &pos, const shared_ptr<Grid_Block> &GB){
    Eigen::Vector3d dpos = pos - origin_ - GB->origin_.cast<double>()*resolution_;
    Eigen::Vector3i posid;
    posid.x() = floor(dpos(0) / resolution_);
    posid.y() = floor(dpos(1) / resolution_);
    posid.z() = floor(dpos(2) / resolution_);

    return posid(2) * block_size_(0) * block_size_(1) + posid(1) * block_size_(0) + posid(0);
}

inline int BlockMapLite::GetLocalVoxId(const Eigen::Vector3i &pos, const shared_ptr<Grid_Block> &GB){
    Eigen::Vector3i dpos = pos - GB->origin_;
    return dpos(2)*block_size_(0) * block_size_(1) + dpos(1)*block_size_(0) + dpos(0);
}

inline Eigen::Vector3d BlockMapLite::Id2LocalPos(const shared_ptr<Grid_Block> &GB, const int &id){
    int x = id % block_size_(0);
    int y = ((id - x)/block_size_(0)) % block_size_(1);
    int z = ((id - x) - y*block_size_(0))/block_size_(1)/block_size_(0);
    return Eigen::Vector3d((double(x)+0.5)*resolution_,(double(y)+0.5)*resolution_,(double(z)+0.5)*resolution_)+origin_ + GB->origin_.cast<double>() * resolution_;
}

inline uint64_t BlockMapLite::LocalId2GlobalId(const int &lid, const shared_ptr<Grid_Block> &GB){
    uint64_t x = lid % block_size_(0);
    uint64_t y = ((lid - x)/block_size_(0)) % block_size_(1);
    uint64_t z = ((lid - x) - y*block_size_(0))/block_size_(1)/block_size_(0);
    x += GB->origin_(0);
    y += GB->origin_(1);
    z += GB->origin_(2);
    return x + y * voxel_num_(0) + z * voxel_num_(0) * voxel_num_(1);
}

inline bool BlockMapLite::InsideMap(const Eigen::Vector3i &pos){
    // cout<<"dpos:"<<dpos.transpose()<<endl;
    if(pos(0) < 0 || pos(1) < 0 || pos(2) < 0 ||
        pos(0) >=  voxel_num_(0) || pos(1) >= voxel_num_(1) || pos(2) >= voxel_num_(2))
        return false;
    return true;
}

inline bool BlockMapLite::InsideMap(const Eigen::Vector3d &pos){
    if(pos(0) < map_lowbd_(0)|| pos(1) < map_lowbd_(1)|| pos(2) < map_lowbd_(2)||
        pos(0) >  map_upbd_(0) || pos(1) > map_upbd_(1) || pos(2) > map_upbd_(2) )
        return false;
    return true;
}

inline bool BlockMapLite::InsideExpMap(const Eigen::Vector3d &pos){
    if(pos(0) < stat_lowbd_(0)|| pos(1) < stat_lowbd_(1)|| pos(2) < stat_lowbd_(2)||
        pos(0) >  stat_upbd_(0) || pos(1) > stat_upbd_(1) || pos(2) > stat_upbd_(2) )
        return false;
    return true;
}

inline bool BlockMapLite::InsideLocalMap(const Eigen::Vector3i &pos){
    if(pos(0) < local_origin_idx_(0) || pos(1) < local_origin_idx_(1) || pos(2) < local_origin_idx_(2) ||
        pos(0) >  local_up_idx_(0) || pos(1) > local_up_idx_(1) || pos(2) > local_up_idx_(2))
        return false;
    return true;
}

inline bool BlockMapLite::InsideLocalMap(const Eigen::Vector3d &pos){
    if(pos(0) < local_lowbd_(0)|| pos(1) < local_lowbd_(1)|| pos(2) < local_lowbd_(2)||
        pos(0) >  local_upbd_(0) || pos(1) > local_upbd_(1) || pos(2) > local_upbd_(2) )
        return false;
    return true;
}

inline bool BlockMapLite::InsideLocalMap(const int &bid){
    if(bid < 0 || bid >= block_num_(0) * block_num_(1) * block_num_(2)) return false; // outside map
    int x = bid % block_num_(0);
    if(x < local_origin_idx_(0) || x > local_up_idx_(0)) return false;

    int y = ((bid - x)/block_num_(0)) % block_num_(1);
    if(y < local_origin_idx_(1) || y > local_up_idx_(1)) return false;

    int z = ((bid - x) - y*block_num_(0))/block_num_(1)/block_num_(0);
    if(z < local_origin_idx_(2) || z > local_up_idx_(2)) return false;
    return true;
}

inline Eigen::Vector3d BlockMapLite::IdtoPos(const uint64_t &id){
    int x = id % voxel_num_(0);
    int y = ((id - x)/voxel_num_(0)) % voxel_num_(1);
    int z = ((id - x) - y*voxel_num_(0))/voxel_num_(1)/voxel_num_(0);
    return Eigen::Vector3d((double(x)+0.5)*resolution_,(double(y)+0.5)*resolution_,(double(z)+0.5)*resolution_)+origin_;
}

inline void BlockMapLite::GetCastLine(const Eigen::Vector3d &start, const Eigen::Vector3d &end, list<Eigen::Vector3d> &line){
    RayCaster rc;
    Eigen::Vector3d ray_iter;
    Eigen::Vector3d half_res = Eigen::Vector3d(0.5, 0.5, 0.5) * resolution_;
    line.clear();
    rc.setInput((start - origin_) / resolution_, (end - origin_) / resolution_);
    while (rc.step(ray_iter))
    {
        ray_iter = (ray_iter) * resolution_ + origin_ + half_res;
        line.emplace_back(ray_iter);
    }
}

inline uint64_t BlockMapLite::PostoId(const Eigen::Vector3d &pos){
    uint64_t x, y, z;
    x = (pos(0)-origin_(0))/resolution_;
    y = (pos(1)-origin_(1))/resolution_;
    z = (pos(2)-origin_(2))/resolution_;
    x += y*voxel_num_(0);
    x += z*voxel_num_(0)*voxel_num_(1);
    return x;
}

inline Eigen::Vector3i BlockMapLite::PostoId3(const Eigen::Vector3d &pos){
    return Eigen::Vector3i((int)floor((pos(0) - origin_(0))/resolution_), (int)floor((pos(1) - origin_(1))/resolution_),
         (int)floor((pos(2) - origin_(2))/resolution_)); 
}


inline VoxelState BlockMapLite::GetVoxState(const int &id){
    Eigen::Vector3d pos = IdtoPos(id);
    if(!InsideMap(pos)) return VoxelState::out;

    int blockid = GetLocalBlockId(pos);
    if(blockid == -1) {
        return VoxelState::outlocal;
    }
    else if(GBS_local_[blockid]->state_ == GBSTATE::MIXED){
        float odds = GBS_local_[blockid]->odds_log_[GetVoxId(pos, GBS_local_[blockid])];
        if(odds > 0) return VoxelState::occupied;
        else if(odds < 0 && odds >= thr_min_) return VoxelState::free;
        else return VoxelState::unknown;
    } 
    else {
        return VoxelState::unknown;
    }
}

inline VoxelState BlockMapLite::GetVoxState(const Eigen::Vector3i &id){
    int voxid = id(0) + id(1) * voxel_num_(0) + id(2) * voxel_num_(0) * voxel_num_(1);
    return GetVoxState(voxid);
}

inline VoxelState BlockMapLite::GetVoxState(const Eigen::Vector3d &pos){
    if(!InsideMap(pos)) return VoxelState::out;
    
    int blockid = GetLocalBlockId(pos);

    if(blockid != -1){
        shared_ptr<Grid_Block> GB_ptr = GBS_local_[blockid];
        if(GB_ptr->state_ == MIXED){
            float odds = GBS_local_[blockid]->odds_log_[GetVoxId(pos, GBS_local_[blockid])];
            // cout<<odds<<"  "<<thr_min_<<endl;
            if(odds > 0) return VoxelState::occupied;
            else if(odds < 0 && odds > thr_min_ - 1e-3) return VoxelState::free;
            else return VoxelState::unknown;
        }
        else{
            return VoxelState::unknown;
        }
    }
    else{
        return VoxelState::outlocal;
    }
}

inline VoxelState BlockMapLite::GetExpVoxState(const Eigen::Vector3d &pos){
    if(!InsideExpMap(pos)) return VoxelState::out;
    
    int blockid = GetLocalBlockId(pos);

    if(blockid != -1){
        shared_ptr<Grid_Block> GB_ptr = GBS_local_[blockid];
        if(GB_ptr->state_ == MIXED){
            float odds = GBS_local_[blockid]->odds_log_[GetVoxId(pos, GBS_local_[blockid])];
            // cout<<odds<<"  "<<thr_min_<<endl;
            if(odds > 0) return VoxelState::occupied;
            else if(odds < 0 && odds > thr_min_ - 1e-3) return VoxelState::free;
            else return VoxelState::unknown;
        }
        else{
            return VoxelState::unknown;
        }
    }
    else{
        return VoxelState::outlocal;
    }
}

inline void BlockMapLite::GetRayEndInsideMap(const Eigen::Vector3d &start, Eigen::Vector3d &end, bool &occ){
    double lx, ly, lz;
    if(end(0) > map_upbd_(0)){
        lx = (map_upbd_(0) - start(0)) / (end(0) - start(0)) - 1e-4;
        occ = 0;
    }    
    else if(end(0) < map_lowbd_(0)){
        lx = (start(0) - map_lowbd_(0)) / (start(0) - end(0)) - 1e-4;
        occ = 0;
    }    
    else lx = 1.0;

    if(end(1) > map_upbd_(1)){
        ly = (map_upbd_(1) - start(1)) / (end(1) - start(1)) - 1e-4;
        occ = 0;
    }    
    else if(end(1) < map_lowbd_(1)){
        ly = (start(1) - map_lowbd_(1)) / (start(1) - end(1)) - 1e-4;
        occ = 0;
    }    
    else ly = 1.0;

    if(end(2) > map_upbd_(2)){
        lz = (map_upbd_(2) - start(2)) / (end(2) - start(2)) - 1e-4;
        occ = 0;
    }    
    else if(end(2) < map_lowbd_(2)){
        lz = (start(2) - map_lowbd_(2)) / (start(2) - end(2)) - 1e-4;
        occ = 0;
    }    
    else lz = 1.0;

    end = (end - start) * min(lx, min(ly, lz)) + start;
}

inline void BlockMapLite::GetRayEndInsideLocalAndGlobalMap(const Eigen::Vector3d &start, Eigen::Vector3d &end, bool &occ){
    Eigen::Vector3d upbd, lowbd;
    for(int dim = 0; dim < 3; dim++){
        upbd(dim) = min(local_upbd_(dim), map_upbd_(dim));
        lowbd(dim) = max(local_lowbd_(dim), origin_(dim));
    }

    double lx, ly, lz;
    if(end(0) > upbd(0)){
        lx = (upbd(0) - start(0)) / (end(0) - start(0)) - 1e-4;
        occ = 0;
    }    
    else if(end(0) < lowbd(0)){
        lx = (start(0) - lowbd(0)) / (start(0) - end(0)) - 1e-4;
        occ = 0;
    }    
    else lx = 1.0;

    if(end(1) > upbd(1)){
        ly = (upbd(1) - start(1)) / (end(1) - start(1)) - 1e-4;
        occ = 0;
    }    
    else if(end(1) < lowbd(1)){
        ly = (start(1) - lowbd(1)) / (start(1) - end(1)) - 1e-4;
        occ = 0;
    }    
    else ly = 1.0;

    if(end(2) > upbd(2)){
        lz = (upbd(2) - start(2)) / (end(2) - start(2)) - 1e-4;
        occ = 0;
    }    
    else if(end(2) < lowbd(2)){
        lz = (start(2) - lowbd(2)) / (start(2) - end(2)) - 1e-4;
        occ = 0;
    }    
    else lz = 1.0;

    end = (end - start) * min(lx, min(ly, lz)) + start;
}


inline void BlockMapLite::SpointToUV(const double &x, const double &y, Eigen::Vector2i &uv){
    // return Eigen::Vector2i(fx_ * x + cx_, fy_ * y + cy_);
    uv.x() = fx_ * x + cx_;
    uv.y() = fy_ * y + cy_;
}

inline void BlockMapLite::UVToPoint(const int &u, const int &v, const double &depth, Eigen::Vector3d &point){
    point.x() = (u - cx_) * depth / fx_;
    point.y() = (v - cy_) * depth / fy_;
    point.z() = depth;
} 

// inline void BlockMapLite::UpdateSwarmData(){
//     Eigen::Quaterniond qua;
//     Eigen::Vector3d pos;
//     Eigen::Matrix4d pose;
//     for(int i = 0; i < swarm_pose_.size(); i++){
//         pose.setZero();
//         if(i + 1 == self_id_) continue;
//         qua.x() = SDM_->Poses_[i].orientation.x;
//         qua.y() = SDM_->Poses_[i].orientation.y;
//         qua.z() = SDM_->Poses_[i].orientation.z;
//         qua.w() = SDM_->Poses_[i].orientation.w;
//         pos(0) = SDM_->Poses_[i].position.x;
//         pos(1) = SDM_->Poses_[i].position.y;
//         pos(2) = SDM_->Poses_[i].position.z;
//         // cout<<"use:"<<pos.transpose()<<" "<<endl;
//         pose.block(0, 0, 3, 3) = qua.toRotationMatrix();
//         pose.block(0, 3, 3, 1) = pos;
//         pose(3, 3) = 1.0;
//         swarm_pose_[i].second = pose;
//         swarm_pose_[i].first = SDM_->Pose_t_[i];
//     }
// }

// inline void BlockMapLite::LoadSwarmFilter(){
//     if(!use_swarm_) return;
//     UpdateSwarmData();
//     swarm_filter_dict_->clear();
//     double cur_t, max_r;
//     Eigen::Matrix3d rot;
//     Eigen::Vector3d p, c, r, n;
//     Eigen::Vector3d cam = cam2world_.block(0,3,3,1);
//     for(int i = 0; i < swarm_pose_.size(); i++){
//         if(cur_t - swarm_pose_[i].first > swarm_tol_ || i + 1 == self_id_) continue;
//         c =  swarm_pose_[i].second.block(0, 3, 3, 1);
//         c = IdtoPos(PostoId(c));
//         r = robots_scale_[i]/2 + Eigen::Vector3d::Ones() * resolution_;
//         max_r = r.maxCoeff();
//         r = r.cwiseProduct(r);
//         if((cam - c).norm() > max_r + max_range_) continue;
//         int k = ceil(max_r / resolution_);
//         rot = swarm_pose_[i].second.block(0, 0, 3, 3).transpose(); 
//         for(p(0) = c(0) - k * resolution_; p(0) <= c(0) + k * resolution_ + 1e-3; p(0) += resolution_){
//             for(p(1) = c(1) - k * resolution_; p(1) <= c(1) + k * resolution_ + 1e-3; p(1) += resolution_){
//                 for(p(2) = c(2) - k * resolution_; p(2) <= c(2) + k * resolution_ + 1e-3; p(2) += resolution_){
//                     if(!InsideLocalMap(p))continue;
//                     n = rot * (p - c);
//                     if(n(0)*n(0)/r(0) + n(1)*n(1)/r(1) + n(2)*n(2)/r(2) < 1.0){//inside ellipsoid
//                         swarm_filter_dict_->insert({PostoId(p), 0});
//                     }
//                 }
//             }
//         }
//     }
//     // Debug();
// }

inline std_msgs::ColorRGBA BlockMapLite::Getcolor(const double z){
    std_msgs::ColorRGBA color;
    double difz = z - origin_(2);
    color.a = 1.0;
    if(difz > map_upbd_(2)){
        return color_list_.back();
    }
    else if(difz < origin_(2)){
        return color_list_.front();
    }
    else{
        
        int hieghtf = floor(difz / colorhsize_);
        int hieghtc = hieghtf + 1;
        double gain = (difz - colorhsize_*hieghtf)/colorhsize_;
        color.r = color_list_[hieghtf].r*(1.0-gain) + color_list_[hieghtc].r*gain;
        color.g = color_list_[hieghtf].g*(1.0-gain) + color_list_[hieghtc].g*gain;
        color.b = color_list_[hieghtf].b*(1.0-gain) + color_list_[hieghtc].b*gain;
    }
    return color;
}

inline Eigen::Vector3i BlockMapLite::Pos2Id3Inside(const Eigen::Vector3d &p){
    Eigen::Vector3d P;
    P(0) = min(p(0), local_upbd_(0) - 1e-3);
    P(1) = min(p(1), local_upbd_(1) - 1e-3);
    P(2) = min(p(2), local_upbd_(2) - 1e-3);
    P(0) = max(P(0), local_origin_(0) + 1e-3);
    P(1) = max(P(1), local_origin_(1) + 1e-3);
    P(2) = max(P(2), local_origin_(2) + 1e-3);
    return PostoId3(P);
}



inline bool BlockMapLite::GetXLineIntersec(const Eigen::Matrix3d &R, const Eigen::Vector3d &trans, const double &y, const double &z, double &xd, double &xu){
    double py, pz;
    py = y - trans(1);
    pz = z - trans(2);
    double xp_s = pow(max_range_, 2) - pow(py, 2) - pow(pz, 2);
    if(xp_s <= 0){
        return false;
    }
    double qxp_s = sqrt(xp_s);
    xd = -qxp_s;
    xu = qxp_s;
    Eigen::Vector3d c;
    double x;
    for(int i = 0; i < FOV_.rows(); i++){
        c = R * FOV_.row(i).transpose();
        if(c(0) == 0){
            if(c(1) * py + c(2) * pz >= 0) return false;
            else continue;
        }
        else {
            x = -(c(1) * py + c(2) * pz) / c(0);
            if(c(0) > 0){
                if(xd > x) return false;
                if(xu > x) xu = x;
            }
            else{
                if(xu < x) return false;
                if(xd < x) xd = x;
            }
        }
    }
    xu += trans(0);
    xd += trans(0);
    xu = min(max(xu, map_lowbd_(0)), map_upbd_(0));
    xd = min(max(xd, map_lowbd_(0)), map_upbd_(0));
    return true;
}

inline bool BlockMapLite::Cast2Img(const Eigen::Matrix3d &R, const Eigen::Vector3d &trans, const Eigen::Vector3d &p, int &x, int &y, double &depth){
    Eigen::Vector3d pc = R*(p - trans);
    depth = pc(2);
    if(depth < 1e-3) return false;
    // for(int i = 0; i < FOV_.rows(); i++){
    //     if(FOV_.row(i).dot(pc) > 0) {
    //         // ROS_WARN("out1");
    //         // getchar();
    //         return false;
    //     }
    // }
    // pc /= depth;
    x = floor(pc(0) * fx_ / depth + cx_);
    if(x < 0 || x >= u_max_) {
        // ROS_WARN("out2");
        // getchar();
        return false;
    }
    y = floor(pc(1) * fy_ / depth + cy_);
    if(y < 0 || y >= v_max_) {
        // ROS_WARN("out3");
        // getchar();
        return false;
    }
    return true;
}


#endif