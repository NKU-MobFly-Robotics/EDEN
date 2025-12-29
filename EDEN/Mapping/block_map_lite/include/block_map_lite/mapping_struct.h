#ifndef MAPPING_STRUCT_H_
#define MAPPING_STRUCT_H_
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <deque>
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <list>
#include <bitset>
#include <memory>
#include <math.h>
#include <std_msgs/ColorRGBA.h>
using namespace std;
using namespace Eigen;

namespace BlockMapStruct{

enum GBSTATE{
    UNKNOWN,
    MIXED
};

enum VoxelState{
    unknown, 
    free, 
    occupied,
    out,
    outlocal
};

// struct SwarmBlock{
//     uint16_t id_;
//     Eigen::Vector3d up_, down_;
//     vector<double> exploration_rate_;
//     vector<double> last_pub_rate_;
//     vector<bool> to_pub_;
//     uint8_t sub_num_;
// };

struct EROI{  // explorable reagion of interest
    uint8_t f_state_;                    //0: unexplored; 1:exploring; 2: explored; 
    // uint8_t owner_;
    // float owner_dist_;

    Eigen::Vector3d up_, down_, center_;

    vector<uint8_t> local_vps_;         //0: unsampled; 1:alive; 2: dead; 
    double last_sample_;
    double last_strong_check_;
    uint32_t unknown_num_, thresh_num_;
    uint8_t flags_;                     //(local new)(show)(sample)(temp)
};

struct Grid_Block{
    Grid_Block() {state_ = UNKNOWN;};
    ~Grid_Block() {};
    // void Awake(float occ, float free){       //if state == UNKNOWN/OCCUPIED/FREE, init odds_log_ of this block 
    //     if(state_ == UNKNOWN){
    //         odds_log_.resize(block_size_.x() * block_size_.y() * block_size_.z(), free - 999.0);
    //         flags_.resize(odds_log_.size(), 0);
    //     }
    //     state_ = MIXED;
    // }
    void Reset(double v, int r){
        odds_log_.resize(r, v);
        // if(odds_log_[0] == 0) {
        //     ROS_ERROR("reset 0 %lf", v);
        //     getchar();
        // }
        flags_.resize(odds_log_.size(), 0);
        state_ = UNKNOWN;
        show_ = false;
    }
    Vector3i origin_;
    unsigned state_;
    vector<float> odds_log_;
    vector<uint8_t> flags_;  //0000 00_(is ray end occupied flag)_(casted)
    int id_;
    bool show_;
};

struct FFD_Grid{
    double far_depth_;
    double close_depth_;
    double max_depth_;

    double dist2depth_;
    bool is_frontier_;
    bool new_iter_;
};
}

#endif