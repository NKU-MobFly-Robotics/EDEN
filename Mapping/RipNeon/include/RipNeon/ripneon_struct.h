#ifndef RIPNEON_MAP_STRUCT_H_
#define RIPNEON_MAP_STRUCT_H_

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

#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;
namespace PointBlockStruct{

enum VoxelState{
    unknown, 
    free, 
    frontier, 
    occupied,
    out,
    outlocal

};

#define PV_FRONTIER 1 // is frontier
#define PV_FULL 2 // voxel pts full
#define PV_BFS 4 // in breadth first search list
#define PV_OCC 8 // not occupied pt, but lidar point casted in the voxel
#define PV_PREFRONTIER 16 // is frontier before update, for frontier change 


#define PV_BFS_OCC 12
#define PV_FRONTIER_BFS 5
#define PV_BFS_PREFRONTIER 20


#define PV_RESET_FRONTIER 254
#define PV_RESET_FULL 253
#define PV_RESET_BFS 251
#define PV_RESET_OCC 247

#define PV_RESET_FULL_FRONTIER 252
#define PV_RESET_BFS_OCC_PREFRONTIER 227


struct PointVox{
    PointVox():
    // inf_num_(0),
    log_odds_(0),
    PvFlags_(0)//,
    // pts_{}
    {
    }
    PointVox(float &d):
    // inf_num_(0),
    log_odds_(d),
    PvFlags_(0)//,
    // pts_{}
    {
    }
    // ~PointVox(){}
    // list<Eigen::Vector3f> pts_;
    // uint8_t inf_num_;
    uint8_t PvFlags_; // 0000 (lidar occ pt)(BFS)(full pts)(frontier)
    float log_odds_;
};

struct RdImg{
    vector<vector<pair<float, float>>> depth_; // min, max
    vector<pair<uint16_t, uint16_t>> points_idx_; // for quick refresh
    vector<vector<bool>> void_; // this ray contains occ/lidar point
    // vector<vector<pair<float, float>>> forntier_cheese_;
    // vector<vector<bool>> frontier_;
};

#define PVB_SHOW 1
#define PVB_EMPTY 2
#define PVB_OCC 4 // not occupied block, but lidar point casted in the block
#define PVB_UP 8 

#define PVB_EMPTY_OCC 6 

#define PVB_RESET_SHOW 254
#define PVB_RESET_EMPTY 253
#define PVB_RESET_OCC 251
#define PVB_RESET_UP 247


struct PV_Block{
    PV_Block() {};
    ~PV_Block() {};
    void Reset(int r, float &l){
        vox_.resize(r, l);
        occ_idx_.clear();
        PvbFlags_ = 0;
    }
    vector<PointVox> vox_;
    vector<uint16_t> occ_idx_;
    vector<uint16_t> fron_idx_;
    uint8_t PvbFlags_; // 0000 (in update)(occ casted)(empty block)(in show list)
    Eigen::Vector3i origin_;
    uint32_t id_;
};


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
                            