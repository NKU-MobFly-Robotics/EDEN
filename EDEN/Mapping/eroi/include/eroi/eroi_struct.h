#ifndef EROI_STRUCT_H_
#define EROI_STRUCT_H_
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

namespace EROIStruct{
enum SensorType{LIVOX, CAMERA};

struct EroiNode{
    uint8_t f_state_;                    //0: unexplored; 1:exploring; 2: explored; 3: invalid; 
    uint8_t owner_;
    float owner_dist_;

    Eigen::Vector3d up_, down_, center_;

    vector<uint8_t> local_vps_;         //0: unsampled; 1:alive; 2: dead; 
    vector<uint8_t> valid_vps_;       // the idx that are valid in dtg
    double last_sample_;
    double last_strong_check_;
    int unknown_num_, thresh_num_;
    uint8_t flags_;                     //(local new)(show)(sample)(temp)
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
                            