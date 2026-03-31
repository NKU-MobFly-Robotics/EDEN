#ifndef TRAJ_DEBUG_OPT_H_
#define TRAJ_DEBUG_OPT_H_
#include "gcopter_debug/trajectory.hpp"
#include "gcopter_debug/gcopter_aggressive.hpp"
// #include "gcopter_debug/gcopter_cover.hpp"
#include "gcopter_debug/gcopter_norm.hpp"
#include "gcopter_debug/fastcheck_traj.hpp"
#include "gcopter_debug/trajectory4.hpp"
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>
#include<fstream>

using namespace std;
enum TrajType{
    AggresiveBranch,
    CoverBranch,
    FormTraj,
    Norm
};

struct TrajOptInput{
    // pair<Eigen::Vector3d, Eigen::Vector3d> corridor_bbx; // up, down

    vector<Eigen::MatrixX4d> corridors_norm;  
    vector<Eigen::Matrix3Xd> corridorVs_norm;

    vector<Eigen::MatrixX4d> corridors_stem;
    vector<Eigen::Matrix3Xd> corridorVs_stem;

    vector<Eigen::MatrixX4d> corridors_main;    //branch
    vector<Eigen::Matrix3Xd> corridorVs_main;

    vector<Eigen::MatrixX4d> corridors_sub;     //branch
    vector<Eigen::Matrix3Xd> corridorVs_sub;

    Eigen::Matrix3Xd corridorVs_Ep;

    Eigen::MatrixX4d camFov;
    Eigen::Matrix3Xd camV;

    Eigen::Matrix4Xd initState;
    // Eigen::Matrix4Xd tailState; // only for norm traj

    Eigen::Vector4d endState_norm;          

    Eigen::Vector4d endState_stem;
    Eigen::Vector4d endState_main;
    Eigen::Vector4d endState_sub;
    // int self_id;

    vector<Trajectory4<5>> ref_trajs;
    vector<pair<Eigen::Vector3d, Eigen::Vector3d>> ref_traj_bbxs; // <up, down>
    vector<pair<Eigen::Vector3d, Eigen::Vector3d>> traj_bbxs_norm; // <up, down>
    vector<pair<Eigen::Vector3d, Eigen::Vector3d>> traj_bbxs_sub; // <up, down>
    vector<pair<Eigen::Vector3d, Eigen::Vector3d>> traj_bbxs_main; // <up, down>
    vector<pair<Eigen::Vector3d, Eigen::Vector3d>> traj_bbxs_stem; // <up, down>
    vector<double> swarm_t;
    
    vector<Eigen::Vector3d> poses;
    double start_t;
};

class TrajOptimizerDebug{
public:
    void Init(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
    bool AggressiveBranchTrajOptimize(TrajOptInput &toi);
    bool CoverBranchTrajOptimize(TrajOptInput &toi);
    bool FormTrajOptimize(TrajOptInput &toi);
    bool NormTrajOptimize(TrajOptInput &toi);

    bool Optimize(
                    const vector<Eigen::MatrixX4d> &corridors,
                    const vector<Eigen::Matrix3Xd> &corridorVs,
                    const double &min_t,
                    const Eigen::Matrix3d &initState,
                    const Eigen::Matrix3d &endState,
                    const int &self_id,
                    vector<Trajectory4<5>> &trajs,
                    vector<double> &swarm_t,
                    vector<Eigen::Vector3d> &poses,
                    double start_t);
                                             
    Trajectory4<5> norm_traj_, main_traj_, sub_traj_, stem_traj_;    
    
    vector<double> weightVec_; //[pos, vel, acc, jerk, yv, ya, avoidT]
    vector<double> upboundVec_; //[maxvel, maxacc, maxjerk, maxyawvel, maxyawacc, swarm]
private:
    void Debug(list<Eigen::Vector3d> &debug_list);
    void GetInterPts(const vector<Eigen::Matrix3Xd> &corridorVs, vector<Eigen::Matrix3Xd> &corridorVsIters);
    ros::Publisher swarm_traj_pub_;
    ros::NodeHandle nh_, nh_private_;
    double smoothingEps_;
    double weightT_, weight_minT_;
    int integralIntervs_;

    double trajlength_;

    double relCostTol_;

    double trajStamp;

    ofstream debug_write_;
    bool debug_;

};
#endif

