#ifndef EDEN_H_
#define EDEN_H_

#include <ros/ros.h>
#include <thread>
#include <Eigen/Eigen>
#include <vector>
#include <list>
#include <tr1/unordered_map>
#include <fstream>
#include <unistd.h>
#include <sys/resource.h>
#include <eroi/eroi.h>
#include <mr_dtg_plus/mr_dtg_plus.h>
#include <block_map_lite/color_manager.h>
#include <block_map_lite/block_map_lite.h>
#include <lowres_map_lite/lowres_map_lite.h>
#include <gcopter/traj_opt.h>
// #include <frontier_grid/frontier_grid.h>
#include <swarm_data/swarm_data.h>
// #include <graph_partition/graph_partition.h>
// #include <group_work/group_work.h>
#include <data_statistics/computation_statistician.h>

#include <visualization_msgs/MarkerArray.h>
#include <swarm_exp_msgs/LocalTraj.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

class SingleExp{
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry>
    SyncPolicyImageOdom;
typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImageOdom>> SynchronizerImageOdom;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry>
    SyncPolicyPCLOdom;
typedef shared_ptr<message_filters::Synchronizer<SyncPolicyPCLOdom>> SynchronizerPCLOdom;
public:
    SingleExp(){};
    ~SingleExp(){cout<<"------==========v_mean:"<<v_total_ / max(v_num_, 1)<<endl;};
    void init(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
    void SetPlanInterval(const double &intv);
    bool TrajCheck();
    int AllowPlan(const double &T);
    bool EdenPlan();
    bool TrajReplan();
    bool AllowReplan(){return ros::WallTime::now().toSec() > traj_replan_t_ && ros::WallTime::now().toSec() + 0.2 < traj_end_t_;};
    bool GoHome();
    bool FindShorterPath();
    int ViewPointsCheck(const double &t);
    void ForceUpdateEroiDtg();
    void StopDebugFunc();

    double plan_t_;

private:

    /**
     * @brief update voxel map through depth image
     * 
     * @param img 
     * @param odom 
     */
    void ImgOdomCallback(const sensor_msgs::ImageConstPtr& img,
        const nav_msgs::OdometryConstPtr& odom);
    /**
    * @brief update voxel map through point cloud
    * 
    * @param pcl 
    * @param odom 
    */
    void PCLOdomCallback(const sensor_msgs::PointCloud2ConstPtr& pcl,
            const nav_msgs::OdometryConstPtr& odom);
    void BodyOdomCallback(const nav_msgs::OdometryConstPtr& odom);
    void ReloadMap(const ros::TimerEvent &e);
    void UpdateDTG(const ros::TimerEvent &e);
    void DataStatistic(const ros::TimerEvent &e);

    void TargetPlanning(const Eigen::Vector4d &p, const Eigen::Vector4d &v,
                        uint8_t &plan_res, vector<Eigen::Vector3d> &path_stem, vector<Eigen::Vector3d> &path_main, 
                        vector<Eigen::Vector3d> &path_sub, vector<Eigen::Vector3d> &path_norm, 
                        pair<uint32_t, uint8_t> &tar_stem, pair<uint32_t, uint8_t> &tar_main, 
                        pair<uint32_t, uint8_t> &tar_sub, pair<uint32_t, uint8_t> &tar_norm, 
                        double &y_stem, double &y_main, double &y_sub, double &y_norm);
    void PublishTraj();
    void ShowTraj();
    void ShowPathCorridor(vector<Eigen::Vector3d> &path, vector<Eigen::MatrixX4d> &h);
    void Debug(vector<Eigen::Vector3d> &pts);
    ros::NodeHandle nh_, nh_private_;
    ros::Publisher traj_pub_, vis_pub_, debug_pub_;
    ros::Subscriber odom_sub_;
    ros::Timer reload_map_timer_, dtg_timer_, stat_timer_;

    shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> vi_odom_sub_;
    shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
    shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> pcl_sub_;
    SynchronizerImageOdom sync_image_odom_;
    SynchronizerPCLOdom sync_pointcloud_odom_;

    Eigen::MatrixX4d camFov_;
    Eigen::Matrix3Xd camV_;

    lowres_lite::LowResMap LRM_;
    BlockMapLite BM_;
    ComputationStatistician CS_;
    // SwarmDataManager SDM_;
    EroiGrid EROI_;
    ColorManager CM_;
    DTGPlus::MultiDtgPlus DTG_;

    Trajectory4<5> exc_traj_;    
    TrajOptimizer TrajOpt_;
    bool run_branch_;
    
    bool stat_;
    double replan_t_, traj_replan_t_, reach_out_t_, traj_length_, traj_sub_length_, replan_duration_, traj_replan_duration_;
    double traj_start_t_, traj_end_t_, check_duration_;
    int64_t target_f_id_, target_v_id_;
    vector<Eigen::Vector3d> path_norm_, path_stem_, path_main_, path_sub_;
    Eigen::Vector4d tar_norm_, tar_stem_, tar_main_, tar_sub_;
    int traj_type_; // 1: branch, 2: norm
    Eigen::Vector3d p_, v_;
    double yaw_, yaw_v_;
    Eigen::Matrix4d robot_pose_;
    int v_num_;
    double v_total_, len_, ts_;


    bool dtg_flag_, have_odom_;
    double last_map_update_t_, update_interval_;
};
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
                            