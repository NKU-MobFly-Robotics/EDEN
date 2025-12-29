#include <mr_dtg_plus/mr_dtg_plus.h>
using namespace DTGPlus;

void MultiDtgPlus::AlignInit(ros::NodeHandle &nh, ros::NodeHandle &nh_private){
    std::string ns = ros::this_node::getName();
    nh_ = nh;
    nh_private_ = nh_private;
    nh_private.param(ns + "/Exp/UAV_id", uav_id_, 10);
    nh_private.param(ns + "/MR_DTG/show_edge_details", show_e_details_, false);
    nh_private.param(ns + "/MR_DTG/H_thresh", H_thresh_, 5.0);
    nh_private.param(ns + "/MR_DTG/MaintainHdists", maintain_h_dist_, false);
    nh_private.param(ns + "/MR_DTG/PlanDebug", debug_plan_, false);
    nh_private.param(ns + "/MR_DTG/AccGain", acc_gain_, 2.0);
    nh_private.param(ns + "/MR_DTG/YawGain", yaw_gain_, 2.5);
    nh_private.param(ns + "/MR_DTG/YawSliceAng", yaw_slice_ang_, 0.3);
    nh_private.param(ns + "/MR_DTG/RefineNum", refine_num_, 5);
    nh_private.param(ns + "/MR_DTG/GainThreshFactor", g_thr_fac_, 0.9);
    nh_private.param(ns + "/block_map/sensor_max_range", sensor_range_, 5.0);
    nh_private.param(ns + "/Exp/drone_num", drone_num_, 1);
    nh_private.param(ns + "/Exp/lambdaE", lambda_e_, 0.5);
    nh_private.param(ns + "/Exp/lambdaA", lambda_a_, 0.5);
    nh_private.param(ns + "/opt/MaxVel", v_max_, 0.5);
    nh_private.param(ns + "/opt/MaxAcc", a_max_, 0.5);
    nh_private.param(ns + "/opt/YawVel", yv_max_, 0.5);
    
    origin_ = LRM_->origin_;
    vox_scl_ = LRM_->blockscale_;
    map_upbd_ = LRM_->map_upbd_;
    map_lowbd_ = LRM_->map_lowbd_;
    vox_num_ = LRM_->block_num_;
    cur_hid_ = uav_id_;
    cout<<"cur_hid_ 0:"<<cur_hid_<<endl;
    // cout<<"uav_id_ :"<<uav_id_<<endl;
    // cout<<"uav_id_ :"<<uav_id_<<endl;
    if(drone_num_ > 1){
        use_swarm_ = true;
        // swarm_timer_ = nh.createTimer(ros::Duration(SDM_->local_comm_intv_), &MultiDtgPlus::DTGCommunicationCallback, this);
    }
    topo_pub_ = nh.advertise<visualization_msgs::MarkerArray>(ns + "/MR_DTG/Graph", 10);
    debug_pub_ = nh.advertise<visualization_msgs::Marker>(ns + "/MR_DTG/Debug", 10);
    show_timer_ = nh.createTimer(ros::Duration(0.2), &MultiDtgPlus::ShowAll, this);
    if(maintain_h_dist_) {
        maintain_timer_ = nh.createTimer(ros::Duration(0.25), &MultiDtgPlus::DistMaintTimerCallback, this);
        eng_ = default_random_engine(rd_());
    }
    H_depot_.resize(vox_num_(0) * vox_num_(1) * vox_num_(2));
    F_depot_.resize(vox_num_(0) * vox_num_(1) * vox_num_(2));
    

    time_t now = time(0);
    std::string Time_ = ctime(&now);
    tm* t=localtime(&now);
    string path = "/home/charliedog/rosprojects/DoomSea/debug/"+to_string(t->tm_year+1900)+"_"+to_string(t->tm_mon+1)+"_"+to_string(t->tm_mday)
    +"_"+to_string(t->tm_hour)+"_"+to_string(t->tm_min)+"_"+to_string(t->tm_sec);
    if(debug_plan_) debug_f_.open(path+"_dtg_debug"+".txt", std::ios::out); 
}


    