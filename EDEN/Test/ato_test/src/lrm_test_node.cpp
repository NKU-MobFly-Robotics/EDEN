#include <ros/ros.h>
#include <stdio.h>
#include <string>
#include <block_map_lite/block_map_lite.h>
#include <lowres_map_lite/lowres_map_lite.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <glog/logging.h>

BlockMapLite *BM_;
lowres_lite::LowResMap *LRM_;
vector<Eigen::Vector3d> new_free_nodes_;

ros::Timer reload_timer_;
double last_map_update_t_, update_interval_ = 0.1;
double t_total_ = 0;;
int c = 0;
bool have_odom = false;
Eigen::Vector3d cur_p;
void ImgOdomCallback(const sensor_msgs::ImageConstPtr& img,
                            const nav_msgs::OdometryConstPtr& odom){
    double tc = ros::WallTime::now().toSec();

    if(tc - last_map_update_t_ < update_interval_) return;
    last_map_update_t_ = tc; 
    ROS_WARN("update!");

    BM_->OdomCallback(odom);
    BM_->InsertImg(img);
    // BM_->VisTotalMap();
    // BM_->ChangePtsDebug();
    t_total_ += ros::WallTime::now().toSec() - tc;
    c++;
    have_odom = true;
    cur_p(0) = odom->pose.pose.position.x;
    cur_p(1) = odom->pose.pose.position.y;
    cur_p(2) = odom->pose.pose.position.z;
    if(!BM_->changed_pts_.empty()){
        LRM_->SetNodeStatesBf(new_free_nodes_);
        new_free_nodes_.clear();

        // ROS_WARN("check1!");
        // LRM_->CheckThorough();
        BM_->changed_pts_.clear();
    }
}

void ReloadMap(const ros::TimerEvent &e){
    if(!have_odom) return;
    BM_->ResetLocalMap(cur_p);

    // BM_->ChangePtsDebug();
    LRM_->ReloadNodesBf(cur_p);
    LRM_->SetNodeStatesBf(new_free_nodes_);
    new_free_nodes_.clear();

    // ROS_WARN("check2!");
    // LRM_->CheckThorough();
    BM_->changed_pts_.clear();
}

// void UpdateLRM(const ros::TimerEvent &e){
//     LRM_->SetNodeStatesBf();
// }

int main(int argc, char** argv){
    ros::init(argc, argv, "map_test_nod");
    ros::NodeHandle nh, nh_private("~");

    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InstallFailureSignalHandler();

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry>
        SyncPolicyImageOdom;
    typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImageOdom>> SynchronizerImageOdom;
    last_map_update_t_ = ros::WallTime::now().toSec() - 1000.0;
    shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> vi_odom_sub_;
    shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
    SynchronizerImageOdom sync_image_odom_;
    depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh, "/depth", 10));
    vi_odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/vi_odom", 10));
    sync_image_odom_.reset(new message_filters::Synchronizer<SyncPolicyImageOdom>(
        SyncPolicyImageOdom(10000), *depth_sub_, *vi_odom_sub_));
    sync_image_odom_->registerCallback(boost::bind(&ImgOdomCallback,  _1, _2));
    reload_timer_ = nh.createTimer(ros::Duration(0.2), &ReloadMap);
    lowres_lite::LowResMap LRM;
    LRM_ = &LRM;
    // LRM_->AlignInit();

    BlockMapLite BM;
    BM_ = &BM;

    Eigen::Vector3d origin;
    Eigen::Vector3i block_size, block_num, local_block_num;
    LRM_->SetMap(BM_);
    LRM_->AlignInit(nh, nh_private, origin, block_size, block_num, local_block_num);
    SwarmDataManager SDM;
    SDM.init(nh, nh_private);
    BM.SetSwarmDataManager(&SDM);
    BM_->AlignInit(nh, nh_private, origin, block_size, block_num, local_block_num);
    // BM_->init(nh, nh_private);
    ros::spin();
    cout<<"avg t:"<<t_total_ / c<<endl;
    return 0;
}