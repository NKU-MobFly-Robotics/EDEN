#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CompressedImage.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// #include<block_map/block_map.h>
// #include<murder/murder.h>
using namespace std;
Eigen::Vector3d up, down;
double colorhsize_;
std::vector<std_msgs::ColorRGBA> color_list_;
ros::Subscriber vis_sub_, odom_sub_, show_sub_, compressed_sub_;
ros::Publisher vis_pub_, odom_pub_, pose_vis_pub_, show_pub_, image_pub_;
tf::TransformBroadcaster *br_;
geometry_msgs::Pose pose_;
visualization_msgs::MarkerArray vis_models_, vis_traj_;
int traj_pt_count_ = 10;
double v_max_, vel_cur_ = 0.0;
Eigen::Vector3d lp(0,0,99999.0);
bool pub_traj_, first_odom_ = true;
// BlockMap *VM_;
// double last_map_update_t_, update_interval_ = 0.1;
// void ImgOdomCallback(const sensor_msgs::ImageConstPtr& img,
//                             const nav_msgs::OdometryConstPtr& odom){
//     double tc = ros::WallTime::now().toSec();

//     if(tc - last_map_update_t_ < update_interval_) return;
//     last_map_update_t_ = tc; 
//     // ros::WallTime t0 = ros::WallTime::now();
//     // ROS_WARN("update!");
//     VM_->OdomCallback(odom);
//     VM_->InsertImg(img);
// }


// inline std_msgs::ColorRGBA Getcolor(const double z){
//     std_msgs::ColorRGBA color;
//     double difz = z - down(2);
//     color.a = 1.0;
//     if(difz > up(2) - down(2)){
//         return color_list_.back();
//     }
//     else if(difz < 0.0){
//         return color_list_.front();
//     }
//     else{
        
//         int hieghtf = floor(difz / colorhsize_);
//         int hieghtc = hieghtf + 1;
//         double gain = (difz - colorhsize_*double(hieghtf))/colorhsize_;
//         // cout<<"gain:"<<gain<<endl;
//         // cout<<"hieghtf:"<<hieghtf<<endl;
//         // cout<<"hieghtc:"<<hieghtc<<endl;
//         color.r = color_list_[hieghtf].r*(1.0-gain) + color_list_[hieghtc].r*gain;
//         color.g = color_list_[hieghtf].g*(1.0-gain) + color_list_[hieghtc].g*gain;
//         color.b = color_list_[hieghtf].b*(1.0-gain) + color_list_[hieghtc].b*gain;
//     }
//     return color;
// }


// void MarkerCallbackMap(const visualization_msgs::MarkerArray &msg){
//     visualization_msgs::MarkerArray mka;
//     mka.markers.resize(msg.markers.size());
//     for(int i = 0; i < msg.markers.size(); i++){
//         mka.markers[i].action = msg.markers[i].action;
//         mka.markers[i].pose = msg.markers[i].pose;
//         mka.markers[i].type = msg.markers[i].type;
//         mka.markers[i].scale = msg.markers[i].scale;
//         mka.markers[i].header.frame_id = msg.markers[i].header.frame_id;
//         mka.markers[i].header.stamp = ros::Time::now();
//         mka.markers[i].id = msg.markers[i].id;
//         for(int j = 0; j < msg.markers[i].points.size(); j++){
//             geometry_msgs::Point pt = msg.markers[i].points[j];
//             if(pt.x > up(0) || pt.y > up(1) || pt.z > up(2) ||
//                 pt.x < down(0) || pt.y < down(1)|| pt.z < down(2)) continue;
            
//             mka.markers[i].points.emplace_back(pt);
//             mka.markers[i].colors.push_back(Getcolor(pt.z));
//         }

//     }
//     vis_pub_.publish(mka);
// }

// void MarkerCallbackShow(const visualization_msgs::MarkerArray &msg){
//     visualization_msgs::MarkerArray mka;
//     // if(msg.markers.size() == 0) return;
//     // mka.markers.resize(1);
//     // mka.markers[0].action = visualization_msgs::Marker::DELETEALL;
//     // show_pub_.publish(mka);

//     // mka.markers.resize(msg.markers.size());
//     mka.markers.clear();
//     // mka = msg;
//     for(int i = 0; i < msg.markers.size(); i++){
//         if(msg.markers[i].id == 99 || msg.markers[i].id == 98) continue;
//         mka.markers.emplace_back(msg.markers[i]);
//         // mka.markers[i].action = msg.markers[i].action;
//         // mka.markers[i].pose = msg.markers[i].pose;
//         // mka.markers[i].type = msg.markers[i].type;
//         // mka.markers[i].scale = msg.markers[i].scale;
//         // mka.markers[i].header.frame_id = msg.markers[i].header.frame_id;
//         if(mka.markers[i].type == visualization_msgs::Marker::CUBE){
//             mka.markers[i].color.r = 1.0;
//             mka.markers[i].color.g = 0.3;
//         }

//         mka.markers[i].header.stamp = ros::Time::now();
//         if(mka.markers[i].id == -1){
//             mka.markers[i].type = visualization_msgs::Marker::SPHERE_LIST;
//             mka.markers[i].scale.x = 0.15;
//             mka.markers[i].scale.y = 0.15;
//             mka.markers[i].scale.z = 0.15;
//             mka.markers[i].color.r = 1.0;
//             mka.markers[i].color.g = 0.3;
//             mka.markers[i].color.a = 1.0;
//         }
//         // mka.markers[i].id = msg.markers[i].id;
//         mka.markers[i].lifetime = ros::Duration(100.0);
//     }
//     show_pub_.publish(mka);
// }

void V2Color(double v, float &r, float &g, float &b){
    // v_max_
    if(v < v_max_ * 0.25){
        r = 0;
        g = (v) / (v_max_ * 0.25);
        b = 1.0;
    }
    else if(v < v_max_ * 0.5){
        r = 0;
        g = 1.0;
        b = (v_max_ * 0.5 - v) / (v_max_ * 0.25);
    }
    else if(v < v_max_ * 0.75){
        r = (v - v_max_ * 0.5) / (v_max_ * 0.25);
        g = 1.0;
        b = 0.0;
    }
    else{
        r = 1.0;
        g = (v_max_ - v) / (v_max_ * 0.25);
        b = 0.0;
    }
    // cout<<"vel:"<<sqrt(pow(msg.twist.twist.linear.x, 2)+pow(msg.twist.twist.linear.y, 2)+pow(msg.twist.twist.linear.z, 2))<<endl;
    // cout<<"vel:"<<v<<endl;

    //     cout<<"r:"<<r<<endl;
    //     cout<<"g:"<<g<<endl;
    //     cout<<"b:"<<b<<endl;

}

inline void LoadVisModels(){





    traj_pt_count_--;


    double cur_t = ros::WallTime::now().toSec();
    int i = 0;
    vis_models_.markers[i*5 + 0].header.stamp = ros::Time::now();
    vis_models_.markers[i*5 + 1].header.stamp = ros::Time::now();
    vis_models_.markers[i*5 + 2].header.stamp = ros::Time::now();
    vis_models_.markers[i*5 + 3].header.stamp = ros::Time::now();
    vis_models_.markers[i*5 + 4].header.stamp = ros::Time::now();

    vis_models_.markers[i*5 + 0].pose.orientation = pose_.orientation;
    vis_models_.markers[i*5 + 1].pose.orientation = pose_.orientation;
    vis_models_.markers[i*5 + 2].pose.orientation = pose_.orientation;
    vis_models_.markers[i*5 + 3].pose.orientation = pose_.orientation;
    vis_models_.markers[i*5 + 4].pose = pose_;

    // vis_traj_.markers.resize(1);
    vis_models_.markers[5].header.frame_id = "world";
    vis_models_.markers[5].header.stamp = ros::Time::now();
    vis_models_.markers[5].id = traj_pt_count_;
    if(pub_traj_)  vis_models_.markers[5].action = visualization_msgs::Marker::ADD;
    else{
        vis_models_.markers[5].action = visualization_msgs::Marker::DELETE;
        vis_models_.markers[5].id = -10000;
    }
    vis_models_.markers[5].type = visualization_msgs::Marker::SPHERE;
    // vis_models_.markers[5].scale.x = 0.55;
    // vis_models_.markers[5].scale.y = 0.55;
    // vis_models_.markers[5].scale.z = 0.55;
    vis_models_.markers[5].scale.x = 0.25;
    vis_models_.markers[5].scale.y = 0.25;
    vis_models_.markers[5].scale.z = 0.25;
    geometry_msgs::Point pu;
    while (vis_models_.markers.size() > 6)vis_models_.markers.pop_back();
    V2Color(vel_cur_, vis_models_.markers[5].color.r, vis_models_.markers[5].color.g, vis_models_.markers[5].color.b);

    // vis_models_.markers[5].pose.position.x = lp.x();
    // vis_models_.markers[5].pose.position.y = lp.y();
    // vis_models_.markers[5].pose.position.z = lp.z();
    // vis_models_.markers[5].pose.orientation.x = 0.0;
    // vis_models_.markers[5].pose.orientation.y = 0.0;
    // vis_models_.markers[5].pose.orientation.z = 0.0;
    // vis_models_.markers[5].pose.orientation.w = 1.0;
    vis_models_.markers[5].color.a = 0.80;

    Eigen::Vector3d pc;
    pc.x() = pose_.position.x;
    pc.y() = pose_.position.y;
    pc.z() = pose_.position.z;
    int sample_num = floor((pc - lp).norm() / 0.15);
    Eigen::Vector3d dir = (pc - lp) / sample_num;
    for(int i = 0; i < sample_num; i++){
        Eigen::Vector3d p = lp + dir * i;
        pu.x = p.x();
        pu.y = p.y();
        pu.z = p.z();
        if(i > 0){
            traj_pt_count_--;
            vis_models_.markers.emplace_back(vis_models_.markers[5]);
            vis_models_.markers.back().id = traj_pt_count_;
        }
        vis_models_.markers.back().pose.position = pu;
    }

    // vis_models_.markers[5].color.r = 0.0/255.0 * (1 - vel_cur_ * vel_cur_ / v_max_ / v_max_) + 255.0/255.0 * vel_cur_ * vel_cur_ / v_max_ / v_max_;
    // vis_models_.markers[5].color.g = 255.0/255.0 * (1 - vel_cur_ * vel_cur_ / v_max_ / v_max_) + 0.0/255.0 * vel_cur_ * vel_cur_ / v_max_ / v_max_;
    // vis_models_.markers[5].color.b = 125.0/255.0 * (1 - vel_cur_ * vel_cur_ / v_max_ / v_max_) + 0.0/255.0 * vel_cur_ * vel_cur_ / v_max_ / v_max_;



    Eigen::Quaterniond rot;
    rot.x() = pose_.orientation.x;
    rot.y() = pose_.orientation.y;
    rot.z() = pose_.orientation.z;
    rot.w() = pose_.orientation.w;
    Eigen::Vector3d pos;
    pos(0) = pose_.position.x;
    pos(1) = pose_.position.y;
    pos(2) = pose_.position.z;

    Eigen::Vector3d p(0.15, 0.15, -0.02);
    vector<Eigen::Vector3d> pl;
    pl.emplace_back(p);
    p(0) = -p(0);
    pl.emplace_back(p);
    p(1) = -p(1);
    pl.emplace_back(p);
    p(0) = -p(0);
    pl.emplace_back(p);

    for(int j = 0; j < 4; j++){
        p = rot.toRotationMatrix() * pl[j] + pos;
        vis_models_.markers[i*5 + j].pose.position.x = p(0);
        vis_models_.markers[i*5 + j].pose.position.y = p(1);
        vis_models_.markers[i*5 + j].pose.position.z = p(2);
    }
}
void CreateVisModels(){


    // double vel = sqrt()




    vis_models_.markers.resize(6 * 1);
    // for(int i = 0; i < SDM_.drone_num_; i++){
    int i = 0;
        vis_models_.markers[i*5 + 0].header.frame_id = "world";
        vis_models_.markers[i*5 + 0].header.stamp = ros::Time::now();
        vis_models_.markers[i*5 + 0].id = i*5 + 0;
        vis_models_.markers[i*5 + 0].action = visualization_msgs::Marker::ADD;
        vis_models_.markers[i*5 + 0].type = visualization_msgs::Marker::SPHERE;
        vis_models_.markers[i*5 + 0].scale.x = 0.18;
        vis_models_.markers[i*5 + 0].scale.y = 0.18;
        vis_models_.markers[i*5 + 0].scale.z = 0.04;
        vis_models_.markers[i*5 + 0].color.r = 0.5;
        vis_models_.markers[i*5 + 0].color.g = 1.0;
        vis_models_.markers[i*5 + 0].color.b = 0.0;
        vis_models_.markers[i*5 + 0].color.a = 1.0;
        pose_.orientation.w = 1.0;
        pose_.orientation.x = 0.0;
        pose_.orientation.y = 0.0;
        pose_.orientation.z = 0.0;
        pose_.position.x = 0.0;
        pose_.position.y = 0.0;
        pose_.position.z = 0.0;

        vis_models_.markers[i*5 + 1] = vis_models_.markers[i*5 + 0];
        vis_models_.markers[i*5 + 1].id = i*5 + 1;
        vis_models_.markers[i*5 + 2] = vis_models_.markers[i*5 + 0];
        vis_models_.markers[i*5 + 2].id = i*5 + 2;
        vis_models_.markers[i*5 + 3] = vis_models_.markers[i*5 + 0];
        vis_models_.markers[i*5 + 3].id = i*5 + 3;

        vis_models_.markers[i*5 + 4] = vis_models_.markers[i*5 + 0];
        vis_models_.markers[i*5 + 4].id = i*5 + 4;
        vis_models_.markers[i*5 + 4].type = visualization_msgs::Marker::LINE_LIST;
        vis_models_.markers[i*5 + 4].scale.x = 0.025;
        vis_models_.markers[i*5 + 4].scale.y = 0.025;
        vis_models_.markers[i*5 + 4].scale.z = 0.025;

        geometry_msgs::Point pt;

        pt.x = 0.15;
        pt.y = 0.15;
        pt.z = -0.02;
        vis_models_.markers[i*5 + 4].points.emplace_back(pt);
        pt.y = -0.15;
        pt.x = -0.15;
        vis_models_.markers[i*5 + 4].points.emplace_back(pt);
        pt.x = -0.15;
        pt.y = 0.15;
        vis_models_.markers[i*5 + 4].points.emplace_back(pt);
        pt.x = 0.15;
        pt.y = -0.15;
        vis_models_.markers[i*5 + 4].points.emplace_back(pt);
    // }
}

void OdomCallback(const nav_msgs::Odometry &msg){
    nav_msgs::Odometry odom;
    odom = msg;
    odom.header.frame_id = "world";
    odom.header.stamp = ros::Time::now();
    odom_pub_.publish(odom);

    pose_ = msg.pose.pose;

    tf::Transform trans;
    tf::Quaternion q;
    q.setX(odom.pose.pose.orientation.x);
    q.setY(odom.pose.pose.orientation.y);
    q.setZ(odom.pose.pose.orientation.z);
    q.setW(odom.pose.pose.orientation.w);
    trans.setRotation(q);
    trans.setOrigin(tf::Vector3(odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z));
    br_->sendTransform(tf::StampedTransform(trans, ros::Time::now(), "world", "UAV_base"));
    Eigen::Vector3d pc;
    pc(0) = odom.pose.pose.position.x;
    pc(1) = odom.pose.pose.position.y;
    pc(2) = odom.pose.pose.position.z;


    if((pc - lp).norm() > 0.15 && !first_odom_ && (pc - lp).norm() < 7.15){
    // if((pc - lp).norm() > 0.40){
        // pose_vis_pub_.publish(vis_models_);
        // ros::Duration(0.0025).sleep();
        vel_cur_ = min(sqrt(pow(msg.twist.twist.linear.x, 2)+pow(msg.twist.twist.linear.y, 2)+pow(msg.twist.twist.linear.z, 2)), v_max_);
        
        pub_traj_ = true;

    }
    else{
        if((pc - lp).norm() > 7.15) lp = pc;
        pub_traj_ = false;
    }
    if(first_odom_) first_odom_ = false;
    LoadVisModels();
    if(pub_traj_) lp = pc;
    pose_vis_pub_.publish(vis_models_);

}

    void compressedCallback(const sensor_msgs::CompressedImageConstPtr& msg)
    {
        try
        {
            // 将压缩图像解码为OpenCV格式
            cout<<"img"<<endl;
            cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
            
            // 创建sensor_msgs::Image消息
            sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(
                msg->header, "bgr8", image).toImageMsg();
            
            // 发布普通图像
            image_pub_.publish(image_msg);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

int main(int argc, char** argv){
    ros::init(argc, argv, "demo");
    ros::NodeHandle nh, nh_private_("~");
    vector<double> CR, CB, CG;
    std::string ns = ros::this_node::getName();
    // nh_private_.param(ns + "/block_map/HeightcolorR", 
    //     CR, {});
    // nh_private_.param(ns + "/block_map/HeightcolorG", 
    //     CG, {});
    // nh_private_.param(ns + "/block_map/HeightcolorB", 
    //     CB, {});
    // nh_private_.param(ns + "/block_map/minX", 
    //     down.x(), -10.0);
    // nh_private_.param(ns + "/block_map/minY", 
    //     down.y(), -10.0);
    // nh_private_.param(ns + "/block_map/minZ", 
    //     down.z(), 0.0);
    // nh_private_.param(ns + "/block_map/maxX", 
    //     up.x(), 10.0);
    // nh_private_.param(ns + "/block_map/maxY", 
    //     up.y(), 10.0);
    // nh_private_.param(ns + "/block_map/maxZ", 
    //     up.z(), 0.0);
    nh_private_.param("VelMax", 
        v_max_, 4.0);
    cout<<"======================v_max_:"<<v_max_<<endl;
    std_msgs::ColorRGBA color;
    colorhsize_ = (up(2) - down(2)) / (CG.size() - 1);
    for(int i = 0; i < CG.size(); i++){
        color.a = 1.0;
        color.r = CR[i]/255;
        color.g = CG[i]/255;
        color.b = CB[i]/255;
        color_list_.push_back(color);
    }

    br_ = new tf::TransformBroadcaster;


    // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry>
    //     SyncPolicyImageOdom;
    // typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImageOdom>> SynchronizerImageOdom;
    // last_map_update_t_ = ros::WallTime::now().toSec() - 1000.0;
    // shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> vi_odom_sub_;
    // shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
    // SynchronizerImageOdom sync_image_odom_;
    // depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh, "/depth", 10));
    // vi_odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/vi_odom", 10));
    // sync_image_odom_.reset(new message_filters::Synchronizer<SyncPolicyImageOdom>(
    //     SyncPolicyImageOdom(10000), *depth_sub_, *vi_odom_sub_));
    // sync_image_odom_->registerCallback(boost::bind(&ImgOdomCallback,  _1, _2));
    // BlockMap VM;
    // VM_ = &VM;
    // VM_->init(nh, nh_private_);
    // Murder M_planner_;
    // M_planner_.init(nh, nh_private_);

    // vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/replay/map", 10);
    // vis_sub_ = nh.subscribe("vis_sub", 100000000, &MarkerCallbackMap);
    // show_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/replay/show", 10);
    // show_sub_ = nh.subscribe("show", 10, &MarkerCallbackShow);
    first_odom_ = true;
    compressed_sub_ = nh.subscribe("/camera/image/compressed", 1, 
                                    compressedCallback);

    image_pub_ = nh.advertise<sensor_msgs::Image>("/camera/Img", 10);
    odom_pub_ = nh.advertise<nav_msgs::Odometry>("/replay/odom", 10);
    pose_vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>(ns + "/Poses", 5);
    CreateVisModels();
    odom_sub_ = nh.subscribe("odomshow", 50000000, &OdomCallback);
    ros::spin();
    return 0;
}
