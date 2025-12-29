#include <map_visualization/map_visualization.h>

void MapVisualization::init(ros::NodeHandle &nh, ros::NodeHandle &nh_private){
    std::string ns = ros::this_node::getName();
    nh_private.param("Res", 
        res_, 0.099);
    nh_private.param("MinX", 
        xmin, -500.0);
    nh_private.param("MaxX", 
        xmax, 500.0);
    nh_private.param("MinY", 
        ymin, -500.0);
    nh_private.param("MaxY", 
        ymax, 500.0);
    nh_private.param("MinZ", 
        zmin, -0.2);
    nh_private.param("MaxZ", 
        zmax, 1.5);
    pts_sub_ = nh.subscribe("/makers", 10, &MapVisualization::MarkerCallBack, this);
    pts_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/map_pts", 100);
    pts_timer_ = nh.createTimer(ros::Duration(0.5), &MapVisualization::PtsPubTimer, this);
    // pts_timer_ = nh.createTimer(ros::Duration(7.0), &MapVisualization::PtsPubTimer, this);
    kdTree_ = kd_create(3);
    need_pub_ = false;
}

void MapVisualization::MarkerCallBack(const visualization_msgs::MarkerArrayConstPtr &mka){
    for(const auto &m : mka->markers){
        if(m.action == visualization_msgs::Marker::ADD){
            for(const auto &p : m.points){
                kdres *nearest = kd_nearest_range3(kdTree_, p.x, p.y, p.z, res_);
                if (kd_res_size(nearest) <= 0) {
                    kd_res_free(nearest);
                    if(p.z < zmin || p.z > zmax) continue;
                    if(p.y < ymin || p.y > ymax) continue;
                    if(p.x < xmin || p.x > xmax) continue;
                    bool *b = new bool;
                    kd_insert3(kdTree_, p.x, p.y, p.z, b);
                    pcl::PointXYZ pt;
                    pt.x = p.x;
                    pt.y = p.y;
                    pt.z = p.z;
                    cloud_.emplace_back(pt);
                    need_pub_ = true;
                }
                else{
                    kd_res_free(nearest);
                }
            }
        }
    }
}

void MapVisualization::PtsPubTimer(const ros::TimerEvent &e){
    if(!need_pub_) return;
    cloud_.width = cloud_.points.size();
    cloud_.height = 1;
    cloud_.is_dense = true;
    cloud_.header.frame_id = "world";
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud_, cloud_msg);
    pts_pub_.publish(cloud_msg);
}
