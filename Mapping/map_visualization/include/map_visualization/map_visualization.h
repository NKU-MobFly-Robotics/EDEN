#ifndef MAP_VISUALIZATION_H_
#define MAP_VISUALIZATION_H_
#include <ros/ros.h>
#include <thread>
#include <Eigen/Eigen>
#include <vector>
#include <list>
#include <visualization_msgs/MarkerArray.h>
#include <tr1/unordered_map>
#include <sensor_msgs/PointCloud2.h>
#include <queue>
#include <random>
#include <mutex>
#include <bitset>
#include <pcl_conversions/pcl_conversions.h>

#include <glog/logging.h>
#include <kdtree/kdtree.h>

using namespace std;
class MapVisualization{
public:
    MapVisualization(){};
    ~MapVisualization(){
        kd_free(kdTree_);
    };

    void init(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
    void MarkerCallBack(const visualization_msgs::MarkerArrayConstPtr &mka);
private:
    void PtsPubTimer(const ros::TimerEvent &e);
    ros::Subscriber pts_sub_;
    ros::Publisher pts_pub_;
    ros::Timer pts_timer_;
    kdtree *kdTree_;
    pcl::PointCloud<pcl::PointXYZ> cloud_;
    double res_;
    double xmin, xmax;
    double ymin, ymax;
    double zmin, zmax;
    bool need_pub_;
};


#endif