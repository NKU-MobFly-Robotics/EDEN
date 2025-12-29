#include <cmath>
#include <fstream>
#include <iostream>
#include <random>
#include <vector>

#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Eigen>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "depth_render.cuh"

using namespace cv;
using namespace std;
using namespace Eigen;

class PointcloudRender {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<PointcloudRender> Ptr;
  typedef std::shared_ptr<const PointcloudRender> ConstPtr;

  void initialize(ros::NodeHandle &nh);

  void setPointcloud(const pcl::PointCloud<pcl::PointXYZ> &cloud, bool add_floor = false);
  void odometryCallbck(const nav_msgs::Odometry &odom);

  void pubCameraPose(const ros::TimerEvent &event);

  void renderCallback(const ros::TimerEvent &event);
  void renderDepthImage();
  void renderDepthPointcloud();

private:
  int *depth_hostptr;
  cv::Mat depth_mat;

  // camera param
  int width, height;
  double fx, fy, cx, cy;

  DepthRender depthrender;
  ros::Publisher pub_depth;
  ros::Publisher pub_color;
  ros::Publisher pub_pose;
  ros::Publisher pub_pcl_wolrd;
  ros::Publisher pub_caminfo;
  ros::Publisher pub_odom;

  sensor_msgs::PointCloud2 local_map_pcl;
  sensor_msgs::PointCloud2 local_depth_pcl;

  ros::Subscriber odom_sub;
  ros::Subscriber global_map_sub, local_map_sub;

  ros::Timer local_sensing_timer, estimation_timer;

  bool init_pose, init_cloud;

  Matrix4d cam02body;
  Matrix4d cam2world;
  Eigen::Quaterniond cam2world_quat;
  nav_msgs::Odometry _odom;
  geometry_msgs::TransformStamped camera_pose_;

  double render_rate, sensor_pose_rate;
  double inflate_radius;
  double sensing_horizon;
  double _resolution, _inv_resolution;

  ros::Time last_odom_stamp = ros::TIME_MAX;
  Eigen::Vector3d last_pose_world;

  vector<float> cloud_data;

  bool verbose;
};