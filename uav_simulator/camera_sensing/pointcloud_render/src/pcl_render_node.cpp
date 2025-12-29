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
#include "quadrotor_msgs/PositionCommand.h"

using namespace cv;
using namespace std;
using namespace Eigen;

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

sensor_msgs::PointCloud2 local_map_pcl;
sensor_msgs::PointCloud2 local_depth_pcl;

ros::Subscriber odom_sub;
ros::Subscriber global_map_sub, local_map_sub;

ros::Timer local_sensing_timer, estimation_timer;

bool has_global_map(false);
bool has_local_map(false);
bool has_odom(false);
bool init(false);

Matrix4d cam02body;
Matrix4d cam2world;
Eigen::Quaterniond cam2world_quat;
nav_msgs::Odometry _odom;

double sensing_horizon, sensing_rate, estimation_rate, point_inflate_radius;
double _x_size, _y_size, _z_size;
double _gl_xl, _gl_yl, _gl_zl;
double _resolution, _inv_resolution;
int _GLX_SIZE, _GLY_SIZE, _GLZ_SIZE;

ros::Time last_odom_stamp = ros::TIME_MAX;
Eigen::Vector3d last_pose_world;

cv::Mat noise_mask_;

void render_currentpose();
void render_pcl_world();

std::unique_ptr<tf2_ros::TransformListener> tf_listener_ptr_;
tf2_ros::Buffer tf_buffer_;

inline Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i &index) {
  Eigen::Vector3d pt;
  pt(0) = ((double)index(0) + 0.5) * _resolution + _gl_xl;
  pt(1) = ((double)index(1) + 0.5) * _resolution + _gl_yl;
  pt(2) = ((double)index(2) + 0.5) * _resolution + _gl_zl;

  return pt;
};

inline Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d &pt) {
  Eigen::Vector3i idx;
  idx(0) = std::min(std::max(int((pt(0) - _gl_xl) * _inv_resolution), 0), _GLX_SIZE - 1);
  idx(1) = std::min(std::max(int((pt(1) - _gl_yl) * _inv_resolution), 0), _GLY_SIZE - 1);
  idx(2) = std::min(std::max(int((pt(2) - _gl_zl) * _inv_resolution), 0), _GLZ_SIZE - 1);

  return idx;
};

void rcvOdometryCallbck(const nav_msgs::Odometry &odom) {
  /*if(!has_global_map)
    return;*/
  has_odom = true;
  _odom = odom;
  Matrix4d Pose_receive = Matrix4d::Identity();

  Eigen::Vector3d request_position;
  Eigen::Quaterniond request_pose;
  request_position.x() = odom.pose.pose.position.x;
  request_position.y() = odom.pose.pose.position.y;
  request_position.z() = odom.pose.pose.position.z;
  request_pose.x() = odom.pose.pose.orientation.x;
  request_pose.y() = odom.pose.pose.orientation.y;
  request_pose.z() = odom.pose.pose.orientation.z;
  request_pose.w() = odom.pose.pose.orientation.w;
  Pose_receive.block<3, 3>(0, 0) = request_pose.toRotationMatrix();
  Pose_receive(0, 3) = request_position(0);
  Pose_receive(1, 3) = request_position(1);
  Pose_receive(2, 3) = request_position(2);

  Matrix4d body_pose = Pose_receive;
  // convert to cam pose
  cam2world = body_pose * cam02body;
  cam2world_quat = cam2world.block<3, 3>(0, 0);
  if (!init)
    init = true;

  last_odom_stamp = odom.header.stamp;

  last_pose_world(0) = odom.pose.pose.position.x;
  last_pose_world(1) = odom.pose.pose.position.y;
  last_pose_world(2) = odom.pose.pose.position.z;

  // publish tf
  /*static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(cam2world(0,3), cam2world(1,3), cam2world(2,3) ));
  transform.setRotation(tf::Quaternion(cam2world_quat.x(), cam2world_quat.y(), cam2world_quat.z(),
  cam2world_quat.w()));
  br.sendTransform(tf::StampedTransform(transform, last_odom_stamp, "world", "camera")); //publish
  transform from world frame to quadrotor frame.*/
}

void pubCameraPose(const ros::TimerEvent &event) {
  if (!init)
    return;
  geometry_msgs::TransformStamped camera_pose;
  camera_pose.header = _odom.header;
  camera_pose.header.frame_id = "world";
  camera_pose.child_frame_id = "camera";
  camera_pose.transform.translation.x = cam2world(0, 3);
  camera_pose.transform.translation.y = cam2world(1, 3);
  camera_pose.transform.translation.z = cam2world(2, 3);
  camera_pose.transform.rotation.x = cam2world_quat.x();
  camera_pose.transform.rotation.y = cam2world_quat.y();
  camera_pose.transform.rotation.z = cam2world_quat.z();
  camera_pose.transform.rotation.w = cam2world_quat.w();
  pub_pose.publish(camera_pose);

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;

  transform.setOrigin(tf::Vector3(cam2world(0, 3), cam2world(1, 3), cam2world(2, 3)));
  q.setW(cam2world_quat.w());
  q.setX(cam2world_quat.x());
  q.setY(cam2world_quat.y());
  q.setZ(cam2world_quat.z());
  transform.setRotation(q);
  // br.sendTransform(tf::StampedTransform(transform, last_odom_stamp, "world", "camera"));
}

void renderSensedPoints(const ros::TimerEvent &event) {
  if (!has_global_map && !has_local_map)
    return;

  render_currentpose();
  // render_pcl_world();
}

vector<float> cloud_data;
void rcvGlobalPointCloudCallBack(const sensor_msgs::PointCloud2 &pointcloud_map) {
  if (has_global_map)
    return;

  ROS_WARN("[PCL Render] Global pointcloud received");
  // load global map
  pcl::PointCloud<pcl::PointXYZ> cloudIn;
  pcl::PointXYZ pt_in;
  // transform map to point cloud format
  pcl::fromROSMsg(pointcloud_map, cloudIn);
  for (int i = 0; i < int(cloudIn.points.size()); i++) {
    pt_in = cloudIn.points[i];
    cloud_data.push_back(pt_in.x);
    cloud_data.push_back(pt_in.y);
    cloud_data.push_back(pt_in.z);
  }
  // printf("global map has points: %d.\n", (int)cloud_data.size() / 3);
  // pass cloud_data to depth render
  depthrender.set_data(cloud_data);
  depth_hostptr = (int *)malloc(width * height * sizeof(int));

  has_global_map = true;
}

void rcvLocalPointCloudCallBack(const sensor_msgs::PointCloud2 &pointcloud_map) {
  // ROS_WARN("Local Pointcloud received..");
  // load local map
  pcl::PointCloud<pcl::PointXYZ> cloudIn;
  pcl::PointXYZ pt_in;
  // transform map to point cloud format
  pcl::fromROSMsg(pointcloud_map, cloudIn);

  if (cloudIn.points.size() == 0)
    return;
  for (int i = 0; i < int(cloudIn.points.size()); i++) {
    pt_in = cloudIn.points[i];
    Eigen::Vector3d pose_pt(pt_in.x, pt_in.y, pt_in.z);
    // pose_pt = gridIndex2coord(coord2gridIndex(pose_pt));
    cloud_data.push_back(pose_pt(0));
    cloud_data.push_back(pose_pt(1));
    cloud_data.push_back(pose_pt(2));
  }
  // printf("local map has points: %d.\n", (int)cloud_data.size() / 3 );
  // pass cloud_data to depth render
  depthrender.set_data(cloud_data);
  depth_hostptr = (int *)malloc(width * height * sizeof(int));

  has_local_map = true;
}

void render_pcl_world() {
  // for debug purpose
  pcl::PointCloud<pcl::PointXYZ> localMap;
  pcl::PointXYZ pt_in;

  Eigen::Vector4d pose_in_camera;
  Eigen::Vector4d pose_in_world;
  Eigen::Vector3d pose_pt;

  for (int u = 0; u < width; u++)
    for (int v = 0; v < height; v++) {
      float depth = depth_mat.at<float>(v, u);

      if (depth == 0.0)
        continue;

      pose_in_camera(0) = (u - cx) * depth / fx;
      pose_in_camera(1) = (v - cy) * depth / fy;
      pose_in_camera(2) = depth;
      pose_in_camera(3) = 1.0;

      pose_in_world = cam2world * pose_in_camera;

      if ((pose_in_world.segment(0, 3) - last_pose_world).norm() > sensing_horizon)
        continue;

      pose_pt = pose_in_world.head(3);
      // pose_pt = gridIndex2coord(coord2gridIndex(pose_pt));
      pt_in.x = pose_pt(0);
      pt_in.y = pose_pt(1);
      pt_in.z = pose_pt(2);

      localMap.points.push_back(pt_in);
    }

  localMap.width = localMap.points.size();
  localMap.height = 1;
  localMap.is_dense = true;

  pcl::toROSMsg(localMap, local_map_pcl);
  local_map_pcl.header.frame_id = "/map";
  local_map_pcl.header.stamp = last_odom_stamp;

  pub_pcl_wolrd.publish(local_map_pcl);
}

void render_currentpose() {
  double this_time = ros::Time::now().toSec();

  Matrix4d cam_pose = cam2world.inverse();

  double pose[4 * 4];

  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++) {
      // pose[j + 4 * i] = cam_pose(i, j);
      pose[j + 4 * i] = cam_pose(i, j);
    }

  depthrender.render_pose(pose, depth_hostptr);
  // depthrender.render_pose(cam_pose, depth_hostptr);

  depth_mat = cv::Mat::zeros(height, width, CV_32FC1);
  double min = 0.5;
  double max = 1.0f;
  for (int i = 0; i < height; i++)
    for (int j = 0; j < width; j++) {
      float depth = (float)depth_hostptr[i * width + j] / 1000.0f;
      depth = depth < 500.0f ? depth : 500.0f;
      max = depth > max ? depth : max;
      depth_mat.at<float>(i, j) = depth;
    }
  // ROS_INFO("render cost %lf ms.", (ros::Time::now().toSec() - this_time) * 1000.0f);
  // printf("max_depth %lf.\n", max);

  // noisy depth image
  // cv::Mat depth_mat_noisy = cv::Mat::zeros(height, width, CV_32FC1);
  // cv::Mat noise_mat = cv::Mat::zeros(height, width, CV_32FC1);
  // for (int i = 0; i < height; i++)
  //   for (int j = 0; j < width; j++) {
  //     // float noise = noise_mask_.at<float>(i, j);
  //     float depth = depth_mat.at<float>(i, j);
  //     float noise_sd = 0.001063 + 0.0007278 * depth + 0.003949 * depth * depth;
  //     float noise_mean = 0.0;
  //     std::random_device rd{};
  //     std::mt19937 gen{rd()};
  //     std::normal_distribution<float> d{noise_mean, noise_sd};
  //     float noise = d(gen);
  //     noise_mat.at<float>(i, j) = noise;
  //   }
  // depth_mat_noisy = depth_mat + noise_mat;

  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = last_odom_stamp;
  out_msg.header.frame_id = "SQ01s/camera";
  out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  out_msg.image = depth_mat.clone();
  // out_msg.image = depth_mat_noisy.clone();
  pub_depth.publish(out_msg.toImageMsg());

  cv::Mat adjMap;
  // depth_mat.convertTo(adjMap,CV_8UC1, 255 / (max-min), -min);
  depth_mat.convertTo(adjMap, CV_8UC1, 255 / 13.0, -min);
  cv::Mat falseColorsMap;
  cv::applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_RAINBOW);
  cv_bridge::CvImage cv_image_colored;
  cv_image_colored.header.frame_id = "depthmap";
  cv_image_colored.header.stamp = last_odom_stamp;
  cv_image_colored.encoding = sensor_msgs::image_encodings::BGR8;
  cv_image_colored.image = falseColorsMap;
  pub_color.publish(cv_image_colored.toImageMsg());
  // cv::imshow("depth_image", adjMap);

  // For running Faster code, FST
  sensor_msgs::CameraInfo camera_info;
  camera_info.header.stamp = last_odom_stamp;
  camera_info.header.frame_id = "SQ01s/camera";
  camera_info.width = width;
  camera_info.height = height;
  camera_info.K[0] = fx;
  camera_info.K[4] = fy;
  camera_info.K[2] = cx;
  camera_info.K[5] = cy;
  camera_info.binning_x = 0;
  camera_info.binning_y = 0;
  pub_caminfo.publish(camera_info);

  // // For running EWOK code
  // static tf::TransformBroadcaster br;
  // tf::Transform transform;
  // transform.setOrigin(tf::Vector3(cam2world(0, 3), cam2world(1, 3), cam2world(2, 3)));
  // transform.setRotation(
  //     tf::Quaternion(cam2world_quat.x(), cam2world_quat.y(), cam2world_quat.z(),
  //     cam2world_quat.w()));
  // br.sendTransform(tf::StampedTransform(transform, last_odom_stamp, "world", "SQ01s/camera"));
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pcl_render");
  ros::NodeHandle nh("~");

  nh.getParam("cam_width", width);
  nh.getParam("cam_height", height);
  nh.getParam("cam_fx", fx);
  nh.getParam("cam_fy", fy);
  nh.getParam("cam_cx", cx);
  nh.getParam("cam_cy", cy);
  nh.getParam("sensing_horizon", sensing_horizon);
  nh.getParam("sensing_rate", sensing_rate);
  nh.getParam("estimation_rate", estimation_rate);
  nh.getParam("point_inflate_radius", point_inflate_radius);

  vector<string> axis = {"x", "y", "z"};
  Eigen::Vector3d map_min, map_max;
  for (int i = 0; i < 3; ++i) {
    nh.getParam("/map_config/map_size/map_min_" + axis[i], map_min[i]);
    nh.getParam("/map_config/map_size/map_max_" + axis[i], map_max[i]);
  }
  _x_size = map_max(0) - map_min(0);
  _y_size = map_max(1) - map_min(1);
  _z_size = map_max(2) - map_min(2);

  noise_mask_ = cv::Mat::zeros(height, width, CV_32FC1);
  // gaussian noise mask for depth image on each pixel
  cv::randu(noise_mask_, -0.5, 0.5);

  depthrender.set_para(fx, fy, cx, cy, width, height, point_inflate_radius);

  // cam02body <<  0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
  //               0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
  //               -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
  //               0.0, 0.0, 0.0, 1.0;

  cam02body << 0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

  // init cam2world transformation
  cam2world = Matrix4d::Identity();
  // subscribe point cloud
  global_map_sub = nh.subscribe("global_map", 1, rcvGlobalPointCloudCallBack);
  local_map_sub = nh.subscribe("local_map", 1, rcvLocalPointCloudCallBack);
  odom_sub = nh.subscribe("odometry", 50, rcvOdometryCallbck);

  // publisher depth image and color image
  pub_depth = nh.advertise<sensor_msgs::Image>("/uav_simulator/depth_image", 1000);
  pub_color = nh.advertise<sensor_msgs::Image>("colordepth", 1000);
  pub_pose = nh.advertise<geometry_msgs::TransformStamped>("/uav_simulator/sensor_pose", 1000);
  pub_pcl_wolrd = nh.advertise<sensor_msgs::PointCloud2>("rendered_pcl", 1);
  pub_caminfo = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 10);

  double sensing_duration = 1.0 / sensing_rate;
  double estimate_duration = 1.0 / estimation_rate;

  local_sensing_timer = nh.createTimer(ros::Duration(sensing_duration), renderSensedPoints);
  estimation_timer = nh.createTimer(ros::Duration(estimate_duration), pubCameraPose);
  // cv::namedWindow("depth_image",1);

  _inv_resolution = 1.0 / _resolution;

  _gl_xl = -_x_size / 2.0;
  _gl_yl = -_y_size / 2.0;
  _gl_zl = 0.0;

  _GLX_SIZE = (int)(_x_size * _inv_resolution);
  _GLY_SIZE = (int)(_y_size * _inv_resolution);
  _GLZ_SIZE = (int)(_z_size * _inv_resolution);

  tf_listener_ptr_ =
      std::unique_ptr<tf2_ros::TransformListener>(new tf2_ros::TransformListener(tf_buffer_));

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();
    status = ros::ok();
    rate.sleep();
  }
}