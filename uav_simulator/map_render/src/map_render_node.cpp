#include <iostream>
#include <string>

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <ros/package.h>

#include "mesh_render/mesh_render.h"
#include "pointcloud_render.h"

using namespace std;

PointcloudRender::Ptr pointcloud_render_;
MeshRender::Ptr mesh_render_;

// Help 
bool savePointCloud(const pcl::PointCloud<pcl::PointXYZ> &cloud, const string &cloud_file) {
  if (cloud_file.find(".pcd") != string::npos) {
    pcl::io::savePCDFileASCII(cloud_file, cloud);
    ROS_INFO("[MapRender] Saved %lu data points to %s.", cloud.points.size(),
             cloud_file.c_str());
    return true;
  } else if (cloud_file.find(".ply") != string::npos) {
    pcl::io::savePLYFileASCII(cloud_file, cloud);
    ROS_INFO("[MapRender] Saved %lu data points to %s.", cloud.points.size(),
             cloud_file.c_str());
    return true;
  } else {
    ROS_ERROR("[MapRender] Output file type not supported.");
    return false;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "map_render_node");
  ros::NodeHandle nh("~");

  string map_file;
  nh.getParam("/map_config/map_file", map_file);
  if (map_file.empty()) {
    ROS_ERROR("[MapRender] map_file is empty!");
    return -1;
  }
  // if map_file is absolute path, use it directly
  if (map_file[0] == '/') {
    ROS_INFO("[MapRender] Using absolute path: %s", map_file.c_str());
  } else {
    // if map_file is relative path, use ros package path
    string map_render_path;
    map_render_path = ros::package::getPath("map_render");
    map_file = map_render_path + "/resource/" + map_file;
    ROS_INFO("[MapRender] Using relative path: %s", map_file.c_str());
  }

  if (map_file.find(".pcd") != string::npos || map_file.find(".ply") != string::npos) {
    // if map_file end with ".pcd" or ".ply", use pcl render
    pointcloud_render_.reset(new PointcloudRender());
    pointcloud_render_->initialize(nh);

    /* load cloud from pcd */
    pcl::PointCloud<pcl::PointXYZ> cloud;
    int status = -1;
    if (map_file.find(".pcd") != string::npos)
      status = pcl::io::loadPCDFile<pcl::PointXYZ>(map_file, cloud);
    else if (map_file.find(".ply") != string::npos)
      status = pcl::io::loadPLYFile<pcl::PointXYZ>(map_file, cloud);

    if (status == -1) {
      ROS_ERROR("[MapRender] Failed to load %s", map_file.c_str());
      return -1;
    } else {
      ROS_INFO("[MapRender] Loaded %s with %lu points", map_file.c_str(), cloud.points.size());
    }

    pointcloud_render_->setPointcloud(cloud);
  } else if (map_file.find(".stl") != string::npos || map_file.find(".obj") != string::npos) {
    // if map_file end with ".stl" or ".obj", use mesh render
    // WARNING: .obj not fully tested yet
    mesh_render_.reset(new MeshRender());
    mesh_render_->initialize(nh);
  } else {
    ROS_ERROR("[MapRender] Input file type not supported. File: %s", map_file.c_str());
    return -1;
  }

  // TODO: random_forest

  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}