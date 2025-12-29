#include <iostream>
#include <vector>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;

void addPointcloudBBox(const Eigen::Vector3d bbox_min, const Eigen::Vector3d bbox_max,
                       const double step, pcl::PointCloud<pcl::PointXYZ> &cloud) {
  pcl::PointXYZ p;

  for (double x = bbox_min[0]; x <= bbox_max[0]; x += step) {
    for (double y = bbox_min[1]; y <= bbox_max[1]; y += step) {
      for (double z = bbox_min[2]; z <= bbox_max[2]; z += step) {
        p.x = x;
        p.y = y;
        p.z = z;
        cloud.push_back(p);
      }
    }
  }
}

void removePointcloudBBox(const Eigen::Vector3d bbox_min, const Eigen::Vector3d bbox_max,
                          pcl::PointCloud<pcl::PointXYZ> &cloud) {
  pcl::PointCloud<pcl::PointXYZ> cloud_filtered;

  for (int i = 0; i < (int)cloud.points.size(); i++) {
    if (cloud.points[i].x < bbox_min[0] || cloud.points[i].x > bbox_max[0] ||
        cloud.points[i].y < bbox_min[1] || cloud.points[i].y > bbox_max[1] ||
        cloud.points[i].z < bbox_min[2] || cloud.points[i].z > bbox_max[2]) {
      cloud_filtered.push_back(cloud.points[i]);
    }
  }

  cloud = cloud_filtered;
}

bool savePointcloud(const string &output_file, const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  if (output_file.find(".pcd") != string::npos)
    return pcl::io::savePCDFile(output_file, cloud) == 0;
  else if (output_file.find(".ply") != string::npos)
    return pcl::io::savePLYFile(output_file, cloud) == 0;

  std::cerr << "file type not supported." << std::endl;
  return false;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pointcloud_modifier");
  ros::NodeHandle nh;

  string input_file, output_file;

  if (argc == 3) {
    input_file = argv[1];
    output_file = argv[2];
  } else {
    cout << "Usage: rosrun map_render pointcloud_modifier input_file output_file" << endl;
    return -1;
  }

  if (input_file.find(".pcd") == string::npos && input_file.find(".ply") == string::npos) {
    cout << "file type not supported." << endl;
    return -1;
  }

  int status = -1;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  if (input_file.find(".pcd") != string::npos)
    status = pcl::io::loadPCDFile<pcl::PointXYZ>(input_file, cloud);
  else if (input_file.find(".ply") != string::npos)
    status = pcl::io::loadPLYFile<pcl::PointXYZ>(input_file, cloud);
  if (status == -1) {
    cout << "can't read file." << endl;
    return -1;
  }

  // Add bbox1
  // -4.3; -18.7; 14.9
  // 4.9; -14.3; 14.9
  Eigen::Vector3d bbox_min1 = Eigen::Vector3d(-4, -19.1, 14.7);
  Eigen::Vector3d bbox_max1 = Eigen::Vector3d(4.5, -14.2, 15.0);

  // Add bbox2
  Eigen::Vector3d bbox_min2 = Eigen::Vector3d(-4, -13.9, 14.7);
  Eigen::Vector3d bbox_max2 = Eigen::Vector3d(4.5, -9.2, 15.0);

  // double step = 0.1;
  // addPointcloudBBox(bbox_min, bbox_max, step, cloud);

  // Remove bbox
  removePointcloudBBox(bbox_min1, bbox_max1, cloud);
  removePointcloudBBox(bbox_min2, bbox_max2, cloud);

  // save
  if (!savePointcloud(output_file, cloud)) {
    std::cerr << "Failed to save pointcloud." << std::endl;
    return -1;
  }

  return 0;
}