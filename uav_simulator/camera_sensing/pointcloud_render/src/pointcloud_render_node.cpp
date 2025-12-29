#include "pointcloud_render.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "pcl_render");
  ros::NodeHandle nh("~");

  PointcloudRender pointcloud_render;
  pointcloud_render.initialize(nh);

  string map_file;
  nh.getParam("/map_config/map_file", map_file);

  if (map_file.empty()) {
    ROS_ERROR("[PointcloudRenderNode] map_file is empty!");
    return -1;
  }
  // if map_file is absolute path, use it directly
  if (map_file[0] == '/') {
    ROS_INFO("[PointcloudRenderNode] Using absolute path: %s", map_file.c_str());
  } else {
    // if map_file is relative path, use ros package path
    string map_render_path;
    map_render_path = ros::package::getPath("map_render");
    map_file = map_render_path + "/resource/" + map_file;
    ROS_INFO("[PointcloudRenderNode] Using relative path: %s", map_file.c_str());
  }

  /* load cloud from pcd */
  pcl::PointCloud<pcl::PointXYZ> cloud;
  int status = -1;
  if (map_file.find(".pcd") != string::npos)
    status = pcl::io::loadPCDFile<pcl::PointXYZ>(map_file, cloud);
  else if (map_file.find(".ply") != string::npos)
    status = pcl::io::loadPLYFile<pcl::PointXYZ>(map_file, cloud);
  else if (map_file.find(".stl") != string::npos || map_file.find(".obj") != string::npos)
    ROS_ERROR("[PointcloudRenderNode] STL and OBJ file type is not supported in pcl_renderer. Please use "
              "mesh_renderer.");
  else
    ROS_ERROR("[PointcloudRenderNode] Input file type not supported.");

  if (status == -1) {
    ROS_INFO("[PointcloudRenderNode] Failed to load %s", map_file.c_str());
    return -1;
  } else {
    ROS_INFO("[PointcloudRenderNode] Loaded %s with %lu points", map_file.c_str(), cloud.points.size());
  }

  pointcloud_render.setPointcloud(cloud);

  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}