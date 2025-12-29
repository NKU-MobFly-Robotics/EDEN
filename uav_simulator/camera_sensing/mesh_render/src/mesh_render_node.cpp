#include "mesh_render/mesh_render.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "mesh_renderer_node");
  ros::NodeHandle nh("~");

  MeshRender mesh_render;
  int state = mesh_render.initialize(nh);
  if (state == -1) {
    ROS_ERROR("Failed to initialize MeshRender");
    return -1;
  }

  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }
}
