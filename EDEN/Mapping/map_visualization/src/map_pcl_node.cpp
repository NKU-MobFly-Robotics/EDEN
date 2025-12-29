#include <ros/ros.h>
#include <stdio.h>
#include <string>
#include <std_msgs/Empty.h>
#include <glog/logging.h>
#include <map_visualization/map_visualization.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "map_pcl_node");
    ros::NodeHandle nh, nh_private("~");
    MapVisualization mv;
    mv.init(nh, nh_private);
    
    ros::spin();

    return 0;
}