#include <ros/ros.h>
#include <stdio.h>
#include <string>
#include <std_msgs/Empty.h>
#include <glog/logging.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "map_test_nod");
    ros::NodeHandle nh, nh_private("~");
    ros::Publisher d = nh.advertise<std_msgs::Empty>("/debug_topic", 1);
    std_msgs::Empty e;
    ROS_WARN("press enter to pub");
    while (ros::ok())
    {
        getchar();
        d.publish(e);
    }
    

    return 0;
}