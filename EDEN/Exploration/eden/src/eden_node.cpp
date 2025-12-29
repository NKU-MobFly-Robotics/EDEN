#include <ros/ros.h>
#include <stdio.h>
#include <string>
#include <eden/eden_fsm.h>

#include <glog/logging.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "eden_node");
    ros::NodeHandle nh, nh_private("~");

    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InstallFailureSignalHandler();

    SingleExpFSM S_FSM;
    S_FSM.init(nh, nh_private);
    ros::spin();
    return 0;
}