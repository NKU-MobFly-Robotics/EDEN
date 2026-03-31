#include <ros/ros.h>
#include <stdio.h>
#include <string>
#include <single_exp/single_exp_fsm.h>

#include <glog/logging.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "single_exp_node");
    ros::NodeHandle nh, nh_private("~");

    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InstallFailureSignalHandler();

    SingleExpFSM S_FSM;
    S_FSM.init(nh, nh_private);
    ros::spin();
    return 0;
}