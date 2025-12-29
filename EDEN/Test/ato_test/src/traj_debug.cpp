#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
// #include <mav_msgs/conversions.h>
// #include <mav_msgs/default_topics.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Empty.h>
#include <mavros_msgs/PositionTarget.h>
// #include <lowres_map/lowres_map.h>

// #include<block_map/block_map.h>
#include<gcopter_debug/traj_opt.h>

#include <glog/logging.h>

TrajOptimizerDebug *traj_opt_;
ros::Publisher traj_pub_;
ros::Publisher corridor_pub_;
int idx = 0;


void ShowPathCorridor(vector<Eigen::MatrixX4d> &h, int id){
    visualization_msgs::MarkerArray mka;
    mka.markers.resize(h.size());
    // mka.markers[0].header.frame_id = "world";
    // mka.markers[0].header.stamp = ros::Time::now();
    // mka.markers[0].id = id;
    // mka.markers[0].action = visualization_msgs::Marker::ADD;
    // mka.markers[0].type = visualization_msgs::Marker::LINE_STRIP;
    // mka.markers[0].scale.x = 0.07;
    // mka.markers[0].scale.y = 0.07;
    // mka.markers[0].scale.z = 0.07;
    // mka.markers[0].color.a = 1.0;
    // mka.markers[0].color.r = 0.9;
    // mka.markers[0].color.g = 0.9;
    // mka.markers[0].color.b = 0.9;

    for(int i = 0; i < h.size(); i++){
        mka.markers[i].pose.position.x = (h[i](1, 3) - h[i](0, 3)) / 2;
        mka.markers[i].pose.position.y = (h[i](3, 3) - h[i](2, 3)) / 2;
        mka.markers[i].pose.position.z = (h[i](5, 3) - h[i](4, 3)) / 2;
        mka.markers[i].pose.orientation.w = 1.0;

        mka.markers[i].scale.x = (- h[i](1, 3) - h[i](0, 3));
        mka.markers[i].scale.y = (- h[i](3, 3) - h[i](2, 3));
        mka.markers[i].scale.z = (- h[i](5, 3) - h[i](4, 3));
        mka.markers[i].type = visualization_msgs::Marker::CUBE;
        mka.markers[i].header.frame_id = "world";
        mka.markers[i].header.stamp = ros::Time::now();
        mka.markers[i].id = id+i;
        mka.markers[i].action = visualization_msgs::Marker::ADD;
        // mka.markers[i].lifetime = ros::Duration(1.0);
        if(id == 200){
            mka.markers[i].color.a = 0.2;
            mka.markers[i].color.g = 0.9;
            mka.markers[i].color.b = 0.5;
        }
        else{
            mka.markers[i].color.a = 0.2;
            mka.markers[i].color.g = 0.5;
            mka.markers[i].color.b = 0.9;
        }

    }
    corridor_pub_.publish(mka);
}

void Debug(list<Eigen::Vector3d> &pts){
    visualization_msgs::Marker mk, mkr1, mkr2;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = idx;
    // mk.id = scan_count_;
    // scan_count_++;
    mk.action = visualization_msgs::Marker::ADD;
    mk.type = visualization_msgs::Marker::SPHERE_LIST;
    mk.scale.x = 0.05;
    mk.scale.y = 0.05;
    mk.scale.z = 0.05;
    mk.color.a = 1.0;
    mk.color.b = 1.0;
    mk.color.r = idx * 0.3;
    idx++;

    mk.pose.position.x = 0;
    mk.pose.position.y = 0;
    mk.pose.position.z = 0;
    mk.pose.orientation.x = 0;
    mk.pose.orientation.y = 0;
    mk.pose.orientation.z = 0;
    mk.pose.orientation.w = 1;

    geometry_msgs::Point pt;
    for(auto &p : pts){
        pt.x = p.x();
        pt.y = p.y();
        pt.z = p.z();
        mk.points.push_back(pt);
    }
    traj_pub_.publish(mk);
}

void TrajTest(){

    TrajOptInput toi;
    toi.initState.resize(4, 3);
    toi.initState<<1.00298923320101396,0.98886982328386064,1.78958897358353819,
    -0.00143307993344636,-0.05825286355607370,-0.75275297672828101,
    0.83456222745930209,-0.16076812268444063,1.47292978530773433,
    0.00006624605901534,0.00030318104340647,0.00312989064036944;

    toi.endState_stem<<  4.55000000000006821,-0.34999999999996589,1.55000000000000027,0.00000000000000000;

    toi.endState_main << 7.30000000000007443,-0.39999999999996017,2.20000000000000018,0.00000000000000000;

    toi.endState_sub<< 4.60000000000006803,-0.29999999999996591,1.60000000000000031,0.00000000000000000;
    toi.camFov.resize(4, 4);
    toi.camFov<< -0.813416, 0.581683, 0, 0
                ,-0.813416,  -0.581683, 0, 0
                ,-0.14112,  0, -0.989992, 0
                ,-0.14112,  0, 0.989992, 0;
    toi.camV.resize(3, 5);
    toi.camV<< 0, 1, 1, 1, 1
    ,0,0.715112 ,0.715112 ,-0.715112 ,-0.715112 
    ,0,-0.715112 ,0.715112 ,-0.715112 ,0.715112;
    toi.corridorVs_stem.resize(3);
    toi.corridorVs_stem[0].resize(3, 8);
    toi.corridorVs_stem[1].resize(3, 8);
    toi.corridorVs_stem[2].resize(3, 8);
    // toi.corridorVs_stem[3].resize(3, 8);
    toi.corridorVs_stem[0]<<
    3.44490000000005692,0.75510000000004551,3.44490000000005692,0.75510000000004551,3.44490000000005692,0.75510000000004551,3.44490000000005692,0.75510000000004551,
    0.64490000000004544,0.64490000000004544,-0.64489999999993175,-0.64489999999993175,0.64490000000004544,0.64490000000004544,-0.64489999999993175,-0.64489999999993175,
    1.14690000000000025,1.14690000000000025,1.14690000000000025,1.14690000000000025,0.55310000000000004,0.55310000000000004,0.55310000000000004,0.55310000000000004;
    toi.corridorVs_stem[1]<<
    5.54490000000002237,1.45510000000003403,5.54490000000002237,1.45510000000003403,5.54490000000002237,1.45510000000003403,5.54490000000002237,1.45510000000003403,
    0.64490000000004544,0.64490000000004544,-1.34489999999997734,-1.34489999999997734,0.64490000000004544,0.64490000000004544,-1.34489999999997734,-1.34489999999997734,
    1.84690000000000021,1.84690000000000021,1.84690000000000021,1.84690000000000021,0.55310000000000004,0.55310000000000004,0.55310000000000004,0.55310000000000004;

    toi.corridorVs_stem[2]<<
    5.54490000000002237,2.85510000000006814,5.54490000000002237,2.85510000000006814,5.54490000000002237,2.85510000000006814,5.54490000000002237,2.85510000000006814,
    0.64490000000004544,0.64490000000004544,-1.34489999999997734,-1.34489999999997734,0.64490000000004544,0.64490000000004544,-1.34489999999997734,-1.34489999999997734,
    1.84690000000000021,1.84690000000000021,1.84690000000000021,1.84690000000000021,0.55310000000000004,0.55310000000000004,0.55310000000000004,0.55310000000000004;

    // toi.corridorVs_stem[3]<<
    // 10.4449,  9.1551, 10.4449,  9.1551, 10.4449,  9.1551, 10.4449,  9.1551,
    // 6.2449,  6.2449,  4.2551,  4.2551,  6.2449,  6.2449,  4.2551,  4.2551,
    // 2.7469,  2.7469,  2.7469,  2.7469,  0.7531,  0.7531,  0.7531,  0.7531;

    toi.corridors_stem.resize(3);
    toi.corridors_stem[0].resize(6, 4);
    toi.corridors_stem[1].resize(6, 4);
    toi.corridors_stem[2].resize(6, 4);
    // toi.corridors_stem[3].resize(6, 4);
    toi.corridors_stem[0].transpose()<<
    1,      -1,       0,       0,       0,       0,
    0,       0,       1,      -1,       0,       0,
    0,       0,       0,       0,       1,      -1,
    -3.44490000000005692,0.75510000000004551,-0.64490000000004544,-0.64489999999993175,-1.14690000000000025,0.55310000000000004;

    toi.corridors_stem[1].transpose()<<
        1,      -1,       0,       0,       0,       0,
        0,       0,       1,      -1,      0,      0,
        0,       0,       0,       0,       1,      -1,
    -5.54490000000002237,1.45510000000003403,-0.64490000000004544,-1.34489999999997734,-1.84690000000000021,0.55310000000000004;

    toi.corridors_stem[2].transpose()<<
        1,      -1,       0,       0,       0,       0,
        0,       0,       1,      -1,      0,      0,
        0,       0,       0,       0,       1,      -1,
    -5.54490000000002237,2.85510000000006814,-0.64490000000004544,-1.34489999999997734,-1.84690000000000021,0.55310000000000004;

    // toi.corridors_stem[3].transpose()<<
    //     1,      -1,       0,       0,       0,       0,
    //     0,       0,       1,      -1,      0,      0,
    //     0,       0,       0,       0,       1,      -1,
    //     -10.4449,   9.1551,  -6.9449,   4.2551,  -2.7469,   0.7531;


    toi.corridorVs_main.resize(2);
    toi.corridorVs_main[0].resize(3, 8);
    toi.corridorVs_main[1].resize(3, 8);
    // toi.corridorVs_main[2].resize(3, 8);
    // toi.corridorVs_main[3].resize(3, 8);
    toi.corridorVs_main[0]<<
    6.94490000000005647,2.85510000000006814,6.94490000000005647,2.85510000000006814,6.94490000000005647,2.85510000000006814,6.94490000000005647,2.85510000000006814,
    0.64490000000004544,0.64490000000004544,-1.34489999999997734,-1.34489999999997734,0.64490000000004544,0.64490000000004544,-1.34489999999997734,-1.34489999999997734,
    2.44689999999999985,2.44689999999999985,2.44689999999999985,2.44689999999999985,0.55310000000000004,0.55310000000000004,0.55310000000000004,0.55310000000000004;

    toi.corridorVs_main[1]<<
    9.04490000000002148,3.55510000000005677,9.04490000000002148,3.55510000000005677,9.04490000000002148,3.55510000000005677,9.04490000000002148,3.55510000000005677,
    0.64490000000004544,0.64490000000004544,-1.34489999999997734,-1.34489999999997734,0.64490000000004544,0.64490000000004544,-1.34489999999997734,-1.34489999999997734,
    2.44689999999999985,2.44689999999999985,2.44689999999999985,2.44689999999999985,0.55310000000000004,0.55310000000000004,0.55310000000000004,0.55310000000000004;

    // toi.corridorVs_main[2]<<
    // 14.4449, 11.9551, 14.4449, 11.9551, 14.4449, 11.9551, 14.4449, 11.9551,
    // 2.0449,  2.0449, -1.3449, -1.3449,  2.0449,  2.0449, -1.3449, -1.3449,
    // 2.4469,  2.4469,  2.4469,  2.4469,  0.5531,  0.5531,  0.5531,  0.5531;

    // toi.corridorVs_main[3]<<
    // 13.2449, 11.2551, 13.2449, 11.2551, 13.2449, 11.2551, 13.2449, 11.2551,
    // 10.4449, 10.4449,  5.6551,  5.6551, 10.4449, 10.4449,  5.6551,  5.6551,
    //  2.7469,  2.7469,  2.7469,  2.7469,  0.7531,  0.7531,  0.7531, 0.7531;

    toi.corridors_main.resize(2);
    toi.corridors_main[0].resize(6, 4);
    toi.corridors_main[1].resize(6, 4);
    // toi.corridors_main[2].resize(6, 4);
    // toi.corridors_main[3].resize(6, 4);
    toi.corridors_main[0].transpose()<<
    1,      -1,       0,       0,       0,       0,
    0,       0,       1,      -1,       0,       0,
    0,       0,       0,       0,       1,      -1,
    -6.94490000000005647,2.85510000000006814,-0.64490000000004544,-1.34489999999997734,-2.44689999999999985,0.55310000000000004;
    
    toi.corridors_main[1].transpose()<<
    1,      -1,       0,       0,       0,       0,
    0,       0,       1,      -1,       0,       0,
    0,       0,       0,       0,       1,      -1,
    -9.04490000000002148,3.55510000000005677,-0.64490000000004544,-1.34489999999997734,-2.44689999999999985,0.55310000000000004;
    
    // toi.corridors_main[2].transpose()<<
    // 1,      -1,       0,       0,       0,       0,
    // 0,       0,       1,      -1,       0,       0,
    // 0,       0,       0,       0,       1,      -1,
    // -14.4449,  11.9551,  -2.0449,  -1.3449,  -2.4469,   0.5531;

    // toi.corridors_main[3].transpose()<<
    // 1,      -1,       0,       0,       0,       0,
    // 0,       0,       1,      -1,       0,       0,
    // 0,       0,       0,       0,       1,      -1,
    // -13.2449,  11.2551, -10.4449,   5.6551,  -2.7469,   0.7531;


    toi.corridorVs_sub.resize(2);
    toi.corridorVs_sub[0].resize(3, 8);
    toi.corridorVs_sub[1].resize(3, 8);
    // toi.corridorVs_sub[2].resize(3, 8);
    toi.corridorVs_sub[0]<<
    5.54490000000002237,2.85510000000006814,5.54490000000002237,2.85510000000006814,5.54490000000002237,2.85510000000006814,5.54490000000002237,2.85510000000006814,
    0.64490000000004544,0.64490000000004544,-1.34489999999997734,-1.34489999999997734,0.64490000000004544,0.64490000000004544,-1.34489999999997734,-1.34489999999997734,
    1.84690000000000021,1.84690000000000021,1.84690000000000021,1.84690000000000021,0.55310000000000004,0.55310000000000004,0.55310000000000004,0.55310000000000004;
    toi.corridorVs_sub[1]<<
    5.54490000000002237,2.85510000000006814,5.54490000000002237,2.85510000000006814,5.54490000000002237,2.85510000000006814,5.54490000000002237,2.85510000000006814,
    0.64490000000004544,0.64490000000004544,-1.34489999999997734,-1.34489999999997734,0.64490000000004544,0.64490000000004544,-1.34489999999997734,-1.34489999999997734,
    1.84690000000000021,1.84690000000000021,1.84690000000000021,1.84690000000000021,0.55310000000000004,0.55310000000000004,0.55310000000000004,0.55310000000000004;
    // toi.corridorVs_sub[2]<<
    // 5.54490000000002237,2.85510000000006814,5.54490000000002237,2.85510000000006814,5.54490000000002237,2.85510000000006814,5.54490000000002237,2.85510000000006814,
    // 0.64490000000004544,0.64490000000004544,-1.34489999999997734,-1.34489999999997734,0.64490000000004544,0.64490000000004544,-1.34489999999997734,-1.34489999999997734,
    // 1.84690000000000021,1.84690000000000021,1.84690000000000021,1.84690000000000021,0.55310000000000004,0.55310000000000004,0.55310000000000004,0.55310000000000004;

    toi.corridors_sub.resize(2);
    toi.corridors_sub[0].resize(6, 4);
    toi.corridors_sub[1].resize(6, 4);
    // toi.corridors_sub[2].resize(6, 4);
    toi.corridors_sub[0].transpose()<<
    1,      -1,       0,       0,       0,       0,
    0,       0,       1,      -1,       0,       0,
    0,       0,       0,       0,       1,      -1,
    -5.54490000000002237,2.85510000000006814,-0.64490000000004544,-1.34489999999997734,-1.84690000000000021,0.55310000000000004;
    
    toi.corridors_sub[1].transpose()<<
    1,      -1,       0,       0,       0,       0,
    0,       0,       1,      -1,       0,       0,
    0,       0,       0,       0,       1,      -1,
    -5.54490000000002237,2.85510000000006814,-0.64490000000004544,-1.34489999999997734,-1.84690000000000021,0.55310000000000004;
    
    // toi.corridors_sub[2].transpose()<<
    // 1,      -1,       0,       0,       0,       0,
    // 0,       0,       1,      -1,       0,       0,
    // 0,       0,       0,       0,       1,      -1,
    // -5.54490000000002237,2.85510000000006814,-0.64490000000004544,-1.34489999999997734,-1.84690000000000021,0.55310000000000004;
    // PosDim = CorridorEpV.cols();
    // vPos = CorridorEpV;
    toi.corridorVs_Ep.resize(3, 8);
    Eigen::Vector3d pmax, pmin; 
    for(int i = 0; i < 3; i++){
        
        pmax(i) = min(toi.endState_stem(i, 0) + 0.35, toi.corridorVs_stem.back().row(i).maxCoeff());
        pmin(i) = max(toi.endState_stem(i, 0) - 0.35, toi.corridorVs_stem.back().row(i).minCoeff());
        pmax(i) = min(pmax(i), toi.corridorVs_main.front().row(i).maxCoeff());
        pmin(i) = max(pmin(i), toi.corridorVs_main.front().row(i).minCoeff());
        pmax(i) = min(pmax(i), toi.corridorVs_sub.front().row(i).maxCoeff());
        pmin(i) = max(pmin(i), toi.corridorVs_sub.front().row(i).minCoeff());
    }
    // cout<<"setup8.1"<<endl;

    for(int dim1 = 0; dim1 <= 1; dim1++){
        for(int dim2 = 0; dim2 <= 1; dim2++){
            for(int dim3 = 0; dim3 <= 1; dim3++){
                toi.corridorVs_Ep(0, 4*dim3 + 2*dim2 + dim1) = dim1 ? pmax(0) : pmin(0);
                toi.corridorVs_Ep(1, 4*dim3 + 2*dim2 + dim1) = dim2 ? pmax(1) : pmin(1);
                toi.corridorVs_Ep(2, 4*dim3 + 2*dim2 + dim1) = dim3 ? pmax(2) : pmin(2);
            }
        }
    }
    // cout<<"setup8.2"<<endl;

    for(int j = 1; j < 8; j++){
        toi.corridorVs_Ep.col(j) = toi.corridorVs_Ep.col(j) - toi.corridorVs_Ep.col(0);
    }    
    
    
    toi.traj_bbxs_main.resize(toi.corridors_main.size());
    toi.traj_bbxs_sub.resize(toi.corridors_sub.size());
    toi.traj_bbxs_stem.resize(toi.corridors_stem.size());
    toi.traj_bbxs_norm.resize(toi.corridors_norm.size());





    if(!traj_opt_->AggressiveBranchTrajOptimize(toi)){
        ROS_ERROR("opt failed");
    }

    ROS_WARN("?");
    ros::Duration(1.5).sleep();
    list<Eigen::Vector3d> trj;
    for(double t = 0; t < traj_opt_->stem_traj_.getTotalDuration(); t += 0.025){
        Eigen::Vector3d p = traj_opt_->stem_traj_.getPos(t).head(3);
        trj.emplace_back(p);
        // cout<<"p:"<<p.transpose()<<endl;
    }
    ROS_WARN("??");

    list<Eigen::Vector3d> trj1;
    for(double t = 0; t < traj_opt_->main_traj_.getTotalDuration(); t += 0.025){
        Eigen::Vector3d p = traj_opt_->main_traj_.getPos(t).head(3);
        trj1.emplace_back(p);
        // cout<<"pm:"<<p.transpose()<<endl;
    }
    list<Eigen::Vector3d> trj2;
    for(double t = 0; t < traj_opt_->sub_traj_.getTotalDuration(); t += 0.025){
        Eigen::Vector3d p = traj_opt_->sub_traj_.getPos(t).head(3);
        trj2.emplace_back(p);
        // cout<<"ps:"<<p.transpose()<<endl;
    }
    ROS_WARN("???");

    Debug(trj);
    // ros::Duration(0.5).sleep();
    // ShowPathCorridor(toi.corridors_stem, 200);
    ros::Duration(0.5).sleep();
    // getchar();
    Debug(trj1);
    // ros::Duration(0.5).sleep();
    // ShowPathCorridor(toi.corridors_main, 400);
    ros::Duration(0.5).sleep();
    ShowPathCorridor(toi.corridors_sub, 200);
    ROS_WARN("????");

    Debug(trj2);
    ROS_WARN("?????");

    // cout<<toi.corridors_sub[1].row(0).maxCoeff()<<endl;
    // cout<<toi.corridors_sub[1].row(3).maxCoeff()<<endl;
}



int main(int argc, char** argv){
    ros::init(argc, argv, "traj_test");
    ros::NodeHandle nh, nh_private("~");

    string ns = ros::this_node::getName(), occ_path, free_path;
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InstallFailureSignalHandler();
    traj_pub_ = nh.advertise<visualization_msgs::Marker>("/debug", 1);
    corridor_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/debug_cor", 1);
    TrajOptimizerDebug traj_opt;
    traj_opt_ = &traj_opt;
    traj_opt.Init(nh, nh_private);
    while(ros::ok()){
        TrajTest();
        getchar();
    }


    ros::spin();
    return 0;
}