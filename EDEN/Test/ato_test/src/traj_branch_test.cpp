#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
// #include <mav_msgs/conversions.h>
// #include <mav_msgs/default_topics.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Empty.h>
#include <mavros_msgs/PositionTarget.h>
#include <lowres_map/lowres_map.h>

#include<block_map/block_map.h>
#include<gcopter/traj_opt.h>

#include <glog/logging.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

ros::Timer motion_timer_;
ros::Publisher vis_pub_, debug_pub_;
ros::Subscriber target_sub_, state_sub_;
Eigen::Vector4d stem_init, stem_end, main_end, sub_end, swarm_init, swarm_end;
Eigen::MatrixX4d camFov_;
Eigen::Matrix3Xd camV_;
double theta_, psi_;
int point_count_;
int state_; // (swarminit, swarmterm) -> main -> sub -> (steminit, stemterm) -> 
bool init_;
BlockMap *BM_;
lowres::LowResMap *LRM_;
TrajOptimizer *TO_;
vector<Trajectory4<5>> swarm_trajs_;
vector<pair<Eigen::Vector3d, Eigen::Vector3d>> swarm_trajs_bbx_;
vector<Eigen::Vector3d> swarm_poses_;
double tc_;

void LoadFOVLines(visualization_msgs::Marker &mk, Eigen::Vector4d pose){
    Eigen::Vector3d upleft(cos(psi_), cos(psi_) / tan(theta_ * 0.5), sin(psi_ * 0.5)), 
    upright(cos(psi_), -cos(psi_) / tan(theta_ * 0.5), sin(psi_ * 0.5)), 
    downleft(cos(psi_), cos(psi_) / tan(theta_ * 0.5), -sin(psi_ * 0.5)), 
    downright(cos(psi_), -cos(psi_) / tan(theta_ * 0.5), -sin(psi_ * 0.5)), ori(0,0,0);
    double ks = 1.0;
    vector<Eigen::Vector3d> fov_line_pts_;
    fov_line_pts_.emplace_back(upleft * ks / upleft(0));
    // cout<<"fov_line_pts_.back()"<<fov_line_pts_.back().transpose()<<endl;
    fov_line_pts_.emplace_back(ori);
    fov_line_pts_.emplace_back(upright * ks / upright(0));
    fov_line_pts_.emplace_back(ori);
    fov_line_pts_.emplace_back(downleft * ks / downleft(0));
    fov_line_pts_.emplace_back(ori);
    fov_line_pts_.emplace_back(downright * ks / downright(0));
    fov_line_pts_.emplace_back(ori);

    fov_line_pts_.emplace_back(upleft * ks / upleft(0));
    fov_line_pts_.emplace_back(upright * ks / upright(0));

    fov_line_pts_.emplace_back(upright * ks / upright(0));
    fov_line_pts_.emplace_back(downright * ks / downright(0));

    fov_line_pts_.emplace_back(downright * ks / downright(0));
    fov_line_pts_.emplace_back(downleft * ks / downleft(0));

    fov_line_pts_.emplace_back(downleft * ks / downleft(0));
    fov_line_pts_.emplace_back(upleft * ks / upleft(0));
    geometry_msgs::Point gpt;
    Eigen::Vector3d ept;
    Eigen::Matrix3d R;
    R << cos(pose(3)), -sin(pose(3)), 0, sin(pose(3)), cos(pose(3)), 0, 0, 0, 1;  

    for(auto &pfov : fov_line_pts_){
        ept = R * (pfov) + pose.head(3);
        gpt.x = ept(0);
        gpt.y = ept(1);
        gpt.z = ept(2);
        mk.points.emplace_back(gpt);
    }
}

void VisMotion(double tc){
    visualization_msgs::MarkerArray mka;
    mka.markers.resize(2);
    mka.markers[0].header.frame_id = "world";
    mka.markers[0].header.stamp = ros::Time::now();
    mka.markers[0].id = 10;
    mka.markers[0].action = visualization_msgs::Marker::ADD;
    mka.markers[0].type = visualization_msgs::Marker::LINE_LIST;
    mka.markers[0].scale.x = 0.07;
    mka.markers[0].scale.y = 0.07;
    mka.markers[0].scale.z = 0.07;
    mka.markers[0].color.a = 1.0;
    mka.markers[0].color.r = 1.0;
    mka.markers[0].color.g = 0.0;
    mka.markers[0].color.b = 0.0;
    mka.markers[1] = mka.markers[0];
    mka.markers[1].id = 11;
    mka.markers[1].color.r = 0.0;
    mka.markers[1].color.b = 1.0;

    geometry_msgs::Point pt;
    Eigen::Vector4d pos;
    for(auto &traj : swarm_trajs_){
        if(traj.getTotalDuration() > tc){
            pos = traj.getPos(tc);
            LoadFOVLines(mka.markers[1], pos);
        }
    }
    if(TO_->stem_traj_.getTotalDuration() > tc){
        pos = TO_->stem_traj_.getPos(tc);
        LoadFOVLines(mka.markers[0], pos);
    }
    else{
        tc = tc - TO_->stem_traj_.getTotalDuration();
        if(tc < TO_->main_traj_.getTotalDuration()){
            pos = TO_->main_traj_.getPos(tc);
            LoadFOVLines(mka.markers[0], pos);
        }
        if(tc < TO_->sub_traj_.getTotalDuration()){
            pos = TO_->sub_traj_.getPos(tc);
            LoadFOVLines(mka.markers[0], pos);
        }
    }
    
    vis_pub_.publish(mka);
}


void VisCorridors(vector<Eigen::MatrixX4d> &h, int id){
    visualization_msgs::MarkerArray mka;
    mka.markers.resize(h.size());

    
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
        mka.markers[i].id = id-i;
        mka.markers[i].action = visualization_msgs::Marker::ADD;
        // mka.markers[i].lifetime = ros::Duration(1.0);
        mka.markers[i].color.a = 0.1;
        mka.markers[i].color.b = 1.0;
        mka.markers[i].color.g = 0.2;
        // mka.markers[i].lifetime = ros::Duration(1.0);
    }
    vis_pub_.publish(mka);

}

void VisFov(Eigen::Vector4d pose, int id){
    visualization_msgs::MarkerArray mka;
    mka.markers.resize(1);
    mka.markers[0].header.frame_id = "world";
    mka.markers[0].header.stamp = ros::Time::now();
    mka.markers[0].id = id;
    mka.markers[0].action = visualization_msgs::Marker::ADD;
    mka.markers[0].type = visualization_msgs::Marker::LINE_STRIP;
    mka.markers[0].scale.x = 0.07;
    mka.markers[0].scale.y = 0.07;
    mka.markers[0].scale.z = 0.07;
    mka.markers[0].color.a = 1.0;
    mka.markers[0].color.r = 1.0;
    mka.markers[0].color.g = 0.0;
    mka.markers[0].color.b = 0.0;

    Eigen::Vector3d upleft(cos(psi_), cos(psi_) / tan(theta_ * 0.5), sin(psi_ * 0.5)), 
    upright(cos(psi_), -cos(psi_) / tan(theta_ * 0.5), sin(psi_ * 0.5)), 
    downleft(cos(psi_), cos(psi_) / tan(theta_ * 0.5), -sin(psi_ * 0.5)), 
    downright(cos(psi_), -cos(psi_) / tan(theta_ * 0.5), -sin(psi_ * 0.5)), ori(0,0,0);
    double ks = 1.0;
    vector<Eigen::Vector3d> fov_line_pts_;
    fov_line_pts_.emplace_back(upleft * ks / upleft(0));
    // cout<<"fov_line_pts_.back()"<<fov_line_pts_.back().transpose()<<endl;
    fov_line_pts_.emplace_back(ori);
    fov_line_pts_.emplace_back(upright * ks / upright(0));
    fov_line_pts_.emplace_back(ori);
    fov_line_pts_.emplace_back(downleft * ks / downleft(0));
    fov_line_pts_.emplace_back(ori);
    fov_line_pts_.emplace_back(downright * ks / downright(0));
    fov_line_pts_.emplace_back(ori);

    fov_line_pts_.emplace_back(upleft * ks / upleft(0));
    fov_line_pts_.emplace_back(upright * ks / upright(0));

    fov_line_pts_.emplace_back(upright * ks / upright(0));
    fov_line_pts_.emplace_back(downright * ks / downright(0));

    fov_line_pts_.emplace_back(downright * ks / downright(0));
    fov_line_pts_.emplace_back(downleft * ks / downleft(0));

    fov_line_pts_.emplace_back(downleft * ks / downleft(0));
    fov_line_pts_.emplace_back(upleft * ks / upleft(0));
    geometry_msgs::Point gpt;
    Eigen::Vector3d ept;
    Eigen::Matrix3d R;
    R << cos(pose(3)), -sin(pose(3)), 0, sin(pose(3)), cos(pose(3)), 0, 0, 0, 1;  

    for(auto &pfov : fov_line_pts_){
        ept = R * (pfov) + pose.head(3);
        gpt.x = ept(0);
        gpt.y = ept(1);
        gpt.z = ept(2);
        mka.markers[0].points.emplace_back(gpt);
    }
    vis_pub_.publish(mka);
}

void VisTraj(Trajectory4<5> &traj, int id, int type){
    visualization_msgs::MarkerArray mka;
    mka.markers.resize(1);
    mka.markers[0].header.frame_id = "world";
    mka.markers[0].header.stamp = ros::Time::now();
    mka.markers[0].id = id;
    mka.markers[0].action = visualization_msgs::Marker::ADD;
    mka.markers[0].type = visualization_msgs::Marker::LINE_STRIP;
    mka.markers[0].scale.x = 0.07;
    mka.markers[0].scale.y = 0.07;
    mka.markers[0].scale.z = 0.07;
    mka.markers[0].color.a = 1.0;
    mka.markers[0].color.r = 0.0;
    mka.markers[0].color.g = 0.0;
    mka.markers[0].color.b = 0.0;
    if(type == 1){ // norm not ref
        mka.markers[0].color.b = 1.0;
    }
    else if(type == 2) // norm ref
    {
        mka.markers[0].color.r = 1.0;
    }
    else if(type == 3){ // main 
        mka.markers[0].color.g = 1.0;
    }
    else if(type == 4){ // sub 
        mka.markers[0].color.g = 1.0;
        mka.markers[0].color.b = 1.0;
    }
    else if(type == 5){ // stem 
        mka.markers[0].color.b = 1.0;
        mka.markers[0].color.r = 1.0;
    }
    for(double t = 0; t < traj.getTotalDuration(); t += 0.025){
        Eigen::Vector4d p = traj.getPos(t);
        geometry_msgs::Point pt;
        pt.x = p(0);
        pt.y = p(1);
        pt.z = p(2);
        mka.markers[0].points.emplace_back(pt);
    }
    vis_pub_.publish(mka);
}

bool GetCorridor(const Eigen::Vector3d &ps, const Eigen::Vector3d &pe, vector<Eigen::MatrixX4d> &h, vector<Eigen::Matrix3Xd> &p){
    vector<Eigen::Vector3d> path;
    if(!LRM_->GetPath(ps.head(3), pe.head(3), path, false, 5000)){
        ROS_ERROR("path search failed");
        return false;
    }
    vector<Eigen::Vector3d> path_pruned;
    
    if(LRM_->FindCorridors(path, h, p, path_pruned, INFINITY)){
        return true;
    }
    else {
        ROS_ERROR("corridor search failed");
        return false;
    }
}

bool PlanTraj(){
    TrajOptInput toi;

    if(!GetCorridor(swarm_init.head(3), swarm_end.head(3), toi.corridors_norm, toi.corridorVs_norm)){
        return false;
    }
    for(auto &c : toi.corridorVs_norm){
        toi.traj_bbxs_norm.emplace_back();
        for(int dim = 0; dim < 3; dim++){
            toi.traj_bbxs_norm.back().first(dim) = c(dim, 0);
            toi.traj_bbxs_norm.back().second(dim) = c(dim, 0);
        }
        for(int i = 1; i < c.cols(); i++){
            for(int dim = 0; dim < 3; dim++){
                toi.traj_bbxs_norm.back().first(dim) = max(c(dim, i), toi.traj_bbxs_norm.back().first(dim));
                toi.traj_bbxs_norm.back().second(dim) = max(c(dim, i), toi.traj_bbxs_norm.back().second(dim));
            }
        }
    }
    double tc = ros::WallTime::now().toSec();
    // toi.ref_trajs = swarm_trajs_;
    // toi.ref_traj_bbxs = swarm_trajs_bbx_;
    // toi.swarm_t.resize(toi.ref_trajs.size(), tc);
    toi.start_t = tc;
    toi.initState.resize(4, 3);
    toi.initState.setZero();
    toi.initState.col(0) = swarm_init;
    // toi.tailState.resize(4, 3);
    // toi.tailState.setZero();
    toi.endState_norm.col(0) = swarm_end;

    if(TO_->NormTrajOptimize(toi)){
        ROS_WARN("PlanTraj2");
        swarm_trajs_.emplace_back(TO_->norm_traj_);
        swarm_trajs_bbx_.emplace_back();
        int c = 0;
        for(double t = 0; t < TO_->norm_traj_.getTotalDuration(); t += 0.025, c++){
            if(c == 0){
                swarm_trajs_bbx_.back().first = TO_->norm_traj_.getPos(t).head(3);
                swarm_trajs_bbx_.back().second = swarm_trajs_bbx_.back().first;
            }
            else{
                Eigen::Vector3d p = TO_->norm_traj_.getPos(t).head(3);
                for(int dim = 0; dim < 3; dim++){
                    swarm_trajs_bbx_.back().first(dim) = max(p(dim), swarm_trajs_bbx_.back().first(dim));
                    swarm_trajs_bbx_.back().second(dim) = min(p(dim), swarm_trajs_bbx_.back().second(dim));
                }
            }
        }
        VisTraj(TO_->norm_traj_, 0, 1);

        return true;
    }
    else{
        ROS_ERROR("norm opt failed");
        return false;
    }
}

bool PlanBranch(){
    TrajOptInput toi;
    cout<<"stem_init:"<<stem_init.transpose()<<endl;
    cout<<"stem_end:"<<stem_end.transpose()<<endl;
    cout<<"sub_end:"<<sub_end.transpose()<<endl;
    cout<<"main_end:"<<main_end.transpose()<<endl;
    if(!GetCorridor(stem_init.head(3), stem_end.head(3), toi.corridors_stem, toi.corridorVs_stem)){
        ROS_WARN("stem failed");
        return false;
    }
    for(auto &c : toi.corridorVs_stem){
        toi.traj_bbxs_stem.emplace_back();
        for(int dim = 0; dim < 3; dim++){
            toi.traj_bbxs_stem.back().first(dim) = c(dim, 0);
            toi.traj_bbxs_stem.back().second(dim) = c(dim, 0);
        }
        for(int i = 1; i < c.cols(); i++){
            for(int dim = 0; dim < 3; dim++){
                toi.traj_bbxs_stem.back().first(dim) = max(c(dim, i), toi.traj_bbxs_stem.back().first(dim));
                toi.traj_bbxs_stem.back().second(dim) = min(c(dim, i), toi.traj_bbxs_stem.back().second(dim));
            }
        }
    }

    if(!GetCorridor(stem_end.head(3), sub_end.head(3), toi.corridors_sub, toi.corridorVs_sub)){
        ROS_WARN("sub failed");
        return false;
    }
    for(auto &c : toi.corridorVs_sub){
        toi.traj_bbxs_sub.emplace_back();
        for(int dim = 0; dim < 3; dim++){
            toi.traj_bbxs_sub.back().first(dim) = c(dim, 0);
            toi.traj_bbxs_sub.back().second(dim) = c(dim, 0);
        }
        for(int i = 1; i < c.cols(); i++){
            for(int dim = 0; dim < 3; dim++){
                toi.traj_bbxs_sub.back().first(dim) = max(c(dim, i), toi.traj_bbxs_sub.back().first(dim));
                toi.traj_bbxs_sub.back().second(dim) = min(c(dim, i), toi.traj_bbxs_sub.back().second(dim));
            }
        }
    }

    if(!GetCorridor(stem_end.head(3), main_end.head(3), toi.corridors_main, toi.corridorVs_main)){
        ROS_WARN("main failed");
        return false;
    }
    for(auto &c : toi.corridorVs_main){
        toi.traj_bbxs_main.emplace_back();
        for(int dim = 0; dim < 3; dim++){
            toi.traj_bbxs_main.back().first(dim) = c(dim, 0);
            toi.traj_bbxs_main.back().second(dim) = c(dim, 0);
        }
        for(int i = 1; i < c.cols(); i++){
            for(int dim = 0; dim < 3; dim++){
                toi.traj_bbxs_main.back().first(dim) = max(c(dim, i), toi.traj_bbxs_main.back().first(dim));
                toi.traj_bbxs_main.back().second(dim) = min(c(dim, i), toi.traj_bbxs_main.back().second(dim));
            }
        }
    }

    double tc = ros::WallTime::now().toSec();
    toi.ref_trajs = swarm_trajs_;
    toi.ref_traj_bbxs = swarm_trajs_bbx_;
    toi.swarm_t.resize(toi.ref_trajs.size(), tc);
    toi.start_t = tc;
    toi.poses = swarm_poses_;
    toi.initState.resize(4, 3);
    toi.initState.setZero();
    toi.initState.col(0) = stem_init;
    toi.endState_stem = stem_end;
    toi.endState_main = main_end;
    toi.endState_sub = sub_end;
    toi.camFov = camFov_;
    toi.camV = camV_;

    if(TO_->AggressiveBranchTrajOptimize(toi)){
        ROS_WARN("PlanTraj2");
        swarm_trajs_.emplace_back(TO_->norm_traj_);
        VisTraj(TO_->main_traj_, 3, 3);
        VisTraj(TO_->sub_traj_, 4, 4);
        VisTraj(TO_->stem_traj_, 5, 5);
        VisCorridors(toi.corridors_stem, -50);
        VisCorridors(toi.corridors_main, -100);
        VisCorridors(toi.corridors_sub, -150);
        return true;
    }
    else{
        ROS_ERROR("norm opt failed");
        return false;
    }
}

// void ShowResult(const ros::TimerEvent &e){

// }

void StateCallback(const geometry_msgs::PoseWithCovarianceStamped &msg){
    switch (state_)
    {
        case 0:{
            state_++;
            break;
        }
        case 1:{
            state_++;
            break;
        }
        case 2:{
            state_++;
            break;
        }
        case 3:{
            state_ = 4;
            tc_ = ros::WallTime::now().toSec();
            if(!PlanBranch()){
                state_ = 1;
            }
            break;
        }
        default:{
            PlanBranch();
            int id = 10000;
            for(auto & traj : swarm_trajs_){
                VisTraj(traj, id, 1);
                id++;
            }
            tc_ = ros::WallTime::now().toSec();
        }
    }
    ROS_WARN("state: %d", state_);
}

void GoalCallback(const geometry_msgs::PoseStamped &msg){
    Eigen::Vector4d p;
    p(0) = msg.pose.position.x;
    p(1) = msg.pose.position.y;
    p(2) = 2.0;
    p(3) = atan2(msg.pose.orientation.z, msg.pose.orientation.w) * 2;
    cout<<"p:"<<p.transpose()<<endl;
    Eigen::Vector3d p3 = p.head(3);
    cout<<"fea:"<<LRM_->IsFeasible(p3)<<endl;;

    switch (state_)
    {
        case 0:{
            if(init_){
                swarm_init = p;
                init_ = false;
            }
            else{
                swarm_end = p;
                init_ = true;
                PlanTraj();
            }
            break;
        }
        case 1:{
            cout<<"set main_end:"<<p.transpose()<<endl;
            VisFov(p, -2);
            main_end = p;
            break;
        }
        case 2:{
            cout<<"set sub_end:"<<p.transpose()<<endl;
            VisFov(p, -3);
            sub_end = p;
            break;
        }
        default:{
            if(init_){

                cout<<"set stem_init:"<<p.transpose()<<endl;
                VisFov(p, -4);
                stem_init = p;
                init_ = false;
            }
            else{
                cout<<"set stem_end:"<<p.transpose()<<endl;
                VisFov(p, -5);
                stem_end = p;
                init_ = true;
            }
            break;
        }
    }
    ROS_WARN("state: %d, init: %d", state_, init_);
}

void TrajTimerCallback(const ros::TimerEvent &e){
    if(state_ != 4) return;
    double tc = ros::WallTime::now().toSec() - tc_;
    VisMotion(tc);
}

void Debug(list<Eigen::Vector3d> &pts, int id){
    visualization_msgs::Marker mk;// mkr1, mkr2;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = id;
    if(id == 0){
        mk.pose.position.z = 0.0;
        mk.color.g = 1.0;
    }
    else{
        mk.color.b = 1.0;
        mk.pose.position.z = 0.0;
    }
    // scan_count_++;
    mk.action = visualization_msgs::Marker::ADD;
    mk.type = visualization_msgs::Marker::LINE_STRIP;
    mk.scale.x = 0.03;
    mk.scale.y = 0.03;
    mk.scale.z = 0.03;
    mk.color.a = 1.0;
    mk.pose.position.x = 0;
    mk.pose.position.y = 0;
    mk.pose.orientation.x = 0;
    mk.pose.orientation.y = 0;
    mk.pose.orientation.z = 0;
    mk.pose.orientation.w = 1;

    geometry_msgs::Point pt;
    for(auto &p : pts){
        // cout<<p.transpose()<<endl;
        pt.x = p.x();
        pt.y = p.y();
        pt.z = p.z();
        mk.points.push_back(pt);
    }
    debug_pub_.publish(mk);
}


int main(int argc, char** argv){
    ros::init(argc, argv, "frontier_test");
    ros::NodeHandle nh, nh_private("~");

    string ns = ros::this_node::getName(), occ_path, free_path;
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InstallFailureSignalHandler();

    debug_pub_ = nh.advertise<visualization_msgs::Marker>("/DebugVis", 10);
    vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/Vis", 10);
    target_sub_ = nh.subscribe("/move_base_simple/goal", 10, &GoalCallback);
    state_sub_ = nh.subscribe("/initialpose", 10, &StateCallback);
    motion_timer_ = nh.createTimer(ros::Duration(0.01), &TrajTimerCallback);

    Eigen::Vector3d ps, pe;
    tc_ = 1000000;
    nh_private.param(ns + "/block_map/OccPath", 
        occ_path, occ_path);
    nh_private.param(ns + "/block_map/FreePath", 
        free_path, free_path);
    nh_private.param(ns + "/Test/StartX", 
        ps(0), 7.5);
    nh_private.param(ns + "/Test/StartY", 
        ps(1), 7.5);
    nh_private.param(ns + "/Test/StartZ", 
        ps(2), 1.5);
    nh_private.param(ns + "/Test/EndX", 
        pe(0), -7.5);
    nh_private.param(ns + "/Test/EndY", 
        pe(1), -7.5);
    nh_private.param(ns + "/Test/EndZ", 
        pe(2), 1.5);
    nh_private.param(ns + "/Test/State", 
        state_, 0);
    
    // Murder M_planner_;
    // M_planner_.init(nh, nh_private);
    BlockMap BM;
    SwarmDataManager SDM;
    lowres::LowResMap LRM;
    ColorManager CM;
    TrajOptimizer TO;

    stem_init = Eigen::Vector4d(-3.49074 , 2.14317 ,       2, 0.361981);
    stem_end = Eigen::Vector4d(2.12674,  3.82568 ,       2, 0.578547);
    sub_end = Eigen::Vector4d(7.22573, 5.49026,        2, 0.502333);
    main_end = Eigen::Vector4d(3.06728, 7.45348,       2, 1.23064);

    SDM.init(nh, nh_private);
    BM.SetSwarmDataManager(&SDM);
    CM.init(nh, nh_private);
    LRM.SetColorManager(&CM);
    LRM.SetMap(&BM);
    LRM.init(nh, nh_private);
    TO.Init(nh, nh_private);
    Eigen::Matrix4d T;

    // M_planner_.GetBlockMapPtr(BM);
    BM.init(nh, nh_private);
    TO_ = &TO;
    BM_ = &BM;
    LRM_ = &LRM;
    cout<<"occ_path:"<<occ_path<<endl;
    cout<<"free_path:"<<free_path<<endl;
    init_ = 1;
    state_ = 0;

    BM.LoadRawMap(occ_path, free_path, 1, true);
    LRM.UpdateLocalBBX(T, pair<Eigen::Vector3d, Eigen::Vector3d>{Eigen::Vector3d(10, 10, 5), Eigen::Vector3d(-10, -10, 0)});

    theta_ = M_PI * 0.45;
    psi_ = M_PI * 0.35;
    camFov_.resize(4, 4);
    camFov_ << -sin(theta_ * 0.5), cos(theta_ * 0.5), 0, 0,
                -sin(theta_ * 0.5), -cos(theta_ * 0.5), 0, 0,
                -sin(psi_ * 0.5), 0, cos(psi_ * 0.5), 0, 
                -sin(psi_ * 0.5), 0, -cos(psi_ * 0.5), 0;
    camV_.resize(3,5);
    camV_<<0.0, 1.0 , 1.0, 1.0, 1.0,
            0.0, 1.0 / tan(theta_ * 0.5), 1.0 / tan(theta_ * 0.5), -1.0 / tan(theta_ * 0.5), -1.0 / tan(theta_ * 0.5),
            0.0, 1.0 / tan(psi_ * 0.5), -1.0 / tan(psi_ * 0.5), 1.0 / tan(psi_ * 0.5), -1.0 / tan(psi_ * 0.5);
    ROS_WARN("finish loading");
    ros::Duration(1.2).sleep();
    for(int i = 0; i < 10; i++){
        ros::spinOnce();
        ros::Duration(0.2).sleep();
    }


    point_count_ = 0;
    ros::spin();
    return 0;
}