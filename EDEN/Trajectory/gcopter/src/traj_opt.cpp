#include <gcopter/traj_opt.h>

void TrajOptimizer::Init(ros::NodeHandle &nh, ros::NodeHandle &nh_private){
    upboundVec_.resize(6);
    weightVec_.resize(9);

    string ns = ros::this_node::getName();
    nh_private.param(ns + "/opt/MaxVel", upboundVec_[0], 2.0);
    nh_private.param(ns + "/opt/MaxAcc", upboundVec_[1], 2.0);
    nh_private.param(ns + "/opt/MaxJer", upboundVec_[2], 15.0);
    nh_private.param(ns + "/opt/MaxYawVel", upboundVec_[3], 2.0);
    nh_private.param(ns + "/opt/MaxYawAcc", upboundVec_[4], 2.0);
    // nh_private.param(ns + "/opt/MaxYawJer", upboundVec_[5], 15.0);
    nh_private.param(ns + "/opt/SwarmAvoid", upboundVec_[5], 3.0);
    nh_private.param(ns + "/opt/WeiPos", weightVec_[0], 1000.0);
    nh_private.param(ns + "/opt/WeiVel", weightVec_[1], 100.0);
    nh_private.param(ns + "/opt/WeiAcc", weightVec_[2], 100.0);
    nh_private.param(ns + "/opt/WeiJer", weightVec_[3], 50.0);
    nh_private.param(ns + "/opt/WeiYawVel", weightVec_[4], 50.0);
    nh_private.param(ns + "/opt/WeiYawAcc", weightVec_[5], 50.0);
    nh_private.param(ns + "/opt/WeiSwarm", weightVec_[6], 100.0);
    nh_private.param(ns + "/opt/WeiVelFov", weightVec_[7], 500.0);
    nh_private.param(ns + "/opt/WeiYawReach", weightVec_[8], 500.0);
    nh_private.param(ns + "/opt/WeiT", weightT_, 20.0);
    nh_private.param(ns + "/opt/WeiminT", weight_minT_, 500.0);
    nh_private.param(ns + "/opt/smoothingEps", smoothingEps_, 0.01);
    nh_private.param(ns + "/opt/integralIntervs", integralIntervs_, 16);
    nh_private.param(ns + "/opt/RelCostTol", relCostTol_, 0.00001);
    // cout<<"max VVV:"<<upboundVec_[0]<<endl;

    nh_private.param(ns + "/opt/debug", debug_, false);
    time_t now = time(0);
    std::string Time_ = ctime(&now);
    tm* t=localtime(&now);
    string path = "/home/charliedog/rosprojects/DoomSea/debug/"+to_string(t->tm_year+1900)+"_"+to_string(t->tm_mon+1)+"_"+to_string(t->tm_mday)
    +"_"+to_string(t->tm_hour)+"_"+to_string(t->tm_min)+"_"+to_string(t->tm_sec);


    if(debug_)  debug_write_.open(path+"debug"+".txt", std::ios::out); ;
    swarm_traj_pub_ = nh.advertise<visualization_msgs::Marker>(ns+"/opt/debug", 10);
}

bool TrajOptimizer::Optimize(
                    const vector<Eigen::MatrixX4d> &corridors,
                    const vector<Eigen::Matrix3Xd> &corridorVs,
                    const double &min_t,
                    const Eigen::Matrix3d &initState,
                    const Eigen::Matrix3d &endState,
                    const int &self_id,
                    vector<Trajectory4<5>> &trajs,
                    vector<double> &swarm_t,
                    vector<Eigen::Vector3d> &poses,
                    double start_t){

    // gcopter::GCOPTER_PolytopeSFC gcopter;
    
    // magnitudeBounds = [v_max, a_max]^T
    // penaltyWeights = [pos_weight, vel_weight, acc_weight]^T
    // initialize some constraint parameters
    std::vector<FastCheckTraj> swarm_trajs;
    double dt = 0.05;
    // list<Eigen::Vector3d> debug_pts;
    for(uint64_t i = 0; i < trajs.size(); i++){
        if(i+1 == self_id) continue;
        if(trajs[i].getPieceNum() == 0){
            swarm_trajs.push_back(FastCheckTraj(10.0, dt, poses[i]));
        }

        if(trajs[i].getTotalDuration() + swarm_t[i] > start_t){
            double st = start_t - swarm_t[i];
            swarm_trajs.push_back(FastCheckTraj(trajs[i], dt, st));
        }
        else{
            // std::cout<<i<<"  --  "<<poses[i].transpose()<<std::endl;
            swarm_trajs.push_back(FastCheckTraj(10.0, dt, poses[i]));
        }

    }
    // Debug(debug_pts);

    Eigen::VectorXd magnitudeBounds(6);
    Eigen::VectorXd penaltyWeights(7);
    magnitudeBounds(0) = upboundVec_[0];
    magnitudeBounds(1) = upboundVec_[1];
    magnitudeBounds(2) = upboundVec_[2];
    magnitudeBounds(3) = upboundVec_[3];
    magnitudeBounds(4) = upboundVec_[4];
    magnitudeBounds(5) = upboundVec_[5];
    penaltyWeights(0) = weightVec_[0];
    penaltyWeights(1) = weightVec_[1];
    penaltyWeights(2) = weightVec_[2];
    penaltyWeights(3) = weightVec_[3];
    penaltyWeights(4) = weightVec_[4];
    penaltyWeights(5) = weightVec_[5];
    penaltyWeights(6) = weightVec_[6];

    vector<Eigen::Matrix3Xd> corridorVstemp;
    corridorVstemp.resize(corridorVs.size()*2 - 1);
    
    for(int i = 0; i < int(corridorVs.size()) - 1; i++){
        corridorVstemp[i*2 + 1].resize(3, 8);
        Eigen::Vector3d up_corn, down_corn;
        for(int dim = 0; dim < 3; dim++){
            down_corn(dim) = max(corridorVs[i](dim, 7), corridorVs[i + 1](dim, 7));
            up_corn(dim) = min(corridorVs[i](dim, 0), corridorVs[i + 1](dim, 0));
        }
        for(int dim1 = 0; dim1 <= 1; dim1++){
            for(int dim2 = 0; dim2 <= 1; dim2++){
                for(int dim3 = 0; dim3 <= 1; dim3++){
                    corridorVstemp[i*2 + 1](0, 4*dim3 + 2*dim2 + dim1) = dim1 ? up_corn(0) : down_corn(0);
                    corridorVstemp[i*2 + 1](1, 4*dim3 + 2*dim2 + dim1) = dim2 ? up_corn(1) : down_corn(1);
                    corridorVstemp[i*2 + 1](2, 4*dim3 + 2*dim2 + dim1) = dim3 ? up_corn(2) : down_corn(2);
                }
            }
        }
        for(int j = 1; j < 8; j++){
            corridorVstemp[i*2 + 1].col(j) = corridorVstemp[i*2 + 1].col(j) - corridorVstemp[i*2 + 1].col(0);
        }
        // corridorVstemp[i*2 + 1].col(0) = 
    }
    for(int i = 0; i < int(corridorVs.size()); i++){
        corridorVstemp[i*2] = corridorVs[i];
        Eigen::Vector3d start_p = corridorVstemp[i*2].col(7);
        for(int j = 1; j < 7; j++){
            corridorVstemp[i*2].col(7-j) = corridorVstemp[i*2].col(7-j) - corridorVstemp[i*2].col(7);
        }
        corridorVstemp[i*2].col(7) = corridorVstemp[i*2].col(0) - corridorVstemp[i*2].col(7);
        corridorVstemp[i*2].col(0) = start_p;
    }
    // const int quadratureRes = integralIntervs_;
    // Trajectory4<5> traj_tempt = traj;    
    // traj.clear();

    // if (!gcopter.setup(weightT_, weight_minT_, min_t,
    //                     initState, endState,
    //                     corridors, corridorVstemp,
    //                     INFINITY,
    //                     smoothingEps_,
    //                     quadratureRes,
    //                     magnitudeBounds,
    //                     penaltyWeights,
    //                     swarm_trajs))
    // {
    //     return false;
    // }

    // if (std::isinf(gcopter.optimize(traj, relCostTol_)))
    // {
    //     ROS_ERROR("inf");
    //     cout<<"min_t:"<<min_t<<endl;
    //     cout<<"corridors:"<<corridors.size()<<endl;
    //     traj = traj_tempt;
    //     return false;
    // }

    // if (traj.getPieceNum() > 0)
    // {
    //     trajStamp = ros::Time::now().toSec();
    //     return true;
    // }
    // else return false;
    return false;
}

bool TrajOptimizer::AggressiveBranchTrajOptimize(TrajOptInput &toi){

    Eigen::VectorXd magnitudeBounds(6);
    Eigen::VectorXd penaltyWeights(9);
    magnitudeBounds(0) = upboundVec_[0];
    magnitudeBounds(1) = upboundVec_[1];
    magnitudeBounds(2) = upboundVec_[2];
    magnitudeBounds(3) = upboundVec_[3];
    magnitudeBounds(4) = upboundVec_[4];
    magnitudeBounds(5) = upboundVec_[5];
    penaltyWeights(0) = weightVec_[0];
    penaltyWeights(1) = weightVec_[1];
    penaltyWeights(2) = weightVec_[2];
    penaltyWeights(3) = weightVec_[3];
    penaltyWeights(4) = weightVec_[4];
    penaltyWeights(5) = weightVec_[5];
    penaltyWeights(6) = weightVec_[6];
    penaltyWeights(7) = weightVec_[7];
    penaltyWeights(8) = weightVec_[8];

    vector<Eigen::Matrix3Xd> corridor_stem_i, corridor_main_i, corridor_sub_i;
    GetInterPts(toi.corridorVs_stem, corridor_stem_i);
    GetInterPts(toi.corridorVs_main, corridor_main_i);
    GetInterPts(toi.corridorVs_sub, corridor_sub_i);

    gcopter_aggressive::GCOPTER_AGGRESSIVE gcopter;

    SWARM_TRAJs sts;
    pair<Eigen::Vector3d, Eigen::Vector3d> bbx;
    sts.seg_intersec_ids_stem_.resize(toi.traj_bbxs_stem.size());
    sts.seg_intersec_ids_sub_.resize(toi.traj_bbxs_sub.size());
    sts.seg_intersec_ids_main_.resize(toi.traj_bbxs_main.size());
    for(uint64_t i = 0 ; i < toi.ref_traj_bbxs.size(); i++){
        bbx = toi.ref_traj_bbxs[i];
        // cout<<"bbx ref:"<<bbx.first.transpose()<<"   "<<bbx.second.transpose()<<endl;
        for(uint64_t j = 0; j < toi.traj_bbxs_stem.size(); j++){
            bool intersec = true;
            // cout<<"stem bbx ref:"<<toi.traj_bbxs_stem[j].first.transpose()<<"   "<<toi.traj_bbxs_stem[j].second.transpose()<<endl;
            for(int dim = 0; dim < 3; dim ++){
                if(bbx.first(dim) < toi.traj_bbxs_stem[j].second(dim)){
                    intersec = false;
                    break;
                }
                if(bbx.second(dim) > toi.traj_bbxs_stem[j].first(dim)){
                    intersec = false;
                    break;
                }
            }
            if(intersec){
                sts.seg_intersec_ids_stem_[j].emplace_back(i);
                // cout<<"sts.seg_intersec_ids_stem_[j]:"<<i<<endl;
            }
        }
        for(uint64_t j = 0; j < toi.traj_bbxs_sub.size(); j++){
            bool intersec = true;
            for(int dim = 0; dim < 3; dim ++){
                if(bbx.first(dim) < toi.traj_bbxs_sub[j].second(dim)){
                    intersec = false;
                    break;
                }
                if(bbx.second(dim) > toi.traj_bbxs_sub[j].first(dim)){
                    intersec = false;
                    break;
                }
            }
            if(intersec){
                sts.seg_intersec_ids_sub_[j].emplace_back(i);
                // cout<<"sts.seg_intersec_ids_sub_[j]:"<<i<<endl;
            }
        }
        for(uint64_t j = 0; j < toi.traj_bbxs_main.size(); j++){
            bool intersec = true;
            for(int dim = 0; dim < 3; dim ++){
                if(bbx.first(dim) < toi.traj_bbxs_main[j].second(dim)){
                    intersec = false;
                    break;
                }
                if(bbx.second(dim) > toi.traj_bbxs_main[j].first(dim)){
                    intersec = false;
                    break;
                }
            }
            if(intersec){
                sts.seg_intersec_ids_main_[j].emplace_back(i);
                // cout<<"sts.seg_intersec_ids_main_[j]:"<<i<<endl;
            }
        }
        sts.trajs_.emplace_back(toi.ref_trajs[i]);
        sts.nVs_.emplace_back(-toi.ref_trajs[i].getVel(0).head(3));
        sts.Ps_.emplace_back(toi.ref_trajs[i].getPos(0).head(3));
        sts.durations_.emplace_back(toi.ref_trajs[i].getTotalDuration());
        sts.Ve_.emplace_back(toi.ref_trajs[i].getVel(sts.durations_.back()).head(3));
        sts.Pe_.emplace_back(toi.ref_trajs[i].getPos(sts.durations_.back()).head(3));
        sts.Ts_.emplace_back(toi.swarm_t[i] - toi.start_t);
    }
    for(uint64_t i = 0 ; i < toi.poses.size(); i++){
        sts.SwarmPos_.emplace_back(toi.poses[i]);
    }

    const int quadratureRes = integralIntervs_;

    Trajectory4<5> sub_traj_tempt = sub_traj_;    
    Trajectory4<5> main_traj_tempt = main_traj_;    
    Trajectory4<5> stem_traj_tempt = stem_traj_;    

    // cout<<"toi.endState_stem:"<<toi.endState_stem.transpose()<<endl;
    // cout<<"toi.endState_main:"<<toi.endState_main.transpose()<<endl;
    // cout<<"toi.endState_sub:"<<toi.endState_sub.transpose()<<endl;
    gcopter.setup(  weightT_,
                    toi.initState,
                    toi.endState_stem,
                    toi.endState_main,
                    toi.endState_sub,
                    toi.corridors_stem,
                    corridor_stem_i,
                    toi.corridors_main,
                    corridor_main_i,
                    toi.corridors_sub,
                    corridor_sub_i, 
                    toi.camFov,
                    toi.camV,
                    INFINITY,
                    smoothingEps_,
                    quadratureRes,
                    magnitudeBounds,
                    penaltyWeights,
                    &sts);
    if (std::isinf(gcopter.optimize(stem_traj_, main_traj_, sub_traj_, relCostTol_)))
    {

        
        ROS_ERROR("inf");


        if(debug_) {

            debug_write_<<"relCostTol_:"<<relCostTol_<<endl;
            debug_write_<<"stem_traj_ coef:"<<endl;
            debug_write_<<"stem_traj_ durations:\n"<<stem_traj_.getDurations().transpose()<<endl;
            for(int i = 0; i < stem_traj_.getPieceNum(); i++){
                debug_write_<<"i"<<i<<"\n"<<stem_traj_[i].getCoeffMat()<<endl;
            }

            debug_write_<<"main_traj_ coef:"<<endl;
            debug_write_<<"main_traj_ durations:\n"<<main_traj_.getDurations().transpose()<<endl;
            for(int i = 0; i < main_traj_.getPieceNum(); i++){
                debug_write_<<"i"<<i<<"\n"<<main_traj_[i].getCoeffMat()<<endl;
            }

            debug_write_<<"sub_traj_ coef:"<<endl;
            debug_write_<<"sub_traj_ durations:\n"<<sub_traj_.getDurations().transpose()<<endl;
            for(int i = 0; i < sub_traj_.getPieceNum(); i++){
                debug_write_<<"i"<<i<<"\n"<<sub_traj_[i].getCoeffMat()<<endl;
            }
            debug_write_<<"AggressiveBranchTrajOptimize:"<<endl;

            debug_write_<<"toi initState :\n"<<endl;
            for(int r = 0; r < toi.initState.rows(); r++){
                for(int c = 0; c < toi.initState.cols(); c++){
                    debug_write_<<fixed<<std::setprecision(std::numeric_limits<double>::max_digits10)<<toi.initState(r, c)<<",";
                }
                debug_write_<<endl;
            }
            debug_write_<<";"<<endl;


            debug_write_<<"toi endState_stem ^T:\n";
            for(int i = 0; i < 4; i++){
                debug_write_<<fixed<<std::setprecision(std::numeric_limits<double>::max_digits10)<<toi.endState_stem(i)<<",";
            }
            debug_write_<<";"<<endl;

            debug_write_<<"toi endState_main ^T:\n";
            for(int i = 0; i < 4; i++){
                debug_write_<<fixed<<std::setprecision(std::numeric_limits<double>::max_digits10)<<toi.endState_main(i)<<",";
            }
            debug_write_<<";"<<endl;

            debug_write_<<"toi endState_sub ^T:\n";
            for(int i = 0; i < 4; i++){
                debug_write_<<fixed<<std::setprecision(std::numeric_limits<double>::max_digits10)<<toi.endState_sub(i)<<",";
            }
            debug_write_<<";"<<endl;

            debug_write_<<"toi camFov ^T:\n"<<toi.camFov.transpose()<<endl;
            debug_write_<<"toi camV ^T:\n"<<toi.camV.transpose()<<endl;

            debug_write_<<"toi corridorVs_stem:"<<endl;
            for(uint64_t i = 0; i < toi.corridorVs_stem.size(); i++){
                debug_write_<<"i"<<i<<"\n"<<endl;
                for(int r = 0; r < toi.corridorVs_stem[i].rows(); r++){
                    for(int c = 0; c < toi.corridorVs_stem[i].cols(); c++){
                        debug_write_<<fixed<<std::setprecision(std::numeric_limits<double>::max_digits10)<<toi.corridorVs_stem[i](r, c)<<",";
                    }
                    debug_write_<<endl;
                }
                debug_write_<<";"<<endl;
            }
            debug_write_<<"toi corridors_stem ^T:"<<endl;
            for(uint64_t i = 0; i < toi.corridors_stem.size(); i++){
                for(int j = 0; j < toi.corridors_stem[i].rows(); j++){
                    debug_write_<<fixed<<std::setprecision(std::numeric_limits<double>::max_digits10)<<toi.corridors_stem[i](j, 3);
                    if(j + 1 == toi.corridors_stem[i].rows()) debug_write_<<";"<<endl; 
                    else debug_write_<<","; 
                }
                debug_write_<<endl;
            }
            
            debug_write_<<"toi corridorVs_main:"<<endl;
            for(uint64_t i = 0; i < toi.corridorVs_main.size(); i++){
                debug_write_<<"i"<<i<<"\n"<<endl;
                for(int r = 0; r < toi.corridorVs_main[i].rows(); r++){
                    for(int c = 0; c < toi.corridorVs_main[i].cols(); c++){
                        debug_write_<<fixed<<std::setprecision(std::numeric_limits<double>::max_digits10)<<toi.corridorVs_main[i](r, c)<<",";
                    }
                    debug_write_<<endl;
                }
                debug_write_<<";"<<endl;
            }
            debug_write_<<"toi corridors_main ^T:"<<endl;
            for(uint64_t i = 0; i < toi.corridors_main.size(); i++){
                for(int j = 0; j < toi.corridors_main[i].rows(); j++){
                    debug_write_<<fixed<<std::setprecision(std::numeric_limits<double>::max_digits10)<<toi.corridors_main[i](j, 3);
                    if(j + 1 == toi.corridors_main[i].rows()) debug_write_<<";"<<endl; 
                    else debug_write_<<","; 
                }
                debug_write_<<endl;
            }
            debug_write_<<"toi corridorVs_sub:"<<endl;
            for(uint64_t i = 0; i < toi.corridorVs_sub.size(); i++){
                debug_write_<<"i"<<i<<"\n"<<endl;
                for(int r = 0; r < toi.corridorVs_sub[i].rows(); r++){
                    for(int c = 0; c < toi.corridorVs_sub[i].cols(); c++){
                        debug_write_<<fixed<<std::setprecision(std::numeric_limits<double>::max_digits10)<<toi.corridorVs_sub[i](r, c)<<",";
                    }
                    debug_write_<<endl;
                }
                debug_write_<<";"<<endl;            
            }
            debug_write_<<"toi corridors_sub ^T:"<<endl;
            for(uint64_t i = 0; i < toi.corridors_sub.size(); i++){
                for(int j = 0; j < toi.corridors_sub[i].rows(); j++){
                    debug_write_<<fixed<<std::setprecision(std::numeric_limits<double>::max_digits10)<<toi.corridors_sub[i](j, 3);
                    if(j + 1 == toi.corridors_sub[i].rows()) debug_write_<<";"<<endl; 
                    else debug_write_<<","; 
                }
                debug_write_<<endl;
            }
            // getchar();
        }
        sub_traj_ = sub_traj_tempt;
        main_traj_ = main_traj_tempt;
        stem_traj_ = stem_traj_tempt;
        return false;
    }
    else{
        // ROS_ERROR("ok");


        if(debug_) {

            debug_write_<<"relCostTol_:"<<relCostTol_<<endl;
            debug_write_<<"stem_traj_ coef:"<<endl;
            debug_write_<<"stem_traj_ durations:\n"<<stem_traj_.getDurations().transpose()<<endl;
            for(int i = 0; i < stem_traj_.getPieceNum(); i++){
                debug_write_<<"i"<<i<<"\n"<<stem_traj_[i].getCoeffMat()<<endl;
            }

            debug_write_<<"main_traj_ coef:"<<endl;
            debug_write_<<"main_traj_ durations:\n"<<main_traj_.getDurations().transpose()<<endl;
            for(int i = 0; i < main_traj_.getPieceNum(); i++){
                debug_write_<<"i"<<i<<"\n"<<main_traj_[i].getCoeffMat()<<endl;
            }

            debug_write_<<"sub_traj_ coef:"<<endl;
            debug_write_<<"sub_traj_ durations:\n"<<sub_traj_.getDurations().transpose()<<endl;
            for(int i = 0; i < sub_traj_.getPieceNum(); i++){
                debug_write_<<"i"<<i<<"\n"<<sub_traj_[i].getCoeffMat()<<endl;
            }
            debug_write_<<"AggressiveBranchTrajOptimize:"<<endl;

            debug_write_<<"toi initState :\n"<<endl;
            for(int r = 0; r < toi.initState.rows(); r++){
                for(int c = 0; c < toi.initState.cols(); c++){
                    debug_write_<<fixed<<std::setprecision(std::numeric_limits<double>::max_digits10)<<toi.initState(r, c)<<",";
                }
                debug_write_<<endl;
            }
            debug_write_<<";"<<endl;


            debug_write_<<"toi endState_stem ^T:\n";
            for(int i = 0; i < 4; i++){
                debug_write_<<fixed<<std::setprecision(std::numeric_limits<double>::max_digits10)<<toi.endState_stem(i)<<",";
            }
            debug_write_<<";"<<endl;

            debug_write_<<"toi endState_main ^T:\n";
            for(int i = 0; i < 4; i++){
                debug_write_<<fixed<<std::setprecision(std::numeric_limits<double>::max_digits10)<<toi.endState_main(i)<<",";
            }
            debug_write_<<";"<<endl;

            debug_write_<<"toi endState_sub ^T:\n";
            for(int i = 0; i < 4; i++){
                debug_write_<<fixed<<std::setprecision(std::numeric_limits<double>::max_digits10)<<toi.endState_sub(i)<<",";
            }
            debug_write_<<";"<<endl;

            debug_write_<<"toi camFov ^T:\n"<<toi.camFov.transpose()<<endl;
            debug_write_<<"toi camV ^T:\n"<<toi.camV.transpose()<<endl;

            debug_write_<<"toi corridorVs_stem:"<<endl;
            for(uint64_t i = 0; i < toi.corridorVs_stem.size(); i++){
                debug_write_<<"i"<<i<<"\n"<<endl;
                for(int r = 0; r < toi.corridorVs_stem[i].rows(); r++){
                    for(int c = 0; c < toi.corridorVs_stem[i].cols(); c++){
                        debug_write_<<fixed<<std::setprecision(std::numeric_limits<double>::max_digits10)<<toi.corridorVs_stem[i](r, c)<<",";
                    }
                    debug_write_<<endl;
                }
                debug_write_<<";"<<endl;
            }
            debug_write_<<"toi corridors_stem ^T:"<<endl;
            for(uint64_t i = 0; i < toi.corridors_stem.size(); i++){
                for(int j = 0; j < toi.corridors_stem[i].rows(); j++){
                    debug_write_<<fixed<<std::setprecision(std::numeric_limits<double>::max_digits10)<<toi.corridors_stem[i](j, 3);
                    if(j + 1 == toi.corridors_stem[i].rows()) debug_write_<<";"<<endl; 
                    else debug_write_<<","; 
                }
                debug_write_<<endl;
            }
            
            debug_write_<<"toi corridorVs_main:"<<endl;
            for(uint64_t i = 0; i < toi.corridorVs_main.size(); i++){
                debug_write_<<"i"<<i<<"\n"<<endl;
                for(int r = 0; r < toi.corridorVs_main[i].rows(); r++){
                    for(int c = 0; c < toi.corridorVs_main[i].cols(); c++){
                        debug_write_<<fixed<<std::setprecision(std::numeric_limits<double>::max_digits10)<<toi.corridorVs_main[i](r, c)<<",";
                    }
                    debug_write_<<endl;
                }
                debug_write_<<";"<<endl;
            }
            debug_write_<<"toi corridors_main ^T:"<<endl;
            for(uint64_t i = 0; i < toi.corridors_main.size(); i++){
                for(int j = 0; j < toi.corridors_main[i].rows(); j++){
                    debug_write_<<fixed<<std::setprecision(std::numeric_limits<double>::max_digits10)<<toi.corridors_main[i](j, 3);
                    if(j + 1 == toi.corridors_main[i].rows()) debug_write_<<";"<<endl; 
                    else debug_write_<<","; 
                }
                debug_write_<<endl;
            }
            debug_write_<<"toi corridorVs_sub:"<<endl;
            for(uint64_t i = 0; i < toi.corridorVs_sub.size(); i++){
                debug_write_<<"i"<<i<<"\n"<<endl;
                for(int r = 0; r < toi.corridorVs_sub[i].rows(); r++){
                    for(int c = 0; c < toi.corridorVs_sub[i].cols(); c++){
                        debug_write_<<fixed<<std::setprecision(std::numeric_limits<double>::max_digits10)<<toi.corridorVs_sub[i](r, c)<<",";
                    }
                    debug_write_<<endl;
                }
                debug_write_<<";"<<endl;            
            }
            debug_write_<<"toi corridors_sub ^T:"<<endl;
            for(uint64_t i = 0; i < toi.corridors_sub.size(); i++){
                for(int j = 0; j < toi.corridors_sub[i].rows(); j++){
                    debug_write_<<fixed<<std::setprecision(std::numeric_limits<double>::max_digits10)<<toi.corridors_sub[i](j, 3);
                    if(j + 1 == toi.corridors_sub[i].rows()) debug_write_<<";"<<endl; 
                    else debug_write_<<","; 
                }
                debug_write_<<endl;
            }
            // getchar();
        }
        // sub_traj_ = sub_traj_tempt;
        // main_traj_ = main_traj_tempt;
        // stem_traj_ = stem_traj_tempt;
    }

    return true;
}

// bool TrajOptimizer::CoverBranchTrajOptimize(TrajOptInput &toi){
//     Eigen::VectorXd magnitudeBounds(6);
//     Eigen::VectorXd penaltyWeights(8);
//     magnitudeBounds(0) = upboundVec_[0];
//     magnitudeBounds(1) = upboundVec_[1];
//     magnitudeBounds(2) = upboundVec_[2];
//     magnitudeBounds(3) = upboundVec_[3];
//     magnitudeBounds(4) = upboundVec_[4];
//     magnitudeBounds(5) = upboundVec_[5];
//     penaltyWeights(0) = weightVec_[0];
//     penaltyWeights(1) = weightVec_[1];
//     penaltyWeights(2) = weightVec_[2];
//     penaltyWeights(3) = weightVec_[3];
//     penaltyWeights(4) = weightVec_[4];
//     penaltyWeights(5) = weightVec_[5];
//     penaltyWeights(6) = weightVec_[6];
//     penaltyWeights(7) = weightVec_[7];

//     vector<Eigen::Matrix3Xd> corridor_stem_i, corridor_main_i, corridor_sub_i;
//     GetInterPts(toi.corridorVs_stem, corridor_stem_i);
//     GetInterPts(toi.corridorVs_main, corridor_main_i);
//     GetInterPts(toi.corridorVs_sub, corridor_sub_i);

//     gcopter_cover::GCOPTER_COVER gcopter;

//     SWARM_TRAJs sts;
//     pair<Eigen::Vector3d, Eigen::Vector3d> bbx;
//     sts.seg_intersec_ids_stem_.resize(toi.traj_bbxs_stem.size());
//     sts.seg_intersec_ids_sub_.resize(toi.traj_bbxs_sub.size());
//     sts.seg_intersec_ids_main_.resize(toi.traj_bbxs_main.size());
//     for(uint64_t i = 0 ; i < toi.ref_traj_bbxs.size(); i++){
//         bbx = toi.ref_traj_bbxs[i];
//         // cout<<"bbx ref:"<<bbx.first.transpose()<<"   "<<bbx.second.transpose()<<endl;
//         for(uint64_t j = 0; j < toi.traj_bbxs_stem.size(); j++){
//             bool intersec = true;
//             // cout<<"stem bbx ref:"<<toi.traj_bbxs_stem[j].first.transpose()<<"   "<<toi.traj_bbxs_stem[j].second.transpose()<<endl;
//             for(int dim = 0; dim < 3; dim ++){
//                 if(bbx.first(dim) < toi.traj_bbxs_stem[j].second(dim)){
//                     intersec = false;
//                     break;
//                 }
//                 if(bbx.second(dim) > toi.traj_bbxs_stem[j].first(dim)){
//                     intersec = false;
//                     break;
//                 }
//             }
//             if(intersec){
//                 sts.seg_intersec_ids_stem_[j].emplace_back(i);
//                 // cout<<"sts.seg_intersec_ids_stem_[j]:"<<i<<endl;
//             }
//         }
//         for(uint64_t j = 0; j < toi.traj_bbxs_sub.size(); j++){
//             bool intersec = true;
//             for(int dim = 0; dim < 3; dim ++){
//                 if(bbx.first(dim) < toi.traj_bbxs_sub[j].second(dim)){
//                     intersec = false;
//                     break;
//                 }
//                 if(bbx.second(dim) > toi.traj_bbxs_sub[j].first(dim)){
//                     intersec = false;
//                     break;
//                 }
//             }
//             if(intersec){
//                 sts.seg_intersec_ids_sub_[j].emplace_back(i);
//                 // cout<<"sts.seg_intersec_ids_sub_[j]:"<<i<<endl;
//             }
//         }
//         for(uint64_t j = 0; j < toi.traj_bbxs_main.size(); j++){
//             bool intersec = true;
//             for(int dim = 0; dim < 3; dim ++){
//                 if(bbx.first(dim) < toi.traj_bbxs_main[j].second(dim)){
//                     intersec = false;
//                     break;
//                 }
//                 if(bbx.second(dim) > toi.traj_bbxs_main[j].first(dim)){
//                     intersec = false;
//                     break;
//                 }
//             }
//             if(intersec){
//                 sts.seg_intersec_ids_main_[j].emplace_back(i);
//                 // cout<<"sts.seg_intersec_ids_main_[j]:"<<i<<endl;
//             }
//         }
//         sts.trajs_.emplace_back(toi.ref_trajs[i]);
//         sts.nVs_.emplace_back(-toi.ref_trajs[i].getVel(0).head(3));
//         sts.Ps_.emplace_back(toi.ref_trajs[i].getPos(0).head(3));
//         sts.durations_.emplace_back(toi.ref_trajs[i].getTotalDuration());
//         sts.Ve_.emplace_back(toi.ref_trajs[i].getVel(sts.durations_.back()).head(3));
//         sts.Pe_.emplace_back(toi.ref_trajs[i].getPos(sts.durations_.back()).head(3));
//         sts.Ts_.emplace_back(toi.swarm_t[i] - toi.start_t);
//     }
//     for(uint64_t i = 0 ; i < toi.poses.size(); i++){
//         sts.SwarmPos_.emplace_back(toi.poses[i]);
//     }

//     const int quadratureRes = integralIntervs_;

//     Trajectory4<5> sub_traj_tempt = sub_traj_;    
//     Trajectory4<5> main_traj_tempt = main_traj_;    
//     Trajectory4<5> stem_traj_tempt = stem_traj_;    


//     gcopter.setup(  weightT_,
//                     toi.initState,
//                     toi.endState_stem,
//                     toi.endState_main,
//                     toi.endState_sub,
//                     toi.corridors_stem,
//                     corridor_stem_i,
//                     toi.corridors_main,
//                     corridor_main_i,
//                     toi.corridors_sub,
//                     corridor_sub_i, 
//                     toi.camFov,
//                     // toi.camV,
//                     INFINITY,
//                     smoothingEps_,
//                     quadratureRes,
//                     magnitudeBounds,
//                     penaltyWeights,
//                     &sts);
//     if (std::isinf(gcopter.optimize(stem_traj_, main_traj_, sub_traj_, relCostTol_)))
//     {
//         ROS_ERROR("inf");
//         sub_traj_ = sub_traj_tempt;
//         main_traj_ = main_traj_tempt;
//         stem_traj_ = stem_traj_tempt;
//         return false;
//     }
//     return true;

// }

bool TrajOptimizer::FormTrajOptimize(TrajOptInput &toi){
    return false;

}

bool TrajOptimizer::NormTrajOptimize(TrajOptInput &toi){

    Eigen::VectorXd magnitudeBounds(6);
    Eigen::VectorXd penaltyWeights(9);
    magnitudeBounds(0) = upboundVec_[0];
    magnitudeBounds(1) = upboundVec_[1];
    magnitudeBounds(2) = upboundVec_[2];
    magnitudeBounds(3) = upboundVec_[3];
    magnitudeBounds(4) = upboundVec_[4];
    magnitudeBounds(5) = upboundVec_[5];
    penaltyWeights(0) = weightVec_[0];
    penaltyWeights(1) = weightVec_[1];
    penaltyWeights(2) = weightVec_[2];
    penaltyWeights(3) = weightVec_[3];
    penaltyWeights(4) = weightVec_[4];
    penaltyWeights(5) = weightVec_[5];
    penaltyWeights(6) = weightVec_[6];
    penaltyWeights(7) = weightVec_[7];
    penaltyWeights(8) = weightVec_[8];
    // ROS_WARN("NormTrajOptimize0");

    vector<Eigen::Matrix3Xd> corridor_norm;
    GetInterPts(toi.corridorVs_norm, corridor_norm);
    // ROS_WARN("NormTrajOptimize1");

    gcopter_norm::GCOPTER_NORM gcopter;

    SWARM_TRAJs sts;
    pair<Eigen::Vector3d, Eigen::Vector3d> bbx;
    sts.seg_intersec_ids_norm_.resize(toi.traj_bbxs_norm.size());
    for(uint64_t i = 0 ; i < toi.ref_traj_bbxs.size(); i++){
        bbx = toi.ref_traj_bbxs[i];
        for(uint64_t j = 0; j < toi.traj_bbxs_norm.size(); j++){
            bool intersec = true;
            for(int dim = 0; dim < 3; dim ++){
                if(bbx.first(dim) < toi.traj_bbxs_norm[j].second(dim)){
                    intersec = false;
                    break;
                }
                if(bbx.second(dim) > toi.traj_bbxs_norm[j].first(dim)){
                    intersec = false;
                    break;
                }
            }
            if(intersec){
                sts.seg_intersec_ids_norm_[i].emplace_back(j);
            }
        }
        sts.trajs_.emplace_back(toi.ref_trajs[i]);
        sts.nVs_.emplace_back(-toi.ref_trajs[i].getVel(0).head(3));
        sts.Ps_.emplace_back(toi.ref_trajs[i].getPos(0).head(3));
        sts.durations_.emplace_back(toi.ref_trajs[i].getTotalDuration());
        sts.Ve_.emplace_back(toi.ref_trajs[i].getVel(sts.durations_.back()).head(3));
        sts.Pe_.emplace_back(toi.ref_trajs[i].getPos(sts.durations_.back()).head(3));
        sts.Ts_.emplace_back(toi.swarm_t[i] - toi.start_t);
    }
    // ROS_WARN("NormTrajOptimize2");

    for(uint64_t i = 0 ; i < toi.poses.size(); i++){
        sts.SwarmPos_.emplace_back(toi.poses[i]);
    }
    // ROS_WARN("NormTrajOptimize3");


    const int quadratureRes = integralIntervs_;

    Trajectory4<5> norm_traj_tempt = norm_traj_;    
    // ROS_WARN("NormTrajOptimize4");

    gcopter.setup(  weightT_,
                    toi.initState,
                    toi.endState_norm,
                    toi.corridors_norm,
                    corridor_norm,
                    INFINITY,
                    smoothingEps_,
                    quadratureRes,
                    magnitudeBounds,
                    penaltyWeights,
                    &sts);
    // ROS_WARN("NormTrajOptimize5");

    if (std::isinf(gcopter.optimize(norm_traj_, relCostTol_)))
    {
        ROS_ERROR("inf");
        norm_traj_ = norm_traj_tempt;
        return false;
    }
    return true;
}

void TrajOptimizer::GetInterPts(const vector<Eigen::Matrix3Xd> &corridorVs, vector<Eigen::Matrix3Xd> &corridorVsIters){
    corridorVsIters.resize(corridorVs.size()*2 - 1);
    
    for(int i = 0; i < int(corridorVs.size()) - 1; i++){
        corridorVsIters[i*2 + 1].resize(3, 8);
        Eigen::Vector3d up_corn, down_corn;
        for(int dim = 0; dim < 3; dim++){
            down_corn(dim) = max(corridorVs[i](dim, 7), corridorVs[i + 1](dim, 7));
            up_corn(dim) = min(corridorVs[i](dim, 0), corridorVs[i + 1](dim, 0));
        }
        for(int dim1 = 0; dim1 <= 1; dim1++){
            for(int dim2 = 0; dim2 <= 1; dim2++){
                for(int dim3 = 0; dim3 <= 1; dim3++){
                    corridorVsIters[i*2 + 1](0, 4*dim3 + 2*dim2 + dim1) = dim1 ? up_corn(0) : down_corn(0);
                    corridorVsIters[i*2 + 1](1, 4*dim3 + 2*dim2 + dim1) = dim2 ? up_corn(1) : down_corn(1);
                    corridorVsIters[i*2 + 1](2, 4*dim3 + 2*dim2 + dim1) = dim3 ? up_corn(2) : down_corn(2);
                }
            }
        }
        for(int j = 1; j < 8; j++){
            corridorVsIters[i*2 + 1].col(j) = corridorVsIters[i*2 + 1].col(j) - corridorVsIters[i*2 + 1].col(0);
        }
    }
    for(int i = 0; i < int(corridorVs.size()); i++){
        corridorVsIters[i*2] = corridorVs[i];
        Eigen::Vector3d start_p = corridorVsIters[i*2].col(7);
        for(int j = 1; j < 7; j++){
            corridorVsIters[i*2].col(7-j) = corridorVsIters[i*2].col(7-j) - corridorVsIters[i*2].col(7);
        }
        corridorVsIters[i*2].col(7) = corridorVsIters[i*2].col(0) - corridorVsIters[i*2].col(7);
        corridorVsIters[i*2].col(0) = start_p;
    }
}


void TrajOptimizer::Debug(list<Eigen::Vector3d> &debug_list){
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = -2;
    mk.action = visualization_msgs::Marker::ADD;
    mk.type = visualization_msgs::Marker::SPHERE_LIST;
    mk.scale.x = 0.1;
    mk.scale.y = 0.1;
    mk.scale.z = 0.1;
    mk.color.a = 1.0;
    mk.color.r = 0.9;
    mk.color.g = 0.0;
    mk.color.b = 0.1;
    for(auto &pt : debug_list){
        geometry_msgs::Point p;
        p.x = pt(0);
        p.y = pt(1);
        p.z = pt(2);
        mk.points.emplace_back(p);
    }
    swarm_traj_pub_.publish(mk);
}
