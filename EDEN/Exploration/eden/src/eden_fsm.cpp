#include <eden/eden_fsm.h>

void SingleExpFSM::init(ros::NodeHandle &nh, ros::NodeHandle &nh_private){
    ros::NodeHandle nh_ = nh;
    trigger_sub_ = nh_.subscribe("/start_trigger", 1, &SingleExpFSM::TriggerCallback, this);
    debug_sub_ = nh_.subscribe("/debug_topic", 1, &SingleExpFSM::DebugSub, this);
    fsm_timer_ = nh.createTimer(ros::Duration(0.01), &SingleExpFSM::FSMCallback, this);
    S_planner_.init(nh, nh_private);
    exploring_ = false;
    state_ = M_State::SLEEP;
    t0_ = ros::WallTime::now().toSec();
    force_sample_ = false;
}

void SingleExpFSM::TriggerCallback(const std_msgs::EmptyConstPtr &msg){
    start_trigger_ = true;
    exploring_ = true;
}

void SingleExpFSM::DebugSub(const std_msgs::Empty &msg){
    S_planner_.StopDebugFunc();
    getchar();
}

void SingleExpFSM::FSMCallback(const ros::TimerEvent &e){
    bool exc_plan = false;
    int ap = S_planner_.AllowPlan(ros::WallTime::now().toSec());
    
    if(ap == 0) exc_plan = true;                                                                           
    else if(ap == 1) exc_plan = true;                  // satisfy plan interval   
    else if(ap == 2) exc_plan = true;                   // infeasible position           
    // else if(ap == 3 /*&& (state_ == M_State::EXCUTE || state_ == M_State::LOCALPLAN)*/) exc_plan = true; //only allow check viewpoints and check the traj feasibility
    else if(ap == 3) {
        // if(state_ == M_State::LOCALPLAN){
        //     exc_plan = false;
        //     S_planner_.ForceUpdateEroiDtg();

        // } 
        // else{
            exc_plan = true;
        // }
    }
    // else if(ap == 4 && state_ == M_State::LOCALPLAN) exc_plan = true;
    else if(ap == 5) exc_plan = true;                   // traj time up

    if(force_sample_){
        force_sample_ = false;
        exc_plan = false;
        S_planner_.ForceUpdateEroiDtg();
    }
    // cout<<"ap:"<<ap<<endl;
    // cout<<"state_:"<<int(state_)<<endl;

    if(!exc_plan) {
        if(state_ == M_State::LOCALPLAN){
            cout<<"ap:"<<ap<<endl;
            // ROS_WARN("not allow plan");
            // cout<<"state_:"<<state_<<endl;
            // cout<<"plan:"<<S_planner_.plan_t_ - t0_<<endl;
            // cout<<"dt:"<<S_planner_.plan_t_ - ros::WallTime::now().toSec()<<endl;
        }
        return;
    }
    switch (state_)
    {
        case M_State::EXCUTE :{
            /* trajectory check */
            if(!S_planner_.TrajCheck()){     
                if(ap == 2){                 //recovery
                    S_planner_.SetPlanInterval(0.009);
                    break;
                }
                else{   //try to replan
                    ROS_WARN("traj occ or relpan");
                    ChangeState(M_State::LOCALPLAN);
                    S_planner_.SetPlanInterval(0.009);
                    break;
                }
            }

            /* viewpoints check */
            int vp_st = S_planner_.ViewPointsCheck(0.005);
            if(vp_st == 1){        //local plan
                if(ap == 5) ROS_WARN("traj time out");
                if(vp_st == 1) ROS_WARN("target explored");
                ChangeState(M_State::LOCALPLAN);
                S_planner_.SetPlanInterval(0.009);
                break;
            }
            
            // if(S_planner_.FindShorterPath()){
            //     ROS_WARN("shorter path");
            //     ChangeState(M_State::LOCALPLAN);
            // }
            // if(S_planner_.AllowReplan()){
            //     ChangeState(M_State::TRAJREPLAN);
            //     S_planner_.SetPlanInterval(0.009);
            //     break;
            // }

            /* still EXCUTE */
            S_planner_.SetPlanInterval(0.05);
            break;
        }
        case M_State::FINISH :{
            S_planner_.SetPlanInterval(0.009);
            break;
        }
        case M_State::LOCALPLAN :{
            if(ap == 2 || ap == 3){                                       
                S_planner_.SetPlanInterval(0.009);
                // ChangeState(M_State::LOCALPLAN);
                break;
            }

            if(S_planner_.EdenPlan()){                 //plan success
                S_planner_.SetPlanInterval(0.009);
                ChangeState(M_State::EXCUTE);
                break;
            }
            else{
                if(ap == 2)
                    S_planner_.SetPlanInterval(0.009);
                else
                    S_planner_.SetPlanInterval(0.009);
                break;
            }
            break;
        }
        // case M_State::TRAJREPLAN :{
        //     S_planner_.TrajReplan();
        //     S_planner_.SetPlanInterval(0.009);
        //     ChangeState(M_State::EXCUTE);
        //     break;
        // }
        case M_State::SLEEP :{
            if(!exploring_){
                if(start_trigger_)
                    // S_planner_.Stay(S_planner_.init_pose_);
                S_planner_.SetPlanInterval(0.009);
            }
            else{
                ChangeState(M_State::LOCALPLAN);
                S_planner_.SetPlanInterval(0.009);
            }
            break;
        }
    }
}