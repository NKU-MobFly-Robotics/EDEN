#ifndef EDEN_FSM_H_
#define EDEN_FSM_H_
#include <eden/eden.h>
class SingleExpFSM{
public:
    SingleExpFSM(){};
    ~SingleExpFSM(){};
    void Init(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
    enum M_State{EXCUTE, SLEEP, RECOVER, FINISH, LOCALPLAN, TRAJREPLAN};
    vector<string> state_names = {"EXCUTE", "SLEEP", "RECOVER", "FINISH", "LOCALPLAN", "TRAJREPLAN"};
    void init(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
private:
    void FSMCallback(const ros::TimerEvent &e);
    void TriggerCallback(const std_msgs::EmptyConstPtr &msg);
    void ChangeState(const M_State &state);
    void DebugSub(const std_msgs::Empty &msg);

    bool exploring_, start_trigger_;
    bool force_sample_;
    ros::Subscriber trigger_sub_, debug_sub_;
    SingleExp S_planner_;
    ros::Timer fsm_timer_;
    M_State state_;
    double t0_;
};
inline void SingleExpFSM::ChangeState(const M_State &state){
    // ROS_WARN("FSM from %d to %d", state_, state);
    if(state == LOCALPLAN) force_sample_ = true;
    std::cout << "from  \033[0;46m"<<state_names[state_]<<"\033[0m to \033[0;46m" <<state_names[state]<<"\033[0m" << std::endl;
    state_ = state;
}
#endif

//                 @@@@@
//                 ##**#%@@
//           @@@@%*-:::--+%
//         @@%#*%#-::::=:.+@%
//        @#=:..-#-.:::+++#@%
//       @#:...:.:#-...:*@@%
//     @%=........-#:++=+%@%
//    @%- . =:.....+*-...:#@%
//    @%: . =#......#: ... +@%
//    @%:   -:.....:#..... :%@
//      @*: ...... -* ..... =@%
//       @%=:... . -= ..... +@%
//        @@*==--::--:...  .%@%
//        @+=++++=======--=*@%
//       @#++++==========+@@
//      @#++++====-=======*%
//     %*+++===--===----===+%
//    #*++===--=*%%%#=-=====*@
//  *++==--+%@@%%%@%*--====#@
// #*====-=#@        @#=--===%@
// #*+=--+##          @@*=--=+#@
// #*+++=*#              @%+=+++%
// #*++=+#*                #++++*#
// #*++=+#                  #=+++#
// #*+++#                   %*=++*#
// #++=*%                   #%+=++#
                            