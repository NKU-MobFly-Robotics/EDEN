


#ifndef LOW_RESOLUTION_MAP_LITE_H_
#define LOW_RESOLUTION_MAP_LITE_H_
#include <ros/ros.h>
#include <thread>
#include <Eigen/Eigen>
#include <vector>
#include <list>
#include <block_map_lite/block_map_lite.h>
#include <block_map_lite/raycast.h>
#include <block_map_lite/color_manager.h>
#include <visualization_msgs/MarkerArray.h>
#include <tr1/unordered_map>
#include <queue>
#include <random>
#include <mutex>
#include <bitset>

#include <glog/logging.h>




using namespace std;

namespace lowres_lite{
// For path searching
enum SchState{ in_close = 1, in_open = 2, not_expand = 3};

struct sch_node{
    sch_node(){
        parent_ = NULL;
        f_score_ = 0;
        g_score_ = 0;
        h_g_score_ = 0;
        root_id_ = 0;
        status_ = not_expand;
    }
    sch_node(shared_ptr<sch_node> &node){
        pos_ = node->pos_;
        parent_ = node->parent_;
        status_ = node->status_;
        f_score_ = node->f_score_;
        h_g_score_ = node->g_score_;
        g_score_ = node->h_g_score_;
    }
    Eigen::Vector3i pos_;
    shared_ptr<sch_node> parent_;
    SchState status_;
    uint32_t root_id_;
    double f_score_;
    double g_score_, h_g_score_;
};

class ACompare {
public:
  bool operator()(shared_ptr<sch_node> node1, shared_ptr<sch_node> node2) {
    return node1->f_score_ > node2->f_score_;
  }
};

class DCompare {
public:
  bool operator()(shared_ptr<sch_node> node1, shared_ptr<sch_node> node2) {
    return node1->g_score_ > node2->g_score_;
  }
};

// struct lr_root{
//     uint32_t root_id_;
//     u_char in_dir_; // (root)0(x+)(x-) (y+)(y-)(z+)(z-)
//     u_char out_dir_; // 00(x+)(x-) (y+)(y-)(z+)(z-)
//     float dist_;
// };

//For data maintaining
struct LR_node{
    LR_node(){
        topo_sch_ = NULL;
        free_num_ = 0;
        parent_dir_ = 0;
        children_dir_ = 0;
        root_id_ = 0;
        path_g_ = 999999.0;
    }
    shared_ptr<sch_node> topo_sch_; //for graph searching, multi-threads are not allowed
    float path_g_;
    uint32_t root_id_; // hnode idx
    uint8_t parent_dir_; // 00000000(parent 0~27=3*3*3, 0: root or not covered), if 0, not searched
    uint32_t children_dir_; // 00000000 00000000 00000000 00000000 (each flag represents the corresponding position exists)
    uint16_t free_num_, unknown_num_;
    uint8_t flags_;  //0000 ()()(local search origin)()
};

struct LR_block{
    LR_block(){
        local_grid_.clear();
        show_flag_ = false;
    };
    ~LR_block(){};
    vector<shared_ptr<LR_node>> local_grid_; 
    Eigen::Vector3i origin_;    
    bool show_flag_;     
};

class LowResMap{
typedef tr1::unordered_map<int, pair<int, Eigen::Vector3i>> target_dict;
typedef priority_queue<shared_ptr<sch_node>, vector<shared_ptr<sch_node>>, DCompare> prio_D;
typedef priority_queue<shared_ptr<sch_node>, vector<shared_ptr<sch_node>>, ACompare> prio_A;
public:
    LowResMap(){};
    // void init(const ros::NodeHandle &nh, 
    //         const ros::NodeHandle &nh_private);

    void AlignInit(const ros::NodeHandle &nh, 
        const ros::NodeHandle &nh_private,
        Eigen::Vector3d &origin, Eigen::Vector3i &block_size, 
        Eigen::Vector3i &block_num, Eigen::Vector3i &local_block_num);
    void SetMap(BlockMapLite *map){map_ = map;};
    // void SetColorManager(ColorManager *CM){CM_ = CM;};
    inline Eigen::Vector3d GetStdPos(const Eigen::Vector3d &pos);
    inline bool InsideMap(const Eigen::VectorXd &pos);//
    inline bool InsideMap(const Eigen::Vector3i &pos);//
    inline bool InsideMap(const Eigen::Vector3d &pos);//
    inline bool BlockInsideMap(const Eigen::Vector3i &pos);//
    inline bool BlockInsideLocalMap(const Eigen::Vector3i &pos);//
    inline bool InsideLocalMap(const Eigen::VectorXd &pos);//
    inline bool InsideLocalMap(const Eigen::Vector3i &pos);//
    inline bool InsideLocalMap(const Eigen::Vector3d &pos);//

    // /**
    //  * @brief Set the Eternal Node 
    //  * 
    //  * @param pos robot pos
    //  */
    // void SetEternalNode(const Eigen::Vector3d &pos);

    /**
     * @brief Broute force check, not fast. Call before planning. Expand lowresmap, erase occupied nodes in a bounding box
     * 
     * @param bbx 
     */
    void UpdateLocalBBX(const pair<Eigen::Vector3d, Eigen::Vector3d> bbx);

    // /**
    //  * @brief Call before planning. Expand lowresmap through Djkstra algorithm, erase occupied nodes in a bounding box
    //  * 
    //  * @param rob_pose robot state
    //  * @param u2f_list free voxels that are changed from unknown voxels
    //  * @param u2o_list occupied voxels that are changed from unknown voxels
    //  * @param o2f_list free voxels that are changed from occupied voxels
    //  * @param f2o_list occupied voxels that are changed from free voxels
    //  * @param search_origins <bid, nid>, dijkstra search origins,for later voronoi partition 
    //  * @param vp_margins    <hid, nid>, dijkstra search origins,for later voronoi partition 
    //  * @return true 
    //  * @return false 
    //  */
    // bool UpdateLRM(const Eigen::Matrix4d &rob_pose, const vector<Eigen::Vector3d> &u2f_list, const vector<Eigen::Vector3d> &u2o_list, 
    //                 const vector<Eigen::Vector3d> &o2f_list, const vector<Eigen::Vector3d> &f2o_list, vector<pair<int, int>> &search_origins, 
    //                 vector<list<pair<uint32_t, Eigen::Vector3d>>> &vp_margins);


    /* for fast DTG update; not completed */
    /**
     * @brief reload local map, refresh h margins, clear out hn
     * 
     * @param r_pos                     robot pos
     * @param candidate_search_origins  search_origins for incremental Dijkstra
     * @param h_margins                 Voronoi partition margins
     * @param hns                       Voronoi partition nodes, i.e. history nodes
     */
    void ReloadNodes(const Eigen::Vector3d &r_pos, vector<pair<int, int>> &candidate_search_origins, list<list<pair<uint32_t, Eigen::Vector3d>>> &h_margins, 
                    list<pair<uint32_t, Eigen::Vector3d>> &hns);
    inline void PostoId3(const Eigen::Vector3d &pos, Eigen::Vector3i &id3);              //dont check
    // set node states, only set the nodes containing changed voxels
    void SetNodeStates(const vector<Eigen::Vector3d> &u2f_list, const vector<Eigen::Vector3d> &u2o_list, 
        const vector<Eigen::Vector3d> &o2f_list, const vector<Eigen::Vector3d> &f2o_list, vector<pair<int, int>> &candidate_search_origins); 

    
    /* broute force DTG update */
    void ReloadNodesBf(const Eigen::Vector3d &r_pos);
    void SetNodeStatesBf(vector<Eigen::Vector3d> &new_free_nodes); 

    /**
     * @brief Get the Path, A*
     * 
     * @param start 
     * @param end 
     * @param path      end-->start
     * @param dist      distance
     * @return true     Path is found
     * @return false 
     */
    bool GetPath(const Eigen::Vector3d &start, const Eigen::Vector3d &end, vector<Eigen::Vector3d> &path, double &dist);

    /**
     * @brief Get the Path, A*
     * 
     * @param start 
     * @param end 
     * @param path      start-->end
     * @param local     search in local map
     * @return true     Path is found
     * @return false 
     */
    bool GetPath(const Eigen::Vector3d &start, const Eigen::Vector3d &end, list<Eigen::Vector3d> &path, bool local = false, int search_num = 500);

    // /**
    //  * @brief Force clear corresponding nodes
    //  * 
    //  * @param occ_list newly detected occupied pts 
    //  */
    // void ClearInfeasible(vector<Eigen::Vector3d> &occ_list);

    // /**
    //  * @brief Force clear corresponding nodes, and break paths
    //  * 
    //  * @param occ_list 
    //  */
    // void ClearInfeasibleTopo(vector<Eigen::Vector3d> &occ_list);

    // /**
    //  * @brief call after ClearInfeasible()
    //  * 
    //  */
    // void PruneBlock();  

    // /**
    //  * @brief Clear expired Xnode. Clear dead blocks. Update topological relationships. 
    //  * 
    //  */
    // void PruneTopoBlock();  

    /**
     * @brief check if the path is feasible
     * 
     * @param path to be checked
     * @param allow_uknown if true, unknown space is allowed
     * @return 0: feasible, 1: unknown, 2: collide
     */
    int PathCheck(list<Eigen::Vector3d> &path, bool allow_uknown = false);

    /**
     * @brief 
     * 
     * @param path to be checked
     * @param allow_uknown if true, unknown space is allowed
     * @return 0: feasible, 1: unknown, 2: collide
     */
    int PathCheck(vector<Eigen::Vector3d> &path, bool allow_uknown = false);

    /**
     * @brief prune path and get rectangle safe corridors
     * 
     * @param path input raw path
     * @param corridors safecorridors
     * @param corridorVs vertexs of corridors
     * @param pruned_path shorten path
     * @param prune_length
     * @return 0: failed, 1: get whole corridor, 2: get a part
     */
    int FindCorridors(const vector<Eigen::Vector3d> path, 
                    vector<Eigen::MatrixX4d> &corridors, 
                    vector<Eigen::Matrix3Xd> &corridorVs,
                    vector<Eigen::Vector3d> &pruned_path,
                    bool allow_unknown = false,
                    double prune_length = INFINITY);

    inline bool IsFeasible(const Eigen::VectorXd &pos, bool allow_uknown = false);

    inline bool IsFeasible(const Eigen::Vector3d &pos, bool allow_uknown = false);

    inline bool IsFeasible(const Eigen::Vector3i &pos, bool allow_uknown = false); //dont check

    // /**
    //  * @brief get path 
    //  * 
    //  * @param start     the index of start pos 
    //  * @param target_id the id of the end hnode
    //  * @param path      start--->target_id
    //  * @param length    path length
    //  * @return true     path exist
    //  * @return false    path not exist
    //  */
    // inline bool RetrieveHPath(const int &start, const uint32_t &target_id, list<Eigen::Vector3d> &path, double &length);

    // /**
    //  * @brief get path 
    //  * 
    //  * @param start     the index of start pos 
    //  * @param target_id the id of the end hnode
    //  * @param path      start--->target_id
    //  * @param length    path length
    //  * @return true     path exist
    //  * @return false    path not exist
    //  */
    // inline bool RetrieveHPath(const Eigen::Vector3d &start, const uint32_t &target_id, list<Eigen::Vector3d> &path, double &length);

    /**
     * @brief 
     * 
     * @param start 
     * @param path 
     * @param length    not used
     * @param r         true: root -> start, false: start-> root
     * @return true 
     * @return false 
     */
    inline bool RetrieveHPath(const Eigen::Vector3i &start, vector<Eigen::Vector3d> &path, double &length, bool r = false);

    /**
     * @brief 
     * 
     * @param start 
     * @param path 
     * @param length    not used
     * @param r         true: root -> start, false: start-> root
     * @return true 
     * @return false 
     */
    inline bool RetrieveHPath(const Eigen::Vector3i &start, list<Eigen::Vector3d> &path, double &length, bool r = false);

    /**
     * @brief 
     * 
     * @param start 
     * @param path 
     * @param length    not used
     * @param r         true: root -> start, false: start-> root
     * @return true 
     * @return false 
     */
    inline bool RetrieveHPath(const Eigen::Vector3d &start, list<Eigen::Vector3d> &path, double &length, bool r = false);

    // /**
    //  * @brief get the shortest(Manhattan) path 
    //  * 
    //  * @param start     start pos 
    //  * @param target_id the id of the end hnode
    //  * @param path      start--->target_id
    //  * @param length    path length
    //  * @return true     path exist
    //  * @return false    path not exist
    //  */
    // inline bool RetrieveHPathShortest(const Eigen::Vector3d &start, uint32_t &target_id, list<Eigen::Vector3d> &path, double &length);

    // /**
    //  * @brief get the shortest Manhattan distance to root node 
    //  * 
    //  * @param start     start pos
    //  * @param target_id hnode id
    //  * @param length    length
    //  * @return true     have path
    //  * @return false    no path
    //  */
    // inline bool ShortestLength2Root(const Eigen::Vector3d &start, uint32_t &target_id, double &length);

    // /**
    //  * @brief get path of current djkstra, the end is robot_pos
    //  * 
    //  * @param start     the index of start pos 
    //  * @param path      start--->robot_pos
    //  * @param length    path length
    //  */
    // inline void RetrieveHPathTemp(const int &start, list<Eigen::Vector3d> &path, double &length);

    /**
     * @brief prune the path
     * 
     * @param path          raw path
     * @param pruned_path   shortten path
     * @param length        length of the pruned path
     * @return true         success
     * @return false        fail
     */
    bool PrunePath(const list<Eigen::Vector3d> &path, list<Eigen::Vector3d> &pruned_path, double &length);

    /**
     * @brief 
     * 
     * @param path          raw path
     * @param pruned_path   shortten path
     * @param length        length of the pruned path
     * @return true         success
     * @return false        fail
     */
    bool PrunePath(const vector<Eigen::Vector3d> &path, vector<Eigen::Vector3d> &pruned_path, double &length);

    /**
     * @brief clear all the topoloigical relationships
     * 
     */
    void ClearTopo();

    /**
     * @brief voronoi partition from search origins
     * 
     * @param search_origins voronoi centers
     * @param margins        voronoi partition margins
     */
    void VoronoiPartitionSearch(list<pair<uint32_t, Eigen::Vector3d>> &search_origins, list<Eigen::Vector3i> &margins);

    /**
     * @brief Get the Distances to targets, dijkstra
     * 
     * @param origin            search origin
     * @param tars              targets
     * @param dist              distances
     * @param paths             path to each point
     * @param clear_searched    clear searched nodes
     */
    void GetDists(const Eigen::Vector3d &origin, vector<Eigen::Vector3d> &tars, vector<double> &dist, vector<vector<Eigen::Vector3d>> &paths, bool clear_searched = true);

    /**
     * @brief retrieve paths between hnodes from voronoi margins
     * 
     * @param idx_dict  hid->index
     * @param margins   voronoi partition margins
     * @param hh_idx    hnodes idx of idx_dict
     * @param paths     corresponding paths
     * @param lengths   path lengths
     */
    void VoronoiPathRetrieve(tr1::unordered_map<uint32_t, uint32_t> &idx_dict, list<Eigen::Vector3i> &margins, 
                                list<pair<uint32_t, uint32_t>> &hh_idx, list<list<Eigen::Vector3d>> &paths, list<double> &lengths);

    /**
     * @brief check all local nodes
     * 
     * @return true 
     * @return false 
     */
    bool CheckThorough();
    

    // /**
    //  * @brief Check if the hnode is alive, if dead try to save it. For multi robots and obstacles.
    //  * 
    //  * @param pos position of the hnode
    //  * @param id  id of the hnode
    //  * @return true 
    //  * @return false 
    //  */
    // inline bool CheckAddHnode(const Eigen::Vector3d &pos, const int &id);

    /**
     * @brief check homotopy, not 114514
     * 
     * @param pos position of the hnode
     * @param id  id of the hnode
     * @return true 
     * @return false 
     */
    bool CheckPathHomo(list<Eigen::Vector3d> &path1, list<Eigen::Vector3d> &path2);


    void GetPathInLocal(const double &ye_origin, const vector<Eigen::Vector3d> &path_origin, double &ye_local, vector<Eigen::Vector3d> &path_local);
    inline double GetRootHnCost(){return cur_h_cost_;};


    inline Eigen::Vector3d GetRobotSize(){return Robot_size_;}

    inline bool StrangePoint(Eigen::Vector3d &p); 
    // call after dijkstra search
    inline bool Connected(const Eigen::Vector3d &p); 

    inline shared_ptr<LR_node> GlobalPos2LocalNode(const Eigen::Vector3i &pos);// check if pos is in the block
    inline shared_ptr<LR_node> GlobalPos2LocalNode(const Eigen::Vector3d &pos);// check if pos is in the block
    inline shared_ptr<LR_node> GlobalPos2LocalNode(const int &id);//don't check, carefully use
    inline shared_ptr<LR_node> GetNode(const Eigen::Vector3d &pos, const shared_ptr<LR_block> &FG);//don't check, pos: world
    inline bool GlobalPos2LocalBlock3Id(const Eigen::Vector3d &pos, Eigen::Vector3i &blkid);//check
    inline bool GlobalBlockPos2LocalBlock3Id(const Eigen::Vector3i &pos, Eigen::Vector3i &blkid);//check
    inline bool GlobalPos2GlobalBlock3Id(const Eigen::Vector3d &pos, Eigen::Vector3i &blkid);//check
    inline bool GlobalPos2LocalBidNid(const Eigen::Vector3i &pos, int &bid, int &nid);//check
    inline bool GlobalPos2LocalBidNid(const Eigen::Vector3d &pos, int &bid, int &nid);//check
    inline int GlobalPos2LocalBid(const Eigen::Vector3d &pos);//check, pos: world
    inline int GlobalPos2LocalBid(const Eigen::Vector3i &pos);//check, pos: world

    inline int GetBlkNid(const Eigen::Vector3d &pos, const shared_ptr<LR_block> &FG);//don't check, pos: world
    inline int GetBlkNid(const Eigen::Vector3i &pos, const shared_ptr<LR_block> &FG);//don't check, pos: world

    inline Eigen::Vector3d GlobalId2GlobalPos(int id); // global id
    inline Eigen::Vector3d GlobalId2GlobalPos(const Eigen::Vector3i &id);//  global id
    inline Eigen::Vector3i GlobalPos2GlobalId3(const Eigen::Vector3d &pos);//  global id
    inline int GlobalBid32GlobalBid(const Eigen::Vector3i &pos);//  global pos
    inline int GlobalBid32GlobalBid(const Eigen::Vector3d &pos);//  global pos
    inline int GlobalBid2LocalBid(const int &bid);//  global pos
    inline Eigen::Vector3i BlockNodeId2GlobalNodeId(const shared_ptr<LR_block> &FG, const int &nid);
    inline int GlobalPos2GlobalNodeId(const Eigen::Vector3d &pos);
    inline int GlobalPos2GlobalNodeId(const Eigen::Vector3i &pos);
    Eigen::Vector3i block_size_, voxel_num_, block_num_;       //block_size: size of voxels in a block, block_num_:total number of blocks
    Eigen::Vector3d origin_, mapscale_, blockscale_, node_scale_, map_upbd_, map_lowbd_;
    Eigen::Vector3d local_origin_, local_mapscale_, local_map_upbd_, local_map_lowbd_;
    Eigen::Vector3i local_block_num_, local_origin_idx_, local_up_idx_; // block idx
    Eigen::Vector3i local_origin_node_idx_, local_up_node_idx_, local_node_num_;
    double cur_h_cost_;
    shared_ptr<LR_node> /*Xnode_,*/ Outnode_; 

private:
    void ShowGridLocal(const ros::TimerEvent& e);
    void InitializeQuickCompute();
    // change indx, add new local nodes, delete old local nodes

    void GetEdgeNodes(vector<Eigen::Vector3i> &edge_in, vector<Eigen::Vector3i> &edge_out, Eigen::Vector3i &move_diff);

    inline int LocalBid2GlobalBid(const int &bid);//check, pos: local block id. carefully use 
    inline Eigen::Vector3i LocalBid2LocalPos3i(const int &bid);//check, pos: local block id. carefully use 
    inline int LocalPos2LocalBid(const Eigen::Vector3i &pos);//check, pos: local block id. carefully use 
    inline void CheckNode(const Eigen::Vector3i pos);                                    // broute force check, pos: global pos
    inline void StopageDebug(string c);
    inline void InitializeBlock(const int &lid, const int &bid);
    inline uint8_t PathDir2PathId(const vector<Eigen::Vector3i> &dirs); // {d1, d2, d3} ==> 1-216
    inline void GetChildrenDirs(const uint32_t &cdirs, vector<Eigen::Vector3i> &dirs);
    bool ExpandPath(Eigen::Vector3i &cor_start, Eigen::Vector3i &cor_end, const Eigen::Vector3d &pos, bool allow_unknown);
    void CoarseExpand(Eigen::Vector3i &coridx_start, Eigen::Vector3i &coridx_end, bool allow_unknown);
    void FineExpand(Eigen::Vector3i &coridx_start, Eigen::Vector3i &coridx_end, Eigen::Vector3d &up_corner, Eigen::Vector3d &down_corner, bool allow_unknown);
    void ClearSearched(vector<shared_ptr<sch_node>> &node_list, bool clear_g = false);

    ros::NodeHandle nh_, nh_private_;
    ros::Publisher marker_pub_;
    ros::Timer show_timer_;

    bool debug_, showmap_;
    BlockMapLite *map_;
    Eigen::Vector3d expand_r_, Robot_size_;
    Eigen::Vector3i v_n_, b_n_, l_b_n_;//for quick compute
    Eigen::Vector3i node_size_;                                //expand, for collision check;
    int nv_num_;
    double resolution_;
    mutex mtx_;
    list<int> Showblocklist_;

    Eigen::Vector3d Robot_pos_;
    Eigen::Vector3i corridor_exp_r_;
    double seg_length_, prune_seg_length_;

    // list<pair<Eigen::Vector3i, lr_root>> idx_path_clear_;
    list<uint32_t> h_id_clear_;                                           //for DTG edge checking
    vector<shared_ptr<LR_block>> LG_;


    // for quick compute
    vector<Eigen::Vector3i> connect_diff_;  // [iter], size = 27, 26 connection, children dirs(int32) -> diffs
    vector<bool> connection_check_; // [dict], size = 27*27, check if a node is occlude, 
    // will it occlude diagnal connected neighbours, rid x pid ==> occlude ? free
    vector<uint32_t> occdiff_2_ch_dir_; // [dict], size = 27, neighbour diff id ==> banned dirs

    // vector<pair<Eigen::Vector3i, uint8_t>> occlude_check_list_; // [iter], size = 18, <diff, id0 = 0-17>


    // vector<vector<Eigen::Vector3i>> path_dirs_; // [dict], size = 217 = 6 * 6 * 6 + 1(0), 26 connection path, parent_dir(0~216) ==> path dir

    // vector<Eigen::Vector3i> pdir_2_pdiff_; // [dict], size = 217, parent_dir(0~216) ==> parent diff(26 connection)
    // vector<uint8_t> pdir_2_chcn_; // [dict], size = 217, parent_dir(0~216) ==> children connection id(0~25, 26 connection, in parent node)
};

inline bool LowResMap::IsFeasible(const Eigen::VectorXd &pos, bool allow_uknown){
    Eigen::Vector3d p(pos(0), pos(1), pos(2));
    int blockid = GlobalPos2LocalBid(p);
    if(blockid != -1){
        shared_ptr<LR_block> fp_ptr = LG_[blockid];
        if(fp_ptr != NULL){
            shared_ptr<LR_node> node = GetNode(p, fp_ptr);
            if(node == Outnode_) return false;
            if(node != NULL && node->free_num_ == nv_num_) return true;
            else if(allow_uknown && node != NULL && node->unknown_num_ + node->free_num_ == nv_num_) return true;
            else return false;
        }
        else{
            return false;
        }
    }
    else{
        return false;
    }
}

inline bool LowResMap::IsFeasible(const Eigen::Vector3d &pos, bool allow_uknown){
    int blockid = GlobalPos2LocalBid(pos);
    if(blockid != -1){
        shared_ptr<LR_block> fp_ptr = LG_[blockid];
        if(fp_ptr != NULL){
            shared_ptr<LR_node> node = GetNode(pos, fp_ptr);
            if(node == Outnode_) return false;
            // if(allow_uknown){
            //     cout<<"pos:"<<pos.transpose()<<endl;
            //     cout<<(node != NULL)<<endl;
            //     if(node != NULL){
            //         cout<<"node->free_num_:"<<int(node->free_num_)<<endl;
            //         cout<<"node->unknown_num_:"<<int(node->unknown_num_)<<endl;
            //         cout<<"node->unknown_num_ + node->free_num_ == nv_num_:"<<int(node->unknown_num_ + node->free_num_ == nv_num_)<<endl;
            //     }
            // }
            if(node != NULL && node->free_num_ == nv_num_) return true;
            else if(allow_uknown && node != NULL && node->unknown_num_ + node->free_num_ == nv_num_) return true;
            else return false;
        }
        else{
            if(allow_uknown) return true;
            return false;
        }
    }
    else{
        return false;
    }
}

inline bool LowResMap::IsFeasible(const Eigen::Vector3i &pos, bool allow_uknown){
    Eigen::Vector3d p = GlobalId2GlobalPos(pos);
    int blockid = GlobalPos2LocalBid(p);
    if(blockid < 0 || blockid >= LG_.size()){
        return false;
    }
    shared_ptr<LR_block> fp_ptr = LG_[blockid];
    if(fp_ptr != NULL){
        // Eigen::Vector3d p = IdtoPos(pos);
        shared_ptr<LR_node> node = GetNode(p, fp_ptr);
        if(node == Outnode_) return false;
        if(node != NULL && node->free_num_ == nv_num_) return true;
        else if(allow_uknown && node != NULL && node->unknown_num_ + node->free_num_ == nv_num_) return true;
        else return false;
    }
    else{
        return false;
    }
}

inline bool LowResMap::RetrieveHPath(const Eigen::Vector3i &start, vector<Eigen::Vector3d> &path, double &length, bool r){
    Eigen::Vector3d p;
    Eigen::Vector3i c, par;
    c = start;
    auto n = GlobalPos2LocalNode(start);
    shared_ptr<LR_node> n2; //debug
    if(n == Outnode_) return false;
    path.clear();
    path.emplace_back(GlobalId2GlobalPos(start));
    // cout<<"path:"<<GlobalId2GlobalPos(start).transpose()<<endl;
    length = n->path_g_;
    while(n->parent_dir_ != 13){
        c = c + connect_diff_[n->parent_dir_];

        n = GlobalPos2LocalNode(c);
        // if(n == Outnode_){
        //     StopageDebug("RetrieveHPath out!");
        // }
        // cout<<"path:"<<GlobalId2GlobalPos(c).transpose()<<endl;
        path.emplace_back(GlobalId2GlobalPos(c));
    }
    if(r){
        reverse(path.begin(), path.end());
    }
    return true;
}


inline bool LowResMap::RetrieveHPath(const Eigen::Vector3i &start, list<Eigen::Vector3d> &path, double &length, bool r){
    Eigen::Vector3d p;
    Eigen::Vector3i c, par;
    c = start;
    auto n = GlobalPos2LocalNode(start);
    shared_ptr<LR_node> n2; //debug
    if(n == Outnode_) return false;
    path.clear();
    path.emplace_back(GlobalId2GlobalPos(start));
    while(n->parent_dir_ != 13){
        c = c + connect_diff_[n->parent_dir_];
        // cout<<"connect_diff_[n->parent_dir_]:"<<connect_diff_[n->parent_dir_].transpose()<<endl;
        // cout<<"root_id:"<<int(n->root_id_)<<endl;
        // cout<<"path_g_:"<<n->path_g_<<endl;
        n = GlobalPos2LocalNode(c);
        // if((connect_diff_[n->parent_dir_] + connect_diff_[n2->parent_dir_]).norm() == 0){
        //     cout<<"n->parent_dir_:"<<int(n->parent_dir_)<<endl;
        //     cout<<"n2->parent_dir_:"<<int(n2->parent_dir_)<<endl;
        //     cout<<"connect_diff_[n->parent_dir_]:"<<connect_diff_[n->parent_dir_].transpose()<<endl;
        //     cout<<"connect_diff_[n2->parent_dir_]:"<<connect_diff_[n2->parent_dir_].transpose()<<endl;

        //     StopageDebug("RetrieveHPath manba out out out");
        // }
        // if(n2 == Outnode_) {
        //     cout<<"n->parent_dir_:"<<int(n->parent_dir_)<<endl;
        //     cout<<"n->path_g_:"<<(n->path_g_)<<endl;

        //     StopageDebug("RetrieveHPath manba out");
        // }
        // n = n2;
        path.emplace_back(GlobalId2GlobalPos(c));
    }
    if(r){
        reverse(path.begin(), path.end());
    }
    return true;
}

inline bool LowResMap::RetrieveHPath(const Eigen::Vector3d &start, list<Eigen::Vector3d> &path, double &length, bool r){
    Eigen::Vector3d p;
    Eigen::Vector3i c, par;
    // if(InsideLocalMap(start)) return false;
    PostoId3(start, c);
    auto n = GlobalPos2LocalNode(c);
    if(n == Outnode_) return false;
    path.clear();
    if(r) path.emplace_front(start);
    else path.emplace_back(start);
    while(n->parent_dir_ != 13){
        c = c + connect_diff_[n->parent_dir_];
        n = GlobalPos2LocalNode(c);
        // if(n == Outnode_) StopageDebug("RetrieveHPath manba out");
        // path.emplace_back(GlobalId2GlobalPos(c));
        if(r) path.emplace_front(GlobalId2GlobalPos(c));
        else path.emplace_back(GlobalId2GlobalPos(c));
    }
    // if(!r){
    //     reverse(path.begin(), path.end());
    // }
    return true;
}

inline shared_ptr<LR_node> LowResMap::GlobalPos2LocalNode(const Eigen::Vector3i &pos){
    int bid, nid;
    if(GlobalPos2LocalBidNid(pos, bid, nid)){
        return LG_[bid]->local_grid_[nid];
    }
    else{
        return Outnode_;
    }
}


inline shared_ptr<LR_node> LowResMap::GlobalPos2LocalNode(const Eigen::Vector3d &pos){
    int blockid = GlobalPos2LocalBid(pos);
    if(blockid != -1){
        shared_ptr<LR_block> fp_ptr = LG_[blockid];
        if(fp_ptr != NULL){
            return GetNode(pos, fp_ptr);
        }
        else{
            return NULL;
        }
    }
    else{
        return Outnode_;
    }
}

inline shared_ptr<LR_node> LowResMap::GlobalPos2LocalNode(const int &id){
    Eigen::Vector3d pos = GlobalId2GlobalPos(id);
    int blockid = GlobalPos2LocalBid(pos);
    if(blockid == -1) {
        return Outnode_;
    }
    else if(LG_[blockid] == NULL){
        return NULL;
    }
    return LG_[blockid]->local_grid_[GetBlkNid(pos, LG_[blockid])];
}

inline shared_ptr<LR_node> LowResMap::GetNode(const Eigen::Vector3d &pos, const shared_ptr<LR_block> &FG){
    return FG->local_grid_[GetBlkNid(pos, FG)];
}

inline Eigen::Vector3d LowResMap::GetStdPos(const Eigen::Vector3d &pos){
    Eigen::Vector3i std_pos;
    PostoId3(pos, std_pos);
    return GlobalId2GlobalPos(std_pos);
}

inline bool LowResMap::InsideMap(const Eigen::VectorXd &pos){
    if(pos(0) < map_lowbd_(0) || pos(1) < map_lowbd_(1) || pos(2) < map_lowbd_(2) ||
        pos(0) >  map_upbd_(0) || pos(1) > map_upbd_(1) || pos(2) > map_upbd_(2) )
        return false;
    return true;
}

inline bool LowResMap::InsideMap(const Eigen::Vector3d &pos){
    if(pos(0) < map_lowbd_(0) || pos(1) < map_lowbd_(1) || pos(2) < map_lowbd_(2) ||
        pos(0) >  map_upbd_(0) || pos(1) > map_upbd_(1) || pos(2) > map_upbd_(2) )
        return false;
    return true;
}

inline bool LowResMap::InsideMap(const Eigen::Vector3i &pos){
    if(pos(0) < 0 || pos(1) < 0 || pos(2) < 0 ||
        pos(0) >=  voxel_num_(0) || pos(1) >= voxel_num_(1) || pos(2) >= voxel_num_(2))
        return false;
    return true;
}

inline bool LowResMap::BlockInsideMap(const Eigen::Vector3i &pos){
    if(pos(0) < 0 || pos(1) < 0 || pos(2) < 0 ||
        pos(0) >=  block_num_(0) || pos(1) >= block_num_(1) || pos(2) >= block_num_(2))
        return false;
    return true;
}

inline bool LowResMap::BlockInsideLocalMap(const Eigen::Vector3i &pos){
    if(pos(0) < local_origin_idx_(0) || pos(1) < local_origin_idx_(1) || pos(2) < local_origin_idx_(2) ||
        pos(0) >  local_up_idx_(0) || pos(1) > local_up_idx_(1) || pos(2) > local_up_idx_(2))
        return false;
    return true;
}

inline bool LowResMap::InsideLocalMap(const Eigen::VectorXd &pos){
    if(pos(0) < local_map_lowbd_(0) || pos(1) < local_map_lowbd_(1) || pos(2) < local_map_lowbd_(2) ||
        pos(0) >  local_map_upbd_(0) || pos(1) > local_map_upbd_(1) || pos(2) > local_map_upbd_(2) )
        return false;
    return true;
}

inline bool LowResMap::InsideLocalMap(const Eigen::Vector3i &pos){
    if(pos(0) < local_origin_node_idx_(0) || pos(1) < local_origin_node_idx_(1) || pos(2) < local_origin_node_idx_(2) ||
        pos(0) >  local_up_node_idx_(0) || pos(1) > local_up_node_idx_(1) || pos(2) > local_up_node_idx_(2))
        return false;
    return true;
}

inline bool LowResMap::InsideLocalMap(const Eigen::Vector3d &pos){
    if(pos(0) < local_map_lowbd_(0) || pos(1) < local_map_lowbd_(1) || pos(2) < local_map_lowbd_(2) ||
        pos(0) >  local_map_upbd_(0) || pos(1) > local_map_upbd_(1) || pos(2) > local_map_upbd_(2) )
        return false;
    return true;
}

inline bool LowResMap::StrangePoint(Eigen::Vector3d &p){
    Eigen::Vector3d dp = p - origin_;
    for(int dim = 0; dim < 3; dim++){
        if(abs(dp(dim) / node_scale_(dim) - floor(dp(dim) / node_scale_(dim)) - 0.5) > 0.5 - 1e-4){
            return true;
        }
    }
    return false;
}

inline bool LowResMap::Connected(const Eigen::Vector3d &p){
    if(InsideLocalMap(p)){
        int bid, nid;
        if(!GlobalPos2LocalBidNid(p, bid, nid)){
            return false;
        }
        return (LG_[bid]->local_grid_[nid]->topo_sch_ != NULL);
    }
    return false;
}

inline bool LowResMap::GlobalPos2LocalBlock3Id(const Eigen::Vector3d &pos, Eigen::Vector3i &blkid){
    if(InsideLocalMap(pos)){
        Eigen::Vector3d dpos = pos - local_origin_;
        blkid.x() = floor(dpos.x() / blockscale_.x());
        blkid.y() = floor(dpos.y() / blockscale_.y());
        blkid.z() = floor(dpos.z() / blockscale_.z());
        return true;
    }
    else{
        return false;
    }
}

inline bool LowResMap::GlobalBlockPos2LocalBlock3Id(const Eigen::Vector3i &pos, Eigen::Vector3i &blkid){
    if(BlockInsideLocalMap(pos)){
        blkid= pos - local_origin_idx_;
        blkid(0) = blkid(0) / local_block_num_(0);
        blkid(1) = blkid(1) / local_block_num_(1);
        blkid(2) = blkid(2) / local_block_num_(2);
        return true;
    }
    else{
        return false;
    }
}

inline bool LowResMap::GlobalPos2GlobalBlock3Id(const Eigen::Vector3d &pos, Eigen::Vector3i &blkid){
    if(InsideMap(pos)){
        Eigen::Vector3d dpos = pos - origin_;
        blkid.x() = floor(dpos.x() / blockscale_.x());
        blkid.y() = floor(dpos.y() / blockscale_.y());
        blkid.z() = floor(dpos.z() / blockscale_.z());
        return true;
    }
    else{
        return false;
    }
}

inline bool LowResMap::GlobalPos2LocalBidNid(const Eigen::Vector3i &pos, int &bid, int &nid){
    if(InsideLocalMap(pos)){
        int x = (pos(0) - local_origin_node_idx_(0)) /  block_size_(0);
        int y = (pos(1) - local_origin_node_idx_(1)) /  block_size_(1);
        int z = (pos(2) - local_origin_node_idx_(2)) /  block_size_(2);
        bid = x + l_b_n_(0) * y + l_b_n_(1) * z;
        x = pos(0) - LG_[bid]->origin_(0);
        y = pos(1) - LG_[bid]->origin_(1);
        z = pos(2) - LG_[bid]->origin_(2);
        nid = x + y * block_size_(0) + z * block_size_(1) * block_size_(0);
        return true;
    }
    else return false;
}

inline bool LowResMap::GlobalPos2LocalBidNid(const Eigen::Vector3d &pos, int &bid, int &nid){
    if(InsideLocalMap(pos)){
        int x = (pos(0) - local_origin_(0)) /  blockscale_(0);
        int y = (pos(1) - local_origin_(1)) /  blockscale_(1);
        int z = (pos(2) - local_origin_(2)) /  blockscale_(2);
        bid = x + l_b_n_(0) * y + l_b_n_(1) * z;
        x = (pos(0) - origin_(0)) / node_scale_(0);
        y = (pos(1) - origin_(1)) / node_scale_(1);
        z = (pos(2) - origin_(2)) / node_scale_(2);
        x = x - LG_[bid]->origin_(0);
        y = y - LG_[bid]->origin_(1);
        z = z - LG_[bid]->origin_(2);
        nid = x + y * block_size_(0) + z * block_size_(1) * block_size_(0);
        return true;
    }
    else return false;
}

inline int LowResMap::GlobalPos2LocalBid(const Eigen::Vector3d &pos){
    if(InsideLocalMap(pos)){
        Eigen::Vector3d dpos = pos - local_origin_;
        Eigen::Vector3i posid;
        posid.x() = floor(dpos.x() / blockscale_.x());
        posid.y() = floor(dpos.y() / blockscale_.y());
        posid.z() = floor(dpos.z() / blockscale_.z());
        return posid(2)*l_b_n_(1) + posid(1)*l_b_n_(0) + posid(0);
    }
    else{
        return -1;
    }
}

inline int LowResMap::GlobalPos2LocalBid(const Eigen::Vector3i &pos){
    if(BlockInsideLocalMap(pos)){
        Eigen::Vector3i dpos = pos - local_origin_idx_;
        return dpos(2)*l_b_n_(1) + dpos(1)*l_b_n_(0) + dpos(0);
    }
    else{
        return -1;
    }
}

inline int LowResMap::LocalBid2GlobalBid(const int &bid){
    if(bid < 0 || bid >= l_b_n_(2)){
        return -1;
    }
    int x = bid % local_block_num_(0);
    int y = ((bid - x)/local_block_num_(0)) % local_block_num_(1);
    int z = ((bid - x) - y*local_block_num_(0))/l_b_n_(1);
    Eigen::Vector3i p(x, y, z);
    p = p + local_origin_idx_;
    if(!BlockInsideLocalMap(p)) return -1;
    return p(0) + p(1) * block_num_(0) + p(2) * block_num_(0) * block_num_(1);
}

inline Eigen::Vector3i LowResMap::LocalBid2LocalPos3i(const int &bid){
    int x = bid % local_block_num_(0);
    int y = ((bid - x)/local_block_num_(0)) % local_block_num_(1);
    int z = ((bid - x) - y*local_block_num_(0))/l_b_n_(1);
    Eigen::Vector3i p(x, y, z);
    return p;
}

inline int LowResMap::LocalPos2LocalBid(const Eigen::Vector3i &pos){

    if(pos(0) < 0 || pos(1) < 0 || pos(2) < 0 ||
        pos(0) >  local_up_idx_(0) || pos(1) > local_up_idx_(1) || pos(2) > local_up_idx_(2)){
            return -1;
        }
    else{
        Eigen::Vector3i blkid;
        blkid(0) = pos(0) / local_block_num_(0);
        blkid(1) = pos(1) / local_block_num_(1);
        blkid(2) = pos(2) / local_block_num_(2);
        return pos(2)*l_b_n_(1) + pos(1)*l_b_n_(0) + pos(0);
    }
}

inline int LowResMap::GetBlkNid(const Eigen::Vector3d &pos, const shared_ptr<LR_block> &FG){
    Eigen::Vector3d dpos = pos - origin_ - FG->origin_.cast<double>().cwiseProduct(node_scale_);
    Eigen::Vector3i posid;
    posid.x() = floor(dpos(0) / node_scale_(0));
    posid.y() = floor(dpos(1) / node_scale_(1));
    posid.z() = floor(dpos(2) / node_scale_(2));

    return posid(2)*block_size_(0)*block_size_(1) + posid(1)*block_size_(0) + posid(0);
}

inline int LowResMap::GetBlkNid(const Eigen::Vector3i &pos, const shared_ptr<LR_block> &FG){
    Eigen::Vector3i dpos = pos - FG->origin_;
    return dpos(2)*block_size_(0)*block_size_(1) + dpos(1)*block_size_(0) + dpos(0);
}


inline Eigen::Vector3d LowResMap::GlobalId2GlobalPos(int id){
    int x = id % voxel_num_(0);
    int y = ((id - x)/voxel_num_(0)) % voxel_num_(1);
    int z = ((id - x) - y*voxel_num_(0))/v_n_(1);
    return Eigen::Vector3d((double(x)+0.5)*node_scale_(0),(double(y)+0.5)*node_scale_(1),(double(z)+0.5)*node_scale_(2))+origin_;
}   

inline Eigen::Vector3d LowResMap::GlobalId2GlobalPos(const Eigen::Vector3i &id){
    Eigen::Vector3d p((id(0)+0.5)*node_scale_(0),(id(1)+0.5)*node_scale_(1),(id(2)+0.5)*node_scale_(2));
    p += origin_;
    return p;
}

inline Eigen::Vector3i LowResMap::GlobalPos2GlobalId3(const Eigen::Vector3d &pos){
    Eigen::Vector3d dpos = pos - origin_;
    Eigen::Vector3i posid;
    posid.x() = floor(dpos.x() / node_scale_.x());
    posid.y() = floor(dpos.y() / node_scale_.y());
    posid.z() = floor(dpos.z() / node_scale_.z());
    return posid;
}

inline int LowResMap::GlobalBid32GlobalBid(const Eigen::Vector3i &pos){
    if(!BlockInsideMap(pos)) return -1;
    return pos(0) + pos(1) * b_n_(0) + pos(2) * b_n_(1);
}

inline int LowResMap::GlobalBid32GlobalBid(const Eigen::Vector3d &pos){
    if(InsideMap(pos)){
        Eigen::Vector3d dpos = pos - origin_;
        Eigen::Vector3i posid;
        posid.x() = floor(dpos.x() / blockscale_.x());
        posid.y() = floor(dpos.y() / blockscale_.y());
        posid.z() = floor(dpos.z() / blockscale_.z());
        return posid(2)*b_n_(1) + posid(1)*b_n_(0) + posid(0);
    }
    else{
        return -1;
    }
}

inline int LowResMap::GlobalBid2LocalBid(const int &bid){
    Eigen::Vector3i p;
    p(0) = bid % block_num_(0);
    p(1) = ((bid - p(0))/block_num_(0)) % block_num_(1);
    p(2) = ((bid - p(0)) - p(1)*block_num_(0))/b_n_(1);
    if(!BlockInsideLocalMap(p)) return -1;
    p = p - local_origin_idx_;
    return p(2)*l_b_n_(1) + p(1)*l_b_n_(0) + p(0);
}

inline Eigen::Vector3i LowResMap::BlockNodeId2GlobalNodeId(const shared_ptr<LR_block> &FG, const int &nid){
    Eigen::Vector3i p;
    p(0) = nid % block_size_(0);
    p(1) = ((nid - p(0))/block_size_(0)) % block_size_(1);
    p(2) = ((nid -p(0)) - p(1)*block_size_(0))/(block_size_(1)*block_size_(0));
    p += FG->origin_;
    return p;
}

inline int LowResMap::GlobalPos2GlobalNodeId(const Eigen::Vector3d &pos){
    if(InsideMap(pos)){
        Eigen::Vector3d dpos = pos - origin_;
        Eigen::Vector3i posid;
        posid.x() = floor(dpos.x() / node_scale_.x());
        posid.y() = floor(dpos.y() / node_scale_.y());
        posid.z() = floor(dpos.z() / node_scale_.z());
        return posid(2)*v_n_(1) + posid(1)*v_n_(0) + posid(0);
    }
    else{
        return -1;
    }
}

inline int LowResMap::GlobalPos2GlobalNodeId(const Eigen::Vector3i &pos){
    if(InsideMap(pos)){
        return pos(2)*v_n_(1) + pos(1)*v_n_(0) + pos(0);
    }
    else{
        return -1;
    }
}

inline void LowResMap::PostoId3(const Eigen::Vector3d &pos, Eigen::Vector3i &id3){
    id3(0) = floor((pos(0)-origin_(0)) / node_scale_(0)); 
    id3(1) = floor((pos(1)-origin_(1)) / node_scale_(1)); 
    id3(2) = floor((pos(2)-origin_(2)) / node_scale_(2)); 
}

inline void LowResMap::CheckNode(const Eigen::Vector3i pos){

    int blockid = GlobalPos2LocalBid(pos);
    if(blockid == -1) return;
    int nodeid = GetBlkNid(pos, LG_[blockid]);

    Eigen::Vector3d startpos = pos.cast<double>().cwiseProduct(node_scale_) + origin_;
    Eigen::Vector3d chk_pos;
    startpos(0) += resolution_/2;
    startpos(1) += resolution_/2;
    startpos(2) += resolution_/2;
    int f = 0, u = 0;
    for(int x = 0; x < node_size_(0); x++)
        for(int y = 0; y < node_size_(1); y++)
            for(int z = 0; z < node_size_(2); z++){
        // volumetric_mapping::WorldBase::CellStatus cstatus = Octomap_->getCellStatusPoint(startpos+Eigen::Vector3d(x,y,z)*resolution_);
        chk_pos = startpos+Eigen::Vector3d(x,y,z)*resolution_;
        VoxelState cstatus = map_->GetVoxState(chk_pos);
        if(cstatus == VoxelState::free) f++;
        else if(cstatus == VoxelState::unknown) u++;
    }
    LG_[blockid]->local_grid_[nodeid]->free_num_ = f;
    LG_[blockid]->local_grid_[nodeid]->unknown_num_ = u;            
}


inline void LowResMap::StopageDebug(string c){
    std::cout << "\033[0;31m "<<c<<" \033[0m" << std::endl;    
    ros::shutdown();
    getchar();
}

inline void LowResMap::InitializeBlock(const int &lid, const int &bid){
    LG_[lid] = make_shared<LR_block>();
    LG_[lid]->local_grid_.resize(block_size_(0)*block_size_(1)*block_size_(2));
    int c = 0;
    // GlobalId2GlobalPos()
    Eigen::Vector3i bidx3;
    bidx3(0) = bid % block_num_(0);
    bidx3(1) = ((bid - bidx3(0))/block_num_(0)) % block_num_(1);
    bidx3(2) = ((bid - bidx3(0)) - bidx3(1)*block_num_(0))/b_n_(1);
    LG_[lid]->origin_ = bidx3.cwiseProduct(block_size_);    

    Eigen::Vector3i it;
    Eigen::Vector3d p, offset(-1e-3, -1e-3, -1e-3), startpos, chk_pos;
    VoxelState cstatus;
    for(int x = 0; x < block_size_(0); x++)
        for(int y = 0; y < block_size_(1); y++)
            for(int z = 0; z < block_size_(2); z++){
        it = LG_[lid]->origin_ + Eigen::Vector3i(x, y, z);
        p = it.cast<double>().cwiseProduct(node_scale_) + origin_ + offset + node_scale_;
        if(!map_->InsideMap(p)){
            LG_[lid]->local_grid_[c] = Outnode_;
        }
        LG_[lid]->local_grid_[c] = make_shared<LR_node>();

        // startpos = it.cast<double>().cwiseProduct(node_scale_) + origin_;
        // startpos(0) += resolution_/2;
        // startpos(1) += resolution_/2;
        // startpos(2) += resolution_/2;
        // int f = 0, u = 0;
        // for(int x = 0; x < node_size_(0); x++)
        //     for(int y = 0; y < node_size_(1); y++)
        //         for(int z = 0; z < node_size_(2); z++){
        //     chk_pos = startpos+Eigen::Vector3d(x,y,z)*resolution_;
        //     cstatus = map_->GetVoxState(chk_pos);
        //     if(cstatus == VoxelState::free) f++;
        //     else if(cstatus == VoxelState::unknown) u++;
        // }
        LG_[lid]->local_grid_[c]->free_num_ = 0;            
        LG_[lid]->local_grid_[c]->unknown_num_ = nv_num_;        
        c++;    
    }
}


inline uint8_t LowResMap::PathDir2PathId(const vector<Eigen::Vector3i> &dirs){
    int s = dirs.size();
    if(s > 3) StopageDebug("PathDir2PathId: large s");
    uint8_t id = 0;
    int g = 1;
    for(int i = 0; i < s; i++){
        for(int dim = 0; dim < 3; dim++){
            if(dirs[i](dim) == 0){
                continue;
            }
            else if(dirs[i](dim) == 1){
                id += g * (dim*2 + 1);
                break;
            }
            else if(dirs[i](dim) == -1){
                id += g * (dim*2 + 2);
                break;
            }
            else{
                StopageDebug("PathDir2PathId: impossible dir");
            }
        }
        g *= 6;
    }
    return id;
}

inline void LowResMap::GetChildrenDirs(const uint32_t &cdirs, vector<Eigen::Vector3i> &dirs){
    int n = 1;
    dirs.clear();
    for(int i = 0; i < 13; i++){
        if(cdirs & n){
            dirs.emplace_back(connect_diff_[i]);
        }
        n <<= 1;
    }
    n <<= 1;
    for(int i = 14; i < 27; i++){
        if(cdirs & n){
            dirs.emplace_back(connect_diff_[i]);
        }
        n <<= 1;
    }
}
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
                            