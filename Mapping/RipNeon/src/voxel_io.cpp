#include <RipNeon/ripneon_struct.h>
#include <RipNeon/ripneon.h>

void RipNeon::SaveData(const uint32_t &bid, shared_ptr<PV_Block> &GB){
    if(GB->PvbFlags_ & PVB_EMPTY) return; // dont save empty GB

    if(!GB->PvbFlags_ & PVB_SHOW) {
        // nothing
    }
    string p = map_path_ + "/" + to_string(bid);

    int voxel_num = block_size_(0) * block_size_(1) * block_size_(2);
    int data_size = ceil(voxel_num / 4);
    uint8_t data[data_size];
    uint8_t data_seg[4]; //00 : unknown; 01: free; 10: occ; 11: frontier;

    /* open file */
    std::ofstream outfile(p.c_str(), ios::binary);
    if(!outfile.is_open()){
        ROS_ERROR("open file failed");
        ros::shutdown();
        return;
    }
    
    float lo;
    uint64_t idx;
    for(int i = 0, j = 0; i < data_size; i++){
        for(int k = 0; k < 4; k++){
            if(j >= voxel_num){
                data_seg[k] = 0;
            }
            else{
                lo = GB->vox_[j].log_odds_;

                if(lo >= 0) {
                    idx = LocalId2GlobalId(j, GB);
                    changed_pts_.erase(idx);
                    data_seg[k] = 1;
                }
                else if(lo > free_thr_) {
                    idx = LocalId2GlobalId(j, GB);
                    changed_pts_.erase(idx);
                    if(GB->vox_[j].PvFlags_ & PV_FRONTIER){
                        data_seg[k] = 3;
                    }
                    else {
                        data_seg[k] = 2;
                    }
                }
                else {
                    data_seg[k] = 0;
                }
            }
            j++;
        }
        data[i] = EncodeVoxel(data_seg);
    }
    outfile.write(reinterpret_cast<char*>(&data), sizeof(uint8_t[data_size]));
    outfile.close();
    return;
}

void RipNeon::ReadData(const uint32_t &bid, shared_ptr<PV_Block> &GB){
    GB = make_shared<PV_Block>();
    if(bid < 0){
        GB = NULL;
        return;
    }

    float thr_max = thr_max_ - 1e-3f;
    string p = map_path_ + "/" + to_string(bid);
    GB->origin_ = Id2BlockIdx3(bid).cwiseProduct(block_size_);

    int voxel_num = block_size_(0) * block_size_(1) * block_size_(2);
    float loggodds_init = thr_min_ - 100.0f;
    GB->Reset(voxel_num, loggodds_init);
    GB->id_ = bid;
    int data_size = ceil(voxel_num / 4);
    uint8_t data[data_size];
    std::ifstream infile(p.c_str(), ios::binary);
    if(!infile.is_open()){ // not exist, use unknown 
        GB->PvbFlags_ |= PVB_EMPTY;
        return;
    }
    infile.read(reinterpret_cast<char*>(&data), sizeof(uint8_t[data_size]));
    infile.close();

    uint64_t idx;
    uint8_t data_seg[4];
    for(int i = 0, j = 0; i < data_size; i++){
        DecodeVoxel(data[i], data_seg);
        for(int k = 0; k < 4; k++){
            if(j >= voxel_num){
                // do nothing
            }
            else{
                if(data_seg[k] == 0) {
                    GB->vox_[j].log_odds_ = loggodds_init;
                }
                else if(data_seg[k] == 1) {
                    idx = LocalId2GlobalId(j, GB);
                    changed_pts_.insert({idx, {1, 0}});
                    GB->vox_[j].log_odds_ = thr_max;
                    GB->occ_idx_.emplace_back(j);
                }
                else if(data_seg[k] == 3) {
                    idx = LocalId2GlobalId(j, GB);
                    changed_pts_.insert({idx, {0, 0}});
                    GB->vox_[j].PvFlags_ |= PV_FRONTIER;
                    GB->vox_[j].log_odds_ = thr_min_;
                    GB->fron_idx_.emplace_back(j);
                }
                else {
                    idx = LocalId2GlobalId(j, GB);
                    changed_pts_.insert({idx, {0, 0}});
                    GB->vox_[j].log_odds_ = thr_min_;
                }
            }
            j++;
        }
    }

    GB->PvbFlags_ |= PVB_SHOW;
    return;
}

void RipNeon::SaveDataThread(){
    shared_ptr<PV_Block> GB;
    string p;

    vox_io_mutex_.lock();
    int voxel_num = block_size_(0) * block_size_(1) * block_size_(2);
    vox_io_mutex_.unlock();

    int data_size = ceil(voxel_num / 4);
    uint8_t data[data_size];
    uint8_t data_seg[4]; //00 : unknown; 01: free; 10: occ; 11: frontier;
    int id, lid;
    float lo;

    while(1){
        vox_io_mutex_.lock();
        if(out_idxs_.empty()){
            vox_io_mutex_.unlock();
            return;
        }
        lid = GetLocalBlockId(out_idxs_.back());
        id = GetBlockId(out_idxs_.back());
        if(id == -1){
            cout<<"out_idxs_.back():"<<out_idxs_.back().transpose()<<endl;
            cout<<"lid:"<<lid<<endl;
            cout<<"block_num_:"<<block_num_.transpose()<<endl;
            cout<<"local_block_num_:"<<local_block_num_.transpose()<<endl;
            cout<<"local_origin_idx_:"<<local_origin_idx_.transpose()<<endl;
            StopageDebug("save id == -1");
        }

        out_idxs_.pop_back();
        if(id < -1) continue;
        vox_io_mutex_.unlock();
        GB = PVBs_[lid];
        if(GB->PvbFlags_ & PVB_EMPTY) continue;

        p = map_path_ + "/" + to_string(id);

        /* open file */
        std::ofstream outfile(p.c_str(), ios::binary);
        if(!outfile.is_open()){
            ROS_ERROR("open file failed");
            ros::shutdown();
            return;
        }

        for(int i = 0, j = 0; i < data_size; i++){
            for(int k = 0; k < 4; k++){
                if(j >= voxel_num){
                    data_seg[k] = 0;
                }
                else{
                    lo = GB->vox_[j].log_odds_;

                    if(lo >= 0) {
                        data_seg[k] = 1;
                    }
                    else if(lo > free_thr_) {

                        if(GB->vox_[j].PvFlags_ & PV_FRONTIER){
                            data_seg[k] = 3;
                            // to add to changed_pts_
                        }
                        else {
                            data_seg[k] = 2;
                            // to add to changed_pts_
                        }
                    }
                    else {
                        data_seg[k] = 0;
                            // to add to changed_pts_
                    }
                }
                j++;
            }
            data[i] = EncodeVoxel(data_seg);
        }
        outfile.write(reinterpret_cast<char*>(&data), sizeof(uint8_t[data_size]));
        outfile.close();
    }
}

void RipNeon::ReadDataThread(){
    int bid, lid;
    string p;
    string mp = map_path_ + "/";

    vox_io_mutex_.lock();
    int voxel_num = block_size_(0) * block_size_(1) * block_size_(2);
    float loggodds_init = thr_min_ - 100.0f;
    float thr_max = thr_max_ - 1e-3f;
    float thr_min = thr_min_;
    Eigen::Vector3i bs = block_size_;
    Eigen::Vector3i bn = block_num_;
    Eigen::Vector3i vn = voxel_num_;

    int bnx1 = bn_x_(1);

    vox_io_mutex_.unlock();
    Eigen::Vector3i bid3;

    uint64_t idx_key;
    vector<pair<uint64_t, pair<uint8_t, uint16_t>>> idx_keys;
    int data_size = ceil(voxel_num / 4);
    uint8_t data[data_size];
    uint8_t data_seg[4];
    while(1){
        vox_io_mutex_.lock();
        for(auto &k : idx_keys){
            changed_pts_.insert(k);
        }
        idx_keys.clear();
        if(new_idxs_.empty()){
            vox_io_mutex_.unlock();
            return;
        }

        lid = GetLocalBlockId(new_idxs_.back().second);
        bid = new_idxs_.back().first;
        new_idxs_.pop_back();

        vox_io_mutex_.unlock();
        if(bid < 0) continue;
        auto &GB = PVBs_[lid];

        p = mp + to_string(bid);
        GB = make_shared<PV_Block>();

        bid3(0) = bid % bn(0);
        bid3(1) = ((bid - bid3(0))/bn(0)) % bn(1);
        bid3(2) = ((bid - bid3(0)) - bid3(1)*bn(0))/bnx1;
        GB->origin_ = bid3.cwiseProduct(bs);
        // cout<<"or init:"<<GB->origin_.transpose()<<endl;

        GB->Reset(voxel_num, loggodds_init);
        GB->id_ = bid;

        std::ifstream infile(p.c_str(), ios::binary);
        if(!infile.is_open()){ // not exist, use unknown 
            GB->PvbFlags_ |= PVB_EMPTY;
            continue;
        }



        infile.read(reinterpret_cast<char*>(&data), sizeof(uint8_t[data_size]));
        infile.close();

        for(int i = 0, j = 0; i < data_size; i++){
            DecodeVoxel(data[i], data_seg);
            for(int k = 0; k < 4; k++){
                if(j >= voxel_num){
                    // do nothing
                }
                else{
                    if(data_seg[k] == 0) {
                        GB->vox_[j].log_odds_ = loggodds_init;
                    }
                    else if(data_seg[k] == 1) {
                        GB->vox_[j].log_odds_ = thr_max;
                        GB->occ_idx_.emplace_back(j);

                        idx_key = LocalId2GlobalId(j, GB, bs, vn);
                        idx_keys.push_back({idx_key, {1, 0}});
                    }
                    else if(data_seg[k] == 3) {
                        GB->vox_[j].PvFlags_ |= PV_FRONTIER;
                        GB->vox_[j].log_odds_ = thr_min;
                        GB->fron_idx_.emplace_back(j);
                        idx_key = LocalId2GlobalId(j, GB, bs, vn);
                        idx_keys.push_back({idx_key, {0, 0}});
                    }
                    else {
                        GB->vox_[j].log_odds_ = thr_min;
                        idx_key = LocalId2GlobalId(j, GB, bs, vn);
                        idx_keys.push_back({idx_key, {0, 0}});
                    }
                }
                j++;
            }
        }
        GB->PvbFlags_ |= PVB_SHOW;
        // vox_io_mutex_.lock();
        // read_blk_num_++;
        // cout<<"read v num:"<<idx_keys.size()<<endl;
        // cout<<"data_size:"<<data_size<<endl;
        // cout<<"p:"<<p<<endl;
        // vox_io_mutex_.unlock();
    }
    // std::ifstream infile(p.c_str(), ios::binary);
}